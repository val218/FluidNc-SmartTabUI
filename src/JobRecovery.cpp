#include "JobRecovery.h"
#include "FluidNCModel.h"
#include "System.h"
#include "Scene.h"
#include <LittleFS.h>
#include <string>
#include <vector>
#include <algorithm>

// LovyanGFX sprite forward — we cast in draw
#include <LovyanGFX.hpp>

// Declarations for functions defined in other translation units
extern "C" void fnc_term_inject(const char* line);
extern std::string myModes;  // defined in FluidNCModel.cpp

// ── Constants ────────────────────────────────────────────────────────────────
#define CHKPT_PATH  "/job_checkpoint.bin"
#define SAVE_INTERVAL_LINES  20   // save every 20 lines

// ── State ─────────────────────────────────────────────────────────────────────
static JobCheckpoint _cp;
static bool          _hasCheckpoint = false;
static RecoveryPhase _phase = RecoveryPhase::None;
static bool          _toolBreak = false;
static uint32_t      _currentLine = 0;
static uint32_t      _linesSinceLastSave = 0;
static std::string   _jobPath;
// Last 10 lines of G-code for display
static std::vector<std::string> _lastLines;
static uint32_t      _resumeLine = 0;   // user-confirmed resume line

// Rect struct for touch zones
struct Rect { int x, y, w, h; };
static Rect _btnA = {0,0,0,0}, _btnB = {0,0,0,0}, _btnC = {0,0,0,0};

static bool inRect(const Rect& r, int x, int y) {
    return x>=r.x && x<r.x+r.w && y>=r.y && y<r.y+r.h;
}

// ── LittleFS helpers ──────────────────────────────────────────────────────────
static void saveCheckpoint() {
    memcpy(_cp.magic, "JCP", 4);
    _cp.savedAt = millis();
    File f = LittleFS.open(CHKPT_PATH, FILE_WRITE);
    if (f) {
        f.write((const uint8_t*)&_cp, sizeof(_cp));
        f.close();
        dbg_printf("JobRecovery: saved line=%u\n", _cp.lastLine);
    } else {
        dbg_println("JobRecovery: SAVE FAILED");
    }
}

static bool loadCheckpoint() {
    File f = LittleFS.open(CHKPT_PATH, "r");
    if (!f) return false;
    bool ok = (f.read((uint8_t*)&_cp, sizeof(_cp)) == sizeof(_cp));
    f.close();
    if (!ok || memcmp(_cp.magic, "JCP", 3) != 0) return false;
    return true;
}

static void clearCheckpoint() {
    _cp.dirty = false;
    saveCheckpoint();
    LittleFS.remove(CHKPT_PATH);
    _hasCheckpoint = false;
}

// ── Snapshot current machine state into _cp ────────────────────────────────
static void snapshotState() {
    _cp.axisX = (int32_t)myAxes[0];
    _cp.axisY = (int32_t)myAxes[1];
    _cp.axisZ = (int32_t)myAxes[2];
    _cp.feedRate = myFeed;
    _cp.spindleSpeed = mySpeed;
    _cp.tool = mySelectedTool;
    _cp.lastLine = _currentLine;
    // Parse WCS from myModes string
    const char* m = myModes.c_str();
    if      (strstr(m,"G54")) strncpy(_cp.wcs,"G54",8);
    else if (strstr(m,"G55")) strncpy(_cp.wcs,"G55",8);
    else if (strstr(m,"G56")) strncpy(_cp.wcs,"G56",8);
    else                      strncpy(_cp.wcs,"G54",8);
    if (strstr(m,"G20")) strncpy(_cp.units,"G20",4);
    else                 strncpy(_cp.units,"G21",4);
    if (strstr(m,"G91")) strncpy(_cp.distMode,"G91",4);
    else                 strncpy(_cp.distMode,"G90",4);
    if      (strstr(m,"Mist"))  strncpy(_cp.coolant,"M7",8);
    else if (strstr(m,"Flood")) strncpy(_cp.coolant,"M8",8);
    else                        strncpy(_cp.coolant,"M9",8);
    strncpy(_cp.jobPath, _jobPath.c_str(), 127);
    _cp.dirty = true;
}

// ── Public API ────────────────────────────────────────────────────────────────
void jobrecov_init() {
    // LittleFS already mounted by init_system() — just load checkpoint
    bool loaded = loadCheckpoint();
    _hasCheckpoint = loaded && _cp.dirty;
    if (loaded) {
        char msg[80];
        snprintf(msg, sizeof(msg), "[Recovery] dirty=%d line=%u path=%s",
                 _cp.dirty, _cp.lastLine, _cp.jobPath);
        fnc_term_inject(msg);
    } else {
        fnc_term_inject("[Recovery] No checkpoint found");
    }
}

bool jobrecov_hasDirty() { return _hasCheckpoint; }
const JobCheckpoint& jobrecov_getCheckpoint() { return _cp; }
RecoveryPhase jobrecov_getPhase() { return _phase; }

void jobrecov_jobStarted(const char* path) {
    _jobPath = path;
    _currentLine = 0;
    _linesSinceLastSave = 0;
    _lastLines.clear();
    snapshotState();
    _cp.dirty = true;
    saveCheckpoint();
}

void jobrecov_lineExecuted(uint32_t line) {
    _currentLine = line;
    _linesSinceLastSave++;
    if (_linesSinceLastSave >= SAVE_INTERVAL_LINES) {
        _linesSinceLastSave = 0;
        snapshotState();
        saveCheckpoint();
    }
}

void jobrecov_saveNow() {
    snapshotState();
    saveCheckpoint();
}

void jobrecov_jobComplete() {
    clearCheckpoint();
    _phase = RecoveryPhase::None;
}

void jobrecov_cancelResume() {
    _phase = RecoveryPhase::None;
    clearCheckpoint();
}

void jobrecov_startResume(bool toolBreak) {
    _toolBreak = toolBreak;
    // Calculate resume line — back up 5 lines from checkpoint
    uint32_t backoff = 5;
    _resumeLine = (_cp.lastLine > backoff) ? (_cp.lastLine - backoff) : 0;
    if (_toolBreak) {
        _phase = RecoveryPhase::ManualJog;
    } else {
        _phase = RecoveryPhase::Confirm;
    }
}

// Build and send the G-code preamble that restores machine state
static void sendResumePreamble() {
    float ax = _cp.axisX / 10000.0f;
    float ay = _cp.axisY / 10000.0f;
    float az = _cp.axisZ / 10000.0f;

    // 1. Restore modal state
    char preamble[512];
    snprintf(preamble, sizeof(preamble),
        "%s\n"          // WCS (G54-G59)
        "%s\n"          // Units (G20/G21)
        "G90\n"         // Always use absolute for the approach
        "T%u M6\n"      // Tool number
        "M3 S%u\n"      // Spindle on at last known speed (M3 = CW)
        "%s\n"          // Coolant
        "G53 G0 Z0\n"   // Retract Z to machine home
        "G0 X%.4f Y%.4f\n"  // Move XY to resume position
        "G1 Z%.4f F%u\n"    // Lower Z to cutting depth at feed rate
        "%s\n",         // Restore distance mode (G90/G91)
        _cp.wcs,
        _cp.units,
        _cp.tool,
        _cp.spindleSpeed > 0 ? _cp.spindleSpeed : 1000u,
        _cp.coolant,
        ax, ay,
        az,
        _cp.feedRate > 0 ? _cp.feedRate : 500u,
        _cp.distMode
    );

    // Send line by line
    char* p = preamble;
    while (*p) {
        char* nl = strchr(p, '\n');
        if (!nl) break;
        *nl = '\0';
        if (*p) send_line(p);
        p = nl + 1;
    }
    // Then run the job from resume line
    char runcmd[160];
    snprintf(runcmd, sizeof(runcmd), "$Localfs/Run=%s", _cp.jobPath);
    // Note: actual line seeking not supported in FluidNC — operator must
    // manually confirm the resume line in their CAM or we restart from line 0
    // For now we send the file and note the resume line in terminal
    char linemsg[64];
    snprintf(linemsg, sizeof(linemsg), "> Resume from approx. line %u", _resumeLine);
    fnc_term_inject(linemsg);
    send_line(runcmd);
}

void jobrecov_advance(int choice) {
    switch (_phase) {
    case RecoveryPhase::Prompt:
        if (choice == 0) {           // YES — resume
            _phase = RecoveryPhase::AskToolBreak;
        } else {                     // NO — discard
            jobrecov_cancelResume();
        }
        break;

    case RecoveryPhase::AskToolBreak:
        if (choice == 1) {           // YES — tool break/estop involved
            _toolBreak = true;
            _phase = RecoveryPhase::ShowLines;
        } else {                     // NO — standard resume
            _toolBreak = false;
            _phase = RecoveryPhase::Confirm;
        }
        break;

    case RecoveryPhase::ShowLines:
        // choice = selected resume line offset (0-9 = last 10 lines)
        _resumeLine = (_cp.lastLine > (uint32_t)(9 - choice)) ?
                      (_cp.lastLine - (uint32_t)(9 - choice)) : 0;
        _phase = RecoveryPhase::ManualJog;
        break;

    case RecoveryPhase::ManualJog:
        if (choice == 0) {           // Done jogging
            _phase = _toolBreak ? RecoveryPhase::ToolChange : RecoveryPhase::Confirm;
        }
        break;

    case RecoveryPhase::ToolChange:
        if (choice == 0) {           // Tool changed + probed
            _phase = RecoveryPhase::Confirm;
        }
        break;

    case RecoveryPhase::Confirm:
        if (choice == 0) {           // CONFIRM — execute preamble
            _phase = RecoveryPhase::Executing;
            // First home to establish reference
            send_line("$HZ");
            // After homing: preamble will be sent via pending action
            // For now, send immediately (homing should have been done before resume)
            sendResumePreamble();
            _phase = RecoveryPhase::None;
            clearCheckpoint();
        } else {                     // CANCEL
            jobrecov_cancelResume();
        }
        break;

    default: break;
    }
}

// ── Draw ─────────────────────────────────────────────────────────────────────
void jobrecov_draw(void* canvasPtr, int W, int H, int TOP, int NAV_Y) {
    auto* canvas = (lgfx::LGFX_Sprite*)canvasPtr;
    if (_phase == RecoveryPhase::None) return;

    // Colours
    const uint16_t C_BG    = 0x0862;
    const uint16_t C_PANEL = 0x10A3;
    const uint16_t C_WHITE = 0xFFFF;
    const uint16_t C_DIM   = 0x4A4A;
    const uint16_t GREEN   = 0x07E0;
    const uint16_t RED     = 0xF800;
    const uint16_t YELLOW  = 0xFFE0;
    const uint16_t ORANGE  = 0xFD20;
    const uint16_t CYAN    = 0x07FF;

    // Full-screen dim
    canvas->fillRect(0, TOP, W, NAV_Y - TOP, 0x8000);

    int pw = W - 16, cx = W/2;
    int avH = NAV_Y - TOP - 8;
    int px = 8, py = TOP + 4;

    auto fillR = [&](int x,int y,int w,int h,int r,uint16_t c){
        canvas->fillRoundRect(x,y,w,h,r,c);
    };
    auto strokeR = [&](int x,int y,int w,int h,int r,uint16_t c){
        canvas->drawRoundRect(x,y,w,h,r,c);
    };
    auto txt = [&](const char* s, int x, int y, uint16_t c, int datum=4){
        canvas->setFont(&fonts::Font2);
        canvas->setTextDatum((lgfx::v1::textdatum_t)datum);
        canvas->setTextColor(c);
        canvas->drawString(s, x, y);
    };
    auto txt0 = [&](const char* s, int x, int y, uint16_t c, int datum=4){
        canvas->setFont(&fonts::Font0);
        canvas->setTextDatum((lgfx::v1::textdatum_t)datum);
        canvas->setTextColor(c);
        canvas->drawString(s, x, y);
    };
    auto btn = [&](Rect& r, int x, int y, int w, int h, uint16_t fill, uint16_t border, const char* label, uint16_t tc){
        r = {x,y,w,h};
        fillR(x,y,w,h,4,fill);
        strokeR(x,y,w,h,4,border);
        txt(label, x+w/2, y+h/2, tc);
    };

    int bh = 30, gap = 6;

    switch (_phase) {

    case RecoveryPhase::Prompt: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,YELLOW);
        txt("INTERRUPTED JOB FOUND", cx, py+16, YELLOW);
        txt0(_cp.jobPath, cx, py+32, C_DIM);
        // Connection status indicator
        bool connected = (state != Disconnected);
        uint16_t stcol = connected ? GREEN : RED;
        const char* ststr = connected ? "FluidNC Connected" : "FluidNC Not Connected";
        canvas->fillCircle(px+14, py+48, 5, stcol);
        txt0(ststr, px+24, py+48, stcol, 4);  // 4=middle_left
        // Stats
        char line1[48]; snprintf(line1,sizeof(line1),"Last line: %u", _cp.lastLine);
        char line2[48];
        float x2=_cp.axisX/10000.0f, y2=_cp.axisY/10000.0f, z2=_cp.axisZ/10000.0f;
        snprintf(line2,sizeof(line2),"Position: X%.2f Y%.2f Z%.2f",x2,y2,z2);
        txt0(line1, cx, py+64, C_WHITE);
        txt0(line2, cx, py+78, C_WHITE);
        txt0("Resume this job?", cx, py+96, CYAN);
        int bw2=(pw-18)/2, by2=py+avH-bh-8;
        btn(_btnA, px+6,        by2, bw2, bh, GREEN,  GREEN,  "YES - Resume",  0x0000);
        btn(_btnB, px+12+bw2,   by2, bw2, bh, C_PANEL, RED,  "NO - Discard",  RED);
        break;
    }

    case RecoveryPhase::AskToolBreak: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,ORANGE);
        txt("TOOL BREAK OR E-STOP?", cx, py+16, ORANGE);
        txt0("Was this job stopped due to:", cx, py+34, C_WHITE);
        txt0("tool break, E-stop, power loss,", cx, py+48, C_WHITE);
        txt0("or any sudden stop?", cx, py+62, C_WHITE);
        txt0("YES = manual jog clear + tool change required", cx, py+80, YELLOW);
        txt0("NO  = standard resume (machine stopped normally)", cx, py+94, C_DIM);
        int bw2=(pw-18)/2, by2=py+avH-bh-8;
        btn(_btnA, px+6,      by2, bw2, bh, 0xC000, RED,   "YES (Tool Break)",  C_WHITE);
        btn(_btnB, px+12+bw2, by2, bw2, bh, GREEN,  GREEN, "NO (Clean Stop)",   0x0000);
        break;
    }

    case RecoveryPhase::ShowLines: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,CYAN);
        txt("CONFIRM RESUME LINE", cx, py+14, CYAN);
        txt0("Select where to resume (back up 3-5 lines):", cx, py+28, C_WHITE);
        // Show last lines with selectable buttons
        int lineH=16, startY=py+40;
        uint32_t startLine = (_cp.lastLine > 9) ? _cp.lastLine-9 : 0;
        for(int i=0;i<10;i++){
            uint32_t ln = startLine+i;
            bool isDefault=(ln==_resumeLine);
            int ly=startY+i*lineH;
            if(isDefault) canvas->fillRect(px+4,ly-2,pw-8,lineH,0x2106);
            char lb[32]; snprintf(lb,sizeof(lb),"Line %u%s",ln,isDefault?" ◄ default":"");
            canvas->setFont(&fonts::Font0); canvas->setTextDatum(4);
            canvas->setTextColor(isDefault?YELLOW:C_DIM);
            canvas->drawString(lb,cx,ly+6);
        }
        btn(_btnA, px+6, py+avH-bh-8, pw-12, bh, CYAN, CYAN, "Use selected line", 0x0000);
        break;
    }

    case RecoveryPhase::ManualJog: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,RED);
        txt("MANUAL JOG REQUIRED", cx, py+14, RED);
        txt0("Machine stopped mid-cut.", cx, py+30, C_WHITE);
        txt0("Use MPG wheel to:", cx, py+46, YELLOW);
        txt0("1. Jog Z UP to clear workpiece", cx, py+62, C_WHITE);
        txt0("2. Jog X/Y clear of workpiece", cx, py+78, C_WHITE);
        txt0("DO NOT auto-move — path may be obstructed", cx, py+94, RED);
        btn(_btnA, px+6, py+avH-bh-8, pw-12, bh, GREEN, GREEN, "Done — machine is clear", 0x0000);
        break;
    }

    case RecoveryPhase::ToolChange: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,ORANGE);
        txt("TOOL CHANGE REQUIRED", cx, py+14, ORANGE);
        txt0("1. Insert new/checked tool", cx, py+32, C_WHITE);
        txt0("2. Run tool length probe (Home tab)", cx, py+48, C_WHITE);
        txt0("3. Verify Z0 is correct", cx, py+64, C_WHITE);
        char tlast[32]; snprintf(tlast,sizeof(tlast),"Last tool: T%u", _cp.tool);
        txt0(tlast, cx, py+80, C_DIM);
        btn(_btnA, px+6, py+avH-bh-8, pw-12, bh, ORANGE, ORANGE, "Tool changed + probed", 0x0000);
        break;
    }

    case RecoveryPhase::Confirm: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,GREEN);
        txt("READY TO RESUME", cx, py+14, GREEN);
        txt0("The pendant will execute this sequence:", cx, py+30, C_WHITE);
        txt0("1. Home Z for reference", cx, py+46, C_DIM);
        char pos[48];
        float rx=_cp.axisX/10000.0f, ry2=_cp.axisY/10000.0f, rz=_cp.axisZ/10000.0f;
        snprintf(pos,sizeof(pos),"2. Move to X%.2f Y%.2f",rx,ry2);
        txt0(pos, cx, py+60, C_DIM);
        snprintf(pos,sizeof(pos),"3. Lower Z to %.2f at F%u",rz,_cp.feedRate);
        txt0(pos, cx, py+74, C_DIM);
        char rline[32]; snprintf(rline,sizeof(rline),"4. Resume from line ~%u",_resumeLine);
        txt0(rline, cx, py+88, C_DIM);
        if(_toolBreak) txt0("(Tool re-probed — Z offset applied)", cx, py+104, YELLOW);
        int bw2=(pw-18)/2, by2=py+avH-bh-8;
        btn(_btnA, px+6,      by2, bw2, bh, GREEN,  GREEN, "CONFIRM RESUME", 0x0000);
        btn(_btnB, px+12+bw2, by2, bw2, bh, C_PANEL, RED,  "CANCEL",         RED);
        break;
    }

    default: break;
    }
}

bool jobrecov_onTouch(int x, int y) {
    if (_phase == RecoveryPhase::None) return false;

    switch (_phase) {
    case RecoveryPhase::Prompt:
        if (inRect(_btnA,x,y)) { jobrecov_advance(0); return true; }  // YES
        if (inRect(_btnB,x,y)) { jobrecov_advance(1); return true; }  // NO
        break;
    case RecoveryPhase::AskToolBreak:
        if (inRect(_btnA,x,y)) { jobrecov_advance(1); return true; }  // YES tool break
        if (inRect(_btnB,x,y)) { jobrecov_advance(0); return true; }  // NO clean stop
        break;
    case RecoveryPhase::ShowLines:
        if (inRect(_btnA,x,y)) { jobrecov_advance(0); return true; }
        break;
    case RecoveryPhase::ManualJog:
        if (inRect(_btnA,x,y)) { jobrecov_advance(0); return true; }
        break;
    case RecoveryPhase::ToolChange:
        if (inRect(_btnA,x,y)) { jobrecov_advance(0); return true; }
        break;
    case RecoveryPhase::Confirm:
        if (inRect(_btnA,x,y)) { jobrecov_advance(0); return true; }  // CONFIRM
        if (inRect(_btnB,x,y)) { jobrecov_advance(1); return true; }  // CANCEL
        break;
    default: break;
    }
    return true;  // consume all touches while overlay is active
}

void jobrecov_showPrompt() {
    if (_hasCheckpoint && _cp.dirty) {
        _phase = RecoveryPhase::Prompt;
        markDirty();
    }
}
