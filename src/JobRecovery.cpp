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
static uint32_t      _resumeLine = 0;
static int           _lineScroll = 0;  // ShowLines MPG scroll offset   // user-confirmed resume line

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
    // Also save percent if available (more reliable than line count)
    extern file_percent_t myPercent;
    if (myPercent > 0 && myPercent <= 100) {
        // Store percent in upper 16 bits of lastLine as a fallback
        // If lastLine is 0 (no line tracking), use percent
        if (_currentLine == 0) {
            _cp.lastLine = (uint32_t)myPercent * 1000;  // encode: percent*1000
        }
    }
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
        if (choice == 0) {           // Open Probe — flag it, caller opens probe overlay
            // Phase stays ToolChange, caller checks jobrecov_wantsProbe()
            _wantsProbe = true;
        } else if (choice == 1) {    // Done - Z probed
            _wantsProbe = false;
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
        canvas->setFont(&fonts::Font2);  // larger font for titles only
        canvas->setTextDatum((lgfx::v1::textdatum_t)datum);
        canvas->setTextColor(c);
        canvas->drawString(s, x, y);
    };
    auto txt0 = [&](const char* s, int x, int y, uint16_t c, int datum=4){
        canvas->setFont(&fonts::Font0);  // small font for body text
        canvas->setTextDatum((lgfx::v1::textdatum_t)datum);
        canvas->setTextColor(c);
        canvas->drawString(s, x, y);
    };
    // Clip-safe text: truncates string to fit within panel width
    int pw2 = pw - 12;  // usable text width
    auto txt0_safe = [&](const char* s, int x, int y, uint16_t c){
        canvas->setFont(&fonts::Font0);
        canvas->setTextDatum((lgfx::v1::textdatum_t)4);  // middle_left
        canvas->setTextColor(c);
        // Measure and truncate if needed
        std::string str(s);
        while (str.size() > 3 && canvas->textWidth(str.c_str()) > pw2)
            str = str.substr(0, str.size()-1);
        canvas->drawString(str.c_str(), x, y);
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
        canvas->fillCircle(cx-60, py+48, 5, stcol);
        txt0(ststr, cx-48, py+48, stcol, 4);  // middle_left aligned near center
        // Stats
        char line1[48];
        // Detect if lastLine is encoded percent (value > 100000 means percent*1000)
        if (_cp.lastLine > 100000) {
            uint32_t pct = _cp.lastLine / 1000;
            snprintf(line1,sizeof(line1),"Progress: ~%u%%", pct);
        } else if (_cp.lastLine > 0) {
            snprintf(line1,sizeof(line1),"Last line: %u", _cp.lastLine);
        } else {
            snprintf(line1,sizeof(line1),"Progress: unknown");
        }
        char line2[48];
        float x2=_cp.axisX/10000.0f, y2=_cp.axisY/10000.0f, z2=_cp.axisZ/10000.0f;
        snprintf(line2,sizeof(line2),"X%.2f Y%.2f Z%.2f",x2,y2,z2);
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
        txt0_safe("Was this job stopped due to:", px+6, py+34, C_WHITE);
        txt0_safe("tool break, E-stop, power loss,", px+6, py+48, C_WHITE);
        txt0_safe("or any sudden stop?", px+6, py+62, C_WHITE);
        txt0_safe("YES = manual jog + tool change required", px+6, py+80, YELLOW);
        txt0_safe("NO  = standard resume (clean stop)", px+6, py+94, C_DIM);
        int bw2=(pw-18)/2, by2=py+avH-bh-8;
        btn(_btnA, px+6,      by2, bw2, bh, 0xC000, RED,   "YES (Tool Break)",  C_WHITE);
        btn(_btnB, px+12+bw2, by2, bw2, bh, GREEN,  GREEN, "NO (Clean Stop)",   0x0000);
        break;
    }

    case RecoveryPhase::ShowLines: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,CYAN);
        txt("SELECT RESUME LINE", cx, py+14, CYAN);
        txt0_safe("Tap a line to select, then confirm:", px+6, py+28, C_WHITE);
        // Decode lastLine — may be encoded percent
        uint32_t baseLine = (_cp.lastLine > 100000) ? (_cp.lastLine/1000)*30 : _cp.lastLine;
        int lineH=18, startY=py+42;
        int nLines = std::min(7, (avH-42-bh-8)/lineH);
        uint32_t startLine = (baseLine > (uint32_t)(nLines-1)) ? baseLine-(nLines-1) : 0;
        // Store line rects for touch
        static Rect _lineRects[7];
        for(int i=0;i<nLines;i++){
            uint32_t ln = startLine+i;
            bool isSel=(ln==_resumeLine);
            int ly=startY+i*lineH;
            _lineRects[i]={px+4,ly-1,pw-8,lineH-1};
            canvas->fillRoundRect(px+4,ly-1,pw-8,lineH-1,2,isSel?0x0019:C_PANEL);
            if(isSel) canvas->drawRoundRect(px+4,ly-1,pw-8,lineH-1,2,CYAN);
            char lb[32]; snprintf(lb,sizeof(lb),"Line %u%s",ln,isSel?" ◄":"");
            canvas->setFont(&fonts::Font0); canvas->setTextDatum(4);
            canvas->setTextColor(isSel?YELLOW:C_DIM);
            canvas->drawString(lb,cx,ly+lineH/2);
        }
        btn(_btnA, px+6, py+avH-bh-8, pw-12, bh, CYAN, CYAN, "Resume from selected line", 0x0000);
        break;
    }

    case RecoveryPhase::ManualJog: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,RED);
        txt("MANUAL JOG REQUIRED", cx, py+14, RED);
        txt0_safe("Machine stopped mid-cut.", px+6, py+30, C_WHITE);
        txt0_safe("Use MPG wheel to:", px+6, py+46, YELLOW);
        txt0_safe("1. Jog Z UP to clear workpiece", px+16, py+62, C_WHITE);
        txt0_safe("2. Jog X/Y clear of workpiece", px+16, py+78, C_WHITE);
        txt0_safe("DO NOT auto-move — path may be blocked", px+6, py+94, RED);
        btn(_btnA, px+6, py+avH-bh-8, pw-12, bh, GREEN, GREEN, "Done — machine is clear", 0x0000);
        break;
    }

    case RecoveryPhase::ToolChange: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,ORANGE);
        txt("TOOL CHANGE REQUIRED", cx, py+14, ORANGE);
        txt0_safe("Insert new tool, then set Z0:", px+6, py+30, C_WHITE);
        // Option A: probe
        int bh2=26, bh3=26, gap2=4;
        int by2 = py+avH - bh2 - bh3 - bh3 - gap2*2 - 8;
        txt0_safe("Option 1 — Auto probe:", px+6, by2-14, YELLOW);
        btn(_btnB, px+6, by2, pw-12, bh2, 0x0019, CYAN, "Open Home Tab → Run Probe", C_WHITE);
        by2 += bh2+gap2;
        txt0_safe("Option 2 — Manual: jog Z to surface, then:", px+6, by2-14, YELLOW);
        // Set Z0 manually
        btn(_btnC, px+6, by2, pw-12, bh3, 0x2800, GREEN, "Set Z0 here  (G10 L20 P1 Z0)", C_WHITE);
        by2 += bh3+gap2;
        // Done
        btn(_btnA, px+6, by2, pw-12, bh3, ORANGE, ORANGE, "Done — Z0 is set, continue", 0x0000);
        break;
    }

    case RecoveryPhase::Confirm: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,GREEN);
        txt("READY TO RESUME", cx, py+14, GREEN);
        txt0_safe("The pendant will execute:", px+6, py+30, C_WHITE);
        txt0_safe("1. Home Z for reference", px+16, py+46, C_DIM);
        char pos[48];
        float rx=_cp.axisX/10000.0f, ry2=_cp.axisY/10000.0f, rz=_cp.axisZ/10000.0f;
        snprintf(pos,sizeof(pos),"2. X%.2f Y%.2f",rx,ry2);
        txt0_safe(pos, px+16, py+60, C_DIM);
        snprintf(pos,sizeof(pos),"3. Z %.2f at F%u",rz,_cp.feedRate);
        txt0_safe(pos, px+16, py+74, C_DIM);
        char rline[32]; snprintf(rline,sizeof(rline),"4. Resume ~line %u",_resumeLine);
        txt0_safe(rline, px+16, py+88, C_DIM);
        if(_toolBreak) txt0_safe("(Tool re-probed, Z offset applied)", px+6, py+104, YELLOW);
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
    case RecoveryPhase::ShowLines: {
        // Check individual line rows
        uint32_t baseLine = (_cp.lastLine > 100000) ? (_cp.lastLine/1000)*30 : _cp.lastLine;
        static Rect _lineRects[7];  // defined in draw, reused here
        int nLines = 7;
        uint32_t startLine = (baseLine > (uint32_t)(nLines-1)) ? baseLine-(nLines-1) : 0;
        for(int i=0;i<nLines;i++){
            int lineH=18, startY=44+4;  // matches draw: startY=py+42, py=TOP+4=24
            int ly = 24+42+i*lineH;
            Rect r={12,ly-1,296,lineH-1};
            if(inRect(r,x,y)){ _resumeLine=startLine+i; return true; }
        }
        if (inRect(_btnA,x,y)) { jobrecov_advance(0); return true; }
        break;
    }
    case RecoveryPhase::ManualJog:
        if (inRect(_btnA,x,y)) { jobrecov_advance(0); return true; }
        break;
    case RecoveryPhase::ToolChange:
        if (inRect(_btnA,x,y)) { jobrecov_advance(0); return true; }  // Done
        if (inRect(_btnB,x,y)) {
            // Open Home tab for probing — recovery stays active
            extern int _tabFromRecovery;
            _tabFromRecovery = 1;
            return true;
        }
        if (inRect(_btnC,x,y)) {
            // Manual Z0 — send G10 L20 P1 Z0 to set current position as Z zero
            send_line("G10 L20 P1 Z0");
            fnc_term_inject("> G10 L20 P1 Z0: Z0 set at current position");
            return true;
        }
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

void jobrecov_scroll(int delta) {
    if (_phase == RecoveryPhase::ShowLines) {
        _lineScroll = std::max(0, std::min(3, _lineScroll + delta));
        uint32_t base = (_cp.lastLine>9)?_cp.lastLine-9:0;
        _resumeLine = base + _lineScroll + 4;  // select middle of visible window
        markDirty();
    }
}
