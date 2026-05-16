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
static int           _lineScroll = 0;  // ShowLines MPG scroll offset
static bool          _wantsProbe = false;
static Rect          _lineRects[7] = {};  // set in ShowLines draw

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
        // _resumeLine already set by scroll/tap — just advance
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
        // Title
        canvas->setFont(&fonts::Font2); canvas->setTextDatum(middle_center);
        canvas->setTextColor(YELLOW);
        canvas->drawString("INTERRUPTED JOB FOUND", cx, py+12);
        // File path (truncated)
        std::string jp=_cp.jobPath; if(jp.size()>36) jp=jp.substr(jp.size()-36);
        canvas->setFont(&fonts::Font0); canvas->setTextColor(C_DIM);
        canvas->drawString(jp.c_str(), cx, py+26);
        // Connection status
        bool conn=(state!=Disconnected);
        canvas->fillCircle(px+14, py+40, 5, conn?GREEN:RED);
        canvas->setTextDatum(middle_left);
        canvas->setTextColor(conn?GREEN:RED);
        canvas->drawString(conn?"FluidNC: Connected":"FluidNC: NOT Connected", px+24, py+40);
        // Progress and position
        char prog[32];
        if(_cp.lastLine>100000) snprintf(prog,32,"Progress: ~%u%%",_cp.lastLine/1000);
        else if(_cp.lastLine>0) snprintf(prog,32,"Line: %u",_cp.lastLine);
        else snprintf(prog,32,"Progress: unknown");
        canvas->setTextDatum(middle_center);
        canvas->setTextColor(C_WHITE);
        canvas->drawString(prog, cx, py+56);
        float x2=_cp.axisX/10000.0f,y2=_cp.axisY/10000.0f,z2=_cp.axisZ/10000.0f;
        char pos[40]; snprintf(pos,40,"X%.2f  Y%.2f  Z%.2f",x2,y2,z2);
        canvas->setTextColor(C_DIM);
        canvas->drawString(pos, cx, py+70);
        canvas->setTextColor(CYAN);
        canvas->drawString("Resume this job?", cx, py+86);
        // Buttons
        int bw2=(pw-18)/2, by2=py+avH-bh-6;
        btn(_btnA, px+6,      by2, bw2, bh, GREEN, GREEN,  "YES — Resume", 0x0000);
        btn(_btnB, px+12+bw2, by2, bw2, bh, C_PANEL, RED,  "NO — Discard", RED);
        break;
    }

    case RecoveryPhase::AskToolBreak: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,ORANGE);
        canvas->setFont(&fonts::Font2); canvas->setTextDatum(middle_center);
        canvas->setTextColor(ORANGE);
        canvas->drawString("TOOL BREAK / E-STOP?", cx, py+12);
        canvas->setFont(&fonts::Font0); canvas->setTextColor(C_WHITE);
        canvas->drawString("Was this job stopped by:", cx, py+28);
        canvas->drawString("tool break, E-stop, power loss, crash?", cx, py+42);
        canvas->setTextColor(YELLOW);
        canvas->drawString("YES: manual jog + tool change required", cx, py+58);
        canvas->setTextColor(C_DIM);
        canvas->drawString("NO:  safe auto-resume", cx, py+72);
        int bw2=(pw-18)/2, by2=py+avH-bh-6;
        btn(_btnA, px+6,      by2, bw2, bh, 0xC000, RED,   "YES — Unsafe stop", C_WHITE);
        btn(_btnB, px+12+bw2, by2, bw2, bh, GREEN,  GREEN, "NO — Clean stop",   0x0000);
        break;
    }

    case RecoveryPhase::ShowLines: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,CYAN);
        // Title
        canvas->setFont(&fonts::Font2); canvas->setTextDatum(middle_center);
        canvas->setTextColor(CYAN);
        canvas->drawString("SELECT RESUME LINE", cx, py+12);
        canvas->setFont(&fonts::Font0); canvas->setTextColor(C_DIM);
        canvas->drawString("Turn MPG encoder to scroll, tap line to select", cx, py+24);
        // Work out actual last line (may be encoded percent)
        uint32_t lastLine = (_cp.lastLine > 100000) ? (_cp.lastLine/1000)*100 : _cp.lastLine;
        uint32_t base = (lastLine > 9) ? lastLine - 9 : 0;
        // 7 selectable line rows
        int lineH=19, listY=py+34, numShow=7;
        for(int i=0;i<numShow;i++){
            uint32_t ln = base + (uint32_t)_lineScroll + i;
            bool sel = (ln == _resumeLine);
            int lx=px+4, ly=listY+i*lineH, lw=pw-8;
            _lineRects[i] = {lx, ly, lw, lineH-1};
            if(sel){
                canvas->fillRoundRect(lx,ly,lw,lineH-1,2,0x2106);
                canvas->drawRoundRect(lx,ly,lw,lineH-1,2,CYAN);
            } else {
                canvas->drawRoundRect(lx,ly,lw,lineH-1,2,0x1082);
            }
            canvas->setFont(&fonts::Font2); canvas->setTextDatum(middle_left);
            canvas->setTextColor(sel?YELLOW:C_DIM);
            char lb[32]; snprintf(lb,32,"  Line %u",ln);
            canvas->drawString(lb, lx+4, ly+lineH/2-1);
            if(sel){
                canvas->setTextDatum(middle_right);
                canvas->setTextColor(CYAN);
                canvas->drawString("◄ selected  ", lx+lw-4, ly+lineH/2-1);
            }
            // Default marker
            if(ln == lastLine){
                canvas->setTextDatum(middle_right);
                canvas->setTextColor(0x7BEF);
                canvas->drawString("last saved", lx+lw-4, ly+lineH/2-1);
            }
        }
        // Confirm button
        btn(_btnA, px+4, py+avH-bh-4, pw-8, bh, CYAN, CYAN, "Confirm selected line", 0x0000);
        break;
    }
    case RecoveryPhase::ManualJog: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,RED);
        canvas->setFont(&fonts::Font2); canvas->setTextDatum(middle_center);
        canvas->setTextColor(RED);
        canvas->drawString("MANUAL JOG REQUIRED", cx, py+12);
        // Two step boxes side by side
        int hw=(pw-20)/2, by0=py+26;
        canvas->fillRoundRect(px+4,    by0,hw,36,4,0x2800); canvas->drawRoundRect(px+4,    by0,hw,36,4,GREEN);
        canvas->fillRoundRect(px+12+hw,by0,hw,36,4,0x001A); canvas->drawRoundRect(px+12+hw,by0,hw,36,4,CYAN);
        canvas->setFont(&fonts::Font0); canvas->setTextColor(GREEN);
        canvas->drawString("1. Select Z",px+4+hw/2,by0+12);
        canvas->drawString("Jog UP to clear",px+4+hw/2,by0+26);
        canvas->setTextColor(CYAN);
        canvas->drawString("2. Select X/Y",px+12+hw+hw/2,by0+12);
        canvas->drawString("Jog clear of part",px+12+hw+hw/2,by0+26);
        // Arrow between boxes
        canvas->drawLine(px+4+hw+2,by0+18,px+10+hw,by0+18,C_DIM);
        canvas->fillTriangle(px+12+hw,by0+18,px+9+hw,by0+14,px+9+hw,by0+22,C_DIM);
        // Warning
        canvas->fillRoundRect(px+4,by0+42,pw-8,26,3,0x4000);
        canvas->drawRoundRect(px+4,by0+42,pw-8,26,3,RED);
        canvas->setTextColor(RED);
        canvas->drawString("! Never auto-move after crash",cx,by0+54);
        canvas->setTextColor(C_DIM);
        canvas->drawString("Path may be blocked by debris",cx,by0+66);
        btn(_btnA, px+4, py+avH-bh-4, pw-8, bh, GREEN, GREEN, "Done — machine is clear", 0x0000);
        break;
    }

    case RecoveryPhase::ToolChange: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,ORANGE);
        canvas->setFont(&fonts::Font2); canvas->setTextDatum(middle_center);
        canvas->setTextColor(ORANGE);
        canvas->drawString("TOOL CHANGE", cx, py+12);
        // Steps (left side)
        canvas->setFont(&fonts::Font0); canvas->setTextDatum(middle_left);
        canvas->setTextColor(C_WHITE);
        canvas->drawString("1. Insert new tool",   px+8, py+30);
        canvas->drawString("2. Tap PROBE → set Z", px+8, py+44);
        canvas->drawString("3. Verify Z=0 in DRO", px+8, py+58);
        char tlast[24]; snprintf(tlast,24,"Last: T%u",_cp.tool);
        canvas->setTextColor(C_DIM);
        canvas->drawString(tlast, px+8, py+72);
        // Probe diagram (right side)
        int dx=cx+50, dy=py+24;
        canvas->fillRoundRect(dx+4,dy,    16,28,3,0xFEE8);  // tool body
        canvas->fillRoundRect(dx+7,dy+28, 10,18,2,0xFEE8);  // tip
        canvas->fillRoundRect(dx,dy+48,   24, 8,2,0x2965);  // probe block
        canvas->drawRoundRect(dx,dy+48,   24, 8,2,0xFEE8);
        canvas->drawLine(dx+12,dy+35,dx+12,dy+46,YELLOW);
        canvas->fillTriangle(dx+12,dy+48,dx+9,dy+43,dx+15,dy+43,YELLOW);
        canvas->setTextDatum(middle_left);
        canvas->setTextColor(0xFEE8);
        canvas->drawString("Z0",dx+26,dy+52);
        // Two buttons
        int bw2=(pw-18)/2, by2=py+avH-bh-4;
        btn(_btnA, px+4,      by2, bw2, bh, ORANGE, ORANGE, "Open Probe",    0x0000);
        btn(_btnB, px+10+bw2, by2, bw2, bh, GREEN,  GREEN,  "Done — probed", 0x0000);
        break;
    }

    case RecoveryPhase::Confirm: {
        fillR(px,py,pw,avH,6,C_PANEL); strokeR(px,py,pw,avH,6,GREEN);
        canvas->setFont(&fonts::Font2); canvas->setTextDatum(middle_center);
        canvas->setTextColor(GREEN);
        canvas->drawString("READY TO RESUME", cx, py+12);
        // 4 sequence boxes
        int sbw=(pw-20)/4, sby=py+26, sbh=38;
        const uint16_t sbg[4]={0x0440,0x001A,0x4220,0x0440};
        const uint16_t sbc[4]={GREEN,CYAN,YELLOW,GREEN};
        const char*sl1[4]={"Home Z","Go XY","Lower Z","Run"};
        const char*sl2[4]={"home","rapid","plunge","file"};
        for(int i=0;i<4;i++){
            int sx=px+4+i*(sbw+3);
            canvas->fillRoundRect(sx,sby,sbw,sbh,3,sbg[i]);
            canvas->drawRoundRect(sx,sby,sbw,sbh,3,sbc[i]);
            canvas->setFont(&fonts::Font0); canvas->setTextDatum(middle_center);
            canvas->setTextColor(sbc[i]); canvas->drawString(sl1[i],sx+sbw/2,sby+12);
            canvas->setTextColor(C_DIM);  canvas->drawString(sl2[i],sx+sbw/2,sby+26);
            if(i<3){canvas->drawLine(sx+sbw,sby+sbh/2,sx+sbw+3,sby+sbh/2,C_DIM);}
        }
        // Data
        float rx2=_cp.axisX/10000.0f,ry2=_cp.axisY/10000.0f,rz=_cp.axisZ/10000.0f;
        char p1[40]; snprintf(p1,40,"X%.1f Y%.1f Z%.2f",rx2,ry2,rz);
        char p2[40]; snprintf(p2,40,"F%u  line ~%u",_cp.feedRate,_resumeLine);
        canvas->setFont(&fonts::Font0); canvas->setTextDatum(middle_center);
        canvas->setTextColor(C_DIM);
        canvas->drawString(p1,cx,sby+sbh+12);
        canvas->drawString(p2,cx,sby+sbh+24);
        if(_toolBreak){
            canvas->fillRoundRect(px+4,sby+sbh+32,pw-8,16,3,0x4220);
            canvas->setTextColor(YELLOW);
            canvas->drawString("Tool re-probed — Z offset applied",cx,sby+sbh+40);
        }
        int bw2=(pw-18)/2, by2=py+avH-bh-4;
        btn(_btnA, px+4,      by2, bw2, bh, GREEN,  GREEN, "CONFIRM RESUME", 0x0000);
        btn(_btnB, px+10+bw2, by2, bw2, bh, C_PANEL, RED,  "CANCEL",         RED);
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
        // Tap on a line row to select it, tap Confirm to advance
        uint32_t lastLine = (_cp.lastLine > 100000) ? (_cp.lastLine/1000)*100 : _cp.lastLine;
        uint32_t base = (lastLine > 9) ? lastLine - 9 : 0;
        for(int i=0;i<7;i++){
            if(inRect(_lineRects[i],x,y)){
                _resumeLine = base + (uint32_t)_lineScroll + i;
                markDirty(); return true;
            }
        }
        if(inRect(_btnA,x,y)){ jobrecov_advance(0); return true; }
        break;
    }
