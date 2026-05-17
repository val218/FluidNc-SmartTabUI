// ── Pendant-side G-code streamer ──────────────────────────────────────────────
#include "GcodeStreamer.h"
#include "FluidNCModel.h"   // send_line, state
#include "Scene.h"          // markDirty
#include <cstring>

extern "C" void fnc_term_inject(const char* line);

// ── State ─────────────────────────────────────────────────────────────────────
static StreamState          _state      = StreamState::Idle;
static const std::vector<std::string>* _lines = nullptr;
static int                  _cur        = 0;   // next line to send
static int                  _total      = 0;
static bool                 _waitingOk  = false; // sent a line, awaiting ok
static std::string          _jobName;

// ── Helpers ───────────────────────────────────────────────────────────────────
static bool isSkippable(const std::string& ln) {
    // Skip blank lines, comments, % markers
    const char* p = ln.c_str();
    while (*p == ' ' || *p == '\t') p++;
    return !*p || *p == ';' || *p == '(' || *p == '%';
}

static void sendNextLine() {
    if (!_lines) return;
    // Skip blank/comment lines without waiting for ok
    while (_cur < _total && isSkippable((*_lines)[_cur]))
        _cur++;
    if (_cur >= _total) {
        _state = StreamState::Done;
        _waitingOk = false;
        fnc_term_inject("> Stream complete");
        markDirty();
        return;
    }
    send_line((*_lines)[_cur].c_str());
    _waitingOk = true;
    _cur++;
}

// ── Public API ────────────────────────────────────────────────────────────────
void streamer_start(const std::vector<std::string>& lines,
                    const std::string& jobName, int startLine) {
    _lines      = &lines;
    _total      = (int)lines.size();
    _cur        = std::max(0, std::min(startLine, _total));
    _jobName    = jobName;
    _waitingOk  = false;
    _state      = StreamState::Streaming;
    char msg[80];
    snprintf(msg, sizeof(msg), "> Streaming: %s from line %d", jobName.c_str(), _cur);
    fnc_term_inject(msg);
    sendNextLine();  // send first line immediately
}

void streamer_pause() {
    if (_state == StreamState::Streaming)
        _state = StreamState::Paused;
        // Don't send any more lines — machine drains its planner → Idle → joggable
}

void streamer_resume() {
    if (_state == StreamState::Paused) {
        _state = StreamState::Streaming;
        if (!_waitingOk) sendNextLine();  // if machine already acked, kick next line
    }
}

void streamer_resume_from(int lineIdx) {
    if (!_lines) return;
    _cur       = std::max(0, std::min(lineIdx, _total));
    _waitingOk = false;
    _state     = StreamState::Streaming;
    char msg[64];
    snprintf(msg, sizeof(msg), "> Rewind resume from line %d", _cur);
    fnc_term_inject(msg);
    sendNextLine();
}

void streamer_abort() {
    _state     = StreamState::Idle;
    _waitingOk = false;
    _lines     = nullptr;
    _cur       = 0;
}

// Called from show_ok() every time FluidNC sends "ok"
void streamer_on_ok() {
    if (_state != StreamState::Streaming) {
        _waitingOk = false;
        return;  // Paused or Idle — don't send next line
    }
    _waitingOk = false;
    sendNextLine();
}

StreamState streamer_state()        { return _state; }
int         streamer_current_line() { return _cur; }
int         streamer_total_lines()  { return _total; }
bool        streamer_active()       { return _state == StreamState::Streaming
                                          || _state == StreamState::Paused; }
