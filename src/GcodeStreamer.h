#pragma once
// ── Pendant-side G-code streamer ──────────────────────────────────────────────
// Streams allFileLines[] to FluidNC one line at a time, gating each send on
// the previous "ok" response via the show_ok() callback hook.
// This gives the pendant full control over position in the file — enabling
// true pause, rewind, and mid-file resume without aborting/restarting.
//
// States:
//   IDLE       — not streaming
//   STREAMING  — sending lines, waiting for ok between each
//   PAUSED     — hold requested; stop sending after current ok drains
//                machine finishes current motion → enters Idle → joggable
//   DONE       — last line sent and acked

#include <string>
#include <vector>
#include <functional>

enum class StreamState { Idle, Streaming, Paused, Done };

// Call from TabScene to start streaming allFileLines from a given line index
void streamer_start(const std::vector<std::string>& lines,
                    const std::string& jobName, int startLine = 0);

// Pause: stop sending new lines after current motion drains
void streamer_pause();

// Resume from current position (after a pause)
void streamer_resume();

// Resume from a specific line index (after rewind)
void streamer_resume_from(int lineIdx);

// Abort: stop streaming, reset state
void streamer_abort();

// Called from show_ok() in FluidNCModel.cpp
void streamer_on_ok();

// Query
StreamState streamer_state();
int streamer_current_line();   // index into the lines vector
int streamer_total_lines();
bool streamer_active();        // true if Streaming or Paused
