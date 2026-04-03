// Copyright (c) 2023 - Mitch Bradley
// Tab UI build: macro menu code removed. GPLv3 licence.

#include "FileParser.h"
#include "Scene.h"
#include "GrblParserC.h"
#include <JsonStreamingParser.h>
#include <JsonListener.h>

fileinfo              fileInfo;
std::vector<fileinfo> fileVector;

JsonStreamingParser parser;
bool parser_needs_reset = true;

static bool fileinfoCompare(const fileinfo& f1, const fileinfo& f2) {
    if (!f1.isDir() && f2.isDir()) return true;
    if (f1.isDir() && !f2.isDir()) return false;
    return f1.fileName.compare(f2.fileName) < 0;
}

int fileFirstLine = 0;
std::vector<std::string> fileLines;
extern JsonListener* pInitialListener;

// ── File list parser ──────────────────────────────────────────────────────────
class FilesListListener : public JsonListener {
private:
    bool        haveNewFile;
    std::string current_key;
public:
    void whitespace(char c) override {}
    void startDocument() override {}
    void startArray() override { fileVector.clear(); haveNewFile = false; }
    void startObject() override {}
    void key(const char* key) override {
        current_key = key;
        if (strcmp(key, "name") == 0) haveNewFile = true;
    }
    void value(const char* value) override {
        if (current_key == "name")      fileInfo.fileName = value;
        else if (current_key == "size") fileInfo.fileSize = atoi(value);
    }
    void endArray() override {
        std::sort(fileVector.begin(), fileVector.end(), fileinfoCompare);
        current_scene->onFilesList();
        parser.setListener(pInitialListener);
    }
    void endObject() override {
        if (haveNewFile) { fileVector.push_back(fileInfo); haveNewFile = false; }
    }
    void endDocument() override { init_listener(); }
} filesListListener;

// ── File lines parser ─────────────────────────────────────────────────────────
class FileLinesListener : public JsonListener {
private:
    bool _in_array = false;
    bool _key_is_firstline = false;
public:
    void whitespace(char c) override {}
    void startDocument() override {}
    void startArray() override { fileLines.clear(); _in_array = true; }
    void endArray() override   { _in_array = false; }
    void startObject() override {}
    void key(const char* key) override { _key_is_firstline = (strcmp(key, "firstline") == 0); }
    void value(const char* value) override {
        if (_in_array) fileLines.push_back(value);
        if (_key_is_firstline) fileFirstLine = atoi(value);
    }
    void endObject() override {
        parser.setListener(pInitialListener);
        current_scene->onFileLines(fileFirstLine, fileLines);
    }
    void endDocument() override {}
} fileLinesListener;

// ── Initial listener ──────────────────────────────────────────────────────────
class InitialListener : public JsonListener {
private:
    typedef enum { NONE, PATH, CMD, ARGUMENT, STATUS, ERROR } key_t;
    key_t       _key = NONE;
    std::string _status;
public:
    void whitespace(char c) override {}
    void startDocument() override { _key = NONE; _status = "ok"; }
    void startArray() override {}
    void startObject() override {}
    void endArray() override {}
    void endObject() override { parser_needs_reset = true; }
    void endDocument() override { parser_needs_reset = true; }
    void key(const char* key) override {
        if      (strcmp(key, "files")      == 0) { parser.setListener(&filesListListener); return; }
        else if (strcmp(key, "file_lines") == 0) { parser.setListener(&fileLinesListener); return; }
        else if (strcmp(key, "path")       == 0) _key = PATH;
        else if (strcmp(key, "cmd")        == 0) _key = CMD;
        else if (strcmp(key, "argument")   == 0) _key = ARGUMENT;
        else if (strcmp(key, "status")     == 0) _key = STATUS;
        else if (strcmp(key, "error")      == 0) _key = ERROR;
        else                                     _key = NONE;
    }
    void value(const char* value) override {
        if (_key == ERROR) current_scene->onError(value);
        if (_key == STATUS) _status = value;
        _key = NONE;
    }
} initialListener;

JsonListener* pInitialListener = &initialListener;

void init_listener() {
    parser.setListener(pInitialListener);
    parser_needs_reset = true;
}

void request_file_list(const char* dirname) {
    send_linef("$Files/ListGCode=%s", dirname);
    parser_needs_reset = true;
}

void init_file_list() {
    init_listener();
    request_file_list("/sd");
    parser.reset();
}

// Stub — not used by TabUI but kept in header for link compatibility
void request_macros() {}

void parser_parse_line(const char* line) {
    char c;
    while ((c = *line++) != '\0') parser.parse(c);
}

extern "C" void handle_json(const char* line) {
    if (parser_needs_reset) {
        parser_needs_reset = false;
        parser.setListener(pInitialListener);
        parser.reset();
    }
    parser_parse_line(line);
#define Ack 0xB2
    fnc_realtime((realtime_cmd_t)Ack);
}

std::string wifi_mode, wifi_ssid, wifi_connected, wifi_ip;

void parse_wifi(char* arguments) {
    char* key = arguments;
    char* value;
    while (*key) {
        char* next;
        split(key, &next, ':');
        split(key, &value, '=');
        if      (strcmp(key, "SSID")   == 0) wifi_ssid       = value;
        else if (strcmp(key, "Status") == 0) wifi_connected  = value;
        else if (strcmp(key, "IP")     == 0) wifi_ip         = value;
        key = next;
    }
}

void handle_radio_mode(char* command, char* arguments) {
    char* value;
    split(command, &value, '=');
    wifi_mode = value;
    if (strcmp(value, "No Wifi") != 0) {
        parse_wifi(arguments);
        current_scene->reDisplay();
    }
}

extern "C" void handle_msg(char* command, char* arguments) {
    if (strcmp(command, "RST") == 0) {
        dbg_println("FluidNC Reset");
        state = Disconnected;
        act_on_state_change();
    }
    if (strcmp(command, "Files changed") == 0) init_file_list();
    if (strcmp(command, "JSON") == 0)          handle_json(arguments);
    if (strncmp(command, "Mode=", 5) == 0)     handle_radio_mode(command, arguments);
}
