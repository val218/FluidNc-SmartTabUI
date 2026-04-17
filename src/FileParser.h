// Copyright (c) 2023 - Mitch Bradley
// Tab UI build: stripped. GPLv3 licence.
#pragma once
#include <string>
#include <vector>

typedef void (*callback_t)(void*);

struct fileinfo {
    std::string fileName;
    int         fileSize;
    bool        isDir() const { return fileSize < 0; }
};

extern fileinfo              fileInfo;
extern std::vector<fileinfo> fileVector;

extern void request_file_list(const char* dirname);
extern void request_macros();
extern void init_listener();
extern void init_file_list();
void request_file_preview(const char* path, int firstLine, int nLines);

extern std::string wifi_mode, wifi_ip, wifi_connected, wifi_ssid;
