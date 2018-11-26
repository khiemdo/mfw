// This version file is overwritten on first build, and successive changes are ignored locally by setup script.
#ifndef VERSION_H_
#define VERSION_H_

#ifndef ACTIVE_CONFIG
#warning "Active config is unknown. Please set it with --define ACTIVE_CONFIG=XYZ in compile options. Meanwhile, ACTIVE_CONFIG is set to UNKNOWN"
#define ACTIVE_CONFIG UNKNOWN
#endif

#define XSTR(s) STR(s)
#define STR(s) #s

#define BUILD_DATE Version: n/a - Commmit Hash: n/a - Build Date: n/a - Active Config:
#define BUILD_STRING XSTR(BUILD_DATE ACTIVE_CONFIG)

#endif //VERSION_H_
