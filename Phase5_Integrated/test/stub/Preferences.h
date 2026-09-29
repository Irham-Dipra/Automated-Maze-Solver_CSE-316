/* Host stub: the tests never exercise flash, they only have to link. */
#pragma once
#include <Arduino.h>
class Preferences {
public:
    bool begin(const char *, bool = false) { return false; }
    void end() {}
    size_t putString(const char *, const char *) { return 0; }
    String getString(const char *, const char *d = "") { return String(d); }
};
