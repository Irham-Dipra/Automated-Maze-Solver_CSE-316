#pragma once
#include <Arduino.h>
class BluetoothSerial { public:
  bool begin(const char*){return true;} void println(const String&){}
  bool hasClient(){return true;} int available(){return 0;} int read(){return -1;}
};
