#pragma once
#include <Arduino.h>
// Scripted fake sensor: feed it the exact reading sequence from the log.
extern std::deque<uint16_t> g_script[3];
extern int g_script_idx;
class Adafruit_VL53L0X { public:
  int slot = 0;
  uint16_t last = 100;
  boolean begin(uint8_t=0x29, boolean=false, TwoWire* =&Wire){return true;}
  boolean setMeasurementTimingBudgetMicroSeconds(uint32_t){return true;}
  void startRangeContinuous(uint16_t=50){}
  boolean isRangeComplete(){return true;}
  uint16_t readRange(){
    if(!g_script[slot].empty()){ last=g_script[slot].front(); g_script[slot].pop_front(); }
    return last;
  }
  uint8_t readRangeStatus(){ return (last>=8000)?4:0; }
};
