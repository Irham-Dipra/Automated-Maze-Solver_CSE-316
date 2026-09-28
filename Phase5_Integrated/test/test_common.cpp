#include <Arduino.h>
#include <esp_system.h>
#include <deque>
SerialC Serial, Serial2; TwoWire Wire;
unsigned long g_fake_millis = 0;
unsigned long millis(){ return g_fake_millis; }
void delay(unsigned long ms){ g_fake_millis += ms; }
void delayMicroseconds(unsigned){}
void pinMode(uint8_t,uint8_t){} void digitalWrite(uint8_t,uint8_t){}
int digitalRead(uint8_t){return 1;}
esp_reset_reason_t esp_reset_reason(){return ESP_RST_POWERON;}
std::deque<uint16_t> g_script[3]; int g_script_idx=0;
#define setup sketch_setup
#define loop  sketch_loop
#include SKETCH_INC
#undef setup
#undef loop
static void tick(){ g_fake_millis += 20; tofPoll(S_FRONT); }
int main(){
    for(int i=0;i<S_COUNT;i++){ tof[i].slot=i; tofClear(i); st[i].present=true; }
    for(int i=0;i<4;i++)  g_script[S_FRONT].push_back(400);
    for(int i=0;i<5;i++)  g_script[S_FRONT].push_back(30);
    for(int i=0;i<600;i++) g_script[S_FRONT].push_back(8190);
    for(int i=0;i<9;i++) tick();
    printf("  latch armed: tooClose=%d\n",(int)tofTooClose(S_FRONT));
    int release=-1;
    for(int i=0;i<500;i++){ tick(); if(release<0 && !tofTooClose(S_FRONT)) release=i; }
    printf("  after 10 s of wide-open corridor: F=%s tooClose=%d frontBlocked=%d\n",
           rangeStr(S_FRONT).c_str(),(int)tofTooClose(S_FRONT),(int)frontBlocked());
    if(release<0){ printf("  => JAMMED FOREVER\n"); return 1; }
    printf("  => released after %d ms\n", release*20); return 0;
}
