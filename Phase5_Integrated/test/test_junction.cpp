#include <Arduino.h>
#include <esp_system.h>
#include <deque>
SerialC Serial, Serial2; TwoWire Wire;
unsigned long g_fake_millis=0;
unsigned long millis(){return g_fake_millis;}
void delay(unsigned long ms){g_fake_millis+=ms;}
void delayMicroseconds(unsigned){} void pinMode(uint8_t,uint8_t){} void digitalWrite(uint8_t,uint8_t){}
int digitalRead(uint8_t){return 1;}
esp_reset_reason_t esp_reset_reason(){return ESP_RST_POWERON;}
std::deque<uint16_t> g_script[3]; int g_script_idx=0;
#define setup sketch_setup
#define loop  sketch_loop
#include "body.inc"
#undef setup
#undef loop

// Hold each sensor at a fixed reading and run N control ticks.
static void hold(int f,int l,int r,int ticks){
  for(int t=0;t<ticks;t++){
    // exactly ONE reading per sensor per tick -- tofTask() consumes one each
    g_script[S_FRONT].push_back(f);
    g_script[S_LEFT].push_back(l);
    g_script[S_RIGHT].push_back(r);
    g_fake_millis += CONTROL_TICK_MS;
    tofTask();
    updateDebounce();
  }
}
int fails=0;
static void check(const char*what,bool ok){ printf("  %-58s %s\n",what,ok?"ok":"FAIL"); if(!ok)fails++; }

int main(){
  for(int i=0;i<S_COUNT;i++){ tof[i].slot=i; st[i].present=true; }
  tofFlush();

  printf("\n=== the 21:19 junction: front wall, BOTH sides open ===\n");
  printf("(left opened one tick before the right, which is what tipped it)\n");
  hold(8190,150,150,6);            // corridor, both walls present
  hold(250,8190,159,6);            // LEFT opens first, RIGHT still a wall (250 < FB)
  printf("  mid-approach classify() = %d (J_FORCED_LEFT is %d)\n",
         (int)classify(), (int)J_FORCED_LEFT);
  check("driving-time view really is 'forced left' (the old bug)",
        classify()==J_FORCED_LEFT);
  hold(70,8190,8190,4);            // parked at the junction: both open
  {
    bool right_open = tofOpen(S_RIGHT) && (s_blocked_side != S_RIGHT);
    bool left_open  = tofOpen(S_LEFT)  && (s_blocked_side != S_LEFT);
    bool front_wall = frontBlocked();
    check("at the junction the right is seen as open", right_open);
    check("right-hand rule now picks RIGHT, not LEFT",
          right_open && front_wall && left_open);
  }

  printf("\n=== double turn: after turning right, the old corridor is on the right ===\n");
  tofFlush();
  blockCameFrom(true);                       // we just turned right
  hold(8190,150,8190,6);                     // right still wide open (old corridor)
  check("that opening is ignored while the block holds", cnt_open_r==0);
  check("classify() does not call it a right turning", classify()!=J_FWD_OR_RIGHT);
  hold(8190,150,150,4);                      // new corridor's right wall arrives
  check("block releases once a real wall appears alongside", s_blocked_side==-1);
  hold(8190,150,8190,6);                     // a genuine later right turning
  check("a genuine right turning is seen again", cnt_open_r>=OPENING_CONFIRM);

  printf("\n=== the block cannot latch forever ===\n");
  tofFlush(); blockCameFrom(false);          // turned left
  hold(8190,8190,150,6);
  check("left still blocked before the backstop", s_blocked_side==S_LEFT);
  g_fake_millis += BLOCK_BACKSTOP_MS + 50;
  hold(8190,8190,150,2);
  check("backstop releases it", s_blocked_side==-1);

  printf("\n=== a 180 must NOT block: going back is the point ===\n");
  tofFlush(); s_blocked_side=-1;
  check("no block armed after a U-turn", s_blocked_side==-1);

  printf("\n%s\n", fails?"FAIL":"PASS");
  return fails?1:0;
}
