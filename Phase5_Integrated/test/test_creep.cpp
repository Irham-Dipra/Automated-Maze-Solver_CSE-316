/* Creep-to-park: the robot must finish at frontStopMm() regardless of where
 * braking left it. Reproduces both observed failures -- arriving far short
 * after a slow post-pivot approach, and arriving touching after a fast one. */
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

/* Simulated chassis: a forward/back pulse moves it at MM_PER_MS. */
static double sim_gap;                 /* real front gap, mm */
static const double MM_PER_MS = 0.30;  /* creep speed */

static void simTick(void){
  /* move if a creep pulse is driving */
  if (s_creep_driving && g_state == ST_CREEPING) {
      if (g_last_cmd=='F') sim_gap -= MM_PER_MS * CONTROL_TICK_MS;
      if (g_last_cmd=='B') sim_gap += MM_PER_MS * CONTROL_TICK_MS;
  }
  if (sim_gap < 5) sim_gap = 5;
  uint16_t r = (uint16_t)(sim_gap + 0.5);
  g_script[S_FRONT].push_back(r);
  g_script[S_LEFT].push_back(8190);
  g_script[S_RIGHT].push_back(8190);
  g_fake_millis += CONTROL_TICK_MS;
  tofTask();
}

static int runCreep(double start_gap){
  sim_gap = start_gap;
  tofFlush();
  for (int i=0;i<6;i++) simTick();          /* fill the filters */
  enterState(ST_CREEPING);
  for (int t=0; t<600 && g_state==ST_CREEPING; t++){ simTick(); mazeTick(); }
  return (int)(sim_gap+0.5);
}

int fails=0;
static void check(const char*what,bool ok){ printf("  %-56s %s\n",what,ok?"ok":"FAIL"); if(!ok)fails++; }

int main(){
  for(int i=0;i<S_COUNT;i++){ tof[i].slot=i; st[i].present=true; }
  int target = frontStopMm();
  printf("park target = %d mm (CW/2 - AX = 200 - 130)\n\n", target);

  printf("=== slow arrival after a pivot: braking stopped it far short ===\n");
  int a = runCreep(300);
  printf("  braked at 300 mm -> finished at %d mm\n", a);
  check("crept forward to the park point", abs(a-target) <= CREEP_TOL_MM+10);

  printf("\n=== fast arrival down a long straight: it overran onto the wall ===\n");
  int b = runCreep(25);
  printf("  braked at  25 mm -> finished at %d mm\n", b);
  check("reversed back off the wall to the park point", abs(b-target) <= CREEP_TOL_MM+10);

  printf("\n=== already correct: must not fidget ===\n");
  int before = s_creep_pulses;
  int c = runCreep(target);
  printf("  braked at %3d mm -> finished at %d mm, %d pulses\n", target, c, s_creep_pulses);
  check("no pulses spent when already on target", s_creep_pulses==0);
  (void)before;

  printf("\n=== front sensor dead: accept the brake point, do not nudge blind ===\n");
  sim_gap = 300; tofFlush();
  for(int i=0;i<6;i++){ g_script[S_FRONT].push_back(8190);
                        g_script[S_LEFT].push_back(8190);
                        g_script[S_RIGHT].push_back(8190);
                        g_fake_millis+=CONTROL_TICK_MS; tofTask(); }
  enterState(ST_CREEPING);
  for(int t=0;t<50 && g_state==ST_CREEPING;t++){
      g_script[S_FRONT].push_back(8190);
      g_fake_millis+=CONTROL_TICK_MS; tofTask(); mazeTick(); }
  check("left CREEPING without nudging", g_state!=ST_CREEPING && s_creep_pulses==0);

  printf("\n%s\n", fails?"FAIL":"PASS");
  return fails?1:0;
}
