/* The speed run (MODE:4): follow a stored route instead of the right-hand
 * rule. The dangerous failure here is not a wrong turn, it is losing count --
 * one miscounted junction and every later move lands somewhere else entirely.
 * So what is checked is the counting, and that a route which no longer fits
 * the maze STOPS rather than turning into a wall.
 */
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

int fails=0;
static void check(const char*w,bool ok){ printf("  %-56s %s\n",w,ok?"ok":"FAIL"); if(!ok)fails++; }

/* Feed one tick per sensor, as tofTask() expects. */
static void hold(int f,int l,int r,int ticks){
  for(int t=0;t<ticks;t++){
    g_script[S_FRONT].push_back(f);
    g_script[S_LEFT ].push_back(l);
    g_script[S_RIGHT].push_back(r);
    g_fake_millis += CONTROL_TICK_MS;
    tofTask(); updateDebounce();
  }
}
/* Mirror what ST_DRIVING does every tick: classify, then hand the result to
 * the replay decision -- including on plain corridor, which is what releases
 * the one-action-per-junction latch. */
static void drive(int f,int l,int r,int ticks){
  for(int t=0;t<ticks;t++){
    hold(f,l,r,1);
    /* Only ST_DRIVING consults the route. Once a turn is committed the robot
     * is in APPROACHING and the junction handler is not reached again, so a
     * harness that kept calling it would spend symbols the robot never
     * would. */
    if (g_running && g_state==ST_DRIVING) replayAtJunction(classify());
  }
}
static void setRoute(const char*r){
  strcpy(g_short,r); g_short_len=strlen(r); g_replay_idx=0;
  s_junction_acted=false; g_replay=true; g_running=true;
  enterState(ST_DRIVING);
}

int main(){
  for(int i=0;i<S_COUNT;i++){ tof[i].slot=i; st[i].present=true; }
  const int WALL=180, OPEN=8190, NEAR=250;   /* NEAR < FB, so front blocked */

  /* ---- 1. one symbol per junction, however long it stays in view ------- */
  printf("=== a junction passed straight consumes exactly one symbol ===\n");
  tofFlush(); s_blocked_side=-1; setRoute("SRL");
  hold(OPEN,WALL,WALL,6);                       /* plain corridor */
  check("corridor consumes nothing", g_replay_idx==0);

  /* A 400 mm opening at 250 mm/s sits in view for ~1.6 s == 80 ticks. The
   * old code cleared the counters here, which made classify() rediscover the
   * same opening every few ticks. */
  drive(OPEN,OPEN,WALL,80);
  printf("  80 ticks with the opening in view -> idx=%d\n", g_replay_idx);
  check("one S consumed, not one per tick", g_replay_idx==1);

  drive(OPEN,WALL,WALL,8);                      /* opening drifts past */
  check("the corridor after it consumes nothing", g_replay_idx==1);

  /* ---- 2. a turn is committed, not consumed twice --------------------- */
  printf("\n=== a turning consumes one symbol and goes to APPROACHING ===\n");
  drive(OPEN,WALL,OPEN,9);
  check("route said R -> pend_right", pend_right==true);
  check("consumed exactly one more", g_replay_idx==2);
  check("moved to APPROACHING", g_state==ST_APPROACHING);

  /* ---- 3. desync must stop, not guess --------------------------------- */
  printf("\n=== a route that no longer fits the maze stops the run ===\n");
  tofFlush(); s_blocked_side=-1; setRoute("S");
  drive(NEAR,WALL,OPEN,9);           /* front blocked: straight is a wall */
  check("'straight' into a wall stops the run", g_running==false);
  check("and does not consume the symbol", g_replay_idx==0);

  tofFlush(); s_blocked_side=-1; setRoute("L");
  drive(NEAR,WALL,OPEN,9);           /* route says left; left is a wall */
  check("a left that is not there still enters APPROACHING", g_state==ST_APPROACHING);
  /* ST_LOOKING is where it is verified, with the robot stopped and looking
   * straight down the opening -- that is the whole point of stopping. */
  enterState(ST_LOOKING);
  g_fake_millis += LOOK_TIMEOUT_MS + 1;
  mazeTick();
  check("...and ST_LOOKING refuses to turn into the wall", g_running==false);

  /* ---- 3b. a junction that changes character while still in view ------
   * The side sensors see an opening long before the front sensor believes in
   * a wall past it, so "straight or left" can become "forced left" without
   * the robot having moved on. Exploration records S then L for that; the
   * replay must be able to spend both. */
  printf("\n=== a junction that becomes blocked after being passed ===\n");
  tofFlush(); s_blocked_side=-1; setRoute("SL");
  drive(OPEN,OPEN,WALL,9);          /* straight-or-left: consume the S */
  check("consumed the S", g_replay_idx==1 && g_running);
  drive(NEAR,OPEN,WALL,9);          /* same opening, wall now ahead */
  check("consumed the L too, rather than latching", g_replay_idx==2);
  check("and committed to the turn", g_state==ST_APPROACHING && !pend_right);
  check("run still alive", g_running==true);

  /* ---- 4. running out of moves ---------------------------------------- */
  printf("\n=== running out of moves ===\n");
  tofFlush(); s_blocked_side=-1; setRoute("S");
  drive(OPEN,OPEN,WALL,9);          /* consumes the only symbol */
  check("route exhausted", g_replay_idx==g_short_len);
  drive(OPEN,WALL,WALL,8);          /* corridor: releases the latch */
  drive(OPEN,OPEN,WALL,9);          /* another junction, nothing left */
  check("an extra junction with no moves left stops the run", g_running==false);

  /* ---- 5. the explored path is not corrupted by a replay -------------- */
  printf("\n=== a speed run does not rewrite the explored path ===\n");
  g_replay=false; g_path_len=0; g_path[0]=0;
  recordTurn('R'); recordTurn('S');
  g_replay=true;
  recordTurn('L'); recordTurn('L');
  check("recordTurn is inert during replay", strcmp(g_path,"RS")==0);

  printf("\n%s\n", fails?"FAIL":"PASS");
  return fails?1:0;
}
