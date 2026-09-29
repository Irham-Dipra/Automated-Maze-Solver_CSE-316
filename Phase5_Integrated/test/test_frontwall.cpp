/* Replays 10:35:16 from the log: after a right turn the came-from block is
 * armed on the right, the front wall closes in, the left is a wall. The robot
 * drove 320 -> contact with fv:5 the whole way, never leaving DRIVING,
 * because classify() had no case for "wall ahead, nothing open" and fell
 * through to J_CORRIDOR.
 *
 * Also checks the heading target cannot be whipped by a flickering side. */
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

static void hold(int f,int l,int r,int ticks){
  for(int t=0;t<ticks;t++){
    g_script[S_FRONT].push_back(f);
    g_script[S_LEFT ].push_back(l);
    g_script[S_RIGHT].push_back(r);
    g_fake_millis += CONTROL_TICK_MS;
    tofTask(); updateDebounce();
  }
}
int fails=0;
static void check(const char*w,bool ok){ printf("  %-52s %s\n",w,ok?"ok":"FAIL"); if(!ok)fails++; }

int main(){
  for(int i=0;i<S_COUNT;i++){ tof[i].slot=i; st[i].present=true; }

  printf("=== wall ahead, left walled, right = the corridor we came from ===\n");
  tofFlush();
  blockCameFrom(true);              /* just turned right */
  hold(8190,180,8190,4);            /* clear ahead first */
  check("no junction while the front is clear", classify()==J_CORRIDOR);

  hold(250,180,8190,5);             /* front wall closes in, fv fills */
  printf("  front 250 (FB %d), left 180 wall, right open-but-came-from\n", cfg_front_blk);
  printf("  frontBlocked=%d  cnt_open_l=%d cnt_open_r=%d cnt_deadend=%d -> classify=%d\n",
         (int)frontBlocked(), cnt_open_l, cnt_open_r, cnt_deadend, (int)classify());
  check("wall ahead is NOT reported as clear corridor", classify()!=J_CORRIDOR);
  check("it is treated as a dead end", classify()==J_DEAD_END);

  printf("\n=== ordinary dead end still works ===\n");
  tofFlush(); s_blocked_side=-1;
  hold(250,180,180,6);
  check("front blocked, both sides walls -> dead end", classify()==J_DEAD_END);

  printf("\n=== a real right turning is still seen ===\n");
  tofFlush(); s_blocked_side=-1;
  hold(250,180,8190,9);
  printf("  cnt_open_r=%d (needs %d)  cnt_block_f=%d (needs %d)\n",
         cnt_open_r, OPENING_CONFIRM, cnt_block_f, FRONT_CONFIRM);
  check("front blocked + right open -> forced right", classify()==J_FORCED_RIGHT);

  /* The whole point of the longer side confirm: a couple of noisy pings off a
   * dark wall used to be a junction, which sent the robot through the entire
   * approach-park-look-give-up sequence in the middle of a clean corridor. */
  printf("\n=== a brief flicker is NOT a junction ===\n");
  tofFlush(); s_blocked_side=-1;
  hold(8190,180,180,6);                 /* settled corridor, both walls */
  check("clean corridor reads as corridor", classify()==J_CORRIDOR);
  hold(8190,180,8190,2);                /* right flickers out of range */
  printf("  after a 2-tick flicker: cnt_open_r=%d -> classify=%d\n",
         cnt_open_r, (int)classify());
  check("a 2-tick right flicker does not invent a turning",
        classify()==J_CORRIDOR);
  hold(8190,180,180,4);                 /* wall comes back */
  check("and the corridor is still a corridor after it", classify()==J_CORRIDOR);

  /* But a real opening, which persists, must still be seen. */
  printf("\n=== a real side opening is still seen ===\n");
  hold(8190,180,8190,9);
  check("a sustained right opening -> fwd-or-right", classify()==J_FWD_OR_RIGHT);

  /* U-turn side selection: pick whichever side has more room. */
  printf("\n=== U-turn swings toward the roomier side ===\n");
  tofFlush(); s_blocked_side=-1;
  hold(250,220,120,6);
  check("more room on the left -> U-turn goes left", uturnGoesRight()==false);
  tofFlush();
  hold(250,120,220,6);
  check("more room on the right -> U-turn goes right", uturnGoesRight()==true);

  printf("\n=== a flickering side cannot whip the heading target ===\n");
  tofFlush(); s_blocked_side=-1;
  cfg_kw = 0.02f; g_target_heading = 0;
  float worst = 0;
  for(int t=0;t<40;t++){
      /* right alternates between a plausible wall reading and out-of-range,
       * as in the log. 250 is inside maxWallMm so it is genuinely used --
       * otherwise the plausibility gate would catch it first and the slew
       * limiter would never be exercised. */
      hold(8190, 8190, (t%2)?250:8190, 1);
      float before = g_target_heading;
      driveTick();
      float jump = fabsf(g_target_heading - before);
      if (jump > worst) worst = jump;
  }
  {   /* what the unlimited target would have swung between */
      float lo = 0, hi = -cfg_kw * (float)(250 - wallRefMm()) * 2.0f;
      printf("  unlimited target would swing %.2f deg per tick\n", fabsf(hi-lo));
  }
  printf("  largest single-tick target move: %.2f deg (limit %.1f)\n", worst, WALL_TILT_SLEW_DEG);
  check("the target does move (limiter actually exercised)", worst > 0.5f);
  check("but never more than the slew limit per tick",
        worst <= WALL_TILT_SLEW_DEG + 0.01f);

  printf("\n=== a reading too far to be a corridor wall is ignored ===\n");
  tofFlush(); s_blocked_side=-1; g_target_heading = 0;
  printf("  maxWallMm = %d (CW %d, RW %d, SS %d)\n",
         maxWallMm(), cfg_corridor_w, cfg_robot_w, cfg_span);
  hold(8190, 900, 8190, 6);          /* left far away: an opening, not a wall */
  for(int t=0;t<20;t++) driveTick();
  printf("  left=900 -> target %.2f\n", g_target_heading);
  check("no target tilt from an opening mouth", fabsf(g_target_heading) < 0.5f);

  printf("\n%s\n", fails?"FAIL":"PASS");
  return fails?1:0;
}
