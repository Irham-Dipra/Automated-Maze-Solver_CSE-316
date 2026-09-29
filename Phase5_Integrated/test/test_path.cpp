/* Dead-end elimination on the turn string. The rule under test is
 *      a U b  ->  (a + 2 + b) mod 4     with S=0 L=1 U=2 R=3
 * which is supposed to generate the whole nine-case LSRB table, so the first
 * thing checked is that it really does -- against the table written out by
 * hand, independently of the arithmetic.
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

static const char *red(const char *in, int *rc_out=0){
  static char out[96]; int n=0;
  int rc = reducePath(in, (int)strlen(in), out, (int)sizeof(out), &n);
  if (rc_out) *rc_out = rc;
  out[n] = 0;
  return out;
}

int main(){
  /* ---- 1. the nine collapses, against a hand-written table -------------
   * Each case is "a U b" with a trailing wall so the sentinel cannot also
   * fold in and confuse what is being measured. */
  printf("=== the nine LSRB collapses ===\n");
  struct { const char *in; char want; } T[] = {
    {"LUL",'S'}, {"LUS",'R'}, {"LUR",'U'},
    {"SUL",'R'}, {"SUS",'U'}, {"SUR",'L'},
    {"RUL",'U'}, {"RUS",'L'}, {"RUR",'S'},
  };
  for (unsigned i=0;i<sizeof(T)/sizeof(T[0]);i++){
    char out[96]; int n=0;
    reducePath(T[i].in, 3, out, sizeof(out), &n);
    /* one collapse leaves exactly one move (or none, if it was S) */
    char got = (n == 1) ? out[0] : (n == 0 ? 'S' : '?');
    char msg[64]; snprintf(msg,sizeof(msg),"%s -> %c", T[i].in, T[i].want);
    check(msg, got == T[i].want);
  }

  /* ---- 2. the real maze ------------------------------------------------ */
  printf("\n=== the recorded run ===\n");
  int rc;
  /* What mode 3 records TODAY: turns only, no straights. */
  printf("  without straights: RURURRRRU -> %s  (cannot be trusted)\n",
         red("RURURRRRU"));
  /* The user's stated expectation for the tail: the final R U, followed by
   * driving out of the exit, is a single LEFT. */
  check("tail 'RU' + exit becomes a single left", strcmp(red("RU"), "L") == 0);

  /* A plausible full string for the same run, with the straights present. */
  const char *full = "RSURURRRRU";
  printf("  with straights:    %s -> %s\n", full, red(full));
  check("reduces to a U-free route", red(full, &rc) && rc == 0);

  /* ---- 3. properties --------------------------------------------------- */
  printf("\n=== properties ===\n");
  check("no dead ends -> unchanged except the sentinel",
        strcmp(red("RLSRL"), "RLSRL") == 0);
  check("a leading S is kept (it is the first junction)",
        strcmp(red("SRL"), "SRL") == 0);
  check("reduction is idempotent", strcmp(red(red("RSURURRRRU")), red("RSURURRRRU")) == 0);

  /* Cascading: collapsing one pair must expose the next. R U R -> S, then
   * S U R -> L, then that L with the next U, and so on. */
  check("collapses cascade (RURUR -> L)", strcmp(red("RURUR"), "L") == 0);

  /* Nothing to fold a leading U into: that is a dead end at the start line,
   * which means the robot was placed wrong. It must be reported, not driven. */
  red("URL", &rc);
  check("a U with nothing before it is reported, not silently kept", rc == 1);

  red("", &rc);
  check("an empty path is rejected", rc == 2);

  { char tiny[4]; int n; check("an undersized buffer is refused",
        reducePath("RURUR", 5, tiny, (int)sizeof(tiny), &n) == 2); }

  /* Straight in, dead end, straight back out: the net move is a turn around,
   * which is a U the reduction cannot remove -- and must not hide. */
  red("SUS", &rc);
  check("S U S leaves a U, and says so", rc == 1);

  printf("\n%s\n", fails ? "FAIL" : "PASS");
  return fails ? 1 : 0;
}
