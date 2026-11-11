/* PC test: S3 line calibration learned while driving (line.c).
 *   gcc -std=gnu99 -w -Ihost_stubs -I../../app_program/subsystem3 \
 *       -I../../app_program/common -I../../app_program -DS3_LINE_HW=1 \
 *       -DROBOT_SIMULATION=0 -DS3_BARCODE_HW=1 test_line_learn_host.c -o tl && ./tl
 */
#include <assert.h>
#define tk_ref_flg(a,b) ((b)->flgptn=0xFFFFFFFFu,0)
#include "../../app_program/subsystem3/line.c"
/* stubs */
ID gh_system_flg = 1, gh_line_tsk=1;
uint32_t board_millis(void){return 0;} uint32_t board_micros(void){return 0;}
bool controller_post_event(system_event_t const*p){(void)p;return true;}
bool barcode_scan_active(void){return false;}
bool barcode_result_take(barcode_result_t*p){(void)p;return false;}
uint8_t route_take_junction_action(char*c){*c=0;return 0;}
void route_init(void){} void route_poll_config(void){}
void route_store_waypoint(barcode_result_t const*p,uint32_t t){(void)p;(void)t;}
void mqtt_utk_route_rx(char const*p,uint32_t l){(void)p;(void)l;}
void app_check(ER e,char const*s){(void)e;(void)s;}
int main(void){
  ir_calibration_t cal={IR_DEFAULT_WHITE_COUNTS,IR_DEFAULT_BLACK_COUNTS,false};
  ir_learn_t L; line_learn_init(&L,&cal);
  /* real tape reads 1500 black, floor 300 (defaults assume 2200) */
  float before=line_normalise(1500,&cal);
  for(int i=0;i<50;i++) line_learn(&L,&cal,300,0.0f,"t");
  for(int i=0;i<50;i++) line_learn(&L,&cal,1500,0.0f,"t");
  float after=line_normalise(1500,&cal);
  printf("tape 1500 counts: normalised %.2f with defaults -> %.2f learned (junction level %.2f)\n",before,after,LINE_JUNCTION_LEVEL);
  assert(before<LINE_JUNCTION_LEVEL && after>=0.99f && cal.b_calibrated);
  /* a 1-2 cycle spike must not raise black */
  line_learn(&L,&cal,4000,0.0f,"t"); line_learn(&L,&cal,4000,0.0f,"t"); line_learn(&L,&cal,1500,0.0f,"t");
  assert(cal.black==1500); puts("2-cycle spike ignored: ok");
  /* ramp lifts the front: both sensors read ~3800 for 0.5 s -> ignored */
  for(int i=0;i<50;i++) line_learn(&L,&cal,3800,0.95f,"t");
  assert(cal.black==1500);
  printf("lifted front (both dark 0.5 s): black stays %u; tape still %.2f >= %.2f\n",
         (unsigned)cal.black, line_normalise(1500,&cal), LINE_JUNCTION_LEVEL);
  /* junction detector at the new level */
  { line_junction_t j={0}; float t=line_normalise(1400,&cal);
    bool a=line_junction_update(&j,t,t), b=line_junction_update(&j,t,t);
    printf("tape at 1400 counts = %.2f -> junction %s\n", t, (b&&!a)?"confirmed in 2 cycles":"MISSED"); assert(b && !a); }
  return 0; }
ER ir_sampler_start(void){return 0;}
void ir_sampler_drain(ir_window_t*w){w->count=0;}
