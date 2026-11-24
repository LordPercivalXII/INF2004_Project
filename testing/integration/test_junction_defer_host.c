/* PC test: S3 junction reached while a barcode is still being read
 * (barcode close to the junction), line.c line_publish().
 *   gcc -std=gnu99 -w -Ihost_stubs -I../../app_program/subsystem3 \
 *       -I../../app_program/common -I../../app_program -DS3_LINE_HW=1 \
 *       -DROBOT_SIMULATION=0 -DS3_BARCODE_HW=1 test_junction_defer_host.c \
 *       -o tj && ./tj
 */
#include <assert.h>
#include <string.h>
#define tk_ref_flg(a,b) ((b)->flgptn=g_flags,0)
static unsigned g_flags;
#include "../../app_program/subsystem3/line.c"

/* ---- stubs with controllable state ---- */
ID gh_system_flg = 1, gh_line_tsk = 1;
static uint32_t g_ms, g_us;
uint32_t board_millis(void){return g_ms;}
uint32_t board_micros(void){return g_us;}

static bool g_scan; static uint32_t g_elems, g_last_bar_us;
static bool g_have_result; static barcode_result_t g_result;
static int g_decode_requests;
bool barcode_scan_active(void){return g_scan;}
uint32_t barcode_scan_elements(void){return g_scan ? g_elems : 0U;}
uint32_t barcode_last_bar_end_us(void){return g_last_bar_us;}
void barcode_request_decode(void){g_decode_requests++;}
bool barcode_result_take(barcode_result_t*p){
  if(!g_have_result) return false; *p=g_result; g_have_result=false; return true;}

static char g_pending; static int g_takes;
void route_store_waypoint(barcode_result_t const*p,uint32_t t){(void)t;g_pending=p->text[0];}
uint8_t route_take_junction_action(char*c){g_takes++;*c=g_pending;
  uint8_t a = (g_pending=='L')?BARCODE_CMD_LEFT:BARCODE_CMD_NONE; g_pending=0; return a;}
void route_init(void){} void route_poll_config(void){}
void mqtt_utk_route_rx(char const*p,uint32_t l){(void)p;(void)l;}
void app_check(ER e,char const*s){(void)e;(void)s;}
ER ir_sampler_start(void){return 0;}
void ir_sampler_drain(ir_window_t*w){w->count=0;}

static system_event_t g_last; static int g_junction_events;
bool controller_post_event(system_event_t const*p){
  g_last=*p; if(p->data.line.b_junction) g_junction_events++; return true;}

static line_ctx_t ctx;
static void reset(void){
  memset(&ctx,0,sizeof ctx); g_scan=false; g_elems=0; g_have_result=false;
  g_pending=0; g_takes=0; g_junction_events=0; g_decode_requests=0;
  g_flags = SYSTEM_BIT_MISSION_RUN; g_ms=1000; g_us=1000000; g_last_bar_us=0; }
static void cycle(bool junction){ g_ms+=10; g_us+=10000; line_publish(&ctx,0.0f,true,junction,g_ms); }
static void decoded(char c, uint32_t last_bar_us){
  memset(&g_result,0,sizeof g_result); g_result.text[0]=c;
  g_result.last_bar_end_us=last_bar_us; g_have_result=true; g_scan=false; }

int main(void){
  /* 1. far barcode (decoded long before): junction acts at once with it */
  reset(); decoded('L', g_us); cycle(false); assert(g_pending=='L');
  for(int i=0;i<30;i++) cycle(false);
  cycle(true);
  assert(g_junction_events==1 && g_last.data.line.junction_action==BARCODE_CMD_LEFT);
  assert(g_last.data.line.junction_age_ms==0);
  puts("far barcode: junction turns at once: ok");

  /* 2. barcode decoded a few ms before the junction: used, never dropped
        (the old 300 ms hold discarded this junction) */
  reset(); cycle(false); decoded('L', g_us); cycle(false); cycle(true);
  assert(g_junction_events==1 && g_last.data.line.junction_action==BARCODE_CMD_LEFT);
  puts("barcode decoded just before junction: used, not dropped: ok");

  /* 3. junction reached while the scan waits for its quiet zone: decision
        waits, decode is requested, barcode is used for THIS junction */
  reset(); g_scan=true; g_elems=29; g_last_bar_us=g_us-80000; /* last bar 80 ms ago */
  cycle(true);
  assert(g_junction_events==0 && ctx.b_junction_deferred && g_decode_requests==1);
  cycle(false); cycle(false);
  decoded('L', g_last_bar_us); cycle(false);
  assert(g_junction_events==1 && g_last.data.line.junction_action==BARCODE_CMD_LEFT);
  assert(g_last.data.line.junction_age_ms==30 && g_takes==1);
  puts("junction during scan: waits 30 ms, turns LEFT with this barcode: ok");

  /* 4. "junction" is a wide bar of the barcode itself: bars continue
        after it, decode later -> junction dropped, waypoint kept */
  reset(); g_scan=true; g_elems=6; g_last_bar_us=g_us;
  cycle(true); uint32_t j_us=g_us;
  assert(ctx.b_junction_deferred);
  cycle(false); g_last_bar_us=g_us; cycle(false); g_last_bar_us=g_us;
  decoded('L', j_us+40000); cycle(false);
  assert(g_junction_events==0 && g_pending=='L' && g_takes==0);
  cycle(true);   /* the real junction afterwards */
  assert(g_junction_events==1 && g_last.data.line.junction_action==BARCODE_CMD_LEFT);
  puts("barcode's own wide bar: no junction, barcode kept for real one: ok");

  /* 5. scan ends without a decode: junction acts on the stored waypoint */
  reset(); g_pending='L'; g_scan=true; g_elems=5; g_last_bar_us=g_us-50000;
  cycle(true); cycle(false); g_scan=false; cycle(false);
  assert(g_junction_events==1 && g_last.data.line.junction_action==BARCODE_CMD_LEFT);
  assert(g_last.data.line.junction_age_ms==20);
  puts("failed read: junction uses stored waypoint: ok");

  /* 6. scan never resolves: bounded wait, then act */
  reset(); g_scan=true; g_elems=8; g_last_bar_us=g_us-60000;
  cycle(true);
  for(uint32_t t=0;t+10<LINE_JUNCTION_DEFER_MAX_MS;t+=10){ cycle(false); assert(g_junction_events==0); }
  cycle(false);
  assert(g_junction_events==1 && !ctx.b_junction_deferred);
  assert(g_last.data.line.junction_age_ms==LINE_JUNCTION_DEFER_MAX_MS);
  puts("scan stuck: acts after LINE_JUNCTION_DEFER_MAX_MS with age: ok");

  /* 7. lone dark patch on GP28 (the junction tape itself): no wait */
  reset(); g_pending='L'; g_scan=true; g_elems=1; cycle(true);
  assert(g_junction_events==1 && g_decode_requests==0);
  puts("junction tape on barcode sensor: no wait: ok");

  puts("all junction deferral tests passed");
  return 0;
}
