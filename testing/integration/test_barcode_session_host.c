/* Build from testing/integration:
 *   gcc -std=gnu99 -w -Ihost_stubs -I../../app_program/subsystem3 \
 *       -I../../app_program/common -I../../app_program -DS3_BARCODE_DEBUG=0 \
 *       -DS3_BARCODE_HW=0 -DROBOT_SIMULATION=1 test_barcode_session_host.c \
 *       ../../app_program/subsystem3/code39.c -o tb && ./tb
 */
/* Host test: barcode.c session logic (early decode, overflow slide, junk). */
#include <assert.h>
#include <string.h>
#include "../../app_program/subsystem3/barcode.c"
ID gh_barcode_edge_mbf = 1, gh_barcode_result_mtx = 1;
INT tk_rcv_mbf(ID id, void *m, TMO t){(void)id;(void)m;(void)t;return 0;}
uint32_t board_micros(void){return 0;}
uint32_t board_millis(void){return 0;}
void barcode_sensor_take_counts(uint32_t*a,uint32_t*b){*a=*b=0;}
bool barcode_sensor_take_overflow(void){return false;}
uint8_t barcode_sensor_is_black(void){return 0;}
bool barcode_sensor_init(void){return true;}

static barcode_session_t s; static uint32_t t_us;
static void feed(uint32_t black, uint32_t w){ barcode_element_t e={0}; e.is_black=black; e.width_us=w; t_us+=w; e.end_us=t_us; barcode_session_add(&s,&e); }
/* feed "*X*" with bar/space bias (bars shrink, spaces grow) */
static void feed_code(char c, uint32_t narrow, uint32_t wide, uint32_t bias){
  uint32_t w[60]; char p[2]={c,0}; uint32_t n=code39_encode_widths(p,narrow,wide,w,60); assert(n==29);
  for(uint32_t i=0;i<n;i++){ uint32_t black=(i%2)==0; feed(black, black? w[i]-bias : w[i]+bias); } }
static bool take(char *c){ barcode_result_t r; if(!barcode_result_take(&r)) return false; *c=r.text[0]; return true; }

int main(void){
  char c;
  barcode_session_reset(&s);
  /* 1: clean code then a quiet white -> decoded at once, no end-gap wait */
  feed(0,200000); feed_code('F',13000,32000,0); feed(0,300000);
  assert(take(&c) && c=='F'); puts("1 early decode on quiet zone: ok");
  /* 2: line-tape junk (long black/white) then the barcode */
  feed(1,1600000); feed(0,900000); feed(1,100000); feed(0,250000);
  feed_code('B',13000,32000,0); feed(0,300000);
  assert(take(&c) && c=='B'); puts("2 junk before barcode: ok");
  /* 3: 120 junk elements first (buffer is 99): used to overflow and fail */
  for(int i=0;i<60;i++){ feed(1,40000+(i%7)*3000); feed(0,60000+(i%5)*4000); }
  feed(0,400000); /* merges into last white */
  feed_code('D',13000,32000,0); feed(0,300000);
  assert(take(&c) && c=='D'); puts("3 overflow by junk then barcode: ok");
  /* 4: threshold bias like the log (narrow space ~2x narrow bar) */
  feed_code('C',18000,40000,5000); feed(0,300000);
  assert(take(&c) && c=='C'); puts("4 bar/space bias 5 ms: ok");
  /* 5: partial code then quiet: no false decode */
  { uint32_t w[60]; uint32_t n=code39_encode_widths("A",13000,32000,w,60);
    for(uint32_t i=0;i<20;i++) feed((i%2)==0,w[i]); feed(0,300000); assert(!take(&c)); puts("5 partial: no decode (ok)"); }
  puts("all barcode session tests passed"); return 0; }
