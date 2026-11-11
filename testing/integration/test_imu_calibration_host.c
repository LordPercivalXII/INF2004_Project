/* PC test for the S4 start-up calibration (old peak-to-peak rule vs new).
 *   gcc -std=gnu99 -w -I../../app_program/subsystem4 -I../../app_program/common \
 *       test_imu_calibration_host.c ../../app_program/subsystem4/imu_analysis.c \
 *       ../../app_program/common/rmath.c -o t -lm && ./t
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "imu_processing.h"

static float gauss(void){ float u=(rand()+1.0f)/(RAND_MAX+2.0f), v=(rand()+1.0f)/(RAND_MAX+2.0f);
  return sqrtf(-2.0f*logf(u))*cosf(6.2831853f*v); }

/* returns samples until calibrated (or -1), old rule computed in parallel */
static void run(float noise_g, float spike_rate, int *p_new, int *p_old)
{
  imu_processor_t p; imu_processor_init(&p);
  int n, newt=-1, oldt=-1, oldcount=0; float mn[3],mx[3];
  for(n=0;n<60000 && (newt<0||oldt<0);n++){
    imu_sample_t s={0};
    s.acceleration[0]=-0.010f+noise_g*gauss();
    s.acceleration[1]=-0.015f+noise_g*gauss();
    s.acceleration[2]= 0.965f+noise_g*gauss();
    if ((rand()/(float)RAND_MAX) < spike_rate) s.acceleration[2]+=0.03f; /* desk/cable bump */
    if (newt<0){ imu_processor_update(&p,&s,true,(uint32_t)(n*10)); if(p.output.b_calibrated) newt=n+1; }
    if (oldt<0){ /* old rule */
      int i, still = fabsf(s.acceleration[0])<0.15f && fabsf(s.acceleration[1])<0.15f &&
                     s.acceleration[2]>0.85f && s.acceleration[2]<1.15f;
      for(i=0;i<3;i++){ if(oldcount==0){mn[i]=mx[i]=s.acceleration[i];}
        if(s.acceleration[i]<mn[i])mn[i]=s.acceleration[i]; if(s.acceleration[i]>mx[i])mx[i]=s.acceleration[i];
        still = still && (mx[i]-mn[i]<=0.04f); }
      oldcount = still ? oldcount+1 : 0; if(oldcount==200) oldt=n+1; }
  }
  *p_new=newt; *p_old=oldt;
}
int main(void){
  float noise[]={0.003f,0.005f,0.007f,0.009f}; float spikes[]={0.0f,0.002f};
  srand(1);
  for(int i=0;i<4;i++) for(int j=0;j<2;j++){
    long sn=0,so=0; int failn=0,failo=0;
    for(int k=0;k<20;k++){ int a,b; run(noise[i],spikes[j],&a,&b);
      if(a<0)failn++; else sn+=a; if(b<0)failo++; else so+=b; }
    printf("noise %2.0f mg rms, bumps %.1f%%: new %5.2f s (fail %d/20)   old %6.2f s (fail %d/20, cap 600 s)\n",
      noise[i]*1000, spikes[j]*100,
      failn<20? sn/(20.0-failn)/100.0:-1, failn, failo<20? so/(20.0-failo)/100.0:-1, failo);
  }
  /* a hand on the car: large movement must never calibrate */
  { imu_processor_t p; imu_processor_init(&p); for(int n=0;n<3000;n++){ imu_sample_t s={0};
      s.acceleration[0]=0.08f*sinf(n*0.3f); s.acceleration[1]=0.06f*cosf(n*0.17f); s.acceleration[2]=0.98f+0.05f*sinf(n*0.5f);
      imu_processor_update(&p,&s,true,(uint32_t)(n*10)); assert(!p.output.b_calibrated);} puts("handled car: never calibrates (ok)"); }
  /* tilted (Z not up): never calibrates */
  { imu_processor_t p; imu_processor_init(&p); for(int n=0;n<3000;n++){ imu_sample_t s={0};
      s.acceleration[0]=0.30f; s.acceleration[2]=0.95f; imu_processor_update(&p,&s,true,(uint32_t)(n*10)); assert(!p.output.b_calibrated);} puts("tilted car: never calibrates (ok)"); }
  /* bias is the window mean */
  { imu_processor_t p; imu_processor_init(&p); int n; for(n=0;n<1000 && !p.output.b_calibrated;n++){ imu_sample_t s={0};
      s.acceleration[0]=-0.010f+0.004f*gauss(); s.acceleration[1]=0.020f+0.004f*gauss(); s.acceleration[2]=0.970f+0.004f*gauss();
      imu_processor_update(&p,&s,true,(uint32_t)(n*10)); }
    assert(p.output.b_calibrated); printf("bias = %.4f %.4f %.4f g (expect -0.010 0.020 -0.030)\n", p.bias[0],p.bias[1],p.bias[2]);
    assert(fabsf(p.bias[0]+0.010f)<0.002f && fabsf(p.bias[1]-0.020f)<0.002f && fabsf(p.bias[2]+0.030f)<0.002f); }
  /* late samples (timing gap) do not restart the window */
  { imu_processor_t p; imu_processor_init(&p); uint32_t t=0; int n; for(n=0;n<1000 && !p.output.b_calibrated;n++){ imu_sample_t s={0};
      s.acceleration[2]=0.97f+0.003f*gauss(); t += (n%50==49)?45U:10U; imu_processor_update(&p,&s,true,t);} 
    printf("with a 45 ms gap every 50 samples: calibrated after %d samples\n", n); assert(n<260); }
  puts("all IMU calibration checks passed"); return 0; }
