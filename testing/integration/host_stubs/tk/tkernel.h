#pragma once
/* host stub of the few micro T-Kernel names barcode.c uses */
#include <stdint.h>
#include <stdio.h>
typedef int ID; typedef int INT; typedef int TMO; typedef int ER; typedef unsigned char UB; typedef long SZ; typedef unsigned int UINT; typedef int BOOL; typedef void (*FP)(void); typedef int PRI; typedef unsigned int UW; typedef unsigned int RELTIM; typedef int ATR;
#define E_OK 0
#define E_ID (-18)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TSK_SELF 0
static inline INT tk_can_wup(ID i){(void)i;return 0;}
static inline ER tk_slp_tsk(TMO t){(void)t;return 0;}
static inline ER tk_dly_tsk(TMO t){(void)t;return 0;}
typedef struct { unsigned int wtsk; unsigned int flgptn; } T_RFLG;
typedef struct { int dummy; } T_CMBF; typedef struct { int dummy; } T_CFLG;
#define tm_printf(...) printf((const char*)__VA_ARGS__)
static inline ER tk_loc_mtx(ID i, TMO t){(void)i;(void)t;return E_OK;}
static inline ER tk_unl_mtx(ID i){(void)i;return E_OK;}
INT tk_rcv_mbf(ID id, void *msg, TMO tmo);
