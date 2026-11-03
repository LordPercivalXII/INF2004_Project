/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.00.03
 *
 *    Copyright (C) 2006-2021 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2021/03/31.
 *
 *----------------------------------------------------------------------
 */

/*
 *    libtm.c
 *    T-Monitor compatible calls library
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#if USE_TMONITOR
#include "libtm.h"

#if TK_SUPPORT_SMP && TM_CONSOLE_USB_CDC
#include <tk/smp.h>

LOCAL T_SPLOCK tm_output_lock;
LOCAL UINT tm_output_depth[TK_MAX_CORE];
EXPORT volatile UW tm_printf_smp_contentions;

/* printf emits through putchar/putstring; nested calls on the same core
   must share the outer lock without claiming it a second time. */
EXPORT void tm_console_lock(UINT *intsts)
{
	UINT saved;
	UINT ignored;
	INT core;

	DI(saved);
	core = tk_get_prc() - 1;
	if(tm_output_depth[core] == 0) {
		if(ISpinTryLock(&tm_output_lock, &ignored) != E_OK) {
			(void)atomic_inc((UW *)&tm_printf_smp_contentions);
			(void)ISpinLock(&tm_output_lock, &ignored);
		}
	}
	tm_output_depth[core]++;
	*intsts = saved;
}

EXPORT void tm_console_unlock(UINT intsts)
{
	INT core = tk_get_prc() - 1;

	if(--tm_output_depth[core] == 0) {
		(void)ISpinUnlock(&tm_output_lock, intsts);
	} else {
		EI(intsts);
	}
}
#endif

/*
 * libtm_init() - libtm Initialize
 * supported only on wait != 0 (polling not supported)
 */
EXPORT void libtm_init(void)
{
	tm_com_init();
}

/*
 * tm_getchar() - Get Character
 * supported only on wait != 0 (polling not supported)
 */
EXPORT INT tm_getchar( INT wait )
{
	UB	p;
	UINT	imask;

	DI(imask);
	tm_rcv_dat(&p, 1);
	EI(imask);

	return (INT)p;
}

/*
 * tm_getline() - Get Line
 * special key is not supported
 */
EXPORT INT tm_getline( UB *buff )
{
	UB* p = buff;
	int len = 0;
	static const char LF = CHR_LF;
	INT imask;

	DI(imask);
	while (1) {
		tm_rcv_dat(p, 1);
		tm_snd_dat(p, 1); /* echo back */
		if (*p == CHR_CR) {
			tm_snd_dat((const UB*)&LF, 1);
			break;
		} else if (*p == CHR_ETX) {
			len = -1;
			break;
		}
		p++; len++;
	}
	*p = 0x00;
	EI(imask);

	return len;
}

/*
 * tm_putchar()
 * Ctrl-C is not supported
 */
EXPORT INT tm_putchar( INT c )
{
	static const char CR = CHR_CR;
	UB buf = (UB)c;
	UINT imask;

#if TK_SUPPORT_SMP && TM_CONSOLE_USB_CDC
	tm_console_lock(&imask);
#else
	DI(imask);
#endif
	if (buf == CHR_LF) {
		tm_snd_dat((const UB*)&CR, 1);
	}
	tm_snd_dat(&buf, 1);
#if TK_SUPPORT_SMP && TM_CONSOLE_USB_CDC
	tm_console_unlock(imask);
#else
	EI(imask);
#endif

	return 0;
}

/*
 * tm_putstring() - Put String
 * Ctrl-C is not supported
 */
EXPORT INT tm_putstring( const UB *buff )
{
	const UB* p = buff;
	UINT imask;

#if TK_SUPPORT_SMP && TM_CONSOLE_USB_CDC
	tm_console_lock(&imask);
#else
	DI(imask);
#endif
	while ( *p != (UB)'\0' ) {
		tm_putchar(*p++);
	}
#if TK_SUPPORT_SMP && TM_CONSOLE_USB_CDC
	tm_console_unlock(imask);
#else
	EI(imask);
#endif

	return 0;
}

#endif /* USE_TMONITOR */