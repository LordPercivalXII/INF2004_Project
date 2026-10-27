/*
 *----------------------------------------------------------------------
 *    Device Driver for μT-Kernel 3.00.05
 *
 *    Copyright (C) 2020-2021 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2021/11.
 *
 *----------------------------------------------------------------------
 */


/*
 *	config_device.h
 *	Device configuration definition
 */

#ifndef	__DEV_CONFIG_H__
#define	__DEV_CONFIG_H__

/* ------------------------------------------------------------------------ */
/* Device usage settings
 *	1: Use   0: Do not use
 */

#define DEVCNF_USE_SER		1		// Serial communication device 
#define DEVCNF_USE_ADC		0		// A/D conversion device
		/* 0: the ADC is owned by subsystem 3 (drv_ir_sampler.c), which
		   samples it from a 1 kHz interrupt.  See TEAM.md section 3. */
#define DEVCNF_USE_IIC		1		// I2C communication device
		/* 1: "iica" (I2C0 on GP4/GP5) is used by subsystem 4 for the
		   MPU6050.  See TEAM.md section 3. */
		/* Off by default so no pins are claimed at boot.  To use I2C,
		   set this to 1 and choose the pins in
		   device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h. */

#endif	/* __DEV_CONFIG_H__ */