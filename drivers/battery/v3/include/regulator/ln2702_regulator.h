/*
 * ln2702_regulator.h
 * Samsung LN2702 Voltage Regulator Header
 *
 * Copyright (C) 2019 Samsung Electronics, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LN2702_REGULATOR_H
#define __LN2702_REGULATOR_H __FILE__

#include "../sec_charging_common.h"

#define BITS(_end, _start) ((BIT(_end) - BIT(_start)) + BIT(_end))
#define MASK2SHIFT(_mask)	__ffs(_mask)
#define MIN(a, b)	   ((a < b) ? (a):(b))

#define LN2702_DRIVER_NAME	"ln2702-regulator"
#define LN2702_HW_REV_A1	//HW rev.

/*********************************************************************************
 * register map
 *********************************************************************************/

// masks
#define LN2702_MASK_INFET_EN		0x01
#define LN2702_MASK_SC_OPERATION_MODE	0x03

#ifdef LN2702_HW_REV_A1
   #define LN2702_MASK_BC_SYS		0x0F
#elif  LN2702_HW_REV_B0
   #define LN2702_MASK_BC_SYS		0x1F
#endif

// register addresses
#define LN2702_REG_DEVICE_ID		0x00
#define LN2702_REG_INT_DEVICE_0		0x01
#define LN2702_REG_INT_DEVICE_1		0x02
#define LN2702_REG_INT_HV_SC_0		0x03
#define LN2702_REG_INT_HV_SC_1		0x04
#define LN2702_REG_INT_DEVICE_0_MSK	0x05
#define LN2702_REG_INT_DEVICE_1_MSK	0x06
#define LN2702_REG_INT_HV_SC_0_MSK	0x07
#define LN2702_REG_INT_HV_SC_1_MSK	0x08
#define LN2702_REG_INT_DEVICE_0_STS	0x09
#define LN2702_REG_INT_DEVICE_1_STS	0x0A
#define LN2702_REG_INT_HV_SC_0_STS	0x0B
#define LN2702_REG_INT_HV_SC_1_STS	0x0C
#define LN2702_REG_DEVICE_CTRL_0	0x0D
#define LN2702_REG_DEVICE_CTRL_1	0x0E
#define LN2702_REG_HV_SC_CTRL_0		0x0F
#define LN2702_REG_HV_SC_CTRL_1		0x10
#define LN2702_REG_HV_SC_CTRL_2		0x11
#define LN2702_REG_SC_DITHER_CTRL	0x12
#define LN2702_REG_GLITCH_CTRL		0x13
#define LN2702_REG_FAULT_CTRL		0x14
#define LN2702_REG_TRACK_CTRL		0x15

#define LN2702_REG_STS_D		0x3A
#define LN2702_REG_DEVICE_MARKER	0x46
#define LN2702_MAX_REGISTER		LN2702_REG_DEVICE_MARKER

/*********************************************************************************
 * data structures / platform data
 *********************************************************************************/

// (high-level) operation mode
enum {
    LN2702_OPMODE_UNKNOWN = -1,
    LN2702_OPMODE_STANDBY = 0,
    LN2702_OPMODE_BYPASS  = 1,
    LN2702_OPMODE_SWITCHING = 2,
    LN2702_OPMODE_SWITCHING_ALT = 3,
};

// chip (internal) system state
enum {
    LN2702_STATE_UNKNOWN = -1,
    LN2702_STATE_IDLE = 2,
    LN2702_STATE_SW_ACTIVE  = 7,
    LN2702_STATE_BYPASS_ACTIVE  = 12,
};

//
enum {
	REG_INT_DEVICE_0,
	REG_INT_DEVICE_1,
	REG_INT_HV_SC_0,
	REG_INT_HV_SC_1,
	REG_INT_MAX
};

enum {	
	REG_DEVICE_0_STS,
	REG_DEVICE_1_STS,
	REG_HV_SC_0_STS,
	REG_HV_SC_1_STS,
	REG_STS_MAX
};

/* LN2702_REG_DEVICE_CTRL_0 0x0D */
#define LN2702_BIT_INFET_EN			BIT(0)

/* LN2702_REG_HV_SC_CTRL_0 0x0F */
#define LN2702_BITS_FSW_CFG				BITS(7,3)
#define LN2702_BIT_STANDBY_EN				BIT(2)
#define LN2702_BITS_SC_OPERATION_MODE		BITS(1,0)

struct ln2702_regulator_platform_data {
	int irq_gpio;
	int en_gpio;
};

#define ln2702_regulator_platform_data_t \
	struct ln2702_regulator_platform_data

/**
 * struct ln2702_info - ln2702 regulator instance
 * @monitor_wake_lock: lock to enter the suspend mode
 * @lock: protects concurrent access to online variables
 * @client: pointer to client
 * @regmap: pointer to driver regmap
 * @op_mode : chip operation mode (STANDBY, BYPASS, SWITCHING)
 * @reverse_power : enable reverse power path
 * @pdata: pointer to platform data
 */
struct ln2702_info {
	struct i2c_client       *client;
	struct device           *dev;
	struct mutex		lock;
	struct regmap		*regmap;

	int irq;

	struct power_supply *psy_regulator;
	ln2702_regulator_platform_data_t 	*pdata;

	/* battery info */
	int cable_type;

	unsigned int		op_mode;
	bool			reverse_power;
	bool			auto_recovery;

};

#endif /* __LN2702_REGULATOR_H */
