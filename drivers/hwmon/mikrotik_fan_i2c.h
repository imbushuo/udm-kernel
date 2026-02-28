/* SPDX-License-Identifier: GPL-2.0 */
/*
 * fan_i2c - MikroTik CCR2004-1G-2XS-PCIe fan controller MCU driver
 *
 * Communicates with an external MCU over I2C to control fans, read
 * temperature/voltage/current sensors, and monitor PSU status.
 */
#ifndef _MIKROTIK_FAN_I2C_H
#define _MIKROTIK_FAN_I2C_H

#include <linux/types.h>

/* ---- MCU I2C Register Addresses ---- */
#define MCU_REG_CONFIG		0x00	/* R   128B  Configuration (CRC32) */
#define MCU_REG_RESPONSE	0x80	/* R     8B  BL command response */
#define MCU_REG_VERSION		0x90	/* R     4B  FW/BL version (XOR) */
#define MCU_REG_STATUS		0x94	/* R     4B  Fan fault status (XOR) */
#define MCU_REG_COMMAND		0x98	/* W     3B  Command (PWM/detect) */
#define MCU_REG_SENSOR		0x9B	/* R  32/68B Sensor data (CRC32) */
#define MCU_REG_FW_WRITE	0xB8	/* W    72B  Firmware chunk write */
#define MCU_REG_BREADCRUMB_LO	0xC0	/* R/W   4B  Breadcrumb (fw<=0x4FF) */
#define MCU_REG_BREADCRUMB_HI	0xE4	/* R/W   4B  Breadcrumb (fw>0x4FF) */
#define MCU_REG_FW_ERASE	0xF8	/* W     8B  Flash erase command */

/* ---- Constants ---- */
#define FAN_POLL_INTERVAL	450	/* jiffies between sensor polls */
#define FAN_FW_MAX_SIZE		0x7000	/* max firmware image size (28672) */
#define FAN_FW_CHUNK_SIZE	64	/* firmware write granularity */
#define FAN_FW_HEADER_SIZE	64	/* firmware file header size */
#define FAN_I2C_WRITE_RETRIES	5	/* max I2C write retries */
#define FAN_I2C_READ_RETRIES	10	/* max I2C CRC/XOR read retries */
#define FAN_MCU_POLL_COUNT	100	/* max response polls */
#define FAN_MCU_POLL_DELAY_US	10000	/* 10ms between response polls */
#define FAN_MCU_RESET_DELAY_MS	1	/* delay per loop iteration */
#define FAN_MCU_RESET_LOOPS	200	/* ~200ms total reset wait */
#define FAN_MCU_BL_LOOPS	100	/* ~100ms total BL wait */
#define FAN_BL_VERSION_BIT	0x8000	/* bit 15 set = bootloader mode */
#define FAN_FAULT_MASK		0x0FFC	/* bits 2-11 = individual fan faults */
#define FAN_FW_VER_EXT_THRESH	0x0500	/* fw versions above this use extended sensor data */
#define FAN_CONFIG_SIZE		128	/* MCU config register size */
#define FAN_SENSOR_SIZE_STD	32	/* standard sensor data size */
#define FAN_SENSOR_SIZE_EXT	68	/* extended sensor data size */

/* ---- MCU Command Bytes ---- */
#define MCU_CMD_PWM		0x23
#define MCU_CMD_DETECT		0x22
#define MCU_CMD_DETECT2		0x83
#define MCU_CMD_FW_ERASE	0x11

/* ---- Sensor Labels (index into label table) ---- */
#define LABEL_NONE		0
#define LABEL_FAN		1
#define LABEL_PSU		2
#define LABEL_BOARD		3
#define LABEL_CPU		4
#define LABEL_SFP		5
#define LABEL_POE		6
#define LABEL_SYSTEM		7
#define LABEL_TOTAL_POE_POWER	8
#define LABEL_EXT_PIN2		9
#define LABEL_EXT_PIN3		10
#define LABEL_HIDE		11
#define LABEL_HIDE_REMOTE	12
#define LABEL_HIDE_CPU		13
#define LABEL_HIDE_SOC		14
#define LABEL_HIDE_AVDD		15
#define LABEL_JACK		16
#define LABEL_2PIN		17
#define LABEL_POE_IN		18
#define LABEL_POE_OUT		19
#define LABEL_SWITCH		20
#define LABEL_PHY		21
#define LABEL_SMART_PSU		22
#define LABEL_VOLTAGE		23
#define LABEL_TEMPERATURE	24
#define LABEL_AUX		25
#define LABEL_COUNT		26

/* ---- ADC Sensor Types ---- */
#define ADC_TYPE_TEMP		1
#define ADC_TYPE_VOLTAGE	2
#define ADC_TYPE_CURRENT	3

/* ---- MCU Configuration Structure (128 bytes, register 0x00) ---- */
struct mcu_fan_cfg {
	u8 rpm_ch_pwm_ch;	/* raw config byte 0 */
	u8 abs_fault;		/* absolute fault threshold */
	u8 type;		/* bits 7:3 = label index, bits 2:0 = PWM channel */
	u8 flags;
} __packed;

struct mcu_adc_cfg {
	u8 adc_ch;
	u8 type;		/* bits 7:3 = label index, bits 2:0 = ADC type (1=temp, 2=voltage, 3=current) */
	s16 mul;
	s16 div;
	s16 offset;
	s16 min;
	s16 max;
} __packed;

struct mcu_config {
	__le16 version;			/* +0x00 */
	__le32 pid;			/* +0x02 */
	u8 fan_count;			/* +0x06 */
	u8 fan_map;			/* +0x07 */
	u8 flags;			/* +0x08 */
	u8 reserved[3];			/* +0x09 */
	struct mcu_fan_cfg fan[4];	/* +0x0C (16 bytes) */
	struct mcu_adc_cfg adc[8];	/* +0x1C (96 bytes) */
	__le32 crc32;			/* +0x7C */
} __packed;

/* Flags */
#define MCU_CFG_FLAG_PSU	BIT(0)

/*
 * Sensor data from MCU register 0x9B.
 * Standard (fw <= 0x4FF): 32 bytes.  Extended (fw > 0x4FF): 68 bytes.
 * Last 4 bytes are always CRC32 of preceding data.
 *
 * We store the sensor cache as raw u16 arrays in the driver data.
 */

/* Maximum number of each sensor type */
#define MAX_FANS		12
#define MAX_TEMPS		6
#define MAX_VOLTAGES		2
#define MAX_CURRENTS		2
#define MAX_PSU			2

/* ---- PSU MFR Info Field Names ---- */
#define PSU_MFR_FIELD_COUNT	4	/* MFR_ID, MFR_MODEL, MFR_SN, MFR_REVISION */

/* ---- Driver Private Data ---- */
struct fan_i2c_data {
	struct i2c_client *client;

	struct device *hwmon_dev;

	/* GPIO pins */
	int rst_gpio;
	u8 rst_active_level;
	u8 rst_out_only;

	int irq_gpio;
	u8 irq_out_only;

	int boot0_gpio;

	/* Software fault mask (OR'd with hardware faults) */
	u32 fault_mask;

	/* Sensor data cache - standard (from register 0x9B) */
	u16 fan_rpm[4];		/* fan RPM channels 1-4 */
	s16 temp[4];		/* temperature sensors 1-4 (raw, *1000 = millideg C) */
	s16 power[4];		/* power/voltage sensors 1-4 */

	/* Sensor data cache - extended (fw > FAN_FW_VER_EXT_THRESH) */
	u16 ext_fan[6];		/* extended fan RPM channels 5-10 */
	u16 psu_data[2][6];	/* PSU1/PSU2 data: [power, ?, fan, voltage, current, temp] */

	/* LED trigger for fan-fault */
#ifdef CONFIG_LEDS_TRIGGERS
	struct led_trigger led_trig;
#endif

	/* Workqueue and delayed work for periodic polling */
	struct workqueue_struct *wq;
	struct delayed_work dwork;

	/* Polling state */
	unsigned long last_poll_jiffies;
	u8 stopped;
	u8 pwm_pending;
	u16 gpio_retry;

	/* Current PWM value (-1 = unset) */
	s32 pwm_value;

	/* MCU config (raw 128 bytes) */
	u8 config[FAN_CONFIG_SIZE];

	/* Current firmware version */
	s32 fw_version;
};

/* ---- Exported Symbols ---- */
int fan_write_breadcrumb(int value);
unsigned int fan_read_breadcrumb(void);

#endif /* _MIKROTIK_FAN_I2C_H */
