// SPDX-License-Identifier: GPL-2.0
/*
 * fan_i2c - MikroTik CCR2004-1G-2XS-PCIe fan controller MCU driver
 *
 * This driver communicates with an external MCU over I2C to manage:
 *   - Fan PWM control and RPM monitoring
 *   - Temperature, voltage, and current sensing
 *   - PSU status and manufacturer info
 *   - MCU firmware upgrades
 *
 * The MCU uses a register-based I2C protocol with CRC32 and XOR
 * checksums for data integrity.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/leds.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include "mikrotik_fan_i2c.h"

/* LED trigger helpers - no-op when CONFIG_LEDS_TRIGGERS is disabled */
#ifdef CONFIG_LEDS_TRIGGERS
static inline void fan_led_event(struct fan_i2c_data *data, int brightness)
{
	led_trigger_event(&data->led_trig, brightness);
}
#else
static inline void fan_led_event(struct fan_i2c_data *data, int brightness) {}
#endif

/* ---- Global State ---- */
static struct fan_i2c_data *g_fan_data;
static u16 g_fan_fault_state;
static u16 g_fan_fault_bits;

/* Firmware upgrade state */
static u32 g_fw_total_size;
static u32 g_fw_target_ver;
static u32 g_current_fw_ver;
static u8 g_fw_header[FAN_FW_HEADER_SIZE]; /* saved header for writing last */

/* ---- Sensor Label Table ---- */
static const char * const sensor_labels[LABEL_COUNT] = {
	[LABEL_NONE]		= "",
	[LABEL_FAN]		= "fan",
	[LABEL_PSU]		= "psu",
	[LABEL_BOARD]		= "board",
	[LABEL_CPU]		= "cpu",
	[LABEL_SFP]		= "sfp",
	[LABEL_POE]		= "poe",
	[LABEL_SYSTEM]		= "system",
	[LABEL_TOTAL_POE_POWER]	= "total_poe_power",
	[LABEL_EXT_PIN2]	= "ext-pin2",
	[LABEL_EXT_PIN3]	= "ext-pin3",
	[LABEL_HIDE]		= "hide",
	[LABEL_HIDE_REMOTE]	= "hide_remote",
	[LABEL_HIDE_CPU]	= "hide_cpu",
	[LABEL_HIDE_SOC]	= "hide_soc",
	[LABEL_HIDE_AVDD]	= "hide_avdd",
	[LABEL_JACK]		= "jack",
	[LABEL_2PIN]		= "2pin",
	[LABEL_POE_IN]		= "poe-in",
	[LABEL_POE_OUT]		= "poe-out",
	[LABEL_SWITCH]		= "switch",
	[LABEL_PHY]		= "phy",
	[LABEL_SMART_PSU]	= "smart-psu",
	[LABEL_VOLTAGE]		= "voltage",
	[LABEL_TEMPERATURE]	= "temperature",
	[LABEL_AUX]		= "aux",
};

/* PSU MFR field names */
static const char * const psu_mfr_fields[PSU_MFR_FIELD_COUNT] = {
	"MFR_ID", "MFR_MODEL", "MFR_SN", "MFR_REVISION",
};

static inline bool fan_i2c_is_extended(struct fan_i2c_data *data)
{
	return data->fw_version > FAN_FW_VER_EXT_THRESH;
}

/* ========================================================================
 * I2C Communication Primitives
 * ======================================================================== */

/*
 * i2c_write_reg - Write register byte + payload to MCU.
 * Returns number of payload bytes written, or negative error.
 */
static int i2c_write_reg(struct fan_i2c_data *data, u8 reg,
			 const void *buf, unsigned int len)
{
	struct i2c_client *client = data->client;
	struct i2c_msg msg;
	u8 txbuf[257]; /* 1 reg byte + up to 256 data bytes */
	int ret;

	if (len > sizeof(txbuf) - 1)
		return -EINVAL;

	txbuf[0] = reg;
	memcpy(&txbuf[1], buf, len);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len + 1;
	msg.buf = txbuf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret == 1)
		return len;
	return ret;
}

/*
 * i2c_write_reg_retry - Write with retry (up to 5 attempts, 100ms between).
 * Returns 0 on success, negative on failure.
 */
static int i2c_write_reg_retry(struct fan_i2c_data *data, u8 reg,
			       const void *buf, unsigned int len)
{
	int ret = -EIO;
	int i;

	for (i = 0; i < FAN_I2C_WRITE_RETRIES; i++) {
		ret = i2c_write_reg(data, reg, buf, len);
		if (ret == (int)len)
			return 0;
		msleep(100);
	}

	if (ret)
		printk(KERN_ERR "fan_i2c:%s offset==0x%02x rc==0x%02x\n",
		       "I2C error", reg, ret);
	return ret;
}

/*
 * i2c_read_reg - Read from MCU register.
 * Sends register byte, then reads data.
 * Returns number of bytes read, or 0 on failure.
 */
static int i2c_read_reg(struct fan_i2c_data *data, u8 reg,
			void *buf, unsigned int len)
{
	struct i2c_client *client = data->client;
	struct i2c_msg msgs[2];
	u8 reg_byte = reg;
	int ret;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg_byte;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret == 2)
		return len;
	return 0;
}

/*
 * i2c_read_reg_crc32 - Read with CRC32 validation and retry.
 * Last 4 bytes of read data must equal ~CRC32 of preceding bytes.
 * Returns 0 on success, -EIO on failure.
 */
static int i2c_read_reg_crc32(struct fan_i2c_data *data, u8 reg,
			      void *buf, unsigned int len)
{
	int data_len = len - 4;
	int i;

	for (i = 0; i < FAN_I2C_READ_RETRIES; i++) {
		u32 calc_crc, read_crc;

		if (i2c_read_reg(data, reg, buf, len) != (int)len)
			continue;

		calc_crc = ~crc32_le(~0, buf, data_len);
		memcpy(&read_crc, buf + data_len, 4);
		if (calc_crc == read_crc)
			return 0;
	}
	return -EIO;
}

/*
 * i2c_read_4b_xor_check - Read 4 bytes with XOR checksum.
 * Byte 3 must equal XOR of bytes 0-2. Retries up to 10 times.
 * Returns 4 on success, -EIO on failure.
 */
static int i2c_read_4b_xor_check(struct fan_i2c_data *data, u8 reg,
				 void *buf)
{
	u8 *b = buf;
	int i;

	for (i = 0; i < FAN_I2C_READ_RETRIES; i++) {
		if (i2c_read_reg(data, reg, buf, 4) != 4)
			continue;
		if ((b[0] ^ b[1] ^ b[2]) == b[3])
			return 4;
	}
	return -EIO;
}

/*
 * i2c_read_4b_checked - Read 4 bytes with XOR check.
 * Returns 0 on success, -EIO on failure.
 */
static int i2c_read_4b_checked(struct fan_i2c_data *data, u8 reg,
			       void *buf)
{
	if (i2c_read_4b_xor_check(data, reg, buf) == 4)
		return 0;
	return -EIO;
}

/* Forward declarations */
static void mcu_reset(struct fan_i2c_data *data);
static int gpio_probe(struct i2c_client *client, struct fan_i2c_data *data);

/*
 * i2c_master_send_checked - Send data, reset MCU on I2C timeout.
 */
static int i2c_master_send_checked(struct fan_i2c_data *data,
				   const void *buf, int len)
{
	int ret;

	ret = i2c_transfer_buffer_flags(data->client, (void *)buf, len, 0);
	if (ret == len)
		return 0;

	if (ret == -ETIMEDOUT) {
		/* Reset MCU to prevent I2C bus lockup */
		mcu_reset(data);
		printk(KERN_ERR "fan_i2c: i2c_master_send timed out, "
		       "reset fan_i2c to prevent possible buss lock.\n");
	} else if (ret >= 0) {
		return -EIO;
	}
	return ret;
}

/* ========================================================================
 * GPIO Helpers
 * ======================================================================== */

static void fan_gpio_set_value(int gpio, int value)
{
	struct gpio_desc *desc = gpio_to_desc(gpio);

	gpiod_set_raw_value_cansleep(desc, value);
}

static void fan_gpio_set_output(int gpio, int value)
{
	struct gpio_desc *desc = gpio_to_desc(gpio);

	gpiod_direction_output_raw(desc, value);
}

static void fan_gpio_set_input(int gpio)
{
	struct gpio_desc *desc = gpio_to_desc(gpio);

	gpiod_direction_input(desc);
}

/* ========================================================================
 * MCU Control
 * ======================================================================== */

static void schedule_poll_work(struct fan_i2c_data *data);

/*
 * mcu_reset - Reset the MCU via GPIO.
 *
 * Sequence: assert RST, brief delay, IRQ high, brief delay,
 *           deassert RST, wait ~200ms.
 */
static void mcu_reset(struct fan_i2c_data *data)
{
	int i;

	if (!(data->gpio_retry & 0x100)) {
		printk(KERN_ERR "fan_i2c: MCU reset failed, "
		       "gpios not ready yet\n");
		return;
	}

	schedule_poll_work(data);

	/* Assert RST */
	fan_gpio_set_output(data->rst_gpio, 0);
	fan_gpio_set_value(data->rst_gpio, data->rst_active_level);
	udelay(10);

	/* Set IRQ high */
	fan_gpio_set_output(data->irq_gpio, 1);
	udelay(10);

	/* Deassert RST */
	fan_gpio_set_value(data->rst_gpio, !data->rst_active_level);

	/* Wait ~200ms for MCU boot */
	for (i = 0; i < FAN_MCU_RESET_LOOPS; i++)
		udelay(1000);

	/* Release GPIOs if not out-only */
	if (!data->irq_out_only)
		fan_gpio_set_input(data->irq_gpio);
	if (!data->rst_out_only)
		fan_gpio_set_input(data->rst_gpio);
}

/*
 * mcu_activate_bootloader - Enter bootloader mode via GPIO.
 *
 * Sequence: assert RST, brief delay, IRQ low, ~1ms delay,
 *           deassert RST, wait ~100ms, IRQ high, release GPIOs.
 */
static void mcu_activate_bootloader(struct fan_i2c_data *data)
{
	int i;

	if (!(data->gpio_retry & 0x100)) {
		printk(KERN_ERR "fan_i2c: BL activation failed, "
		       "gpios not ready yet\n");
		return;
	}

	schedule_poll_work(data);

	/* Assert RST */
	fan_gpio_set_output(data->rst_gpio, 0);
	fan_gpio_set_value(data->rst_gpio, data->rst_active_level);
	udelay(10);

	/* Pull IRQ low (bootloader select) */
	fan_gpio_set_output(data->irq_gpio, 0);
	udelay(1000);

	/* Deassert RST */
	fan_gpio_set_value(data->rst_gpio, !data->rst_active_level);

	/* Wait ~100ms for bootloader */
	for (i = 0; i < FAN_MCU_BL_LOOPS; i++)
		udelay(1000);

	/* Set IRQ high */
	fan_gpio_set_output(data->irq_gpio, 1);

	/* Release GPIOs */
	if (!data->irq_out_only)
		fan_gpio_set_input(data->irq_gpio);
	if (!data->rst_out_only)
		fan_gpio_set_input(data->rst_gpio);
}

/*
 * read_fw_version - Read firmware/bootloader version from MCU.
 *
 * Returns version (u16), or negative error.
 * BL versions have bit 15 set (>= 0x8000).
 * Response format: [ver_lo, ver_hi, check_byte(must be 1), xor]
 */
static int read_fw_version(struct fan_i2c_data *data)
{
	u8 buf[4];
	int ret;

	ret = i2c_read_4b_checked(data, MCU_REG_VERSION, buf);
	if (ret)
		return ret;

	/* check_byte must be 1 */
	if (buf[2] != 1)
		return -EIO;

	return (u16)(buf[0] | (buf[1] << 8));
}

/*
 * mcu_wait_response - Wait for MCU response with specific status byte.
 * Polls register 0x80 (8 bytes), checks first byte.
 */
static int mcu_wait_response(struct fan_i2c_data *data, void *out_buf,
			     u8 expected_status)
{
	u8 buf[8];
	int i;

	for (i = 0; i < FAN_MCU_POLL_COUNT; i++) {
		memset(buf, 0xFF, sizeof(buf));
		if (i2c_read_reg(data, MCU_REG_RESPONSE, buf, 8) == 8 &&
		    buf[0] == expected_status)
			break;
		usleep_range(FAN_MCU_POLL_DELAY_US, FAN_MCU_POLL_DELAY_US + 1000);
	}

	memcpy(out_buf, buf, 8);
	return 8;
}

static void schedule_poll_work(struct fan_i2c_data *data)
{
	if (!data->stopped) {
		queue_delayed_work_on(WORK_CPU_UNBOUND, data->wq,
				      &data->dwork, FAN_POLL_INTERVAL);
		data->pwm_pending = 1;
	}
}

/* ========================================================================
 * Sensor Polling
 * ======================================================================== */

/*
 * poll_sensor_data - Read sensors and status from MCU.
 * Rate-limited to every FAN_POLL_INTERVAL jiffies.
 */
static int poll_sensor_data(struct fan_i2c_data *data)
{
	unsigned int sensor_len;
	int ret;
	u8 sensor_buf[FAN_SENSOR_SIZE_EXT];
	u8 status_buf[4];
	u16 fault_bits;

	/* Rate limit */
	if (time_before(jiffies, data->last_poll_jiffies + FAN_POLL_INTERVAL))
		return 0;

	/* Read sensor data */
	sensor_len = fan_i2c_is_extended(data) ?
		     FAN_SENSOR_SIZE_EXT : FAN_SENSOR_SIZE_STD;
	if (!i2c_read_reg_crc32(data, MCU_REG_SENSOR, sensor_buf, sensor_len)) {
		/* Standard sensor data: skip 2-byte header at offset 0 */
		memcpy(data->fan_rpm, &sensor_buf[2], 8);
		memcpy(data->temp, &sensor_buf[10], 8);
		memcpy(data->power, &sensor_buf[18], 8);

		/* Extended sensor data */
		if (fan_i2c_is_extended(data)) {
			memcpy(data->ext_fan, &sensor_buf[28], 12);
			memcpy(data->psu_data[0], &sensor_buf[40], 12);
			memcpy(data->psu_data[1], &sensor_buf[52], 12);
		}

		data->last_poll_jiffies = jiffies;
	}

	/* Read status register */
	ret = i2c_read_4b_checked(data, MCU_REG_STATUS, status_buf);
	if (ret || status_buf[2] != 2) {
		/* Communication error or invalid status - report all faults */
		fan_led_event(data, LED_FULL);
		g_fan_fault_bits |= FAN_FAULT_MASK;
		g_fan_fault_state = 1;
		return ret ? ret : -EIO;
	}

	data->last_poll_jiffies = jiffies;

	/* Combine hardware fault bits with software mask */
	fault_bits = (status_buf[0] | (status_buf[1] << 8)) | data->fault_mask;

	if (g_fan_fault_bits != fault_bits) {
		g_fan_fault_bits = fault_bits;
		fan_led_event(data, (fault_bits & FAN_FAULT_MASK) ?
				   LED_FULL : LED_OFF);
		g_fan_fault_state = (g_fan_fault_bits & FAN_FAULT_MASK) ? 1 : 0;
	}

	return 0;
}

/*
 * fan_timer_callback - Delayed work callback (~every 4.5s).
 */
static void fan_timer_callback(struct work_struct *work)
{
	struct fan_i2c_data *data =
		container_of(work, struct fan_i2c_data, dwork.work);
	u8 cmd[3];

	if (!data->stopped) {
		/* Write pending PWM value */
		if (data->pwm_pending) {
			if (data->pwm_value >= 0) {
				cmd[0] = MCU_CMD_PWM;
				cmd[1] = (u8)data->pwm_value;
				cmd[2] = cmd[1] ^ MCU_CMD_PWM;
				if (!i2c_write_reg_retry(data, MCU_REG_COMMAND,
							 cmd, 3))
					data->pwm_pending = 0;
			} else {
				data->pwm_pending = 0;
			}
		}

		/* Retry GPIO probe if needed (low byte = retry flag) */
		if (data->gpio_retry & 0xFF) {
			printk(KERN_INFO "fan_i2c: retrying gpio_probe\n");
			gpio_probe(data->client, data);
		}
	}

	poll_sensor_data(data);

	if (!data->stopped)
		queue_delayed_work_on(WORK_CPU_UNBOUND, data->wq,
				      &data->dwork, FAN_POLL_INTERVAL);
}

/* ========================================================================
 * hwmon Show/Store Functions
 * ======================================================================== */

/*
 * Helper: get fan_i2c_data from hwmon device attribute.
 */
static struct fan_i2c_data *fan_data_from_dev(struct device *dev)
{
	return dev_get_drvdata(dev);
}

/* Fan RPM (up to 12 channels) */
static ssize_t fan_input_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	int idx = to_sensor_dev_attr(attr)->index;
	int val;

	poll_sensor_data(data);

	switch (idx) {
	case 1 ... 4:
		val = data->fan_rpm[idx - 1];
		break;
	case 5 ... 10:
		val = data->ext_fan[idx - 5];
		break;
	case 11:
		val = (s16)data->psu_data[0][2]; /* PSU1 fan */
		break;
	case 12:
		val = (s16)data->psu_data[1][2]; /* PSU2 fan */
		break;
	default:
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", val);
}

/* Fan fault (individual bit from status register) */
static ssize_t fan_fault_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	int idx = to_sensor_dev_attr(attr)->index;
	u8 status_buf[4];
	u16 faults;
	int ret, val;

	ret = i2c_read_4b_checked(data, MCU_REG_STATUS, status_buf);
	faults = (status_buf[0] | (status_buf[1] << 8)) | data->fault_mask;

	if (ret || status_buf[2] != 2)
		val = 1;
	else
		val = (faults >> idx) & 1;

	return sprintf(buf, "%d\n", val);
}

/* Temperature in millidegrees Celsius (raw value * 1000) */
static ssize_t temp_input_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	int idx = to_sensor_dev_attr(attr)->index;
	int val;

	poll_sensor_data(data);

	switch (idx) {
	case 1 ... 4:
		val = data->temp[idx - 1] * 1000;
		break;
	case 5:
		val = (s16)data->psu_data[0][5] * 1000; /* PSU1 temp */
		break;
	case 6:
		val = (s16)data->psu_data[1][5] * 1000; /* PSU2 temp */
		break;
	default:
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", val);
}

/* Power/PSU sensor values */
static ssize_t power_input_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	int idx = to_sensor_dev_attr(attr)->index;
	int val;

	poll_sensor_data(data);

	switch (idx) {
	case 1 ... 4:
		val = data->power[idx - 1];
		break;
	case 5:
		val = (s16)data->psu_data[0][4]; /* PSU1 current */
		break;
	case 6:
		val = (s16)data->psu_data[1][4]; /* PSU2 current */
		break;
	case 7:
		val = (s16)data->psu_data[0][3]; /* PSU1 voltage */
		break;
	case 8:
		val = (s16)data->psu_data[1][3]; /* PSU2 voltage */
		break;
	case 9:
		val = data->psu_data[0][1]; /* PSU1 power */
		break;
	case 10:
		val = data->psu_data[1][1]; /* PSU2 power */
		break;
	default:
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", val);
}

/* Sensor label */
static ssize_t sensor_label_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int idx = to_sensor_dev_attr(attr)->index;

	if (idx < 0 || idx >= LABEL_COUNT)
		idx = 0;
	return sprintf(buf, "%s\n", sensor_labels[idx]);
}

/* Voltage value * 100 (millivolts) */
static ssize_t voltage_input_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int idx = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%d\n", 100 * idx);
}

/* Global fan fault state */
static ssize_t fan_state_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_fan_fault_state);
}

/* PWM store (0-255) */
static ssize_t pwm_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	unsigned long long val;
	u8 cmd[3];
	int ret;

	ret = kstrtoull(buf, 10, &val);
	if (ret)
		return ret;
	if (val > 255)
		return -EINVAL;

	cmd[0] = MCU_CMD_PWM;
	cmd[1] = (u8)val;
	cmd[2] = cmd[1] ^ MCU_CMD_PWM;

	ret = i2c_write_reg_retry(data, MCU_REG_COMMAND, cmd, 3);
	if (ret)
		return ret;

	data->pwm_value = val;
	return count;
}

/* Detect command store (used via detect2 attr on some boards) */
static ssize_t __maybe_unused detect_cmd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	unsigned long long val;
	u8 cmd[3];
	int ret;

	ret = kstrtoull(buf, 10, &val);
	if (ret)
		return ret;
	if (val > 255)
		return -EINVAL;

	cmd[0] = MCU_CMD_DETECT2;
	cmd[1] = (u8)val;
	cmd[2] = cmd[1] ^ MCU_CMD_DETECT2;

	ret = i2c_write_reg_retry(data, MCU_REG_COMMAND, cmd, 3);
	if (ret)
		return ret;

	return count;
}

/* Send fan detect command */
static ssize_t detect_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	u8 cmd[3] = { MCU_CMD_DETECT, MCU_CMD_DETECT, MCU_CMD_DETECT };
	int ret;

	ret = i2c_write_reg_retry(data, MCU_REG_COMMAND, cmd, 3);
	return sprintf(buf, "Fan detect command sent, rc==%d\n", ret);
}

/* Show firmware version */
static ssize_t version_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	u8 vbuf[4];
	int ret;

	ret = i2c_read_4b_xor_check(data, MCU_REG_VERSION, vbuf);
	if (ret == 4) {
		u16 ver = vbuf[0] | (vbuf[1] << 8);
		return sprintf(buf, "%d\n", ver);
	}

	if (vbuf[2] != 1)
		return -EIO;
	return ret;
}

/* Show raw fan input from status register (used via fan_input_raw attr) */
static ssize_t __maybe_unused fan_input_raw_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	u8 status_buf[4];
	u16 status;
	int ret;

	ret = i2c_read_4b_checked(data, MCU_REG_STATUS, status_buf);
	if (ret)
		return ret;

	if (status_buf[2] != 2)
		return -EIO;

	status = status_buf[0] | (status_buf[1] << 8);
	return sprintf(buf, "%d\n", status);
}

/* ========================================================================
 * sysfs Device Attributes (debug, config, status, PSU MFR)
 * ======================================================================== */

/*
 * Format MCU config to human-readable string.
 */
static int config_format_to_string(const u8 *cfg, char *buf)
{
	int len = 0;
	int i;
	u16 ver = cfg[0] | (cfg[1] << 8);
	u32 pid;

	memcpy(&pid, &cfg[2], 4);

	len += sprintf(buf + len, "cfgVer==%d\n", ver);
	len += sprintf(buf + len, "cfgPID==%d\n", pid);
	len += sprintf(buf + len, "cfgFanCnt==%d\n", cfg[6]);
	len += sprintf(buf + len, "cfgFanMap==0x%02x\n", cfg[7]);
	len += sprintf(buf + len, "cfgFlags==0x%02x\n", cfg[8]);

	/* Fan config entries at offset 12, 4 bytes each */
	for (i = 0; i < 4; i++) {
		const u8 *fe = &cfg[12 + 4 * i];

		len += sprintf(buf + len,
			"cfgFan[%d].rpmChPwmCh==0x%02x; .absFault==0x%02x; "
			".type==0x%02x; .flags==0x%02x;\n",
			i, fe[0], fe[1], fe[2], fe[3]);
	}

	/* ADC config entries at offset 28, 12 bytes each */
	for (i = 0; i < 8; i++) {
		const u8 *ae = &cfg[28 + 12 * i];
		s16 mul, div_, offset, min_, max_;

		memcpy(&mul, &ae[2], 2);
		memcpy(&div_, &ae[4], 2);
		memcpy(&offset, &ae[6], 2);
		memcpy(&min_, &ae[8], 2);
		memcpy(&max_, &ae[10], 2);

		len += sprintf(buf + len,
			"cfgAdc[%d].adcCh==%d; .type==0x%02x; "
			".min==%d; .max==%d; .mul==%d; .div==%d; "
			".offset==%d\n",
			i, ae[0], ae[1], min_, max_, mul, div_, offset);
	}

	len += sprintf(buf + len, "cfgCrc32==0x%08x\n",
		       *(u32 *)&cfg[124]);
	return len;
}

/* sysfs: show debug info (version, status, all sensor values) */
static ssize_t status_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	u8 vbuf[4], sbuf[4];
	u8 sensor_buf[FAN_SENSOR_SIZE_EXT];
	unsigned int sensor_len;
	int len = 0, ret;

	/* Read version */
	ret = i2c_read_4b_checked(data, MCU_REG_VERSION, vbuf);
	if (ret) {
		len += sprintf(buf + len, "Error %d\n", ret);
	} else if (vbuf[2] != 1) {
		len += sprintf(buf + len,
			       "Error ver[2] expected %d, but got %d\n",
			       1, vbuf[2]);
		len += sprintf(buf + len, "Error %d\n", -EIO);
	} else {
		u16 ver = vbuf[0] | (vbuf[1] << 8);
		len += sprintf(buf + len, "Version=0x%04x\n", ver);
	}

	/* Read status */
	ret = i2c_read_4b_checked(data, MCU_REG_STATUS, sbuf);
	if (!ret && sbuf[2] != 2) {
		len += sprintf(buf + len,
			       "Error status[2] expected %d, but got %d\n",
			       2, sbuf[2]);
		ret = -EIO;
	}
	if (ret) {
		len += sprintf(buf + len, "Error %d\n", ret);
	} else {
		u16 status = sbuf[0] | (sbuf[1] << 8);
		len += sprintf(buf + len, "Status=0x%04x\n", status);
	}

	/* Read sensor data */
	sensor_len = fan_i2c_is_extended(data) ?
		     FAN_SENSOR_SIZE_EXT : FAN_SENSOR_SIZE_STD;
	ret = i2c_read_reg_crc32(data, MCU_REG_SENSOR, sensor_buf, sensor_len);
	if (ret) {
		unsigned int data_len = sensor_len - 4;
		u32 calc_crc = ~crc32_le(~0, sensor_buf, data_len);
		u32 read_crc;

		len += sprintf(buf + len, "HWmon read Error %d\n", ret);

		memcpy(&read_crc, &sensor_buf[data_len], 4);
		if (read_crc != calc_crc)
			len += sprintf(buf + len,
				"HWmon crc32 does not match %08x != %08x\n",
				calc_crc, read_crc);
	} else {
		/* Update cached data */
		memcpy(data->fan_rpm, &sensor_buf[2], 8);
		memcpy(data->temp, &sensor_buf[10], 8);
		memcpy(data->power, &sensor_buf[18], 8);
		if (fan_i2c_is_extended(data)) {
			memcpy(data->ext_fan, &sensor_buf[28], 12);
			memcpy(data->psu_data[0], &sensor_buf[40], 12);
			memcpy(data->psu_data[1], &sensor_buf[52], 12);
		}

		len += sprintf(buf + len,
			"fan_i2c: fan %d %d %d %d %d %d %d %d %d %d %d %d \t"
			"    temp %d %d %d %d pwr %d %d %d %d \t"
			"    psu %d %d %d %d %d %d %d %d %d %d \n",
			data->fan_rpm[0], data->fan_rpm[1],
			data->fan_rpm[2], data->fan_rpm[3],
			data->ext_fan[0], data->ext_fan[1],
			data->ext_fan[2], data->ext_fan[3],
			data->ext_fan[4], data->ext_fan[5],
			(s16)data->psu_data[0][2],
			(s16)data->psu_data[1][2],
			data->temp[0], data->temp[1],
			data->temp[2], data->temp[3],
			data->power[0], data->power[1],
			data->power[2], data->power[3],
			(s16)data->psu_data[0][5],
			(s16)data->psu_data[1][5],
			(s16)data->psu_data[0][4],
			(s16)data->psu_data[1][4],
			(s16)data->psu_data[0][3],
			(s16)data->psu_data[1][3],
			data->psu_data[0][1],
			data->psu_data[1][1],
			data->psu_data[0][0],
			data->psu_data[1][0]);
	}

	return len;
}

/* sysfs: show MCU config (used via config text attr) */
static ssize_t __maybe_unused config_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	u8 cfg[FAN_CONFIG_SIZE];
	int ret, len;

	ret = i2c_read_reg_crc32(data, MCU_REG_CONFIG, cfg, sizeof(cfg));
	if (ret) {
		u32 calc_crc, read_crc;

		len = sprintf(buf, "Error reading config: %d\n", ret);

		calc_crc = ~crc32_le(~0, cfg, 124);
		memcpy(&read_crc, &cfg[124], 4);
		if (read_crc == calc_crc)
			len += sprintf(buf + len,
				"crcRx==0x%08x != crcCalc==0x%08x\n",
				read_crc, read_crc);

		len += config_format_to_string(cfg, buf + len);
		return len;
	}

	/* Update cached config */
	memcpy(data->config, cfg, sizeof(cfg));
	return config_format_to_string(cfg, buf);
}

/* sysfs: show PSU manufacturer info */
static ssize_t psu_mfr_info_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	u8 cmd1[3] = { '$', '%', '$' ^ '%' };
	u8 cmd2[3] = { '%', '%', '%' ^ '%' };
	u8 mfr_buf[128];
	u32 calc_crc, read_crc;
	int len = 0, ret, read_len, psu, field, pos;
	int i;

	poll_sensor_data(data);
	data->last_poll_jiffies = jiffies + FAN_POLL_INTERVAL;

	/* Send MFR query command */
	ret = i2c_write_reg_retry(data, MCU_REG_COMMAND, cmd1, 3);
	if (!ret) {
		printk(KERN_INFO "fan_i2c: polling smart psu MFR data..\n");
		for (i = 0; i < 500; i++)
			udelay(1000);
	}

	ret = i2c_write_reg_retry(data, MCU_REG_COMMAND, cmd2, 3);
	if (ret)
		return sprintf(buf, "error, %d\n", ret);

	/* Read MFR response (128 bytes) */
	read_len = i2c_read_reg(data, MCU_REG_CONFIG, mfr_buf, 128);

	/* Verify CRC32 */
	calc_crc = ~crc32_le(~0, mfr_buf, read_len - 4);
	memcpy(&read_crc, &mfr_buf[read_len - 4], 4);
	if (calc_crc != read_crc)
		return sprintf(buf, "%s", "error, incorrect crc32\n");

	if (read_len != 128)
		return sprintf(buf, "error, %d\n", read_len);

	/* Parse MFR data for 2 PSUs */
	pos = 0;
	for (psu = 0; psu < 2; psu++) {
		len += sprintf(buf + len, "PSU%d:\n", psu);
		for (field = 0; field < PSU_MFR_FIELD_COUNT; field++) {
			u8 field_len;

			len += sprintf(buf + len, "%s=", psu_mfr_fields[field]);
			field_len = mfr_buf[pos];
			pos++;
			if (field_len != 0xFF) {
				len += sprintf(buf + len, "%.*s",
					       field_len, &mfr_buf[pos]);
				pos += field_len;
			}
			buf[len++] = '\n';
			buf[len] = '\0';
		}
		pos = 62; /* PSU2 data starts at fixed offset */
	}

	return len;
}

/* ========================================================================
 * hwmon Attribute Declarations
 * ======================================================================== */

/*
 * We declare all possible sensor attributes and use is_visible to
 * selectively enable them based on the MCU configuration.
 */

/* Per-attribute visibility flags.  Updated during probe. */
static umode_t fan_attr_modes[128]; /* indexed by attr sequential number */

/* Attribute index counter - assigned during SENSOR_DEVICE_ATTR declarations */
enum {
	/* Fan RPM inputs (1-12) */
	ATTR_FAN1_INPUT, ATTR_FAN1_FAULT, ATTR_FAN1_LABEL,
	ATTR_FAN2_INPUT, ATTR_FAN2_FAULT, ATTR_FAN2_LABEL,
	ATTR_FAN3_INPUT, ATTR_FAN3_FAULT, ATTR_FAN3_LABEL,
	ATTR_FAN4_INPUT, ATTR_FAN4_FAULT, ATTR_FAN4_LABEL,
	ATTR_FAN5_INPUT, ATTR_FAN5_FAULT, ATTR_FAN5_LABEL,
	ATTR_FAN6_INPUT, ATTR_FAN6_FAULT, ATTR_FAN6_LABEL,
	ATTR_FAN7_INPUT, ATTR_FAN7_FAULT, ATTR_FAN7_LABEL,
	ATTR_FAN8_INPUT, ATTR_FAN8_FAULT, ATTR_FAN8_LABEL,
	ATTR_FAN9_INPUT, ATTR_FAN9_FAULT, ATTR_FAN9_LABEL,
	ATTR_FAN10_INPUT, ATTR_FAN10_FAULT, ATTR_FAN10_LABEL,

	/* Temperature inputs (1-4) */
	ATTR_TEMP1_INPUT, ATTR_TEMP1_LABEL,
	ATTR_TEMP2_INPUT, ATTR_TEMP2_LABEL,
	ATTR_TEMP3_INPUT, ATTR_TEMP3_LABEL,
	ATTR_TEMP4_INPUT, ATTR_TEMP4_LABEL,

	/* Voltage inputs (1-2) with min/max */
	ATTR_IN1_INPUT, ATTR_IN1_LABEL, ATTR_IN1_MIN, ATTR_IN1_MAX,
	ATTR_IN2_INPUT, ATTR_IN2_LABEL, ATTR_IN2_MIN, ATTR_IN2_MAX,

	/* Current inputs (1-2) */
	ATTR_CURR1_INPUT, ATTR_CURR1_LABEL,
	ATTR_CURR2_INPUT, ATTR_CURR2_LABEL,

	/* PWM */
	ATTR_PWM1, ATTR_PWM1_LABEL,

	/* PSU sensors (11-12 range) */
	ATTR_FAN11_INPUT, ATTR_FAN11_LABEL,
	ATTR_FAN12_INPUT, ATTR_FAN12_LABEL,
	ATTR_TEMP11_INPUT, ATTR_TEMP11_LABEL,
	ATTR_TEMP12_INPUT, ATTR_TEMP12_LABEL,
	ATTR_IN11_INPUT, ATTR_IN11_LABEL,
	ATTR_IN12_INPUT, ATTR_IN12_LABEL,
	ATTR_POWER11_INPUT, ATTR_POWER11_LABEL,
	ATTR_POWER12_INPUT, ATTR_POWER12_LABEL,
	ATTR_PSU11_STATE,
	ATTR_PSU12_STATE,

	/* Non-indexed device attributes */
	ATTR_FAN_STATE,
	ATTR_STATUS,
	ATTR_VERSION,
	ATTR_DETECT,
	ATTR_PSU_MFR_INFO,

	ATTR_COUNT
};

/* Fan attributes */
#define FAN_ATTRS(n, fidx, bidx)					\
static SENSOR_DEVICE_ATTR(fan##n##_input, 0444, fan_input_show,		\
			  NULL, fidx);					\
static SENSOR_DEVICE_ATTR(fan##n##_fault, 0444, fan_fault_show,		\
			  NULL, bidx);					\
static SENSOR_DEVICE_ATTR(fan##n##_label, 0444, sensor_label_show,	\
			  NULL, 0)

FAN_ATTRS(1,  1, 2);
FAN_ATTRS(2,  2, 3);
FAN_ATTRS(3,  3, 4);
FAN_ATTRS(4,  4, 5);
FAN_ATTRS(5,  5, 6);
FAN_ATTRS(6,  6, 7);
FAN_ATTRS(7,  7, 8);
FAN_ATTRS(8,  8, 9);
FAN_ATTRS(9,  9, 10);
FAN_ATTRS(10, 10, 11);

/* PSU fan attributes (11, 12) */
static SENSOR_DEVICE_ATTR(fan11_input, 0444, fan_input_show, NULL, 11);
static SENSOR_DEVICE_ATTR(fan11_label, 0444, sensor_label_show, NULL, 0);
static SENSOR_DEVICE_ATTR(fan12_input, 0444, fan_input_show, NULL, 12);
static SENSOR_DEVICE_ATTR(fan12_label, 0444, sensor_label_show, NULL, 0);

/* Temperature attributes */
#define TEMP_ATTRS(n, tidx)						\
static SENSOR_DEVICE_ATTR(temp##n##_input, 0444, temp_input_show,	\
			  NULL, tidx);					\
static SENSOR_DEVICE_ATTR(temp##n##_label, 0444, sensor_label_show,	\
			  NULL, 0)

TEMP_ATTRS(1, 1);
TEMP_ATTRS(2, 2);
TEMP_ATTRS(3, 3);
TEMP_ATTRS(4, 4);

/* PSU temperature attributes */
static SENSOR_DEVICE_ATTR(temp11_input, 0444, temp_input_show, NULL, 5);
static SENSOR_DEVICE_ATTR(temp11_label, 0444, sensor_label_show, NULL, 0);
static SENSOR_DEVICE_ATTR(temp12_input, 0444, temp_input_show, NULL, 6);
static SENSOR_DEVICE_ATTR(temp12_label, 0444, sensor_label_show, NULL, 0);

/* Voltage attributes with min/max */
static SENSOR_DEVICE_ATTR(in1_input, 0444, voltage_input_show, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_label, 0444, sensor_label_show, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_min, 0444, voltage_input_show, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_max, 0444, voltage_input_show, NULL, 0);
static SENSOR_DEVICE_ATTR(in2_input, 0444, voltage_input_show, NULL, 0);
static SENSOR_DEVICE_ATTR(in2_label, 0444, sensor_label_show, NULL, 0);
static SENSOR_DEVICE_ATTR(in2_min, 0444, voltage_input_show, NULL, 0);
static SENSOR_DEVICE_ATTR(in2_max, 0444, voltage_input_show, NULL, 0);

/* PSU voltage attributes */
static SENSOR_DEVICE_ATTR(in11_input, 0444, power_input_show, NULL, 7);
static SENSOR_DEVICE_ATTR(in11_label, 0444, sensor_label_show, NULL, 0);
static SENSOR_DEVICE_ATTR(in12_input, 0444, power_input_show, NULL, 8);
static SENSOR_DEVICE_ATTR(in12_label, 0444, sensor_label_show, NULL, 0);

/* Current attributes */
static SENSOR_DEVICE_ATTR(curr1_input, 0444, power_input_show, NULL, 5);
static SENSOR_DEVICE_ATTR(curr1_label, 0444, sensor_label_show, NULL, 0);
static SENSOR_DEVICE_ATTR(curr2_input, 0444, power_input_show, NULL, 6);
static SENSOR_DEVICE_ATTR(curr2_label, 0444, sensor_label_show, NULL, 0);

/* Power attributes */
static SENSOR_DEVICE_ATTR(power11_input, 0444, power_input_show, NULL, 9);
static SENSOR_DEVICE_ATTR(power11_label, 0444, sensor_label_show, NULL, 0);
static SENSOR_DEVICE_ATTR(power12_input, 0444, power_input_show, NULL, 10);
static SENSOR_DEVICE_ATTR(power12_label, 0444, sensor_label_show, NULL, 0);

/* PSU state */
static SENSOR_DEVICE_ATTR(psu11_state, 0444, power_input_show, NULL, 1);
static SENSOR_DEVICE_ATTR(psu12_state, 0444, power_input_show, NULL, 2);

/* PWM */
static SENSOR_DEVICE_ATTR(pwm1, 0644, NULL, pwm_store, 0);
static SENSOR_DEVICE_ATTR(pwm1_label, 0444, sensor_label_show, NULL, 0);

/* Non-indexed device attributes */
static DEVICE_ATTR_RO(fan_state);
static DEVICE_ATTR_RO(status);
static DEVICE_ATTR_RO(version);
static DEVICE_ATTR_RO(detect);
static DEVICE_ATTR_RO(psu_mfr_info);

/*
 * Master attribute list with visibility tracking.
 * Index in this array matches the ATTR_xxx enum.
 */
static struct attribute *fan_i2c_attrs[] = {
	/* Fan 1-10: input, fault, label */
	[ATTR_FAN1_INPUT]  = &sensor_dev_attr_fan1_input.dev_attr.attr,
	[ATTR_FAN1_FAULT]  = &sensor_dev_attr_fan1_fault.dev_attr.attr,
	[ATTR_FAN1_LABEL]  = &sensor_dev_attr_fan1_label.dev_attr.attr,
	[ATTR_FAN2_INPUT]  = &sensor_dev_attr_fan2_input.dev_attr.attr,
	[ATTR_FAN2_FAULT]  = &sensor_dev_attr_fan2_fault.dev_attr.attr,
	[ATTR_FAN2_LABEL]  = &sensor_dev_attr_fan2_label.dev_attr.attr,
	[ATTR_FAN3_INPUT]  = &sensor_dev_attr_fan3_input.dev_attr.attr,
	[ATTR_FAN3_FAULT]  = &sensor_dev_attr_fan3_fault.dev_attr.attr,
	[ATTR_FAN3_LABEL]  = &sensor_dev_attr_fan3_label.dev_attr.attr,
	[ATTR_FAN4_INPUT]  = &sensor_dev_attr_fan4_input.dev_attr.attr,
	[ATTR_FAN4_FAULT]  = &sensor_dev_attr_fan4_fault.dev_attr.attr,
	[ATTR_FAN4_LABEL]  = &sensor_dev_attr_fan4_label.dev_attr.attr,
	[ATTR_FAN5_INPUT]  = &sensor_dev_attr_fan5_input.dev_attr.attr,
	[ATTR_FAN5_FAULT]  = &sensor_dev_attr_fan5_fault.dev_attr.attr,
	[ATTR_FAN5_LABEL]  = &sensor_dev_attr_fan5_label.dev_attr.attr,
	[ATTR_FAN6_INPUT]  = &sensor_dev_attr_fan6_input.dev_attr.attr,
	[ATTR_FAN6_FAULT]  = &sensor_dev_attr_fan6_fault.dev_attr.attr,
	[ATTR_FAN6_LABEL]  = &sensor_dev_attr_fan6_label.dev_attr.attr,
	[ATTR_FAN7_INPUT]  = &sensor_dev_attr_fan7_input.dev_attr.attr,
	[ATTR_FAN7_FAULT]  = &sensor_dev_attr_fan7_fault.dev_attr.attr,
	[ATTR_FAN7_LABEL]  = &sensor_dev_attr_fan7_label.dev_attr.attr,
	[ATTR_FAN8_INPUT]  = &sensor_dev_attr_fan8_input.dev_attr.attr,
	[ATTR_FAN8_FAULT]  = &sensor_dev_attr_fan8_fault.dev_attr.attr,
	[ATTR_FAN8_LABEL]  = &sensor_dev_attr_fan8_label.dev_attr.attr,
	[ATTR_FAN9_INPUT]  = &sensor_dev_attr_fan9_input.dev_attr.attr,
	[ATTR_FAN9_FAULT]  = &sensor_dev_attr_fan9_fault.dev_attr.attr,
	[ATTR_FAN9_LABEL]  = &sensor_dev_attr_fan9_label.dev_attr.attr,
	[ATTR_FAN10_INPUT] = &sensor_dev_attr_fan10_input.dev_attr.attr,
	[ATTR_FAN10_FAULT] = &sensor_dev_attr_fan10_fault.dev_attr.attr,
	[ATTR_FAN10_LABEL] = &sensor_dev_attr_fan10_label.dev_attr.attr,

	/* Temperature 1-4 */
	[ATTR_TEMP1_INPUT] = &sensor_dev_attr_temp1_input.dev_attr.attr,
	[ATTR_TEMP1_LABEL] = &sensor_dev_attr_temp1_label.dev_attr.attr,
	[ATTR_TEMP2_INPUT] = &sensor_dev_attr_temp2_input.dev_attr.attr,
	[ATTR_TEMP2_LABEL] = &sensor_dev_attr_temp2_label.dev_attr.attr,
	[ATTR_TEMP3_INPUT] = &sensor_dev_attr_temp3_input.dev_attr.attr,
	[ATTR_TEMP3_LABEL] = &sensor_dev_attr_temp3_label.dev_attr.attr,
	[ATTR_TEMP4_INPUT] = &sensor_dev_attr_temp4_input.dev_attr.attr,
	[ATTR_TEMP4_LABEL] = &sensor_dev_attr_temp4_label.dev_attr.attr,

	/* Voltage 1-2 */
	[ATTR_IN1_INPUT]   = &sensor_dev_attr_in1_input.dev_attr.attr,
	[ATTR_IN1_LABEL]   = &sensor_dev_attr_in1_label.dev_attr.attr,
	[ATTR_IN1_MIN]     = &sensor_dev_attr_in1_min.dev_attr.attr,
	[ATTR_IN1_MAX]     = &sensor_dev_attr_in1_max.dev_attr.attr,
	[ATTR_IN2_INPUT]   = &sensor_dev_attr_in2_input.dev_attr.attr,
	[ATTR_IN2_LABEL]   = &sensor_dev_attr_in2_label.dev_attr.attr,
	[ATTR_IN2_MIN]     = &sensor_dev_attr_in2_min.dev_attr.attr,
	[ATTR_IN2_MAX]     = &sensor_dev_attr_in2_max.dev_attr.attr,

	/* Current 1-2 */
	[ATTR_CURR1_INPUT] = &sensor_dev_attr_curr1_input.dev_attr.attr,
	[ATTR_CURR1_LABEL] = &sensor_dev_attr_curr1_label.dev_attr.attr,
	[ATTR_CURR2_INPUT] = &sensor_dev_attr_curr2_input.dev_attr.attr,
	[ATTR_CURR2_LABEL] = &sensor_dev_attr_curr2_label.dev_attr.attr,

	/* PWM */
	[ATTR_PWM1]        = &sensor_dev_attr_pwm1.dev_attr.attr,
	[ATTR_PWM1_LABEL]  = &sensor_dev_attr_pwm1_label.dev_attr.attr,

	/* PSU sensors */
	[ATTR_FAN11_INPUT] = &sensor_dev_attr_fan11_input.dev_attr.attr,
	[ATTR_FAN11_LABEL] = &sensor_dev_attr_fan11_label.dev_attr.attr,
	[ATTR_FAN12_INPUT] = &sensor_dev_attr_fan12_input.dev_attr.attr,
	[ATTR_FAN12_LABEL] = &sensor_dev_attr_fan12_label.dev_attr.attr,
	[ATTR_TEMP11_INPUT]= &sensor_dev_attr_temp11_input.dev_attr.attr,
	[ATTR_TEMP11_LABEL]= &sensor_dev_attr_temp11_label.dev_attr.attr,
	[ATTR_TEMP12_INPUT]= &sensor_dev_attr_temp12_input.dev_attr.attr,
	[ATTR_TEMP12_LABEL]= &sensor_dev_attr_temp12_label.dev_attr.attr,
	[ATTR_IN11_INPUT]  = &sensor_dev_attr_in11_input.dev_attr.attr,
	[ATTR_IN11_LABEL]  = &sensor_dev_attr_in11_label.dev_attr.attr,
	[ATTR_IN12_INPUT]  = &sensor_dev_attr_in12_input.dev_attr.attr,
	[ATTR_IN12_LABEL]  = &sensor_dev_attr_in12_label.dev_attr.attr,
	[ATTR_POWER11_INPUT]= &sensor_dev_attr_power11_input.dev_attr.attr,
	[ATTR_POWER11_LABEL]= &sensor_dev_attr_power11_label.dev_attr.attr,
	[ATTR_POWER12_INPUT]= &sensor_dev_attr_power12_input.dev_attr.attr,
	[ATTR_POWER12_LABEL]= &sensor_dev_attr_power12_label.dev_attr.attr,
	[ATTR_PSU11_STATE] = &sensor_dev_attr_psu11_state.dev_attr.attr,
	[ATTR_PSU12_STATE] = &sensor_dev_attr_psu12_state.dev_attr.attr,

	/* Non-indexed */
	[ATTR_FAN_STATE]   = &dev_attr_fan_state.attr,
	[ATTR_STATUS]      = &dev_attr_status.attr,
	[ATTR_VERSION]     = &dev_attr_version.attr,
	[ATTR_DETECT]      = &dev_attr_detect.attr,
	[ATTR_PSU_MFR_INFO]= &dev_attr_psu_mfr_info.attr,

	NULL,
};

static umode_t fan_i2c_is_visible(struct kobject *kobj,
				  struct attribute *attr, int index)
{
	if (index < ATTR_COUNT)
		return fan_attr_modes[index];
	return 0;
}

static const struct attribute_group fan_i2c_group = {
	.attrs = fan_i2c_attrs,
	.is_visible = fan_i2c_is_visible,
};

static const struct attribute_group *fan_i2c_groups[] = {
	&fan_i2c_group,
	NULL,
};

/*
 * Helper: enable a fan attribute triple (input, fault, label).
 */
static void enable_fan_attrs(int fan_idx, int label_idx)
{
	int base = ATTR_FAN1_INPUT + (fan_idx * 3);

	if (fan_idx < 0 || fan_idx >= 10)
		return;

	fan_attr_modes[base + 0] = 0444; /* input */
	fan_attr_modes[base + 1] = 0444; /* fault */

	if (label_idx > 0) {
		fan_attr_modes[base + 2] = 0444; /* label */
		to_sensor_dev_attr(
			container_of(fan_i2c_attrs[base + 2],
				     struct device_attribute, attr)
		)->index = label_idx;
	}
}

/*
 * Helper: enable a temperature attribute pair (input, label).
 */
static void enable_temp_attrs(int temp_idx, int label_idx)
{
	int base = ATTR_TEMP1_INPUT + (temp_idx * 2);

	if (temp_idx < 0 || temp_idx >= 4)
		return;

	fan_attr_modes[base + 0] = 0444;
	if (label_idx > 0) {
		fan_attr_modes[base + 1] = 0444;
		to_sensor_dev_attr(
			container_of(fan_i2c_attrs[base + 1],
				     struct device_attribute, attr)
		)->index = label_idx;
	}
}

/*
 * Helper: enable a voltage attribute set (input, label, min, max).
 */
static void enable_voltage_attrs(int volt_idx, int label_idx,
				 s16 min_val, s16 max_val)
{
	int base = ATTR_IN1_INPUT + (volt_idx * 4);

	if (volt_idx < 0 || volt_idx >= 2)
		return;

	fan_attr_modes[base + 0] = 0444; /* input */
	if (label_idx > 0) {
		fan_attr_modes[base + 1] = 0444; /* label */
		to_sensor_dev_attr(
			container_of(fan_i2c_attrs[base + 1],
				     struct device_attribute, attr)
		)->index = label_idx;
	}

	/* Set min/max thresholds */
	if (min_val) {
		to_sensor_dev_attr(
			container_of(fan_i2c_attrs[base + 2],
				     struct device_attribute, attr)
		)->index = min_val;
		if (!max_val || min_val >= max_val)
			fan_attr_modes[base + 2] = 0444;
		else
			fan_attr_modes[base + 2] = 0444;
	}
	if (max_val) {
		to_sensor_dev_attr(
			container_of(fan_i2c_attrs[base + 3],
				     struct device_attribute, attr)
		)->index = max_val;
		fan_attr_modes[base + 3] = 0444;
	}
}

/*
 * Helper: enable a current attribute pair (input, label).
 */
static void enable_current_attrs(int curr_idx, int label_idx)
{
	int base = ATTR_CURR1_INPUT + (curr_idx * 2);

	if (curr_idx < 0 || curr_idx >= 2)
		return;

	fan_attr_modes[base + 0] = 0444;
	if (label_idx > 0) {
		fan_attr_modes[base + 1] = 0444;
		to_sensor_dev_attr(
			container_of(fan_i2c_attrs[base + 1],
				     struct device_attribute, attr)
		)->index = label_idx;
	}
}

/*
 * Enable all PSU-related attributes.
 */
static void enable_psu_attrs(void)
{
	int i;
	int psu_indices[] = {
		ATTR_FAN11_INPUT, ATTR_FAN11_LABEL,
		ATTR_FAN12_INPUT, ATTR_FAN12_LABEL,
		ATTR_TEMP11_INPUT, ATTR_TEMP11_LABEL,
		ATTR_TEMP12_INPUT, ATTR_TEMP12_LABEL,
		ATTR_IN11_INPUT, ATTR_IN11_LABEL,
		ATTR_IN12_INPUT, ATTR_IN12_LABEL,
		ATTR_POWER11_INPUT, ATTR_POWER11_LABEL,
		ATTR_POWER12_INPUT, ATTR_POWER12_LABEL,
		ATTR_PSU11_STATE, ATTR_PSU12_STATE,
	};

	for (i = 0; i < ARRAY_SIZE(psu_indices); i++)
		fan_attr_modes[psu_indices[i]] = 0444;

	/* Set all PSU labels to "smart-psu" */
	to_sensor_dev_attr(
		container_of(fan_i2c_attrs[ATTR_FAN11_LABEL],
			     struct device_attribute, attr)
	)->index = LABEL_SMART_PSU;
	to_sensor_dev_attr(
		container_of(fan_i2c_attrs[ATTR_FAN12_LABEL],
			     struct device_attribute, attr)
	)->index = LABEL_SMART_PSU;
	to_sensor_dev_attr(
		container_of(fan_i2c_attrs[ATTR_TEMP11_LABEL],
			     struct device_attribute, attr)
	)->index = LABEL_SMART_PSU;
	to_sensor_dev_attr(
		container_of(fan_i2c_attrs[ATTR_TEMP12_LABEL],
			     struct device_attribute, attr)
	)->index = LABEL_SMART_PSU;
	to_sensor_dev_attr(
		container_of(fan_i2c_attrs[ATTR_IN11_LABEL],
			     struct device_attribute, attr)
	)->index = LABEL_SMART_PSU;
	to_sensor_dev_attr(
		container_of(fan_i2c_attrs[ATTR_IN12_LABEL],
			     struct device_attribute, attr)
	)->index = LABEL_SMART_PSU;
	to_sensor_dev_attr(
		container_of(fan_i2c_attrs[ATTR_POWER11_LABEL],
			     struct device_attribute, attr)
	)->index = LABEL_SMART_PSU;
	to_sensor_dev_attr(
		container_of(fan_i2c_attrs[ATTR_POWER12_LABEL],
			     struct device_attribute, attr)
	)->index = LABEL_SMART_PSU;
}

/* ========================================================================
 * sysfs Binary Attributes (upgrade, config, scratch)
 * ======================================================================== */

/* Binary write: prefix data with register byte (offset) */
static ssize_t __maybe_unused sysfs_bin_write(struct file *f, struct kobject *kobj,
			       struct bin_attribute *attr, char *buf,
			       loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	u8 txbuf[257]; /* 1 register byte + up to 256 data bytes */
	int ret;

	if (count > sizeof(txbuf) - 1)
		return -EINVAL;

	txbuf[0] = (u8)off;
	memcpy(&txbuf[1], buf, count);

	ret = i2c_transfer_buffer_flags(data->client, txbuf, count + 1, 0);
	if (ret == (int)(count + 1))
		return count;
	if (!ret)
		return -EIO;
	return ret;
}

/* Binary read: I2C register read */
static ssize_t sysfs_bin_read(struct file *f, struct kobject *kobj,
			      struct bin_attribute *attr, char *buf,
			      loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	int ret;

	if (off > 255)
		ret = i2c_transfer_buffer_flags(data->client, buf, count, 1);
	else
		ret = i2c_read_reg(data, (u8)off, buf, count);

	if (ret == (int)count)
		return count;
	if (ret >= 0)
		return -EIO;
	return ret;
}

/* Scratch write: raw I2C send */
static ssize_t scratch_write(struct file *f, struct kobject *kobj,
			     struct bin_attribute *attr, char *buf,
			     loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	int ret;

	ret = i2c_master_send_checked(data, buf, count);
	if (ret)
		return ret;
	return count;
}

/* Eeprom read: at offset 0, send address then read back */
static ssize_t eeprom_read(struct file *f, struct kobject *kobj,
			   struct bin_attribute *attr, char *buf,
			   loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	int ret;

	if (off != 0)
		return count;

	/* Send 1 byte (address 0), then read 8 bytes */
	ret = i2c_master_send_checked(data, "\0", 1);
	if (ret)
		return ret;

	ret = i2c_transfer_buffer_flags(data->client, buf, 8, I2C_M_RD);
	if (ret != 8) {
		if (ret >= 0)
			return -EIO;
		return ret;
	}
	return count;
}

/* ========================================================================
 * Firmware Upgrade
 * ======================================================================== */

/*
 * fw_write_chunk_64b - Write a single 64-byte firmware chunk.
 * Sends 72 bytes (64 data + 8 header) to register 0xB8.
 * Header: [xor_data, xor_hdr, offset_lo, offset_hi, 0, 0, 0, 0x10]
 */
static int fw_write_chunk_64b(struct fan_i2c_data *data,
			      const u8 *chunk, u16 offset)
{
	u8 pkt[72]; /* 64 data + 8 header */
	u8 xor_data = 0, xor_hdr = 0;
	u8 resp[8];
	int i;

	memcpy(pkt, chunk, 64);

	/* Compute data XOR checksum */
	for (i = 0; i < 64; i++)
		xor_data ^= pkt[i];

	/* Build header at pkt[64..71] */
	pkt[64] = 0; /* placeholder for header XOR, computed below */
	pkt[65] = xor_data;
	pkt[66] = offset & 0xFF;
	pkt[67] = (offset >> 8) & 0xFF;
	pkt[68] = 0;
	pkt[69] = 0;
	pkt[70] = 0;
	pkt[71] = 0x10;

	/* Compute header XOR (XOR of bytes 64-71 with byte 64 as 0) */
	for (i = 64; i < 72; i++)
		xor_hdr ^= pkt[i];
	pkt[64] = xor_hdr;

	if (i2c_write_reg(data, MCU_REG_FW_WRITE, pkt, 72) != 72)
		return -EIO;

	mcu_wait_response(data, resp, 5);
	if (resp[0] != 5)
		return -EIO;

	return 0;
}

/*
 * fw_write_data - Write firmware data in 64-byte chunks.
 */
static int fw_write_data(struct fan_i2c_data *data,
			 const u8 *buf, unsigned int len, u16 start_offset)
{
	unsigned int full_chunks = len >> 6;
	unsigned int remainder = len & 0x3F;
	unsigned int i;
	int ret;

	for (i = 0; i < full_chunks; i++) {
		ret = fw_write_chunk_64b(data, buf + (i << 6),
					 start_offset + (i << 6));
		if (ret)
			return ret;
	}

	if (remainder) {
		u8 pad_chunk[64];

		memset(pad_chunk, 0xFF, sizeof(pad_chunk));
		memcpy(pad_chunk, buf + (i << 6), remainder);
		ret = fw_write_chunk_64b(data, pad_chunk,
					 start_offset + (i << 6));
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * sysfs_fw_upgrade_write - Firmware upgrade binary write handler.
 */
static ssize_t upgrade_write(struct file *f, struct kobject *kobj,
			     struct bin_attribute *attr, char *buf,
			     loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct fan_i2c_data *data = fan_data_from_dev(dev);
	int ret, fw_ver;
	ssize_t result = count;

	printk(KERN_INFO "fan_i2c: info: offset==0x%04x, len==%zu\n",
	       (unsigned int)off, count);

	/* First write (offset 0): validate firmware file */
	if (off == 0) {
		u32 header_crc, calc_crc;
		u32 fw_data_size, fw_version;

		printk(KERN_INFO "fan_i2c: info: validate fw upgrade file\n");
		g_fw_total_size = 0;

		if (count < 64) {
			printk(KERN_ERR "fan_i2c: error: invalid fw file\n");
			return -EINVAL;
		}

		/* Verify header CRC32 (first 60 bytes) */
		calc_crc = ~crc32_le(~0, buf, 60);
		memcpy(&header_crc, &buf[60], 4);
		if (calc_crc != header_crc) {
			printk(KERN_ERR "fan_i2c: error: invalid fw file\n");
			return -EINVAL;
		}

		/* Extract firmware metadata */
		memcpy(&fw_data_size, &buf[4], 4);
		memcpy(&fw_version, &buf[12], 4);

		g_fw_total_size = fw_data_size + FAN_FW_HEADER_SIZE;
		g_fw_target_ver = fw_version;

		printk(KERN_INFO "fan_i2c: info: upgrade fw ver 0x%04x size %d\n",
		       g_fw_target_ver, g_fw_total_size);

		if (g_fw_total_size > FAN_FW_MAX_SIZE) {
			printk(KERN_ERR "fan_i2c: error: invalid fw file\n");
			return -EFBIG;
		}

		/* Read current firmware version */
		g_current_fw_ver = read_fw_version(data);
		if (g_current_fw_ver < 0 ||
		    (g_current_fw_ver & FAN_BL_VERSION_BIT)) {
			/* Try reset first */
			mcu_reset(data);
			g_current_fw_ver = read_fw_version(data);
			if (g_current_fw_ver < 0) {
				/* Try bootloader activation */
				mcu_activate_bootloader(data);
				g_current_fw_ver = read_fw_version(data);
				if (g_current_fw_ver < 0) {
					printk(KERN_ERR "fan_i2c: error: reading BL version\n");
					printk(KERN_ERR "fan_i2c: error: controller not responding\n");
					return -ENODEV;
				}
				printk(KERN_INFO "fan_i2c: BL Version=0x%04x\n",
				       g_current_fw_ver);
			}
		}

		data->fw_version = g_current_fw_ver;
		printk(KERN_INFO "fan_i2c: info: current fw ver 0x%04x\n",
		       g_current_fw_ver);
	}

	/* Check if upgrade is needed */
	if (!(g_current_fw_ver & FAN_BL_VERSION_BIT)) {
		if (((g_current_fw_ver ^ g_fw_target_ver) & 0x7F00) == 0 &&
		    g_current_fw_ver >= g_fw_target_ver) {
			printk(KERN_INFO "fan_i2c: info: current fw ver 0x%04x, "
			       "upgrade not needed\n", g_current_fw_ver);
			return count;
		}

		if (off == 0) {
			/* Poll and delay before entering bootloader */
			poll_sensor_data(data);
			data->last_poll_jiffies = jiffies + FAN_POLL_INTERVAL;
		}
	} else if (off == 0) {
		printk(KERN_WARNING "fan_i2c: warning: controller already in bl\n");
		poll_sensor_data(data);
		data->last_poll_jiffies = jiffies + FAN_POLL_INTERVAL;
	}

	/* Brief delay before BL operations */
	{
		int i;
		mcu_activate_bootloader(data);
		for (i = 0; i < 10; i++)
			udelay(1000);
	}

	if (off == 0) {
		u8 erase_cmd[8];
		u8 resp[8];
		u8 xor = 0;
		int i;

		/* Verify bootloader is active */
		fw_ver = read_fw_version(data);
		if (fw_ver < 0) {
			printk(KERN_ERR "fan_i2c: error: bl not responding, "
			       "can't upgrade fw %d\n", fw_ver);
			goto out_reset;
		}
		if (!(fw_ver & FAN_BL_VERSION_BIT)) {
			printk(KERN_ERR "fan_i2c: error: failed to activate bl, "
			       "ver 0x%04x\n", fw_ver);
			ret = -EIO;
			goto out_reset;
		}

		printk(KERN_INFO "fan_i2c: info: bl ver 0x%04x\n", fw_ver);
		printk(KERN_INFO "fan_i2c: info: fw upgrade started\n");

		/* Erase flash */
		printk(KERN_INFO "fan_i2c: erase fw\n");
		memset(erase_cmd, 0, sizeof(erase_cmd));
		erase_cmd[7] = MCU_CMD_FW_ERASE;

		/* Compute XOR checksum */
		for (i = 1; i < 8; i++)
			xor ^= erase_cmd[i];
		erase_cmd[0] = xor;

		if (i2c_write_reg(data, MCU_REG_FW_ERASE, erase_cmd, 8) != 8)
			goto fw_write_fail;

		/* Wait for erase: first ack (1), then done (5) */
		mcu_wait_response(data, resp, 1);
		for (i = 0; i < 10; i++)
			mcu_wait_response(data, resp, 5);

		if (resp[0] != 5)
			goto fw_write_fail;

		/* Save header to global buffer for writing last */
		memcpy(g_fw_header, buf, FAN_FW_HEADER_SIZE);

		printk(KERN_INFO "fan_i2c: info: fw write @ 0x%04x\n",
		       FAN_FW_HEADER_SIZE);
	} else {
		printk(KERN_INFO "fan_i2c: info: fw write @ 0x%04x\n",
		       (unsigned int)off);
	}

	/* Write firmware data (skip header at offset 0) */
	if (off == 0) {
		ret = fw_write_data(data, buf + FAN_FW_HEADER_SIZE,
				    count - FAN_FW_HEADER_SIZE,
				    FAN_FW_HEADER_SIZE);
	} else {
		ret = fw_write_data(data, buf, count, off);
	}

	if (ret) {
		printk(KERN_ERR "fan_i2c: error: flash write failed\n");
		mcu_reset(data);
		goto out;
	}

	/* Check if all data has been written */
	if (off + count >= g_fw_total_size) {
		u8 status_buf[8];

		/* Write firmware header last (from saved copy) */
		printk(KERN_INFO "fan_i2c: write fw header\n");
		ret = fw_write_data(data, g_fw_header, FAN_FW_HEADER_SIZE, 0);
		if (ret) {
			printk(KERN_ERR "fan_i2c: error: write fw header failed\n");
			mcu_reset(data);
			goto out;
		}

		/* Reset MCU and verify new firmware */
		printk(KERN_WARNING "fan_i2c: warning: failed to start new fw, try reset\n");
		mcu_reset(data);
		{
			int i;
			for (i = 0; i < 100; i++)
				udelay(1000);
		}

		ret = i2c_read_4b_checked(data, MCU_REG_STATUS, status_buf);
		if (!ret && status_buf[2] == 2) {
			if (status_buf[0] & 1)
				printk(KERN_WARNING "fan_i2c: warning: failed to start "
				       "new fw, blst==0x%04x\n",
				       status_buf[0] | (status_buf[1] << 8));

			fw_ver = read_fw_version(data);
			g_current_fw_ver = fw_ver;
			if (fw_ver < 0) {
				printk(KERN_ERR "fan_i2c: error: fw not responding, "
				       "upgrade failed\n");
				goto out_reset;
			}

			if (fw_ver == (int)g_fw_target_ver) {
				data->fw_version = fw_ver;
				printk(KERN_INFO "fan_i2c: info: fw ver 0x%04x, "
				       "upgrade ok\n", fw_ver);
				data->last_poll_jiffies = jiffies - FAN_POLL_INTERVAL - 1;
				mod_delayed_work(data->wq, &data->dwork, 0);
				return count;
			}

			printk(KERN_ERR "fan_i2c: error: fw ver must be 0x%04x, "
			       "but it is 0x%04x\n", g_fw_target_ver, fw_ver);
			ret = -EIO;
			goto out_reset;
		}

		if (!ret)
			ret = -EIO;
		printk(KERN_WARNING "fan_i2c: warning: failed to start new fw, "
		       "rc==%d\n", ret);
		mcu_reset(data);
	}

out:
	printk(KERN_INFO "exit out ret==%d\n", ret);
	if (ret)
		return ret;
	return result;

fw_write_fail:
	ret = -EIO;
	printk(KERN_ERR "fan_i2c: error: flash write failed\n");
	mcu_reset(data);
	goto out;

out_reset:
	mcu_reset(data);
	printk(KERN_INFO "exit out_reset ret==%d\n", ret);
	data->last_poll_jiffies = jiffies - FAN_POLL_INTERVAL - 1;
	mod_delayed_work(data->wq, &data->dwork, 0);
	return ret;
}

/* Binary attribute definitions */
static BIN_ATTR(upgrade, 0200, sysfs_bin_read, upgrade_write, FAN_FW_MAX_SIZE);
static BIN_ATTR(config, 0444, sysfs_bin_read, NULL, FAN_CONFIG_SIZE);
static BIN_ATTR(scratch, 0644, eeprom_read, scratch_write, FAN_CONFIG_SIZE);

/* ========================================================================
 * LED Trigger
 * ======================================================================== */

#ifdef CONFIG_LEDS_TRIGGERS
static int led_trigger_activate_fn(struct led_classdev *led_cdev)
{
	struct fan_i2c_data *data =
		container_of(led_cdev->trigger, struct fan_i2c_data, led_trig);
	int brightness;

	if (g_fan_fault_bits & FAN_FAULT_MASK)
		brightness = LED_FULL;
	else
		brightness = LED_OFF;

	led_set_brightness(led_cdev, brightness);
	g_fan_fault_state = (g_fan_fault_bits & FAN_FAULT_MASK) ? 1 : 0;

	queue_delayed_work_on(WORK_CPU_UNBOUND, data->wq, &data->dwork, 0);
	return 0;
}
#endif

/* ========================================================================
 * GPIO Probe
 * ======================================================================== */

static int gpio_probe(struct i2c_client *client, struct fan_i2c_data *data)
{
	struct device_node *of_node;
	struct property *dt_delayed;
	int rst_gpio, irq_gpio;
	int ret;
	u32 val;

	printk(KERN_INFO "fan_i2c: gpio probe\n");

	of_node = client->dev.of_node;
	if (!of_node) {
		/* Try platform_data fallback */
		u32 *pdata = dev_get_platdata(&client->dev);

		if (!pdata || pdata[0] == (u32)-1) {
			printk(KERN_INFO "fan_i2c: no reset pin\n");
			irq_gpio = 0;
			goto gpio_failed;
		}
		rst_gpio = pdata[0];
		irq_gpio = pdata[1];
		goto request_gpios;
	}

	dt_delayed = of_find_property(of_node, "gpio-init-delayed", NULL);

	/* Get RST GPIO */
	rst_gpio = of_get_named_gpio(of_node, "rst", 0);
	if (rst_gpio < 0) {
		irq_gpio = rst_gpio;
		printk(KERN_INFO "fan-i2c: rst pin not specified\n");
		goto check_defer;
	}

	/* RST active level */
	if (of_property_read_u32(of_node, "rst-active-level", &val) >= 0)
		data->rst_active_level = val;
	else
		data->rst_active_level = 0;

	/* RST out-only flag */
	if (of_find_property(of_node, "rst-out-only", NULL))
		data->rst_out_only = 1;

	/* Get IRQ GPIO */
	irq_gpio = of_get_named_gpio(of_node, "irq", 0);
	if (irq_gpio < 0) {
		printk(KERN_INFO "fan-i2c: irq pin not specified\n");
		goto check_defer;
	}

	/* IRQ out-only flag */
	if (of_find_property(of_node, "irq-out-only", NULL))
		data->irq_out_only = 1;

	/* Optional BOOT0 GPIO */
	if (of_property_read_u32(of_node, "boot0-active-level", &val) >= 0) {
		int boot0 = of_get_named_gpio(of_node, "boot0", 0);

		if (boot0 < 0) {
			printk(KERN_INFO "fan-i2c: boot0 pin not specified\n");
			goto check_defer;
		}
		data->boot0_gpio = boot0;
		fan_gpio_set_output(boot0, val);
	}

request_gpios:
	/* Request RST GPIO */
	if (data->rst_out_only)
		ret = gpio_request(rst_gpio, "fan-i2c-rst");
	else
		ret = gpio_request_one(rst_gpio, GPIOF_OUT_INIT_HIGH,
				       "fan-i2c-rst");
	if (ret)
		goto gpio_failed;

	/* Request IRQ GPIO */
	if (data->irq_out_only)
		ret = gpio_request(irq_gpio, "fan-i2c-irq");
	else
		ret = gpio_request_one(irq_gpio, GPIOF_OUT_INIT_HIGH,
				       "fan-i2c-irq");
	if (ret) {
		if (!data->rst_out_only)
			fan_gpio_set_input(rst_gpio);
		gpio_free(rst_gpio);
		goto gpio_failed;
	}

	/* Success */
	data->gpio_retry = 0x100; /* GPIOs ready flag */
	data->rst_gpio = rst_gpio;
	data->irq_gpio = irq_gpio;
	printk(KERN_INFO "fan_i2c: gpio_probe successful\n");
	return 0;

check_defer:
	if (irq_gpio == -EPROBE_DEFER) {
		printk(KERN_INFO "fan_i2c: gpios not available yet?!\n");
		if (!data->gpio_retry && dt_delayed) {
			data->gpio_retry = 1; /* retry flag */
			return 0;
		}
		data->gpio_retry = 0;
	}

gpio_failed:
	printk(KERN_WARNING "fan_i2c: gpio_probe unsuccessful, "
	       "driver might not work properly\n");
	if (data->gpio_retry)
		printk(KERN_INFO "fan_i2c: retrying gpio_probe after a while\n");
	return irq_gpio;
}

/* ========================================================================
 * Exported Symbols (Breadcrumb Read/Write)
 * ======================================================================== */

int fan_write_breadcrumb(int value)
{
	u8 reg;

	if (!g_fan_data) {
		printk(KERN_WARNING "fan-i2c bradcrumb write no dev\n");
		return -1;
	}

	reg = fan_i2c_is_extended(g_fan_data) ?
	      MCU_REG_BREADCRUMB_HI : MCU_REG_BREADCRUMB_LO;

	if (i2c_write_reg_retry(g_fan_data, reg, &value, 4)) {
		printk(KERN_WARNING "fan-i2c bradcrumb write failed\n");
		return -2;
	}

	return 0;
}
EXPORT_SYMBOL(fan_write_breadcrumb);

unsigned int fan_read_breadcrumb(void)
{
	u32 value = 0xFFFFFFFF;
	u8 reg;

	if (!g_fan_data) {
		printk(KERN_WARNING "fan-i2c bradcrumb read no dev\n");
		return value;
	}

	reg = fan_i2c_is_extended(g_fan_data) ?
	      MCU_REG_BREADCRUMB_HI : MCU_REG_BREADCRUMB_LO;

	if (i2c_read_4b_checked(g_fan_data, reg, &value)) {
		printk(KERN_WARNING "fan-i2c bradcrumb read failed\n");
		return 0xDEADBEEF;
	}

	return value;
}
EXPORT_SYMBOL(fan_read_breadcrumb);

/* ========================================================================
 * I2C Driver Probe / Remove
 * ======================================================================== */

static int fan_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct fan_i2c_data *data;
	struct device *hwmon_dev;
	const u8 *cfg;
	int ret, fw_ver;
	int fan_count = 0;
	int temp_count = 0, volt_count = 0, curr_count = 0;
	int adc_start, i;
	int last_label_idx = 0, last_pwm_ch = 0;

	printk(KERN_INFO "fan-i2c probe\n");

	/* Check I2C functionality */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* GPIO probe */
	ret = gpio_probe(client, data);
	if (ret < 0)
		goto err_free;

	data->client = client;
	i2c_set_clientdata(client, data);

	/* Initialize poll timing so first poll runs immediately */
	data->last_poll_jiffies = jiffies - FAN_POLL_INTERVAL - 1;
	data->stopped = 1;

	/* Read firmware version */
	fw_ver = read_fw_version(data);
	if (fw_ver < 0 || (fw_ver & FAN_BL_VERSION_BIT)) {
		/* FW not responding or in BL mode - try reset */
		mcu_reset(data);
		fw_ver = read_fw_version(data);
		if (fw_ver < 0) {
			printk(KERN_ERR "fan-i2c: Error reading fw version: %d, "
			       "try to activate BL\n", fw_ver);
			mcu_activate_bootloader(data);
			fw_ver = read_fw_version(data);
			if (fw_ver < 0)
				printk(KERN_ERR "fan-i2c: Error reading BL version\n");
			else
				printk(KERN_INFO "fan-i2c: BL Version=0x%04x\n", fw_ver);
		}
	}

	if (fw_ver >= 0 && !(fw_ver & FAN_BL_VERSION_BIT)) {
		data->fw_version = fw_ver;
		printk(KERN_INFO "fan-i2c: FW Version=0x%04x\n", fw_ver);
	}

	/* Read MCU config */
	ret = i2c_read_reg_crc32(data, MCU_REG_CONFIG, data->config,
				 FAN_CONFIG_SIZE);
	if (ret)
		goto err_gpio;

	data->pwm_value = -1;

	/* Initialize attribute visibility - start with everything hidden */
	memset(fan_attr_modes, 0, sizeof(fan_attr_modes));

	/* Always enable non-indexed attrs */
	fan_attr_modes[ATTR_FAN_STATE] = 0444;
	fan_attr_modes[ATTR_STATUS] = 0444;
	fan_attr_modes[ATTR_VERSION] = 0444;
	fan_attr_modes[ATTR_DETECT] = 0444;
	fan_attr_modes[ATTR_PSU_MFR_INFO] = 0444;

	/* Parse fan configuration from MCU config */
	cfg = data->config;

	if (cfg[6] > 4) {
		/* Extended mode: 10 fan entries at config+14 */
		for (i = 0; i < 10; i++) {
			u8 rpm_pwm = cfg[14 + 4 * i];

			if (!rpm_pwm)
				continue;

			last_label_idx = rpm_pwm >> 3;
			last_pwm_ch = rpm_pwm & 7;
			enable_fan_attrs(fan_count, last_label_idx);
			fan_count++;
		}
	} else {
		/* Standard mode: 4 fan entries, type field at config+14, gated by fan_map */
		for (i = 0; i < 4; i++) {
			if (!((cfg[7] >> i) & 1))
				continue;

			{
				u8 rpm_pwm = cfg[14 + 4 * i];

				if (!rpm_pwm)
					continue;

				last_label_idx = rpm_pwm >> 3;
				last_pwm_ch = rpm_pwm & 7;
				enable_fan_attrs(fan_count, last_label_idx);
				fan_count++;
			}
		}
	}

	/* Enable PSU attributes if flag bit 0 is set */
	if (cfg[8] & MCU_CFG_FLAG_PSU)
		enable_psu_attrs();

	/* Set PWM label if we have a label index */
	if (last_label_idx) {
		fan_attr_modes[ATTR_PWM1_LABEL] = 0444;
		to_sensor_dev_attr(
			container_of(fan_i2c_attrs[ATTR_PWM1_LABEL],
				     struct device_attribute, attr)
		)->index = last_label_idx;
	}

	/* Check fan count vs config */
	if (cfg[6] <= 4 && cfg[6] != fan_count) {
		printk(KERN_WARNING "fan-i2c: warning: fan config mismatch: "
		       "expected %d found %d\n", cfg[6], fan_count);

		/* Enable missing fan attrs */
		if (fan_count < cfg[6]) {
			int missing = cfg[6] - fan_count;
			int j = 0;

			while (missing > 0 && j < 4) {
				if (!((cfg[7] >> j) & 1)) {
					data->config[7] |= (1 << j);
					data->fault_mask |= (1 << (j + 2));
					enable_fan_attrs(fan_count + (cfg[6] - fan_count - missing), 0);
				}
				j++;
			}
		}
	}

	/* Enable PWM control if we have a PWM channel */
	if (last_pwm_ch) {
		fan_attr_modes[ATTR_PWM1] = 0644;
		if (last_label_idx > 0)
			fan_attr_modes[ATTR_PWM1_LABEL] = 0444;
	}

	/* Parse ADC configuration for temp/voltage/current sensors */
	adc_start = (cfg[6] > 4) ? 2 : 0;
	for (i = adc_start; i < 8; i++) {
		const u8 *adc = &cfg[28 + 12 * i];
		int label_idx = adc[1] >> 3;
		int adc_type = adc[1] & 7;
		s16 min_val, max_val;

		memcpy(&min_val, &adc[8], 2);
		memcpy(&max_val, &adc[10], 2);

		switch (adc_type) {
		case ADC_TYPE_TEMP:
			if (temp_count >= 4) {
				printk(KERN_WARNING "fan-i2c: warning: "
				       "temperature sensor count expected max %d, "
				       "found %d\n", 4, temp_count + 1);
			} else {
				enable_temp_attrs(temp_count, label_idx);
			}
			temp_count++;
			break;
		case ADC_TYPE_VOLTAGE:
			if (volt_count >= 2) {
				printk(KERN_WARNING "fan-i2c: warning: "
				       "voltage sensor count expected max %d, "
				       "found %d\n", 2, volt_count + 1);
			} else {
				enable_voltage_attrs(volt_count, label_idx,
						     min_val, max_val);
			}
			volt_count++;
			break;
		case ADC_TYPE_CURRENT:
			if (curr_count >= 2) {
				printk(KERN_WARNING "fan-i2c: warning: "
				       "current sensor count expected max %d, "
				       "found %d\n", 2, curr_count + 1);
			} else {
				enable_current_attrs(curr_count, label_idx);
			}
			curr_count++;
			break;
		}
	}

	/* Register hwmon device */
	hwmon_dev = devm_hwmon_device_register_with_groups(&client->dev,
			"fan_i2c_rb", data, fan_i2c_groups);
	if (IS_ERR(hwmon_dev)) {
		ret = PTR_ERR(hwmon_dev);
		goto err_gpio;
	}
	data->hwmon_dev = hwmon_dev;

	/* Initialize delayed work and workqueue */
	INIT_DELAYED_WORK(&data->dwork, fan_timer_callback);
	data->wq = alloc_ordered_workqueue("%s", WQ_MEM_RECLAIM,
					   "fan_i2c_monitor");
	if (!data->wq) {
		ret = -ENOMEM;
		goto err_gpio;
	}

	/* Register LED trigger */
	data->stopped = 0;
#ifdef CONFIG_LEDS_TRIGGERS
	data->led_trig.name = "fan-fault";
	data->led_trig.activate = led_trigger_activate_fn;
	ret = led_trigger_register(&data->led_trig);
	if (ret)
		goto err_wq;
#endif

	/* Create sysfs binary files */
	ret = sysfs_create_bin_file(&data->hwmon_dev->kobj, &bin_attr_upgrade);
	if (ret)
		goto err_led;

	ret = sysfs_create_bin_file(&data->hwmon_dev->kobj, &bin_attr_config);
	if (ret)
		goto err_sysfs_upgrade;

	ret = sysfs_create_bin_file(&data->hwmon_dev->kobj, &bin_attr_scratch);
	if (ret)
		goto err_sysfs_config;

	/* Start polling */
	queue_delayed_work_on(WORK_CPU_UNBOUND, data->wq, &data->dwork,
			      FAN_POLL_INTERVAL);
	printk(KERN_INFO "fan-i2c registered\n");
	g_fan_data = data;

	return 0;

err_sysfs_config:
	sysfs_remove_bin_file(&data->hwmon_dev->kobj, &bin_attr_config);
err_sysfs_upgrade:
	sysfs_remove_bin_file(&data->hwmon_dev->kobj, &bin_attr_upgrade);
err_led:
#ifdef CONFIG_LEDS_TRIGGERS
	led_trigger_unregister(&data->led_trig);
#endif
	data->stopped = 1;
	cancel_delayed_work_sync(&data->dwork);
	flush_workqueue(data->wq);
	destroy_workqueue(data->wq);
err_gpio:
	if (!data->irq_out_only)
		fan_gpio_set_input(data->irq_gpio);
	gpio_free(data->irq_gpio);
	if (!data->rst_out_only)
		fan_gpio_set_input(data->rst_gpio);
	gpio_free(data->rst_gpio);
err_free:
	kfree(data);
	printk(KERN_ERR "fan-i2c probe failed %d\n", ret);
	return ret;
}

static int fan_i2c_remove(struct i2c_client *client)
{
	struct fan_i2c_data *data = i2c_get_clientdata(client);

	printk(KERN_INFO "fan-i2c unregistering...\n");

	data->stopped = 1;
	cancel_delayed_work_sync(&data->dwork);
	flush_workqueue(data->wq);
	destroy_workqueue(data->wq);

	sysfs_remove_bin_file(&data->hwmon_dev->kobj, &bin_attr_scratch);
	sysfs_remove_bin_file(&data->hwmon_dev->kobj, &bin_attr_config);
	g_fan_data = NULL;
	sysfs_remove_bin_file(&data->hwmon_dev->kobj, &bin_attr_upgrade);

#ifdef CONFIG_LEDS_TRIGGERS
	led_trigger_unregister(&data->led_trig);
#endif

	if (!data->rst_out_only)
		fan_gpio_set_input(data->rst_gpio);
	gpio_free(data->rst_gpio);

	if (!data->irq_out_only)
		fan_gpio_set_input(data->irq_gpio);
	gpio_free(data->irq_gpio);

	kfree(data);
	printk(KERN_INFO "fan-i2c unregistered\n");
	return 0;
}

/* ========================================================================
 * Module Registration
 * ======================================================================== */

static const struct i2c_device_id fan_i2c_id[] = {
	{ "fan_i2c_rb", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fan_i2c_id);

static const struct of_device_id fan_i2c_of_match[] = {
	{ .compatible = "rb,fan_i2c_rb" },
	{ }
};
MODULE_DEVICE_TABLE(of, fan_i2c_of_match);

static struct i2c_driver fan_i2c_driver = {
	.driver = {
		.name = "fan_i2c_rb",
		.of_match_table = fan_i2c_of_match,
	},
	.probe = fan_i2c_probe,
	.remove = fan_i2c_remove,
	.id_table = fan_i2c_id,
};

module_i2c_driver(fan_i2c_driver);

MODULE_DESCRIPTION("MikroTik CCR2004-1G-2XS-PCIe fan controller MCU driver");
MODULE_LICENSE("GPL");
