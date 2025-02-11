/*
 * Copyright (c) 2025 Telraam Rearwindow BV
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_lp5810

/**
 * @file
 * @brief LP5810 LED driver
 *
 * The LP5810 is a 4-channel RGBW LED driver with
 * autonomous animation engine control. The device has
 * ultra-low normal operation current with 0.4mA (typical)
 * when illuminate LEDs.
 * Both analog dimming and PWM dimming methods are
 * adopted to achieve powerful dimming performance.
 * The output current of each LED can be adjusted
 * with 256 steps from 0.1mA to 25.5mA or 0.2mA to
 * 51mA. The 8-bit PWM generator enables smooth
 * and audible-noise-free dimming control for LED
 * brightness.
 * The autonomous animation engine can significantly
 * reduce the real-time loading of controller. Each LED
 * can be configured through the related registers to
 * realize vivid and fancy lighting effects. The device
 * can generate 6MHz clock signal and use it for
 * synchronizing the lighting effects among multiple
 * devices.
 * NOTE: in this driver we will only use the autonomous
 * animation engine for blinking the LEDs.
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#define LOG_LEVEL CONFIG_LED_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lp5810);

#include "led_context.h"
/* LP5810 max supported LED id */
#define LP5810_MAX_LED_ID 3
/* Register Addresses */

#define LP5810_ENABLE              0x000
#define LP5810_CONFIG_0            0x001
#define LP5810_CONFIG_1            0x002
#define LP5810_CONFIG_2            0x003
#define LP5810_CONFIG_3            0x004
#define LP5810_CONFIG_4            0x005
#define LP5810_CONFIG_5            0x006
#define LP5810_CONFIG_6            0x007
#define LP5810_CONFIG_7            0x008
#define LP5810_CONFIG_8            0x009
#define LP5810_CONFIG_9            0x00A
#define LP5810_CONFIG_10           0x00B
#define LP5810_CONFIG_11           0x00C
#define LP5810_CONFIG_12           0x00D
#define LP5810_CMD_UPDATE          0x010
#define LP5810_CMD_START           0x011
#define LP5810_CMD_STOP            0x012
#define LP5810_CMD_PAUSE           0x013
#define LP5810_CMD_CONTINUE        0x014
#define LP5810_LED_EN              0x020
#define LP5810_FAULT_CLEAR         0x022
#define LP5810_RESET               0x023
#define LP5810_MANUAL_DC_0         0x030
#define LP5810_MANUAL_DC_1         0x031
#define LP5810_MANUAL_DC_2         0x032
#define LP5810_MANUAL_DC_3         0x033
#define LP5810_MANUAL_PWM_0        0x040
#define LP5810_MANUAL_PWM_1        0x041
#define LP5810_MANUAL_PWM_2        0x042
#define LP5810_MANUAL_PWM_3        0x043
#define LP5810_AUTO_DC_0	       0x050
#define LP5810_AUTO_DC_1	       0x051
#define LP5810_AUTO_DC_2	       0x052
#define LP5810_AUTO_DC_3	       0x053
/*Flag Registers*/
#define LP5810_TSD_CONFIG_STATUS   0x300
#define LP5810_LOD_STATUS_0        0x301
#define LP5810_LOD_STATUS_1        0x302
#define LP5810_LSD_STATUS_0        0x303
#define LP5810_LSD_STATUS_1        0x304
#define LP5810_AUTO_PWM_0          0x305
#define LP5810_AUTO_PWM_1          0x306
#define LP5810_AUTO_PWM_2          0x307
#define LP5810_AUTO_PWM_3          0x308
#define LP5810_AEP_STATUS_0        0x315
#define LP5810_AEP_STATUS_1        0x316

/* Autonomous Animation Engine registers are being copied for each LED, so I will define their base address first and use
 * references to them */
#define LP5810_AUTO_ANIM_LED0_BASE 0x080
#define LP5810_AUTO_ANIM_LED1_BASE 0x09A
#define LP5810_AUTO_ANIM_LED2_BASE 0x0B4
#define LP5810_AUTO_ANIM_LED3_BASE 0x0CE

/* Autonomous Animation Engine reference registers*/
#define LP5810_LEDX_AUTO_PAUSE     0x000
#define LP5810_LEDX_AUTO_PLAYBACK  0x001
/* Autonomous Animation Engine uses 3 AEU adding their reference here (relative to LP5810_AUTO_ANIM_LEDx_BASE)*/
#define LP5810_LEDX_AEU1_BASE      0x002
#define LP5810_LEDX_AEU2_BASE      0x00A
#define LP5810_LEDX_AEU3_BASE      0x012

/* AEU registers, relative to LP5810_LEDX_AEUy_BASE, which is relative to LP5810_AUTO_ANIM_LEDx_BASE*/
#define LP5810_AEU_PWM_1           0x000
#define LP5810_AEU_PWM_2           0x001
#define LP5810_AEU_PWM_3           0x002
#define LP5810_AEU_PWM_4           0x003
#define LP5810_AEU_PWM_5           0x004
#define LP5810_AEU_T12             0x005
#define LP5810_AEU_T34             0x006
#define LP5810_AEU_PLAYBACK        0x007


/* Values for ENABLE register. */
#define LP5810_ENABLE_CHIP_EN (1 << 0)

/* Values for Config3 register. */
#define LP5810_ENABLE_AUTO_0  (1 << 0)
#define LP5810_ENABLE_AUTO_1  (1 << 1)
#define LP5810_ENABLE_AUTO_2  (1 << 2)
#define LP5810_ENABLE_AUTO_3  (1 << 3)

/* Value for Cmd Update*/
#define LP5810_CMD_UPDATE_VALUE  0x55
/* Value for Cmd Start*/
#define LP5810_CMD_START_VALUE   0xFF
/* Value for Cmd Stop*/
#define LP5810_CMD_STOP_VALUE    0xAA
/* Value for Cmd Pause*/
#define LP5810_CMD_PAUSE_VALUE   0x33

/* Values for Led enable*/
#define LP5810_LED_ENABLE_0  (1 << 0)
#define LP5810_LED_ENABLE_1  (1 << 1)
#define LP5810_LED_ENABLE_2  (1 << 2)
#define LP5810_LED_ENABLE_3  (1 << 3)

/* Values for LEDx Auto playback */
#define LP5810_LEDX_AEU_USE_SHIFT  4
#define LP5810_LEDX_AEU_USE_1      0x00
#define LP5810_LEDX_AEU_USE_1_2    0x01
#define LP5810_LEDX_AEU_USE_1_2_3  0x02
#define LP5810_LEDX_PT_SHIFT       0
#define LP5810_LEDX_PT_INFINITE    0x0F

/* shift values for AEU time T1 and T2*/
#define LP5810_AEU_T12_T1_SHIFT    0
#define LP5810_AEU_T12_T2_SHIFT    4
/* shift values for AEU time T3 and T4*/
#define LP5810_AEU_T34_T3_SHIFT    0
#define LP5810_AEU_T34_T4_SHIFT    4

/* values for aeu playback*/
#define LP5810_AEU_NO_PLAYBACK     0
#define LP5810_AEU_PLAYBACK_1NCE   1
#define LP5810_AEU_PLAYBACK_2CE    2
#define LP5810_AEU_INFINITE_PB     3

#define LP5810_MAXNR_PROGRAMMABLE_TIME_OPTIONS 16
static uint16_t lp5810_programmable_time_options[LP5810_MAXNR_PROGRAMMABLE_TIME_OPTIONS] =
    {0, 90, 180, 360, 540, 800, 1070, 1520, 2060, 2500, 3040, 4020, 5010, 5990, 7060, 8050};

static uint8_t  lp5810_enable_auto_mask[4]    = {LP5810_ENABLE_AUTO_0, LP5810_ENABLE_AUTO_1, LP5810_ENABLE_AUTO_2, LP5810_ENABLE_AUTO_3};
static uint8_t  lp5810_led_enable_mask[4]     = {LP5810_LED_ENABLE_0, LP5810_LED_ENABLE_1, LP5810_LED_ENABLE_2, LP5810_LED_ENABLE_3};
static uint16_t lp5810_manual_pwm_register[4] = {LP5810_MANUAL_PWM_0, LP5810_MANUAL_PWM_1, LP5810_MANUAL_PWM_2, LP5810_MANUAL_PWM_3};
static uint16_t lp5810_led_base_register[4]   = {LP5810_AUTO_ANIM_LED0_BASE, LP5810_AUTO_ANIM_LED1_BASE, LP5810_AUTO_ANIM_LED2_BASE, LP5810_AUTO_ANIM_LED3_BASE};

struct lp5810_config {
	struct i2c_dt_spec i2c;
};

struct lp5810_data {
	struct led_data dev_data;
};

static uint8_t get_programmable_time_option(uint16_t time_ms)
{
    uint8_t i;
    for (i = 0; i < LP5810_MAXNR_PROGRAMMABLE_TIME_OPTIONS; i++) {
        if (time_ms <= lp5810_programmable_time_options[i]) {
			//check if you want to round up or down
        	if (i>0 && (time_ms - lp5810_programmable_time_options[i-1]) < (lp5810_programmable_time_options[i] - time_ms))
        	{
        		return i-1;
        	}
        	return i;
        }
    }
    return LP5810_MAXNR_PROGRAMMABLE_TIME_OPTIONS - 1;
}

static int update_register(const struct device *dev, uint16_t reg, uint8_t val)
{
	const struct lp5810_config *config = dev->config;
	uint8_t reg_major = reg >> 8;
	uint8_t reg_minor = reg & 0xFF;
	if(reg_major>0x03) {
		LOG_ERR("Invalid register address");
		return -EINVAL;
	}
	uint16_t i2c_addr = config->i2c.addr + reg_major;
	return i2c_reg_write_byte(config->i2c.bus, i2c_addr, reg_minor, val);
}

static int read_register(const struct device *dev, uint16_t reg, uint8_t *val)
{
	const struct lp5810_config *config = dev->config;
	uint8_t reg_major = reg >> 8;
	uint8_t reg_minor = reg & 0xFF;
	if(reg_major>0x03) {
		LOG_ERR("Invalid register address");
		return -EINVAL;
	}
	uint16_t i2c_addr = config->i2c.addr + reg_major;
	return i2c_reg_read_byte(config->i2c.bus, i2c_addr, reg_minor, val);
}


/*
 * @brief check if automation is enabled for this LED.
 *
 * @param dev  LP5810 device.
 * @param led  Requested LED.
 *
 * @return true if automation is enabled, false otherwise.
 */
static bool is_automation_enabled(const struct device *dev, uint32_t led)
{
	uint8_t auto_en_map;
	if (read_register(dev, LP5810_CONFIG_3, &auto_en_map)) {
		LOG_ERR("failed to read auto enable map");
		return false;
	}
	return ((auto_en_map & lp5810_enable_auto_mask[led]) != 0);
}

/* enum for led status */
enum lp5810_led_status {
    LP5810_LED_DISABLE,
    LP5810_LED_ENABLE,
};

/*
 * @brief Set the status of the given LED.
 *
 * @param dev    LP5810 device.
 * @param led    Requested LED.
 * @param status Enable or disable the LED.
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static int set_led_status(const struct device *dev, uint32_t led, enum lp5810_led_status status)
{
    uint8_t current_led_en_map;
    if (read_register(dev, LP5810_LED_EN, &current_led_en_map)) {
        LOG_ERR("failed to read led enable map");
        return -EIO;
    }
	uint8_t new_led_en_map = current_led_en_map;
    if (status == LP5810_LED_ENABLE) {
        new_led_en_map |= lp5810_led_enable_mask[led];
    } else {
        new_led_en_map &= ~lp5810_led_enable_mask[led];
    }

	if (new_led_en_map != current_led_en_map) {
		if (update_register(dev, LP5810_LED_EN, new_led_en_map)){
			LOG_ERR("failed to update led enable map");
			return -EIO;
		}
	}
    return 0;
}

/* enum for disable and enable automation */
enum lp5810_automation {
    LP5810_DISABLE_AUTO,
    LP5810_ENABLE_AUTO,
};

/*
 * @brief Set automation for the given LED.
 *
 * @param dev  LP5810 device.
 * @param led  Requested LED.
 * @param mode Enable or disable automation.
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static int set_automation(const struct device *dev, uint32_t led, enum lp5810_automation mode)
{
    uint8_t current_auto_en_map;
    if (read_register(dev, LP5810_CONFIG_3, &current_auto_en_map)) {
        LOG_ERR("failed to read auto enable map");
        return -EIO;
    }
	uint8_t new_auto_en_map = current_auto_en_map;
	if (mode == LP5810_ENABLE_AUTO) {
        new_auto_en_map |= lp5810_enable_auto_mask[led];
    } else {
        new_auto_en_map &= ~lp5810_enable_auto_mask[led];
    }
	if (new_auto_en_map == current_auto_en_map) {
        return 0;
    }

	if (update_register(dev, LP5810_CONFIG_3, new_auto_en_map)){
		LOG_ERR("failed to update auto enable map");
		return -EIO;
	}
    return update_register(dev, LP5810_CMD_UPDATE, LP5810_CMD_UPDATE_VALUE); //needed for config registers
}

/*
 * @brief Stop automation.
 *
 * @param dev    LP5810 device.
 * @param led    Requested LED.
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static int stop_automation(const struct device *dev, uint32_t led)
{
	//first stop automation
	int ret = update_register(dev, LP5810_CMD_STOP, LP5810_CMD_STOP_VALUE);
	if (ret) {
        LOG_ERR("failed to stop automation engine");
        return ret;
    }
	return set_automation(dev, led, LP5810_DISABLE_AUTO);
}

static int update_aeu1_pwms(const struct device *dev, uint32_t led, uint16_t min_brightness_perc, uint16_t max_brightness_perc)
{
	uint16_t base_register = lp5810_led_base_register[led] + LP5810_LEDX_AEU1_BASE;
	// convert min and max brightness to 0-255 range
	uint8_t min_brightness = (uint8_t)((min_brightness_perc * 255) / 100);
	uint8_t max_brightness = (uint8_t)((max_brightness_perc * 255) / 100);

    int ret = update_register(dev, base_register + LP5810_AEU_PWM_1, min_brightness);
    if (ret) {
        LOG_ERR("failed to set PWM1");
        return ret;
    }
    ret = update_register(dev, base_register + LP5810_AEU_PWM_2, max_brightness);
    if (ret) {
        LOG_ERR("failed to set PWM2");
        return ret;
    }
    ret = update_register(dev, base_register + LP5810_AEU_PWM_3, max_brightness);
    if (ret) {
        LOG_ERR("failed to set PWM3");
        return ret;
    }
    ret = update_register(dev, base_register + LP5810_AEU_PWM_4, min_brightness);
    if (ret) {
        LOG_ERR("failed to set PWM4");
        return ret;
    }
    ret = update_register(dev, base_register + LP5810_AEU_PWM_5, min_brightness);
    if (ret) {
        LOG_ERR("failed to set PWM5");
        return ret;
    }
    return 0;
}

static int led_set_manual_brightness(const struct device *dev, uint32_t led,
					 uint8_t brightness_perc)
{
	uint8_t value = (uint8_t)(((uint16_t)brightness_perc * 255) / 100);
	return update_register(dev, lp5810_manual_pwm_register[led], value);
}
static int led_set_automation_brightness(const struct device *dev, uint32_t led,
					 uint8_t brightness_perc)
{
	struct lp5810_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	int ret = update_aeu1_pwms(dev, led, dev_data->min_brightness, brightness_perc);
	if (ret) {
		LOG_ERR("failed to set pwms");
		return ret;
	}
	//restart the automation
	return update_register(dev, LP5810_CMD_START, LP5810_CMD_START_VALUE);
}

static int lp5810_led_set_brightness(const struct device *dev, uint32_t led,
				     uint8_t brightness_perc)
{
	struct lp5810_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;

	if (led > LP5810_MAX_LED_ID) {
		LOG_ERR("invalid led id %d", led);
		return -EINVAL;
	}
	if (brightness_perc < dev_data->min_brightness ||
		brightness_perc > dev_data->max_brightness) {
		LOG_ERR("invalid brightness value %d (min %d, max %d)", brightness_perc, dev_data->min_brightness, dev_data->max_brightness);
		return -EINVAL;
	}

	int ret = set_led_status(dev, led, LP5810_LED_ENABLE);
	if (ret) {
		LOG_ERR("failed to enable LED");
		return ret;
	}

	if (is_automation_enabled(dev, led))
	{
		return led_set_automation_brightness(dev, led, brightness_perc);
	}
	return led_set_manual_brightness(dev, led, brightness_perc);
}

static int lp5810_led_blink(const struct device *dev, uint32_t led,
				uint32_t delay_on, uint32_t delay_off)
{
	struct lp5810_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	if (led > LP5810_MAX_LED_ID) {
		return -EINVAL;
	}
	int ret = set_led_status(dev, led, LP5810_LED_ENABLE);
	if (ret) {
		LOG_ERR("failed to enable LED");
		return ret;
	}
	//check if we were in manual mode
	if (!is_automation_enabled(dev, led))
	{
		//set manual PWM to 0
		ret = led_set_manual_brightness(dev, led, dev_data->min_brightness);
		if (ret) {
			LOG_ERR("failed to set brightness");
			return ret;
		}
		ret = set_automation(dev, led, LP5810_ENABLE_AUTO);
		if (ret) {
            LOG_ERR("failed to enable automation");
            return ret;
        }
	}
	//set all automation settings
	//  Automation engine will be used like this to have breathing effect
	//    PWM2 ++++ PWM3
	//        +|  |+
	//       + |  | + PWM4
	// PWM1 +  |  |  ++++ PWM5
	//      |T1|T2|T3|T4|
	// we will set T1 = T2 and T1+T2= delay_on
	// and we will set T3 = T4 and T3+T4 = delay_off
	// each time slot is 1 programmable time option

	//set the automation
	// no pause before or after automation
	ret = update_register(dev, lp5810_led_base_register[led] + LP5810_LEDX_AUTO_PAUSE , 0);
	if (ret) {
        LOG_ERR("failed to set pause");
        return ret;
    }
	// set the playback to infinite and only use AEU1
	ret = update_register(dev, lp5810_led_base_register[led] + LP5810_LEDX_AUTO_PLAYBACK , (LP5810_LEDX_AEU_USE_1 << LP5810_LEDX_AEU_USE_SHIFT) | (LP5810_LEDX_PT_INFINITE << LP5810_LEDX_PT_SHIFT));
	if (ret) {
        LOG_ERR("failed to set playback");
        return ret;
    }
	// set PWM values
	// set PWM1 to min_brightness
	ret = update_aeu1_pwms(dev, led, dev_data->min_brightness, dev_data->max_brightness);
	if (ret) {
        LOG_ERR("failed to set pwms");
        return ret;
    }
	//set timing
	// let first determine time
	uint8_t time_option_T1T2 = get_programmable_time_option(delay_on/2);
	uint8_t time_option_T3T4 = get_programmable_time_option(delay_off/2);
	// set T1 and T2
	ret = update_register(dev, lp5810_led_base_register[led] + LP5810_LEDX_AEU1_BASE + LP5810_AEU_T12, (time_option_T1T2 << LP5810_AEU_T12_T1_SHIFT) | (time_option_T1T2 << LP5810_AEU_T12_T2_SHIFT));
	if (ret) {
        LOG_ERR("failed to set T1 and T2");
        return ret;
    }
	// set T3 and T4
	ret = update_register(dev, lp5810_led_base_register[led] + LP5810_LEDX_AEU1_BASE + LP5810_AEU_T34, (time_option_T3T4 << LP5810_AEU_T34_T3_SHIFT) | (time_option_T3T4 << LP5810_AEU_T34_T4_SHIFT));
	if (ret) {
        LOG_ERR("failed to set T3 and T4");
        return ret;
    }
	// we do this aeu1 1 time, so no playback
	ret = update_register(dev, lp5810_led_base_register[led] + LP5810_LEDX_AEU1_BASE + LP5810_AEU_PLAYBACK, LP5810_AEU_NO_PLAYBACK);
	if (ret) {
        LOG_ERR("failed to set playback");
        return ret;
    }
	//start the automation
	return update_register(dev, LP5810_CMD_START, LP5810_CMD_START_VALUE);
}

static inline int lp5810_led_on(const struct device *dev, uint32_t led)
{
	struct lp5810_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	int ret;
	if (led > LP5810_MAX_LED_ID) {
		LOG_ERR("Invalid led id %d", led);
		return -EINVAL;
	}
	if (is_automation_enabled(dev, led))
	{
		ret = stop_automation(dev, led);
		if (ret) {
			LOG_ERR("failed to stop automation");
			return ret;
		}
	}

	ret = set_led_status(dev, led, LP5810_LED_ENABLE);
	if (ret) {
        LOG_ERR("failed to enable LED");
        return ret;
    }

	return led_set_manual_brightness(dev, led, dev_data->max_brightness);
}

static inline int lp5810_led_off(const struct device *dev, uint32_t led)
{
	struct lp5810_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	int ret;

	if (led > LP5810_MAX_LED_ID) {
		return -EINVAL;
	}

	if (is_automation_enabled(dev, led))
	{
		ret = stop_automation(dev, led);
		if (ret) {
			LOG_ERR("failed to stop automation");
			return ret;
		}
	}

	ret = led_set_manual_brightness(dev, led, dev_data->min_brightness);
	if (ret) {
		LOG_ERR("failed to set brightness");
		return ret;
	}
	return set_led_status(dev, led, LP5810_LED_DISABLE);
}

static int lp5810_led_init(const struct device *dev)
{
	const struct lp5810_config *config = dev->config;
	struct lp5810_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	int ret;
	uint8_t value;

	LOG_INF("driver started");

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}
	//will do next test once, lowest 2 bytes of the address should be 0 (is being used for the register address)
	if ((config->i2c.addr & 0x03 )!= 0) {
		LOG_ERR("I2C address incorrect");
		return -EINVAL;
	}

	/* Hardware specific limits in msec */
	//  Automation engine will be used like this to have breathing effect
	//    PWM2 ++++ PWM3
	//        +|  |+
	//       + |  | + PWM4
	// PWM1 +  |  |  ++++ PWM5
	//      |T1|T2|T3|T4|
	// we will set T1 = T2 and T1+T2= delay_on
	// and we will set T3 = T4 and T3+T4 = delay_off
	// each time slot is 1 programmable time option
	dev_data->min_period = lp5810_programmable_time_options[0]*2;
	dev_data->max_period = lp5810_programmable_time_options[LP5810_MAXNR_PROGRAMMABLE_TIME_OPTIONS-1]*2;
	/* Brightness in percentage*/
	dev_data->min_brightness = 0;
	dev_data->max_brightness = 100;

	// first interaction with chip seems to be failing, so let's read ENABLE register first
	// we will try in a for loop with 10msec sleep
	for (int i = 0; i < 10; i++) {
		ret = read_register(dev, LP5810_ENABLE, &value);
		if (ret == 0) {
			// if enabled, reset the chip
			if ((value & LP5810_ENABLE_CHIP_EN) != 0) {
				LOG_INF("Reset LP5810 chip");
				ret = update_register(dev, LP5810_RESET, 0x01);
				if (ret != 0) {
					LOG_ERR("Failed to reset LP5810 chip %d", ret);
					return ret;
				}
				// wait for reset to complete
				k_sleep(K_MSEC(10));
			}
			break;
		}
		k_sleep(K_MSEC(10));
	}

	//let's first disable the chip (might be on, from previous reset)
	ret = update_register(dev, LP5810_ENABLE, 0);
	if (ret != 0) {
        LOG_ERR("Failed to disable LP5810 chip %d", ret);
        return ret;
    }
	//disable all leds
	ret = update_register(dev, LP5810_LED_EN, 0x00);
	if (ret != 0) {
		LOG_ERR("Failed to disable all LP5810 LEDs %d", ret);
		return ret;
	}

	//set LED drive mode as direct drive mode
	ret = update_register(dev, LP5810_CONFIG_1, 0x00);
	if (ret != 0) {
        LOG_ERR("Failed to set LP5810 drive mode %d", ret);
        return ret;
    }
	// disable all fault detection (LOD and LSD)
	ret = update_register(dev, LP5810_CONFIG_12, 0);
	if (ret != 0) {
		LOG_ERR("Failed to set LP5810 lsd threshold %d", ret);
		return ret;
	}

	// disable all automation
	ret = update_register(dev, LP5810_CONFIG_2, 0);
	if (ret != 0) {
		LOG_ERR("Failed to disable all LP5810 Auto Animation %d", ret);
		return ret;
	}

	// update config
	ret = update_register(dev, LP5810_CMD_UPDATE, LP5810_CMD_UPDATE_VALUE);
	if (ret != 0) {
		LOG_ERR("Failed to send update command to LP5810 %d", ret);
		return ret;
	}
	k_sleep(K_SECONDS(1));
	ret = read_register(dev, LP5810_TSD_CONFIG_STATUS, &value);
	if (ret != 0) {
		LOG_ERR("Failed to read LP5810 tsd config status %d", ret);
		return ret;
	}
	if ((value & 0x01) != 0) {
		LOG_ERR("LP5810 TSD config status is not 0");
		return -EIO;
	}

	//Set max DC current for all LEDs in manual mode, this will be board specific
	//evaluated these by looking at LOD status registers (will get 1 if set too high)
	ret = update_register(dev, LP5810_MANUAL_DC_0, 0xD);
	if (ret != 0) {
        LOG_ERR("Failed to set LP5810 peak current DC0 %d", ret);
        return ret;
    }
	ret = update_register(dev, LP5810_MANUAL_DC_1, 0xB);
	if (ret != 0) {
        LOG_ERR("Failed to set LP5810 peak current DC1 %d", ret);
        return ret;
    }
	ret = update_register(dev, LP5810_MANUAL_DC_2, 0x9);
	if (ret != 0) {
        LOG_ERR("Failed to set LP5810 peak current DC2 %d", ret);
        return ret;
    }
	ret = update_register(dev, LP5810_MANUAL_DC_3, 0x0);
	if (ret != 0) {
		LOG_ERR("Failed to set LP5810 peak current DC3 %d", ret);
		return ret;
	}

	//Set max DC current for all LEDs in automation mode
	ret = update_register(dev, LP5810_AUTO_DC_0, 0x0D);
	if (ret != 0) {
		LOG_ERR("Failed to set LP5810 peak current DC0 %d", ret);
		return ret;
	}
	ret = update_register(dev, LP5810_AUTO_DC_1, 0x0B);
	if (ret != 0) {
		LOG_ERR("Failed to set LP5810 peak current DC1 %d", ret);
		return ret;
	}
	ret = update_register(dev, LP5810_AUTO_DC_2, 0x09);
	if (ret != 0) {
		LOG_ERR("Failed to set LP5810 peak current DC2 %d", ret);
		return ret;
	}
	ret = update_register(dev, LP5810_AUTO_DC_3, 0x00);
	if (ret != 0) {
		LOG_ERR("Failed to set LP5810 peak current DC3 %d", ret);
		return ret;
	}

	//enable the chip
	ret = update_register(dev, LP5810_ENABLE, LP5810_ENABLE_CHIP_EN);
	if (ret != 0) {
		LOG_ERR("Failed to enable LP5810 chip %d", ret);
		return ret;
	}
	return 0;
}

static const struct led_driver_api lp5810_led_api = {
	.blink = lp5810_led_blink,
	.set_brightness = lp5810_led_set_brightness,
	.on = lp5810_led_on,
	.off = lp5810_led_off,
};

#define LP5810_DEFINE(id)						\
	static const struct lp5810_config lp5810_config_##id = {	\
		.i2c = I2C_DT_SPEC_INST_GET(id),			\
	};								\
									\
	struct lp5810_data lp5810_data_##id;				\
	DEVICE_DT_INST_DEFINE(id, &lp5810_led_init, NULL,		\
			&lp5810_data_##id,				\
			&lp5810_config_##id, POST_KERNEL,		\
			CONFIG_LED_INIT_PRIORITY,			\
			&lp5810_led_api);				\

DT_INST_FOREACH_STATUS_OKAY(LP5810_DEFINE)
