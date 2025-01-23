/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/led.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#define NUM_LEDS 3
#define LED_R 2
#define LED_G 1
#define LED_B 0

#define BLINK_DELAY_ON 500
#define BLINK_DELAY_OFF 500
#define DELAY_TIME 2000
#define COLORS_TO_SHOW 7

static uint8_t colors[COLORS_TO_SHOW][3] = {
	{ 100, 100, 100 }, /*< White  */
	{ 100, 100,   0 }, /*< Yellow */
	{ 100,   0, 100 }, /*< Pink */
	{   0, 100, 100 }, /*< Cyan */
    {  63,  12,  94 }, /*< Purple */
	{ 100,  65,   0 }, /*< Orange */
    {  24,  71,  54 }, /*< Mint */
};


static void all_leds_off(const struct device *dev)
{
	for (int i = 0; i < NUM_LEDS; i++) {
		led_off(dev, i);
	}
}

static void test_individual_leds(const struct device *dev)
{
	for (int i = 0; i < NUM_LEDS; i++) {
		all_leds_off(dev);
		led_on(dev, i);
		k_msleep(DELAY_TIME);
	}
	all_leds_off(dev);
}

static void show_colors(const struct device *dev)
{
	for (int i = 0; i < COLORS_TO_SHOW; i++) {
		int ret = led_set_brightness(dev, LED_R, colors[i][0]);
		if (ret) {
			LOG_ERR("Failed to set color red.");
			return;
		}
		ret = led_set_brightness(dev, LED_G, colors[i][1]);
		if (ret) {
			LOG_ERR("Failed to set color green.");
			return;
		}
		ret = led_set_brightness(dev, LED_B, colors[i][2]);
		if (ret) {
			LOG_ERR("Failed to set color blue.");
			return;
		}
		k_msleep(DELAY_TIME);
	}
	all_leds_off(dev);
}

void main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(ti_lp5810);
	if (!device_is_ready(dev)) {
		LOG_ERR("LED device %s is not ready", dev->name);
		return;
	}
	LOG_INF("Found LED device %s", dev->name);

	LOG_INF("Testing leds: static color");
	test_individual_leds(dev);

	LOG_INF("mixing colors, static");
	show_colors(dev);

	LOG_INF("mixing colors, blinking");
	int ret = led_blink(dev, LED_R, BLINK_DELAY_ON, BLINK_DELAY_OFF);
	if (ret) {
		LOG_ERR("Failed to blink red.");
		return;
	}
	ret = led_blink(dev, LED_G, BLINK_DELAY_ON, BLINK_DELAY_OFF);
	if (ret) {
		LOG_ERR("Failed to blink green.");
		return;
	}
	ret = led_blink(dev, LED_B, BLINK_DELAY_ON, BLINK_DELAY_OFF);
	if (ret) {
		LOG_ERR("Failed to blink blue.");
		return;
	}
	show_colors(dev);

}
