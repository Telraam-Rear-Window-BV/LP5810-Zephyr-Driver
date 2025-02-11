/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/led.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

//testboard only has 3 leds: RGB, no seperate white led
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
static void test_individual_leds_blinking(const struct device *dev)
{
	for (int i = 0; i < NUM_LEDS; i++) {
		all_leds_off(dev);
		int ret = led_blink(dev, i, BLINK_DELAY_ON, BLINK_DELAY_OFF);
		if (ret) {
			LOG_ERR("Failed to blink %d", i);
			return;
		}
		k_msleep(2*DELAY_TIME);
	}
	all_leds_off(dev);
}

static void test_white_blinking(const struct device *dev)
{
	int ret = 0;
	all_leds_off(dev);
	for (int i = 0; i < NUM_LEDS; i++) {
	    ret = led_blink(dev, i, BLINK_DELAY_ON, BLINK_DELAY_OFF);
		if (ret) {
			LOG_ERR("Failed to blink %d", i);
			return;
		}
		ret = led_set_brightness(dev, i, 100);
		if (ret) {
			LOG_ERR("Failed to set brightness %d", i);
			return;
		}
	}
	k_msleep(2*DELAY_TIME);
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

	LOG_INF("RGB colors, seperately blinking");
	test_individual_leds_blinking(dev);

	LOG_INF("combined white blinking");
	test_white_blinking(dev);
}

static int cmd_color(const struct shell *shell, size_t argc, char *argv[])
{
	const struct device *const dev = DEVICE_DT_GET_ANY(ti_lp5810);
	if (argc <= 3)
	{
		shell_print(shell,"provide 3 color values");
		return 0;
	}

	//convert string to int
	uint8_t color[3];
	for (int i = 0; i < 3; i++)
	{
		color[i] = strtol(argv[i + 1], NULL, 0);
	}
	for (int i = 0; i < 3; i++) {
		led_on(dev, i);
	}
	int ret = led_set_brightness(dev, LED_R, color[0]);
	if (ret) {
		LOG_ERR("Failed to set brightness red.");
		return 0;
	}
	ret = led_set_brightness(dev, LED_G, color[1]);
	if (ret) {
		LOG_ERR("Failed to set brightness green.");
		return 0;
	}
	ret = led_set_brightness(dev, LED_B, color[2]);
	if (ret) {
		LOG_ERR("Failed to set brightness blue.");
		return 0;
	}

	return 0;
}
static int cmd_blink(const struct shell *shell, size_t argc, char *argv[])
{
	const struct device *const dev = DEVICE_DT_GET_ANY(ti_lp5810);
	if (argc <= 4)
	{
		shell_print(shell,"provide 3 color values and time in msec");
		return 0;
	}

	//convert string to int
	uint8_t color[3];
	for (int i = 0; i < 3; i++)
	{
		color[i] = strtol(argv[i + 1], NULL, 0);
	}
	int delay = strtol(argv[4], NULL, 0);
	int ret = led_blink(dev, LED_R, delay, delay);
	if (ret) {
		LOG_ERR("Failed to blink red.");
		return 0;
	}
	ret = led_blink(dev, LED_G, delay, delay);
	if (ret) {
		LOG_ERR("Failed to blink red.");
		return 0;
	}
	ret = led_blink(dev, LED_B, delay, delay);
	if (ret) {
		LOG_ERR("Failed to blink red.");
		return 0;
	}

	ret = led_set_brightness(dev, LED_R, color[0]);
	if (ret) {
		LOG_ERR("Failed to set color red.");
		return 0;
	}
	ret = led_set_brightness(dev, LED_G, color[1]);
	if (ret) {
		LOG_ERR("Failed to set color green.");
		return 0;
	}
	ret = led_set_brightness(dev, LED_B, color[2]);
	if (ret) {
		LOG_ERR("Failed to set color blue.");
		return 0;
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_test_led,
	SHELL_CMD_ARG(color, NULL, "steady color\n", cmd_color, 1, 3),
	SHELL_CMD_ARG(blink, NULL, "reset the k210 chip\n", cmd_blink, 1, 4),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(test_led, &sub_test_led, "testing LEDs related commands.", NULL);