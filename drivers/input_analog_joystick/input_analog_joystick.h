#pragma once

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>

struct joystick_data {
  const struct device *adc;
  const struct device *dev;
  struct adc_sequence adc_seq;
  int16_t buffer[2];
  struct k_timer timer;
  struct k_work work;
};

struct joystick_config {
  uint8_t channel_x;
  uint8_t channel_y;
  uint32_t min_mv;
  uint32_t max_mv;
};