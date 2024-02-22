#pragma once

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>

struct joystick_data {
  const struct device *adc;
  struct adc_sequence as;
  int16_t xy_raw[2];
  sensor_trigger_handler_t trigger_handler;
  struct sensor_trigger trigger;
  int32_t trigger_fs;
  struct k_timer timer;
  struct k_work work;
};

struct joystick_config {
  uint8_t channel_x;
  uint8_t channel_y;
  uint32_t min_mv;
  uint32_t max_mv;
};