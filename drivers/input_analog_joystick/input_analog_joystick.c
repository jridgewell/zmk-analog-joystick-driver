/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_analog_joystick

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/adc/adc.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
#include <zephyr/logging/log.h>

#include "input_analog_joystick.h"

LOG_MODULE_REGISTER(zmk_input_analog_joystick);

#define X_AXIS_TO_ADC_CHAN_ID (0)
#define Y_AXIS_TO_ADC_CHAN_ID (1)

#define ADC_INPUT_POS_OFFSET SAADC_CH_PSELP_PSELP_AnalogInput0

K_THREAD_STACK_DEFINE(joystick_trigger_stack_area,
                      CONFIG_INPUT_ANALOG_JOYSTICK_WORKQUEUE_STACK_SIZE);
static struct k_work_q joystick_work_q;

static void joystick_timer_cb(struct k_timer *timer) {
  struct joystick_data *data = CONTAINER_OF(timer, struct joystick_data, timer);
  k_work_submit_to_queue(&joystick_work_q, &data->work);
}

static inline int32_t normalize(int32_t mv, int32_t min_mv, int32_t max_mv) {
  double val =
      CLAMP(2.0 * (mv < min_mv ? 0 : mv - min_mv) / (max_mv - min_mv) - 1.0,
            -1.0, 1.0);
  return (int32_t)(127 * val);
}

static void joystick_work_cb(struct k_work *work) {
  struct joystick_data *data = CONTAINER_OF(work, struct joystick_data, work);
  const struct device *dev = data->dev;
  const struct joystick_config *config = dev->config;
  struct adc_sequence *seq = &data->adc_seq;

  int rc = 0;

  rc = adc_read(data->adc, seq);
  seq->calibrate = false;

  int32_t x_mv = data->buffer[X_AXIS_TO_ADC_CHAN_ID];
  int32_t y_mv = data->buffer[Y_AXIS_TO_ADC_CHAN_ID];

  adc_raw_to_millivolts(adc_ref_internal(data->adc), ADC_GAIN_1_6,
                        seq->resolution, &x_mv);
  adc_raw_to_millivolts(adc_ref_internal(data->adc), ADC_GAIN_1_6,
                        seq->resolution, &y_mv);

  int32_t dx = normalize(x_mv, config->min_mv, config->max_mv);
  int32_t dy = normalize(y_mv, config->min_mv, config->max_mv);

  input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
  input_report_rel(dev, INPUT_REL_X, dy, true, K_FOREVER);
}

static int joystick_channel_setup(const struct device *adc, uint8_t channel_id,
                                  uint8_t adc_channel) {
  struct adc_channel_cfg channel_cfg = {
      .gain = ADC_GAIN_1_6,
      .reference = ADC_REF_INTERNAL,
      .acquisition_time = ADC_ACQ_TIME_DEFAULT,
      .channel_id = 1 + channel_id,
      .input_positive = ADC_INPUT_POS_OFFSET + adc_channel,
  };

  int rc = adc_channel_setup(adc, &channel_cfg);
  LOG_DBG("AIN%u setup returned %d", channel_id, rc);
  return rc;
}

static int joystick_init(const struct device *dev) {
  struct joystick_data *data = dev->data;
  const struct joystick_config *config = dev->config;

  if (data->adc == NULL) {
    return -ENODEV;
  }

  data->dev = dev;

  int rc = joystick_channel_setup(data->adc, X_AXIS_TO_ADC_CHAN_ID,
                                  config->channel_x);
  if (rc < 0) {
    return rc;
  }

  rc = joystick_channel_setup(data->adc, Y_AXIS_TO_ADC_CHAN_ID,
                              config->channel_y);
  if (rc < 0) {
    return rc;
  }

  data->adc_seq = (struct adc_sequence){
      .channels =
          BIT(1 + X_AXIS_TO_ADC_CHAN_ID) | BIT(1 + Y_AXIS_TO_ADC_CHAN_ID),
      .buffer = data->buffer,
      .buffer_size = sizeof(data->buffer),
      .oversampling = 0,
      .resolution = 12,
      .calibrate = true,
  };

  k_work_init(&data->work, joystick_work_cb);
  k_work_queue_start(&joystick_work_q, joystick_trigger_stack_area,
                     K_THREAD_STACK_SIZEOF(joystick_trigger_stack_area),
                     CONFIG_INPUT_ANALOG_JOYSTICK_WORKQUEUE_PRIORITY, NULL);

  k_timer_init(&data->timer, joystick_timer_cb, NULL);
  k_timer_start(&data->timer, K_NO_WAIT, K_MSEC(100));
  return rc;
}

#define JOYSTICK_INST(n)                                                  \
  static struct joystick_data joystick_data_##n = {                       \
      .adc = DEVICE_DT_GET(DT_INST_IO_CHANNELS_CTLR_BY_NAME(n, x_axis))}; \
                                                                          \
  static const struct joystick_config joystick_config_##n = {             \
      .channel_x = DT_INST_IO_CHANNELS_INPUT_BY_NAME(n, x_axis),          \
      .channel_y = DT_INST_IO_CHANNELS_INPUT_BY_NAME(n, y_axis),          \
      .max_mv = DT_INST_PROP(n, max_mv),                                  \
      .min_mv = COND_CODE_0(DT_INST_NODE_HAS_PROP(n, min_mv), (0),        \
                            (DT_INST_PROP(n, min_mv))),                   \
  };                                                                      \
                                                                          \
  DEVICE_DT_INST_DEFINE(n, joystick_init, NULL, &joystick_data_##n,       \
                        &joystick_config_##n, POST_KERNEL,                \
                        CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(JOYSTICK_INST)
