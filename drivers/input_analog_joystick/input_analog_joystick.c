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

#ifdef CONFIG_ADC_NRFX_SAADC
#define ADC_INPUT_POS_OFFSET SAADC_CH_PSELP_PSELP_AnalogInput0
#else
#define ADC_INPUT_POS_OFFSET 0
#endif

K_THREAD_STACK_DEFINE(joystick_trigger_stack_area,
                      CONFIG_INPUT_ANALOG_JOYSTICK_WORKQUEUE_STACK_SIZE);
static struct k_work_q joystick_work_q;
static bool is_joystick_work_q_ready = false;

static int joystick_sample_fetch(const struct device *dev,
                                 enum sensor_channel chan) {
  struct joystick_data *drv_data = dev->data;
  const struct joystick_config *drv_cfg = dev->config;
  struct adc_sequence *as = &drv_data->as;

  if (chan != SENSOR_CHAN_POS_DX && chan != SENSOR_CHAN_POS_DY &&
      chan != SENSOR_CHAN_ALL) {
    LOG_ERR("Selected channel is not supported: %d.", chan);
    return -ENOTSUP;
  }

  int rc = 0;

  rc = adc_read(drv_data->adc, as);
  // First read is setup as calibration
  as->calibrate = false;

  int32_t x_mv = drv_data->xy_raw[X_AXIS_TO_ADC_CHAN_ID];
  int32_t y_mv = drv_data->xy_raw[Y_AXIS_TO_ADC_CHAN_ID];

  adc_raw_to_millivolts(adc_ref_internal(drv_data->adc), ADC_GAIN_1_6,
                        as->resolution, &x_mv);
  adc_raw_to_millivolts(adc_ref_internal(drv_data->adc), ADC_GAIN_1_6,
                        as->resolution, &y_mv);
  LOG_DBG("x: %d y: %d", x_mv, y_mv);

  return rc;
}

static int joystick_channel_get(const struct device *dev,
                                enum sensor_channel chan,
                                struct sensor_value *val) {
  struct joystick_data *drv_data = dev->data;
  const struct joystick_config *drv_cfg = dev->config;
  struct adc_sequence *as = &drv_data->as;

  int32_t x_mv = drv_data->xy_raw[X_AXIS_TO_ADC_CHAN_ID];
  int32_t y_mv = drv_data->xy_raw[Y_AXIS_TO_ADC_CHAN_ID];

  adc_raw_to_millivolts(adc_ref_internal(drv_data->adc), ADC_GAIN_1_3,
                        as->resolution, &x_mv);
  adc_raw_to_millivolts(adc_ref_internal(drv_data->adc), ADC_GAIN_1_3,
                        as->resolution, &y_mv);

  double out = 0.0;
  switch (chan) {
    // convert from millivolt to normalized output in [-1.0, 1.0]
    case SENSOR_CHAN_POS_DX:
      out = 2.0 * x_mv / (drv_cfg->max_mv - drv_cfg->min_mv) - 1.0;
      sensor_value_from_double(val, out);
      break;
    case SENSOR_CHAN_POS_DY:
      out = 2.0 * y_mv / (drv_cfg->max_mv - drv_cfg->min_mv) - 1.0;
      sensor_value_from_double(val, out);
      break;
    case SENSOR_CHAN_ALL:
      out = 2.0 * x_mv / (drv_cfg->max_mv - drv_cfg->min_mv) - 1.0;
      sensor_value_from_double(val, out);
      out = 2.0 * y_mv / (drv_cfg->max_mv - drv_cfg->min_mv) - 1.0;
      sensor_value_from_double(val + 1, out);
      break;
    default:
      return -ENOTSUP;
  }

  return 0;
}

static int joystick_trigger_set(const struct device *dev,
                                const struct sensor_trigger *trig,
                                sensor_trigger_handler_t handler) {
  struct joystick_data *drv_data = dev->data;
  enum sensor_channel chan = trig->chan;
  enum sensor_trigger_type type = trig->type;
  LOG_DBG("chan %u type %u", chan, type);

  /*
      if (chan != SENSOR_CHAN_ALL || type != SENSOR_TRIG_DATA_READY) {
          return -ENOTSUP;
      }
  */
  drv_data->trigger = *trig;
  drv_data->trigger_handler = handler;

  k_timer_start(&drv_data->timer, K_MSEC(1000), K_MSEC(1000));
  return 0;
}

static void joystick_timer_cb(struct k_timer *item) {
  struct joystick_data *drv_data =
      CONTAINER_OF(item, struct joystick_data, timer);
  k_work_submit_to_queue(&joystick_work_q, &drv_data->work);
}

static void joystick_work_fun(struct k_work *item) {
  /*
  struct joystick_data *drv_data =
      CONTAINER_OF(item, struct joystick_data, work);
  struct device *dev = CONTAINER_OF(&drv_data, struct device, data);
  k_timer_stop(&drv_data->timer);

  joystick_sample_fetch(dev, SENSOR_CHAN_ALL);

  if (drv_data->trigger_handler) {
      drv_data->trigger_handler(dev, &drv_data->trigger);
  }
  k_timer_start(&drv_data->timer, K_MSEC(1000), K_MSEC(1000));
  */
}

static int joystick_init(const struct device *dev) {
  struct joystick_data *drv_data = dev->data;
  const struct joystick_config *drv_cfg = dev->config;

  if (drv_data->adc == NULL) {
    return -ENODEV;
  }

  struct adc_channel_cfg channel_cfg = {
      .gain = ADC_GAIN_1_6,
      .reference = ADC_REF_INTERNAL,
      .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
      .channel_id = X_AXIS_TO_ADC_CHAN_ID,
      .input_positive = ADC_INPUT_POS_OFFSET + drv_cfg->channel_x,
  };

  int rc = adc_channel_setup(drv_data->adc, &channel_cfg);
  LOG_DBG("AIN%u setup returned %d", drv_cfg->channel_x, rc);
  if (rc < 0) {
    return rc;
  }

  channel_cfg.channel_id = Y_AXIS_TO_ADC_CHAN_ID;
  channel_cfg.input_positive = ADC_INPUT_POS_OFFSET + drv_cfg->channel_y;

  rc = adc_channel_setup(drv_data->adc, &channel_cfg);
  LOG_DBG("AIN%u setup returned %d", drv_cfg->channel_y, rc);
  if (rc < 0) {
    return rc;
  }

  drv_data->as = (struct adc_sequence){
      .channels = BIT(X_AXIS_TO_ADC_CHAN_ID) | BIT(Y_AXIS_TO_ADC_CHAN_ID),
      .buffer = drv_data->xy_raw,
      .buffer_size = sizeof(drv_data->xy_raw),
      .oversampling = 0,
      .resolution = 12,
      .calibrate = true,
  };

  k_timer_init(&drv_data->timer, joystick_timer_cb, NULL);
  k_work_init(&drv_data->work, joystick_work_fun);
  if (!is_joystick_work_q_ready) {
    k_work_queue_start(&joystick_work_q, joystick_trigger_stack_area,
                       K_THREAD_STACK_SIZEOF(joystick_trigger_stack_area),
                       CONFIG_INPUT_ANALOG_JOYSTICK_WORKQUEUE_PRIORITY, NULL);
    is_joystick_work_q_ready = true;
  }
  return rc;
}

#define JOYSTICK_INST(n)                                                  \
  static struct joystick_data joystick_data_##n = {                       \
      .adc = DEVICE_DT_GET(DT_INST_IO_CHANNELS_CTLR_BY_NAME(n, x_axis))}; \
  static const struct joystick_config joystick_config_##n = {             \
      .channel_x = DT_INST_IO_CHANNELS_INPUT_BY_NAME(n, x_axis),          \
      .channel_y = DT_INST_IO_CHANNELS_INPUT_BY_NAME(n, y_axis),          \
      .max_mv = DT_INST_PROP(n, max_mv),                                  \
      .min_mv = COND_CODE_0(DT_INST_NODE_HAS_PROP(n, min_mv), (0),        \
                            (DT_INST_PROP(n, min_mv))),                   \
  };                                                                      \
  DEVICE_DT_INST_DEFINE(n, joystick_init, NULL, &joystick_data_##n,       \
                        &joystick_config_##n, POST_KERNEL,                \
                        CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(JOYSTICK_INST)
