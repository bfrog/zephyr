/*
 * Copyright (c) 2019 LeanUp Design
 * Copyright (c) 2019 Tom Burdick <tom.burdick@electromatic.us>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file TDK ICM 20649 3 axis gyro and accelerometer driver data
 *
 * Defines structs used for zephyr's device driver model
 */

#ifndef ZEPHYR_RTIO_SENSOR_ICM20649_DATA_H
#define ZEPHYR_RTIO_SENSOR_ICM20649_DATA_H

#include <drivers/rtio.h>
#include <drivers/rtio/sensor/icm20649.h>
#include "../../rtio_context.h"
#include <zephyr/types.h>
#include <gpio.h>

/**
 * @private
 * @brief Static driver instance config struct
 */
struct icm20649_drv_config {
	char *dev_name;
};

/**
 * @private
 * @brief Driver data struct
 */
struct icm20649_drv_data {
    struct rtio_context context;
    struct rtio_icm20649_config config;
	struct device *spi;
	struct device *gpio;
	struct rtio_config config;
	struct gpio_callback gpio_cb;
	struct counter_callback counter_cb;
};

#endif /* ZEPHYR_RTIO_SENSOR_ICM20649_DATA_H */
