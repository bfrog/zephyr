/**
 * @file
 *
 * @brief Configuration options for the ICM20649 rtio sensor driver.
 */

/*
 * Copyright (c) 2019 Thomas Burdick <thomas.burdick@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_INCLUDE_DRIVERS_RTIO_SENSOR_ICM20649_H_
#define ZEPHYR_INCLUDE_DRIVERS_RTIO_SENSOR_ICM20649_H_

#include <drivers/rtio.h>
#include <drivers/rtio_sensor.h>

#ifdef __cplusplus
extern "C" {
#endif



/**
 * @brief Gyro scale options
 */
enum icm20649_gyro_scale {
	ICM20649_500_DPS,
	ICM20649_1000_DPS,
	ICM20649_2000_DPS,
	ICM20649_4000_DPS,
};

/**
 * @brief Accelerometer scale options
 */
enum icm20649_accel_scale {
	ICM20649_2_G,
	ICM20649_4_G,
	ICM20649_8_G,
	ICM20649_16_G,
	ICM20649_30_G,
};

/**
 * @brief Accelerometer low pass filter options
 *
 * Filter options are given as the cutoff (NBW in the datasheet)
 *
 * Sets the filter enable (FCHOICE in data sheet) flag and
 * DLPF value.
 */
enum icm20649_accel_filter {
	ICM20649_ACCEL_LPF_265_0_HZ,
	ICM20649_ACCEL_LPF_111_4_HZ,
	ICM20649_ACCEL_LPF_50_4_HZ,
	ICM20649_ACCEL_LPF_23_9_HZ,
	ICM20649_ACCEL_LPF_11_5_HZ,
	ICM20649_ACCEL_LPF_5_7_HZ,
	ICM20649_ACCEL_LPF_473_0_HZ,
	ICM20649_ACCEL_LPF_DISABLED,
};

/**
 * @brief Gyro low pass filter options
 *
 * Filter options are given as the cutoff (NBW in the datasheet)
 *
 * Sets the filter enable (FCHOICE in data sheet) flag and
 * DLPF value.
 */
enum icm20649_gyro_filter {
	ICM20649_GYRO_LPF_229_8_HZ,
	ICM20649_GYRO_LPF_187_6_HZ,
	ICM20649_GYRO_LPF_154_3_HZ,
	ICM20649_GYRO_LPF_73_3_HZ,
	ICM20649_GYRO_LPF_35_9_HZ,
	ICM20649_GYRO_LPF_17_8_HZ,
	ICM20649_GYRO_LPF_8_9_HZ,
	ICM20649_GYRO_LPF_376_5_HZ,
	ICM20649_GYRO_LPF_DISABLED,
};

/**
 * @brief Temperature low pass filter options
 *
 * Filter options are given as the cutoff (NBW in the datasheet)
 *
 * Sets the TEMP_DLPF value.
 */
enum icm20649_temp_filter {
	ICM20649_TEMP_LPF_7932_0_HZ,
	ICM20649_TEMP_LPF_217_9_HZ,
	ICM20649_TEMP_LPF_123_5_HZ,
	ICM20649_TEMP_LPF_65_9_HZ,
	ICM20649_TEMP_LPF_34_1_HZ,
	ICM20649_TEMP_LPF_17_3_HZ,
	ICM20649_TEMP_LPF_8_8_HZ,

};

/**
 * @brief Accelerometer configuration
 */
struct icm20649_accel_config {
	/**
	 * @brief Enabled state turns on power to this part of the sensor
	 */
	bool enabled;

	/**
	 * @brief Perform a self test
	 *
	 * This will enable the self test option for the accel channels.
	 *
	 * This tests the physical MEMS accelerometer sensor by actuating
	 * the device to produce an output signal. The self test procedure then
	 * verifies the output signal is within range of the expected output
	 * from the manufacturer before configuring the device.
	 *
	 * This adds a significant delay to sensor samples being streamed.
	 */
	bool self_test;

	/**
	 * @brief Low power mode duty cycles power to the MEMS sensor
	 */
	bool low_power_mode;

	/**
	 * @brief Digital low pass filtering of the sensor
	 */
	enum icm20649_accel_filter filter;

	/**
	 * @brief Full scale of the sensor samples
	 */
	enum icm20649_accel_scale scale;

	/**
	 * @brief Sample rate divider
	 *
	 * Max value is 4096 (12 bits), values greater than 4096 will be masked
	 * off to the 12th bit.
	 *
	 * The output datarate can be computed as 1125.0/(1+sample_rate_div)
	 *
	 * Note that when the low power mode and filter are disabled this has no
	 * effect. The output data rate is then 4500Hz.
	 *
	 * Note that when both the accelerometer and gyro are enabled with the
	 * FIFO the output datarates are locked together.
	 */
	u16_t sample_rate_div;
};

/**
 * @brief Gyro configuration
 */
struct icm20649_gyro_config {
	/**
	 * @brief Enabled state turns on power to this sensor
	 *
	 * If the sensor was previously not enabled there is a 35ms delay in
	 * turning the device on.
	 */
	bool enabled;

	/**
	 * @brief Perform a self test if set to true
	 *
	 * This will enable the self test option for the gyro channels.
	 *
	 * This tests the physical MEMS gyro sensor by actuating the gyro
	 * to produce an output signal. The self test procedure then verifies
	 * the output signal is within range of the expected output from
	 * the manufacturer before configuring the device.
	 *
	 * This adds a significant delay to sensor samples being streamed.
	 */
	bool self_test;

	/**
	 * @brief Low power mode duty cycles power to the MEMS sensor
	 */
	bool low_power_mode;

	/**
	 * @brief Digital low pass filtering of the sensor
	 */
	enum icm20649_gyro_filter filter;

	/**
	 * @brief Full scale of the sensor samples
	 */
	enum icm20649_gyro_scale scale;

	/**
	 * @brief Sample rate divider
	 *
	 * The output datarate can be computed as 1100.0/(1+sample_rate_div)
	 *
	 * Note that when the low power mode and filter are disabled this has no
	 * effect. The output data rate is then 9000Hz.
	 *
	 * Note that when both the accelerometer and gyro are enabled with the
	 * FIFO the output datarates are locked together.
	 *
	 * The FIFO, when enabled, uses this sample rate divider.
	 */
	u8_t sample_rate_div;
};

/**
 * @brief Temp configuration
 */
struct icm20649_temp_config {
	/**
	 * @brief Enabled state turns on power to this sensor
	 */
	bool enabled;

	/**
	 * @brief Digital low pass filtering of the sensor
	 */
	enum icm20649_temp_filter filter;
};

/**
 * @brief Configuration settings for ICM20649
 *
 * To be used in rtio_config struct as the driver_config member
 */
struct icm20649_config {
	/**
	 * @brief Reset device before doing configuration.
	 *
	 * This does cause some delay in reconfiguring to allow the device to
	 * start.
	 */
	bool reset;

	/**
	 * @brief Sleep turns off all circuitry
	 */
	bool sleep;

	/**
	 * @brief Enable digital circuitry low power mode,
	 *
	 * Accel and Gyro must be in low power mode for this to take affect
	 */
	bool digital_low_power;

	/**
	 * @brief Accelerometer configuration
	 */
	struct icm20649_accel_config accel_config;

	/**
	 * @brief Gyro configuration
	 */
	struct icm20649_gyro_config gyro_config;

	/**
	 * @brief Temp configuration
	 */
	struct icm20649_temp_config temp_config;

	/**
	 * @brief Enable the hardware fifo
	 *
	 * Uses the accel, gyro, and temp config options to enable or disable
	 * their usage with the fifo. When enabled the output data rate
	 * is tied together for all sensors and is set by the gyro
	 * sample_rate_div. Implicitly enables the FIFO related interrupts.
	 */
	bool fifo_enabled;

	/**
	 * @brief Enable the data ready interrupt.
	 *
	 * Is not used if fifo_enabled is true.
	 */
	bool data_ready_enabled;
}


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_RTIO_SENSOR_ICM20649_H_ */
