/*
 * Copyright (c) 2019  Thomas Burdick <thomas.burdick@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RTIO_SENSOR_H_
#define ZEPHYR_INCLUDE_DRIVERS_RTIO_SENSOR_H_
/**
 * @brief RTIO Sensor Interface
 * @defgroup rtio_sensor_interface RTIO Sensor Interface
 * @ingroup io_interfaces
 * @{
 */

#include <kernel.h>
#include <device.h>
#include <drivers/rtio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Real-Time IO Sensor API
 *
 * Each  RTIO Sensor acts as a pollable stream of buffers with a driver that
 * provides a way of deserializing the raw sensor data buffers into meaningful
 * values.
 *
 * Each RTIO Sensor is expected to output buffers containing sample sets (such
 * as 9 axis IMU data) in a device specific format understood by the driver.
 * Each driver must then implement the sensor reader api and functionality
 * to deserialize this stream of buffers into meaningful SI unit data. The
 * caller may request different numerical formats of this data.
 *
 * Sample sets may contain 0 or more sensor channel values. A sensor channel
 * is understood to be something like the X channel or Y channel of a
 * 3 dimensional accelerometer.
 *
 * A sample set may vary in what it contains, even in the same block.
 * Some devices allow for different sampling rates of different channels.
 *
 * It must be understood by the user of this API what devices they are using
 * and what channels may or may not be available at any given time.
 *
 * For each new rtio_block received by from the sensor a rtio_sensor_reader
 * should be re-initialized. This allows the driver to provide an optimal
 * reader for the rtio_block avoiding unnecessary branching when possible.
 *
 * *TODO* To start only 32bit floating point numerical formatted values are
 * provided. Q format values with scale factors would be very useful for
 * use with CMSIS DSP or many SoCs that do not contain FPUs. The reader
 * *should* provide simple ways of converting integer readings into
 * useful numerical formats in common functions.
 */

/**
 * @private
 */
struct rtio_sensor_reader;

/**
 * @private
 * @brief Get the number of sample sets in the current block
 */
typedef ssize_t (*rtio_sensor_reader_sample_sets_t)(const struct sensor_reader *reader);

/**
 * @private
 * @brief Iterate to the next sample set in the block
 *
 * The first call to next on an iterator will do *nothing* as
 * its already at the 0th position. Instead it will replace the
 * noop func with a true next func on its first call.
 *
 * This allows for a straight forward while loop to be created.
 *
 * @returns 0 when complete, 1 otherwise
 *
 */
typedef int (*rtio_sensor_reader_next_t)(struct rtio_sensor_reader *reader);

/**
 * @private
 * @brief Get the value of a sample sets channel, if it exists
 *
 * The channel is an index into a device specified sample channel.
 * For example, A 3 axis accelerometer may assign channel 0 to
 * the X axis, 1 to the Y axis, and 2 to the Z axis.
 *
 * Note that if the channel value is non existent in the sample set
 * an -EINVAL should be returned by the reader, otherwise 0 on success.
 *
 * *TODO* discuss how the channel identifier might be obtained for something like
 * the X channel so that the same API calls may be used for multiple devices.
 */
typedef int (*rtio_sensor_reader_get_float_t)(const struct sensor_reader *reader,
					      const u32_t channel, float *val);


/**
 * @private
 * @brief Calculate or return the timestamp of the current sample set
 *
 * The timestamp may be part of the sample set or may be interpolated from the
 * blocks begin and end timestamps along with the number of sample sets in the
 * block or it may be in the block data itself.
 *
 * Information about if the timestamp is interpolated or not, and the clock
 * source may be found by potentially future functions as needed.
 *
 * @return -EINVAL if no timestamp is available, 0 otherwise
 */
typedef int (*rtio_sensor_reader_timestamp_t)(const struct rtio_sensor_reader *reader,
					      u32_t *timestamp);

/**
 * @brief RTIO Sensor Reader
 */
struct rtio_sensor_reader {
	rtio_sensor_sample_sets_t sample_sets;
	rtio_sensor_reader_next_t next;
	rtio_sensor_reader_get_float_t get_float;
	rtio_sensor_reader_timestamp_t timestamp;

	struct rtio_block *block;
	size_t position;
};

/**
 * @brief Get the number of sample sets in a sensor reader
 *
 * The reader is only valid after it has been initialized by the device driver
 * with a valid block and until it has finished reading the block.
 *
 * @param reader Pointer to a valid rtio_sensor_reader.
 *
 * @return Number of sample sets or -EINVAL if the sensor_reader
 * is invalid.
 */
ssize_t rtio_sensor_reader_sample_sets(const struct rtio_sensor_reader *reader)
{
	if(reader->block == NULL) {
		return -EINVAL;
	}
	return reader->sample_sets(reader);
}




/**
 * @brief Iterate to the next sample set
 *
 * The first call to this does nothing but is required as the initial
 * state is a no-op call. After the first call this moves
 * the position of the reader to the index into the buffer where
 * the next sample set lies until the last sample set is read.
 *
 * If the next sample set is beyond the end of the buffer this should
 * return 0 and set its *block to NULL to remain invalid after.
 *
 * @param reader Pointer to a valid rtio_sensor_reader.
 *
 * @return -EINVAL if invalid, 0 if no more sample sets remain, 1 if more do
 */
int rtio_sensor_reader_next(struct rtio_sensor_reader *reader)
{
	if(reader->block == NULL) {
		return -EINVAL;
	}
	return reader->next(reader);
}

/**
 * @brief Get a sample sets value for a channel as a float
 *
 * @param reader Pointer to a valid rtio_sensor_reader.
 * @param channel An channel id provided by the sensor driver
 * @param[out] value A pointer to a valid float where the value will be assigned
 *
 * @return -EINVAL if invalid reader or channel given, 0 if value assigned
 */
int rtio_sensor_reader_get_float(struct rtio_sensor_reader *reader, u32_t channel, float *value)
{
	if(reader->block == NULL) {
		return -EINVAL;
	}
	return reader->get_float(reader, channel, value);
}

/**
 * @brief Get a timestamp for the current sample set
 *
 * @param reader Pointer to a valid rtio_sensor_reader.
 * @param[out] timestamp A pointer to a valid u32_t where the timestamp will be assigned.
 *
 * @return -EINVAL if invalid reader or timestamp unavailable, 0 if value assigned
 */
int rtio_sensor_reader_timestamp(struct rtio_sensor_reader *reader, u32_t *timestamp)
{
	if(reader->block == NULL) {
		return -EINVAL;
	}
	return reader->timestamp(reader, timestamp);
}


/**
 * @private
 * @brief Initialize a block reader with the given device and block
 *
 * The purpose of this call is to allow for distinct statically created
 * rtio_sensor_reader functions to exist for various configurations a sensor
 * may be in. This avoids the need, potentially, of having many branch
 * statements in the reader itself which is expected to be called
 * much more frequently than this function.
 *
 * The caller is still responsible for freeing the rtio_block.
 */
typedef int (*rtio_sensor_reader_t)(struct device *dev,
				    struct rtio_block *block,
				    struct rtio_sensor_reader *reader);

/**
 * @brief Real-Time IO Sensor Driver API
 *
 * This extends the RTIO device API with sensor specific functionality.
 *
 * Specifically this API *requires* a sensor provide a way of initializing
 * a sensor reader based on the device and a rtio_block.
 */
struct rtio_sensor_api {
	/** Configuration function pointer *must* be implemented */
	struct rtio_api rtio_api;

	/** Function to initialize RTIO Block Reader using this device and a block  */
	rtio_sensor_reader_t sensor_reader;
};


/**
 * @brief Initialize a block reader with the given device and block
 *
 * The purpose of this call is to allow for distinct statically created
 * rtio_sensor_reader functions to exist for various configurations a sensor
 * may be in. This avoids the need, potentially, of having many branch
 * statements in the reader itself which is expected to be called
 * much more frequently than this function.
 *
 * The caller is still responsible for freeing the rtio_block.
 *
 * @param dev A valid pointer to an instance of device that implements rtio_sensor_device
 * @param block A valid pointer to an rtio_block instance
 * @param reader A pointer to a rtio_sensor_reader which is initialize on success
 *
 * @return 0 on success, -EINVAL if the block isn't understood by the device driver
 */
int rtio_sensor_reader(struct device *dev,
		       struct rtio_block *block,
		       struct rtio_sensor_reader *reader)
{
	const struct rtio_sensor_api *api =
		(const struct rtio_sensor_api *)dev->driver_api;

	return api->sensor_reader(dev, block, reader);
}


#ifdef __cplusplus
}
#endif

#endif
