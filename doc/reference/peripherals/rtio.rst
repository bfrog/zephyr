.. _rtio_interface:

Real-Time IO
############

The real-time I/O or RTIO subsystem exposes an API for efficiently
moving information in a latency and processing friendly way, between
a Zephyr application and external I2C or SPI devices (such as sensors).

Overview
********

RTIO provides an API for allocating and describing continguous blocks of
memory as well as an API for sourcing these blocks from a physical device.

Blocks, defined by :c:type:`struct rtio_block`, provide the unit of data
being transferred. These blocks contain a layout identifier provided by
the source of the block, timestamps for when the block
began and finished being created, as well as the length and size in bytes.

Blocks are used to allow for simple DMA transfers as well as controlling the
latency between a source and sink.

The block layout identifier is a way for the source of the block to describe
whats in the block. This can be used to identify which channels of information
are in the block, the sample size, and more. It is source defined and
not unique between different sources.

The begin and end timestamps may be used to interpolate when events have
occured without needing a timestamp for every reading. For example when
reading a hardware FIFO from a device sampling at a constant rate the
begin and and timestamps may be used to infer when each sample in the block
happened in your application by doing simple linear interpolation.

They may also be used to understand the sample rate of the device better
as often times devices have nominal sampling rates that vary device to device.

Sources of blocks can be sensors such as a high sample rate accelerometer, gyro,
and magnetometer. In these cases the driver will typically provide blocks
encoded in the sensor's native format which is often times the most compact
representation.

Intermediary processing steps representing both sinks and sources of data are
also possible. One possible task is taking 6 or 9 axis IMU sensor and
producing a orientation quaternion vectors relative to earth.

The driving event of actually producing or consuming data is the `rtio_trigger`
function. It is meant to be an ISR safe function, meaning it can be called in any interrupt
handler such as a GPIO interrupt callback. In most cases the trigger call can be setup
to happen automatically for you based on options given to the `rtio_configure` function.

As an example to trigger a sample reading based on a data ready GPIO interrupt
rtio_configure should be called with the appropriately trigger source
configuration for that GPIO pin along with any device specific configuration
options to enable data ready signaling.

Alternatively a periodic or timer based sample reading is also possible with
rtio_configure by specifying the trigger source as a counter device and
describing a period to trigger at.

The last option is to manually trigger the device reading. This can be done
any time the application wishes a device read to occur so long as only one
thread calls trigger at any given time.

The RTIO subsystem lets you both efficiently move data around and
efficiently process chunks of data at a time in a flexible way.


API Reference
*************

.. doxygengroup:: rtio_interface
   :project: Zephyr
