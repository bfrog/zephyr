/*
 * Copyright (c) 2019  Thomas Burdick <thomas.burdick@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RTIO_H_
#define ZEPHYR_INCLUDE_DRIVERS_RTIO_H_
/**
 * @brief RTIO Interface
 * @defgroup rtio_interface RTIO Interface
 * @ingroup io_interfaces
 * @{
 */

#include <kernel.h>
#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Real-Time IO device API for moving bytes around fast
 *
 * Each Real-Time IO device provides a way to read and/or write
 * blocks of memory allocated from an rtio_block_allocator given by
 * the application. The layout of the each block is identified in the
 * block metadata. Each device may provide functionality to safely encode
 * or decode blocks of memory.
 */

/**
 * @brief Block of memory
 *
 * This block of memory has important metadata commonly useful such as
 * layout and timestamps of when a read or write began and ended.
 * The allocated size in bytes and length of data stored in bytes are also
 * provided.
 */
struct rtio_block {
	/**
	 * @private
	 * Word spacing for usage in k_fifo
	 */
	void *fifo_reserved;

	/** Timestamp in cycles from k_cycle_get_32() marking when a read or
	 * write began
	 */
	u32_t begin_tstamp;

	/** Timestamp in cycles from k_cycle_get_32() marking when a read or
	 * write was complete
	 */
	u32_t end_tstamp;

	/** Byte layout designator given by each driver to specify
	 * the layout of the contained buffer
	 */
	u32_t layout;

	/** Length of data in this buffer. */
	size_t len;

	/** Amount of data this buffer can store. */
	size_t size;

	/** Start of the buffer data .*/
	u8_t *_data;
};

/**
 * @brief Initialize an rtio_block
 */
static inline void rtio_block_init(struct rtio_block *block, u8_t *data,
				   u32_t size)
{
	block->layout = 0;
	block->begin_tstamp = 0;
	block->end_tstamp = 0;
	block->len = 0;
	block->size = size;
	block->_data = data;
}

/**
 * @brief Begin a block read or write
 */
static inline void rtio_block_begin(struct rtio_block *block)
{
	block->len = 0;
	block->begin_tstamp = k_cycle_get_32();
}

/**
 * @brief End a block read or write.
 */
static inline void rtio_block_end(struct rtio_block *block)
{
	block->end_tstamp = k_cycle_get_32();
}


/**
 * @brief Unused number of bytes in the buffer
 */
static inline u16_t rtio_block_available(struct rtio_block *block)
{
	return block->size - block->len;
}

/**
 * @private
 * @brief Implement a push/pull function pair
 *
 * Implements a pair of functions for pushing or pulling a fixed size value
 * into an rtio block.
 */
#define Z_RTIO_BLOCK_PUSH_PULL_IMPL(_type)				\
	static inline int rtio_block_push_##_type(			\
		struct rtio_block *block, _type val)			\
	{								\
		if (rtio_block_available(block) >= sizeof(_type)) {	\
			memcpy(&block->_data[block->len], &val,		\
			       sizeof(_type));				\
			block->len += sizeof(_type);			\
			return 0;					\
		} else {						\
			return -ENOMEM;					\
		}							\
	}								\
									\
	static inline int rtio_block_pull_##_type(			\
		struct rtio_block *block, u16_t offset, _type * val)	\
	{								\
		if (block->len - offset >= sizeof(_type)) {		\
			memcpy(val, &block->_data[offset],		\
			       sizeof(_type));				\
			return offset + sizeof(_type);			\
		} else {						\
			return -EINVAL;					\
		}							\
	}

/**
 * @brief Push/pull a value in/out of the block
 *
 * Implementations for common data types you might wish to
 * store in a rtio_block
 */
Z_RTIO_BLOCK_PUSH_PULL_IMPL(u8_t);
Z_RTIO_BLOCK_PUSH_PULL_IMPL(u16_t);
Z_RTIO_BLOCK_PUSH_PULL_IMPL(u32_t);
Z_RTIO_BLOCK_PUSH_PULL_IMPL(u64_t);
Z_RTIO_BLOCK_PUSH_PULL_IMPL(s8_t);
Z_RTIO_BLOCK_PUSH_PULL_IMPL(s16_t);
Z_RTIO_BLOCK_PUSH_PULL_IMPL(s32_t);
Z_RTIO_BLOCK_PUSH_PULL_IMPL(s64_t);

struct rtio_block_allocator;

/**
 * @private
 * @brief Function definition for allocating rtio blocks from a given allocator
 */
typedef int (*rtio_block_alloc_t)(struct rtio_block_allocator *allocator,
				  struct rtio_block **block, size_t size,
				  u32_t timeout);

/**
 * @private
 * @brief Function definition for freeing rtio blocks
 */
typedef void (*rtio_block_free_t)(struct rtio_block_allocator *allocator,
				  struct rtio_block *block);

/**
 * @brief An rtio block allocator interface
 */
struct rtio_block_allocator {
	rtio_block_alloc_t alloc;
	rtio_block_free_t free;
};

/**
 * @brief An rtio mempool block allocator
 */
struct rtio_mempool_block_allocator {
	struct rtio_block_allocator allocator;
	struct k_mem_pool *mempool;
};

/**
 * @brief An rtio_block allocated from a mempool
 */
struct rtio_mempool_block {
	struct rtio_block block;
	struct k_mem_block_id id;
};

/**
 * @brief Allocate a block from a mempool
 */
static inline int rtio_mempool_block_alloc(
	struct rtio_block_allocator *allocator, struct rtio_block **block,
	size_t size, u32_t timeout)
{
	struct rtio_mempool_block_allocator *mempool_allocator =
		(struct rtio_mempool_block_allocator *)allocator;
	struct k_mem_block memblock;
	size_t block_size = size + sizeof(struct rtio_mempool_block);

	int res = k_mem_pool_alloc(mempool_allocator->mempool,
				   &memblock, block_size,
				   timeout);
	if (res == 0) {
		struct rtio_mempool_block *mempool_block =
			(struct rtio_mempool_block *)(memblock.data);
		u8_t *dataptr = (u8_t *)memblock.data + sizeof(*mempool_block);

		mempool_block->id = memblock.id;
		*block = (struct rtio_block *)mempool_block;
		rtio_block_init(*block, dataptr, size);
	}
	return res;
}

static inline void rtio_mempool_block_free(
	struct rtio_block_allocator *allocator,
	struct rtio_block *block)
{
	struct rtio_mempool_block *mempool_block =
		(struct rtio_mempool_block *)block;

	k_mem_pool_free_id(&mempool_block->id);
}

/**
 * @brief Define an rtio block mempool allocator
 */
#define RTIO_MEMPOOL_ALLOCATOR_DEFINE(_name, _minsz, _maxsz, _nmax)	\
	K_MEM_POOL_DEFINE(rtio_pool_mempool_##_name,			\
			  sizeof(struct rtio_mempool_block) + _minsz,	\
			  sizeof(struct rtio_mempool_block) + _maxsz,	\
			  _nmax,					\
			  4);						\
									\
	struct rtio_mempool_block_allocator _name = {			\
	.allocator = {							\
	.alloc = rtio_mempool_block_alloc,				\
	.free = rtio_mempool_block_free					\
},									\
	.mempool = &rtio_pool_mempool_##_name				\
}


/**
 * @brief Allocate a rtio_block from an rtio_pool with a timeout
 *
 * This call is not safe to do with a timeout other than K_NO_WAIT
 * in an interrupt handler.
 *
 * The allocator *must* have a memory layout equivalent to
 * struct rtio_block_allocator
 */
static inline int rtio_block_alloc(void *allocatorv,
				   struct rtio_block **block,
				   size_t size,
				   u32_t timeout)
{
	struct rtio_block_allocator *allocator =
		(struct rtio_block_allocator *)allocatorv;

	return allocator->alloc(allocator, block, size, timeout);

}

/**
 * @brief Free a rtio_block back to the allocator
 */
static inline void rtio_block_free(void *allocatorv, struct rtio_block *block)
{
	struct rtio_block_allocator *allocator =
		(struct rtio_block_allocator *)allocatorv;
	allocator->free(allocator, block);
}

/**
 * @brief Output config describing where and when to push a blocks
 *
 * Output config should at least have a valid allocator and k_fifo pointer given
 *
 * Drivers should make a best attempt at fulfilling the policy of when
 * to return and notify the caller the block is ready by pushing it
 * into the fifo.
 *
 * If the buffer is full before the time or the desired number of
 * items is fulfilled its likely an error on the authors part and should
 * be logged as a warning when possible. This is a buffer overrun
 * scenario on the reader side.
 *
 * The behavior when the time is K_FOREVER and number is 0 the buffer
 * will be marked completed immediately and no read or write will take place.
 */
struct rtio_output_config {
	/**
	 * Allocator for rtio_blocks
	 */
	struct rtio_allocator *allocator;

	/**
	 * Fifo to output blocks to
	 */
	struct k_fifo *fifo;

	/**
	 *@brief Timeout in cycles to read
	 *
	 * The timeout is
	 *
	 * Should be set using something like K_MSEC(5) or K_FOREVER
	 */
	u32_t timeout;

	/** The number of bytes to read before making the block ready
	 * Note that the byte size should be computed by the driver from
	 * the size of the sample set multiplied by some number of samples
	 */
	size_t byte_size;
};

/**
 * @brief Common trigger sources
 */
enum rtio_trigger_source {
	GPIO_IRQ,
	COUNTER_IRQ,
	FUNCTION_CALL,

	/* All driver specific sources should use this plus an additive value
	 * to label the trigger source
	 */
	RTIO_TRIGGER_SOURCE_COUNT
};

/**
 * @brief Trigger source configuration
 *
 * The source of the trigger may be an internal event along with the
 * associated GPIO device and pin, a counter timeout, or a function call
 */
struct rtio_trigger_config {
	enum rtio_trigger_source source;
	union {
		struct rtio_trigger_gpio {
			struct device *gpio;
			u32_t pin;
		} gpio_config;
		struct rtio_trigger_counter {
			struct device *counter;
			u32_t timeout;
		} counter_config;
	} options;
};

/**
 * @brief Check if a block has met an output policy expectations
 *
 * @return true if the policy has been met, false otherwise
 */
static inline bool
rtio_output_policy_check(struct rtio_output_config *cfg,
			 struct rtio_block *block)
{
	if (k_cycle_get_32() - block->begin_tstamp > cfg->time) {
		return true;
	}
	if (block->len > cfg->byte_size) {
		return true;
	}
	return false;
}

/**
 * @brief A rtio configuration
 */
struct rtio_config {
	/** output configuration if applicable */
	struct rtio_output_config *output_config;

	/** trigger configuration if applicable */
	struct rtio_trigger_config *trigger_config;

	/** driver specific configuration */
	void *driver_config;
};

/**
 * @private
 * @brief Function definition for configuring a RTIO device
 */
typedef int (*rtio_configure_t)(struct device *dev,
				struct rtio_config *config);

/**
 * @private
 * @brief Function definition for triggering a device read or write
 */
typedef int (*rtio_trigger_t)(struct device *dev);

/**
 * @brief Real-Time IO API
 */
struct rtio_api {
	/** Configuration function pointer *must* be implemented */
	rtio_configure_t configure;

	/** Trigger function pointer *must* be implemented and IRQ safe */
	rtio_trigger_t trigger;
};

/**
 * @brief Configure the device
 *
 * This will reconfigure the device given a device specific configuration
 * structure and possibly updates the block layout. This call will wait for
 * any pending IO operations to be completed and prevent further IO operations
 * until complete.
 *
 * Layout is assigned to the new buffer byte layout for the rtio device.
 * Its expected the caller understands the new byte layout.
 *
 * @param dev RTIO device to configure
 * @param config Configuration settings
 */
__syscall int rtio_configure(struct device *dev,
			     struct rtio_config *config);

static inline int z_impl_rtio_configure(struct device *dev,
					struct rtio_config *config)
{
	const struct rtio_api *api = dev->driver_api;

	return api->configure(dev, config);
}

/**
 * @brief Trigger a device read or write
 *
 * Triggers a read or write to be done by the device.
 *
 * If the configuration specifies a GPIO or timer trigger
 * then the driver should setup an appropriate interrupt
 * call that in turns calls rtio_trigger on the device.
 *
 * The cause of the GPIO trigger may further be defined by the
 * driver config. For example a data ready or fifo full level
 * or edge trigger would be very common scenarios.
 *
 * @param dev RTIO device to trigger
 */
__syscall int rtio_trigger(struct device *dev);

static inline int z_impl_rtio_trigger(struct device *dev)
{
	const struct rtio_api *api = dev->driver_api;

	return api->trigger(dev);
}

/**
 * @brief Driver specific data
 *
 * This is to be used by drivers in their data struct if they wish to use
 * take advantage of the common rtio functionality for configuration
 * of triggers.
 */
struct rtio_driver_data {
	/**
	 * @brief Semaphore to be used by configure and trigger
	 *
	 * This is used to avoid interrupts causing a rtio_trigger
	 * manipulating data while the device is reconfiguring. This provides
	 * the atomicity of configure for the device from the applications
	 * perspective.
	 *
	 * This is taken without blocking when rtio_begin_trigger()
	 * is called, and taken waiting forever when rtio_begin_configuration()
	 * is called.
	 */
	struct k_sem sem;

	/**
	 * @brief The current RTIO configuration for the device
	 *
	 * This is copied in whenever rtio_begin_configuration() is called
	 */
	struct rtio_config config;

	/**
	 * @brief The current block being filled
	 */
	struct rtio_block *block;

	/**
	 * @brief Timer if needed by the output policy
	 */
	struct k_timer output_timer;
};


/**
 * @brief Begin configuring the device
 *
 * This must *not* be called in an ISR as it waits until any currently executing
 * trigger functions complete.
 *
 * rtio_end_configuration *must* be called afterwards when configuration is
 * done.
 *
 * @returns 0 on success, -
 */
int rtio_begin_configuration(struct device *dev, struct rtio_drv_data *data);

/**
 * @brief End configuring the device
 *
 * This will give back the semaphore allowing trigger to execute.
 */
void rtio_end_configuration(struct device *dev, struct rtio_drv_data *data);

/**
 * @brief Begin trigger call
 *
 * This may be called in any context that can take a semaphore without
 * blocking, including an ISR.
 *
 * If the current block is NULL it will attempt to allocate a new one.
 * The size of the block is given by rtio_drv_data block_alloc_size;
 * Not all allocators take size into account.
 *
 * Returns the current block to be filled on success.
 *
 * On success rtio_end_trigger *must* be called afterwards when done.
 *
 * @returns 0 on success
 * -EBUSY if not possible to do without blocking
 * -ENOMEM if not possible because allocation isn't possible.
 */
int rtio_begin_trigger(struct device *dev, struct rtio_drv_data *data,
		       struct rtio_block **block);

/**
 * @brief End trigger call
 *
 * This will give back the semaphore allowing configuration or the next trigger
 * to execute.
 *
 * It will also put the current block into the output fifo if the output policy
 * has been met.
 *
 * The current block in rtio_drv_data may be NULL after this call. This is
 * to avoid attempting allocation in this call, which would make it fallible.
 */
void rtio_end_trigger(struct device *dev, struct rtio_drv_data *data);


/**
 * @brief Output timeout call, used by rtio when the timeout expires on the
 * output block.
 *
 * The tmr *must* point to the timer struct in rtio_drv_data. The address of the
 * timer is used to obtain a pointer to struct rtio_drv_data.
 */
void rtio_output_timeout(struct k_timer *tmr);

/**
 * @brief Put the current block into the fifo if the output policy has been met
 *
 * This sets the current block to NULL if the block is put into the FIFO.
 *
 * @returns true if block is put into fifo, false otherwise
 */
static inline bool rtio_output_block(struct rtio_drv_data *data)
{
	struct rtio_output_config *out_cfg = &data->output_config;
	if(rtio_output_policy_check(out_cfg, data->block)) {
		k_fifo_put(out_cfg->fifo, data->block);
		data->block = NULL;
		return true;
	}
	return false;
}


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <syscalls/rtio.h>

#endif
