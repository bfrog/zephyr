/*
 * Copyright (c) 2018 Workaround GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief LP5521 LED driver
 *
 * The LP5521 is a 3-channel LED driver that communicates over I2C. The three 
 * channels are expected to be connected to a red, green, and blue LED.
 * Each LED can be driven by two different sources.
 *
 * 1. The brightness of each LED can be configured directly by setting a
 * register that drives the PWM of the connected LED.
 *
 * 2. A program for each channel can be transferred to the driver and run.
 * Up to 16 commands can be defined in each program. Possible commands are:
 *   - Set the brightness.
 *   - Fade in/out the brightness over time.
 *   - Loop parts of the program or the whole program.
 *   - Add delays.
 *   - Synchronize between the engines.
 *
 * After the program has been transferred, it can run infinitely without
 * communication between the host MCU and the driver.
 */

#include <i2c.h>
#include <led.h>
#include <device.h>
#include <zephyr.h>

#ifdef DT_TI_LP5521_ENABLE_GPIOS
#include <gpio.h>
#endif

#define LOG_LEVEL CONFIG_LED_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(lp5562);


#include "led_context.h"

/* Registers */
#define LP5521_ENABLE             0x00
#define LP5521_OP_MODE            0x01
#define LP5521_R_PWM              0x02
#define LP5521_G_PWM              0x03
#define LP5521_B_PWM              0x04
#define LP5521_R_CURRENT          0x05
#define LP5521_G_CURRENT          0x06
#define LP5521_B_CURRENT          0x07
#define LP5521_CONFIG             0x08
#define LP5521_R_PC               0x09
#define LP5521_G_PC               0x0A
#define LP5521_B_PC               0x0B
#define LP5521_STATUS             0x0C
#define LP5521_RESET              0x0D
#define lp5521_GPO                0x0E
#define LP5521_PROG_MEM_R_BASE    0x10
#define LP5521_PROG_MEM_G_BASE    0x30
#define LP5521_PROG_MEM_B_BASE    0x50
#define LP5521_LED_MAP            0x70

/*
 * The wait command has six bits for the number of steps (max 63) with up to
 * 15.6ms per step if the prescaler is set to 1. We round the step length
 * however to 16ms for easier handliung, so the maximum blinking period is
 * therefore (16 * 63) = 1008ms. We round it down to 1000ms to be on the safe
 * side.
 */
#define LP5521_MAX_BLINK_PERIOD K_MSEC(1000)
/*
 * The minimum waiting period is 0.49ms with the prescaler set to 0 and one
 * step. We round up to a full millisecond.
 */
#define LP5521_MIN_BLINK_PERIOD K_MSEC(1)

/* Brightness limits in percent */
#define LP5521_MIN_BRIGHTNESS 0
#define LP5521_MAX_BRIGHTNESS 100

/* Values for ENABLE register. */
#define LP5521_ENABLE_CHIP_EN (1 << 6)
#define LP5521_ENABLE_LOG_EN  (1 << 7)

/* Values for CONFIG register. */
#define LP5521_CONFIG_EXTERNAL_CLOCK         0x00
#define LP5521_CONFIG_INTERNAL_CLOCK         0x01
#define LP5521_CONFIG_CLOCK_AUTOMATIC_SELECT 0x02
#define LP5521_CONFIG_PWRSAVE_EN             (1 << 5)
/* Enable 558 Hz frequency for PWM. Default is 256. */
#define LP5521_CONFIG_PWM_HW_FREQ_558        (1 << 6)

/* Values for execution engine programs. */
#define LP5521_PROG_COMMAND_SET_PWM (1 << 6)
#define LP5521_PROG_COMMAND_RAMP_TIME(prescale, step_time) \
	(((prescale) << 6) | (step_time))
#define LP5521_PROG_COMMAND_STEP_COUNT(fade_direction, count) \
	(((fade_direction) << 7) | (count))

/* Helper definitions. */
#define LP5521_PROG_MAX_COMMANDS 16
#define LP5521_MASK              0x03
#define LP5521_CHANNEL_MASK(channel) ((LP5521_MASK) << (channel << 1))

/*
 * Available channels. There are four LED channels usable with the LP5521. While
 * they can be mapped to LEDs of any color, the driver's typical application is
 * with a red, a green, a blue and a white LED. Since the data sheet's
 * nomenclature uses RGBW, we keep it that way.
 */
enum lp5521_led_channels {
	LP5521_CHANNEL_B,
	LP5521_CHANNEL_G,
	LP5521_CHANNEL_R,
	
    LP5521_CHANNEL_COUNT,
};

/* Operational modes of the execution engines. */
enum lp5521_engine_op_modes {
	LP5521_OP_MODE_DISABLED = 0x00,
	LP5521_OP_MODE_LOAD = 0x01,
	LP5521_OP_MODE_RUN = 0x02,
	LP5521_OP_MODE_DIRECT_CTRL = 0x03,
};

/* Execution state of the engines. */
enum lp5521_engine_exec_states {
	LP5521_ENGINE_MODE_HOLD = 0x00,
	LP5521_ENGINE_MODE_STEP = 0x01,
	LP5521_ENGINE_MODE_RUN = 0x02,
	LP5521_ENGINE_MODE_EXEC = 0x03,
};

/* Fading directions for programs executed by the engines. */
enum lp5521_engine_fade_dirs {
	LP5521_FADE_UP = 0x00,
	LP5521_FADE_DOWN = 0x01,
};

struct lp5521_data {
	struct device *i2c;
#ifdef DT_TI_LP5521_0_ENABLE_GPIOS
    struct device *gpio;
#endif
	struct led_data dev_data;
};

/*
 * @brief Get the register for the given LED channel used to directly write a
 *	brightness value instead of using the execution engines.
 *
 * @param channel LED channel.
 * @param reg     Pointer to the register address.
 *
 * @retval 0       On success.
 * @retval -EINVAL If an invalid channel is given.
 */
static int lp5521_get_pwm_reg(enum lp5521_led_channels channel, u8_t *reg)
{
	switch (channel) {
	case LP5521_CHANNEL_R:
		*reg = LP5521_R_PWM;
		break;
	case LP5521_CHANNEL_G:
		*reg = LP5521_G_PWM;
		break;
	case LP5521_CHANNEL_B:
		*reg = LP5521_B_PWM;
		break;
	default:
		LOG_ERR("Invalid channel given.");
		return -EINVAL;
	}

	return 0;
}

/*
 * @brief Helper to get the register bit shift for the channels.
 *
 * The channel with the highest index is placed on the lowest two bits in the
 * OP_MODE and ENABLE registers.
 *
 * @param channel Channel the shift is requested for.
 * @param shift  Pointer to the shift value.
 *
 * @retval 0       On success.
 * @retval -EINVAL If a source is given that is not a valid engine.
 */
static int lp5521_get_engine_reg_shift(enum lp5521_led_channels channel,
				       u8_t *shift)
{
	switch (channel) {
	case LP5521_CHANNEL_R:
		*shift = 4;
		break;
	case LP5521_CHANNEL_G:
		*shift = 2;
		break;
	case LP5521_CHANNEL_B:
		*shift = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * @brief Get the base address for programs of the given execution engine.
 *
 * @param engine    Engine the base address is requested for.
 * @param base_addr Pointer to the base address.
 *
 * @retval 0       On success.
 * @retval -EINVAL If a source is given that is not a valid engine.
 */
static int lp5521_get_engine_ram_base_addr(enum lp5521_led_channels channel,
					   u8_t *base_addr)
{
	switch (channel) {
	case LP5521_CHANNEL_R:
		*base_addr = LP5521_PROG_MEM_R_BASE;
		break;
	case LP5521_CHANNEL_G:
		*base_addr = LP5521_PROG_MEM_G_BASE;
		break;
	case LP5521_CHANNEL_B:
		*base_addr = LP5521_PROG_MEM_B_BASE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * @brief Convert a time in milliseconds to a combination of prescale and
 *	step_time for the execution engine programs.
 *
 * This function expects the given time in milliseconds to be in the allowed
 * range the device can handle (0ms to 1000ms).
 *
 * @param data      Capabilities of the driver.
 * @param ms        Time to be converted in milliseconds [0..1000].
 * @param prescale  Pointer to the prescale value.
 * @param step_time Pointer to the step_time value.
 */
static void lp5521_ms_to_prescale_and_step(struct led_data *data, u32_t ms,
					   u8_t *prescale, u8_t *step_time)
{
	/*
	 * One step with the prescaler set to 0 takes 0.49ms. The max value for
	 * step_time is 63, so we just double the millisecond value. That way
	 * the step_time value never goes above the allowed 63.
	 */
	if (ms < 31) {
		*prescale = 0;
		*step_time = ms << 1;

		return;
	}

	/*
	 * With a prescaler value set to 1 one step takes 15.6ms. So by dividing
	 * through 16 we get a decent enough result with low effort.
	 */
	*prescale = 1;
	*step_time = ms >> 4;

	return;
}

/*
 * @brief Request the current channel operational mode
 *
 * @param dev     LP5521 device.
 * @param channel Channel to check.
 * @param op_mode Operational mode to return
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static int lp5521_get_channel_op_mode(struct device *dev,
        enum lp5521_led_channels channel,
        enum lp5521_engine_op_modes *op_mode)
{
    struct lp5521_data *data = dev->driver_data;
	u8_t shift, reg;
	int ret;

	ret = lp5521_get_engine_reg_shift(channel, &shift);
	if (ret) {
		return ret;
	}

	if (i2c_reg_read_byte(data->i2c, DT_TI_LP5521_0_BASE_ADDRESS,
				LP5521_OP_MODE, &reg)) {
		LOG_ERR("Failed to read op mode register.");
		return -EIO;
	}

    *op_mode = reg >> shift & LP5521_MASK;

	return 0;
}

/*
 * @brief Request whether an engine for a channel is currently running.
 *
 * @param dev    LP5521 device.
 * @param channel Channel to check.
 *
 * @return Indication of the engine execution state.
 *
 * @retval true  If the engine is currently running.
 * @retval false If the engine is not running or an error occurred.
 */
static bool lp5521_is_engine_executing(struct device *dev,
				       enum lp5521_led_channels channel)
{
	struct lp5521_data *data = dev->driver_data;
	u8_t enabled, shift;
	int ret;

	ret = lp5521_get_engine_reg_shift(channel, &shift);
	if (ret) {
		return false;
	}

	if (i2c_reg_read_byte(data->i2c, DT_TI_LP5521_0_BASE_ADDRESS,
				LP5521_ENABLE, &enabled)) {
		LOG_ERR("Failed to read ENABLE register.");
		return false;
	}

	enabled = (enabled >> shift) & LP5521_MASK;

	if (enabled == LP5521_ENGINE_MODE_RUN) {
		return true;
	}

	return false;
}

/*
 * @brief Set an register shifted for the given execution engine.
 *
 * @param dev    LP5521 device.
 * @param channel Channel the value is shifted for.
 * @param reg    Register address to set.
 * @param val    Value to set.
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static int lp5521_set_engine_reg(struct device *dev,
				 enum lp5521_led_channels channel,
				 u8_t reg, u8_t val)
{
	struct lp5521_data *data = dev->driver_data;
	u8_t shift;
	int ret;

	ret = lp5521_get_engine_reg_shift(channel, &shift);
	if (ret) {
		return ret;
	}

	if (i2c_reg_update_byte(data->i2c, DT_TI_LP5521_0_BASE_ADDRESS,
				   reg,
				   LP5521_MASK << shift,
				   val << shift)) {
		return -EIO;
	}

	return 0;
}

/*
 * @brief Set the operational mode of the given engine.
 *
 * @param dev    LP5521 device.
 * @param channel Channel the engine operational mode is changed for.
 * @param mode   Mode to set.
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static inline int lp5521_set_engine_op_mode(struct device *dev,
					    enum lp5521_led_channels channel,
					    enum lp5521_engine_op_modes mode)
{
	return lp5521_set_engine_reg(dev, channel, LP5521_OP_MODE, mode);
}

/*
 * @brief Set the execution state of the given engine.
 *
 * @param dev    LP5521 device.
 * @param channel Channel the engine execution state is changed for.
 * @param state  State to set.
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static inline int lp5521_set_engine_exec_state(struct device *dev,
					enum lp5521_led_channels channel,
					enum lp5521_engine_exec_states state)
{
	return lp5521_set_engine_reg(dev, channel, LP5521_ENABLE, state);
}

/*
 * @brief Start the execution of the program of the given engine.
 *
 * @param dev    LP5521 device.
 * @param channel Channel engine that is started.
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static inline int lp5521_start_program_exec(struct device *dev,
					    enum lp5521_led_channels channel)
{
	if (lp5521_set_engine_op_mode(dev, channel, LP5521_OP_MODE_RUN)) {
		return -EIO;
	}

	return lp5521_set_engine_exec_state(dev, channel,
					    LP5521_ENGINE_MODE_RUN);

}

/*
 * @brief Stop the execution of the program of the given engine.
 *
 * @param dev    LP5521 device.
 * @param channel Channel engine that is stopped.
 *
 * @retval 0    On success.
 * @retval -EIO If the underlying I2C call fails.
 */
static inline int lp5521_stop_program_exec(struct device *dev,
					   enum lp5521_led_channels channel)
{
	if (lp5521_set_engine_op_mode(dev, channel, LP5521_OP_MODE_DIRECT_CTRL)) {
		return -EIO;
	}

	return lp5521_set_engine_exec_state(dev, channel,
					    LP5521_ENGINE_MODE_HOLD);
}

/*
 * @brief Program a command to the memory of the given execution engine.
 *
 * @param dev           LP5521 device.
 * @param channel       Channel engine that is programmed.
 * @param command_index Index of the command that is programmed.
 * @param command_msb   Most significant byte of the command.
 * @param command_lsb   Least significant byte of the command.
 *
 * @retval 0       On success.
 * @retval -EINVAL If the given command index is out of range or an invalid
 *		   channel is passed.
 * @retval -EIO    If the underlying I2C call fails.
 */
static int lp5521_program_command(struct device *dev,
				  enum lp5521_led_channels channel,
				  u8_t command_index,
				  u8_t command_msb,
				  u8_t command_lsb)
{
	struct lp5521_data *data = dev->driver_data;
	u8_t prog_base_addr;
	int ret;

	if (command_index >= LP5521_PROG_MAX_COMMANDS) {
		return -EINVAL;
	}

	ret = lp5521_get_engine_ram_base_addr(channel, &prog_base_addr);
	if (ret) {
		LOG_ERR("Failed to get base RAM address.");
		return ret;
	}

	if (i2c_reg_write_byte(data->i2c, DT_TI_LP5521_0_BASE_ADDRESS,
			       prog_base_addr + (command_index << 1),
			       command_msb)) {
		LOG_ERR("Failed to update LED.");
		return -EIO;
	}

	if (i2c_reg_write_byte(data->i2c, DT_TI_LP5521_0_BASE_ADDRESS,
			       prog_base_addr + (command_index << 1) + 1,
			       command_lsb)) {
		LOG_ERR("Failed to update LED.");
		return -EIO;
	}

	return 0;
}

/*
 * @brief Program a command to set a fixed brightness to the given engine.
 *
 * @param dev           LP5521 device.
 * @param channel       Channel engine to be programmed.
 * @param command_index Index of the command in the program sequence.
 * @param brightness    Brightness to be set for the LED in percent.
 *
 * @retval 0       On success.
 * @retval -EINVAL If the passed arguments are invalid or out of range.
 * @retval -EIO    If the underlying I2C call fails.
 */
static int lp5521_program_set_brightness(struct device *dev,
					 enum lp5521_led_channels channel,
					 u8_t command_index,
					 u8_t brightness)
{
	struct lp5521_data *data = dev->driver_data;
	struct led_data *dev_data = &data->dev_data;
	u8_t val;

	if ((brightness < dev_data->min_brightness) ||
			(brightness > dev_data->max_brightness)) {
		return -EINVAL;
	}

	val = (brightness * 0xFF) / dev_data->max_brightness;

	return lp5521_program_command(dev, channel, command_index,
			LP5521_PROG_COMMAND_SET_PWM, val);
}

/*
 * @brief Program a command to ramp the brightness over time.
 *
 * In each step the PWM value is increased or decreased by 1/255th until the
 * maximum or minimum value is reached or step_count steps have been done.
 *
 * @param dev           LP5521 device.
 * @param channel       Channel engine to be programmed.
 * @param command_index Index of the command in the program sequence.
 * @param time_per_step Time each step takes in milliseconds.
 * @param step_count    Number of steps to perform.
 * @param fade_dir      Direction of the ramp (in-/decrease brightness).
 *
 * @retval 0       On success.
 * @retval -EINVAL If the passed arguments are invalid or out of range.
 * @retval -EIO    If the underlying I2C call fails.
 */
static int lp5521_program_ramp(struct device *dev,
			       enum lp5521_led_channels channel,
			       u8_t command_index,
			       u32_t time_per_step,
			       u8_t step_count,
			       enum lp5521_engine_fade_dirs fade_dir)
{
	struct lp5521_data *data = dev->driver_data;
	struct led_data *dev_data = &data->dev_data;
	u8_t prescale, step_time;

	if ((time_per_step < dev_data->min_period) ||
			(time_per_step > dev_data->max_period)) {
		return -EINVAL;
	}

	lp5521_ms_to_prescale_and_step(dev_data, time_per_step,
			&prescale, &step_time);

	return lp5521_program_command(dev, channel, command_index,
			LP5521_PROG_COMMAND_RAMP_TIME(prescale, step_time),
			LP5521_PROG_COMMAND_STEP_COUNT(fade_dir, step_count));
}

/*
 * @brief Program a command to do nothing for the given time.
 *
 * @param dev           LP5521 device.
 * @param channel       Channel engine to be programmed.
 * @param command_index Index of the command in the program sequence.
 * @param time          Time to do nothing in milliseconds.
 *
 * @retval 0       On success.
 * @retval -EINVAL If the passed arguments are invalid or out of range.
 * @retval -EIO    If the underlying I2C call fails.
 */
static inline int lp5521_program_wait(struct device *dev,
				      enum lp5521_led_channels channel,
				      u8_t command_index,
				      u32_t time)
{
	/*
	 * A wait command is a ramp with the step_count set to 0. The fading
	 * direction does not matter in this case.
	 */
	return lp5521_program_ramp(dev, channel, command_index,
			time, 0, LP5521_FADE_UP);
}

/*
 * @brief Program a command to go back to the beginning of the program.
 *
 * Can be used at the end of a program to loop it infinitely.
 *
 * @param dev           LP5521 device.
 * @param channel       Channel engine to be programmed.
 * @param command_index Index of the command in the program sequence.
 *
 * @retval 0       On success.
 * @retval -EINVAL If the given command index is out of range or an invalid
 *		   channel is passed.
 * @retval -EIO    If the underlying I2C call fails.
 */
static inline int lp5521_program_go_to_start(struct device *dev,
					     enum lp5521_led_channels channel,
					     u8_t command_index)
{
	return lp5521_program_command(dev, channel, command_index, 0x00, 0x00);
}

/*
 * @brief Change the brightness of a running blink program.
 *
 * We know that the current program executes a blinking pattern
 * consisting of following commands:
 *
 * - set_brightness high
 * - wait on_delay
 * - set_brightness low
 * - wait off_delay
 * - return to start
 *
 * In order to change the brightness during blinking, we overwrite only
 * the first command and start execution again.
 *
 * @param dev           LP5521 device.
 * @param channel       Channel engine running the blinking program.
 * @param brightness_on New brightness value.
 *
 * @retval 0       On Success.
 * @retval -EINVAL If the channel ID or brightness is out of range.
 * @retval -EIO    If the underlying I2C call fails.
 */
static int lp5521_update_blinking_brightness(struct device *dev,
					     enum lp5521_led_channels channel,
					     u8_t brightness_on)
{
	int ret;

	ret = lp5521_stop_program_exec(dev, channel);
	if (ret) {
		return ret;
	}

	ret = lp5521_set_engine_op_mode(dev, channel, LP5521_OP_MODE_LOAD);
	if (ret) {
		return ret;
	}


	ret = lp5521_program_set_brightness(dev, channel, 0, brightness_on);
	if (ret) {
		return ret;
	}

	ret = lp5521_start_program_exec(dev, channel);
	if (ret) {
		LOG_ERR("Failed to execute program.");
		return ret;
	}

	return 0;
}

static int lp5521_led_blink(struct device *dev, u32_t led,
			    u32_t delay_on, u32_t delay_off)
{
	int ret;
	enum lp5521_led_channels channel = led;
	u8_t command_index = 0;

    if(led >= LP5521_CHANNEL_COUNT) {
        return -EINVAL;
    }

	ret = lp5521_set_engine_op_mode(dev, channel, LP5521_OP_MODE_LOAD);
	if (ret) {
		return ret;
	}

    ret = lp5521_program_ramp(dev, channel, command_index, 
            10, 50, LP5521_FADE_UP);
	if (ret) {
		return ret;
	}

	ret = lp5521_program_wait(dev, channel, ++command_index, delay_on);
	if (ret) {
		return ret;
	}

    ret = lp5521_program_ramp(dev, channel, ++command_index, 
            10, 50, LP5521_FADE_DOWN);
	if (ret) {
		return ret;
	}

	ret = lp5521_program_wait(dev, channel, ++command_index, delay_off);
	if (ret) {
		return ret;
	}

	ret = lp5521_program_go_to_start(dev, channel, ++command_index);
	if (ret) {
		return ret;
	}

	ret = lp5521_start_program_exec(dev, channel);
	if (ret) {
		LOG_ERR("Failed to execute program.");
		return ret;
	}

	return 0;
}

static int lp5521_led_set_brightness(struct device *dev, u32_t led, u8_t value)
{
	struct lp5521_data *data = dev->driver_data;
	struct led_data *dev_data = &data->dev_data;
	int ret;
	u8_t val, reg;
	enum lp5521_led_channels channel = led;

	if ((value < dev_data->min_brightness) ||
			(value > dev_data->max_brightness)) {
		return -EINVAL;
	}

    if(led >= LP5521_CHANNEL_COUNT) {
        return -EINVAL;
    }


	if (lp5521_is_engine_executing(dev, channel)) {
        /*
         * LED is blinking currently. Restart the blinking with
         * the passed brightness.
         */
        return lp5521_update_blinking_brightness(dev,
                channel, value);
	}

	val = (value * 0xFF) / dev_data->max_brightness;

	ret = lp5521_get_pwm_reg(channel, &reg);
	if (ret) {
		return ret;
	}

	if (i2c_reg_write_byte(data->i2c, DT_TI_LP5521_0_BASE_ADDRESS,
			       reg, val)) {
		LOG_ERR("LED write failed");
		return -EIO;
	}

	return 0;
}

static inline int lp5521_led_on(struct device *dev, u32_t led)
{
	struct lp5521_data *data = dev->driver_data;
	struct led_data *dev_data = &data->dev_data;

    if(led >= LP5521_CHANNEL_COUNT) {
        return -EINVAL;
    }


	return lp5521_led_set_brightness(dev, led, dev_data->max_brightness);
}

static inline int lp5521_led_off(struct device *dev, u32_t led)
{
	struct lp5521_data *data = dev->driver_data;
	struct led_data *dev_data = &data->dev_data;

	int ret;

    u8_t op_mode;

    if(led >= LP5521_CHANNEL_COUNT) {
        return -EINVAL;
    }

     
	ret = lp5521_get_channel_op_mode(dev, led, &op_mode);
    if (ret) {
        LOG_ERR("Failed to read channel op mode.");
        return -EIO;
    }
    if(op_mode == LP5521_OP_MODE_RUN) {
		ret = lp5521_stop_program_exec(dev, led);
		if (ret) {
			return ret;
		}
	}

	return lp5521_led_set_brightness(dev, led, dev_data->min_brightness);
}

static int lp5521_led_init(struct device *dev)
{
	struct lp5521_data *data = dev->driver_data;
	struct led_data *dev_data = &data->dev_data;

#if defined(DT_TI_LP5521_0_ENABLE_GPIOS)
	data->gpio = device_get_binding(DT_TI_LP5521_0_ENABLE_GPIOS_CONTROLLER);
	if (data->gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device!",
				DT_TI_LP5521_0_ENABLE_GPIOS_CONTROLLER);
		return -EINVAL;
	}
	gpio_pin_configure(data->gpio, DT_TI_LP5521_0_ENABLE_GPIOS_PIN,
			   GPIO_DIR_OUT);
	gpio_pin_write(data->gpio, DT_TI_LP5521_0_ENABLE_GPIOS_PIN, 1);

	k_sleep(1);
#endif

	data->i2c = device_get_binding(DT_TI_LP5521_0_BUS_NAME);
	if (data->i2c == NULL) {
		LOG_ERR("Failed to get I2C device");
		return -EINVAL;
	}

	/* Hardware specific limits */
	dev_data->min_period = LP5521_MIN_BLINK_PERIOD;
	dev_data->max_period = LP5521_MAX_BLINK_PERIOD;
	dev_data->min_brightness = LP5521_MIN_BRIGHTNESS;
	dev_data->max_brightness = LP5521_MAX_BRIGHTNESS;

	if (i2c_reg_write_byte(data->i2c, DT_TI_LP5521_0_BASE_ADDRESS,
				LP5521_ENABLE,
				LP5521_ENABLE_CHIP_EN)) {
		LOG_ERR("Enabling LP5521 LED chip failed.");
		return -EIO;
	}

    k_sleep(1);

	if (i2c_reg_write_byte(data->i2c, DT_TI_LP5521_0_BASE_ADDRESS,
				LP5521_CONFIG, 0x59)) {
		LOG_ERR("Configuring LP5521 LED chip failed.");
		return -EIO;
	}

	if (i2c_reg_write_byte(data->i2c, DT_TI_LP5521_0_BASE_ADDRESS,
				LP5521_OP_MODE, 0x3F)) {
		LOG_ERR("Disabling all engines failed.");
		return -EIO;
	}

    /*
    if(i2c_reg_write_byte(data->i2c, DT_TI_LP5521_0_BASE_ADDRESS,
                LP5521_R_PWM, 128)) {
        LOG_ERR("Failed to enable red channel");
        return -EIO;
    }

    if(i2c_reg_write_byte(data->i2c, DT_TI_LP5521_0_BASE_ADDRESS,
                LP5521_G_PWM, 192)) {
        LOG_ERR("Failed to enable green channel");
        return -EIO;
    }

    if(i2c_reg_write_byte(data->i2c, DT_TI_LP5521_0_BASE_ADDRESS,
                LP5521_B_PWM, 255)) {
        LOG_ERR("Failed to enable blue channel");
        return -EIO;
    }
    */


	return 0;
}

static struct lp5521_data lp5521_led_data;

static const struct led_driver_api lp5521_led_api = {
	.blink = lp5521_led_blink,
	.set_brightness = lp5521_led_set_brightness,
	.on = lp5521_led_on,
	.off = lp5521_led_off,
};

DEVICE_AND_API_INIT(lp5521_led, DT_TI_LP5521_0_LABEL,
		&lp5521_led_init, &lp5521_led_data,
		NULL, POST_KERNEL, CONFIG_LED_INIT_PRIORITY,
		&lp5521_led_api);
