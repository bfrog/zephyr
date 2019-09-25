#include "icm20649_regs.h"
#include "icm20649_data.h"

#include <spi.h>
#include <init.h>
#include <sensor.h>
#include <logging/log.h>

#define LOG_LEVEL CONFIG_RTIO_SENSOR_LOG_LEVEL
LOG_MODULE_REGISTER(ICM20649);

#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

#define SENSOR_PI_DOUBLE			(SENSOR_PI / 1000000.0)
#define SENSOR_DEG2RAD_DOUBLE			(SENSOR_PI_DOUBLE / 180)
#define SENSOR_G_DOUBLE				(SENSOR_G / 1000000.0)

static struct spi_cs_control icm20649_cs_ctrl;
static struct spi_config icm20649_spi_conf = {
	.frequency = DT_TDK_ICM20649_0_SPI_MAX_FREQUENCY,
	.operation = (SPI_OP_MODE_MASTER | SPI_MODE_CPOL |
		      SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE),
	.slave     = DT_TDK_ICM20649_0_BASE_ADDRESS,
	.cs = &icm20649_cs_ctrl,
};

#define ICM20649_MAX_SERIAL_READ 16

static int icm20649_spi_read(struct icm20649_data *data, u8_t reg,
			     u8_t *val, u8_t len)
{
	struct spi_config *spi_cfg = &icm20649_spi_conf;
	u8_t buffer_tx[2] = { reg_addr | ICM20649_SPI_READ, 0};

	const struct spi_buf tx_buf = {
			.buf = buffer_tx,
			.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1,
		},
		{
			.buf = value,
			.len = len,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};

	if (len > ICM20649_MAX_SERIAL_READ) {
		return -EIO;
	}

	if (spi_transceive(data->spi, spi_cfg, &tx, &rx)) {
		return -EIO;
	}

	return 0;
}

static int icm20649_spi_write(struct icm20649_data *data, u8_t reg_addr,
		u8_t *value, u8_t len)
{
	struct spi_config *spi_cfg = &icm20649_spi_conf;
	u8_t buffer_tx[1] = { reg_addr & ~ICM20649_SPI_READ };
	const struct spi_buf tx_buf[2] = {
		{
			.buf = buffer_tx,
			.len = 1,
		},
		{
			.buf = value,
			.len = len,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};

	if (len > 64) {
		return -EIO;
	}

	if (spi_write(data->spi, spi_cfg, &tx)) {
		return -EIO;
	}

	return 0;
}

static inline int icm20649_write_reg8(struct device *dev, u8_t addr, u8_t val) {
	struct icm20649_data *data = dev->driver_data;
	return icm20649_spi_write(data, addr, &val, 1);
}

static inline int icm20649_read_reg8(struct device *dev, u8_t addr, u8_t *val) {
	struct icm20649_data *data = dev->driver_data;
	return icm20649_spi_read(data, addr, val, 1);
}

static inline int icm20649_read_s16(struct device *dev, u8_t addr_l, u8_t addr_h, s16_t *val) {
	u8_t l, h = 0;
	int err = icm20649_read_reg8(dev, addr_l, &l);
	err |= icm20649_read_reg8(dev, addr_h, &h);
	*val = (s16_t)((u16_t)(l) |
				((u16_t)(h) << 8));
	return err;
}

static inline int icm20649_read_u16(struct device *dev, u8_t addr_h, u8_t addr_l, u16_t *val) {
	u8_t l, h = 0;
	int err = icm20649_read_reg8(dev, addr_h, &h);
	err |= icm20649_read_reg8(dev, addr_l, &l);
	*val = ((u16_t)(l) |
				((u16_t)(h) << 8));
	return err;
}

static inline int icm20649_update_reg8(struct device *dev, u8_t addr, u8_t mask,
		u8_t val) {
	struct icm20649_data *data = dev->driver_data;

	u8_t tmp_val;

	int ret = icm20649_spi_read(data, addr, &tmp_val, 1);
	tmp_val = (tmp_val & ~mask) | (val & mask);

	ret |= icm20649_raw_write(data, addr, &tmp_val, 1);

#ifdef CONFIG_ICM20649_VERIFY_WRITE
	u8_t new_val;
	ret |= icm20649_spi_read(data, addr, &new_val, 1);
	if(new_val != tmp_val) {
		LOG_WRN("failed to verify write for addr %x expected %x got %x", addr, tmp_val, new_val);
	}
#endif
	return ret;
}

static inline int icm20649_set_user_bank(struct device *dev, u8_t bank) {
	return icm20649_write_reg8(dev, ICM20649_REG_BANK_SEL,
			bank << ICM20649_SHIFT_BANK_SEL & ICM20649_MASK_BANK_SEL);
}

static inline int icm20649_set_reset(struct device *dev) {
	LOG_DBG("reset");
	int ret = icm20649_update_reg8(dev, ICM20649_REG_PWR_MGMT_1,
			ICM20649_MASK_PWR_MGMT_1_DEVICE_RESET,
			(1 << ICM20649_SHIFT_PWR_MGMT_1_DEVICE_RESET));
	LOG_DBG("reset done");
	return ret;
}

static inline int icm20649_set_clock(struct device *dev) {
	LOG_DBG("set clock");
	int ret = icm20649_update_reg8(dev, ICM20649_REG_PWR_MGMT_1,
			ICM20649_MASK_PWR_MGMT_1_CLKSEL,
			(1 << ICM20649_SHIFT_PWR_MGMT_1_CLKSEL));
	LOG_DBG("set clock done");
	return ret;
}

static inline int icm20649_set_sleep(struct device *dev, bool sleep) {
	return icm20649_update_reg8(dev, ICM20649_REG_PWR_MGMT_1,
				    ICM20649_MASK_PWR_MGMT_1_SLEEP,
				    sleep << ICM20649_SHIFT_PWR_MGMT_1_SLEEP);
}

static inline int icm20649_set_user_ctrl(struct device *dev,
					 struct icm20649_config *cfg) {
	return icm20649_update_reg8(dev, ICM20649_REG_USER_CTRL,
			ICM20649_MASK_USER_CTRL_DMP_EN |
			ICM20649_MASK_USER_CTRL_FIFO_EN |
			ICM20649_MASK_USER_CTRL_I2C_IF_DIS ,
			0 << ICM20649_SHIFT_USER_CTRL_DMP_EN |
			cfg->fifo_enabled << ICM20649_SHIFT_USER_CTRL_FIFO_EN |
			0 << ICM20649_SHIFT_USER_CTRL_I2C_IF_DIS
			);
}


static inline int icm20649_set_accel_smplrt_div_1(struct device *dev,
						  struct icm20649_config *cfg)
{

	return icm20649_write_reg8(dev, ICM20649_REG_ACCEL_SMPLRT_DIV_1,
				   (u8_t)(cfg->accel->sample_rate_div >> 8 & ICM20649_MASK_ACCEL_SMPLRT_DIV_1));
}

static inline int icm20649_set_accel_smplrt_div_2(struct device *dev,
						  struct icm20649_config *cfg)
{
	return icm20649_write_reg8(dev, ICM20649_REG_ACCEL_SMPLRT_DIV_2,
				   (u8_t)(cfg->accel->sample_rate_div & 0x00FF));
}

static inline int icm20649_set_accel_config(struct device *dev,
		struct icm20649_config *cfg)
{
	u8_t mask = ICM20649_MASK_GYRO_CONFIG_1_GYRO_FCHOICE
		| ICM20649_MASK_GYRO_CONFIG_1_GYRO_DLPFCFG
		| ICM20649_MASK_GYRO_CONFIG_1_GYRO_FS_SEL;
	u8_t fchoice = 0;
	u8_t dlpfcfg = 0;
	if(cfg->gyro->filter != ICM20649_GYRO_LPF_DISABLED) {
		fchoice = 1 << ICM20649_SHIFT_GYRO_CONFIG_1_GYRO_FCHOICE;
		dlpfcfg = cfg->gyro->filter << ICM20649_SHIFT_GYRO_CONFIG_1_GYRO_DLPFCFG;
	}
	u8_t fs = cfg->gyro->scale << ICM20649_SHIFT_GYRO_CONFIG_1_GYRO_FS_SEL;
	u8_t val = fchoice | dlpfcfg | fs;
	return icm20649_update_reg8(dev, ICM20649_REG_GYRO_CONFIG_1,
			mask,
			val);
}

static inline int icm20649_set_accel_lpf(struct device *dev, u8_t lpf_mode) {

}


static inline int icm20649_set_gyro_sample_rate_div(struct device *dev,
		u8_t rate_div) {
	return icm20649_write_reg8(dev, ICM20649_REG_GYRO_SMPLRT_DIV, rate_div);
}

static int icm20649_set_odr_align(struct device *dev, u8_t align) {
	return icm20649_update_reg8(dev, ICM20649_REG_ODR_ALIGN_EN,
			ICM20649_MASK_ODR_ALIGN_EN,
			align << ICM20649_SHIFT_ODR_ALIGN_EN);
}

static inline int icm20649_get_who_am_i(struct device *dev, u8_t *whoami) {
	return icm20649_read_reg8(dev, ICM20649_REG_WHO_AM_I, whoami);
}

static int icm20649_check_who_am_i(struct device *dev) {
	u8_t chip_id;

	if(icm20649_set_user_bank(dev, 0) < 0) {
		LOG_ERR("failed to set user bank to 0");
		return -EIO;
	}

	if(icm20649_get_who_am_i(dev, &chip_id) < 0) {
		LOG_ERR("failed reading chip id");
		return -EIO;
	}

	if(chip_id != ICM20649_VAL_WHO_AM_I) {
		LOG_ERR("invalid chip id 0x%x", chip_id);
		return -EIO;
	}
	return 0;
}

static int icm20649_reset(struct device *dev) {
	if(icm20649_set_user_bank(dev, 0) < 0) {
		LOG_ERR("failed to set user bank to 0");
		return -EIO;
	}

	if(icm20649_set_reset(dev) < 0) {
		LOG_ERR("failed to reset device");
		return -EIO;
	}

	if(icm20649_set_auto_clock(dev) < 0) {
		LOG_ERR("failed to auto clock");
		return -EIO;
	}

}

static inline int icm20649_set_int_pin_cfg(struct device *dev,
					   struct icm20649_config *cfg)
{
	return icm20649_update_reg8(dev, ICM20649_REG_INT_PIN_CFG,
			ICM20649_MASK_INT_PIN_CFG_INT1_ACTL |
			ICM20649_MASK_INT_PIN_CFG_INT1_OPEN |
			ICM20649_MASK_INT_PIN_CFG_INT1_LATCH_EN,
			0 << ICM20649_SHIFT_INT_PIN_CFG_INT1_ACTL |
			0 << ICM20649_SHIFT_INT_PIN_CFG_INT1_OPEN |
			0 << ICM20649_SHIFT_INT_PIN_CFG_INT1_LATCH_EN
			);
}

static inline int icm20649_set_int_enable_3(struct device *dev,
					    struct icm20649_config *cfg)
{
	return icm20649_update_reg8(dev, ICM20649_REG_INT_ENABLE_3,
			ICM20649_MASK_INT_ENABLE_3_FIFO_WM_EN,
			cfg->fifo_enabled << ICM20649_SHIFT_INT_ENABLE_3_FIFO_WM_EN);
}

static inline int icm20649_set_int_enable_2(struct device *dev,
					    struct icm20649_config *cfg)
{
	return icm20649_update_reg8(dev, ICM20649_REG_INT_ENABLE_2,
			ICM20649_MASK_INT_ENABLE_2_FIFO_OVERFLOW_EN,
			cfg->fifo_enabled << ICM20649_SHIFT_INT_ENABLE_2_FIFO_OVERFLOW_EN);
}

static inline int icm20649_set_int_enable_1(struct device *dev) {
	u8_t val = 0;
	if(!cfg->fifo_enabled) {
		val = cfg->data_ready_enabled <<
			ICM20649_SHIFT_INT_ENABLE_1_RAW_DATA_0_RDY_EN;
	}
	return icm20649_update_reg8(dev, ICM20649_REG_INT_ENABLE_1,
				    ICM20649_MASK_INT_ENABLE_1_RAW_DATA_0_RDY_EN,
				    val);
}

static inline int icm20649_fifo_watermark_int_status(struct device *dev,
						     u8_t *status)
{
	return icm20649_read_reg8(dev, ICM20649_REG_INT_STATUS_3, status);
}

static inline int icm20649_fifo_watermark_int_clear(struct device *dev)
{
	return icm20649_write_reg8(dev, ICM20649_REG_INT_STATUS_3, 0);
}

int icm20649_fifo_overflow_int_status(struct device *dev, u8_t *status)
{
	return icm20649_read_reg8(dev, ICM20649_REG_INT_STATUS_2, status);
}

static inline int icm20649_fifo_overflow_int_clear(struct device *dev)
{
	return icm20649_write_reg8(dev, ICM20649_REG_INT_STATUS_2, 0);
}

static inline int icm20649_set_fifo_en_2(struct device *dev) {
	u8_t accel_en = cfg->fifo_enabled && cfg->accel_config->enabled ? 1 : 0;
	u8_t gyro_en = cfg->fifo_enabled && cfg->gyro_config->enabled ? 1 : 0;
	u8_t temp_en = cfg->fifo_enabled && cfg->temp_config->enabled ? 1 : 0;
	u8_t mask = ICM20649_MASK_FIFO_EN_2_ACCEL_FIFO_EN |
		ICM20649_MASK_FIFO_EN_2_GYRO_Z_FIFO_EN |
		ICM20649_MASK_FIFO_EN_2_GYRO_Y_FIFO_EN |
		ICM20649_MASK_FIFO_EN_2_GYRO_X_FIFO_EN |
		ICM20649_MASK_FIFO_EN_2_TEMP_FIFO_EN;
	u8_t val = accel_en << ICM20649_SHIFT_FIFO_EN_2_ACCEL_FIFO_EN |
		gyro_en << ICM20649_SHIFT_FIFO_EN_2_GYRO_Z_FIFO_EN |
		gyro_en << ICM20649_SHIFT_FIFO_EN_2_GYRO_Y_FIFO_EN |
		gyro_en << ICM20649_SHIFT_FIFO_EN_2_GYRO_X_FIFO_EN |
		temp_en << ICM20649_SHIFT_FIFO_EN_2_TEMP_FIFO_EN
	return icm20649_update_reg8(dev, ICM20649_REG_FIFO_EN_2, mask, val);
}

static inline int icm20649_set_fifo_en_1(struct device *dev,
					 struct icm20649_config *cfg) {
	return icm20649_update_reg8(dev, ICM20649_REG_FIFO_EN_1,
				    ICM20649_MASK_FIFO_EN_SLV_3_FIFO_EN |
				    ICM20649_MASK_FIFO_EN_SLV_2_FIFO_EN |
				    ICM20649_MASK_FIFO_EN_SLV_1_FIFO_EN |
				    ICM20649_MASK_FIFO_EN_SLV_0_FIFO_EN,
				    0 << ICM20649_SHIFT_FIFO_EN_SLV_3_FIFO_EN |
				    0 << ICM20649_SHIFT_FIFO_EN_SLV_2_FIFO_EN |
				    0 << ICM20649_SHIFT_FIFO_EN_SLV_1_FIFO_EN |
				    0 << ICM20649_SHIFT_FIFO_EN_SLV_0_FIFO_EN
	);
}


static inline int icm20649_set_fifo_mode(struct device *dev,
					 struct icm20649_config *cfg)
{
	return icm20649_update_reg8(dev, ICM20649_REG_FIFO_MODE,
			ICM20649_MASK_FIFO_MODE,
			ICM20649_FIFO_MODE_STREAM);
}


static inline int icm20649_set_gyro_config_1(struct device *dev,
					     struct icm20649_config *cfg) {
	u8_t mask = ICM20649_MASK_GYRO_CONFIG_1_GYRO_FCHOICE
		| ICM20649_MASK_GYRO_CONFIG_1_GYRO_DLPFCFG
		| ICM20649_MASK_GYRO_CONFIG_1_GYRO_FS_SEL;
	u8_t fchoice = 0;
	u8_t dlpfcfg = 0;
	if(cfg->gyro->filter != ICM20649_GYRO_LPF_DISABLED) {
		fchoice = 1 << ICM20649_SHIFT_GYRO_CONFIG_1_GYRO_FCHOICE;
		dlpfcfg = cfg->gyro->filter << ICM20649_SHIFT_GYRO_CONFIG_1_GYRO_DLPFCFG;
	}
	u8_t fs = cfg->gyro->scale << ICM20649_SHIFT_GYRO_CONFIG_1_GYRO_FS_SEL;
	u8_t val = fchoice | dlpfcfg | fs;
	return icm20649_update_reg8(dev, ICM20649_REG_GYRO_CONFIG_1,
			mask,
			val);
}

static inline int icm20649_fifo_reset(struct device *dev) {
	u16_t len = ICM20649_FIFO_SIZE;
	u8_t tries = 0;
	int ret = 0;
	while(len != 0 && tries < 6) {
		ret = icm20649_update_reg8(dev, ICM20649_REG_USER_CTRL,
				ICM20649_MASK_USER_CTRL_DMP_EN |
				ICM20649_MASK_USER_CTRL_FIFO_EN,
				0 << ICM20649_SHIFT_USER_CTRL_DMP_EN |
				0 << ICM20649_SHIFT_USER_CTRL_FIFO_EN
				);
		ret |= icm20649_update_reg8(dev, ICM20649_REG_FIFO_RST,
				ICM20649_MASK_FIFO_RST,
				1 << ICM20649_SHIFT_FIFO_RST);
		ret |= icm20649_fifo_count(dev, &len);
		if(ret) {
			return ret;
		}
	}
	ret |= icm20649_update_reg8(dev, ICM20649_REG_FIFO_RST,
			ICM20649_MASK_FIFO_RST,
			0 << ICM20649_SHIFT_FIFO_RST);
	ret |= icm20649_update_reg8(dev, ICM20649_REG_USER_CTRL,
			ICM20649_MASK_USER_CTRL_DMP_EN |
			ICM20649_MASK_USER_CTRL_FIFO_EN,
			0 << ICM20649_SHIFT_USER_CTRL_DMP_EN |
			1 << ICM20649_SHIFT_USER_CTRL_FIFO_EN
			);

	return ret;
}

static inline int icm20649_fifo_count(struct device *dev, u16_t *cnt) {
	struct icm20649_data *data = dev->driver_data;
	u8_t read_buf[2];
    int ret = icm20649_spi_read(data, ICM20649_REG_FIFO_COUNTH, read_buf, 2);
	*cnt = ((u16_t)read_buf[0] << 8) + (u16_t)read_buf[1];
	return ret;
}


static int icm20649_configure_device(struct device *dev,
			      struct icm20649_config *cfg) {

	u8_t chip_id;

	if(icm20649_set_user_bank(dev, 0) < 0) {
		LOG_ERR("Failed to set user bank to 0");
		return -EIO;
	}

	if(icm20649_set_sleep(dev, cfg->sleep) < 0) {
		LOG_ERR("Set sleep state to %d failed", cfg->sleep);
		return -EIO;
	}

	/* no need to do more if sensor is put to sleep */
	if(cfg->sleep) {
		return 0;
	}

	if(icm20649_set_pwr_mgmt_1(dev, cfg) < 0) {
		LOG_WRN("Failed to set PWR_MGMT_1 register");
		return -EIO;
	}

	if(icm20649_set_pwr_mgmt_2(dev, cfg) < 0) {
		LOG_WRN("Failed to set PWR_MGMT_2 register");
		return -EIO;
	}

	/** TODO
	 * Disable digital low power mode before manipulating other
	 * registers and turn it back on if enabled once done
	 */

	if(icm20649_set_int_pin_cfg(dev, cfg) < 0) {
		LOG_WRN("Failed to set INT_PIN_CFG register");
		return -EIO;
	}

	if(icm20649_set_int_enable(dev, cfg) < 0) {
		LOG_WRN("Failed to set INT_ENABLE register");
		return -EIO;
	}

	if(icm20649_set_int_enable_1(dev, cfg) < 0) {
		LOG_WRN("Failed to set INT_ENABLE_1 register");
		return -EIO;
	}

	if(icm20649_set_int_enable_2(dev, cfg) < 0) {
		LOG_WRN("Failed to set INT_ENABLE_2 register");
		return -EIO;
	}

	if(icm20649_set_int_enable_3(dev, cfg) < 0) {
		LOG_WRN("Failed to set INT_ENABLE_3 register");
		return -EIO;
	}

	if(icm20649_set_fifo_en_2(dev, cfg) < 0) {
		LOG_WRN("Failed to set FIFO_EN_2 register");
		return -EIO;
	}

	if(icm20649_set_fifo_mode(dev, cfg) < 0) {
		LOG_WRN("Failed to set FIFO_MODE register");
		return -EIO;
	}

	if(icm20649_set_user_bank(dev, 0) < 0) {
		LOG_ERR("Failed to set user bank to 0");
		return -EIO;
	}

	if(icm20649_set_gyro_smplrt_div(dev, cfg) < 0) {
		LOG_WRN("Failed to set GYRO_SMPLRT_DIV register");
		return -EIO;
	}

	if(icm20649_set_gyro_config_1(dev, cfg) < 0) {
		LOG_WRN("Failed to set GYRO_CONFIG_1 register");
		return -EIO;
	}

	if(icm20649_set_accel_smplrt_div_1(dev, cfg) < 0) {
		LOG_WRN("Failed to set ACCEL_SMPLRT_DIV_1 register");
		return -EIO;
	}

	if(icm20649_set_accel_smplrt_div_2(dev, cfg) < 0) {
		LOG_WRN("Failed to set ACCEL_SMPLRT_DIV_2 register");
		return -EIO;
	}

	if(icm20649_set_accel_config(dev, cfg) < 0) {
		LOG_WRN("Failed to set ACCEL_CONFIG register");
		return -EIO;
	}

	if(icm20649_set_temp_config(dev, cfg) < 0) {
		LOG_WRN("Failed to set TEMP_CONFIG register");
		return -EIO;
	}

	LOG_DBG("Successfully initialized ICM20649");
	return 0;
}

static struct icm20649_drv_config icm20649_drv_config = {
	.dev_name = DT_TDK_ICM20649_0_BUS_NAME,
};


int icm20649_init(struct device *dev) {
	const struct icm20649_drv_config * const config = &icm20649_config;
	struct icm20649_drv_data *data = dev->driver_data;

	data->spi = device_get_binding(config->dev_name);
	if (!data->spi) {
		LOG_ERR("SPI device not found: %s",
			    config->dev_name);
		return -EINVAL;
	}

	icm20649_cs_ctrl.gpio_dev = device_get_binding(DT_TDK_ICM20649_0_CS_GPIO_CONTROLLER);
	icm20649_cs_ctrl.gpio_pin = DT_TDK_ICM20649_0_CS_GPIO_PIN;
	icm20649_cs_ctrl.delay = 0;
	if (!icm20649_cs_ctrl.gpio_dev) {
		LOG_ERR("GPIO device for SPI CS not found: %s", DT_TDK_ICM20649_0_CS_GPIO_CONTROLLER);
		return -ENODEV;
	}
	icm20649_spi_conf.cs = &icm20649_cs_ctrl;

	LOG_DBG("SPI GPIO CS configured on %s:%u",
			DT_TDK_ICM20649_0_CS_GPIO_CONTROLLER,
			DT_TDK_ICM20649_0_CS_GPIO_PIN);

	if (icm20649_check_who_am_i(dev) < 0) {
		LOG_ERR("WHO_AM_I did not match ICM20649 expected value");
		return -EIO;
	}

	return 0;
}


static int icm20649_configure(struct device *dev, struct rtio_config *cfg)
{
	struct icm20649_drv_data *data = dev->driver_data;
	int res = 0;

	/** TODO use a common rtio_driver_data struct and configuration call */
	/* res = rtio_begin_configure(dev, data->rtio_drv_data);
	 * if(res != 0) {
	 * return -EINVA;
	 * }
	 */
	struct icm20649_config *cfg = cfg->driver_config;
	res = icm20649_configure_device(dev, cfg);
	/**
	 * rtio_end_configure(dev, data->rtio_drv_data);
	 */
	return res;
}

static const struct rtio_sensor_api icm20649_api_funcs = {
	.rtio_api = {
		.configure = icm20649_configure,
		.trigger = icm20649_trigger
},
	.sensor_reader = icm20649_sensor_reader,
};



static struct icm20649_drv_data icm20649_drv_data;

DEVICE_AND_API_INIT(icm20649, DT_TDK_ICM20649_0_LABEL, icm20649_init,
		    &icm20649_drv_data, NULL, POST_KERNEL,
		    CONFIG_RTIO_SENSOR_INIT_PRIORITY, &icm20649_api_funcs);
