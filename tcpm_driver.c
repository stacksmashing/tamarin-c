#include "tamarin_hw.h"
#include "tcpm_driver.h"

/* I2C wrapper functions - get I2C port / slave addr from config struct. */
int16_t tcpc_write(int16_t port, int16_t reg, int16_t val)
{
	uint8_t buf[] = {
		reg & 0xff,
		val & 0xff,
	};

	i2c_write_blocking(FUSB_I2C_INST, FUSB_I2C_ADDR, buf, sizeof(buf), false);

	return 0;
}

int16_t tcpc_write16(int16_t port, int16_t reg, int16_t val)
{
	uint8_t buf[] = {
		reg & 0xff,
		val & 0xff,
		(val >> 8) & 0xff,
	};

	i2c_write_blocking(FUSB_I2C_INST, FUSB_I2C_ADDR, buf, sizeof(buf), false);

	return 0;
}

int16_t tcpc_read(int16_t port, int16_t reg, int16_t *val)
{
	uint8_t buf[] = {
		reg & 0xff,
		0,
	};

	i2c_write_blocking(FUSB_I2C_INST, FUSB_I2C_ADDR, &buf[0], 1, true);
	i2c_read_blocking(FUSB_I2C_INST, FUSB_I2C_ADDR, &buf[1], 1, false);

	*val = buf[1];

	return 0;
}

int16_t tcpc_read16(int16_t port, int16_t reg, int16_t *val)
{
	uint8_t buf[] = {
		reg & 0xff,
		0,
		0,
	};

	i2c_write_blocking(FUSB_I2C_INST, FUSB_I2C_ADDR, &buf[0], 1, true);
	i2c_read_blocking(FUSB_I2C_INST, FUSB_I2C_ADDR, &buf[1], 2, false);
	*val = buf[1];
	*val |= (buf[2] << 8);

	return 0;
}

int16_t tcpc_xfer(int16_t port,
	      const uint8_t * out, int16_t out_size,
	      uint8_t * in, int16_t in_size, int16_t flags)
{
	if (out_size) {
		i2c_write_blocking(FUSB_I2C_INST, FUSB_I2C_ADDR, out, out_size,
				   !(flags & I2C_XFER_STOP));
	}

	if (in_size) {
		i2c_read_blocking(FUSB_I2C_INST, FUSB_I2C_ADDR, in, in_size,
				  !(flags & I2C_XFER_STOP));
	}

	return 0;
}
