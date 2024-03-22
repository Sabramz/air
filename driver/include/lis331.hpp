/**
 * @file src/lis331.hpp
 * @brief LIS331 driver.
 */

#pragma once

#include <cstdint>

#include "i2c.hpp"

class lis331 {
public:
	enum reg {
		WHO_AM_I = 0x0F,
		CTRL_REG1 = 0x20,
		CTRL_REG2 = 0x21,
		CTRL_REG3 = 0x22,
		CTRL_REG4 = 0x23,
		CTRL_REG5 = 0x24,
		REFERENCE = 0x25,
		STATUS_REG = 0x27,
		OUT_X_L = 0x28,
		OUT_X_H = 0x29,
		OUT_Y_L = 0x2A,
		OUT_Y_H = 0x2B,
		OUT_Z_L = 0x2C,
		OUT_Z_H = 0x2D,
		INT1_CFG = 0x30,
		INT1_SOURCE = 0x31,
		INT1_THS = 0x32,
		INT1_DURATION = 0x33,
		INT2_CFG = 0x34,
		INT2_SOURCE = 0x35,
		INT2_THS = 0x36,
		INT2_DURATION = 0x37
	};

	/**
	 * @brief Initializer
	 *
	 * @param[in] i2c - A pointer to an instance of i2c.
	 */
	lis331(i2c *i2c);

	~lis331();

	/**
	 * @brief Indicate that new data is available
	 *
	 * @return Boolean indicating if there is new data to read.
	 */
	bool new_data() const;

	/**
	 * @brief Read x-axis acceleration
	 *
	 * @param[out] data - 2 btye array [low, high]
	 * @return Number of btyes read
	 */
	int read_x(uint8_t *data) const;

	/**
	 * @brief Read y-axis acceleration
	 *
	 * @param[out] data - 2 btye array [low, high]
	 * @return Number of btyes read
	 */
	int read_y(uint8_t *data) const;

	/**
	 * @brief Read z-axis acceleration
	 *
	 * @param[out] data - 2 btye array [low, high]
	 * @return Number of btyes read
	 */
	int read_z(uint8_t *data) const;

private:
	i2c *i2cd;
};
