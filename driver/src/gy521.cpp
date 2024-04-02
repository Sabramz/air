/**
 * @file include/gy521.cpp
 * @brief Drivers for the GY-521 6 axis accelerometer.
 */

#include "gy521.hpp"

#include <chrono>
#include <cstdint>

#include "defines.hpp"

static constexpr uint8_t REG_INTERRUPT_STATUS = 0x3A;
static constexpr uint8_t REG_ACCEL_X = 0x3B;
static constexpr uint8_t REG_ACCEL_Y = 0x3D;
static constexpr uint8_t REG_ACCEL_Z = 0x3F;
static constexpr uint8_t REG_GYRO_X = 0x43;
static constexpr uint8_t REG_GYRO_Y = 0x45;
static constexpr uint8_t REG_GYRO_Z = 0x47;

gy521::gy521(
	const gpiod::chip &chip, uint32_t int_pin, uint8_t dev_addr, int adapter)
	: interrupt(chip.get_line(int_pin)),
	  i2cd(i2c(dev_addr, adapter)) {
	interrupt.request({
		.consumer = GPIO_CONSUMER,
		.request_type = gpiod::line_request::EVENT_RISING_EDGE,
		.flags = 0,
	});
}

gy521::~gy521() {
	active = false;
	int_thread->join();
	interrupt.release();
}

void gy521::on_interrupt(std::function<void()> callback) {
	// Thread function
	auto executor = [&]() {
		while (active) {
			if (interrupt.event_wait(std::chrono::milliseconds(100))) {
				interrupt.event_read();
				i2cd.read_byte(REG_INTERRUPT_STATUS);
				callback();
			}
		}
	};

	int_thread = std::make_unique<std::thread>(executor);
}

uint16_t gy521::read_x_axis() const {
	return i2cd.read_word(REG_ACCEL_X);
}

uint16_t gy521::read_y_axis() const {
	return i2cd.read_word(REG_ACCEL_Y);
}

uint16_t gy521::read_z_axis() const {
	return i2cd.read_word(REG_ACCEL_Z);
}

uint16_t gy521::read_x_rot() const {
	return i2cd.read_word(REG_GYRO_X);
}

uint16_t gy521::read_y_rot() const {
	return i2cd.read_word(REG_GYRO_Y);
}

uint16_t gy521::read_z_rot() const {
	return i2cd.read_word(REG_GYRO_Z);
}