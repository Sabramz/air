/**
 * @file src/main.cpp
 * @brief Car entry point.
 */
#include <iostream>
#include <vector>

#include <driver/device.hpp>
#include <driver/pinmap.hpp>
#include <driver/rc552.hpp>
#include <shared/menu.hpp>

#include "demos.hpp"

static const std::vector<menu_item> car_menu = {
	{.text = "Demos", .action = &demo_submenu}};

int main() {
	rc552 rfid(gpio_pins, RASPI_38, RASPI_40, RASPI_7, "0.0");

	for (int i = 0; i < 50; i++) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if (rfid.PICC_IsNewCardPresent()) {
			if (rfid.PICC_ReadCardSerial()) {
				rfid.PICC_DumpMifareClassicToSerial();
			}
		}
	}

	// show_menu("Car Actions", car_menu, false);
	return 0;
}
