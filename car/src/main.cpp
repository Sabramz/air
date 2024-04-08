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
	rc552 rfid(gpio_pins, RASPI_7, RASPI_38, RASPI_40, "0.0");

	while(r)
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		if (rfid.PICC_IsNewCardPresent()) {
			printf(" CARD PRESENT ");
			// if (rfid.PICC_ReadCardSerial()) {
			// 	printf(" IT WORKED ");
			// 	// rfid.PICC_DumpMifareClassicToSerial();
			// } else {
			//     printf(" GET FUCKED ");
			// }
		} else {
			printf(" CARD NOT PRESENT ");
		}
	}

	// show_menu("Car Actions", car_menu, false);
	return 0;
}
