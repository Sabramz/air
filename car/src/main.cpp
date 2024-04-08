/**
 * @file src/main.cpp
 * @brief Car entry point.
 */
#include <iostream>
#include <vector>

#include <driver/MFRC522.h>
#include <driver/device.hpp>
#include <driver/pinmap.hpp>
#include <shared/menu.hpp>

#include "demos.hpp"

static const std::vector<menu_item> car_menu = {
	{.text = "Demos", .action = &demo_submenu}};

int main() {
	MFRC522 rfid(gpio_pins, RASPI_36, RASPI_38, RASPI_40, "0.0");

	std::this_thread::sleep_for(std::chrono::milliseconds(4));

	rfid.PCD_Init();

	std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    rfid.PCD_DumpVersionToSerial();

	// while (!rfid.PICC_IsNewCardPresent()) {
	// 	printf(" CARD NOT PRESENT ");
	// }

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

	// show_menu("Car Actions", car_menu, false);
	return 0;
}
