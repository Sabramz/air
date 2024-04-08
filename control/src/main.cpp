/**
 * @file src/main.cpp
 * @brief Control entry point.
 */
#include <vector>

#include <shared/menu.hpp>
#include <shared/utils.hpp>

#include "demos.hpp"

static const std::vector<menu_item> car_menu = {
	{.text = "Demos", .action = &demo_submenu},
	{.text = "About this program", .action = &print_about}};

int main() {
	show_menu("Control Actions", car_menu, false);
	return 0;
}
