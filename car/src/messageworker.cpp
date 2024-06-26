/**
 * @file src/messageworker.cpp
 * @brief Message handler for car
 */
#include "messageworker.hpp"

#include <functional>
#include <sstream>
#include <stdexcept>

#include <shared/messages.hpp>

static constexpr std::string MSG_HEADER = "AIRv1.0";
static constexpr std::string CHECK = "CHK";
static constexpr std::string ACKNOWLEDGE = "ACK";
static constexpr std::string STANDBY = "SBY";
static constexpr std::string GO_REQUESTED = "GRQ";
static constexpr std::string CLEAR = "CLR";

static std::string format_checkin();

static constexpr uint8_t MESSAGE_TIMEOUT =
	4; /*time to wait for message (in frames)*/

message_worker::message_worker(const std::shared_ptr<tdma> &tdma_handler_in)
	: tdma_handler(tdma_handler_in),
	  car_id(get_id()),
	  current_pos(tdma_handler->get_timeslot()) {}

std::optional<std::string> message_worker::send_checkin() {
	tdma_handler->tx_sync(format_checkin()); // send check in

	std::string control_id;

	control_id = tdma_handler->rx_sync(MESSAGE_TIMEOUT); // receive check in

	if (!validate_id(control_id)) {
		return std::nullopt;
	}

	return control_id;
}

#include <iostream>

message_worker::command message_worker::send_request(uint8_t desired_pos) {
	tdma_handler->tx_sync(format_request(desired_pos));
	std::string command;
	uint32_t iterator = 0;

	while (iterator < 3) {
		iterator++;

		std::string response =
			tdma_handler->rx_sync(MESSAGE_TIMEOUT); // receive check in

		std::istringstream parts(response);
		std::string ack;

		parts >> ack;
		if (parts.eof() || ack != ACKNOWLEDGE) {
			continue;
		}
		parts >> command;
		if (!parts.eof()) {
			continue;
		}

		break;
	}

	tdma_handler->tx_sync(ACKNOWLEDGE);

	std::cout << command << std::endl;
	if (command == STANDBY) {
		return SBY;
	}

	if (command == GO_REQUESTED) {
		return GRQ;
	}

	throw std::invalid_argument("Unsupported");
}

void message_worker::send_clear() {
	tdma_handler->tx_sync(CLEAR);
}

void message_worker::send_acknowledge() {
	tdma_handler->tx_sync(ACKNOWLEDGE);
}

std::string format_checkin() {
	return MSG_HEADER + " " + CHECK;
}

std::string message_worker::format_request(uint8_t desired_pos) {
	std::string formatted_request;
	formatted_request.append(*car_id + " ");
	formatted_request.push_back((char)(current_pos + '0'));
	formatted_request.push_back((char)(desired_pos + '0'));
	return formatted_request;
}
