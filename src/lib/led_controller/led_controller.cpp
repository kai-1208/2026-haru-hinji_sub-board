#include "led_controller.hpp"

LedController::LedController(PinName tx_pin, PinName rx_pin)
    : _nano_serial(tx_pin, rx_pin) {
}

void LedController::setup() {
    _nano_serial.set_baud(9600);
}

void LedController::sendLedState(LedState state) {
    char c = static_cast<char>(state);
    _nano_serial.write(&c, 1);
}