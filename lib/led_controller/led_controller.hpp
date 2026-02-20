#ifndef LED_CONTROLLER_HPP
#define LED_CONTROLLER_HPP

#include "mbed.h"

enum class LedState : char {
    OFF         = '0',
    CommLost    = '1',
    Normal      = '2',
    Auto        = '3',
    SemiAuto    = '4',
    HighSpeed   = '5',
    LowSpeed    = '6',
    Unknown     = '8'
};

class LedController {
public:
    LedController(PinName tx_pin, PinName rx_pin);

    void setup();

    /**
     * @param state 送信したい状態(LedState列挙型)
     */
    void sendLedState(LedState state);

private:
    BufferedSerial _nano_serial;
};

#endif // LED_CONTROLLER_HPP