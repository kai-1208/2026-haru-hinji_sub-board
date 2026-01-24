#ifndef SECOND_NUCLEO_HPP
#define SECOND_NUCLEO_HPP

#include "mbed.h"
#include "drivers/BufferedSerial.h" // ← UnbufferedSerial ではなく

class SecondNucleo
{
    public:
        SecondNucleo(BufferedSerial &uart); // ← スペル修正
        bool send_message(const char *message);
        bool send_message_with_clear(const char *message); // バッファクリア付き送信
        std::string receive_message(char *buffer, size_t size);
        void clear_buffers(); // バッファクリア機能
    private:
        BufferedSerial &uart;
        static constexpr size_t BUFFER_SIZE = 256;
        char buffer[BUFFER_SIZE];
};


#endif