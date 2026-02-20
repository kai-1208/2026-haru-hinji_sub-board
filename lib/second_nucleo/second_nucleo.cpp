#include "second_nucleo.hpp"

SecondNucleo::SecondNucleo(BufferedSerial &uart) : uart(uart) {}

bool SecondNucleo::send_message(const char *message) {
    if (message == nullptr || strlen(message) == 0) {
        return false; // メッセージが空の場合は送信しない
    }
    std::string msg(message);
    if (!msg.empty() && msg.back() == '\n') {
        msg.pop_back();
    }
    msg += "|";
    message = msg.c_str();
    uart.write(message, strlen(message));
    return true;
}

void SecondNucleo::clear_buffers() {
    // 送信バッファをクリア（可能な限り）
    uart.sync(); // 送信完了を待つ
    
    // 受信バッファをクリア
    char temp_buf[BUFFER_SIZE];
    while (uart.readable()) {
        uart.read(temp_buf, BUFFER_SIZE);
    }
}

bool SecondNucleo::send_message_with_clear(const char *message) {
    clear_buffers(); // 未送信データを破棄
    return send_message(message); // メッセージを送信
}

std::string SecondNucleo::receive_message(char *buffer, size_t size) {
    static std::string recv_buffer;
    if (size > BUFFER_SIZE) {
        size = BUFFER_SIZE; // バッファサイズを超えないように調整
    }
    // 受信データを一時バッファに読み込む
    char temp_buf[BUFFER_SIZE];
    int bytes_read = uart.read(temp_buf, size - 1); // 1バイトは終端文字用に残す
    if (bytes_read > 0) {
        temp_buf[bytes_read] = '\0';
        recv_buffer += temp_buf;
        size_t delim_pos = recv_buffer.find('|');
        if (delim_pos != std::string::npos) {
            // 区切り文字までを返す
            size_t copy_len = std::min(delim_pos, size - 1);
            strncpy(buffer, recv_buffer.c_str(), copy_len);
            buffer[copy_len] = '\0';
            std::string result(buffer);
            recv_buffer.erase(0, delim_pos + 1); // 区切り文字まで消す
            return result;
        }
    }
    buffer[0] = '\0';
    return ""; // 読み取り失敗
}