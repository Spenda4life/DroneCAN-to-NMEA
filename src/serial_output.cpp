#include "serial_output.h"
#include "config.h"

#include <Arduino.h>

void serial_output_init(void) {
    // Initialize UART2 with configured pins and baud rate
    Serial2.begin(SERIAL_BAUD, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);

    // Configure the RS485 DE/RE pin as output, default to receive mode
    pinMode(RS485_DE_PIN, OUTPUT);
    digitalWrite(RS485_DE_PIN, LOW);
}

void serial_send_sentence(const char* sentence, int len) {
    // Assert transmit-enable (DE HIGH = transmit mode)
    digitalWrite(RS485_DE_PIN, HIGH);
    delayMicroseconds(100);   // Allow DE line to settle before driving data

    Serial2.write((const uint8_t*)sentence, len);
    Serial2.flush();          // Block until all bytes have been transmitted

    delayMicroseconds(100);   // Hold DE HIGH until last bit has left the wire

    // Return to receive mode
    digitalWrite(RS485_DE_PIN, LOW);
}
