#ifndef SERIAL_WRAPPER_H
#define SERIAL_WRAPPER_H

extern "C" {

#include <stdint.h>

void ser_setup();
void ser_cleanup();
void ser_open(const char* portName, int n);
uint8_t ser_isOpen();
void ser_close();
uint8_t ser_send(const uint8_t* msg, unsigned n);

// uint8_t startStreaming(int rate);
// uint8_t stopStreaming(int rate);

uint8_t ser_newPacketAvailable();
uint8_t* ser_getLastPacket();

void ser_startLogging();
void ser_stopLogging();

void ser_openLog(const char* filename);
uint8_t ser_logPacketAvailable();
uint8_t* ser_getLogPacket();

}

#endif
