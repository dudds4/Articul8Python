#ifndef SERIAL_WRAPPER_H
#define SERIAL_WRAPPER_H

extern "C" {

#include <stdint.h>

void ser_setup();
void ser_cleanup();
void ser_open(const char* portName, int n, unsigned port, int nLras);
uint8_t ser_isOpen(unsigned port);
void ser_close(unsigned port);
uint8_t ser_send(const uint8_t* msg, unsigned n, unsigned port);

// uint8_t startStreaming(int rate);
// uint8_t stopStreaming(int rate);

uint8_t ser_newPacketAvailable(unsigned port);
uint8_t* ser_getLastPacket(unsigned port);
uint8_t* ser_getLraPacket(unsigned port);

void ser_startLogging(unsigned port);
void ser_stopLogging(unsigned port);

void ser_openLog(const char* filename, unsigned port);
uint8_t ser_logPacketAvailable(unsigned port);
uint8_t* ser_getLogPacket(unsigned port);

double ser_getFrequency(unsigned port);

bool ser_startRecording();
bool ser_stopRecording();
bool ser_startExercise();
bool ser_stopExercise();

}

#endif
