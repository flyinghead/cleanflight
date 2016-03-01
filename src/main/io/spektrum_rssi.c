#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "common/maths.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "io/serial.h"

#include "config/config.h"

#define SPEK_FRAME_SIZE 16

static bool enabled = false;
static volatile uint8_t spekFrame[SPEK_FRAME_SIZE];
static uint32_t timeLastDataReceived;

void rssiDataReceived(uint16_t c);

//From rx.c:
extern uint16_t rssi;

void spektrumRssiInit(/* serialConfig_t *initialSerialConfig */)
{
    serialPortConfig_t *rssiPortConfig = findSerialPortConfig(FUNCTION_SPEKTRUM_RSSI);
    if (!rssiPortConfig) {
        return;
    }

    enabled = openSerialPort(rssiPortConfig->identifier, FUNCTION_SPEKTRUM_RSSI, rssiDataReceived, 115200, MODE_RX, SERIAL_NOT_INVERTED);
}

void updateSpekRssi(volatile uint8_t *frame) {
    static uint32_t prevFrameLosses = 0;
    static uint32_t prevMainAntennaFades = 0;
    
    //int holds = spekFrame[1];
    uint16_t frameLosses = (frame[2] << 8) + frame[3];
    uint16_t mainAntennaFades = (frame[4] << 8) + frame[5];
    //int satFades = spekFrame[6] * 256 + spekFrame[7];
    rssi = constrain((rssi / 1023.0 * 0.85
                      + (1 - frameLosses + prevFrameLosses) * 0.1
                      + (1 - mainAntennaFades + prevMainAntennaFades) * 0.05) * 1023, 0, 1023);
    prevFrameLosses = frameLosses;
    prevMainAntennaFades = mainAntennaFades;
}

// Receive ISR callback
void rssiDataReceived(uint16_t c) {
    uint32_t time;
    uint32_t timeInterval;
    static uint8_t framePosition;
    
    time = micros();
    timeInterval = time - timeLastDataReceived;
    timeLastDataReceived = time;
    if (timeInterval > 5000) {
        framePosition = 0;
    }

    spekFrame[framePosition] = (uint8_t)c;
    if (framePosition == SPEK_FRAME_SIZE - 1) {
        updateSpekRssi(spekFrame);
        framePosition = 0;
    } else {
        framePosition++;
    }
}

void checkSpektrumRssiTimeout() {
    if (enabled && micros() - timeLastDataReceived > 1000 * 1000) {
        // No data received for 1 sec
        rssi = 0;
    }
}
