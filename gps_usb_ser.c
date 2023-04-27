#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ctype.h>
#include <stdio.h>              // Standard input/output definitions
#include <string.h>             // String function definitions
#include <stdlib.h>             // String function definitions

#include <wiringSerial.h>

#include "global.h"

#include "gps_usb_ser.h"

volatile int usbPortPtr;

int connectUSB(char* usbPort, int baudRate) {
    usbPortPtr = serialOpen(usbPort, baudRate);
    return usbPortPtr;
}

void disconnectUSB() {
    serialClose(usbPortPtr);
}

