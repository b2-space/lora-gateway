#include <sys/types.h>
#include <sys/stat.h>
#include <ctype.h>
#include <stdio.h>              // Standard input/output definitions
#include <string.h>             // String function definitions
#include <stdlib.h>             // String function definitions
#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>

#include <wiringSerial.h>

#include "global.h"
#include "lifo_buffer.h"

extern lifo_buffer_t GPS_USB_Upload_Buffer;

#include "gpsusb.h"

volatile int fd;


int connectUSB(char* portname, int baudRate) {
    fd = serialOpen(portname, baudRate);
    return fd;
}

void disconnectUSB() {
    serialClose(fd);
    serialFlush(fd);
}


void sendUSB(char* latLongAz) {
    serialPuts(fd, latLongAz);
}

int sendGpsUsb(received_t *t ) {
    char* token = strtok(t->Message, ",");
    char gpsStr[64] = "";

    int i = 0;
 
    // Keep printing tokens while one of the
    // delimiters present in str[].
    while (token != NULL) {
        switch(i) {
            case 3:
            case 4:
                strcat(gpsStr, token);
                strcat(gpsStr, ",");
            break;
            case 5:
                strcat(gpsStr, token);
                strcat(gpsStr, "\r\n");
            break;
        }
        i++;
        token = strtok(NULL, ",");
    }

    sendUSB(gpsStr);

    return i;
}

void *GpsUsbLoop( void *some_void_ptr ) {
    if (Config.EnableGPSUSB) {
        if (Config.GPSUSBPort) {
            if (connectUSB(Config.GPSUSBPort, 9600) > 0) {
                LogMessage( "GPS USB Serial Port openned\n" );
                received_t *dequeued_telemetry_ptr;

                // Keep looping until the parent quits
                while ( true )
                {
                    dequeued_telemetry_ptr = lifo_buffer_waitpop(&GPS_USB_Upload_Buffer);
        
                    if(dequeued_telemetry_ptr != NULL)
                    {
                        if(sendGpsUsb(dequeued_telemetry_ptr ))
                        {
                            free(dequeued_telemetry_ptr);
                        }
                        else
                        {
                            if(!lifo_buffer_requeue(&GPS_USB_Upload_Buffer, dequeued_telemetry_ptr))
                            {
                                /* Requeue failed, drop packet */
                                free(dequeued_telemetry_ptr);
                            }
                        }
	            }
	            else
	            {
                        /* NULL returned: We've been asked to quit */
                        /* Don't bother free()ing stuff, as application is quitting */
		        break;
	            }
	        }
            }
            else {
                LogMessage( "GPS USB Serial Port not openned\n" );
            }
        }
        else {
            LogMessage( "GPS USB Serial Port not set\n" );
        }

    }
}
