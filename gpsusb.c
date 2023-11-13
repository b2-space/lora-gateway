#include <unistd.h>
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
#include <math.h>

#include <wiringSerial.h>

#include "global.h"
#include "lifo_buffer.h"

extern lifo_buffer_t GPS_USB_Upload_Buffer;

extern int gpsUsbOpen;

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

char* nmeaTimestamp(char *datetime) {

   char* timeStamp = calloc(64, 1);
   char *rest = datetime;
   char *timeToken;

   while ((timeToken = strtok_r(rest, ":", &rest))) {
        strcat(timeStamp, timeToken);
   }
   return timeStamp;
}

char *nmeaLatitude(char *latitude) {

    char *ns = calloc(2,1);
    float latitudeValue = atof(latitude);

    if (latitudeValue < 0.00001) {
        ns =  "S";
        latitudeValue *= -1.0;
    }
    else {
        ns = "N";
    }

    float fLat = floor(latitudeValue);
    float minutesLat = (latitudeValue - fLat) * 60.00;

    //LogMessage( "Called nmeaLatitude with Latitude %s, converted to: %09.4f, %s\n" , latitude, (fLat*100.0) + minutesLat, ns); 
    char *ret = calloc(32, 1);
    sprintf(ret, "%09.4f,%s,", (fLat*100.0) + minutesLat, ns);
    return ret;
}

char *nmeaLongitude(char *longitude) {

    char *ew = calloc(2,1);
    float longitudeValue = atof(longitude);

    if (longitudeValue < 0.00001) {
        ew =  "W";
        longitudeValue *= -1.0;
    }
    else {
        ew = "E";
    }

    float fLong = floor(longitudeValue);
    float minutesLong = (longitudeValue - fLong) * 60.00;

    //LogMessage( "Called nmeaLongitude with Longitude %s, converted to: %010.4f, %s\n" , longitude, (fLong*100.0) + minutesLong, ew); 
    char *ret = calloc(32, 1);
    sprintf(ret, "%010.4f,%s,", (fLong*100.0) + minutesLong, ew);
    return ret;
}

char *nmeaAltitude(char *altitude) {

    char *ret = calloc(32, 1);
    sprintf(ret, "%.1f,M,", atof(altitude));
    return ret;
}

// we are generating the format (without spaces):
// $GPGGA, 125546.00, 5228.0434, N, 00201.2780, W, 1, 13, 1.00, 201.0, M, , , , 0000*40
//  given: $$B2S-INDRA-1,211,12:55:46,52.46739,-2.02130,201,10,0,0*F0D0
int sendNmeaUsb(received_t *t) {
    //LogMessage( "Parsing: %s\n" , t->Message);

    int i = 0;

    char *nmeaString = calloc(256, 1);
    char *rest = t->Message + 2; // Exclude starting $$ from module name
    char *token;
    strcat(nmeaString, "$GPGGA,");
    bool valid = true;

    while (valid && (token = strtok_r(rest, ",", &rest))) {
        switch(i) {
            case 1:
            case 6:
            case 7:
            case 8:
                //LogMessage( "Value of i: %d, token: %s\n" , i, token);
                break;
            case 0:
                if (strlen(Config.GPSUSBObjName) && strcmp(Config.GPSUSBObjName, token)) {
                    // Don't process this message since we are looking for other object
                    valid = false;
                }
                break;
            case 2:
                strcat(nmeaString, nmeaTimestamp(token));
                strcat(nmeaString, ".00,"); // 181908.00
                break;
            case 3:
    		//LogMessage( "latitude token %s\n" , token);
                if (strcmp(token, "0.00000") == 0) {
		    valid = false;
                    //LogMessage("zero latitude found\n");
                    break;
                }
                strcat(nmeaString, nmeaLatitude(token));
                break;
            case 4:
                strcat(nmeaString, nmeaLongitude(token));// 07044.3966270,W
                strcat(nmeaString, "1,");                    // Quality
                strcat(nmeaString, "13,");                   // Number of satllites
                strcat(nmeaString, "1.00,");                 // HDOP
                break;
            case 5:
                strcat(nmeaString, nmeaAltitude(token)); // 495.144,M
                strcat(nmeaString, ",");  // Geoidal Separation
                strcat(nmeaString, ",");            // Age
                strcat(nmeaString, ",0000");    // Correction
                break;
        }
        i++;
    }

    // Calculate and append checksum 
    int checksum = 0; 
    for (int i = 1; i < strlen(nmeaString); i++) { 
        checksum ^= nmeaString[i]; 
    } 
    sprintf(nmeaString + strlen(nmeaString), "*%02X", checksum);

    strcat(nmeaString, "\r\n");
    if (valid) {
        sendUSB(nmeaString);
    }
    return i;
}

int sendGpsUsb(received_t *t ) {

    char gpsStr[64] = "";
    char *token;

    int i = 0;
 
    // Keep printing tokens while one of the
    // delimiters present in str[].
    while ((token = strtok(t->Message, ","))) {
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
    }

    sendUSB(gpsStr);

    return i;
}

void *GpsUsbLoop( void *some_void_ptr ) {
    if (Config.EnableGPSUSB) {
        if (Config.GPSUSBPort) {
            if (connectUSB(Config.GPSUSBPort, 9600) > 0) {
                gpsUsbOpen = 1;
                //LogMessage( "GPS USB Serial Port openned\n" );
                received_t *dequeued_telemetry_ptr;

                // Keep looping until the parent quits
                while ( true )
                {
                    dequeued_telemetry_ptr = lifo_buffer_waitpop(&GPS_USB_Upload_Buffer);
        
                    if(dequeued_telemetry_ptr != NULL)
                    {
                        if(sendNmeaUsb(dequeued_telemetry_ptr ))
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
                LogMessage( "GPS USB Serial Port not openned\n");
            }
        }
        else {
            LogMessage( "GPS USB Serial Port not set\n");
        }
    }
    return NULL;
}
