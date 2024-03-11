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
#include "mavlink/common/mavlink.h"

extern lifo_buffer_t GPS_USB_Upload_Buffer;

extern int gpsUsbOpen;

#include "gpsusb.h"

volatile int fd;

// Function pointer to select either NMEA or MAVLink output type
typedef int (*GPSUsbSendFunction)(received_t *t);


int connectUSB(char* portname, int baudRate) {
    fd = serialOpen(portname, baudRate);
    return fd;
}

void disconnectUSB() {
    serialClose(fd);
    serialFlush(fd);
}


void sendUSB(char* str) {
    serialPuts(fd, str);
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
                } else if ((Config.GPSUSBObjChannel >= 0) && (Config.GPSUSBObjChannel != t->Metadata.Channel)) {
                    // Don't process this message since we are looking for other channel
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

// Function to convert HH:MM:SS to seconds since midnight
static int hhmmss_to_seconds(const char* hhmmss) {
    int hours, minutes, seconds;
    sscanf(hhmmss, "%d:%d:%d", &hours, &minutes, &seconds);
    return hours * 3600 + minutes * 60 + seconds;
}

// Function to get the number of seconds since the GPS epoch (1980-01-06)
static uint32_t get_time_since_gps_epoch(const char* gpshour) {
    // Get the current date and time
    time_t rawtime;
    struct tm * timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);

    // Parse the gpshour and set the hours, minutes, and seconds
    int seconds_since_midnight = hhmmss_to_seconds(gpshour);
    timeinfo->tm_hour = 0;
    timeinfo->tm_min = 0;
    timeinfo->tm_sec = 0;

    // Convert back to time_t and then to seconds since the GPS epoch
    time_t today_midnight = mktime(timeinfo);
    uint32_t seconds_since_gps_epoch = (uint32_t)(today_midnight - 315964800) + seconds_since_midnight; // 315964800 is the number of seconds from Unix epoch to GPS epoch

    return seconds_since_gps_epoch;
}

int sendMavlinkUsb(received_t *t) {
    bool valid = false;

    // Parse the custom telemetry string
    char *telemetry_str = t->Message;
    if (sscanf(telemetry_str, "%[^,],%ld,%8[^,],%lf,%lf,%ld",
        t->Telemetry.Callsign, &t->Telemetry.SentenceId, t->Telemetry.TimeString,
        &t->Telemetry.Latitude, &t->Telemetry.Longitude, &t->Telemetry.Altitude) == 6) {
        valid = true;
    }

    if (valid) {
        if (strlen(Config.GPSUSBObjName) && strcmp(Config.GPSUSBObjName, t->Telemetry.Callsign)) {
            // Don't process this message since we are looking for other object
            valid = false;
        } else if ((Config.GPSUSBObjChannel >= 0) && (Config.GPSUSBObjChannel != t->Metadata.Channel)) {
            // Don't process this message since we are looking for other channel
            valid = false;
        }
    }

    if (valid) {
        // Convert latitude and longitude to 1E7 format and altitude to millimeters
        int32_t lat_1E7 = (int32_t)(t->Telemetry.Latitude * 1E7);
        int32_t lon_1E7 = (int32_t)(t->Telemetry.Longitude * 1E7);
        int32_t alt_mm = t->Telemetry.Altitude * 1000;

        // Get the time in seconds since the GPS epoch
        // You will need to provide the actual date to this function
        uint32_t time_since_gps_epoch = get_time_since_gps_epoch(t->Telemetry.TimeString);

        uint8_t system_id = 1; // System ID of the sender
        uint8_t component_id = 1; // Component ID of the sender

        // Create a MAVLink GPS_RAW_INT message
        mavlink_message_t gps_msg;
        mavlink_msg_gps_raw_int_pack(system_id, component_id, &gps_msg, time_since_gps_epoch * 1E6, 3, lat_1E7, lon_1E7, alt_mm, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

        // Create a MAVLink GLOBAL_POSITION_INT message
        mavlink_message_t global_msg;
        mavlink_msg_global_position_int_pack(system_id, component_id, &global_msg, 0, lat_1E7, lon_1E7, alt_mm, 0, 0, 0, 0, 0);

        // Serialize the GPS_RAW_INT message
        uint8_t gps_buffer[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_to_send_buffer(gps_buffer, &gps_msg);

        // Serialize the message
        uint8_t global_buffer[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_to_send_buffer(global_buffer, &global_msg);

        // Send the message through USB
        sendUSB((char*)gps_buffer);
        sendUSB((char*)global_buffer);
    }
    return valid;
}

void *GpsUsbLoop( void *some_void_ptr ) {
    if (Config.EnableGPSUSB) {
        if (Config.GPSUSBPort) {
            if (connectUSB(Config.GPSUSBPort, 9600) > 0) {
                gpsUsbOpen = 1;
                //LogMessage( "GPS USB Serial Port openned\n" );
                received_t *dequeued_telemetry_ptr;
                // Create a function pointer and assign it to either sendNmeaUsb or sendMavlinkUsb
                GPSUsbSendFunction gpsUsbSendFunc;
                if (strcmp(Config.GPSUSBOutput, "MAVLINK") == 0) {
                    gpsUsbSendFunc = &sendMavlinkUsb; // Output Mavlink format
                    LogMessage("USB output on %s: MAVlink\n", Config.GPSUSBPort);
                } else {
                    gpsUsbSendFunc = &sendNmeaUsb; // Output NMEA format
                    LogMessage("USB output on %s: NMEA\n", Config.GPSUSBPort);
                }
                
                // Keep looping until the parent quits
                while ( true )
                {
                    dequeued_telemetry_ptr = lifo_buffer_waitpop(&GPS_USB_Upload_Buffer);

                    if(dequeued_telemetry_ptr != NULL)
                    {
                        if(gpsUsbSendFunc(dequeued_telemetry_ptr))
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
