#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ctype.h>
#include <stdio.h>              // Standard input/output definitions
#include <string.h>             // String function definitions
#include <stdlib.h>             // String function definitions
#include <stddef.h>             // to use offsetof
#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>
#include <math.h>

#include <wiringSerial.h>

#include "global.h"
#include "lifo_buffer.h"
#include "mavlink/common/mavlink.h"
#include "antennatracker.h"

extern lifo_buffer_t GPS_USB_Upload_Buffer;

extern int gpsUsbOpen;

#include "gpsusb.h"

volatile int fd;

bool manualAATmodeOn = false;

// Function pointer to select either NMEA or MAVLink output type
typedef int (*GPSUsbCreateFunction)(received_t *t);
typedef void (*GPSUsbSendFunction)();

// Create a function pointer and assign it to either sendNmeaUsb or sendMavlinkUsb
GPSUsbCreateFunction gpsUsbCreateFunc;
GPSUsbSendFunction gpsUsbSendFunc;

static char* read_file_to_string(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (file == NULL) {
        LogMessage("ManualAAT: Error opening file: %s\n", filename);
        return NULL;
    }

    // Determine file size
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    // Allocate memory for the string
    char* content = (char*)malloc(file_size + 1);
    if (content == NULL) {
        LogMessage("ManualAAT: Memory allocation failed\n");
        fclose(file);
        return NULL;
    }

    // Read file contents into the string
    size_t bytes_read = fread(content, 1, file_size, file);
    if (bytes_read < file_size) {
        LogMessage("ManualAAT: Error reading file\n");
        free(content);
        fclose(file);
        return NULL;
    }

    // Null-terminate the string
    content[file_size] = '\0';

    fclose(file);
    return content;
}

/**
 * @brief Turn manual mode on and send packet, or turn it off and keep forwarding received packets
 * 
 * @param enabled true for manual mode, false for normal mode
 */
void gpsUsbManualMode(bool enabled) {
    if (gpsUsbOpen) {
        if (enabled) {
            received_t manualData;
            const char* filename = "manualAATstring.txt";
            char* fileContent = read_file_to_string(filename);

            if (fileContent != NULL) {
                LogMessage("ManualAAT: File contents:\n%s\n", fileContent);
                strcpy(manualData.Message, fileContent);
                free(fileContent);  // Don't forget to free the allocated memory
                gpsUsbCreateFunc(&manualData);
                gpsUsbSendFunc();
            }
            manualAATmodeOn = true;
        } else {
            /* Empty buffer */
            while(lifo_buffer_pop(&GPS_USB_Upload_Buffer) != NULL);
            manualAATmodeOn = false;
        }
    } else {
        LogMessage("ManualAAT: error, gpsusb not open\n");
    }
}

int connectUSB(char* portname, int baudRate) {
    fd = serialOpen(portname, baudRate);
    return fd;
}

void disconnectUSB() {
    serialClose(fd);
    serialFlush(fd);
}

void sendStringUSB(char* str) {
    serialPuts(fd, str);
}

void sendUSB(uint8_t* array, uint16_t size) {
    for(size_t i = 0; i < size; i++) {
        serialPutchar(fd, array[i]);
    }
}

// NMEA message buffer
 #define NMEA_STR_SIZE	220u
int nmea_str_len;
char final_nmea_sentence[NMEA_STR_SIZE];

void sendNmeaUsb() {
    if (nmea_str_len > 0) {
        // Send NMEA sentence through USB
        //LogMessage("Sending NMEA string (%d): %.*s\n", nmea_str_len, nmea_str_len-2, final_nmea_sentence);
        sendStringUSB(final_nmea_sentence);
    }
}

// Generating this format:
//  $GPGGA,125546.00,5228.0434,N,00201.2780,W,1,13,1.00,201.0,M,,,,*1C
// given:
//  $$B2S-INDRA-1,211,12:55:46,52.46739,-2.02130,201,10,0,0*F0D0
int createNmeaUsb(received_t *t) {
    bool valid = false;
	char *telemetry_str = t->Message;
	int satellites = 13, quality = 1;
	float hdop = 1.00;

	// Parse the telemetry string
    if (sscanf(telemetry_str, "%[^,],%lld,%8[^,],%lf,%lf,%lld",
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
		// Convert latitude and longitude to NMEA format
		int lat_deg = (int)fabs(t->Telemetry.Latitude);
		float lat_min = (fabs(t->Telemetry.Latitude) - lat_deg) * 60;
		int lon_deg = (int)fabs(t->Telemetry.Longitude);
		float lon_min = (fabs(t->Telemetry.Longitude) - lon_deg) * 60;

		// Prepare NMEA sentence
		char nmea_sentence[NMEA_STR_SIZE] = {};
		nmea_str_len = snprintf(nmea_sentence, sizeof(nmea_sentence),
				 "$GPGGA,%02d%02d%02d.00,%02d%07.4f,%c,%03d%07.4f,%c,%d,%02d,%04.2f,%3.1f,M,,,,",
				 (t->Telemetry.TimeString[0]-'0')*10 + (t->Telemetry.TimeString[1]-'0'), // hours
				 (t->Telemetry.TimeString[3]-'0')*10 + (t->Telemetry.TimeString[4]-'0'), // minutes
				 (t->Telemetry.TimeString[6]-'0')*10 + (t->Telemetry.TimeString[7]-'0'), // seconds
				 lat_deg, lat_min, t->Telemetry.Latitude >= 0 ? 'N' : 'S',
				 lon_deg, lon_min, t->Telemetry.Longitude >= 0 ? 'E' : 'W',
				 quality, satellites, hdop, (float)t->Telemetry.Altitude);

		// Calculate checksum
		int checksum = 0;
		for (int i = 1; nmea_sentence[i] != '\0' && nmea_sentence[i] != '*'; i++) {
			checksum ^= nmea_sentence[i];
		}

		// Append checksum to NMEA sentence
		nmea_str_len = snprintf(final_nmea_sentence, NMEA_STR_SIZE, "%s*%02X\r\n", nmea_sentence, checksum);

		if (nmea_str_len >= (NMEA_STR_SIZE - 1)) {
			valid = false;
            nmea_str_len = 0;
		}
	}

	if (valid) {
		return nmea_str_len; // Success
	} else {
		return 0;
	}
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

// Mavlink message buffers
uint8_t mavlink_gps_buffer[MAVLINK_MAX_PACKET_LEN];
uint16_t mavlink_gps_buffer_len;
uint8_t mavlink_global_buffer[MAVLINK_MAX_PACKET_LEN];
uint16_t mavlink_global_buffer_len;

void sendMavlinkUsb() {
    // Send the message through USB
    if (mavlink_gps_buffer_len > 0) {
        // LogMessage("Sending MAVLink GPS_RAW_INT msg (%dB)\n", mavlink_gps_buffer_len);
        sendUSB(mavlink_gps_buffer, mavlink_gps_buffer_len);
    }
    if (mavlink_global_buffer_len) {
        // LogMessage("Sending MAVLink GLOBAL_POSITION_INT msg (%dB)\n", mavlink_global_buffer_len);
        sendUSB(mavlink_global_buffer, mavlink_global_buffer_len);
    }
}

int createMavlinkUsb(received_t *t) {
    bool valid = false;

    // Parse the custom telemetry string
    char *telemetry_str = t->Message;
    if (sscanf(telemetry_str, "%[^,],%lld,%8[^,],%lf,%lf,%lld",
        t->Telemetry.Callsign, &t->Telemetry.SentenceId, t->Telemetry.TimeString,
        &t->Telemetry.Latitude, &t->Telemetry.Longitude, &t->Telemetry.Altitude) == 6) {
        valid = true;
    }

    if (t->Telemetry.Altitude > 5000) {
        /* AAT does not support this altitude: change lat, lon, alt for same angles with lower altitude */
        if (Config.EnableAntennaTracker) {
            double lat_new, lon_new, alt_new;
            int result = anttrack_calc_nearer_lower_point(t->Telemetry.Latitude, t->Telemetry.Longitude, t->Telemetry.Altitude,
                5000.0, &lat_new, &lon_new, &alt_new);
            if (result == 0) {
                t->Telemetry.Latitude = lat_new;
                t->Telemetry.Longitude = lon_new;
                t->Telemetry.Altitude = alt_new;
            } else {
                LogMessage("GPSUSB Error: ant track interpolation failed\n");
            }
        } else {
            LogMessage("GPSUSB Error: Alt>5000 but AntTrack disabled\n");
        }
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
        mavlink_msg_gps_raw_int_pack(system_id, component_id, &gps_msg, time_since_gps_epoch * 1E6, 3, lat_1E7, lon_1E7, alt_mm,
            UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT8_MAX, alt_mm, 0, 0, 0, 0, 0);

        // Create a MAVLink GLOBAL_POSITION_INT message
        mavlink_message_t global_msg;
        mavlink_msg_global_position_int_pack(system_id, component_id, &global_msg, 0, lat_1E7, lon_1E7, alt_mm, alt_mm, 0, 0, 0, UINT16_MAX);

        // Serialize the GPS_RAW_INT message
        mavlink_gps_buffer_len = mavlink_msg_to_send_buffer(mavlink_gps_buffer, &gps_msg);

        // Serialize the message
        mavlink_global_buffer_len = mavlink_msg_to_send_buffer(mavlink_global_buffer, &global_msg);
    }

	if (valid) {
		return (mavlink_gps_buffer_len + mavlink_global_buffer_len); // Success
	} else {
		return 0;
	}
}

void *GpsUsbLoop( void *some_void_ptr ) {
    if (Config.EnableGPSUSB) {
        if (Config.GPSUSBPort) {
            if (connectUSB(Config.GPSUSBPort, Config.GPSUSBBaudrate) > 0) {
                gpsUsbOpen = 1;
                //LogMessage( "GPS USB Serial Port openned\n" );
                received_t *dequeued_telemetry_ptr;
                if (strcmp(Config.GPSUSBOutput, "MAVLINK") == 0) {
                    // Output Mavlink format
                    gpsUsbCreateFunc = &createMavlinkUsb;
                    gpsUsbSendFunc = &sendMavlinkUsb;
                    LogMessage("USB output on %s: MAVlink\n", Config.GPSUSBPort);
                } else {
                    // Output NMEA format
                    gpsUsbCreateFunc = &createNmeaUsb;
                    gpsUsbSendFunc = &sendNmeaUsb;
                    LogMessage("USB output on %s: NMEA\n", Config.GPSUSBPort);
                }

                // Keep looping until the parent quits
                while ( true )
                {
                    if (manualAATmodeOn) {
                        dequeued_telemetry_ptr = lifo_buffer_waitpop(&GPS_USB_Upload_Buffer);

                        if(dequeued_telemetry_ptr != NULL)
                        {
                            gpsUsbCreateFunc(dequeued_telemetry_ptr);
                            free(dequeued_telemetry_ptr);
                            gpsUsbSendFunc();
                        }
                        else
                        {
                            /* NULL returned: We've been asked to quit */
                            /* Don't bother free()ing stuff, as application is quitting */
                            break;
                        }
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
