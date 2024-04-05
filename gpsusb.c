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

void sendStringUSB(char* str) {
    serialPuts(fd, str);
}

void sendUSB(uint8_t* array, uint16_t size) {
    for(size_t i = 0; i < size; i++) {
        serialPutchar(fd, array[i]);
    }
}

// Generating this format:
//  $GPGGA,125546.00,5228.0434,N,00201.2780,W,1,13,1.00,201.0,M,,,,*1C
// given:
//  $$B2S-INDRA-1,211,12:55:46,52.46739,-2.02130,201,10,0,0*F0D0
int sendNmeaUsb(received_t *t) {
 #define NMEA_STR_SIZE	220u
    bool valid = false;
	char *telemetry_str = t->Message;
	int satellites = 13, quality = 1;
	float hdop = 1.00;
	int nmea_str_len;

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
		char final_nmea_sentence[NMEA_STR_SIZE];
		nmea_str_len = snprintf(final_nmea_sentence, NMEA_STR_SIZE, "%s*%02X\r\n", nmea_sentence, checksum);

		if (nmea_str_len < (NMEA_STR_SIZE - 1)) {
			// Send NMEA sentence through USB
			//LogMessage("Sending NMEA string (%d): %.*s\n", nmea_str_len, nmea_str_len-2, final_nmea_sentence);
			sendStringUSB(final_nmea_sentence);
		} else {
			valid = false;
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

int sendMavlinkUsb(received_t *t) {
    bool valid = false;

    // Parse the custom telemetry string
    char *telemetry_str = t->Message;
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
        uint16_t gps_buffer_len = mavlink_msg_to_send_buffer(gps_buffer, &gps_msg);

        // Serialize the message
        uint8_t global_buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t global_buffer_len = mavlink_msg_to_send_buffer(global_buffer, &global_msg);

        // Send the message through USB
        // LogMessage("Sending MAVLink GPS_RAW_INT msg (%dB)\n", gps_buffer_len);
        sendUSB(gps_buffer, gps_buffer_len);
        // LogMessage("Sending MAVLink GLOBAL_POSITION_INT msg (%dB)\n", global_buffer_len);
        sendUSB(global_buffer, global_buffer_len);
    }
    return valid;
}

uint16_t calc_skylark_crc16(uint8_t *packet, uint8_t start, uint8_t end) {
    uint16_t CRC16Table[] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };

    uint16_t checksum = 0;
    for (size_t i = start; i < end; i++) {
        checksum = (uint16_t)((checksum >> 8) ^ CRC16Table[(checksum & 0xFF) ^ packet[i]]);
    }
    return checksum;
}

int sendSkylarkAATUsb(received_t *t) {
    //#define SKYLART_AAT_LONG
    typedef struct __attribute__((__packed__)) {
        uint8_t header1;
        uint8_t header2;
        uint8_t class;
        uint8_t id;
        uint16_t length;
        uint16_t AATPitch;  // Pitch(0-9000), Unit: degrees(0-90)*100
        int16_t AATYaw;     // Heading(-18000-18000), Unit: degrees(+/-180)*100
#ifdef SKYLART_AAT_LONG
        uint32_t longitude;
        uint32_t latitude;
        uint16_t altitude;
        uint16_t empty[3];
        uint8_t gps_num;
        uint8_t flag;
#endif
        uint16_t checksum;  // CRC16 CheckSum from Class to Payload end
    } SkylarkAAT_CMD;

    SkylarkAAT_CMD cmd = {
        .header1    = 0x54, // Defined by Skylark
        .header2    = 0x4D, // Defined by Skylark
        .class      = 0x0B, // Defined by Skylark
        .id         = 0x0B, // Defined by Skylark
        .length     = 0x16, // Payload length
#ifdef SKYLART_AAT_LONG
        .gps_num    = 10,   // Number of satellites
        .flag       = 0,    // Defined by Skylark
#endif
    };

    bool valid = false;
	char *telemetry_str = t->Message;

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

#ifdef SKYLART_AAT_LONG
        // Convert latitude and longitude to 1E7 format and altitude to millimeters
        int32_t lat_1E7 = (int32_t)(t->Telemetry.Latitude * 1E7);
        int32_t lon_1E7 = (int32_t)(t->Telemetry.Longitude * 1E7);
        int32_t alt_mm = t->Telemetry.Altitude * 1000;

        // Fill with telemetry coordinates
        cmd.AATPitch = 0;
        cmd.AATYaw = 0;
        cmd.latitude = lat_1E7;
        cmd.longitude = lon_1E7;
        cmd.altitude = t->Telemetry.Altitude;
#else
        // Fill with tracking angle

        // Example values
        static uint16_t pitch;
        static int16_t yaw;
        pitch += 100; // +1º
        yaw += 100; // +1º
        if (pitch > 9000) {
            pitch = 0;
        }
        if (yaw > 18000) {
            yaw = -18000;
        }
        cmd.AATPitch = pitch;
        cmd.AATYaw = yaw;
#endif

        cmd.checksum = calc_skylark_crc16((uint8_t *)&cmd, offsetof(SkylarkAAT_CMD, class), offsetof(SkylarkAAT_CMD, checksum));

		// Send Skylark AAT through USB
		//LogMessage("Sending Skylark AAT cmd\n");
        sendUSB((uint8_t *) &cmd, sizeof(SkylarkAAT_CMD));
	}

	if (valid) {
		return sizeof(SkylarkAAT_CMD); // Success
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
                // Create a function pointer and assign it to either sendNmeaUsb or sendMavlinkUsb
                GPSUsbSendFunction gpsUsbSendFunc;
                if (strcmp(Config.GPSUSBOutput, "MAVLINK") == 0) {
                    gpsUsbSendFunc = &sendMavlinkUsb; // Output Mavlink format
                    LogMessage("USB output on %s: MAVlink\n", Config.GPSUSBPort);
                } else if (strcmp(Config.GPSUSBOutput, "SkylarkAAT") == 0) {
                    gpsUsbSendFunc = &sendSkylarkAATUsb; // Output Skylark AAT Custom format
                    LogMessage("USB output on %s: Skylark AAT\n", Config.GPSUSBPort);
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
