/**
 * @file antennatracker.c
 * @author bruno.santamaria@b2-space.com
 * @brief Uses received coordinates plus local position to calculate azimuth and elevation angles
 * @date 2023-09-27
 * 
 */

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
#include <time.h>

#include "antennatracker.h"
#include "global.h"

#define GW_NUM_CHANNELS     2u

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)
#define EARTH_RADIUS 6371000 // Earth's radius in meters

typedef struct {
    double lat;
    double lon;
    double alt;
} TPosition;

static double calculate_distance(double lat1, double lon1, double lat2, double lon2);
static double calculate_3d_distance(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2);
static double calculate_bearing(double lat1, double lon1, double lat2, double lon2);
static double calculate_elevation(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2);

static TPosition gateway_position;
static bool gateway_position_rx = false;
static TPosition object_position[GW_NUM_CHANNELS];
static bool object_position_rx[GW_NUM_CHANNELS] = {false, false};

#define TRACKING_ANGLES_FILE "tracking_angles.txt"

// Function to calculate the distance between two points considering Earth's curvature
static double calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;

    double a = pow(sin(dLat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dLon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance = EARTH_RADIUS * c;

    return distance;
}

// Function to calculate the 3D distance between two points considering Earth's curvature and altitude difference
static double calculate_3d_distance(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2) {
    double horizontal_distance = calculate_distance(lat1, lon1, lat2, lon2);
    double altitude_difference = alt2 - alt1;
    double distance = sqrt(pow(horizontal_distance, 2) + pow(altitude_difference, 2));
    return distance;
}

// Function to calculate the bearing angle (azimuth)
static double calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = lon2 - lon1;
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double bearing = atan2(y, x);
    bearing = fmod(bearing * RAD_TO_DEG + 360.0, 360.0);
    return bearing;
}

// Function to calculate the elevation angle
static double calculate_elevation(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2) {
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double dAlt = alt2 - alt1;

    double a = pow(sin(dLat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dLon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance = EARTH_RADIUS * c; // Earth's radius in meters

    double elevation = atan2(dAlt, distance) * RAD_TO_DEG;
    return elevation;
}

void anttrack_set_gateway_position(double lat, double lon, double alt) {
    gateway_position.lat = lat * DEG_TO_RAD;
    gateway_position.lon = lon * DEG_TO_RAD;
    gateway_position.alt = alt;
    gateway_position_rx = true;
    if (Config.AntTrackDebug) {
        LogMessage("Gateway pos rx: lat %.3f lon %.3f alt %.1f\n", lat, lon, alt);
    }
}

void anttrack_set_object_position(double lat, double lon, double alt, unsigned int channel) {
    if (channel < GW_NUM_CHANNELS) {
        object_position[channel].lat = lat * DEG_TO_RAD;
        object_position[channel].lon = lon * DEG_TO_RAD;
        object_position[channel].alt = alt;
        object_position_rx[channel] = true;
        if (gateway_position_rx) {
            double distance = calculate_distance(gateway_position.lat, gateway_position.lon,
                object_position[channel].lat, object_position[channel].lon);
            double distance_3d = calculate_3d_distance(gateway_position.lat, gateway_position.lon, gateway_position.alt,
                object_position[channel].lat, object_position[channel].lon, object_position[channel].alt);
            double bearing = calculate_bearing(gateway_position.lat, gateway_position.lon,
                object_position[channel].lat, object_position[channel].lon);
            double elevation = calculate_elevation(gateway_position.lat, gateway_position.lon, gateway_position.alt,
                object_position[channel].lat, object_position[channel].lon, object_position[channel].alt);
            if (Config.AntTrackDebug) {
                LogMessage("AntennaTracker pos rx ch %d: lat %.3f lon %.3f alt %.1f\n", channel, lat, lon, alt);
            }
            LogMessage("-Track Info: Dist: %.0fm", distance);
            LogMessage(", 3D Dist: %.0fm", distance_3d);
            LogMessage(", azimuth: %.0fº", bearing);
            LogMessage(", elevation: %.0fº\n", elevation);
            
            if (Config.AntTrackLog) {
                // Save to file
                FILE *fp;

                if ( ( fp = fopen( TRACKING_ANGLES_FILE, "at" ) ) != NULL )
                {
                    time_t now;
                    struct tm *tm;

                    now = time( 0 );
                    tm = localtime( &now );
                    if (Config.AntTrackDebug) {
                        fprintf( fp, "%02d:%02d:%02d - Track Debug: using "
                        "Gw lat %.3lf lon %.3lf alt %.0lfm,"
                        " obj: lat %.3lf lon %.3lf alt %.0lfm\n",
                        tm->tm_hour, tm->tm_min, tm->tm_sec,
                        gateway_position.lat, gateway_position.lon, gateway_position.alt,
                        object_position[channel].lat, object_position[channel].lon, object_position[channel].alt);
                    }
                    fprintf( fp, "%02d:%02d:%02d - Track Info: Dist: %.0fm"
                    ", 3D Dist: %.0fm"
                    ", azimuth: %.0fº"
                    ", elevation: %.0fº\n",
                    tm->tm_hour, tm->tm_min, tm->tm_sec,
                    distance, distance_3d, bearing, elevation);
                    fclose( fp );
                }
            }
        }
    }
}

void anttrack_set_object_telemetry(char *telemetry, unsigned int channel) {
    double lat, lon, alt;
    int alt_int;

    // Parse 4th 5th and 6th fields of telemetry as lat, lon, alt
    // TODO: telemetry may vary. Set this in gateway.txt config
    int num_params = sscanf(telemetry, "%*[^,],%*[^,],%*[^,],%lf,%lf,%d", &lat, &lon, &alt_int);

    if (num_params == 3 && (lat != 0.0 || lon != 0.0 || alt_int != 0)) {
        alt = alt_int;
        anttrack_set_object_position(lat, lon, alt, channel);
    }
}