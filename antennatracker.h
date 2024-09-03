/**
 * @file antennatracker.h
 * @author bruno.santamaria@b2-space.com
 * @brief Uses received coordinates plus local position to calculate azimuth and elevation angles
 * @date 2023-09-27
 * 
 */

int anttrack_calc_nearer_lower_point(double lat1, double lon1, double alt1, double altitude_limit, double *lat2, double *lon2, double *alt2);
void anttrack_set_gateway_position(double lat, double lon, double alt);
void anttrack_set_object_position(double lat, double lon, double alt, unsigned int channel, char *object_name);
void anttrack_set_object_telemetry(char *telemetry, unsigned int channel);
void *anttrack_loop(void *void_ptr);
