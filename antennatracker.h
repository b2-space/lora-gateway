/**
 * @file antennatracker.h
 * @author bruno.santamaria@b2-space.com
 * @brief Uses received coordinates plus local position to calculate azimuth and elevation angles
 * @date 2023-09-27
 * 
 */

void anttrack_set_gateway_position(double lat, double lon, double alt);
void anttrack_set_object_position(double lat, double lon, double alt, unsigned int channel);
void anttrack_set_object_telemetry(char *telemetry, unsigned int channel);
void *anttrack_loop(void *void_ptr);
