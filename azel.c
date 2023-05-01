#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ctype.h>
#include <stdio.h>              // Standard input/output definitions
#include <string.h>             // String function definitions
#include <stdlib.h>             // String function definitions
#include <math.h>             // String function definitions
#include "azel.h"
#include "global.h"

float calcEarthRadiusInMeters(float latitudeRadians)
{
    // latitudeRadians is geodetic, i.e. that reported by GPS.
    // http://en.wikipedia.org/wiki/Earth_radius
    float a = 6378137.0;  // equatorial radius in meters
    float b = 6356752.3;  // polar radius in meters
    float c = cos(latitudeRadians);
    float s = sin(latitudeRadians);
    float t1 = a * a * c;
    float t2 = b * b * s;
    float t3 = a * c;
    float t4 = b * s;
    return sqrt((t1*t1 + t2*t2) / (t3*t3 + t4*t4));
}

float calcGeocentricLatitude(float lat)
{
    // Convert geodetic latitude 'lat' to a geocentric latitude 'clat'.
    // Geodetic latitude is the latitude as given by GPS.
    // Geocentric latitude is the angle measured from center of Earth between a point and the equator.
    // https://en.wikipedia.org/wiki/Latitude#Geocentric_latitude
    float e2 = 0.00669437999014;
    float clat = atan((1.0 - e2) * tan(lat));
    return clat;
}


float calcDistance (vector8 ap, vector8 bp)
{
    float dx = ap.x - bp.x;
    float dy = ap.y - bp.y;
    float dz = ap.z - bp.z;
    return sqrt (dx*dx + dy*dy + dz*dz);
}

struct vector8 convertLocationToPoint(struct vector8 l)
{
    // Convert (lat, lon, elv) to (x, y, z).
    float lat = l.x * M_PI / 180.0;
    float lon = l.y * M_PI / 180.0;
    float radius = calcEarthRadiusInMeters(lat);
    float clat   = calcGeocentricLatitude(lat);

    float cosLon = cos(lon);
    float sinLon = sin(lon);
    float cosLat = cos(clat);
    float sinLat = sin(clat);

    struct vector8 p;
    p.x = radius * cosLon * cosLat;
    p.y = radius * sinLon * cosLat;
    p.z = radius * sinLat;

    // We used geocentric latitude to calculate (x,y,z) on the Earth's ellipsoid.
    // Now we use geodetic latitude to calculate normal vector from the surface, to correct for elevation.
    float cosGlat = cos(lat);
    float sinGlat = sin(lat);

    float nx = cosGlat * cosLon;
    float ny = cosGlat * sinLon;
    float nz = sinGlat;

    p.nx = nx;
    p.ny = ny;
    p.nz = nz;

    p.x += l.z * nx;
    p.y += l.z * ny;
    p.z += l.z * nz;

    // check for possible Z NaN
    if (isnan(p.z)) {p.z = 0.0;}

    p.radius = radius;

    return p;

}

struct vector8 rotateGlobe (vector8 b, vector8 a, float bradius, float aradius)
{
    // Get modified coordinates of 'b' by rotating the globe so that 'a' is at lat=0, lon=0.
    vector8 br;
    br.x = b.x;
    br.y = b.y - a.y;
    br.z = b.z;
    vector8 brp = convertLocationToPoint(br);

    // Rotate brp cartesian coordinates around the z-axis by a.lon degrees,
    // then around the y-axis by a.lat degrees.
    // Though we are decreasing by a.lat degrees, as seen above the y-axis,
    // this is a positive (counterclockwise) rotation (if B's longitude is east of A's).
    // However, from this point of view the x-axis is pointing left.
    // So we will look the other way making the x-axis pointing right, the z-axis
    // pointing up, and the rotation treated as negative.

    float alat = calcGeocentricLatitude(-a.x * M_PI / 180.0);
    float acos = cos(alat);
    float asin = sin(alat);

    vector8 rg;

    rg.x = (brp.x * acos) - (brp.z * asin);
    rg.y = brp.y;
    rg.z = (brp.x * asin) + (brp.z * acos);
    rg.radius = bradius;

    return rg;
}

struct vector8 normalizeVectorDiff(vector8 b, vector8 a)
{
    vector8 nvd;

    nvd.err = false;

    // Calculate norm(b-a), where norm divides a vector by its length to produce a unit vector.
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float dz = b.z - a.z;
    float dist2 = dx*dx + dy*dy + dz*dz;
    if (abs(dist2) < 0.00001) {
        nvd.err = true;
        return nvd;
    }
    float dist = sqrt(dist2);
    nvd.x = (dx/dist);
    nvd.y = (dy/dist);
    nvd.z = (dz/dist);
    nvd.radius = 1.0;
    return nvd;
}

