#ifndef AZ_EL_H_
#define AZ_EL_H_

typedef struct azimuth_elevation_t {
    float azimuth;
    float elevation;
} azimuth_elevation_t;

typedef struct vector8 {
  float x;
  float y;
  float z;
  float radius;
  float nx;
  float ny;
  float nz;
  int err;
} vector8;

typedef struct lat_lon_el_t {
    float latitude;
    float longitude;
    float elevation;
} lat_lon_el_t;

struct vector8 normalizeVectorDiff(vector8 b, vector8 a);

#endif
