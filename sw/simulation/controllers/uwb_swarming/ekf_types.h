#ifndef EKF_TYPES_H
#define EKF_TYPES_H

#include <stdint.h>

#define EKF_ST_X    0
#define EKF_ST_Y    1
#define EKF_ST_PSI  2
#define EKF_ST_DIM  3

#define EKF_IN_PX   0
#define EKF_IN_PY   1
#define EKF_IN_DPSI 2
#define EKF_IN_DIM  3


struct ekf_input_t {
  uint16_t id;
  float rhoX;
  float rhoY;
  float dPsi;
  float rssi;
  float timestamp;
};

struct ekf_range_measurement_t {
  uint16_t id_A;
  uint16_t id_B;
  float range;
  float timestamp;
};


#endif //EKF_TYPES_H