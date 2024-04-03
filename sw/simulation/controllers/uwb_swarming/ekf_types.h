#ifndef EKF_TYPES_H
#define EKF_TYPES_H

#include <stdint.h>

#define EKF_ST_X    0
#define EKF_ST_Y    1
#define EKF_ST_DIM  2

#define EKF_IN_VX   0
#define EKF_IN_VY   1
#define EKF_IN_DIM  2


struct ekf_input_t {
  uint16_t id;
  float vx;
  float vy;
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