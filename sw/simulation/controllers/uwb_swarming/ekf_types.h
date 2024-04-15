/**
 * @file ekf_types.h
 * @author Sven Pfeiffer
 * @date 15 Apr 2024
 * @brief types and definitions for the relative position ekf
 * 
 * This header file contains:
 * - type definitions that are used to exchange data with the relative position ekf
 * - defines for constants that are used to work with the different ekfs
 * 
 * These definitions are included through this header file so that other
 * classes can prepare data for the ekf (inputs & measurements) or use 
 * results from the ekf (states).
 * 
 * Using this separate header file avoids circular inclusions and keeps the
 * use of the ekf class header file to a minimum.
 * 
 */
#ifndef EKF_TYPES_H
#define EKF_TYPES_H

#include <stdint.h>

/**
 * Estimator variants for comparison
 */
#define ESTIMATOR_NONE 0
#define ESTIMATOR_EKF_REF 1
#define ESTIMATOR_EKF_FULL 2
#define ESTIMATOR_EKF_DYNAMIC 3
#define ESTIMATOR_EKF_DECOUPLED 4
#define ESTIMATOR_MAX 5

/**
 * Indices of specific states in the ekf state space
 */
#define EKF_ST_X    0
#define EKF_ST_Y    1
#define EKF_ST_DIM  2   // Number of states per agent in the ekf

/**
 * Indices of specific inputs in the ekf input space
 */
#define EKF_IN_VX   0
#define EKF_IN_VY   1
#define EKF_IN_DIM  2   // Number of inputs per agent in the ekf


/**
 * @brief data for the ekf prediction step
 */
struct ekf_input_t {
  uint16_t id;
  float vx;
  float vy;
  float timestamp;
};

/**
 * @brief a range measurement between two agents
 */
struct ekf_range_measurement_t {
  uint16_t id_A;
  uint16_t id_B;
  float range;
  float timestamp;
};

/**
 * @brief data needed to initialize a newly encountered agent in the ekf
 */
struct agent_initialization_data_t {
  uint16_t id;
  float x0;
  float y0;
  float stdev_x;
  float stdev_y;
  float timestamp;
};

#endif //EKF_TYPES_H