#ifndef SETTINGS_H
#define SETTINGS_H

/**
 * Holds some higher level settings of the simulator
 *
 */

// Use SEQUENTIAL to launch robots one by one with the specified interval (in seconds), rather than all at once
// #define SEQUENTIAL 5

/**
 * Noise in relative sensing
 */
// TODO: Make runtime variable
#define NOISE_R 0 // STDEV of noise in range, used by omniscient_observer
#define NOISE_B 0 // STDEV of noise in bearing, used by omniscient_observer


/**
 * Noise for ultra-wideband and relative EKF
 */
#define STDEV_INITIAL_POS_XY 100
#define STDEV_INITIAL_YAW 3

#define MEAS_NOISE_UWB 0.1f   // stdev of measurement noise for TWR, in [m]
#define MEAS_NOISE_RSSI 0.5f
#define MEAS_NOISE_VX 0.1f    // stdev of measurement noise on velocity (x), in [m/s]
#define MEAS_NOISE_VY 0.1f    // stdev of measurement noise on velocity (y), in [m/s]
#define MEAS_NOISE_DPSI 0.01f  // stdev of measurement noise on yaw rate in [rad/s]
#define MEAS_NOISE_PSI 0.01f  // stdev of measurement noise on yaw measurement [rad/s]

#define COMMUNICATION_RANGE 20
#define COMMUNICATION_BASE_INTERVAL 0.05f
#define EKF_ESTIMATION_INTERVAL 0.01f
#endif /*SETTINGS_H*/
