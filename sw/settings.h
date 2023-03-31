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

#define MEAS_NOISE_UWB 0.2   // stdev of measurement noise for TWR, in [m]
#define MEAS_NOISE_RHOX 0.2    // stdev of measurement noise on velocity (x), in [m/s]
#define MEAS_NOISE_RHOY 0.2    // stdev of measurement noise on velocity (y), in [m/s]
#define MEAS_NOISE_DPSI 0.1  // stdev of measurement noise on yaw rate in [rad/s]

#define COMMUNICATION_RANGE 10
#endif /*SETTINGS_H*/
