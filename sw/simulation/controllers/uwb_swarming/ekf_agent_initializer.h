/**
 * @file ekf_agent_initializer.h
 * @author Sven Pfeiffer, MAVLab, TU Delft
 * @date 23 Apr 2024
 * @brief Contains the AgentInitializer class, used to calculate
 * initial states for the relative ekf
 * 
 * The AgentInitializer has several slots in which it collects
 * the most recent data on agent states. Based on the data
 * available, it calculates an initial position based on
 * agent trajectory, or multilateration.
 * 
 * Trajectory based initialization makes use of two range
 * measurements and the accumulated change in relative position
 * between the range measurements. From the resulting triangle, 
 * two possible initial positions can be calculated, and the
 * middle point between them is returned.
 * 
 * Multilateration based initialization relies on range 
 * measurements to at least 3 other drones.
 * 
 */

#ifndef _AGENT_INITIALIZER_H_
#define _AGENT_INITIALIZER_H_

#include <stdint.h>

#include "ekf_types.h"

// Number of agents that are initialized at the same time
#define AGENT_INITIALIZER_MAX_SLOTS 1 

// Minimum accummulation time for trajcetory based initialization 
#define TRAJECTORY_ACCUM_TIME 2

// Min/Max number of points for multilateration initialization
#define MULTILAT_MAX_POINTS 5
#define MULTILAT_MIN_POINTS 3 

// time after which the initialization slot is reset
#define AGENT_INITIALIZER_TIMEOUT 3.0f 

/**
 * Structure to add measurements for multilateration
 */
struct multilat_point_t{
    uint16_t id_B;      // ID of the second drone
    float x;            // relative x coordinate of 2nd drone
    float y;            // relative y coordinate of 2nd drone
    float stdev_x;      // standard deviation on x
    float stdev_y;      // standard deviation on y
    float range;        // distance to 2nd drone
    float timestamp;    // time of measurement
};


/**
 * @brief Class to calculate initial positions for agents in the
 * relative localization ekf.
 * 
 * Data is saved in vectors with initialization slots. In every 
 * vector, the data at a given index belongs to the same agent, 
 * the id of which is saved in the _agent_id vector.
 */
class AgentInitializer
{
private:
    uint16_t _self_id; // id of the drone running the estimator
    
    // Slot management data
    bool _is_used[AGENT_INITIALIZER_MAX_SLOTS];
    uint16_t _agent_id[AGENT_INITIALIZER_MAX_SLOTS];
    float _time_added[AGENT_INITIALIZER_MAX_SLOTS];

    // data for trajectory based initialization
    float _traj_relX_accum[AGENT_INITIALIZER_MAX_SLOTS];
    float _traj_relY_accum[AGENT_INITIALIZER_MAX_SLOTS];
    float _traj_last_accum_time[AGENT_INITIALIZER_MAX_SLOTS];
    float _traj_start_time[AGENT_INITIALIZER_MAX_SLOTS];
    float _traj_start_range[AGENT_INITIALIZER_MAX_SLOTS];
    float _traj_end_range[AGENT_INITIALIZER_MAX_SLOTS];

    // points for multilateration based initialization
    multilat_point_t _multilat_pts[AGENT_INITIALIZER_MAX_SLOTS][MULTILAT_MAX_POINTS];

    /**
     * @brief calculate an initial position using the trajectory method
     * 
     * To be able to calculate an initial position from trajectory data,
     * at least two ranges are needed (added with the 'add_direct_range'
     * function) in addition to the velocity between those (added with the
     * 'add_velocity' function).
     * @param[in] id: ID of the agent to calculate the position of
     * @param[in] time_now: Time of initialization
     * @param[out] init_data: initialization data structure containing initial position info
     * @return True if a position was successfully calculated, false otherwise
     */
    bool initial_position_from_traj(const uint16_t id, const float time_now, agent_initialization_data_t* init_data);

    /**
     * @brief calculate an initial position using the multilateration method
     * 
     * Requires at least 3 multilateration points (in 2D), one of which can
     * be the distance to this drone itself. Data can be added with the 
     * 'add_direct_range' (range to this drone only) and 'add_multilat_point'
     * (any range measurement) functions
     * @param[in] id: ID of the agent to calculate the position of
     * @param[in] time_now: Time of initialization
     * @param[out] init_data: initialization data structure containing initial position info
     * @return True if a position was successfully calculated, false otherwise
     */
    bool initial_position_from_multilat(const uint16_t id, const float time_now, agent_initialization_data_t* init_data);

    /**
     * @brief reset an initialization slot after successful initialization
     * or timeout
     * 
     * @param[in] slot_idx: Index (NOT ID) of the slot to reset
     */
    void reset_slot(const uint16_t slot_idx);
    
    /**
     * @brief return the index of the initialization slot used for a given agent id
     * 
     * @param[in] id: ID of the agent in question
     * @param[out] index: Index of the initialization slot
     * @return True if agent is known (i.e. there is already an initialization slot for it)
     */
    bool get_index(const uint16_t id, uint16_t *index);

    /**
     * @brief add a new agent to be tracked by the initializer
     * 
     * Adds the agent to an empty initialization slot and returns its index
     * @param[in] id: ID of the new agent
     * @param[in] time_now: current time (for timeout tracking)
     * @param[out] index: index at which the new agent was added
     * @return True if there was space to add the agent successfully
     */
    bool add_agent(const uint16_t id, const float time_now, uint16_t *index);

public:
    /**
     * @brief Create an AgentInitializer that can calculate an initial
     * position for newly encountered agents in the relative localization ekf
     * @param[in] self_id: id of the agent running the ekf
     */
    AgentInitializer(const uint16_t self_id);

    /**
     * @brief add relative velocities to use for trajectory based initialization
     * 
     * @param[in] id: ID of the agent in question
     * @param[in] rel_vx: relative velocity in x direction
     * @param[in] rel_vy: relative velocity in y direction
     * @param[in] time: current time
     */
    void add_velocities(const uint16_t id, const float rel_vx, const float rel_vy, const float time);
    
    /**
     * @brief Add a direct range measurement from this drone to the tracked drone
     * 
     * Two of these measurements can be used with velocity information for
     * trajectory based initialization, or one of these with two more
     * multilateration points for multilateration based initialization.
     * @param[in] id: ID of the agent in question
     * @param[in] range: distance to the agent in question
     * @param[in] time: time of the measurement
     */
    void add_direct_range(const uint16_t id, const float range, const float time);
    
    /**
     * @brief Add a new distance measurement for multilateration
     * 
     * 3 Multilateration points to different agents are needed for successful
     * initialization based on multilateration.
     * @param[in] id: ID of the tracked agent
     * @param[in] multilat_point: contains distance and position of the other
     * agent.
     */
    void add_multilat_point(const uint16_t id, const multilat_point_t &new_point);
    
    /**
     * @brief Return initial position for an agent if possible, based on available 
     * data
     * 
     * Prioritizes multilateration, but if not enough points are available, uses
     * trajectory instead.
     * @param[in] id: ID of the agent to initialize
     * @param[in] time_now: current time
     * @param[out] init_data: Initial position data
     * @return True if an initial position could be calculated from the available data
     */
    bool get_initial_position(const uint16_t id, const float time_now, agent_initialization_data_t* init_data);

};


#endif // _AGENT_INITIALIZER_H_