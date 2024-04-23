#ifndef _AGENT_INITIALIZER_H_
#define _AGENT_INITIALIZER_H_

#include <stdint.h>

#include "ekf_types.h"

#define AGENT_INITIALIZER_MAX_SLOTS 5 // Number of agents that are initialized at the same time
#define MULTILAT_MAX_POINTS 4 // Max number of points for multilateration initialization
#define INITIALIZER_PERIOD 1
#define AGENT_INITIALIZER_TIMEOUT 3.0f // time to free initializer when not all required information is received

struct multilat_point_t{
    uint16_t id_B;
    float x;
    float y;
    float stdev_x;
    float stdev_y;
    float range;
    float timestamp;
};

class AgentInitializer
{
private:
    uint16_t _self_id;
    bool _is_used[AGENT_INITIALIZER_MAX_SLOTS];
    uint16_t _agent_id[AGENT_INITIALIZER_MAX_SLOTS];
    float _time_added[AGENT_INITIALIZER_MAX_SLOTS];

    // trajectory based initialization
    float _traj_relX_accum[AGENT_INITIALIZER_MAX_SLOTS];
    float _traj_relY_accum[AGENT_INITIALIZER_MAX_SLOTS];
    float _traj_last_accum_time[AGENT_INITIALIZER_MAX_SLOTS];
    float _traj_start_time[AGENT_INITIALIZER_MAX_SLOTS];
    float _traj_start_range[AGENT_INITIALIZER_MAX_SLOTS];
    float _traj_end_range[AGENT_INITIALIZER_MAX_SLOTS];
    bool initial_position_from_traj(const uint16_t id, const float time_now, agent_initialization_data_t* init_data);


    // multilateration based initialization
    multilat_point_t _multilat_pts[AGENT_INITIALIZER_MAX_SLOTS][MULTILAT_MAX_POINTS];

    bool initial_position_from_multilat(const uint16_t id, const float time_now, agent_initialization_data_t* init_data);

    void reset_slot(const uint16_t slot_idx);
    bool get_index(const uint16_t id, uint16_t *index);
    bool add_agent(const uint16_t id, const float time_now, uint16_t *index);

public:
    AgentInitializer(const uint16_t self_id);

    void add_velocities(const uint16_t id, const float rel_vx, const float rel_vy, const float time);
    void add_direct_range(const uint16_t id, const float range, const float time);
    void add_multilat_point(const uint16_t id, const multilat_point_t &new_point);
    bool get_initial_position(const uint16_t id, const float time_now, agent_initialization_data_t* init_data);

};


#endif // _AGENT_INITIALIZER_H_