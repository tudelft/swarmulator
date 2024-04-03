#ifndef _AGENT_INITIALIZER_H_
#define _AGENT_INITIALIZER_H_

#include <stdint.h>


#define AGENT_INITIALIZER_MAX_SLOTS 5 // Number of agents that are initialized at the same time
#define INITIALIZER_PERIOD 1
#define AGENT_INITIALIZER_TIMEOUT 2.0f // time to free initializer when not all required information is received

class AgentInitializer
{
private:
    uint16_t _self_id;
    bool _is_used[AGENT_INITIALIZER_MAX_SLOTS];
    uint16_t _agent_id[AGENT_INITIALIZER_MAX_SLOTS];
    float _relX_accum[AGENT_INITIALIZER_MAX_SLOTS];
    float _relY_accum[AGENT_INITIALIZER_MAX_SLOTS];
    float _start_time[AGENT_INITIALIZER_MAX_SLOTS];
    float _last_time[AGENT_INITIALIZER_MAX_SLOTS];
    float _start_range[AGENT_INITIALIZER_MAX_SLOTS];
    float _end_range[AGENT_INITIALIZER_MAX_SLOTS];

    void reset_slot(const uint16_t slot_idx);
    bool get_index(const uint16_t id, uint16_t *index);

public:
    AgentInitializer(const uint16_t self_id);

    void add_velocities(const uint16_t id, const float rel_vx, const float rel_vy, const float time);
    void add_range(const uint16_t id, const float range, const float time);

    bool get_initial_position(const uint16_t id, const float time_now, float *x0, float *y0, float *var_x, float *var_y);

};


#endif // _AGENT_INITIALIZER_H_