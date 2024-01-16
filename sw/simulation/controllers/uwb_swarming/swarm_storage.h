#ifndef AGENT_STATE_STORAGE_H
#define AGENT_STATE_STORAGE_H

#include <vector>
#include <stdint.h>

#include "uwb_channel.h"
#include "ekf_types.h"
#include "swarm_ranging.h"


#define LINK_NONE 0
#define LINK_PING 1
#define LINK_RANGE 2
#define LINK_EKF 3

#define LINK_TIMEOUT 1 // s

typedef struct agent_state_s{
  float rhoX;
  float rhoY;
  float dPsi; // rad/s

  float relX;
  float relY;
  float relPsi;
}agent_state_t;

typedef struct agent_info_s{
  uint16_t id;
  float range;
  float last_range;

  float rssi;
  float last_seen;

  // uwb_time_t tx_time[4];
  // uwb_time_t rx_time[4];
  // uint8_t seq[4];

  agent_state_t state;
}agent_info_t;


class SwarmStorage
{
private:
    random_generator rg;
    uint16_t _self_id;
    // uwb_time_t _tx_history[TX_HISTORY_LEN]; // tx timestamps of previous messages (most recent first)
    // uint8_t _seq_history[TX_HISTORY_LEN];  // sequence numbers of previous messages (most recent first)
public:
    std::vector<agent_info_t> _agents;
    std::vector<ekf_range_measurement_t> _range_measurements;
    std::vector<ekf_range_measurement_t> _indirect_ranges; // used for animation

    SwarmStorage();

    ~SwarmStorage() {};

    /**
     * Initialize the swarm storage with the agent's own id
     */
    void init(const uint16_t ID);
    
    /**
     * Check if this agent has an entry
     */
    bool is_agent_known(const uint16_t agent_id);

    /**
     * Return index at which the agent of given id is stored in
     * the storage vector
     */
    uint16_t get_storage_index(const uint16_t agent_id);

    /**
     * Update the entry of the agent with given id with a new state
     * (ideally from a state estimator)
     */
    void update_storage_state(const uint16_t agent_id, const agent_state_t &new_state);
    
    /** 
     * Checks if any agents were not seen in a while and removes them
     * from the storage vector. Call this function periodically to avoid
     * excessive memory usage.
     */ 
    void update_storage_periodic(const float current_time_seconds);

    /**
     * Order the storage vector from closest to furthest based on stored
     * rssi values.
     */
    void order_by_rssi();

    /**
     * Returns the number of agents currently stored
     */
    uint16_t agents_in_range();

    /**
     * Request the N first agents in the storage vector. Order the
     * vector first according to a desired criterion (rssi, estimated range)
     * Returns actual number of agent ids returned
     */
    uint16_t get_closest_agents(const uint16_t N_agents, std::vector<uint16_t> &agent_ids);

    /**
     * Request N random ids of agents in range.
     * Returns actual number of agent ids returned
     */
    uint16_t get_random_agents(const uint16_t N_agents, std::vector<uint16_t> &agent_ids);

    /**
     * Return number and ids of all agents in range
     */
    uint16_t get_all_agents(std::vector<uint16_t> &agent_ids);

    /**
     * Return agents and RSSI values
     */
    uint16_t get_all_rssi(std::vector<uint16_t> &agent_ids, std::vector<float> &rssi);

    // /**
    //  * Calculate and enqueue range measurements
    //  */
    // void calculate_range_measurements(const float current_time_secondes, std::vector<ekf_range_measurement_t> *measurements);

    // /**
    //  * Build a new ranging ping
    //  */

    // void create_ranging_ping(swarm_ranging_ping_t &tx_ping, ekf_input_t &own_input);

    // /**
    //  * Update agent info with a newly received ranging ping
    //  */
    // void process_ranging_ping(const float current_time_seconds, const uwb_time_t rx_time, const swarm_ranging_ping_t msg, const float rssi);

   
};


#endif //AGENT_STATE_STORAGE_H