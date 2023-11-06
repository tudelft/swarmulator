#ifndef _SWARM_RANGING_H_
#define _SWARM_RANGING_H_

#include <stdint.h>
#include <vector>

#include "randomgenerator.h"
#include "ekf_types.h"

#define MAX_AGENTS_IN_PING 9
#define SRP_SIZE_BITS(n_agents) (288 + n_agents*80)

#define TX_HISTORY_LEN 10

#define POLL 0
#define RESPONSE 1
#define FINAL 2
#define REPORT 3

#define SPEED_OF_LIGHT 299792458 // m/s

/**
 * IMPORTANT: Timestamps are NOT accurate.
 * They are only used to replicate as much of the
 * on-board code as possible. The actual distance 
 * measurements are generated from the groundtruth.
 * Here, a value of 1 is used to indicate that there is 
 * a valid timestamp.
 */ 
typedef struct uwb_time_s
{
  union
  {
    uint8_t raw[5];
    uint64_t full;
  };
}uwb_time_t;

/**
 * Definition of the swarm ranging ping, exchanged via UWB:
 *  Header..............................(21 Bytes)
 *  Number of Agent blocks in the ping..(1 Byte)
 *  Last Sender (source) Information....(14 Bytes)
 *  Last Agent Information Block........(10 Bytes each)
 * 
 * Total Size: 36 + 10N Bytes
 * Max Size (9 agent blocks): 126 Bytes
*/

typedef struct srp_source_block_s
{
  uint8_t seq;
  uint8_t last_tx[5];
  int16_t velocities[3]; // in mm/s
  int16_t yawrate;       // in mrad/s
} srp_source_block_t;

typedef struct srp_agent_block_s
{
    uint16_t id;
    uint8_t last_seq;
    uint8_t last_rx[5];
    uint16_t last_range;    // in mm
} srp_agent_block_t;

typedef struct swarm_ranging_ping_s{
  union
  {
    uint8_t header[21];
    struct
    {
      uint16_t fcf;
      uint8_t seq;
      uint16_t pan;
      uint64_t destAddress;
      uint64_t sourceAddress;
    } header_s;
  };

  uint8_t n_agents;
  srp_source_block_t sender;
  srp_agent_block_t agents[MAX_AGENTS_IN_PING];
}swarm_ranging_ping_t;


/**
 * Agent ranging tables
 */

typedef struct ranging_table_s{
  uint16_t id;
  uint16_t last_range_mm;
  float t_last_seen;
  float t_last_range;
  float rssi;

  uwb_time_t tx_time[4];
  uwb_time_t rx_time[4];
  uint8_t seq[4];
}ranging_table_t;

class RangingEvaluator
{
private:
  uint64_t _airtime_accum_ns;
  uint64_t _total_airtime_ns;
  float _time_elapsed;
  float _last_load_calculation;
  
public:
  float _current_load;
  float _avg_load;

  RangingEvaluator();

  ~RangingEvaluator() {};

  void add_message_by_payload(uint16_t payload_bits);

  void calculate_load();

};


/**
 * The Swarm Ranging Module which handles UWB communication
 * and calculation of ranges
 */

class SwarmRanging
{
private:
  random_generator _rg;
  uint16_t _self_id;
  uwb_time_t _tx_history[TX_HISTORY_LEN];
  uint8_t _seq_history[TX_HISTORY_LEN];

  std::vector<ranging_table_t> _agents;
  std::vector<ekf_range_measurement_t> _range_measurements;
  std::vector<ekf_range_measurement_t> _animation_buffer;

  void order_by_rssi();

  uint16_t get_storage_index(const uint16_t id);

  /**
   * Process a swarm ranging ping
   */
  void process_ranging_ping(uwb_time_t &rx_time, const swarm_ranging_ping_t &msg, float rssi, std::vector<ekf_input_t> &inputs);

  /**
   * Calculate direct ranges from available timestamps in the 
   * ranging tables, and enqueues them in the _range_measurements vector
   */
  void calculate_direct_ranges();

 /**
   * Set tx time of previously transmitted ping based on seq number
   * Returns false if sequence number unknown
   */
  bool get_tx_time(const uint8_t seq, uwb_time_t &tx_time);

  /**
   * Store timestamps from a ping used as "response" from another
   * agent. This happens for initialization of the ranging table, or
   * if there was a sequence number mismatch.
   */
  void add_timestamps_response(ranging_table_t &agentB, const uwb_time_t &msg_rx, const srp_source_block_t &src, const srp_agent_block_t &msg_agent_block);

  /**
   * Store timestamps from a ping used as "report" from another
   * agent. This happens in normal operation.
   */
  void add_timestamps_report(ranging_table_t &agentB, const uwb_time_t &msg_rx, const srp_source_block_t &src, const srp_agent_block_t &msg_agent_block);

  /**
   * Clears timestamps from the ranging table to start a new ranging
   * sequence. Should be used if communication was interrupted for too long
   */
  void clear_timestamps(ranging_table_t &agentB);

  /**
   * Shifts timestamps when ranging table is full (FINAL > POLL, REPORT > RESPONSE)
   */
  void shift_timestamps(ranging_table_t &agentB);
public:
  RangingEvaluator _evaluator;

  SwarmRanging();

  ~SwarmRanging() {};

  /**
   * Initialize the swarm ranging with the agent's own id
   */
  void init(const uint16_t ID);

  void send_ranging_ping(float rhoX, float rhoY, float dPsi);

  void receive_new_ranging(std::vector<ekf_range_measurement_t> &ranges, std::vector<ekf_input_t> &inputs);

  void receive_all_ranging(std::vector<ekf_range_measurement_t> &ranges, std::vector<ekf_input_t> &inputs);
  
  void rel_loc_animation(const uint16_t ID);

};


#endif // _SWARM_RANGING_H_