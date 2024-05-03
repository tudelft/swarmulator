/**
 * @file swarm_ranging.h
 * @author Sven Pfeiffer, MAVLab, TU Delft
 * @date 03 May 2024
 * @brief Header file for the implementation of the swarm ranging
 * protocol
 * @see https://ieeexplore.ieee.org/abstract/document/9810917 
 * 
 * The swarm ranging protocol implemented in this header and the
 * associated source (.cpp) file follows the algorithm presented
 * by Shan et al. (2022). Changes to the packet structure were made
 * to include velocities of the source agent and past range measurements.
 * Furthermore, we keep a tx history to more efficiently deal with lost
 * or skipped messages.
 * 
 * Paper:
 * F. Shan, H. Huo, J. Zeng, Z. Li, W. Wu and J. Luo, 
 * "Ultra-Wideband Swarm Ranging Protocol for Dynamic and Dense Networks," 
 * in IEEE/ACM Transactions on Networking, vol. 30, no. 6, pp. 2834-2848, 
 * Dec. 2022, doi: 10.1109/TNET.2022.3186071
 */
#ifndef _SWARM_RANGING_H_
#define _SWARM_RANGING_H_

#include <stdint.h>
#include <vector>
#include <shared_mutex>

#include "randomgenerator.h"
#include "ekf_types.h"

// Keeping a history of past tx times & seq numbers
// to improve resilience to lost messages
#define TX_HISTORY_LEN 5

// Maximum total time for a ranging procedure (poll tx to report rx)
#define RANGING_MAX_PERIOD 0.5 // seconds

// index of msg types in the ranging tables
#define POLL 0
#define RESPONSE 1
#define FINAL 2
#define REPORT 3
#define MSG_TYPE_DIM 4

#define SPEED_OF_LIGHT 299792458 // m/s

/**
 * @brief uwb timestamp structure as it would appear
 * when implemented on hardware
 * 
 * IMPORTANT: Timestamps are NOT accurate enough to 
 * calculate ranges, they are only used to replicate 
 * as much of the on-board code as possible. The actual 
 * distance measurements are generated from the groundtruth.
 * A value of 0 is used to indicate an invalid timestamp
 */ 
typedef struct uwb_time_s
{
  union
  {
    uint8_t raw[5];
    uint64_t full;  // in ms
  };
}uwb_time_t;

/**
 * Definition of the swarm ranging ping, exchanged via UWB:
 *  Header..............................(21 Bytes)
 *  Number of Agent blocks in the ping..(1 Byte)
 *  Last Sender (source) Information....(12 Bytes)
 *  Last Agent Information Block........(10 Bytes each)
 * 
 * Total Size: 34 + 10N Bytes
 * Max Size (9 agent blocks): 124 Bytes
 * 
 * Maximum length of the frame prescribed by DW1000 user manual
 * Appendix 1 as 127 Bytes
*/
#define MAX_AGENTS_IN_PING 9
#define SRP_SIZE_BITS(n_agents) (272 + n_agents*80)
/**
 * @brief Contains information on the sender of
 * the packet
 */
typedef struct srp_source_block_s
{
  uint8_t seq;
  uint8_t last_tx[5];
  int16_t velocities[3]; // in mm/s
} srp_source_block_t;

/**
 * @brief Contains information on the last interaction
 * the sender had with another agent
 */
typedef struct srp_agent_block_s
{
    uint16_t id;          // ID of the other agent
    uint8_t last_seq;     // sequence number of last package received from agent
    uint8_t last_rx[5];   // reception time of last package from agent
    uint16_t last_range;  // last range calculated to agent in mm
} srp_agent_block_t;

/**
 * @brief UWB message format as implemented on the crazyflie.
 */
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
 * Agent ranging tables (c.f. Shan et al. 2022)
 */

typedef struct ranging_table_s{
  // additional information on the agent
  uint16_t id;
  uint16_t last_range_mm;
  int number_in_tx_queue;   // -1 if not in tx queue
  float t_last_seen;
  float t_last_range;
  float rssi;

  // The actual ranging table
  uwb_time_t tx_time[MSG_TYPE_DIM]; 
  uwb_time_t rx_time[MSG_TYPE_DIM];
  uint8_t seq[MSG_TYPE_DIM];
}ranging_table_t;

/**
 * @brief Observer class that collects statistics
 * on the uwb channel use, to calculate the air 
 * utilization
 */
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
  
  /**
   * @brief Register data that is transmitted on 
   * the channel
   * 
   * @param[in] payload_bits: number of bits in the data
   */
  void add_message_by_payload(uint16_t payload_bits);

  /**
   * @brief Update the current and average load 
   * from the registered data in the channel
   * 
   * @param[in] time: Current time
   */
  void calculate_load(const float time);

};


/**
 * @brief The Swarm Ranging Module which handles UWB communication
 * and calculation of ranges
 */
class SwarmRanging
{
private:
  random_generator _rg;
  uint16_t _self_id;
  uwb_time_t _tx_history[TX_HISTORY_LEN];
  uint8_t _seq_history[TX_HISTORY_LEN];
  std::shared_mutex _srp_buffer_mutex;

  int _agents_in_tx_queue;
  std::vector<ranging_table_t> _agents;
  std::vector<ekf_input_t> _input_buffer;
  std::vector<ekf_range_measurement_t> _direct_range_buffer;
  std::vector<ekf_range_measurement_t> _secondary_range_buffer;
  std::vector<ekf_range_measurement_t> _animation_buffer;
  std::vector<swarm_ranging_ping_t> _srp_buffer;
  std::vector<float> _rssi_buffer;
  std::vector<swarm_ranging_ping_t> _srp_buffer_not_in_range;

  void order_by_rssi();

  uint16_t get_storage_index(const uint16_t id);

  /**
   * Process a swarm ranging ping
   */
  void process_ranging_ping(uwb_time_t &rx_time, const swarm_ranging_ping_t &msg, float rssi, std::vector<ekf_input_t> &inputs, const float time);

  /**
   * Check if the tx time for the given sequence number is needed,
   * and if so, add it (incoming messages)
   * @param[in] agentB: ranging table of the agent that sent the msg
   * @param[in] tx_time: time at which the msg was sent
   * @param[in] seq: sequence number of the msg
   */
  void process_remote_tx_time(ranging_table_t &agentB, const uwb_time_t &tx_time, const uint8_t seq);

  /**
   * Calculate direct ranges from available timestamps in the 
   * ranging tables, and enqueues them in the _range_measurements vector
   */
  void calculate_direct_ranges(const float time);

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

  void send_ranging_ping(float vx, float vy);

  void process_incoming_data(const float time_now);

  // Call 'process_incoming_data()' before extracting measurements
  void get_new_ranging(std::vector<ekf_range_measurement_t> &ranges, std::vector<ekf_input_t> &inputs);

  // Call 'process_incoming_data()' before extracting measurements
  void get_all_ranging(std::vector<ekf_range_measurement_t> &ranges, std::vector<ekf_input_t> &inputs, const float time);

  void enqueue_srp(const swarm_ranging_ping_t* srp, float rssi, bool in_range);

  void rel_loc_animation(const uint16_t ID);

};


#endif // _SWARM_RANGING_H_