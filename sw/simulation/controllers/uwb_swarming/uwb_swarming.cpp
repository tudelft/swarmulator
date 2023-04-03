#include "uwb_swarming.h"
#include "main.h"
#include "draw.h"
#include "auxiliary.h"
#include "uwb_channel.h"

#include <cmath>

#define SENSOR_MAX_RANGE 1.8

uwb_swarming::uwb_swarming() : Controller()
{
  set_max_sensor_range(SENSOR_MAX_RANGE);
}

void uwb_swarming::init(const uint16_t ID){
  this->ID = ID;
  environment.uwb_channel.join(this->ID);
}

void uwb_swarming::get_velocity_command(const uint16_t ID, float &v_x, float &psirate)
{
  // generate own velocity measurements
  rotate_g2l_xy(agents[this->ID]->state[STATE_VX], agents[this->ID]->state[STATE_VY], agents[this->ID]->state[STATE_YAW],
                own_input.rhoX, own_input.rhoY);
  own_input.rhoX += this->rg.gaussian_float(0, MEAS_NOISE_RHOX);
  own_input.rhoY += this->rg.gaussian_float(0, MEAS_NOISE_RHOY);
  own_input.dPsi = agents[this->ID]->state[STATE_YAWRATE] + this->rg.gaussian_float(0, MEAS_NOISE_DPSI);

  this->communicate();
  // this->estimate_rel_pos();

  v_x = 0.5;
  psirate = rg.uniform_float(-M_PI, M_PI);
  wall_avoidance_turn(ID, v_x, psirate, SENSOR_MAX_RANGE);
}

void uwb_swarming::communicate(){
  environment.uwb_channel.send_ping(this->ID, own_input.rhoX, own_input.rhoY, own_input.dPsi);

  if (this->ID == 0){
    for (uint i_agent=0; i_agent<this->connectivity_vector.size(); i_agent++){
      if (this->connectivity_vector[i_agent] != LINK_NONE){
        environment.uwb_channel.range_to(this->ID, i_agent);
      }
    }
  }
  std::vector<ranging_message> uwb;
  std::vector<float> rssi;
  environment.uwb_channel.receive(this->ID, &uwb, &rssi);

  uint16_t srcID;
  for (uint i=0; i<uwb.size(); i++){
    srcID = uwb[i].id_source;
    while (this->connectivity_vector.size()<=srcID){
      this->connectivity_vector.push_back(LINK_NONE);
    }
    if (uwb[i].type == UWB_MSG_TYPE_PING){
      this->connectivity_vector[srcID] = LINK_PING;
    }
  }
}

void uwb_swarming::estimate_rel_pos(){
  
}

void uwb_swarming::animation(const uint16_t ID)
{
  draw d;

  d.circle_loop(COMMUNICATION_RANGE);

  // float dxg, dyg, dxl, dyl;
  
  // for (uint i=0; i<this->connectivity_vector.size(); i++){
  //   if (i<this->ID && this->connectivity_vector[i]==LINK_PING){
  //     dxg = agents[i]->state[STATE_X] - agents[ID]->state[STATE_X];
  //     dyg = agents[i]->state[STATE_Y] - agents[ID]->state[STATE_Y];
  //     rotate_g2l_xy(dxg, dyg, agents[ID]->state[STATE_YAW], dxl, dyl);
  //     // d.line(dxl, dyl, 1.5, orange); // orange
  //   }
  // }
}