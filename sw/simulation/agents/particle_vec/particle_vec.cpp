#include "particle_vec.h"
#include <vector>
#include "types.h"
#include "draw.h"
#include <iostream>
#include "utils.h"

particle_vec::particle_vec(int i, State s, float tstep){
    state = s;
    dt = tstep;
    orientation = s.psi;
    ID = i;
    controller->set_saturation(0.5);
    manual=false;
}

State particle_vec::state_update(State state)
{
  // NED frame
  // x+ towards North
  // y+ towards East
  Vector<float> v_des = Vector<float>({1.,1.,1.});
  state.acc = v_des - state.vel; 
  state.vel += state.acc * dt;
  state.pos += state.vel * dt * 0.5 + state.acc * pow(dt, 2); 
  return state;
// //   float vx_des, vy_des = 0.;

   
// //   float vx_global, vy_global, dpsirate;
// //   if (!manual) {
// //     controller->get_velocity_command(ID, vx_des, dpsirate); // Command comes out in the local frame
// //   } else {
// //     vx_des = manualx;
// //     vy_des = manualy;
// //     dpsirate = manualpsi_delta;
// //   }
// //   controller->saturate(vx_des);
// //   controller->saturate(vy_des);
// // #if COMMAND_LOCAL
// //   rotate_xy(vx_des, vy_des, state[6], vx_global, vy_global);
// // #else
// //   vx_global = vx_des;
// //   vy_global = vy_des;
// // #endif
// //   state.at(7) = dpsirate;
// //   state.at(6) += state[7] * dt;
// //   orientation = wrapToPi_f(state[6]);

// //   // Acceleration control
// //   float ka = 2;
// //   state.at(4) = ka * (vx_global - state[2]); // Acceleration global frame
// //   state.at(5) = ka * (vy_global - state[3]); // Acceleration global frame
// //   moving = controller->moving;

// //   // Velocity
// //   state.at(2) += state[4] * dt; // Velocity x global frame
// //   state.at(3) += state[5] * dt; // Velocity y global frame

// //   // Position
// //   state.at(0) += state[2] * dt + 0.5 * state[4] * pow(dt, 2); // Position x global frame
// //   state.at(1) += state[3] * dt + 0.5 * state[5] * pow(dt, 2); // Position y global frame

//   return state;
};


void particle_vec::animation()
{
  draw d;

  d.triangle(param->scale());
}
