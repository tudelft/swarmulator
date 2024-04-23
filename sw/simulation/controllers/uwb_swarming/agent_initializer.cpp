#include "agent_initializer.h"

#include <math.h>
#include <iostream>

#include "main.h"
#include "ekf_types.h"
#include "ekf_math.h"

AgentInitializer::AgentInitializer(const uint16_t self_id){
    _self_id = self_id;
    for (uint8_t iSlot=0; iSlot<AGENT_INITIALIZER_MAX_SLOTS; iSlot++){
        reset_slot(iSlot);
    }
}

void AgentInitializer::reset_slot(const uint16_t slot_idx){
    _is_used[slot_idx] = false;
    _agent_id[slot_idx] = _self_id;

    // reset trajectory data
    _traj_relX_accum[slot_idx] = 0;
    _traj_relY_accum[slot_idx] = 0;
    
    _traj_start_time[slot_idx] = 0;
    _traj_start_range[slot_idx] = -1;
    _traj_end_range[slot_idx] = -1;

    _time_added[slot_idx] = -1;
}

bool AgentInitializer::get_index(const uint16_t id, uint16_t *idx){
    bool success = false;
    if (id != _self_id){
        for (uint8_t iSlot=0; iSlot<AGENT_INITIALIZER_MAX_SLOTS; iSlot++){
            if (_is_used[iSlot] && _agent_id[iSlot]==id){
                *idx=iSlot;
                success = true;
            }
        }
    }
    return success;
}

bool AgentInitializer::add_agent(const uint16_t id, const float time_now, uint16_t *index){
    for (uint16_t iSlot=0; iSlot<AGENT_INITIALIZER_MAX_SLOTS; iSlot++){
        if (_is_used[iSlot]){
            if (time_now > _time_added[iSlot] + AGENT_INITIALIZER_TIMEOUT){
                reset_slot(iSlot);
            }else{
                continue;
            }
        } else{
            _is_used[iSlot] = true;
            _agent_id[iSlot] = id;
            _time_added[iSlot] = time_now;
            for (uint16_t iPoint=0; iPoint<MULTILAT_MAX_POINTS; iPoint++){
                _multilat_pts[iSlot][iPoint].id_B = id; // empty (invalid) point
            }
            *index = iSlot;
            return true;
        }
    }
    return false;
}


void AgentInitializer::add_velocities(const uint16_t id, const float rel_vx, const float rel_vy, const float time){
    uint16_t idx;
    if (get_index(id, &idx)){
        float dt = time - _traj_last_accum_time[idx];
        _traj_last_accum_time[idx] = time;

        _traj_relX_accum[idx] += dt*rel_vx;
        _traj_relY_accum[idx] += dt*rel_vy;
    }
}

void AgentInitializer::add_direct_range(const uint16_t id, const float range, const float time){
    uint16_t idx;
    if (get_index(id, &idx) || add_agent(id, time, &idx)){
        if (_traj_start_time[idx] != 0){
            // there's already a first measurement
            _traj_end_range[idx] = range;
        }else{
            _traj_start_time[idx] = time;
            _traj_start_range[idx] = range;    
            _traj_last_accum_time[idx] = time;
        }
        multilat_point_t mlp;
        mlp.id_B = _self_id;
        mlp.range = range;
        mlp.x = 0.0f;
        mlp.y = 0.0f;
        mlp.stdev_x = 0.0f;
        mlp.stdev_y = 0.0f;
        mlp.timestamp = time;
        add_multilat_point(id, mlp);
    }
}

void AgentInitializer::add_multilat_point(const uint16_t id, const multilat_point_t &new_point){
    uint16_t idx;
    if(get_index(id, &idx) || add_agent(id, new_point.timestamp, &idx)){
        for (uint16_t iPoint=0; iPoint<MULTILAT_MAX_POINTS; iPoint++){
            if (_multilat_pts[idx][iPoint].id_B == new_point.id_B ||
                    _multilat_pts[idx][iPoint].id_B == id){
                // Point empty or previous measurement from same agent
                _multilat_pts[idx][iPoint].id_B = new_point.id_B;
                _multilat_pts[idx][iPoint].range = new_point.range;
                _multilat_pts[idx][iPoint].stdev_x = new_point.stdev_x;
                _multilat_pts[idx][iPoint].stdev_y = new_point.stdev_y;
                _multilat_pts[idx][iPoint].x = new_point.x;
                _multilat_pts[idx][iPoint].y = new_point.y;
                _multilat_pts[idx][iPoint].timestamp = new_point.timestamp;
                break;
            }
        }
    }
}


bool AgentInitializer::get_initial_position(const uint16_t id, const float time_now, agent_initialization_data_t* init_data){
    bool success = initial_position_from_multilat(id, time_now, init_data);
    if (success == false){
        success = initial_position_from_traj(id, time_now, init_data);
    }
    if (success){
        uint16_t idx;
        get_index(id, &idx);
        reset_slot(idx);
    }
    return success;
}

bool AgentInitializer::initial_position_from_multilat(const uint16_t id, const float time_now, agent_initialization_data_t* init_data){
    bool success = false;
    uint16_t idx;
    if (get_index(id, &idx)){
        MatrixFloat A(MULTILAT_MAX_POINTS, 3);
        MatrixFloat b(MULTILAT_MAX_POINTS, 1);
        // check if there are enough points
        uint16_t idx_point = 0;
        for (uint16_t iRow=0; iRow<MULTILAT_MAX_POINTS; iRow++){
            if (_multilat_pts[idx][idx_point].id_B == id){
                continue;
            } else{
                A.data[iRow][0] = 1;
                A.data[iRow][1] = -2*_multilat_pts[idx][idx_point].x;
                A.data[iRow][2] = -2*_multilat_pts[idx][idx_point].y;
                b.data[iRow][0] = powf(_multilat_pts[idx][idx_point].range, 2)
                                    - powf(_multilat_pts[idx][idx_point].x, 2)
                                    - powf(_multilat_pts[idx][idx_point].y, 2);
                idx_point++;
            }
        }

        if (idx_point>2){
            // enough points for multilateration
            MatrixFloat AT(3,MULTILAT_MAX_POINTS);
            MatrixFloat ATA(3,3);
            fmat_trans(A, AT);
            fmat_mult(AT, A, ATA);
            MatrixFloat ATAinv(3,3);
            if(fmat_inv(ATA, ATAinv)){
                MatrixFloat tmp(3,MULTILAT_MAX_POINTS);
                fmat_mult(ATAinv, AT, tmp);
                MatrixFloat x(3,1);
                fmat_mult(tmp, b, x);

                init_data->id = id;
                init_data->timestamp = time_now;
                init_data->x0=x.data[0][0];
                init_data->y0=x.data[1][0];
                init_data->stdev_x = 100;
                init_data->stdev_y = 100;
                success = true;
                if (_self_id==0){
                    std::cout << "Initialized " << id << ": Multilat" << std::endl;
                }
            }
        }
    }  
    return success;
}

bool AgentInitializer::initial_position_from_traj(const uint16_t id, const float time_now, agent_initialization_data_t* init_data){
    bool success = false;
    uint16_t idx;
    if (get_index(id, &idx)){
        float dt_tot = time_now - _traj_start_time[idx];
        if (dt_tot >= INITIALIZER_PERIOD &&
                _traj_start_range[idx] > 0 && _traj_end_range[idx] > 0){
            float dx, dy, r0, r1, x0loc, y0loc, varx_loc, vary_loc;
            dx = _traj_relX_accum[idx];
            dy = _traj_relY_accum[idx];
            r0 = _traj_start_range[idx];
            r1 = _traj_end_range[idx];

            float var_dx, var_dy, var_uwb;
            var_dx = powf(MEAS_NOISE_VX,2)*dt_tot;
            var_dy = powf(MEAS_NOISE_VY,2)*dt_tot;
            var_uwb = powf(MEAS_NOISE_UWB,2);

            if (dx==0 && dy==0){
                // invalid initialization data, reset
                reset_slot(idx);
                return success;
            }

            // Calculate p0 and p1 from dx, dy, r0 and r1
            // There are two solutions to this problem
            float tmp = powf(r0, 2) - powf(r1, 2) - powf(dx, 2) - powf(dy, 2);
            float a = 4 * (dx*dx + dy*dy);
            float b = 4 * dx * tmp;
            float c = tmp*tmp - 4*r1*r1*dy*dy;
            
            float discr = powf(b, 2) - 4*a*c;
            // if (true){
            if (discr < 0){
                // invalid measurements, but close to line
                discr = 0;
                float norm = sqrtf(powf(dx, 2.0f)+powf(dy, 2.0f));
                x0loc = r1 * dx/norm;
                y0loc = r1 * dy/norm;
                if (r1<r0){
                    // other drone approaching, not going away
                    x0loc = - x0loc;
                    y0loc = - y0loc;
                }

                // Uncertainty propagation
                float dx_ddx, dx_ddy, dx_dr;
                dx_ddx = r1/norm -r1*powf(dx,2)/powf(norm, 3);
                dx_ddy = r1*dx*dy/powf(norm, 3);
                dx_dr = dx/norm;
                varx_loc = powf(dx_ddx, 2)*var_dx + powf(dx_ddy, 2)*var_dy + powf(dx_dr, 2)*var_uwb;
                
                float dy_ddx, dy_ddy, dy_dr;
                dy_ddx = r1*dx*dy/powf(norm, 3);
                dy_ddy = r1/norm -r1*powf(dy,2)/powf(norm, 3);
                dy_dr = dy/norm;
                vary_loc = powf(dy_ddx, 2)*var_dx + powf(dy_ddy, 2)*var_dy + powf(dy_dr, 2)*var_uwb;

                // if(varx_loc > 100 || vary_loc > 100){
                //     std::cout << "Initializer - Large Variance encountered (single)" << std::endl;
                // }

            } else {
                // Two solutions for x1
                float x1_A = (-b + sqrtf(discr))/(2*a);
                float x1_B = (-b - sqrtf(discr))/(2*a);
                // Associated solutions for x0
                float x0_A = x1_A - dx;
                float x0_B = x1_B - dx;
                // y1 calculated from x1 & r1 (both signs possible)
                float y1_A = sqrtf(r1*r1 - x1_A*x1_A);
                float y1_B = sqrtf(r1*r1 - x1_B*x1_B);
                // y0 calculated from x0 & r0 (both signs possible)
                float y0_A = sqrtf(r0*r0 - x0_A*x0_A);
                float y0_B = sqrtf(r0*r0 - x0_B*x0_B);
                
                // find correct signs from dy
                if (dy>0){
                    // Solution A
                    if (y1_A > y0_A){
                        // y1_A must be positive
                        y0_A = y1_A-dy;
                    } else{
                        y0_A = - y0_A;
                        y1_A = y0_A + dy;
                    }
                    // Solution B
                    if (y1_B > y0_B){
                        // y1_B must be positive
                        y0_B = y1_B-dy;
                    } else{
                        y0_B = - y0_B;
                        y1_B = y0_B + dy;
                    }

                } else if (dy<0){
                    // Solution A
                    if (y1_A > y0_A){
                        y1_A = -y1_A;
                        y0_A = y1_A-dy;
                    } else{
                        // y0_A must be positive
                        y1_A = y0_A + dy;
                    }
                    // Solution B
                    if (y1_B > y0_B){
                        y1_B = -y1_B;
                        y0_B = y1_B-dy;
                    } else{
                        // y0_B must be positive
                        y1_B = y0_B + dy;
                    }

                } else {
                    // dy == 0
                    y0_A = y1_A;

                    y1_B = -y1_A;
                    y0_B = y1_B;
                }
                // Average the two solutions for initial estimate
                x0loc = (x1_A+x1_B)/2;
                y0loc = (y1_A+y1_B)/2;

                varx_loc = powf((x1_A - x0loc),2);
                vary_loc = powf((y1_A - y0loc),2);

/*
                // Estimate variance of the estimate
                float var_a, da_ddx, da_ddy;
                da_ddx = 8*dx;
                da_ddy = 8*dy;
                var_a = powf(da_ddx,2)*var_dx 
                        + powf(da_ddy,2)*var_dy;

                float var_b, db_ddx, db_ddy, db_dr0, db_dr1;
                db_ddx = 4*tmp-8*powf(dx,2);
                db_ddy = -8*dy*dx;
                db_dr0 = 8*r0*dx;
                db_dr1 = -8*r1*dx;
                var_b = powf(db_ddx,2)*var_dx 
                        + powf(db_ddy,2)*var_dy 
                        + powf(db_dr0,2)*var_uwb 
                        + powf(db_dr1,2)*var_uwb;

                float var_c, dc_ddx, dc_ddy, dc_dr0, dc_dr1;
                dc_ddx = -4*dx*tmp;
                dc_ddy = -4*dy*tmp - 8*dy*powf(r1,2);
                dc_dr0 = 4*r0*tmp;
                dc_dr1 = -4*r1*tmp - 8*r1*powf(dy,2);
                var_c = powf(dc_ddx,2)*var_dx 
                        + powf(dc_ddy,2)*var_dy 
                        + powf(dc_dr0,2)*var_uwb 
                        + powf(dc_dr1,2)*var_uwb;

                float var_x1, dx1_da, dx1_db, dx1_dc;
                dx1_da = (b+sqrtf(discr))/(2*powf(a,2)) + 1/(4*a*sqrtf(discr));
                dx1_db = 1/(2*a) + b/(2*a*sqrtf(discr));
                dx1_dc = 1/sqrtf(discr);
                var_x1 = powf(dx1_da, 2)*var_a + powf(dx1_db,2)*var_b + powf(dx1_dc,2)*var_c;

                float var_y1A, var_y1B;
                var_y1A = powf((x1_A/y1_A), 2)*var_x1 + powf((r1/y1_A), 2)*var_uwb;
                var_y1B = powf((x1_B/y1_B), 2)*var_x1 + powf((r1/y1_B), 2)*var_uwb;

                varx_loc += var_x1;
                vary_loc += std::max(var_y1A, var_y1B);
*/
                if(varx_loc > 100000 || vary_loc > 100000){
                    std::cout << "Initializer - Large Variance encountered (avg)" << std::endl;
                }
            }
            init_data->id = id;
            init_data->x0 = x0loc;
            init_data->y0 = y0loc;
            init_data->stdev_x = sqrtf(varx_loc);
            init_data->stdev_y = sqrtf(vary_loc);
            init_data->timestamp = time_now;
            reset_slot(idx);
            success = true;

            if (_self_id==0){
                std::cout << "Initialized " << id <<": Traj" << std::endl;
            }
        }
    }
    return success;
}
