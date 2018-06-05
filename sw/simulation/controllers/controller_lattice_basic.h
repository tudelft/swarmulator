#ifndef CONTROLLER_LATTICE_BASIC_H
#define CONTROLLER_LATTICE_BASIC_H

#include "controller.h"
#include "template_calculator.h"

using namespace std;

class Controller_Lattice_Basic : public Controller
{
  public:
    OmniscientObserver *o; // The omniscient observer is used to simulate sensing the other agents.
    Template_Calculator t;
    vector<float> bdes;
    float _v_adj = 0.1; // Adjustment velocity

    Controller_Lattice_Basic(){};
    ~Controller_Lattice_Basic(){};

    void attractionmotion(const float &v_r, const float &v_b, float &v_x, float &v_y);
    void latticemotion(const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v_x, float &v_y);
    float f_attraction(float u, float b);
    float get_attraction_velocity(float u, float b_eq);
    void actionmotion(const int selected_action, float &v_x, float &v_y);

    virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y) = 0;
};

#endif /*CONTROLLER_LATTICE_BASIC_H*/