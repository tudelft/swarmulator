#include <sstream>
#include <iomanip> // needed for std::setprecision

#include "txtwrite.h"
#include "omniscient_observer.h"
#include "terminalinfo.h"
#include "agent.h"
#include "main.h"
#include "fitness_functions.h"

using namespace std;

txtwrite::txtwrite() {}

void txtwrite::setfilename(const string &s)
{
  filename = s; // Set the name of the file
}

void txtwrite::txtwrite_state(ofstream &logfile)
{
  std::stringstream t; // time
  t << simtime_seconds; // Write down time
  vector<Agent *> state_buff = agents; // Get state
  float f = evaluate_fitness(); // Evaluate fitness
  for (uint16_t i = 0; i < agents.size(); i++) {
    logfile << t.str() << " " // time
            << i + 1 << " " // ID
            << state_buff[i]->state.at(STATE_X) << " "  // position x
            << state_buff[i]->state.at(STATE_Y) << " "; // position y

    logfile << f; // fitness
    logfile << endl; // new line
  }
}