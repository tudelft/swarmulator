/**
 *
 * Swarmulator is a swarm simulation environment.
 * Its design purpose is to be a simple testing platform to observe the emergent behavior of a group of agents.
 * To program specific behaviors, you can do so in the controller.cpp/controller.h file.
 *
 * Mario Coppola, 2017-2020.
 *
 */

/**
 * Include standard and thread libraries
 */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <thread>

/**
 * Include top level threads
 */
#include "main.h" // Contains extern defines for global variables
#include "simulation_thread.h" // Thread that handles the simulation
#include "animation_thread.h" // Thread that handles animation
#include "logger_thread.h" // Thread that handles the logger

/**
 * Parameters from the XML file
 */
std::unique_ptr<parameters_t> param(parameters("conf/parameters.xml", xml_schema::flags::dont_validate));

/**
 * Global variables used throughout simulation
 */
uint nagents; // Number of agents in the simulation
std::vector<Agent *> agents; // Set up the agents
std::shared_mutex main_mutex; // Mutex needed to lock threads
std::shared_mutex environment_mutex; // Mutex needed to lock threads
float realtimefactor; // Real time factor of simulation
float simtime_seconds = 0; // Initial simulation time
bool program_running  = false; // Program running, initiated false until the beginning
Environment environment; // Environment walls
std::string identifier; // Log name identifier

/**
 * The main function launches separate threads that control independent
 * functions of the code. All threads are designed to be optional with the
 * exception of the simulation thread.
 */
int main(int argc, char *argv[])
{
  program_running = true; // Program is running

  // Set FIFO identifier
  if (argc > 2) { // If supplied by the user, use the given input
    std::string s = "";
    s += argv[2];
    identifier = s;
  } else { // If not, use the current date and time
    identifier = currentDateTime();
  }

  // Launch animation thread
#ifdef ANIMATION
  std::thread animation(main_animation_thread);
  animation.detach();
#endif

  // Launch log thread
#ifdef LOG
  std::thread logger(main_logger_thread);
  logger.detach();
#endif

  main_simulation_thread(argc, argv, identifier);

  // Exit
  terminalinfo::info_msg("Swarmulator exiting.");

  return 0;
}
