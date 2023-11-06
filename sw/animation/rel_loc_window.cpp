#include <GL/freeglut.h>
#include <thread>
#include <mutex>

#include "main.h"
#include "draw.h"
#include "terminalinfo.h"
#include "trigonometry.h"
#include "rel_loc_estimator.h"

#include "rel_loc_window.h"

/** Compatibility with old GLUT for mapping the mouse wheel
 * http://iihm.imag.fr/blanch/software/glut-macosx/
 */
#if !defined(GLUT_WHEEL_UP)
#  define GLUT_WHEEL_UP    3 // Mouse wheel scrolled up
#  define GLUT_WHEEL_DOWN  4 // Mouse wheel scrolled down
#  define GLUT_WHEEL_LEFT  5 // Mouse wheel to the left
#  define GLUT_WHEEL_RIGHT 6 // Mouse wheel to the right
#endif

static GLint rel_loc_window_id;

// Initial Scene/Camera settings
static float camera[3] = {0.0f, 0.0f, -50.0f};  // Camera Position
static float center[3] = {0.0f, 0.0f, 0.0f};   // Center Position
static float up[3] = {1.0f, 0.0f, 0.0f};       // Direction of "up" vector
static float right[3] = {0.0f, 1.0f, 0.0f};    // Direction of "right" vector

// Mouse drag
static float drag[2] = {0.0f, 0.0f};
static float camera_zoom = 0.0f;

static uint16_t current_agent = 0;
static uint16_t current_estimator = 0;

void rel_loc_reset_camera()
{
  camera[0] =  0.0f;
  camera[1] =  0.0f;
  camera[2] = 50.0f;

  center[0] =  0.0f;
  center[1] =  0.0f;
  center[2] =  0.0f;

  drag[0] = 0.0f;
  drag[1] = 0.0f;
  camera_zoom = 0.0f;
}

void rel_loc_window_reshape(int w, int h)
{
  if(h == 0){
    h = 1;
  }
  float ratio = 1.0 * w/h;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0,0,w,h);
  gluPerspective(45, ratio, 1, 1000);
  glMatrixMode(GL_MODELVIEW);
}

/**
 * Main animation loop.
 * Takes care of drawing the agents in their corrective location.
 */
void rel_loc_window_display()
{
  glutSetWindow(rel_loc_window_id);
  // Add depth (used internally to block obstructed objects)
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();


  gluLookAt( camera[0]+drag[0], camera[1]+drag[1], camera[2]+camera_zoom,
             center[0]+drag[0], center[1]+drag[1], center[2],
             up[0], up[1],  up[2]);

  draw d;
  d.axes();
  
  agents[current_agent]->controller->rel_loc_animation(current_agent, current_estimator);

  // Swap buffers (color buffers, makes previous render visible)
  glutSwapBuffers();
}

void rel_loc_keyboard_callback(unsigned char key, __attribute__((unused)) int a, __attribute__((unused)) int b)
{
  switch (key) {
    case 'q': // End the simulation and quit
      terminalinfo::info_msg("Quitting Swarmulator.");
      program_running = false;
      break;
    case 'c': // Center the animation
      terminalinfo::info_msg("Recentering Animation.");
      rel_loc_reset_camera();
      break;
    case 'p': // Pause the simulation
      if (!paused) {
        terminalinfo::info_msg("Paused. Press `r' to resume or `s' to step forward.");
        paused = true;
        main_mutex.lock();
      }
      break;
    case 'r': // Resume the simulation (if paused)
      if (paused) {
        terminalinfo::info_msg("Resuming.");
        paused = false;
        main_mutex.unlock();
      }
      break;
    case 's': // Step through the simulation. Very useful for debugging or analyzing what's going on.
      if (paused) {
        terminalinfo::info_msg("Stepping through. Press `s' to keep stepping forwrad to `r' to resume. ");
        main_mutex.unlock();
        int t_wait = (int)1e6 * (1.0 / (param->simulation_updatefreq() * param->simulation_realtimefactor()));
        std::this_thread::sleep_for(std::chrono::microseconds(t_wait));
        main_mutex.lock();
      }
      break;
    case 'm': // Toggle the real time parameter between 1 and default, so as to better understand what's going on
      terminalinfo::info_msg("Toggle realtime factor between 1 and the specified value.");
      if (param->simulation_realtimefactor() != 1) {
        realtimefactor = param->simulation_realtimefactor();
        param->simulation_realtimefactor() = 1;
      } else {
        param->simulation_realtimefactor() = realtimefactor;
      }
      break;
    case 'n': // Quit and restart swarmulator
      terminalinfo::info_msg("Restarting.");
      std::stringstream ss;
      ss << "pkill swarmulator && ./swarmulator " << nagents;
      system(ss.str().c_str());
      break;
  }
}

void rel_loc_special_keys(int key, __attribute__((unused)) int a, __attribute__((unused)) int b)
{ 
  // use arrow keys to move between agents
  // shift > move 10, ctrl > move 100
  int modifier = glutGetModifiers();
  int step_size = 1;
  if (modifier == GLUT_ACTIVE_SHIFT){
    step_size = 10;
  }
  if (modifier == GLUT_ACTIVE_CTRL){
    step_size = 100;
  }

  if (key == GLUT_KEY_RIGHT){
    current_agent += step_size;
    if (current_agent >= agents.size()){
      // wrap back to zero
      current_agent = 0;
    }
  }
  if (key == GLUT_KEY_LEFT){
    current_agent -= step_size;
    if (current_agent >= agents.size()){
      // current agent is unsigned > this wraps back to the highest ID
      current_agent = agents.size()-1;
    }
  }

  if (key == GLUT_KEY_UP){
    current_estimator += 1;
    if (current_estimator >= ESTIMATOR_MAX){
      // wrap back to zero
      current_estimator = 0;
    }
  }
  if (key == GLUT_KEY_DOWN){
    current_estimator -= step_size;
    if (current_estimator >= ESTIMATOR_MAX){
      // current estimator is unsigned > this wraps back to the highest ID
      current_estimator = ESTIMATOR_MAX-1;
    }
  }

}

// Mouse motion (x: left>right, y: up>down)
static int drag_origin[2] = {0, 0};
static bool drag_screen = false;

void rel_loc_mouse_motion_callback(int x, int y)
{
  // Move the center
  if (drag_screen) {
    float deltaLR = (float)(x-drag_origin[0])*0.01f;
    float deltaUD = (float)(y-drag_origin[1])*0.01f;
    drag[0] = up[0]*deltaUD + up[1]*deltaLR;
    drag[1] = right[0]*deltaUD + right[1]*deltaLR;
  }
}

void rel_loc_mouse_click_callback(int button, int state, int x, int y)
{
  // Click - left
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    // Position on window in percentage
    drag_origin[0] = x;
    drag_origin[1] = y;
    drag_screen = true;
  }

  if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
    camera[0] += drag[0];
    camera[1] += drag[1];
    center[0] += drag[0];
    center[1] += drag[1];
    drag[0] = 0.0f;
    drag[1] = 0.0f;
    drag_screen = false;
  }

  // Zoom wheel
  if (button == GLUT_WHEEL_UP) {
    camera_zoom += 10*param->mouse_zoom_speed();
  } else if (button == GLUT_WHEEL_DOWN) {
    camera_zoom -= 10*param->mouse_zoom_speed();
  }

  // Guard on too much / too little zoom
  if (camera_zoom < -200) {
    camera_zoom = -200;
  } else if (camera_zoom > 20) {
    camera_zoom = 20;
  }
}


void rel_loc_window_init()
{
  rel_loc_window_id = glutCreateWindow("Relative Localization");
  glutReshapeFunc(rel_loc_window_reshape);
  glutDisplayFunc(rel_loc_window_display);

  // user interaction callbacks
  glutKeyboardFunc(rel_loc_keyboard_callback);
  glutSpecialFunc(rel_loc_special_keys);
  glutMotionFunc(rel_loc_mouse_motion_callback);
  glutMouseFunc(rel_loc_mouse_click_callback);
}