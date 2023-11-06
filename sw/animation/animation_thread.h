#ifndef ANIMATION_THREAD_H
#define ANIMATION_THREAD_H

#include <GL/freeglut.h>

#include "main.h"
#include "draw.h"
#include "user_interaction.h"
#include "terminalinfo.h"
#include "trigonometry.h"
#include "rel_loc_window.h"

#include "main_window.h"

GLint main_window_id;

#ifdef VERBOSE
  // GLint rel_loc_window_id;
  GLint performance_window_id;
#endif

bool animation_running = false;

void global_animation_loop()
{
  if (!animation_running) {
    terminalinfo::info_msg("Animation started.");
    animation_running = true;
  }

  glutSetWindow(main_window_id);
  main_window_display();
  
  rel_loc_window_display();


  
  if (!program_running) {std::terminate();}
}

/**
 * Thread function that initiates the animation
 */
void main_animation_thread()
{
  // Initialize all variables declared in drawingparams.h
  center_x = 0;
  center_y = 0;
  sx = 0;
  sy = 0;
  zoom = param->zoom(); // From parameters file
  zoom_scale = 0;
  pointer_x = 0;
  pointer_y = 0;
  paused = false;
  xrat = 1.0;
  yrat = 1.0;

  // Initialize Open GL (glut)
  int argc = 1;
  char *argv[1] = {(char *)"  "};
  glutInit(&argc, argv);
  glutInitWindowPosition(-1, -1); // Leave position up to window manager
  glutInitWindowSize(param->window_width(), param->window_height()); // Set dimensions
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

  // Set up main simulation window
  main_window_id = glutCreateWindow("Swarmulator"); // Set window title to Swarmulator
  glutReshapeFunc(main_window_reshape);
  glutDisplayFunc(main_window_display);
  user_interaction(); // Set the user interaction callbacks for the main window

  rel_loc_window_init();


  glutIdleFunc(global_animation_loop); // Set main loop
  glutMainLoop(); // Initiate main drawing loop
}

#endif /*ANIMATION_THREAD_H*/
