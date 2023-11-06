#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <GL/freeglut.h>

void main_window_reshape(int w, int h)
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

void main_window_display()
{
  // Add depth (used internally to block obstructed objects)
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  glLoadIdentity();

  // Get current window size w.r.t. beginning
  xrat = (float)param->window_width() / (float)glutGet(GLUT_WINDOW_WIDTH);
  yrat = (float)param->window_height() / (float)glutGet(GLUT_WINDOW_HEIGHT);

  zoom_scale = -(float)10 / (-(float)10 + (float)zoom);
  glTranslatef(center_x, center_y, -10 + zoom);

  // Draw fixed one time objects
  static draw drawer; // Drawer object
  drawer.data(); // Put data in corner
  drawer.axes(); // Put x and y global axes
  drawer.axis_label(); // Axis label
  environment.animate(); // Animate the environment walls

  // Draw all robots
  uint r = agents.size();
  if (r > 0) {
    for (uint16_t ID = 0; ID < r; ID++) {
      // Input: ID, p_x global, p_y global, orientation global
      drawer.agent(ID, agents[ID]->state.at(STATE_X), agents[ID]->state.at(STATE_Y), agents[ID]->orientation);
      // Input: ID, p_x global, p_y global, v_x global, v_y global
      drawer.velocity_arrow(ID,  agents[ID]->state.at(STATE_X), agents[ID]->state.at(STATE_Y), agents[ID]->state.at(STATE_VX), agents[ID]->state.at(STATE_VY));
    }
  }

  // Swap buffers (color buffers, makes previous render visible)
  glutSwapBuffers();
}

#endif // MAIN_WINDOW_H