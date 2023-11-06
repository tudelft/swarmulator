#ifndef DRAW_H
#define DRAW_H

#include <GL/freeglut.h>
#include <vector>
#include <sstream>      // std::stringstream
#include <mutex>

#include "main.h"
#include "drawingparams.h"
#include "settings.h"

/**
 * @brief Object that takes care of OpenGL drawing for all elements of interest.
 * This class contains general openGL function that
 * can be used to animate and visualize the swarm at runtime.
 * It is launched and managed by the animation thread.
 */

struct vertex{
  float x;
  float y;
  float z;
};

struct color3ub{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

extern color3ub green;
extern color3ub yellow;
extern color3ub orange;
extern color3ub cyan;
extern color3ub red;

class draw
{
public:
  /******* General openGL functions ********/

  /**
   * Draw a simple triangle of size scl
   *
   * @param s Scale of the triangle
   */
  void triangle(const float &s);

  void triangle(const float &scl, const color3ub &color);


  /**
   * Draw a red circle of radius r
   *
   * @param r Radius of the circle
   */
  void circle(const float &r);

  /**
   * Draw a white unfilled circular loop
   *
   * @param r Radius of the circular loop
   */
  void circle_loop(const float &r);

  /**
   * Draw a white line from (0,0) to (x,y)
   *
   * @param x x position (East)
   * @param y y position (North)
   */
  void line(const float &x, const float &y);

  /**
   * @brief Draw a white line with a specified width from (0,0) to (x,y)
   *
   * @param x
   * @param y
   * @param width
   */
  void line(const float &x, const float &y, const float &width);

  /**
   * @brief Draw a colored line with a specified width from (0,0) to (x,y)
   *
   * @param x
   * @param y
   * @param width
   * @param color
   */
  void line(const float &x, const float &y, const float &width, const color3ub &color);


  /**
   * Draw the global x and y axes at (0,0)
   */
  void axes();

  /**
   * Draw a segment (used for walls and obstacles)
   */
  void segment(const float &x0, const float &y0, const float &x1, const float &y1);

  /**
   * Draw a segment (used for walls and obstacles)
   */
  void segment(const float &x0, const float &y0, const float &x1, const float &y1, const color3ub &color);

  /**
  * Draw a small white point
  */
  void point();

  /**
   * Draw relevant simulation data in the bottom left corner (like the time of simulation)
   */
  void data();
  void info_text(std::string str);
  /**
   * Write the x,y label on the global axis along the given dimension dim
   *
   * @param dim Specifies (=0) for writing the x label and (=1) for writing the y label
   */
  void axis_label();

  /**
   * Draw the ID number of an agent on top of the agent
   *
   * @param ID The ID of the robot in question
   */
  void agent_number(const uint16_t &ID);

  /**
   * Draw the agent (uses internal function defined by the agent class)
   *
   * @param ID The ID of the robot in question
   * @param x The x position of the robot
   * @param y The y position of the robot
   * @param orientation The orientation of the robot
   */
  void agent(const uint16_t &ID, const float &x, const float &y, const float &orientation);

  void rel_loc_agent(const uint16_t &ID);
  void agent_raw(const uint16_t ID, const float x, const float y, const float yaw);


  /**
   * Draw a line showing the velocity of the agent
   *
   * @param ID The ID of the robot in question
   * @param x The x position of the robot (East)
   * @param y The y position of the robot (North)
   * @param v_x The velocity of the robot in x (global)
   * @param v_y The velocity of the robot in y (global)
   */
  void velocity_arrow(const uint16_t &ID, const float &x, const float &y, const float &v_x, const float &v_y);

  /**
   * Draw a gray dot indicating food (or something else if you like) at position (x,y)
   *
   * @param x The x position of the food
   * @param y The y position of the food
   */
  void food(const float &x, const float &y);

  void estimate(const uint16_t &ID, const float &rel_x, const float &rel_y, const float &rel_psi);

};

#endif /*DRAW_H*/
