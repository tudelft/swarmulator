#ifndef DRAW_H
#define DRAW_H

#include <GL/freeglut.h>
#include <vector>
#include <sstream>      // std::stringstream
#include <mutex>

#include "main.h"
#include "drawingparams.h"
#include "settings.h"
#include "types.h"

/**
 * @brief Object that takes care of OpenGL drawing for all elements of interest.
 * This class contains general openGL function that
 * can be used to animate and visualize the swarm at runtime.
 * It is launched and managed by the animation thread.
 */
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
  void triangle(const Eigen::MatrixXf points, const float scl=1.0);

  void rect(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3, const Eigen::Vector3f &p4, const float &width =1, const std::array<float, 4> color = {0,0,0, 1});
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
  void circle_loop(const float &r, const float& width);

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
  void line(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const float &width = 1, const std::array<float, 4> color = {0,0,0,1});


  void polyline(const Eigen::MatrixXf &points, const float &width=1, const std::array<float,4> color = {0.0f,0.0f,0.0f,1.0f});
  
  
  void polygon(const Eigen::MatrixXf &points, const float &width=1, const std::array<float,4> color = {0.0f,0.0f,0.0f,1.0f});
  /**
   * Draw the global x and y axes at (0,0)
   */
  void axes();

  /**
   * Draw a segment (used for walls and obstacles)
   */
  void segment(const float &x0, const float &y0, const float &x1, const float &y1);

  /**
  * Draw a small white point
  */
  void point();
  void points(Eigen::MatrixXf p, const std::array<float, 4>& color = {1,0,0,1});
  void points(std::vector<Eigen::Vector3f> points, const std::array<float, 4>& color = {1,0,0,1});

  /**
   * Draw relevant simulation data in the bottom left corner (like the time of simulation)
   */
  void data();

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
  void agent(const uint16_t &ID, const State state);

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
  void velocity_arrow(const uint16_t &ID, const State state);

  /**
   * Draw a gray dot indicating food (or something else if you like) at position (x,y)
   *
   * @param x The x position of the food
   * @param y The y position of the food
   */
  void food(const float &x, const float &y);
  void sphere(const Eigen::Vector3f pos, Eigen::Vector4f axis_angle, float radius);
};

#endif /*DRAW_H*/
