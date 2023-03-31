#ifndef TRIGONOMETRY_H
#define TRIGONOMETRY_H

#include <stdlib.h> // qsort
#include <cmath>
#include <string>

/**
 * Wraps an angle in radians between -PI and +PI, overwrites onto the variable
 *
 * @param ang Angle value (in radians)
 */
inline static void wrapToPi(float &ang)
{
  if (ang >  M_PI) { while (ang >  M_PI) { ang = ang - 2 * M_PI;} }
  else if (ang < -M_PI) { while (ang < -M_PI) { ang = ang + 2 * M_PI;} }
}

/**
 * Wraps an angle in radians between 0 and +2PI, overwrites onto the variable
 *
 * @param ang Angle value (in radians)
 */
inline static void wrapTo2Pi(float &ang)
{
  while (ang > 2 * M_PI) {
    ang = ang - 2 * M_PI;
  }
  while (ang < 0.0) {
    ang = ang + 2 * M_PI;
  }
}

/**
 * Wraps an angle in radians between -PI and +PI, returns the wrapped value
 *
 * @param ang Angle value (in radians)
 * @return The angle value, wrapped to [-PI,PI]
 */
inline static float wrapToPi_f(float ang)
{
  if (ang > M_PI) {
    while (ang > M_PI) {
      ang = ang - 2 * M_PI;
    }
  } else if (ang < -M_PI) {
    while (ang < -M_PI) {
      ang = ang + 2 * M_PI;
    }
  }
  return ang;
}

/**
 * Wraps an angle in radians between 0 and +2PI, returns the wrapped value
 *
 * @param ang Angle value (in radians)
 * @return The angle value, wrapped to [0,2PI]
 */
inline static float wrapTo2Pi_f(float ang)
{
  if (ang > 2 * M_PI) {
    while (ang > 2 * M_PI) {ang = ang - 2 * M_PI;}
  } else if (ang < 0.0) {
    while (ang < 0.0) {
      ang = ang + 2 * M_PI;
    }
  }
  return ang;
}

/**
 * Converts an angle from radians to degrees
 *
 * @param ang Angle value in radians
 * @return The angle value in degrees
 */
inline static float rad2deg(float rad) { return 180.0 / M_PI * rad; }

/**
 * Converts an angle from degrees to radians
 *
 * @param ang Angle value in degrees
 * @return The angle value in radians
 */
inline static float deg2rad(float deg) { return M_PI / 180.0 * deg; }

/**
 * Convert polar (r,theta) coordinates to cartesian (x,y) coordinates
 *
 * @param r Value of r in polar coordinates
 * @param theta Value of theta in polar coordinates (in radians)
 * @param x Value of x in cartesian coordinates (East)
 * @param y Value of y in cartesian coordinates (North)
 */
inline static void polar2cart(const float &r, const float &theta, float &x, float &y)
{
  x = r * cos(wrapToPi_f(theta));
  y = r * sin(wrapToPi_f(theta));
}

/**
 * Convert cartesian (x,y) coordinates to polar (r,theta) coordinates
 *
 * @param x Value of x in cartesian coordinates (East)
 * @param y Value of y in cartesian coordinates (North)
 * @param r Value of r in polar coordinates
 * @param theta Value of theta in polar coordinates
 */
inline static void cart2polar(const float &x, const float &y, float &r, float &theta)
{
  r = sqrt(pow(x, 2) + pow(y, 2));
  theta = atan2(y, x);
}

/**
 * Rotates a vector from local to global coordinates
 *
 * @param x_loc Value of x in local cartesian coordinates
 * @param y_loc Value of y in local cartesian coordinates
 * @param yaw Angle of rotation in rad
 * @param x_global Rotated x in global coordinates
 * @param y_global Rotated y in global coordinates
 */
inline static void rotate_l2g_xy(const float &x_loc, const float &y_loc, const float &yaw, float &x_global, float &y_global)
{
  x_global = x_loc * cos(yaw) - y_loc * sin(yaw);
  y_global = x_loc * sin(yaw) + y_loc * cos(yaw);
}

/**
 * Rotates a vector from global to local coordinates
 *
 * @param x_global Value of x in global cartesian coordinates
 * @param y_global Value of y in global cartesian coordinates
 * @param yaw Angle of rotation in rad
 * @param x_loc Rotated x in local coordinates
 * @param y_loc Rotated y in local coordinates
 */
inline static void rotate_g2l_xy(const float &x_global, const float &y_global, const float &yaw, float &x_loc, float &y_loc)
{
  x_loc =  x_global * cos(yaw) + y_global * sin(yaw);
  y_loc = -x_global * sin(yaw) + y_global * cos(yaw);
}

#endif /*TRIGONOMETRY_H*/
