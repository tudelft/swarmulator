#include "draw.h"
#include "trigonometry.h"
#include <cmath>
#include "fitness_functions.h"

void draw::data()
{
  glRasterPos2f((-3.9 / zoom_scale - center_x), (-3.9 / zoom_scale - center_y));
  glColor3ub(255, 255, 255); // White
  std::stringstream ss;
  ss << "Time[s]:" << simtime_seconds << " \t" << "Fitness: " << evaluate_fitness();
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)ss.str().c_str());
}

void draw::axis_label()
{
  glRasterPos2f(3.9 / zoom_scale - center_x, 0.1 / zoom_scale);
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)std::string("E").c_str());
  glRasterPos2f(0.1 / zoom_scale, 3.9 / zoom_scale - center_y);
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)std::string("N").c_str());
}

void draw::agent_number(const uint16_t &ID)
{
  glRasterPos2f(-0.01, 0.035);
  glColor3f(1.0, 1.0, 1.0); // Background color

  std::stringstream ss;
  ss << (int)ID;
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)ss.str().c_str());
}

void draw::triangle(const float &scl)
{
  glPushMatrix();

  glBegin(GL_POLYGON);
  glColor3ub(200, 000, 000); // Red
  glVertex2f(-1 * scl,  1 * scl);
  glVertex2f(-1 * scl, -1 * scl);
  glVertex2f(2.0 * scl,  0 * scl);
  glEnd();

  glColor3ub(255, 255, 255); // White
  glPopMatrix();
}

void draw::circle(const float &d)
{
  float angle, x, y;
  glPushMatrix();
  glBegin(GL_POLYGON);
  glColor3ub(200, 000, 000); // Redish
  for (int i = 0; i <= 10; i++) { // Resolution
    angle = 2 * M_PI * i / 10;
    x = (d * yrat) * cos(angle);
    y = (d * xrat) * sin(angle);
    glVertex2d(x, y);
  }
  glEnd();

  glColor3ub(255, 255, 255); // White
  glPopMatrix();
}

void draw::circle_loop(const float &r)
{
  int num_segments = 30; // Resolution
  glPushMatrix();
  glLineWidth(1);
  glColor3ub(64, 64, 64); // gray
  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < num_segments; i++) {
    float theta = 2.0f * M_PI * float(i) / float(num_segments);//get the current angle
    float x = r * yrat * cosf(theta);                 //calculate the x component
    float y = r * xrat * sinf(theta);                 //calculate the y component
    glVertex2d(x, y);
  }
  glEnd();
  
  glColor3ub(255, 255, 255); // White
  glPopMatrix();
}

void draw::line(const float &x, const float &y)
{
  glPushMatrix();
  glLineWidth(2.5);
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINES);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(x * xrat, -y * yrat, 0);
  glEnd();
  glPopMatrix();
}

void draw::line(const float &x, const float &y, const float &width)
{
  glPushMatrix();
  glLineWidth(width);
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINES);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(x * xrat, -y * yrat, 0);
  glEnd();
  glPopMatrix();
}

void draw::line(const float &x, const float &y, const float &width, const color3ub &color)
{
  glPushMatrix();
  glLineWidth(width);
  glColor3ub(color.r, color.g, color.b);
  glBegin(GL_LINES);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(x * xrat, -y * yrat, 0);
  glEnd();
  glPopMatrix();
}


void draw::point()
{
  glPointSize(10.0);
  glBegin(GL_POINTS);
  glVertex3f(0, 0, 0);
  glEnd();
}

void draw::axes()
{
  float lineintensity = 1.0;
  glLineWidth(0.5);
  glBegin(GL_LINES);
  glLineStipple(1, 0xAAAA);  // [1]
  glEnable(GL_LINE_STIPPLE);
  glColor3ub(255 * lineintensity, 255 * lineintensity, 255 * lineintensity); // white
  glVertex3f(-1000,  0.0, 0.0);
  glVertex3f(1000.0,  0.0, 0.0);
  glEnd();

  glBegin(GL_LINES);
  glLineStipple(1, 0xAAAA);  // [1]
  glEnable(GL_LINE_STIPPLE);
  glColor3ub(255 * lineintensity, 255 * lineintensity, 255 * lineintensity); // white
  glVertex3f(0.0, -1000.0, 0.0);
  glVertex3f(0.0,  1000.0, 0.0);
  glEnd();
  glPopAttrib();
}

void draw::segment(const float &x0, const float &y0, const float &x1, const float &y1)
{
  glLineWidth(5);
  float lineintensity = 1.0;
  glBegin(GL_LINES);
  glColor3ub(128 * lineintensity, 128 * lineintensity, 128 * lineintensity); // white
  glVertex3f(x0 * xrat, y0 * yrat, 0.0);
  glVertex3f(x1 * xrat, y1 * yrat, 0.0);
  glEnd();
}

void draw::segment(const float &x0, const float &y0, const float &x1, const float &y1, const color3ub &color)
{
  glLineWidth(2);
  glBegin(GL_LINES);
  glColor3ub(color.r, color.g, color.b);
  glVertex3f(x0 * xrat, y0 * yrat, 0.0);
  glVertex3f(x1 * xrat, y1 * yrat, 0.0);
  glEnd();
}

void draw::agent(const uint16_t &ID, const float &x, const float &y, const float &orientation)
{
  glPushMatrix();
  glTranslatef(y * xrat, x * yrat, 0.0); // ENU to NED
  glRotatef(90.0 - rad2deg(orientation), 0.0, 0, 1);
  agents[ID]->animation(); // Uses the animation function defined by the agent in use
  agents[ID]->controller->animation(ID); // Draws additional stuff from the controller, such as sensors
  agent_number(ID);
  glPopMatrix();
}

void draw::velocity_arrow(const uint16_t &ID, const float &x, const float &y, const float &v_x, const float &v_y)
{
  glPushMatrix();
  glTranslatef(y * xrat, x * yrat, 0.0); // ENU to NED
  glRotatef(90.0, 0.0, 0.0, 1.0);
  line(v_x, v_y);
  glPopMatrix();
}

void draw::food(const float &x, const float &y)
{
  glPushMatrix();
  glTranslatef(y * xrat, x * yrat, 0.0);
  glRotatef(90, 0.0, 0.0, 1.0);
  point();
  glPopMatrix();
}
