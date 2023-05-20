#include "draw.h"
#include "trigonometry.h"
#include <cmath>
#include "fitness_functions.h"
#include "eigen3/Eigen/Dense"

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

void draw::triangle(const Eigen::MatrixXf points, const float scl)
{
  glPushMatrix();

  glBegin(GL_POLYGON);
  glColor3ub(200, 000, 000); // Red

  glVertex3f(points(0,0) * scl, points(0,1) * scl, points(0,2) * scl); // top left
  glVertex3f(points(1,0) * scl, points(1,1) * scl, points(1,2) * scl); // apex of sensor
  // glColor3ub(255, 255, 255); // White
  glVertex3f(points(2,0) * scl, points(2,1) * scl, points(2,2) * scl); // top right

  // glColor3ub(255, 255, 255); // White
  // glVertex3f(points(3,0) * scl, points(3,1) * scl, points(3,2) * scl); // bottom left
  // glVertex3f(points(4,0) * scl, points(4,1) * scl, points(4,2) * scl); // apex of sensor
  // glVertex3f(points(2,0) * scl, points(2,1) * scl, points(2,2) * scl); // bottom right


  // glVertex3f(points(4,0) * scl, points(4,1) * scl, points(4,2) * scl); // apex of sensor
  

  // apex of sensor
  // glVertex3f(points(4,0) * scl, points(4,1) * scl, points(4,2) * scl);
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

void draw::line(const Eigen::Vector3f &p1, const Eigen::Vector3f& p2, const float &width, const Vector<float> color)
{
  glPushMatrix();
  glLineWidth(width);
  glColor3f(color[0], color[1], color[2]);
  glBegin(GL_LINES);
  glVertex3f(p1[0], p1[1], p1[2]);
  glVertex3f(p2[0], p2[1], p2[2]);
  glEnd();
  glPopMatrix();
}

void draw::rect(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3, const Eigen::Vector3f &p4, const float &width, const Vector<float> color){
    glPushMatrix();
    glLineWidth(width);
    glColor3f(color[0], color[1], color[2]);
    glBegin(GL_LINE_STRIP);
    glVertex3f(p1[0], p1[1], p1[2]);
    glVertex3f(p2[0], p2[1], p2[2]);
    glVertex3f(p3[0], p3[1], p3[2]);
    glVertex3f(p4[0], p4[1], p4[2]);
    glVertex3f(p1[0], p1[1], p1[2]);
    glEnd();
    glPopMatrix();

}

void draw::polygon(const Eigen::MatrixXf &points, const float &width, const Vector<float> color){
    glPushMatrix();
    glBegin(GL_POLYGON);
    glColor4f(color[0], color[1], color[2], color[3]); // Redish
    for (int i = 0; i < points.rows(); i++) { // Resolution
      glVertex3f(points(i,0),points(i,1),points(i,2));
    }
    // glVertex3f(points(0,0),points(0,1),points(0,2)); // to close the polygon
    glEnd();

    glColor3ub(255, 255, 255); // White
    glPopMatrix();
}

void draw::polyline(const Eigen::MatrixXf &points, const float &width, const Vector<float> color)
{
  glPushMatrix();
  glLineWidth(width);
  glColor3f(color[0], color[1], color[2]);
  int l = points.rows();
  for (int i=0; i < l; i++){
    glBegin(GL_LINES);
    glVertex3f(points(i%l,0), points(i%l,1), points(i%l,2));
    glVertex3f(points((i+1)%l,0), points((i+1)%l,1), points((i+1)%l,2));
    // glVertex3f(0, 0, 0);
    // glVertex3f(1, 1, 5);
    
    glEnd();
  }
  glPopMatrix();
}

void draw::points(Eigen::MatrixXf p, Vector<float> color){
  glPointSize(10.0);
  glColor4f(color[0], color[1], color[2], color[3]);
  glBegin(GL_POINTS);
  for (int i=0; i < p.rows(); i++){
    glVertex3f(p(i,0), p(i,1), p(i,2));
  }
  glEnd();
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

void draw::agent(const uint16_t &ID, const float &x, const float &y, const float &orientation)
{
  glPushMatrix();
  glTranslatef(x * xrat, y * yrat, 0.0); // ENU to NED
  glRotatef(rad2deg(orientation), 0.0, 0, 1); // This brings the reference frame to the body frame 
  s[ID]->animation(); // Uses the animation function defined by the agent in use
  s[ID]->controller->animation(ID); // Draws additional stuff from the controller, such as sensors
  agent_number(ID);
  glPopMatrix();
}
void draw::agent(const uint16_t &ID, const State state)
{
  glPushMatrix();
  // print(state.pose.pos);
  glTranslatef(state.pose.pos[0] * xrat, state.pose.pos[1] * yrat, state.pose.pos[2]); // ENU to NED
  Eigen::Vector4f axis_angle = state.pose.toAxisAngle();
  glRotatef(rad2deg(axis_angle[0]), axis_angle[1], axis_angle[2], axis_angle[3]);
  s[ID]->animation(); // Uses the animation function defined by the agent in use
  s[ID]->controller->animation(ID); // Draws additional stuff from the controller, such as sensors
  agent_number(ID);
  glPopMatrix();
}

void draw::velocity_arrow(const uint16_t &ID, const State state)
{
  glPushMatrix();
  glTranslatef(state.pose.pos[0] * xrat, state.pose.pos[1] * yrat, 0.0); // ENU to NED
  glRotatef(0.0, 0.0, 0.0, 1.0);
  line(state.vel[0], state.vel[1]);
  glPopMatrix();
}

void draw::velocity_arrow(const uint16_t &ID, const float &x, const float &y, const float &v_x, const float &v_y)
{
  glPushMatrix();
  glTranslatef(x * xrat, y * yrat, 0.0); // ENU to NED
  glRotatef(0.0, 0.0, 0.0, 1.0);
  line(v_x, v_y);
  glPopMatrix();
}

void draw::sphere(const Eigen::Vector3f pos, Eigen::Vector4f axis_angle, float radius){
  glPushMatrix();
  glTranslatef(pos[0], pos[1], pos[2]); // ENU to NED
  glRotatef(axis_angle[0], axis_angle[1], axis_angle[2], axis_angle[3]);
  // line(v_x, v_y);
  glutWireSphere(radius,1,1);
  glPopMatrix();
}

void draw::food(const float &x, const float &y)
{
  glPushMatrix();
  glTranslatef(x * xrat, y * yrat, 0.0);
  glRotatef(90, 0.0, 0.0, 1.0);
  point();
  glPopMatrix();
}
