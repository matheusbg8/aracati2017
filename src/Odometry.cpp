#include "Odometry.h"

#include <iostream>
using namespace std;

Odometry::Odometry()
{
}

void Odometry::setInitialPose(double time,
                              double px, double py,
                              double theta)
{
  m_p = Point2d(px,py);
  this->m_theta = theta;
  m_time = time;
  m_isIntialized=true;
}

void Odometry::updateVelocity(double time,
                              double vx, double vy,
                              double wTheta)
{
  double dt = time - m_time;

  if(dt < 0.0)
  { // Invalid time
    cout << "Invalid time!" << endl;
    return;
  }

  double dx = m_v.x*dt,
         dy = m_v.y*dt, // Displacement on body frame
         dYaw = wTheta*dt;

  m_theta -= dYaw;

  // Convert body velocity to global velocity
  double co = cos(m_theta), so=sin(m_theta);
  m_p.x +=  dx*so +dy*co;
  m_p.y +=  dx*co -dy*so;

  // Save current velocity and time
  m_v.x = vx; m_v.y = vy;
  m_wTheta = wTheta;

  m_time = time;

  m_isIntialized=true;
}

bool Odometry::predict(double time,
                       double &x, double &y, double &theta)
{
  x = y = theta = 0.0;
  
  double dt = time - m_time;

  if(dt < 0.0)
  { // Invalid time
    cout << "Invalid time!" << endl;
    return false;
  }

  if(dt == 0.0)
  {
    x=m_p.x;y=m_p.y;theta=this->m_theta;
    return true;
  }

  double dx = m_v.x*dt,
         dy = m_v.y*dt; // Displacement on body frame

  double dYaw = m_wTheta*dt;

  // Convert body velocity to global velocity
  double co = cos(theta), so=sin(theta);
  x = m_p.x + dx*co - dy*so;
  y = m_p.y + dx*so + dy*co;
  theta = m_theta + dYaw;

  if(theta> 2*M_PI) theta-=2*M_PI;
  else if(theta< 0.0) theta+=2*M_PI;
  return true;
}

bool Odometry::isInitialized()
{
  return m_isIntialized;
}
