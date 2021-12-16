#ifndef ODOMETRY
#define ODOMETRY

#include <opencv2/core.hpp>

// Name spaces
using namespace cv;

class Odometry
{
  Point2d m_p; // Current position
  double m_theta=0.0;
  Point2d m_v; // Current velocity
  double m_wTheta=0.0;

  double m_time=0.0;
  bool m_isIntialized=false;
public:
  Odometry();

  void setInitialPose(double time, double px, double py, double theta);

  void updateVelocity(double time,
                      double vx, double vy,
                      double wTheta);

  bool predict(double time,
               double &x, double &y,
               double &theta);

  bool isInitialized();
};

#endif // ODOMETRY
