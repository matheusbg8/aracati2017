#ifndef SONSHAPE_H
#define SONSHAPE_H

#include "AerialImage.h"

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

class SonShape
{

  vector<Point2d> m_pts;
  double m_openning=-1.0,
         m_range=-1.0;

  void getPoints(vector<Point2d> &pts,
                 double heading,
                 const Point2d &p=Point2d(0.0,0.0));

public:
  SonShape();

  void initShape(double opening,
                 double maxRange,
                 double minRange);

  void drawPoly(Mat &img, const AerialImage &ai,
                const Point2d &UTMPosition, double heading,
                const Scalar &color=Scalar(255,0,255),
                int thickness=1);

  Mat cropSonShape(const AerialImage &ai,
                   const Point2d &UTMPosition, double heading);


};

#endif // SONSHAPE_H
