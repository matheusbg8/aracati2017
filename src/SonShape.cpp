#include "SonShape.h"

/**
 * @brief SonShape::getPoints - Return points on current heading
 * "heading" and position "p". All points are in UTM.
 * @param pts - Output transformed points
 * @param heading - Desired heading in rads
 * @param p - Desired position in meters (UTM).
 */
void SonShape::getPoints(vector<Point2d> &pts,double heading,const Point2d &p)
{
  // Apply rotation + translation
  pts.resize(m_pts.size());

  double s= sin(heading), c=cos(heading);

  for(uint i = 0 ; i < pts.size();i++)
  {
    Point2d &nP = pts[i];
    const Point2d & cP = m_pts[i];

    // We flip y signal because image y grows down
    // On UTM to img it will flip again
    nP.x =   cP.x*c - cP.y*s;
    nP.y = -(cP.x*s + cP.y*c);

    nP.x +=  p.x;
    nP.y +=  p.y;

  }
}

SonShape::SonShape()
{

}

/**
 * @brief SonShape::initShape - Create the sonar field
 * of view polygon based on the sonar settings
 * @param opening - Sonar field of view opening in degrees.
 * @param maxRange - Sonar maximum range in meters.
 * @param minRange - Sonar minimum range in meters.
 */
void SonShape::initShape(double opening,
                         double maxRange, double minRange)
{
  int minRangeArcPoints=5, // Must be > 1
      maxRangeArcPoints=15, // Must be > 1
      pId = 0;

  m_range = maxRange;
  m_openning = opening;

  m_pts.resize(minRangeArcPoints+maxRangeArcPoints);

  double radBearing = opening*M_PI/180.0,
         radInc;

  // Resize vector of points
  m_pts.resize(minRangeArcPoints+maxRangeArcPoints);

  // Sonar origin is (0,0)

  // Maximum range arc (15 points)
  int kPts = 0;
  radInc = radBearing/(maxRangeArcPoints-1);

  for(double currentRad = -radBearing/2.0;
      kPts < maxRangeArcPoints;
      currentRad+= radInc) // Clockwise iteration
  {
      m_pts[pId++] = Point2d(sin(currentRad), -cos(currentRad))*maxRange;
      kPts++;
  }

  // Minimum range arc (5 points)
  kPts = 0;
  radInc = radBearing/(minRangeArcPoints-1);
  for(double currentRad = radBearing/2.0;
      kPts < minRangeArcPoints;
      currentRad-= radInc) // Counter-Clockwise iteration
  {
      m_pts[pId++] = Point2d(sin(currentRad), -cos(currentRad))*minRange;
      kPts++;
  }
}

void SonShape::drawPoly(Mat &img,
                        const AerialImage &ai,
                        Point2d &UTMPosition,
                        double heading,
                        const Scalar& color,
                        int thickness)
{
  vector<Point2d> newPts;
  getPoints(newPts,heading,UTMPosition);

  // Draw poly
  int numPts = newPts.size();
  Point pts[newPts.size()];

  for(uint i = 0; i < newPts.size(); i++)
  {
    const Point2d &imgP = ai.UTM2Img(newPts[i]);

    pts[i] = Point(round(imgP.x),
                   round(imgP.y));
  }

  int npts[] = {numPts};
  const Point* ppt[1] = { pts };

  // Draw poly on aerial image
  polylines(img,
            ppt,npts,
            1,true,
            color,thickness);
}

Mat SonShape::cropSonShape(const AerialImage &ai, Point2d &UTMPosition, double heading)
{
  vector<Point2d> newPts;
  getPoints(newPts,heading,UTMPosition);

  int numPts = newPts.size();
  Point pts[newPts.size()],
      minP,maxP;


  for(uint i = 0; i < newPts.size(); i++)
  {
    const Point2d &imgP = ai.UTM2Img(newPts[i]);
    Point &nP = pts[i];

    nP = Point(round(imgP.x),
               round(imgP.y));
    if(i==0)
    {
      minP = maxP = nP;
    }else
    {
      if(minP.x > nP.x) minP.x = nP.x;
      if(minP.y > nP.y) minP.y = nP.y;
      if(maxP.x < nP.x) maxP.x = nP.x;
      if(maxP.y < nP.y) maxP.y = nP.y;
    }
  }

  // Crop sat img
  // A boding box rect with 3 extra pixels each side
  Rect rect(minP,maxP);
  rect.x-=3; rect.y-=3; rect.width+=6; rect.height+=6;

  // Find roi that fits
  int top,bottom, left,right;
  Rect rectThatFits = ai.getTranslateRectToFit(rect,
                                             top,bottom,
                                             left,right);

  if(rectThatFits.width==0 || rectThatFits.height ==0)
  {
      cout << "Sat image crop error! Desired crop out of the map!!" << endl;
      return Mat();
  }

  Mat sonarFoVRect; // Rect crop from the map

  // Fill the missing part with padding
  Mat cropThatFits = ai.getMapImg()(rectThatFits);

  copyMakeBorder(cropThatFits,sonarFoVRect,
                 top,bottom,left,right,
                 BORDER_REFLECT_101);

  // Transform the sonar FoV poly in an image mask
  Mat sonarMask(rect.height, rect.width,CV_8UC1,Scalar(0));

  int npts[] = {numPts};
  const Point* ppt[1] = { pts };
  fillPoly(sonarMask,ppt,npts,1,Scalar(255),
           LINE_8,0,Point(-rect.x,-rect.y));

  // Create the image correspondent of the sonar FoV
  Mat sonFoVImg;
  sonarFoVRect.copyTo(sonFoVImg,sonarMask);

  // Compute Sonar Position on each coordinate system
  double FoVRad = m_openning*M_PI/180.0;

  Point2d sonarPositionOnImg(ai.UTM2Img(UTMPosition)),
          sonarPositionOnResult( sonarPositionOnImg.x - rect.x,
                                 sonarPositionOnImg.y - rect.y),
          sonarFoVSize(2*m_range*cos(M_PI_2-(FoVRad/2.0)), // width
                       m_range // Height
                       );

  // ===== Rotating the FoV image regarding sonar heading =============
  Point2d Utm2Img = ai.UTM2ImgRatio();
  Size finalImgSize(abs(int(sonarFoVSize.x*Utm2Img.x)),
                    abs(int(sonarFoVSize.y*Utm2Img.y)));

  Mat afimTransformMatrix = getRotationMatrix2D(sonarPositionOnResult,
                                                heading*180.0/M_PI,1.0);

  // Translation Correction
  afimTransformMatrix.at<double>(0,2) += -sonarPositionOnResult.x+finalImgSize.width/2;
  afimTransformMatrix.at<double>(1,2) += -sonarPositionOnResult.y+finalImgSize.height;

  // Apply affine transform and warp the image
  warpAffine( sonFoVImg,
              sonFoVImg,
              afimTransformMatrix,
              finalImgSize,
              INTER_NEAREST
             );

  return sonFoVImg;
}
