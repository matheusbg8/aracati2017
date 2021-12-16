#ifndef AERIALIMAGE_H
#define AERIALIMAGE_H

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <geodesy/utm.h>

using namespace std;
using namespace boost::filesystem;
using namespace cv;

class AerialImage
{
  Point2d imgRef[2], /// Two reference points in pixels
      UTMRef[2], /// The same two reference points in UTM
      UTM2Img_; /// The scale factor to transform UTM to pixel

  Mat mapImg; /// Current satellite image
  string mapName;
  bool loadYaml(const path &file);

public:
  AerialImage();

  void resizeToMaxCols(int maxCols);

  Rect getTranslateRectToFit(const Rect &r,
                              int &top, int &bottom,
                              int &left, int &right) const;

  bool loadMap(const path &mapFile);

  bool isOnMap(const Point2f &p);
  bool isOnMapImg(const Point2f &pImg);

  Point2d UTM2Img(const Point2d &UTMp) const;
  Point2d Img2UTM(const Point2d &ImgP) const;
  Point2d UTM2ImgRatio() const;

  const Mat & getMapImg() const;
  string name();

};

template<typename Point_T>
Point_T latLon2UTM(const Point_T &p, int *zone=nullptr)
{
  geographic_msgs::GeoPoint ll;

  ll.latitude = p.x;
  ll.longitude = p.y;
  ll.altitude = 0.0;

  geodesy::UTMPoint pt(ll);
  if(zone!= nullptr) *zone = pt.zone;
  return Point_T(pt.easting,pt.northing);
}

#endif // AERIALIMAGE_H
