#include "AerialImage.h"

bool AerialImage::loadYaml(const path &file)
{
  //  if(!is_regular_file(file))
  //    return false;
    //cout << "Load map config from " << file.string() << endl;

    FileStorage fs;
    fs.open(file.string(), FileStorage::READ |  FileStorage::FORMAT_YAML);

    if (!fs.isOpened())
    {
        cerr << "Failed to open " << file.string() << endl;
        return false;
    }

    const FileNode &nP1 = fs["p1"],
                   &nP2 = fs["p2"];

    int zone1,zone2;
    Point2d p1((double(nP1["lat"])),double(nP1["long"])),
            p2((double(nP2["lat"])),double(nP2["long"]));

    UTMRef[0] = latLon2UTM(p1,&zone1);

    imgRef[0].x =  double(nP1["pix_x"]);
    imgRef[0].y =  double(nP1["pix_y"]);

    UTMRef[1] = latLon2UTM(p2,&zone2);

    imgRef[1].x =  double(nP2["pix_x"]);
    imgRef[1].y =  double(nP2["pix_y"]);

    //cout << UTMRef[0] << " zone " << zone1 << endl
    //    << imgRef[0] << endl << endl
    //     << UTMRef[1] << " zone " << zone2 << endl
    //     << imgRef[1] << endl << endl;

    if(zone1 != zone2)
    {
      cout << "Warnig: Coords in different zone!!!!!" << endl;
      return false;
    }

    Point2d diffImg = imgRef[1] - imgRef[0],
            diffUTM = UTMRef[1] - UTMRef[0];

    // Compute scale transform from UTM to pixel
    // taking two reference points on both coordinate system

    UTM2Img_.x = diffImg.x/diffUTM.x;
    UTM2Img_.y = diffImg.y/diffUTM.y;

    return true;
}


AerialImage::AerialImage()
{

}

void AerialImage::resizeToMaxCols(int maxCols)
{
  double scale = double(maxCols)/mapImg.cols;

  resize(mapImg,mapImg,Size(), scale,scale);

  imgRef[0]*= scale;
  imgRef[1]*= scale;
  UTM2Img_*=scale;
}


/**
 * @brief AerialImage::getRectThatFitsIntoImg
 *  Giving a rect and an image, return an adjusted rect
 * that fits into the image. top,bottom, left and right
 * are output parameters that tells how many pixels
 * were missing on each direction. This information
 * can be used for a padding strategy.
 *  If the gave rect is completely out of the image
 * a empty rect is returned and -1 on the output
 * parameters.
 * @param r - Input - Initial rect.
 * @param top - Outout - changes on top of the rect.
 * @param bottom - Output - changes on bottom of the rect.
 * @param left - Outoput - changes on left of the rect.
 * @param right - Output - changens on right of the rect.
 * @return new rect that fits into the aerial image. An empty rect
 * is returned if it is not possible to fit the rect.
 */
Rect AerialImage::getTranslateRectToFit(const Rect &r, int &top, int &bottom, int &left, int &right) const
{
  top = bottom = left = right = 0;

  int minX = r.x, maxX = r.x + r.width,
      minY = r.y, maxY = r.y + r.height;

  if(maxX < 0 || minX > mapImg.cols ||
     maxY < 0 || minY > mapImg.rows)
  {
      top=bottom=left=right=-1;
      return Rect();
  }
  Rect fr(r);

  if(minX < 0)
  {
      fr.width+=minX; // Increased
      fr.x =0; // moved to right
      left=-minX;
  }

  if(maxX > mapImg.cols)
  {
      fr.width = mapImg.cols - fr.x;
      right = maxX - mapImg.cols;
  }

  if(minY < 0)
  {
      fr.height+=fr.y;
      fr.y = 0;
      top = -minY;
  }

  if(maxY > mapImg.rows)
  {
      fr.height = mapImg.rows - fr.y;
      bottom = maxY - mapImg.rows;
  }
  return fr;
}
bool AerialImage::loadMap(const path &mapFile)
{
  path yamlFile(mapFile),
       imgFile(mapFile);
  bool succes = false;

  yamlFile.replace_extension(".yaml");

  mapName = mapFile.stem().string();

  if(!loadYaml(yamlFile))
    return false;

  imgFile.replace_extension(".jpg");
  succes = is_regular(imgFile);

  // Try png
  if(!succes)
  {
    imgFile.replace_extension(".png");
    succes = is_regular(imgFile);
  }
  // Try jpeg
  if(!succes)
  {
    imgFile.replace_extension(".jpeg");
    succes = is_regular(imgFile);
  }
  if(!succes)
    return false;

  mapImg = imread(imgFile.string());
  if(mapImg.empty())
  {
    cerr << "Error: Could not open map img " << imgFile.string() <<endl;
    return false;
  }
  return true;
}

bool AerialImage::isOnMap(const Point2f &p)
{
  Point2i pImg = UTM2Img(p);
  if(pImg.x < 0 || pImg.x >= mapImg.cols ||
     pImg.y < 0 || pImg.y >= mapImg.rows)
    return false;
  return true;
}

bool AerialImage::isOnMapImg(const Point2f &pImg)
{
  if(pImg.x < 0 || pImg.x >= mapImg.cols ||
     pImg.y < 0 || pImg.y >= mapImg.rows)
    return false;
  return true;
}

Point2d AerialImage::UTM2Img(const Point2d &UTMp) const
{
  return Point2d (imgRef[0].x + (UTMp.x - UTMRef[0].x)*UTM2Img_.x,
                  imgRef[0].y + (UTMp.y - UTMRef[0].y)*UTM2Img_.y);

}

Point2d AerialImage::Img2UTM(const Point2d &ImgP) const
{
  // First convert img point to the origin (reference point 0)
  // Second apply the scale, now it is in UTM units and out of the center
  // Third translate to the UTM reference
  return Point2d (UTMRef[0].x + (ImgP.x - imgRef[0].x)/UTM2Img_.x,
      UTMRef[0].y + (ImgP.y - imgRef[0].y)/UTM2Img_.y);
}

Point2d AerialImage::UTM2ImgRatio() const
{
  return UTM2Img_;
}

const Mat &AerialImage::getMapImg() const
{
  return mapImg;
}

string AerialImage::name()
{
  return mapName;
}
