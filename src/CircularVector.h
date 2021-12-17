#ifndef CIRCULAR_VECTOR_H
#define CIRCULAR_VECTOR_H

#include <opencv4/opencv2/core.hpp>

#include <vector>

using namespace std;
using namespace cv;

class CircularVector
{
  typedef unsigned int uint;

  vector<Point2d> mValue;

// ===== Navigation Stuff ======
  uint n=0, maxSize,index=0;

  uint prvId(uint id);
  uint nxtId(uint id);
  uint elementId(uint position);

public:
  CircularVector(uint maxSize=10u);
  uint size();
  bool empty();
  uint getMaxSize();

  const Point2d &operator [](uint id);
  void push(const Point2d &value);

  const Point2d& last();

  void clear();
};

#endif // CIRCULAR_VECTOR_H
