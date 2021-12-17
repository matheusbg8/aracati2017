#include "CircularVector.h"
#include <opencv2/highgui.hpp>
#include <iostream>

uint CircularVector::prvId(uint id)
{
  // Prevent overflow
  return uint((int(id)-1+maxSize)%maxSize);
}

uint CircularVector::nxtId(uint id)
{
  return uint((id+1)%maxSize);
}

uint CircularVector::elementId(uint position)
{
  if(n < maxSize)
    return position;
  else
    return uint((index+position)%maxSize);
}

uint CircularVector::size()
{
  return uint(n);
}

bool CircularVector::empty()
{
  return n==0;
}

uint CircularVector::getMaxSize()
{
  return maxSize;
}

const Point2d &CircularVector::operator [](uint id)
{
  uint internal_id = elementId(id);
  return mValue[internal_id];
}

CircularVector::CircularVector(uint maxSize):
  mValue(maxSize),
  maxSize(maxSize)
{

}

void CircularVector::push(const Point2d &value)
{
  mValue[uint(index)] = value;

  index = nxtId(index);

  if(n < maxSize)
    n++;
}

const Point2d &CircularVector::last()
{
  return mValue[prvId(index)];
}

void CircularVector::clear()
{
  index = 0;
  n = 0;
}
