/*
 * LidarImageProjector.cpp
 *
 *  Created on: Jun 15, 2010
 *      Author: moosmann
 */

#include "LidarImageProjector.h"

#include <cmath>

bool LidarImageProjector::getImageIndexRel(double xr, double yr, double zr, int &hi, int &vi, double &dist) const
{
  double xrs = xr*xr;
  double yrs = yr*yr;
  double zrs = zr*zr;
  //cout << boost::format("x/y/z = %f / %f / %f --> ") % xrs % yrs % zrs;
  double yawRAD = atan2(yr,xr); // atan2 returns in [-pi..+pi]
  double pitchRAD = atan2(zr,sqrt(xrs+yrs)); // returns in [-pi..+pi]

  dist = sqrt(xrs+yrs+zrs);
  return getImageIndexYP(yawRAD, pitchRAD, hi, vi);
}

