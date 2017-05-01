#include "ORB_Extractor.hpp"

using namespace navi_vision;
using namespace cv;

ORB_Extractor::ORB_Extractor()
{

}

void
ORB_Extractor::extractFeature(Mat& im,std::vector<KeyPoint>& kpv,OutputArray _descriptors)
{

  Mat descriptors;
  _descriptors.create(100, 32, CV_8U);

  // TODO create level pointer
  FAST(im,kpv,28,true);

}

void
ORB_Extractor::computePyramid(Mat& m)
{

}
