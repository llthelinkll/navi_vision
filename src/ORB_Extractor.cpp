#include "ORB_Extractor.hpp"

using namespace navi_vision;
using namespace cv;

ORB_Extractor::ORB_Extractor()
{
  
}

void
ORB_Extractor::extractFeature(Mat& im,std::vector<std::vector<KeyPoint> >& kpv,OutputArray _descriptors)
{

  Mat descriptors;
  _descriptors.create(100, 32, CV_8U);

  Mat dst ;
  for(int i=0;i<maxPyramidLevel;++i){
    std::cout << "/* message */" << '\n';
    pyrDown(im,dst,Size(im.cols / (levelScaleFactor*i),im.rows / (levelScaleFactor*i)));
    std::cout << "end" << '\n';
    FAST(dst,kpv[i],28,true);
    // TODO split image for FAST -> core dumped (make window)
  }

}

void
ORB_Extractor::computePyramid(Mat& m)
{

}
