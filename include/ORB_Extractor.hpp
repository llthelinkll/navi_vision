#ifndef ORB_EXTRACTOR_H
#define ORB_EXTRACTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>

using namespace cv;

namespace navi_vision {
  /*
    image feature extractor class
  */
  class ORB_Extractor{
  public:

    float levelScaleFactor = 1.3;
    int maxPyramidLevel = 8;


    ORB_Extractor();

    /*
      get keypoints from image(Mat)
      input : image (Mat)
      output :  keypoints (vector<KeyPoint>)
                descriptors for each keypoint (OutputArray)
      NOTE : cv::OutputArray is struct the can put Mat vector and it pass by ref itself
    */
    void extractFeature(Mat& m,std::vector<std::vector<KeyPoint> >& kpv,OutputArray descriptors);

    void computePyramid(Mat& m);

  };
}


#endif
