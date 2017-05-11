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
    ORB* orb;
    // scale factor
    float levelScaleFactor = 1.3f;

    // max pyramid level
    unsigned int maxPyramidLevel = 8;

    // scale factor for each level
    std::vector<unsigned int> uivScaleFactorLevel;

    /*
      image in each pyramid level
    */
    std::vector<cv::Mat> mvImagePyramid;

    /*
      this is max of u for each v
    */
    std::vector<int> umax;

    /*
      Pattern for Descriptor creater
    */
    std::vector<cv::Point> pattern;

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
