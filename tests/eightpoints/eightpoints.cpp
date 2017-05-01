#include "ORB_Extractor.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace navi_vision;

int main(int argc, char const *argv[]) {
  /* code */

  std::string path = "/home/thanakorn/datasets/KITTI/sequences/00/image_0/000000.png";

  Mat im;
  im = imread(path,CV_LOAD_IMAGE_UNCHANGED);

  std::vector<KeyPoint> kpv;
  Mat descriptors;
  ORB_Extractor extractor;
  extractor.extractFeature(im,kpv,descriptors);

  std::cout << kpv.size() << '\n';
  for (std::vector<KeyPoint>::iterator it = kpv.begin();it != kpv.end();++it){
    rectangle( im, it->pt, Point(it->pt.x + 10,it->pt.y + 10), Scalar(255,0,0), 2, 8, 0 );
    // std::cout << it->pt.x << '\n';
    // std::cout << it->pt.y << '\n';
  }
  imshow("output",im);
  waitKey(0);

  return 0;
}
