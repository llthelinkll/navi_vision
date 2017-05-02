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
  std::vector<std::vector<KeyPoint> > kpvvAllKeyPoints;
  kpvvAllKeyPoints.reserve(8);
  for(int i =0;i<8;i++)
  {
    kpvvAllKeyPoints.push_back(std::vector<KeyPoint>());
  }

  Mat descriptors;
  ORB_Extractor extractor;
  extractor.extractFeature(im,kpvvAllKeyPoints,descriptors);

  std::cout << kpvvAllKeyPoints.size() << '\n';
  for (std::vector<std::vector<KeyPoint> >::iterator it = kpvvAllKeyPoints.begin();it != kpvvAllKeyPoints.end();++it){
    Mat disp = im.clone();
    for(std::vector<KeyPoint>::iterator kit = it->begin();kit != it->end();++kit)
    {
      rectangle( disp, kit->pt, Point(kit->pt.x + 10,kit->pt.y + 10), Scalar(255,0,0), 2, 8, 0 );
      std::cout << kit->pt.x << '\n';
      std::cout << kit->pt.y << '\n';
    }
    imshow("output",disp);
    waitKey(0);
  }

  return 0;
}
