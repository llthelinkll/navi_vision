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

  // we have to split in to windown before pass to FAST because of core dumped for large image
  int w = 30;

  Mat dst = im ;
  for(int i=0;i<maxPyramidLevel;++i)
  {
    float scale = levelScaleFactor;
    if(scale ==0)
    {
      scale = 1;
    }
    int maxX = (float) dst.cols /  scale;
    int maxY = (float) dst.rows / scale;
    int nx = maxX/w;
    int ny = maxY/w;
    std::cout << "-------------" << '\n';
    std::cout << maxX << '\n';
    std::cout << maxY << '\n';
    std::cout << nx << '\n';
    std::cout << ny << '\n';
    resize(dst, dst, Size(maxX,maxY), 0, 0, INTER_LINEAR);
    for(int x = 0;x<nx;++x)
    {
      for(int y=0;y<ny;++y)
      {
        int startX = x*w;
        int endX = (x+1)*w;
        int startY = y*w;
        int endY = (y+1)*w;
        if (endX > im.cols-1)
        {
          endX = im.cols-1;
        }
        if (endY > im.rows-1)
        {
          endY = im.rows-1;
        }
        std::vector<KeyPoint> tempV;
        FAST(dst.colRange(startX,endX).rowRange(startY,endY),tempV,28,true);
        // set real position to image

        if(!tempV.empty()){
          for(std::vector<KeyPoint>::iterator it = tempV.begin();it != tempV.end();++it)
          {
            it->pt.x += startX;
            it->pt.y += startY;
            kpv[i].push_back(*it);
          }
        }

      }
    }

  }

}

void
ORB_Extractor::computePyramid(Mat& m)
{

}
