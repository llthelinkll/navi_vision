#include "ORB_Extractor.hpp"

using namespace navi_vision;
using namespace cv;

namespace navi_vision
{
  const int PATCH_SIZE = 31;
  const int HALF_PATCH_SIZE = 15;
  const int EDGE_THRESHOLD = 19;


  /*
    compute angle of keypoint (ORB)(Centroid calculation)
  */
  static float IC_Angle(const Mat& image, Point2f pt,  const std::vector<int> & u_max)
  {
      int m_01 = 0, m_10 = 0;

      const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

      // Treat the center line differently, v=0
      for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
          m_10 += u * center[u];

      // Go line by line in the circuI853lar patch
      int step = (int)image.step1();
      std::cout << step << '\n';
      for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
      {
          // Proceed over the two lines
          int v_sum = 0;
          int d = u_max[v];
          std::cout << d << '\n';
          for (int u = -d; u <= d; ++u)
          {
              std::cout << u+v*step << '\n';
              std::cout << u-v*step << '\n';

              int val_plus = center[u + v*step], val_minus = center[u - v*step];
              v_sum += (val_plus - val_minus);
              m_10 += u * (val_plus + val_minus);
          }
          m_01 += v * v_sum;
      }

      return fastAtan2((float)m_01, (float)m_10);
  }
}


ORB_Extractor::ORB_Extractor()
{
  uivScaleFactorLevel.reserve(maxPyramidLevel);
  mvImagePyramid.reserve(maxPyramidLevel);
  uivScaleFactorLevel[0] = 1.0f;
  for(unsigned int i=1;i<maxPyramidLevel;++i)
  {
    uivScaleFactorLevel[i] *= uivScaleFactorLevel[i-1];
  }

  //This is for orientation
  // pre-compute the end of a row in a circular patch
  umax.reserve(HALF_PATCH_SIZE + 1);

  int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
  int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
  const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
  for (v = 0; v <= vmax; ++v)
      umax[v] = cvRound(sqrt(hp2 - v * v));

  // Make sure we are symmetric
  for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
  {
      while (umax[v0] == umax[v0 + 1])
          ++v0;
      umax[v] = v0;
      ++v0;
  }
}

void
ORB_Extractor::extractFeature(Mat& im,std::vector<std::vector<KeyPoint> >& kpv,OutputArray _descriptors)
{

  Mat descriptors;
  _descriptors.create(100, 32, CV_8U);

  // we have to split in to windown before pass to FAST because of core dumped for large image
  // we can adapt threshold for each window(cell)
  int w = 30;
  int imMaxX = im.cols;
  int imMaxY = im.rows;

  Mat dst = im ;
  for(int i=0;i<maxPyramidLevel;++i)
  {
    float scale = levelScaleFactor;
    if(i ==0)
    {
      scale = 1;
    }
    int maxX = (float) dst.cols /  scale;
    int maxY = (float) dst.rows / scale;
    int nx = maxX/w;
    int ny = maxY/w;
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

        for(std::vector<KeyPoint>::iterator it = tempV.begin();it != tempV.end();++it)
        {
          // be aware about keypoint that close to bounder
          if(it->pt.x <= imMaxX - HALF_PATCH_SIZE && it->pt.y <= imMaxY - HALF_PATCH_SIZE && it->pt.y >= HALF_PATCH_SIZE && it->pt.x >= HALF_PATCH_SIZE)
          {
            it->pt.x += startX;
            it->pt.y += startY;
            it->pt.x *= pow(levelScaleFactor,i);
            it->pt.y *= pow(levelScaleFactor,i);
            it->size *= uivScaleFactorLevel[i];
            it->octave = i;
            it->angle = IC_Angle(im,it->pt,umax);
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
