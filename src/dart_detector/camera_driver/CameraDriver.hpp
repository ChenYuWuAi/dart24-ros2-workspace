#include <opencv2/opencv.hpp>

class CameraDriver
{
  public:
  virtual bool closeStream()=0;
  virtual bool read(cv::Mat& image)=0;
};