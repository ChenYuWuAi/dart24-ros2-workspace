#include "DHVideoCapture.h"

DHVideoCapture _videoCapture;

void init(int exposure_time) //1-3ms 0=>auto
{
    
    if (!_videoCapture.open(0,2))
    {
        printf("!!!!!!!!!!!!!!!!!!!!!no camera!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        exit(0);
    }
    _videoCapture.setExposureTime(exposure_time);
    _videoCapture.setVideoFormat(1280, 1024, true);
    _videoCapture.setFPS(30.0);
    _videoCapture.setBalanceRatio(1.6, 1.3, 2.0, true);
    _videoCapture.setGain(1);
    _videoCapture.setGamma(1);

    _videoCapture.startStream();
    _videoCapture.closeStream();

    _videoCapture.startStream();    
}

// _videoCapture.read(Mat)