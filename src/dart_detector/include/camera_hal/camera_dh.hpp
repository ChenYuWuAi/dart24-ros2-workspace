/**
 * @file camera_dh.hpp
 * @note 大华相机HAL驱动实现
 */
#ifndef CAMERA_DH_HPP
#define CAMERA_DH_HPP

#include <camera_hal/camera_driver.hpp>
#include <DHCameraDriver/DHVideoCapture.h>

namespace CameraHAL
{

    class CameraDriver_DH : public CameraDriver
    {
    private:
        DHVideoCapture *_videoCapture;

    public:
        CameraDriver_DH();
        ~CameraDriver_DH();

        bool open(std::unordered_map<std::string, std::string> &params) override;
        bool write(std::string para_name, std::string para_value) override;
        bool read(cv::Mat &image) override;
        bool close() override;
    };
} // namespace CameraHAL
#endif