/**
 * @file camera_v4l2.hpp
 * @note V4L2相机HAL驱动实现
 */
#ifndef CAMERA_V4L2_HPP
#define CAMERA_V4L2_HPP

#include <camera_hal/camera_driver.hpp>
#include "linux/videodev2.h"

#define BUFFER_COUNT 4

namespace CameraHAL
{
    /**
     * @brief V4L2相机驱动
     */

    class CameraDriver_V4L2 : public CameraDriver
    {
    private:
        int fd = -1;
        struct v4l2_format fmt;
        struct v4l2_buffer buf;
        struct v4l2_requestbuffers req;
        void *buffer_start[BUFFER_COUNT];

    public:
        CameraDriver_V4L2();
        ~CameraDriver_V4L2();

        bool open(std::unordered_map<std::string, std::string> &params) override;
        bool write(std::string para_name, std::string para_value) override;
        bool read(cv::Mat &image) override;
        bool close() override;
    };
} // namespace CameraHAL

#endif