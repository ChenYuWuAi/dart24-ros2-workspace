/**
 * @file camera_driver.hpp
 * @note 飞镖应用层相机HAL驱动，兼容DH工业相机和V4L2 Linux相机
 */
#ifndef CAMERA_DRIVER_HPP
#define CAMERA_DRIVER_HPP

#include <opencv2/opencv.hpp>

namespace CameraHAL
{
    /**
     * @brief 相机驱动基类
     */
    class CameraDriver
    {
    public:
        bool isOpened = false;
        
        /**
         * @brief 打开相机
         * @param params 字符串表示的相机参数
         * @return 是否成功
         */
        virtual bool open(std::unordered_map<std::string, std::string> &params) = 0;
        
        /**
         * @brief 写入参数
         * @param para_name 参数名
         * @param para_value 参数值
         * @return 是否成功
         */
        virtual bool write(std::string para_name, std::string para_value) = 0;

        /**
         * @brief 获取MAT格式图像
         * @param image 图像
         * @return 是否成功
         */
        virtual bool read(cv::Mat &image) = 0;

        /**
         * @brief 关闭相机
         * @return 是否成功
         */
        virtual bool close() = 0;
    };
} // namespace CameraHAL

#endif