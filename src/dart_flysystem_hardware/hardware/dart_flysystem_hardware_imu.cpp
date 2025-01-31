/**
 * @file dart_flysystem_hardware_imu.cpp
 * @brief 飞镖飞行系统Hardware IMU Interface
 */
#include "dart_flysystem_hardware/dart_flysystem_hardware_imu.hpp"

// Linux Serial Libary
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <termios.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"

#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define ORIENTATION_UPDATE 0x10
#define READ_UPDATE 0x80

static inline int serialRead(int fd, uint8_t *data, uint32_t len)
{
    return read(fd, data, len);
}

namespace dart_flysystem_hardware
{
    int DartFlySystemHardwareIMU::serial_fd_ = -1;
    volatile char DartFlySystemHardwareIMU::new_data_ = 0;

    hardware_interface::CallbackReturn DartFlySystemHardwareIMU::on_init(const hardware_interface::HardwareInfo &info)
    {
        // 初始化超类
        if (
            SensorInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 检查参数是否存在
        if (info_.hardware_parameters.empty())
        {
            RCLCPP_ERROR(get_logger(), "IMU Hardware Interface Init Error: No Parameters");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters.find("imu_address") == info_.hardware_parameters.end())
        {
            RCLCPP_ERROR(get_logger(), "IMU Hardware Interface Init Error: No IMU Address");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters.find("serial_file") == info_.hardware_parameters.end())
        {
            RCLCPP_ERROR(get_logger(), "IMU Hardware Interface Init Error: No Serial File");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters.find("baudrate") == info_.hardware_parameters.end())
        {
            RCLCPP_ERROR(get_logger(), "IMU Hardware Interface Init Error: No Baudrate");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.sensors.empty())
        {
            RCLCPP_ERROR(get_logger(), "IMU Hardware Interface Init Error: No Sensors");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "IMU Hardware Interface Init");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    speed_t DartFlySystemHardwareIMU::getBaudrate(int baudrate)
    {
        switch (baudrate)
        {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        default:
            return B0;
        }
    }

    hardware_interface::CallbackReturn DartFlySystemHardwareIMU::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        try
        {
            // 读取串口配置
            serial_file_ = info_.hardware_parameters["serial_file"];
            // 将info_.hardware_parameters["baudrate"]转换为波特率
            if (getBaudrate(std::stoi(info_.hardware_parameters["baudrate"])) == B0)
            {
                RCLCPP_ERROR(get_logger(), "Can't Read Baudrate");
                return hardware_interface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(get_logger(), "IMU Serial File: %s, Baudrate: %d", serial_file_.c_str(), std::stoi(info_.hardware_parameters["baudrate"]));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Can't Read Serial File");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (WitInit(WIT_PROTOCOL_NORMAL, std::stoi(info_.hardware_parameters["imu_address"])) != WIT_HAL_OK)
        {
            RCLCPP_ERROR(get_logger(), "Can't Init IMU lib, check address: %s", info_.hardware_parameters["imu_address"].c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void DartFlySystemHardwareIMU::serialWrite(uint8_t *data, uint32_t len)
    {
        write(serial_fd_, data, len);
    }

    void DartFlySystemHardwareIMU::updateNotifier(uint32_t uiReg, uint32_t uiRegNum)
    {
        int i;
        for (i = 0; i < uiRegNum; i++)
        {
            switch (uiReg)
            {
                //            case AX:
                //            case AY:
            case AZ:
                new_data_ |= ACC_UPDATE;
                break;
                //            case GX:
                //            case GY:
            case GZ:
                new_data_ |= GYRO_UPDATE;
                break;
                //            case HX:
                //            case HY:
            case q3:
                new_data_ |= ORIENTATION_UPDATE;
                break;
            }
            uiReg++;
        }
    }

    void DartFlySystemHardwareIMU::serialDaemonThread()
    {
        RCLCPP_INFO(get_logger(), "IMU Serial Thread Begin");
        while (rclcpp::ok())
        {
            RCLCPP_INFO(get_logger(), "Initializing IMU Serial Port %s", serial_file_.c_str());
            // 打开串口
            serial_fd_ = open(serial_file_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); // 参数O_RDWR表示读写方式打开，O_NOCTTY表示不将设备分配为控制终端，O_NDELAY表示不关心DCD信号线
            if (serial_fd_ < 0)
            {
                RCLCPP_ERROR(get_logger(), "Can't Open IMU Serial Port %s, retrying in 1s", serial_file_.c_str());
                serial_thread_running_ = false;
                close(serial_fd_);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
            WitSerialWriteRegister(DartFlySystemHardwareIMU::serialWrite);
            WitRegisterCallBack(DartFlySystemHardwareIMU::updateNotifier);

            // 串口配置
            struct termios options;
            tcgetattr(serial_fd_, &options);

            cfsetispeed(&options, getBaudrate(std::stoi(info_.hardware_parameters["baudrate"])));
            cfsetospeed(&options, getBaudrate(std::stoi(info_.hardware_parameters["baudrate"])));

            options.c_cflag |= (CLOCAL | CREAD);                // Enable the receiver and set local mode
            options.c_cflag &= ~PARENB;                         // No parity
            options.c_cflag &= ~CSTOPB;                         // 1 stop bit
            options.c_cflag &= ~CSIZE;                          // Mask the character size bits
            options.c_cflag |= CS8;                             // 8 data bits
            options.c_cflag &= ~CRTSCTS;                        // Disable hardware flow control
            options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input

            options.c_iflag &= ~(IXON | IXOFF | IXANY);  // 禁用软件流控
            options.c_iflag &= ~(INLCR | ICRNL | IGNCR); // 禁用回车和换行处理
            options.c_oflag &= ~OPOST;                   // Raw output

            tcsetattr(serial_fd_, TCSANOW, &options);

            // clear buffer and error
            tcflush(serial_fd_, TCIFLUSH);
            tcflush(serial_fd_, TCOFLUSH);
            tcflush(serial_fd_, TCIOFLUSH);

            

            for (size_t i = 0; i < 3; i++)
            {
                WitReadReg(AX, 3); // 读取加速度Ax
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                // 读取数据
                int len = serialRead(serial_fd_, serial_buffer_, SERIAL_BUFFER_SIZE);
                if (len > 0)
                    for (size_t i = 0; i < len; i++)
                        WitSerialDataIn(serial_buffer_[i]);
                    

                if (new_data_)
                {
                    RCLCPP_INFO(get_logger(), "IMU Initialized");
                    serial_thread_running_ = true;
                    break;
                }
                else
                {
                    RCLCPP_ERROR(get_logger(), "IMU Not Responding in init procedure, retrying in 1s");
                    serial_thread_running_ = false;
                }
            }

            if (!serial_thread_running_)
            {
                RCLCPP_ERROR(get_logger(), "IMU Not Responding, Closing Serial Port");
                close(serial_fd_);
                return;
            }

            int timeout = 0;

            while (rclcpp::ok())
            {
                if (serial_thread_running_ == false)
                {
                    RCLCPP_ERROR(get_logger(), "IMU Thread should stop");
                    close(serial_fd_);
                    return;
                }

                // 读取数据
                int len = serialRead(serial_fd_, serial_buffer_, SERIAL_BUFFER_SIZE);
                if (len > 0)
                    for (size_t i = 0; i < len; i++)
                        WitSerialDataIn(serial_buffer_[i]);

                if (new_data_ | 0 == 0)
                {
                    timeout++;
                    if (timeout > 1000)
                    {
                        RCLCPP_ERROR(get_logger(), "IMU Timeout, Closing Serial Port");
                        close(serial_fd_);
                        serial_thread_running_ = false;
                        timeout = 0;
                        break;
                    }
                }

                if (new_data_ & ACC_UPDATE)
                {
                    for (size_t i = 0; i < 3; i++)
                        imu_data_raw_linear_acceleration_[i] = (float)sReg[AX + i] / 32768.0f * 16.0f;
                    new_data_ &= ~ACC_UPDATE;

                    timeout = 0;
                }

                if (new_data_ & GYRO_UPDATE)
                {
                    for (size_t i = 0; i < 3; i++)
                        imu_data_raw_angular_velocity_[i] = (float)sReg[GX + i] / 32768.0f * 2000.0f;
                    new_data_ &= ~GYRO_UPDATE;

                    timeout = 0;
                }

                if (new_data_ & ORIENTATION_UPDATE)
                {
                    for (size_t i = 0; i < 4; i++)

                        imu_data_raw_orientation_[i] = (float)sReg[q0 + i] / 32768.0f;
                    new_data_ &= ~ORIENTATION_UPDATE;

                    timeout = 0;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

    hardware_interface::CallbackReturn DartFlySystemHardwareIMU::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        this->serial_thread_ = std::thread(&DartFlySystemHardwareIMU::serialDaemonThread, this);
        int count = 0;
        while (!serial_thread_running_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (count++ > 10)
            {
                RCLCPP_ERROR(get_logger(), "Serial Thread Start Failed!");
                serial_thread_.join();
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DartFlySystemHardwareIMU::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        serial_thread_running_ = false;
        serial_thread_.join();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type DartFlySystemHardwareIMU::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "IMU Data: Acc: %f %f %f, Gyro: %f %f %f, Ori: %f %f %f %f",
                             imu_data_raw_linear_acceleration_[0], imu_data_raw_linear_acceleration_[1], imu_data_raw_linear_acceleration_[2],
                             imu_data_raw_angular_velocity_[0], imu_data_raw_angular_velocity_[1], imu_data_raw_angular_velocity_[2],
                             imu_data_raw_orientation_[0], imu_data_raw_orientation_[1], imu_data_raw_orientation_[2], imu_data_raw_orientation_[3]);
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> DartFlySystemHardwareIMU::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        RCLCPP_INFO(get_logger(), "Export IMU State Interface for %s", info_.sensors[0].name.c_str());

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name,
            "linear_acceleration.x",
            &imu_data_raw_linear_acceleration_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name,
            "linear_acceleration.y",
            &imu_data_raw_linear_acceleration_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name,
            "linear_acceleration.z",
            &imu_data_raw_linear_acceleration_[2]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name,
            "angular_velocity.x",
            &imu_data_raw_angular_velocity_[0]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name,
            "angular_velocity.y",
            &imu_data_raw_angular_velocity_[1]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name,
            "angular_velocity.z",
            &imu_data_raw_angular_velocity_[2]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name,
            "orientation.x",
            &imu_data_raw_orientation_[0]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name,
            "orientation.y",
            &imu_data_raw_orientation_[1]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name,
            "orientation.z",
            &imu_data_raw_orientation_[2]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name,
            "orientation.w",
            &imu_data_raw_orientation_[3]));

        return state_interfaces;
    }

} // namespace dart_flysystem_hardware

// Export to pluginlib
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    dart_flysystem_hardware::DartFlySystemHardwareIMU, hardware_interface::SensorInterface)