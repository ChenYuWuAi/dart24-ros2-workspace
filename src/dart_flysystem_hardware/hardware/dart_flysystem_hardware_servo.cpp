/**
 * @file dart_flysystem_hardware_servo.cpp
 * @brief 飞镖飞行系统硬件控制库
 */

#include "dart_flysystem_hardware/dart_flysystem_hardware_servo.hpp"
#include "rclcpp/rclcpp.hpp"

// PWM硬件控制库
#include "dart_flysystem_hardware/linux_pwm.hpp"

#include <vector>

namespace dart_flysystem_hardware
{
    // PWM->Servo角度转换
#define PWM_SERVO_PWM_PERIOD_NS 20000000.0     // 20ms
#define PWM_SERVO_MIN_NS 500000.0              // 0.5ms
#define PWM_SERVO_MAX_NS 2500000.0             // 2.5ms
#define SERVO_MAX_ANGLE 3.14159265358979323846 // Pi

    static bool setAngleToPwm(const double angle, const std::shared_ptr<linuxPWM::LinuxPwm> &pwm)
    {
        // 转化弧度到PWM脉冲宽度
        return pwm->setDutyCycle(
            PWM_SERVO_MIN_NS + (PWM_SERVO_MAX_NS - PWM_SERVO_MIN_NS) / SERVO_MAX_ANGLE * angle);
    }

    hardware_interface::CallbackReturn
    DartFlySystemHardwareServo::on_init(const hardware_interface::HardwareInfo &info)
    {
        // 初始化超类
        if (
            SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 设置offset角度
        if (info_.hardware_parameters.find("offset_angle") != info_.hardware_parameters.end())
        {
            offset_angle_ = std::stod(info_.hardware_parameters.at("offset_angle"));
        }
        else
        {
            offset_angle_ = 0;
        }

        RCLCPP_INFO(
            get_logger(),
            "Offset angle is %f", offset_angle_);

        // Debug开关读取
        if (const auto debug = info_.hardware_parameters.find("debug"); debug != info_.hardware_parameters.end())
        {
            RCLCPP_INFO(
                get_logger(),
                "Debug mode is %s", debug->second.c_str());
            debug_ = debug->second == "true";
        }
        else
        {
            debug_ = false;
        }

        // 验证配置
        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // 关节必须有一个命令接口
            if (joint.command_interfaces.size() != 1 || joint.state_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    get_logger(),
                    "Joint '%s' have %zu command interfaces and %zu state interfaces found. 1 expected.",
                    joint.name.c_str(), joint.command_interfaces.size(), joint.state_interfaces.size());

                return hardware_interface::CallbackReturn::ERROR;
            }
            // 命令接口必须是唯一角度
            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    get_logger(),
                    "Joint '%s' have %s command interfaces found. '%s' expected.",
                    joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            // 检查joint.parameters，加载PWM Chip和Channel
            try
            {
                const auto pwm_chip = std::stoi(joint.parameters.at("pwm_chip"));
                const auto pwm_channel = std::stoi(joint.parameters.at("pwm_channel"));
                pwm_map_[joint.name] = std::make_shared<linuxPWM::LinuxPwm>();
                // export PWM接口，即调用LinuxPwm::begin()
                if (!pwm_map_[joint.name]->begin(pwm_chip, pwm_channel))
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Failed to export PWM for joint '%s'",
                        joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
                RCLCPP_INFO(
                    get_logger(),
                    "Joint '%s' initialized, which has PWM chip: %d, channel: %d",
                    joint.name.c_str(), pwm_chip, pwm_channel);
            }
            catch (const std::out_of_range &e)
            {
                RCLCPP_FATAL(
                    get_logger(),
                    "%s, Joint '%s' has invalid PWM chip or channel.", e.what(), joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            // angle_map_初始化
            angle_map_[joint.name] = 0;

            // min_angle_map_初始化
            min_angle_map_[joint.name] = joint.parameters.find("min") != joint.parameters.end()
                                             ? std::stof(joint.parameters.at("min"))
                                             : 0;
            // max_angle_map_初始化
            max_angle_map_[joint.name] = joint.parameters.find("max") != joint.parameters.end()
                                             ? std::stof(joint.parameters.at("max"))
                                             : SERVO_MAX_ANGLE;
                                             
            RCLCPP_INFO(
                get_logger(),
                "Joint '%s' initialized, min: %f, max: %f",
                joint.name.c_str(), min_angle_map_[joint.name], max_angle_map_[joint.name]);
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DartFlySystemHardwareServo::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (const auto &joint : info_.joints)
        {
            state_interfaces.emplace_back(
                joint.name, hardware_interface::HW_IF_POSITION, &angle_map_[joint.name]);
            RCLCPP_INFO(
                get_logger(),
                "Joint '%s' state interface %s exported", joint.name.c_str(), joint.command_interfaces[0].name.c_str());
        }
        // 打印state_interfaces的内容
        for (auto &state_interface : state_interfaces)
        {
            RCLCPP_INFO(
                get_logger(),
                "state_interface name: %s, interface_name: %s, interface_value: %f",
                state_interface.get_name().c_str(),
                state_interface.get_interface_name().c_str(),
                state_interface.get_value());
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> DartFlySystemHardwareServo::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (const auto &joint : info_.joints)
        {
            command_interfaces.emplace_back(
                    joint.name, hardware_interface::HW_IF_POSITION, &angle_map_[joint.name]);
            RCLCPP_INFO(
                get_logger(),
                "Joint '%s' command interface %s exported", joint.name.c_str(), joint.command_interfaces[0].name.c_str());
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn DartFlySystemHardwareServo::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {

        for (auto &joint : info_.joints)
        {
            RCLCPP_INFO(
                get_logger(),
                "Configuring joint '%s'", joint.name.c_str());
            // 设置周期
            if (!pwm_map_[joint.name]->setPeriod(PWM_SERVO_PWM_PERIOD_NS))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to set period for joint '%s'",
                    joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            // 设置初始角度
            angle_map_[joint.name] = joint.parameters.find("initial_angle") != joint.parameters.end()
                                         ? std::stoi(joint.parameters.at("initial_angle"))
                                         : 0;
            // 限制角度
            angle_map_[joint.name] = std::max(min_angle_map_[joint.name], std::min(angle_map_[joint.name], max_angle_map_[joint.name]));

            // set_command(joint.name, angle_map_[joint.name]);
            if (!setAngleToPwm(angle_map_[joint.name] + offset_angle_, pwm_map_[joint.name]))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to set initial angle %f for joint '%s'", angle_map_[joint.name] + offset_angle_, joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DartFlySystemHardwareServo::on_activate(
        const rclcpp_lifecycle::State &previous_state)
    {
        for (auto &joint : info_.joints)
        {
            // 启用PWM
            if (!pwm_map_[joint.name]->enable())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to enable PWM for joint '%s'",
                    joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type DartFlySystemHardwareServo::read(const rclcpp::Time &time,
                                                                     const rclcpp::Duration &period)
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DartFlySystemHardwareServo::write(const rclcpp::Time &time,
                                                                      const rclcpp::Duration &period)
    {
        static rclcpp::Time time_throttle;
        for (auto &joint : info_.joints)
        {
            // 限制角度
            angle_map_[joint.name] = std::max(min_angle_map_[joint.name], std::min(angle_map_[joint.name], max_angle_map_[joint.name]));
            // 设置PWM
            double angle = angle_map_[joint.name] + offset_angle_;
            angle = std::max(0.0, std::min(angle, SERVO_MAX_ANGLE));
            if (debug_)
            {
                // Throttle debug info
                if (get_clock()->now().seconds() - time_throttle.seconds() > 1)
                {
                    RCLCPP_INFO(
                        get_logger(),
                        "Setting angle %f for joint '%s'", angle, joint.name.c_str());
                }
            }
            if (!setAngleToPwm(angle, pwm_map_[joint.name]))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to set angle %f for joint '%s'", angle_map_[joint.name], joint.name.c_str());
                return hardware_interface::return_type::ERROR;
            }
        }
        if (debug_)
        {
            // Throttle debug info
            if (get_clock()->now().seconds() - time_throttle.seconds() > 1)
            {
                time_throttle = get_clock()->now();
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn DartFlySystemHardwareServo::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        // 依次关闭所有PWM Lib
        pwm_map_.clear();
        angle_map_.clear();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

} // namespace dart_flysystem_hardware

// Export to pluginlib
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    dart_flysystem_hardware::DartFlySystemHardwareServo, hardware_interface::SystemInterface)
