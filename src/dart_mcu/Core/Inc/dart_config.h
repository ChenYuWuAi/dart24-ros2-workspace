//
// Created by cheny on 24-9-18.
//

#ifndef DART_MCU_DART_CONFIG_H
#define DART_MCU_DART_CONFIG_H

#include "stdint.h"

// 丝杆电机复位堵转速度
#define CONFIG_TARGET_RESET_VELOCITY_YAWLS 1000
#define CONFIG_TARGET_RESET_VELOCITY_PITCHLS 1000
#define CONFIG_TARGET_RESET_VELOCITY_TRIGGERLS 1000

// 该角度直接触发击发
#define CONFIG_TRIGGER_SERVO_TRIGGER_ANGLE_0 90
#define CONFIG_TRIGGER_SERVO_TRIGGER_ANGLE_1 90
// 该角度为重新装填的默认角度
#define CONFIG_TRIGGER_SERVO_RELOAD_ANGLE_0 0
#define CONFIG_TRIGGER_SERVO_RELOAD_ANGLE_1 0

// 复位堵转速度和超时时间
#define CONFIG_TARGET_RESET_VELOCITY_LOAD 1000
#define CONFIG_GATE_VELOCITY_LOAD 100
#define CONFIG_TIMEOUT_RESET_LOAD 1000

// 电机角度限制
#define CONFIG_MOTOR_PITCH_ANGLE_MAX 8192
#define CONFIG_MOTOR_YAW_ANGLE_MAX 8192
#define CONFIG_MOTOR_TRIGGER_ANGLE_MAX 8192
#define CONFIG_MOTOR_LOAD_ANGLE_MAX 8192

// 特殊电机角度定义
#define CONFIG_MOTOR_LOAD_ANGLE_LAUNCH 8192
#define CONFIG_MOTOR_LOAD_ANGLE_RELOAD 1000

// 运动速度定义
#define CONFIG_MOTOR_LOAD_OPERATION_VELOCITY 100


// 飞镖架对外暴露控制变量
/*
classDiagram
    class DartLauncherParams{
        +int32 primary_yaw, 发射目标主偏航角
        +int32 primary_yaw_offset, 发射目标主偏航角偏移量, 调试用
        +int32 primary_pitch, 发射目标主俯仰角
        +int32 primary_force, 发射扳机主位置
        +int32 primary_force_offset, 发射扳机主位置偏移量, 调试用
        +int32[4] auxiliary_yaw_offsets, 发射目标副偏航角偏移量, 按程序发射时使用
        +int32[4] auxiliary_force_offsets, 发射扳机副位置偏移量, 按程序发射时使用
        +uint64 last_param_update_time, 上次参数更新时间, 用于同步, 单位ms 高位优先
    }
 */
typedef struct DartLauncherParams {
    int32_t primary_yaw;
    int32_t primary_yaw_offset;
    int32_t primary_pitch;
    int32_t primary_force;
    int32_t primary_force_offset;
    int32_t auxiliary_yaw_offsets[4];
    int32_t auxiliary_force_offsets[4];
    uint64_t last_param_update_time;
} DartLauncherParams;

/*
 * 拉簧发射架状态回传数据如下：
```mermaid
classDiagram
    class DartLauncherStatus{
        +bool motor_pitch_online, 电机Pitch在线
        +bool motor_yaw_online, 电机Yaw在线
        +bool[2] motor_loader_online, 电机Load[0, 1]在线
        +bool motor_trigger_online, 电机Trigger在线
        +bool judge_online, 裁判系统在线
        +bool rc_online, 遥控器在线
        +uint8 dart_state, 发射架状态, 100 Boot, 101 Protect, 102 Remote, 103-106 Match Enter Wait Launch Reload, 255 Undefined
        +uint8 dart_launch_process, 发射流程(1/4)
        +int32 motor_pitch_angle, 电机Pitch角度
        +int32 motor_yaw_angle, 电机Yaw总角度
        +int32 motor_trigger_angle, 电机Trigger总角度
        +bool servo_trigger_state, 伺服Trigger状态, 0初始, 1触发
        +int32[2] motor_loader_current, 电机Load[0, 1]相电流
        +int32[2] motor_loader_angle, 电机Load[0, 1]总角度
        +double last_launch_speed, 上次发射检测速度
        +int32 last_launch_time, 上次发射检测数据时间UNIX戳，若无时间戳/speed则为0
    }
```
 */
typedef struct DartLauncherStatus {
    bool motor_pitch_online;
    bool motor_yaw_online;
    bool motor_loader_online[2];
    bool motor_trigger_online;
    bool judge_online;
    bool rc_online;
    uint8_t dart_state;
    uint8_t dart_launch_process;
    int32_t motor_pitch_angle;
    int32_t motor_yaw_angle;
    int32_t motor_trigger_angle;
    bool servo_trigger_state;
    int32_t motor_loader_current[2];
    int32_t motor_loader_angle[2];
    double last_launch_speed;
    int32_t last_launch_time;
} DartLauncherStatus;

extern DartLauncherParams dart_launcher_params;
extern DartLauncherStatus dart_launcher_status;

#endif //DART_MCU_DART_CONFIG_H
