//
// Created by cheny on 24-9-18.
//

#include "state_machine.h"
#include "openfsm.h"
#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sound_effect.h"
#include "buzzer_examples.h"
#include "motor_controller.h"
#include "micro_switch.h"
#include "dart_config.h"
#include "servo.h"
#include "dbus.h"
#include "judge_receive.h"

namespace state_machine {

    bool isRemoteOnline() {
        return (HAL_GetTick() - RC_Data.last_update_time) < 1000;
    }

    void setNextStateByRemote(bool enterProtectIfDisconnected = true) {
        // 通过遥控器设置状态机状态
        E_Dart_State next_state = E_Dart_State::Protect;
        if (isRemoteOnline()) {
            if (RC_Data.Switch_Right == RC_SW_UP) {
                next_state = E_Dart_State::Protect;
            } else if (RC_Data.Switch_Right == RC_SW_DOWN) {
                next_state = E_Dart_State::Match;
            } else if (RC_Data.Switch_Right == RC_SW_MID) {
                next_state = E_Dart_State::Remote;
            }
        } else if (enterProtectIfDisconnected) {
            next_state = E_Dart_State::Protect;
        }

        if (next_state != dart_fsm.openFSM_.focusEState()) {
            dart_fsm.openFSM_.nextState(next_state);
        }
    }

#define enterProtectModeIfMotorDisconnected() \
    do { \
        if (motor::MotorYawLS.motor_state_ == motor::E_MotorState::DISCONNECTED || \
            motor::MotorLoad[0].motor_state_ == motor::E_MotorState::DISCONNECTED || \
            motor::MotorLoad[1].motor_state_ == motor::E_MotorState::DISCONNECTED || \
            motor::MotorTriggerLS.motor_state_ == motor::E_MotorState::DISCONNECTED || \
            motor::MotorPitchLS.motor_state_ == motor::E_MotorState::DISCONNECTED) { \
            openFSM_.enterState(E_Dart_State::Protect); \
            return; \
        } \
    } while (0)

#define enableTriggerServotoReload() \
    do{ \
        trigger_servo[0].setAngle(CONFIG_TRIGGER_SERVO_RELOAD_ANGLE_0); \
        trigger_servo[1].setAngle(CONFIG_TRIGGER_SERVO_RELOAD_ANGLE_1); \
        trigger_servo[0].enable(); \
        trigger_servo[1].enable(); \
    }while(0)

#define enableTriggerServotoTrigger() \
    do{ \
        trigger_servo[0].setAngle(CONFIG_TRIGGER_SERVO_TRIGGER_ANGLE_0); \
        trigger_servo[1].setAngle(CONFIG_TRIGGER_SERVO_TRIGGER_ANGLE_1); \
        trigger_servo[0].enable(); \
        trigger_servo[1].enable(); \
    }while(0)

#define setTriggerServotoTrigger() \
    do{ \
        trigger_servo[0].setAngle(CONFIG_TRIGGER_SERVO_TRIGGER_ANGLE_0); \
        trigger_servo[1].setAngle(CONFIG_TRIGGER_SERVO_TRIGGER_ANGLE_1); \
    }while(0)

#define setTriggerServotoReload() \
    do{ \
        trigger_servo[0].setAngle(CONFIG_TRIGGER_SERVO_RELOAD_ANGLE_0); \
        trigger_servo[1].setAngle(CONFIG_TRIGGER_SERVO_RELOAD_ANGLE_1); \
    }while(0)

#define disableTriggerServo() \
    do{ \
        trigger_servo[0].disable(); \
        trigger_servo[1].disable(); \
    }while(0)

#define disableLaser() HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET)
#define enableLaser() HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET)

    void FSM::update() {
        // 状态机更新
        openFSM_.update();
        micro_switch_read();

        // 遥控看门狗
        static TickType_t last_reset_tick = xTaskGetTickCount();
        if (xTaskGetTickCount() - RC_Data.last_update_time > pdMS_TO_TICKS(5) &&
            xTaskGetTickCount() - last_reset_tick > pdMS_TO_TICKS(200)) {
            DT7_Reset();
            last_reset_tick = xTaskGetTickCount();
        }

        static TickType_t last_reset_tick_judge_judge = xTaskGetTickCount();
        if (xTaskGetTickCount() - ext_judge_last_receive_time > pdMS_TO_TICKS(5) &&
            xTaskGetTickCount() - last_reset_tick_judge_judge > pdMS_TO_TICKS(200)) {
            judge_Reset();
            last_reset_tick_judge_judge = xTaskGetTickCount();
        }
    }

    Dart_FSM dart_fsm;

    class ActionWaitForAllMotorOnline : public OpenFSMAction {
    public:
        void enter(OpenFSM &fsm) const override {
        }

        void update(OpenFSM &fsm) const override {
//            电机上电后默认状态为IDLE
            if (motor::MotorYawLS.motor_state_ != motor::E_MotorState::DISCONNECTED &&
                motor::MotorLoad[0].motor_state_ != motor::E_MotorState::DISCONNECTED &&
                motor::MotorLoad[1].motor_state_ != motor::E_MotorState::DISCONNECTED &&
                motor::MotorTriggerLS.motor_state_ != motor::E_MotorState::DISCONNECTED &&
                motor::MotorPitchLS.motor_state_ != motor::E_MotorState::DISCONNECTED) {
                soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_startup));
                fsm.nextAction();
            }
        }

        void exit(OpenFSM &fsm) const override {
        }
    };

    /**
     * @brief 非阻塞电机移动直到限位开关被触发，返回值表示是否执行完成
     * @tparam T
     * @param fsm 状态机对象
     * @param pid PID控制器对象
     * @param triggerFlag 限位开关状态
     * @param target_reset_velocity 目标速度
     * @return
     */
    template<typename T>
    E_ResetActionReturnState
    actionResetLSUntilTrigger(motor_controller::pid_angle_velocity_controller<T> &pid,
                              E_Lead_Screw_Switch_State &triggerFlag,
                              int16_t target_reset_velocity) {
        if (pid.motor_->motor_state_ == motor::E_MotorState::DISCONNECTED) {
            return E_ResetActionReturnState::Failed;
        } else if (pid.motor_->motor_state_ == motor::E_MotorState::RUNNING) {
            if (triggerFlag == E_Lead_Screw_Switch_State::Untriggered) {
                pid.target_velocity_ = target_reset_velocity;
                pid.set_state(motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                return E_ResetActionReturnState::Operating;
            } else if (triggerFlag == E_Lead_Screw_Switch_State::Triggered) {
                pid.motor_->setNextState(motor::E_MotorState::IDLE);
                pid.motor_->resetRound();
                return E_ResetActionReturnState::Finished;
            }
        } else if (triggerFlag == E_Lead_Screw_Switch_State::Untriggered) {
            pid.target_velocity_ = target_reset_velocity;
            pid.set_state(motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
            return E_ResetActionReturnState::Operating;
        }
        return E_ResetActionReturnState::Operating;
    }

    /**
     * @brief 非阻塞电机移动到底限位，返回值表示是否执行完成
     * @tparam TypeTarget
     * @tparam TypeGate
     * @tparam TypeController
     * @param controller_ PID控制器对象
     * @param operation_target_ 目标速度
     * @param gate_velocity_ 限位速度
     * @param timeout_ 超时时间
     * @param running_flag_ 运行标志
     * @param openloop_ 是否开环
     * @return 是否执行完成
     */
    template<typename TypeTarget, typename TypeGate, typename TypeController>
    E_ResetActionReturnState
    actionResetMotorUntilBlocked(motor_controller::pid_angle_velocity_controller<TypeController> &controller_,
                                 TypeTarget operation_target_, TypeGate gate_velocity_,
                                 TickType_t timeout_, bool &running_flag_,
                                 bool openloop_ = false) {

        static TickType_t last_time;
        if (!running_flag_) { // 开始运行
            last_time = xTaskGetTickCount();
            if (openloop_) {
                controller_.
                        set_state(motor_controller::E_PID_Velocity_Angle_Controller_State::OPEN_LOOP);
                controller_.
                        target_openloop_ = operation_target_;
            } else {
                controller_.
                        set_state(motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                controller_.
                        target_velocity_ = operation_target_;
            }
            running_flag_ = true;
            return E_ResetActionReturnState::Operating; // 未完成
        } else { // 运行中
            if (abs(controller_.motor_->current_velocity_) <= abs(gate_velocity_)) {
                if (xTaskGetTickCount() - last_time > timeout_) {
                    // 停止电机，复原状态
                    if (openloop_)
                        controller_.target_openloop_ = 0;
                    else
                        controller_.target_velocity_ = 0;
                    controller_.motor_->setNextState(motor::E_MotorState::IDLE);
                    running_flag_ = false;
                    controller_.motor_->resetRound();
                    return E_ResetActionReturnState::Finished; // 完成
                }
            } else {
                controller_.
                        target_velocity_ = operation_target_;
                last_time = xTaskGetTickCount();
            }
            return E_ResetActionReturnState::Operating; // 未完成
        }
    }


    /**
     * @brief 上电归零操作，将YAW轴丝杆和PITCH轴丝杆移动到限位开关位置，将双扳机舵机移动到初始位置，将装填电机移动到初始位置
     */
    class ActionResetMotors : public OpenFSMAction {
        // Dart_FSM Flags使用
        // 0 for MotorYawLS Reset Success
        // 1 for MotorPitchLS Reset Success
        // 2 for MotorTriggerLS Reset Success
    public:

        void enter(OpenFSM &fsm) const override {
            motor::MotorLoad[0].setNextState(motor::E_MotorState::IDLE);
            motor::MotorLoad[1].setNextState(motor::E_MotorState::IDLE);
            motor::MotorYawLS.setNextState(motor::E_MotorState::RUNNING);
            motor::MotorPitchLS.setNextState(motor::E_MotorState::RUNNING);
            motor::MotorTriggerLS.setNextState(motor::E_MotorState::RUNNING);
            fsm.custom<Dart_FSM>()->ActionResetMotors_Load_0_Success = false;
            fsm.custom<Dart_FSM>()->ActionResetMotors_Load_1_Success = false;
            enableTriggerServotoReload();
        }

        void update(OpenFSM &fsm) const override {
//            使用与逻辑保证所有函数执行完成
            if (actionResetLSUntilTrigger<>(motor_controller::MotorYawLSController,
                                            yaw_switch_state,
                                            CONFIG_TARGET_RESET_VELOCITY_YAWLS) ==
                E_ResetActionReturnState::Finished &&
                actionResetLSUntilTrigger<>(motor_controller::MotorPitchLSController,
                                            pitch_switch_state,
                                            CONFIG_TARGET_RESET_VELOCITY_PITCHLS) ==
                E_ResetActionReturnState::Finished &&
                actionResetLSUntilTrigger<>(motor_controller::MotorTriggerLSController,
                                            trigger_switch_state,
                                            CONFIG_TARGET_RESET_VELOCITY_TRIGGERLS) ==
                E_ResetActionReturnState::Finished &&
                actionResetMotorUntilBlocked<>(motor_controller::MotorLoadController[0],
                                               CONFIG_TARGET_RESET_VELOCITY_LOAD,
                                               CONFIG_GATE_VELOCITY_LOAD,
                                               pdMS_TO_TICKS(CONFIG_TIMEOUT_RESET_LOAD),
                                               fsm.custom<Dart_FSM>()->ActionResetMotors_Load_0_Success, false) ==
                E_ResetActionReturnState::Finished &&
                actionResetMotorUntilBlocked<>(motor_controller::MotorLoadController[1],
                                               CONFIG_TARGET_RESET_VELOCITY_LOAD,
                                               CONFIG_GATE_VELOCITY_LOAD,
                                               pdMS_TO_TICKS(CONFIG_TIMEOUT_RESET_LOAD),
                                               fsm.custom<Dart_FSM>()->ActionResetMotors_Load_1_Success, false) ==
                E_ResetActionReturnState::Finished) {
                fsm.nextAction();
            }
        }
    };

    class ActionProtect : public OpenFSMAction {
    public:
        void enter(OpenFSM &fsm) const override {
            // 保护状态
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_protect));
            // 关闭激光器
            disableLaser();
        }

        void update(OpenFSM &fsm) const override {
            // 保护状态
            motor::MotorLoad[0].setNextState(motor::E_MotorState::IDLE);
            motor::MotorLoad[1].setNextState(motor::E_MotorState::IDLE);
            motor::MotorYawLS.setNextState(motor::E_MotorState::IDLE);
            motor::MotorPitchLS.setNextState(motor::E_MotorState::IDLE);
            motor::MotorTriggerLS.setNextState(motor::E_MotorState::IDLE);
            disableTriggerServo();
            setNextStateByRemote();
        }
    };

    class ActionRemote : public OpenFSMAction {
    public:
        void enter(OpenFSM &fsm) const override {
            enableTriggerServotoReload();
            enableLaser();
            motor_controller::MotorLoadController[0].set_state(
                    motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);

            motor_controller::MotorLoadController[1].set_state(
                    motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);

            motor_controller::MotorYawLSController.set_state(
                    motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);

            motor_controller::MotorPitchLSController.set_state(
                    motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);

            motor_controller::MotorTriggerLSController.set_state(
                    motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);

            // 重置状态变量
            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 0;

            motor_controller::MotorLoadSyncController.reset();
        }

        void update(OpenFSM &fsm) const override {
            // 遥控状态
            motor::MotorLoad[0].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorLoad[1].setNextState(motor::E_MotorState::RUNNING);
            motor::MotorYawLS.setNextState(motor::E_MotorState::RUNNING);
            motor::MotorPitchLS.setNextState(motor::E_MotorState::RUNNING);
            motor::MotorTriggerLS.setNextState(motor::E_MotorState::RUNNING);

            // 响应遥控器指令
            if (RC_Data.Switch_Left == RC_SW_UP || RC_Data.Switch_Left == RC_SW_MID) {
                // 扳机锁定在初始位置，不可触发操作，可以操作Yaw、Pitch、Load电机和扳机丝杆
                // Yaw轴控制
                // <--- 700 --- 900 --- 中点 --- 1100 --- 1310 --->
                // <+
                // 100    +10                      -10      -100>
                // 如果有速度，则转为速度控制模式，否则转为位置控制模式
                if (RC_Data.ch0 > 900 && RC_Data.ch0 < 1100) {
                    if (motor_controller::MotorYawLSController.state_ !=
                        motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL) {
                        motor_controller::MotorYawLSController.set_state(
                                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
                        motor_controller::MotorYawLSController.target_angle_with_rounds_ =
                                motor::MotorYawLS.current_round_ * 8192 + motor::MotorYawLS.current_angle_;
                        dart_launcher_params.primary_yaw = motor_controller::MotorYawLSController.target_angle_with_rounds_;
                    }
                } else {
                    if (motor_controller::MotorYawLSController.state_ !=
                        motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL)
                        motor_controller::MotorYawLSController.set_state(
                                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                    if (RC_Data.ch0 <= 700)
                        motor_controller::MotorYawLSController.target_velocity_ = -200;
                    else if (RC_Data.ch0 > 700 && RC_Data.ch0 <= 900)
                        motor_controller::MotorYawLSController.target_velocity_ = -10;
                    else if (RC_Data.ch0 >= 1100 && RC_Data.ch0 < 1310)
                        motor_controller::MotorYawLSController.target_velocity_ = 10;
                    else if (RC_Data.ch0 >= 1310)
                        motor_controller::MotorYawLSController.target_velocity_ = 200;
                }

                // Pitch轴控制
                // <--- 700 --- 900 --- 中点 --- 1100 --- 1310 --->
                // 如果有速度，则转为速度控制模式，否则转为位置控制模式
                if (RC_Data.ch1 > 900 && RC_Data.ch1 < 1100) {
                    if (motor_controller::MotorPitchLSController.state_ !=
                        motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL) {
                        motor_controller::MotorPitchLSController.set_state(
                                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
                        motor_controller::MotorPitchLSController.target_angle_with_rounds_ =
                                motor::MotorPitchLS.current_round_ * 8192 + motor::MotorPitchLS.current_angle_;
                        dart_launcher_params.primary_pitch = motor_controller::MotorPitchLSController.target_angle_with_rounds_;
                    }
                } else {
                    if (motor_controller::MotorPitchLSController.state_ !=
                        motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL)
                        motor_controller::MotorPitchLSController.set_state(
                                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                    if (RC_Data.ch1 <= 700)
                        motor_controller::MotorPitchLSController.target_velocity_ = -200;
                    else if (RC_Data.ch1 > 700 && RC_Data.ch1 <= 900)
                        motor_controller::MotorPitchLSController.target_velocity_ = -10;
                    else if (RC_Data.ch1 >= 1100 && RC_Data.ch1 < 1310)
                        motor_controller::MotorPitchLSController.target_velocity_ = 10;
                    else if (RC_Data.ch1 >= 1310)
                        motor_controller::MotorPitchLSController.target_velocity_ = 200;
                }

                // Trigger扳机丝杆控制
                // <--- 700 --- 900 --- 中点 --- 1100 --- 1310 --->
                // 如果有速度，则转为速度控制模式，否则转为位置控制模式
                if (RC_Data.ch2 > 900 && RC_Data.ch2 < 1100) {
                    if (motor_controller::MotorTriggerLSController.state_ !=
                        motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL) {
                        motor_controller::MotorTriggerLSController.set_state(
                                motor_controller::E_PID_Velocity_Angle_Controller_State::ANGLE_CONTROL);
                        motor_controller::MotorTriggerLSController.target_angle_with_rounds_ =
                                motor::MotorTriggerLS.current_round_ * 8192 + motor::MotorTriggerLS.current_angle_;
                        dart_launcher_params.primary_force = motor_controller::MotorTriggerLSController.target_angle_with_rounds_;
                    }
                } else {
                    if (motor_controller::MotorTriggerLSController.state_ !=
                        motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL)
                        motor_controller::MotorTriggerLSController.set_state(
                                motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                    if (RC_Data.ch2 <= 700)
                        motor_controller::MotorTriggerLSController.target_velocity_ = -200;
                    else if (RC_Data.ch2 > 700 && RC_Data.ch2 <= 900)
                        motor_controller::MotorTriggerLSController.target_velocity_ = -10;
                    else if (RC_Data.ch2 >= 1100 && RC_Data.ch2 < 1310)
                        motor_controller::MotorTriggerLSController.target_velocity_ = 10;
                    else if (RC_Data.ch2 >= 1310)
                        motor_controller::MotorTriggerLSController.target_velocity_ = 200;
                }

                // Load电机控制
                // < --- 950 --- 中点 --- 1400 --- >
                // Load导轨状态机
                // 0. Lock状态：Load电机不动，均角度闭环在当前位置
                // 1. Operating to Reload状态：Load电机运动到装填位置
                // 2. Operating to Launch状态：Load电机运动到发射位置
                // 3. Operating状态：遥控器控制Load电机运动
                int16_t base_velocity = 0;
                switch (fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State) {
                    case 0:
                        base_velocity = 0;

                        // 状态转移
                        if (RC_Data.ch3 >= 1600) {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 1;
                        } else if ((RC_Data.ch3 > 1400 && RC_Data.ch3 < 1600) |
                                   (RC_Data.ch3 >= 366 && RC_Data.ch3 < 950)) {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 3;
                        } else if (RC_Data.ch3 < 366) {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 2;
                        }
                        break;
                    case 1:
                        base_velocity = CONFIG_MOTOR_LOAD_OPERATION_VELOCITY;
                        // 状态转移
                        if (RC_Data.ch3 >= 1600 |
                            (motor_controller::MotorLoadController[0].current_angle_with_rounds_ >=
                             CONFIG_MOTOR_LOAD_ANGLE_RELOAD |
                             motor_controller::MotorLoadController[1].current_angle_with_rounds_ >=
                             CONFIG_MOTOR_LOAD_ANGLE_RELOAD)) {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 0;
                        }
                        break;

                    case 2:
                        base_velocity = -CONFIG_MOTOR_LOAD_OPERATION_VELOCITY;

                        // 状态转移
                        if (RC_Data.ch3 < 950 | (motor_controller::MotorLoadController[0].current_angle_with_rounds_ <=
                                                 CONFIG_MOTOR_LOAD_ANGLE_LAUNCH |
                                                 motor_controller::MotorLoadController[1].current_angle_with_rounds_ <=
                                                 CONFIG_MOTOR_LOAD_ANGLE_LAUNCH)) {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 0;
                        }
                        break;

                    case 3:
                        if (RC_Data.ch3 <= 950) {
                            base_velocity = CONFIG_MOTOR_LOAD_OPERATION_VELOCITY;
                        } else if (RC_Data.ch3 >= 1400) {
                            base_velocity = -CONFIG_MOTOR_LOAD_OPERATION_VELOCITY;
                        }

                        // 状态转移
                        if (RC_Data.ch3 < 366) {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 2;
                        } else if (RC_Data.ch3 >= 1600) {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 1;
                        } else if (RC_Data.ch3 >= 950 && RC_Data.ch3 < 1400) {
                            fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 0;
                        }

                        break;
                    default:
                        fsm.custom<Dart_FSM>()->ActionRemote_MotorLoad_State = 0;
                }
                motor_controller::MotorLoadController[0].target_velocity_ =
                        base_velocity + motor_controller::MotorLoadSyncController.output;
                motor_controller::MotorLoadController[1].target_velocity_ =
                        -base_velocity + motor_controller::MotorLoadSyncController.output;

            } else if (RC_Data.Switch_Left == RC_SW_DOWN) {
                static bool launch_operating_ = false;
                int16_t base_velocity = 0;
                if (!launch_operating_) { // 解锁扳机，内八触发一次发射，Load电机带动同步带到顶端，扳机解锁
                    bool launch_grant_ = false;
                    if ((RC_Data.ch2 > 1400 && RC_Data.ch0 < 400) ||
                        (RC_Data.ch2 > 1400 && RC_Data.ch0 < 400))
                        launch_grant_ = true;

                    if (launch_grant_) {
                        launch_operating_ = true;
                    }
                } else {

                    static bool launch_complete_ = false;
                    static TickType_t launch_complete_time_;

                    if (!launch_complete_) {
                        base_velocity = -CONFIG_MOTOR_LOAD_OPERATION_VELOCITY;
                        if (abs(motor_controller::MotorLoadController[0].current_angle_with_rounds_ -
                                CONFIG_MOTOR_LOAD_ANGLE_LAUNCH) < 100 &&
                            abs(motor_controller::MotorLoadController[1].current_angle_with_rounds_ -
                                CONFIG_MOTOR_LOAD_ANGLE_LAUNCH) < 100) {
                            launch_complete_ = true;
                            launch_complete_time_ = xTaskGetTickCount();
                            base_velocity = 0;
                            setTriggerServotoTrigger();
                        }
                    } else {
                        if (xTaskGetTickCount() - launch_complete_time_ > pdMS_TO_TICKS(500)) {
                            setTriggerServotoReload();
                            launch_complete_ = false;
                            launch_operating_ = false;
                        }
                    }
                }
                // 设置base_velocity
                motor_controller::MotorLoadController[0].target_velocity_ =
                        base_velocity + motor_controller::MotorLoadSyncController.output;
                motor_controller::MotorLoadController[1].target_velocity_ =
                        -base_velocity + motor_controller::MotorLoadSyncController.output;

                setNextStateByRemote();
            }
        }
    };

    class ActionMatch_Enter : public OpenFSMAction {
        void enter(OpenFSM &fsm) const override {
            // 比赛状态
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_winxp));
        }

        void update(OpenFSM &fsm) const override {
        }

        void exit(OpenFSM &fsm) const override {
        }
    };

    class ActionMatch_Wait : public OpenFSMAction {
        void enter(OpenFSM &fsm) const override {
            // 比赛状态
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_winxp));
        }

        void update(OpenFSM &fsm) const override {
        }

        void exit(OpenFSM &fsm) const override {
        }
    };

    class ActionMatch_Launch : public OpenFSMAction {
        void enter(OpenFSM &fsm) const override {
            // 比赛状态
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_winxp));
        }

        void update(OpenFSM &fsm) const override {
        }

        void exit(OpenFSM &fsm) const override {
        }
    };

    class ActionMatch_Reload : public OpenFSMAction {
        void enter(OpenFSM &fsm) const override {
            // 比赛状态
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_winxp));
        }

        void update(OpenFSM &fsm) const override {
        }

        void exit(OpenFSM &fsm) const override {
        }
    };

    void Dart_FSM::start() {
        OpenFSM::RegisterAction<ActionWaitForAllMotorOnline>("ActionWaitForAllMotorOnline");
        OpenFSM::RegisterAction<ActionResetMotors>("ActionResetMotors");
        OpenFSM::RegisterAction<ActionProtect>("ActionProtect");
        OpenFSM::RegisterAction<ActionRemote>("ActionRemote");
        OpenFSM::RegisterAction<ActionMatch_Enter>("ActionMatch_Enter");
        OpenFSM::RegisterAction<ActionMatch_Wait>("ActionMatch_Wait");
        OpenFSM::RegisterAction<ActionMatch_Launch>("ActionMatch_Launch");
        OpenFSM::RegisterAction<ActionMatch_Reload>("ActionMatch_Reload");

        OpenFSM::RegisterState("StateBoot", {"ActionWaitForAllMotorOnline", "ActionResetMotors"},
                               E_Dart_State::Boot);
        OpenFSM::RegisterState("StateProtect", {"ActionProtect"}, E_Dart_State::Protect);
        OpenFSM::RegisterState("StateRemote", {"ActionRemote"}, E_Dart_State::Remote);
        OpenFSM::RegisterState("StateMatch", {"ActionMatch_Enter", "ActionMatch_Wait", "ActionMatch_Launch",
                                              "ActionMatch_Reload", "ActionMatch_Wait", "ActionMatch_Launch",
                                              "ActionMatch_Reload", "ActionMatch_Wait", "ActionMatch_Launch",
                                              "ActionMatch_Reload"},
                               E_Dart_State::Match);

        OpenFSM::RegisterRelation("StateBoot", {"StateProtect"});
        OpenFSM::RegisterRelation("StateProtect", {"StateRemote", "StateMatch"});
        OpenFSM::RegisterRelation("StateRemote", {"StateProtect", "StateMatch"});
        OpenFSM::RegisterRelation("StateMatch", {"StateProtect", "StateRemote"});

        openFSM_.setCustom(this);
        openFSM_.setStates({E_Dart_State::Boot, E_Dart_State::Protect, E_Dart_State::Remote, E_Dart_State::Match});

        openFSM_.enterState(E_Dart_State::Boot);

        // 启动音效
        soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_dji_startup));
    }

    void fsm_thread(void *parameters) {
        TickType_t last_time;

        dart_fsm.start();

        while (true) {
            dart_fsm.update();
            vTaskDelayUntil(&last_time, pdMS_TO_TICKS(1)); // 1000Hz
        }
        vTaskDelete(nullptr);
    }

} // state_machine