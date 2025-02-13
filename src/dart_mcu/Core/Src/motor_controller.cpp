//
// Created by cheny on 24-9-11.
//

#include "motor_controller.h"
#include "cstring"

#define  update_controller_current(motor_, motor_controller_) \
if (motor_.motor_state_ == motor::RUNNING) { \
motor_.setCurrent(motor_controller_.update()); \
} else { \
motor_controller_.reset(); \
}

namespace motor_controller {
    pid_angle_velocity_controller<double> MotorTriggerLSController(
            pid_controller<double>(0.1, 0.01, 0.01, 100, 100, 100, 1000),
            pid_controller<double>(0.1, 0.01, 0.01, 100, 100, 100, 1000),
            &motor::MotorTriggerLS,
            VELOCITY_CONTROL
    );

    pid_angle_velocity_controller<double> MotorYawLSController(
            pid_controller<double>(0.1, 0.01, 0.01, 100, 100, 100, 1000),
            pid_controller<double>(0.1, 0.01, 0.01, 100, 100, 100, 1000),
            &motor::MotorYawLS,
            VELOCITY_CONTROL
    );

    pid_angle_velocity_controller<double> MotorPitchLSController(
            pid_controller<double>(0.1, 0.01, 0.01, 100, 100, 100, 1000),
            pid_controller<double>(0.1, 0.01, 0.01, 100, 100, 100, 1000),
            &motor::MotorPitchLS,
            VELOCITY_CONTROL
    );

    pid_angle_velocity_controller<double> MotorLoadController[2] = {
            pid_angle_velocity_controller<double>(
                    pid_controller<double>(0.1, 0.01, 0.01, 100, 100, 100, 1000),
                    pid_controller<double>(0.1, 0.01, 0.01, 100, 100, 100, 1000),
                    &motor::MotorLoad[0],
                    VELOCITY_CONTROL
            ),
            pid_angle_velocity_controller<double>(
                    pid_controller<double>(0.1, 0.01, 0.01, 100, 100, 100, 1000),
                    pid_controller<double>(0.1, 0.01, 0.01, 100, 100, 100, 1000),
                    &motor::MotorLoad[1],
                    VELOCITY_CONTROL
            )
    };

    pid_controller<double> MotorLoadSyncController = pid_controller<double>(15.0, 0.5, 2.0, 1000.0, 500.0, 300.0,
                                                                            300.0);

    template<typename T>
    pid_controller<T>::pid_controller(T kp, T ki, T kd, T sum_error_max, T p_max, T i_max, T output_max)
            :
            kp(kp), ki(ki), kd(kd), sum_error_max(sum_error_max), p_max(p_max), i_max(i_max),
            output_max(output_max) {
        cur_error = 0;
        last_error = 0;
        sum_error = 0;
        output = 0;
    }

    template<typename T>
    T pid_controller<T>::update(T current) {
        last_error = cur_error;
        cur_error = target - current;
        sum_error = LIMIT_MIN_MAX(sum_error + cur_error,
                                  -sum_error_max,
                                  sum_error_max);

        output = LIMIT_MIN_MAX((kp * cur_error), -p_max, p_max) +
                 LIMIT_MIN_MAX((ki * sum_error), -i_max, i_max) +
                 kd * (cur_error - last_error);

        output = LIMIT_MIN_MAX(output, -output_max, output_max);

        return output;
    }

    template<typename T>
    void pid_controller<T>::reset() {
        cur_error = 0;
        last_error = 0;
        sum_error = 0;
        output = 0;
    }

    template<typename T>
    pid_angle_velocity_controller<T>::pid_angle_velocity_controller(pid_controller<T> pid_velocity,
                                                                    pid_controller<T> pid_angle,
                                                                    motor::motor_rm *motor,
                                                                    E_PID_Velocity_Angle_Controller_State state
    )  : pid_velocity_(pid_velocity), pid_angle_(pid_angle), motor_(motor), state_(state), target_velocity_(0) {
        target_angle_with_rounds_ = 0;
        current_angle_with_rounds_ = motor_->current_round_ * 8192 + motor_->current_angle_;
    }

    template<typename T>
    T pid_angle_velocity_controller<T>::update() {
        if (state_ == VELOCITY_CONTROL) {
            current_angle_with_rounds_ = motor_->current_round_ * 8192 + motor_->current_angle_;
            current_velocity_ = motor_->current_velocity_ * 0.6 + current_velocity_ * 0.4;
            pid_velocity_.target = target_velocity_;
            return pid_velocity_.update(current_velocity_);
        } else if (state_ == ANGLE_CONTROL) {
            // 限幅
            LIMIT_MIN_MAX(target_angle_with_rounds_, 0, 370000);
            current_angle_with_rounds_ = motor_->current_round_ * 8192 + motor_->current_angle_;
            current_velocity_ = motor_->current_velocity_ * 0.6 + current_velocity_ * 0.4;
            pid_angle_.target = target_angle_with_rounds_;
            pid_velocity_.target = -pid_angle_.update(current_angle_with_rounds_);
            return pid_velocity_.update(current_velocity_);
        } else {
            return target_openloop_;
        }
    }

    template<typename T>
    void pid_angle_velocity_controller<T>::reset() {
        pid_velocity_.reset();
        pid_angle_.reset();
    }

    [[noreturn]] void pid_control_task(void *pvParameter) {
        // 1KHz
        // Initialize PID Controller
        TickType_t xLastWakeTime = xTaskGetTickCount();
        static uint32_t tx_mailbox;
        // Can Frame
        CAN_TxHeaderTypeDef tx_header;
        tx_header.StdId = 0x200;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = 8;
        tx_header.TransmitGlobalTime = DISABLE;

        uint8_t can_array[8];


        // Create Motors
        motor::MotorTriggerLS.create(10000, motor::E_MotorType::M2006, 1);  // 扳机丝杆电机
        motor::MotorYawLS.create(16384, motor::E_MotorType::GM6020, 2);      // 偏航丝杆电机
        motor::MotorPitchLS.create(16384, motor::E_MotorType::GM6020, 1);    // 俯仰丝杆电机
        motor::MotorLoad[0].create(16384, motor::E_MotorType::M3508, 2);    // 装填电机1
        motor::MotorLoad[1].create(16384, motor::E_MotorType::M3508, 3, true);    // 装填电机2

        // Wait for Motor to Connect
        while (motor::MotorYawLS.motor_state_ == motor::E_MotorState::DISCONNECTED ||
               motor::MotorLoad[0].motor_state_ == motor::E_MotorState::DISCONNECTED ||
               motor::MotorLoad[1].motor_state_ == motor::E_MotorState::DISCONNECTED ||
               motor::MotorTriggerLS.motor_state_ == motor::E_MotorState::DISCONNECTED ||
               motor::MotorPitchLS.motor_state_ == motor::E_MotorState::DISCONNECTED)
            vTaskDelayUntil(&xLastWakeTime, 100);

        while (true) {
            {
                // CAN1
                memset(can_array, 0, 8);
                // Will detect Motor Next State Automatically
                tx_header.StdId = 0x200;
                update_controller_current(motor::MotorTriggerLS, MotorTriggerLSController);
                motor::update_can_array(can_array, 0, motor::MotorTriggerLS.updateCurrent());
                update_controller_current(motor::MotorLoad[0], MotorLoadController[0]);
                motor::update_can_array(can_array, 1, motor::MotorLoad[0].updateCurrent());
                update_controller_current(motor::MotorLoad[1], MotorLoadController[1]);
                motor::update_can_array(can_array, 2, motor::MotorLoad[1].updateCurrent());

                HAL_CAN_AddTxMessage(&hcan1, &tx_header, can_array, &tx_mailbox);
            }

            {
                // CAN2
                // Will detect Motor Next State Automatically
                memset(can_array, 0, 8);
                tx_header.StdId = 0x1fe;
                // Update Controller
                update_controller_current(motor::MotorPitchLS, MotorPitchLSController);
                motor::update_can_array(can_array, 0, motor::MotorPitchLS.updateCurrent());

                update_controller_current(motor::MotorYawLS, MotorYawLSController);
                motor::update_can_array(can_array, 1, motor::MotorYawLS.updateCurrent());

                HAL_CAN_AddTxMessage(&hcan2, &tx_header, can_array, &tx_mailbox);
            }
            {
                // 更新同步控制器
                MotorLoadSyncController.update(motor_controller::MotorLoadController[0].current_angle_with_rounds_ -
                                               motor_controller::MotorLoadController[1].current_angle_with_rounds_);

            }
            vTaskDelayUntil(&xLastWakeTime, 1);
        }
        vTaskDelete(nullptr);
    }
}