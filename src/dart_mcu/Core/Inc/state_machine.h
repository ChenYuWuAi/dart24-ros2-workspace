//
// Created by cheny on 24-9-18.
//

#ifndef DART_MCU_DART_STATEMACHINE_H
#define DART_MCU_DART_STATEMACHINE_H

#include "openfsm.h"

using namespace openfsm;

namespace state_machine {

    [[noreturn]] void fsm_thread(void *parameters);

    // 状态机:
    // Update method: Broadcast
    // 0. 上电状态 复位 Action: Reset
    // 1. 保护状态 遥控器右打上
    // 2. 调试模式 遥控器右打中
    // 3. 比赛模式 Action = Wait, Launch, Reload, Reset 遥控器右打下
    enum E_Dart_State {
        Boot = 100,
        Protect = 101,
        Remote = 102,
        Match = 103
    };

    enum E_Match_Actions {
        Enter,
        Wait,
        Launch,
        Reload,
        Undefined
    };

    enum E_ResetActionReturnState {
        Operating,
        Finished,
        Failed
    };

    struct FSM {
        OpenFSM openFSM_;

        virtual void start() = 0;

        void update();
    };

    struct Dart_FSM : public FSM {
        bool boot_success = false;

        // ActionResetMotors
        bool ActionResetMotors_Load_0_Success = false;
        bool ActionResetMotors_Load_1_Success = false;

        // ActionRemote
        uint8_t ActionRemote_MotorLoad_State = 0;
    public:
        void start();
    };

    extern Dart_FSM dart_fsm;

} // state_machine

#endif //DART_MCU_DART_STATEMACHINE_H
