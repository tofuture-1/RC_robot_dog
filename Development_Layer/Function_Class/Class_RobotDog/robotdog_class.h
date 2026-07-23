//
// Created by 24158 on 2026/6/15.
//

#ifndef RC_ROBOTDOG_REBUILD20260605_ROBOTDOG_CLASS_H
#define RC_ROBOTDOG_REBUILD20260605_ROBOTDOG_CLASS_H
#include "foot_control.h"
#include "fs_ia6b.h"
#include "fsm_class.h"

extern volatile bool robot_1ms_cycle_flag;
extern volatile bool robot_100ms_cycle_flag;
extern volatile bool robot_1000ms_cycle_flag ;

class Class_DOG{
public:
    void Chassis_Init();

    void FSi6x_control();
    void FSi6x_control_polar();

    void Chassis_Control();

    void Chassis_fs_ia6b_caculate();

    void Handle_RC_Data(uint8_t *data);



    inline uint8_t Get_flag_jump_status();

    inline float Get_kp();
    inline float Get_kd();
    inline float Get_angle();
    inline float Get_Omega();
    inline float Get_Torque();
    Class_SingleDogFoot_DM Foot1_Left_Back;
    Class_SingleDogFoot_DM Foot2_Right_Back;
    Class_SingleDogFoot_DM Foot3_Left_Front;
    Class_SingleDogFoot_DM Foot4_Right_Front;

    Class_FSM Jump_fsm;

protected:
    ibus_msg fsia6b_msg;

    uint8_t flag_jump = 0;

    float kp = 5.5f,kd = 0.1f,angle = 0.0f,Omega = 0.0f,Torque = 0.00f;//测试用
};

uint8_t Class_DOG::Get_flag_jump_status(){return flag_jump;}
float Class_DOG::Get_kp(){return kp;}
float Class_DOG::Get_kd(){return kd;}
float Class_DOG::Get_angle(){return angle;}
float Class_DOG::Get_Omega(){return Omega;}
float Class_DOG::Get_Torque(){return Torque;}
//有无必要？

#endif //RC_ROBOTDOG_REBUILD20260605_ROBOTDOG_CLASS_H
