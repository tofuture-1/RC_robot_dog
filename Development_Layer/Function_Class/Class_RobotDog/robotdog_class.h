//
// Created by 24158 on 2026/6/15.
//

#ifndef RC_ROBOTDOG_REBUILD20260605_ROBOTDOG_CLASS_H
#define RC_ROBOTDOG_REBUILD20260605_ROBOTDOG_CLASS_H
#include "foot_control.h"
#include "fs_ia6b.h"

extern volatile bool robot_1ms_cycle_flag;
extern volatile bool robot_100ms_cycle_flag;
extern volatile bool robot_1000ms_cycle_flag ;

class CLASS_DOG{
public:
    void Chassis_Init();

    void Chassis_Control();

    void Chassis_fs_ia6b_caculate();

    void Handle_RC_Data(uint8_t *data);

    Class_SingleDogFoot_DM Foot1_Left_Back;
    Class_SingleDogFoot_DM Foot2_Right_Back;
    Class_SingleDogFoot_DM Foot3_Left_Front;
    Class_SingleDogFoot_DM Foot4_Right_Front;

protected:
    ibus_msg fsia6b_msg;

};

#endif //RC_ROBOTDOG_REBUILD20260605_ROBOTDOG_CLASS_H
