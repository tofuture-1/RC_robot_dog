//
// Created by 24158 on 2026/6/15.
//

#include "robotdog_class.h"

#include "callback_task.h"

volatile bool robot_1ms_cycle_flag = false;
volatile bool robot_100ms_cycle_flag = false;
volatile bool robot_1000ms_cycle_flag = false;

float theta = 0;
float theta1 = 0;
float theta2 = 0;
float Xe1;
float Ye1;

float Xe2;
float Ye2;

float Xe3;
float Ye3;

float Xe4;
float Ye4;

float X = 0;
float Y = -0.10;
uint8_t flag_jump = 0;
uint8_t flag_trun = 0;
float kp = 4.5f,kd = 0.1f,angle = 0.0f,Omega = 0.0f,Torque = 0.00f;//测试用

void CLASS_DOG::Chassis_Init()
{

    // Foot1_Left_Back.mo.Set_K_P(7);
    // Foot1_Left_Back.mo.Set_K_D(0.1);
    //
    // Foot1_Left_Back.mi.Set_K_P(7);
    // Foot2_Right_Back.mo.Set_K_P(7);
    // Foot2_Right_Back.mi.Set_K_P(7);
    // Foot3_Left_Front.mo.Set_K_P(7);
    // Foot3_Left_Front.mi.Set_K_P(7);
    // Foot4_Right_Front.mo.Set_K_P(7);
    // Foot4_Right_Front.mi.Set_K_P(7);



//默认使用MIT模式
    Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,kp,kd,angle,Omega,Torque);
    Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,kp,kd,angle,Omega,Torque);
    Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,kp,kd,angle,Omega,Torque);
    Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,kp,kd,angle,Omega,Torque);

}




/**
 * \brief Controls the chassis of the robot dog.
 *
 * This method is responsible for managing the movement and positioning
 * of the robot dog's chassis. It coordinates with the individual foot
 * control mechanisms to ensure smooth and stable motion.
 *
 * \note The specific implementation details, such as the algorithms used
 * for stability and movement, are handled within this function.
 */
void CLASS_DOG::Chassis_Control()
{


    Foot1_Left_Back.Foot_control();

    Foot2_Right_Back.Foot_control();

    Foot3_Left_Front.Foot_control();

    Foot4_Right_Front.Foot_control();

}

void CLASS_DOG::Chassis_fs_ia6b_caculate()
{
    // ibus_parse(fsia6b_msg,)
    //
    // Math_Int_To_Float();


}

void CLASS_DOG::Handle_RC_Data(uint8_t *data)
{
    ibus_parse(&fsia6b_msg, data);
    X = Math_Int_To_Float(fsia6b_msg.ch[3],1000,2000,-0.09f,+0.09f);
    Y = Math_Int_To_Float(fsia6b_msg.ch[2],1000,2000,-0.19f,-0.054f);

    if ( fsia6b_msg.ch[5] > 1500)
    {
        if ( fsia6b_msg.ch[1] > 1550  && theta <= 2*Pi && theta >= 0 && fsia6b_msg.ch[0] < 1550 && fsia6b_msg.ch[0] > 1450)
        {
            theta += 0.0 + Math_Int_To_Float(fsia6b_msg.ch[1],1000,2000,-0.25,+0.25);

            Xe1 = X + 0.04 * cos(theta);
            Ye1 = Y + 0.03 * sin(theta);

            Xe3 = X + 0.04 * cos(theta - Pi);
            Ye3 = Y + 0.03 * sin(theta - Pi);

        }
        else if (fsia6b_msg.ch[1] < 1450 && theta >= -2*Pi && theta <= 0&& fsia6b_msg.ch[0] < 1550 && fsia6b_msg.ch[0] > 1450)
        {
            theta += 0.0 + Math_Int_To_Float(fsia6b_msg.ch[1],1000,2000,-0.20,+0.20);

            Xe1 = X + 0.04 * cos(theta);
            Ye1 = Y + 0.03 * sin(theta);

            Xe3 = X + 0.04 * cos(theta - Pi);
            Ye3 = Y + 0.03 * sin(theta - Pi);
        }
        else if ( fsia6b_msg.ch[0] > 1750  && theta <= 2*Pi && theta >= 0 && fsia6b_msg.ch[1] < 1550 && fsia6b_msg.ch[1] > 1450)
        {
            theta += 0.0 + Math_Int_To_Float(fsia6b_msg.ch[0],1000,2000,-0.20,+0.20);

            Xe1 = X + 0.04 * cos(theta);
            Ye1 = Y + 0.02 * sin(theta);

            Xe3 = X + 0.01 * cos(theta - Pi);
            Ye3 = Y + 0.02 * sin(theta - Pi);


        }
        else if (fsia6b_msg.ch[0] < 1250 && theta >= -2*Pi && theta <= 0 && fsia6b_msg.ch[1] < 1550 && fsia6b_msg.ch[1] > 1450)
        {
            theta += 0.0 + Math_Int_To_Float(fsia6b_msg.ch[0],1000,2000,-0.20,+0.20);

            Xe1 = X - 0.01 * cos(theta);
            Ye1 = Y + 0.02 * sin(theta);

            Xe3 = X - 0.04 * cos(theta - Pi);
            Ye3 = Y + 0.02 * sin(theta - Pi);
        }
        else if (fsia6b_msg.ch[1] < 1550 && fsia6b_msg.ch[1] > 1450 && fsia6b_msg.ch[0] < 1550 && fsia6b_msg.ch[0] > 1450 )
        {
            Xe1 = X;
            Ye1 = Y;
            Xe3 = X;
            Ye3 = Y;
        }else
        {
            theta = 0;//防止长时间累加导致的溢出，用周期性限制
        }
    }else
    {
        // if (theta > 0)
        // {
        //     theta -= 0.01;
        // }else if (theta < 0)
        // {
        //     theta += 0.01;
        //
        //
        // }
        //
        // Xe1 = X + 0.04 * cos(theta);
        // Ye1 = Y + 0.03 * sin(theta);
        //
        // Xe3 = X + 0.04 * cos(theta - Pi);
        // Ye3 = Y + 0.03 * sin(theta - Pi);
        Xe1 = X;
        Ye1 = Y;
        Xe3 = X;
        Ye3 = Y;
    }

    if (fsia6b_msg.ch[6] == 2000)
    {
        Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,0,0,0,0,-8);
        Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,0,0,0,0,8);
        Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,0,0,0,0,-8);
        Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,0,0,0,0,8);
        flag_jump = 1;

    }else if (fsia6b_msg.ch[6] == 1500)
    {
        Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,1.5,kd,angle,Omega,Torque);
        Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,1.5,kd,angle,Omega,Torque);
        Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,1.5,kd,angle,Omega,Torque);
        Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,1.5,kd,angle,Omega,Torque);
        flag_jump = 2;

    }else if (fsia6b_msg.ch[6] == 1000 && flag_jump !=0)
    {
        Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,kp,kd,angle,Omega,Torque);
        Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,kp,kd,angle,Omega,Torque);
        Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,kp,kd,angle,Omega,Torque);
        Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,kp,kd,angle,Omega,Torque);

        flag_jump = 0;
    }
    //注意保护

    Foot1_Left_Back.Set_x_y(Xe1,Ye1);
    Foot2_Right_Back.Set_x_y(Xe3,Ye3);
    Foot3_Left_Front.Set_x_y(Xe3,Ye3);
    Foot4_Right_Front.Set_x_y(Xe1,Ye1);

}