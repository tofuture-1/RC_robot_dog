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

float R = 0.0;
float theta_polar = 0.0;

uint8_t flag_trun = 0;
float fine_FB = 0;
float fine_RL = 0;
float fine_R_hight = 0;
float fine_L_hight = 0;

float fine_R_L_direction = 0.0f;
// float kp = 4.5f,kd = 0.1f,angle = 0.0f,Omega = 0.0f,Torque = 0.00f;//测试用

int8_t jump_set_flag = 0;

void Class_DOG::Chassis_Init()
{




//默认使用MIT模式
    Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,kp,kd,angle,Omega,-Torque);
    Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,kp,kd,angle,Omega,Torque);
    Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,kp,kd,angle,Omega,-Torque);
    Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,kp,kd,angle,Omega,Torque);

    Jump_fsm.Init(4,0);

}



void Class_DOG::FSi6x_control()//直角坐标系
{

    if (fsia6b_msg.ch[5] >= 1751 && flag_jump == 0 && fsia6b_msg.ch[4]==1000)
    {
        X = Math_Int_To_Float(fsia6b_msg.ch[3],1000,2000,-0.06f,+0.06f);
        Y = Math_Int_To_Float(fsia6b_msg.ch[2],1000,2000,-0.060f,-0.19f);
        fine_FB = Math_Int_To_Float(fsia6b_msg.ch[9],1000,2000,+0.02f,-0.02f);
        fine_L_hight = 0.0;
        fine_R_hight = 0.0;//操作逻辑干涉了
    }else if (fsia6b_msg.ch[5] < 1500 && flag_jump == 0&& fsia6b_msg.ch[4]==1000)
    {
        X = Math_Int_To_Float(fsia6b_msg.ch[8],1000,2000,-0.06f,+0.06f);
        Y = Math_Int_To_Float(fsia6b_msg.ch[2],1000,2000,-0.060f,-0.19f);
        fine_FB = Math_Int_To_Float(fsia6b_msg.ch[9],1000,2000,+0.02f,-0.02f);
        fine_L_hight = 0.0;
        fine_R_hight = 0.0;
    }else if (fsia6b_msg.ch[5] >= 1751 && flag_jump == 0 && fsia6b_msg.ch[4]==2000)
    {
        fine_R_hight = Math_Int_To_Float(fsia6b_msg.ch[9],1000,2000,+0.00f,-0.12f);
        fine_L_hight = Math_Int_To_Float(fsia6b_msg.ch[8],1000,2000,+0.00f,-0.12f);
        X = Math_Int_To_Float(fsia6b_msg.ch[3],1000,2000,-0.06f,+0.06f);
        Y = Math_Int_To_Float(fsia6b_msg.ch[2],1000,2000,-0.060f,-0.19f);

    }



    //用于更新数据而不带控制数值更改
    if ( fsia6b_msg.ch[5] > 1751 && flag_jump == 0)
    {
        if ( fsia6b_msg.ch[1] > 1550  && theta <= 2*Pi && theta >= 0 && fsia6b_msg.ch[0] < 1550 && fsia6b_msg.ch[0] > 1450)
        {
            theta += 0.0 + Math_Int_To_Float(fsia6b_msg.ch[1],1000,2000,-0.25,+0.25);

            // Xe1 = X + (0.04 + test1) * cos(theta);
            // Ye1 = Y + (0.03 + test1) * sin(theta);
            //
            // Xe4 = X + (0.04 + test1) * cos(theta - 3*Pi/2 );
            // Ye4 = Y + (0.03 + test1) * sin(theta - 3*Pi/2);
            //
            // Xe2 = X + (0.04 + test1)  * cos(theta - Pi/2);
            // Ye2 = Y + (0.03 + test1) * sin(theta - Pi/2);
            //
            // Xe3 = X + (0.04 + test1)  * cos(theta - Pi);
            // Ye3 = Y + (0.03 + test1) * sin(theta - Pi);
            //特定步幅及速度情况下可以实现右转

            Xe1 = X + (0.04 + fine_FB) * cos(theta);
            Ye1 = Y + (0.03 + fine_FB) * sin(theta);

            Xe4 = X + (0.04 + fine_FB) * cos(theta);
            Ye4 = Y + (0.03 + fine_FB) * sin(theta);

            Xe2 = X + (0.04 + fine_FB)  * cos(theta - Pi);
            Ye2 = Y + (0.03 + fine_FB) * sin(theta - Pi);

            Xe3 = X + (0.04 + fine_FB)  * cos(theta - Pi);
            Ye3 = Y + (0.03 + fine_FB) * sin(theta - Pi);

        }
        else if (fsia6b_msg.ch[1] < 1450 && theta >= -2*Pi && theta <= 0&& fsia6b_msg.ch[0] < 1550 && fsia6b_msg.ch[0] > 1450)
        {
            theta += 0.0 + Math_Int_To_Float(fsia6b_msg.ch[1],1000,2000,-0.20,+0.20);

            Xe1 = X + (0.04 + fine_FB) * cos(theta);
            Ye1 = Y + (0.03 + fine_FB) * sin(theta);

            Xe4 = X + (0.04 + fine_FB) * cos(theta);
            Ye4 = Y + (0.03 + fine_FB) * sin(theta);

            Xe2 = X + (0.04 + fine_FB)  * cos(theta - Pi);
            Ye2 = Y + (0.03 + fine_FB) * sin(theta - Pi);

            Xe3 = X + (0.04 + fine_FB)  * cos(theta - Pi);
            Ye3 = Y + (0.03 + fine_FB) * sin(theta - Pi);
        }
        else if ( fsia6b_msg.ch[0] > 1750  && theta <= 2*Pi && theta >= 0 && fsia6b_msg.ch[1] < 1550 && fsia6b_msg.ch[1] > 1450)
        {
            theta += 0.0 + Math_Int_To_Float(fsia6b_msg.ch[0],1000,2000,-0.20,+0.20);

            Xe1 = X + (0.04 + fine_RL) * cos(theta);
            Ye1 = Y + (0.02 + fine_RL) * sin(theta);
            Xe4 = X + (0.04 + fine_RL) * cos(theta);
            Ye4 = Y + (0.02 + fine_RL) * sin(theta);

            Xe3 = X + (0.01 + fine_RL) * cos(theta - Pi);
            Ye3 = Y + (0.02 * fine_RL) * sin(theta - Pi);
            Xe2 = X + (0.01 + fine_RL) * cos(theta - Pi);
            Ye2 = Y + (0.02 * fine_RL) * sin(theta - Pi);

        }
        else if (fsia6b_msg.ch[0] < 1250 && theta >= -2*Pi && theta <= 0 && fsia6b_msg.ch[1] < 1550 && fsia6b_msg.ch[1] > 1450)
        {
            theta += 0.0 + Math_Int_To_Float(fsia6b_msg.ch[0],1000,2000,-0.20,+0.20);

            Xe1 = X - (0.01 + fine_RL) * cos(theta);
            Ye1 = Y + (0.02 + fine_RL) * sin(theta);
            Xe4 = X - (0.01 + fine_RL) * cos(theta);
            Ye4 = Y + (0.02 + fine_RL) * sin(theta);

            Xe3 = X - (0.04 + fine_RL) * cos(theta - Pi);
            Ye3 = Y + (0.02 + fine_RL) * sin(theta - Pi);
            Xe2 = X - (0.04 + fine_RL) * cos(theta - Pi);
            Ye2 = Y + (0.02 + fine_RL) * sin(theta - Pi);
        }
        else if (fsia6b_msg.ch[1] < 1550 && fsia6b_msg.ch[1] > 1450 && fsia6b_msg.ch[0] < 1550 && fsia6b_msg.ch[0] > 1450 )
        {
            Xe1 = X;
            Ye1 = Y;
            Xe2 = X;
            Ye2 = Y;
            Xe3 = X;
            Ye3 = Y;
            Xe4 = X;
            Ye4 = Y;
            //未推杆时的回位
        }else
        {
            theta = 0;//防止长时间累加导致的溢出，用周期性限制
        }

        // Foot1_Left_Back.Set_x_y(Xe1,Ye1);
        // Foot2_Right_Back.Set_x_y(Xe3,Ye3);
        // Foot3_Left_Front.Set_x_y(Xe3,Ye3);
        // Foot4_Right_Front.Set_x_y(Xe1,Ye1);

        Foot1_Left_Back.Set_x_y(Xe1,Ye1+fine_L_hight);
        Foot2_Right_Back.Set_x_y(Xe2,Ye2+fine_R_hight);
        Foot3_Left_Front.Set_x_y(Xe3,Ye3+fine_L_hight);
        Foot4_Right_Front.Set_x_y(Xe4,Ye4+fine_R_hight);
    }else if (fsia6b_msg.ch[5] < 1500 && flag_jump == 0)
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
        Xe2 = X;
        Ye2 = Y;
        Xe3 = X;
        Ye3 = Y;
        Xe4 = X;
        Ye4 = Y;
        //未推杆时的回位
        // Foot1_Left_Back.Set_x_y(Xe1,Ye1);
        // Foot2_Right_Back.Set_x_y(Xe2,Ye2);
        // Foot3_Left_Front.Set_x_y(Xe3,Ye3);
        // Foot4_Right_Front.Set_x_y(Xe4,Ye4);

    }


        if (fsia6b_msg.ch[6] == 2000)
        {
            Xe1 = X;
            Ye1 = -0.17;
            Xe2 = X;
            Ye2 = -0.17;
            Xe3 = X;
            Ye3 = -0.17;
            Xe4 = X;
            Ye4 = -0.17;
            float Torque1 = 0.00f;
            if (fsia6b_msg.ch[7] == 1000)
            {
                Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,20,0.1,angle,Omega,-Torque1);
                Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,20,0.1,angle,Omega,Torque1);
                Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,20,0.1,angle,Omega,-Torque1);
                Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,20,0.1,angle,Omega,Torque1);
            }else if (fsia6b_msg.ch[7] == 2000)
            {
                Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,90,0.1,angle,Omega,-Torque1);
                Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,90,0.1,angle,Omega,Torque1);
                Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,90,0.1,angle,Omega,-Torque1);
                Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,90,0.1,angle,Omega,Torque1);
            }

                flag_jump = 1;

        }else if (fsia6b_msg.ch[6] == 1500)
        {
            Xe1 = X;
            Ye1 = -0.09;
            Xe2 = X;
            Ye2 = -0.09;
            Xe3 = X;
            Ye3 = -0.09;
            Xe4 = X;
            Ye4 = -0.09;
            // float a = Foot1_Left_Back.mi.Get_Now_Angle();
            // float Torque1 = 0.35f;
            if (fsia6b_msg.ch[7] == 1000)
            {
                Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,2.0,kd,angle,Omega,-Torque);
                Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,2.0,kd,angle,Omega,Torque);
                Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,2.0,kd,angle,Omega,-Torque);
                Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,2.0,kd,angle,Omega,Torque);
            }else if (fsia6b_msg.ch[7] == 2000)
            {
                Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,1.5,kd,angle,Omega,-Torque);
                Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,1.5,kd,angle,Omega,Torque);
                Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,1.5,kd,angle,Omega,-Torque);
                Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,1.5,kd,angle,Omega,Torque);

            }
                flag_jump = 2;

        }else if (fsia6b_msg.ch[6] == 1000 && flag_jump !=0)
        {
            Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,kp,kd,angle,Omega,-Torque);
            Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,kp,kd,angle,Omega,Torque);
            Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,kp,kd,angle,Omega,-Torque);
            Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,kp,kd,angle,Omega,Torque);

            flag_jump = 0;
        }
        //注意保护
        Foot1_Left_Back.Set_x_y(Xe1,Ye1+fine_L_hight);
        Foot2_Right_Back.Set_x_y(Xe2,Ye2+fine_R_hight);
        Foot3_Left_Front.Set_x_y(Xe3,Ye3+fine_L_hight);
        Foot4_Right_Front.Set_x_y(Xe4,Ye4+fine_R_hight);
        // Foot1_Left_Back.Set_x_y(Xe1,Ye1);
        // Foot2_Right_Back.Set_x_y(Xe2,Ye2);
        // Foot3_Left_Front.Set_x_y(Xe3,Ye3);
        // Foot4_Right_Front.Set_x_y(Xe4,Ye4);
        // Foot1_Left_Back.Set_x_y(Xe1,Ye1);
        // Foot2_Right_Back.Set_x_y(Xe3,Ye3);
        // Foot3_Left_Front.Set_x_y(Xe3,Ye3);
        // Foot4_Right_Front.Set_x_y(Xe1,Ye1);
}


void Class_DOG::FSi6x_control_polar()//极坐标系
{
    if (fsia6b_msg.ch[5] >= 1751 && flag_jump != 1 && fsia6b_msg.ch[4]==1000)
    {
        // X = Math_Int_To_Float(fsia6b_msg.ch[3],1000,2000,-0.06f,+0.06f);
        // Y = Math_Int_To_Float(fsia6b_msg.ch[2],1000,2000,-0.060f,-0.19f);
        fine_FB = Math_Int_To_Float(fsia6b_msg.ch[9],1000,2000,+0.02f,-0.02f);
        fine_R_hight = Math_Int_To_Float(fsia6b_msg.ch[8],1000,2000,+0.00f,-0.12f);
        fine_RL = Math_Int_To_Float(fsia6b_msg.ch[9],1000,2000,+0.02f,-0.02f);

        theta_polar = Math_Int_To_Float(fsia6b_msg.ch[3],1000,2000,3.925f,5.495f);
        R = Math_Int_To_Float(fsia6b_msg.ch[2],1000,2000,0.060f,0.19f);

    }else if (fsia6b_msg.ch[5] < 1500 && flag_jump != 1&& fsia6b_msg.ch[4]==1000)
    {
        theta_polar = Math_Int_To_Float(fsia6b_msg.ch[3],1000,2000,3.925f,5.495f);
        R = Math_Int_To_Float(fsia6b_msg.ch[2],1000,2000,0.060f,0.19f);
        //fine_FB = Math_Int_To_Float(fsia6b_msg.ch[9],1000,2000,+0.02f,-0.02f);
        fine_RL = Math_Int_To_Float(fsia6b_msg.ch[9],1000,2000,+0.02f,-0.02f);

        fine_L_hight = 0.0;
        fine_R_hight = 0.0;
    }else if (fsia6b_msg.ch[5] >= 1751 && flag_jump != 1 && fsia6b_msg.ch[4]==2000)
    {
        theta_polar = Math_Int_To_Float(fsia6b_msg.ch[3],1000,2000,3.925f,5.495f);
        R = Math_Int_To_Float(fsia6b_msg.ch[2],1000,2000,0.060f,0.19f);


        fine_FB = Math_Int_To_Float(fsia6b_msg.ch[9],1000,2000,+0.02f,-0.02f);
        fine_L_hight = Math_Int_To_Float(fsia6b_msg.ch[8],1000,2000,+0.00f,-0.12f);

    }



    //极坐标转换成直角坐标
    X = cos(theta_polar) * R;
    Y = sin(theta_polar) * R;

    //用于更新数据而不带控制数值更改
    if ( fsia6b_msg.ch[5] > 1751 && flag_jump == 0)
    {
        if ( fsia6b_msg.ch[1] > 1550  && theta <= 2*Pi && theta >= 0 && fsia6b_msg.ch[0] > 1001 && fsia6b_msg.ch[0] < 1999)
        {
            theta += 0.0 + Math_Int_To_Float(fsia6b_msg.ch[1],1000,2000,-0.030,+0.030);

            //fine_FB = Math_Int_To_Float(fsia6b_msg.ch[1],1500,2000,0.0f,+0.03f);

            fine_R_L_direction = Math_Int_To_Float(fsia6b_msg.ch[0],1000,2000,-0.045,+0.045);

            Xe1 = X + (0.04 + fine_FB + fine_R_L_direction) * cos(theta);
            Ye1 = Y + (0.02 + fine_FB*0.4) * sin(theta);

            Xe4 = X + (0.04 + fine_FB + fine_R_L_direction) * cos(theta);
            Ye4 = Y + (0.02 + fine_FB*0.4) * sin(theta);

            Xe2 = X + (0.04 + fine_FB - fine_R_L_direction)  * cos(theta - Pi);
            Ye2 = Y + (0.02 + fine_FB*0.4) * sin(theta - Pi);

            Xe3 = X + (0.04 + fine_FB - fine_R_L_direction)  * cos(theta - Pi);
            Ye3 = Y + (0.02 + fine_FB*0.4) * sin(theta - Pi);

        }
        else if (fsia6b_msg.ch[1] < 1450 && theta >= -2*Pi && theta <= 0&& fsia6b_msg.ch[0] > 1001 && fsia6b_msg.ch[0] < 1999)
        {
            theta += 0.0 + Math_Int_To_Float(fsia6b_msg.ch[1],1000,2000,-0.030,+0.030);

            //fine_FB = Math_Int_To_Float(fsia6b_msg.ch[1],1000,1500,+0.03f,0.0f);


            fine_R_L_direction = Math_Int_To_Float(fsia6b_msg.ch[0],1000,2000,-0.035,+0.035);

            Xe1 = X + (0.04 + fine_FB + fine_R_L_direction) * cos(theta);
            Ye1 = Y + (0.02 + fine_FB*0.4) * sin(theta);

            Xe4 = X + (0.04 + fine_FB + fine_R_L_direction) * cos(theta);
            Ye4 = Y + (0.02 + fine_FB*0.4) * sin(theta);

            Xe2 = X + (0.04 + fine_FB - fine_R_L_direction)  * cos(theta - Pi);
            Ye2 = Y + (0.02 + fine_FB*0.4) * sin(theta - Pi);

            Xe3 = X + (0.04 + fine_FB - fine_R_L_direction)  * cos(theta - Pi);
            Ye3 = Y + (0.02 + fine_FB*0.4) * sin(theta - Pi);
        }
        else if ( fsia6b_msg.ch[0] > 1650  && theta <= 2*Pi && theta >= 0 )
        {
            theta += 0.0f + Math_Int_To_Float(fsia6b_msg.ch[0],1000,2000,-0.025,+0.025);


            Xe1 = (X + (0.03f + fine_RL) * cos(theta));
            Ye1 = Y + (0.015f + fine_RL*0.4) * sin(theta);
            Xe4 = -(X + (0.03f + fine_RL) * cos(theta));
            Ye4 = Y + (0.015f + fine_RL*0.4) * sin(theta);

            Xe3 = (X + (0.03f + fine_RL) * cos(theta - Pi));
            Ye3 = Y + (0.015f + fine_RL*0.4) * sin(theta - Pi);
            Xe2 = -(X + (0.03f + fine_RL) * cos(theta - Pi));
            Ye2 = Y + (0.015f + fine_RL*0.4) * sin(theta - Pi);

        }
        else if (fsia6b_msg.ch[0] < 1450 && theta >= -2*Pi && theta <= 0 )
        {
            theta += 0.0f + Math_Int_To_Float(fsia6b_msg.ch[0],1000,2000,-0.025,+0.025);

            Xe1 = (X + (0.03f + fine_RL) * cos(theta - Pi));
            Ye1 = Y + (0.015f + fine_RL*0.4) * sin(theta - Pi);
            Xe4 = -(X + (0.03f + fine_RL) * cos(theta - Pi));
            Ye4 = Y + (0.015f + fine_RL*0.4) * sin(theta - Pi);

            Xe3 = (X + (0.03f + fine_RL) * cos(theta));
            Ye3 = Y + (0.015f + fine_RL*0.4) * sin(theta);
            Xe2 = -(X + (0.03f + fine_RL) * cos(theta));
            Ye2 = Y + (0.015f + fine_RL*0.4) * sin(theta);
        }
        else if (fsia6b_msg.ch[1] < 1550 && fsia6b_msg.ch[1] > 1450 && fsia6b_msg.ch[0] < 1550 && fsia6b_msg.ch[0] > 1450 )
        {
            // theta += 0.015;
            //  Xe1 = X;
            //  Ye1 = Y + (0.02 + fine_RL) * sin(theta);
            //  Xe2 = X;
            //  Ye2 = Y + (0.02 + fine_RL) * sin(theta-Pi);
            //  Xe3 = X;
            //  Ye3 = Y + (0.02 + fine_RL) * sin(theta-Pi);
            //  Xe4 = X;
            //  Ye4 = Y + (0.02 + fine_RL) * sin(theta);
            Xe1 = X;
            Ye1 = Y ;
            Xe2 = X;
            Ye2 = Y ;
            Xe3 = X;
            Ye3 = Y ;
            Xe4 = X;
            Ye4 = Y ;
            //未推杆时的回位
        }else
        {
            theta = 0;//防止长时间累加导致的溢出，用周期性限制
        }


        Foot1_Left_Back.Set_x_y(Xe1,Ye1+fine_L_hight);
        Foot2_Right_Back.Set_x_y(Xe2,Ye2+fine_R_hight);
        Foot3_Left_Front.Set_x_y(Xe3,Ye3+fine_L_hight);
        Foot4_Right_Front.Set_x_y(Xe4,Ye4+fine_R_hight);
    }else if (fsia6b_msg.ch[5] < 1500 && flag_jump == 0)
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
        Xe2 = X;
        Ye2 = Y;
        Xe3 = X;
        Ye3 = Y;
        Xe4 = X;
        Ye4 = Y;
        //未推杆时的回位
        // Foot1_Left_Back.Set_x_y(Xe1,Ye1);
        // Foot2_Right_Back.Set_x_y(Xe2,Ye2);
        // Foot3_Left_Front.Set_x_y(Xe3,Ye3);
        // Foot4_Right_Front.Set_x_y(Xe4,Ye4);

    }


        if (fsia6b_msg.ch[6] == 2000)
        {


                flag_jump = 1;

        }else if (fsia6b_msg.ch[6] == 1500)
        {

            // R = 0.075;
            //
            // //极坐标转换成直角坐标
            // X = cos(theta_polar) * R;
            // Y = sin(theta_polar) * R;
            // Xe1 = X;
            // Ye1 = Y;
            // Xe2 = X;
            // Ye2 = Y;
            // Xe3 = X;
            // Ye3 = Y;
            // Xe4 = X;
            // Ye4 = Y;
            //
            // // float Torque1 = 0.35f;
            // if (fsia6b_msg.ch[7] == 1000)
            // {
            //     Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,2.0,kd,angle,Omega,-Torque);
            //     Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,2.0,kd,angle,Omega,Torque);
            //     Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,2.0,kd,angle,Omega,-Torque);
            //     Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,2.0,kd,angle,Omega,Torque);
            // }else if (fsia6b_msg.ch[7] == 2000)
            // {
            //     Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,1.5,kd,angle,Omega,-Torque);
            //     Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,1.5,kd,angle,Omega,Torque);
            //     Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,1.5,kd,angle,Omega,-Torque);
            //     Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,1.5,kd,angle,Omega,Torque);
            //
            // }
            Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,kp,kd,angle,Omega,-Torque);
            Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,kp,kd,angle,Omega,Torque);
            Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,kp,kd,angle,Omega,-Torque);
            Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,kp,kd,angle,Omega,Torque);

                flag_jump = 0;

        }else if (fsia6b_msg.ch[6] == 1000 && flag_jump !=0)
        {
            Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,kp,kd,angle,Omega,-Torque);
            Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,kp,kd,angle,Omega,Torque);
            Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,kp,kd,angle,Omega,-Torque);
            Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,kp,kd,angle,Omega,Torque);

            flag_jump = 0;
        }
        //注意保护





    if (Get_flag_jump_status() == 1)
    {
        Jump_fsm.TIM_Calculate_PeriodElapsedCallback();
        switch(Jump_fsm.Get_Now_Status_Serial())
        {
        case 0://起跳部分
            {
                if (jump_set_flag == 0)
                {
                    // kp = 20;
                    // kd = 0.1;
                    // if (R<=0.17)
                    // {
                    //     R += 0.02;
                    // }
                    R = 0.17;
                    //极坐标转换成直角坐标
                    X = cos(theta_polar) * R;
                    Y = sin(theta_polar) * R;

                    if (fsia6b_msg.ch[1] < 1550 && fsia6b_msg.ch[1] > 1450 && fsia6b_msg.ch[0] < 1550 && fsia6b_msg.ch[0] > 1450 && fsia6b_msg.ch[7] == 1000)
                    {
                        Xe1 = X;
                        Ye1 = Y;
                        Xe2 = X;
                        Ye2 = Y;
                        Xe3 = X;
                        Ye3 = Y;
                        Xe4 = X;
                        Ye4 = Y;

                    }
                    else if (fsia6b_msg.ch[1] < 1550 && fsia6b_msg.ch[1] > 1450 && fsia6b_msg.ch[7] == 2000)
                    {
                        Xe1 = X+0.03f;
                        Ye1 = Y;
                        Xe2 = X-0.03f;
                        Ye2 = Y;
                        Xe3 = X+0.03f;
                        Ye3 = Y;
                        Xe4 = X-0.03f;
                        Ye4 = Y;
                    }
                    float Torque1 = 3.00f;
                    // if (fsia6b_msg.ch[7] == 1000)
                    // {
                        Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,30,0.1,angle,Omega,-Torque1);
                        Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,30,0.1,angle,Omega,Torque1);
                        Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,30,0.1,angle,Omega,-Torque1);
                        Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,30,0.1,angle,Omega,Torque1);
                    //}
                    // else if (fsia6b_msg.ch[7] == 2000)
                    // {
                    //     Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,90,0.1,angle,Omega,-Torque1);
                    //     Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,90,0.1,angle,Omega,Torque1);
                    //     Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,90,0.1,angle,Omega,-Torque1);
                    //     Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,90,0.1,angle,Omega,Torque1);
                    // }
                    jump_set_flag = 1;
                }
                if (Jump_fsm.Status[0].Count_Time>70)
                {
                    Jump_fsm.Set_Status(1);
                    jump_set_flag = 0;
                }
                break;
            }
        case 1://收腿部分
            {
                // kp = 5.5;
                // kd = 0.1;


                if (jump_set_flag == 0)
                {
                    // if (R>=0.06)
                    // {
                    //     R -= 0.01;
                    // }
                    R = 0.06;
                    //极坐标转换成直角坐标
                    X = cos(theta_polar) * R;
                    Y = sin(theta_polar) * R;

                    if (fsia6b_msg.ch[1] < 1550 && fsia6b_msg.ch[1] > 1450 && fsia6b_msg.ch[0] < 1550 && fsia6b_msg.ch[0] > 1450 )
                    {
                        Xe1 = X;
                        Ye1 = Y;
                        Xe2 = X;
                        Ye2 = Y;
                        Xe3 = X;
                        Ye3 = Y;
                        Xe4 = X;
                        Ye4 = Y;

                    }


                    Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,kp,kd,angle,Omega,-Torque);
                    Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,kp,kd,angle,Omega,Torque);
                    Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,kp,kd,angle,Omega,-Torque);
                    Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,kp,kd,angle,Omega,Torque);




                    jump_set_flag = 1;
                }

                 if (Jump_fsm.Status[1].Count_Time>150)
                 {
                     jump_set_flag = 0;
                     Jump_fsm.Set_Status(2);

                 }
                break;
            }
        case 2://落地缓冲
            {
                // kp = 1.5;
                // kd = 0.1;


                if (jump_set_flag == 0)
                {

                    R = 0.085;
                    // theta_polar = 3*Pi/2;
                    //极坐标转换成直角坐标
                    X = -(cos(theta_polar) * R * 0.75);
                    Y = sin(theta_polar) * R;
                    Xe1 = X;
                    Ye1 = Y;
                    Xe2 = X;
                    Ye2 = Y;
                    Xe3 = X;
                    Ye3 = Y;
                    Xe4 = X;
                    Ye4 = Y;
                    // float a = Foot1_Left_Back.mi.Get_Now_Angle();
                    // float Torque1 = 0.35f;
                    if (fsia6b_msg.ch[7] == 1000)
                    {
                        Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,2.0,kd,angle,Omega,-Torque);
                        Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,2.0,kd,angle,Omega,Torque);
                        Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,2.0,kd,angle,Omega,-Torque);
                        Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,2.0,kd,angle,Omega,Torque);
                    }else if (fsia6b_msg.ch[7] == 2000)
                    {
                        Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,1.5,kd,angle,Omega,-Torque);
                        Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,1.5,kd,angle,Omega,Torque);
                        Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,1.5,kd,angle,Omega,-Torque);
                        Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,1.5,kd,angle,Omega,Torque);

                    }


                    jump_set_flag = 1;
                }

                if (Jump_fsm.Status[2].Count_Time>350)
                {
                    //Jump_fsm.Set_Status(3);
                    jump_set_flag = 0;
                }
                break;
            }
            case 3://空状态
                {
                    // kp = 5.5;
                    // kd = 0.1;
                    // //     kp = 4.5;
                    // //     kd = 0.1;
                    // // init_flag = 0;
                    // //
                    // // if (init_flag == 0)
                    // // {
                    // //     Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,kp,kd,angle,Omega,-Torque);
                    // //     Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,kp,kd,angle,Omega,Torque);
                    // //     Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,kp,kd,angle,Omega,-Torque);
                    // //     Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,kp,kd,angle,Omega,Torque);
                    // //
                    // //
                    // //
                    // //     init_flag = 1;
                    // // }
                    // if (jump_set_flag == 0)
                    // {
                    //
                    //     Foot1_Left_Back.Set_x_y(-0.05,-0.08);
                    //     Foot2_Right_Back.Set_x_y(-0.05,-0.08);
                    //     Foot3_Left_Front.Set_x_y(-0.05,-0.08);
                    //     Foot4_Right_Front.Set_x_y(-0.05,-0.08);
                    //
                    //     Foot1_Left_Back.SingleFoot_Init(&hcan1,0x11,0x01,&hcan1,0x12,0x02,Left,kp,kd,angle,Omega,-Torque);
                    //     Foot2_Right_Back.SingleFoot_Init(&hcan1,0x14,0x04,&hcan1,0x13,0x03,Right,kp,kd,angle,Omega,Torque);
                    //     Foot3_Left_Front.SingleFoot_Init(&hcan2,0x18,0x08,&hcan2,0x17,0x07,Left,kp,kd,angle,Omega,-Torque);
                    //     Foot4_Right_Front.SingleFoot_Init(&hcan2,0x15,0x05,&hcan2,0x16,0x06,Right,kp,kd,angle,Omega,Torque);
                    //
                    //
                    //
                    //     jump_set_flag = 1;
                    // }
                    // if (Jump_fsm.Status[3].Count_Time>550)
                    // {
                         Jump_fsm.Set_Status(0);
                    //     jump_set_flag = 0;
                    // }
                    break;
                }
        default:
            // 所有case都不匹配时执行
            break;
        }
    }else if ((Get_flag_jump_status() != 1) && (Jump_fsm.Get_Now_Status_Serial() != 0) )
    {
        Jump_fsm.Set_Status(0);
        jump_set_flag = 0;
    }




        Foot1_Left_Back.Set_x_y(Xe1,Ye1+fine_L_hight);
        Foot2_Right_Back.Set_x_y(Xe2,Ye2+fine_R_hight);
        Foot3_Left_Front.Set_x_y(Xe3,Ye3+fine_L_hight);
        Foot4_Right_Front.Set_x_y(Xe4,Ye4+fine_R_hight);
        // Foot1_Left_Back.Set_x_y(Xe1,Ye1);
        // Foot2_Right_Back.Set_x_y(Xe2,Ye2);
        // Foot3_Left_Front.Set_x_y(Xe3,Ye3);
        // Foot4_Right_Front.Set_x_y(Xe4,Ye4);
        // Foot1_Left_Back.Set_x_y(Xe1,Ye1);
        // Foot2_Right_Back.Set_x_y(Xe3,Ye3);
        // Foot3_Left_Front.Set_x_y(Xe3,Ye3);
        // Foot4_Right_Front.Set_x_y(Xe1,Ye1);
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
void Class_DOG::Chassis_Control()
{


    //FSi6x_control();
    FSi6x_control_polar();
    Foot1_Left_Back.Foot_control();

    Foot2_Right_Back.Foot_control();

    Foot3_Left_Front.Foot_control();

    Foot4_Right_Front.Foot_control();



}

void Class_DOG::Chassis_fs_ia6b_caculate()
{
    // ibus_parse(fsia6b_msg,)
    //
    // Math_Int_To_Float();


}

void Class_DOG::Handle_RC_Data(uint8_t *data)
{
    ibus_parse(&fsia6b_msg, data);

}


