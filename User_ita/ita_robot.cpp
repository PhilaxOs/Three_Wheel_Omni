#include "ita_robot.h"
/**
 * @file ita_robot.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 人机交互控制逻辑
 * @version 1.1
 * @date 2023-08-29
 * @date 2024-01-17
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
static bool DR16_Has_Remote_Input(Class_DR16 &dr16)
{
    return (dr16.Get_Left_X() != 0.0f ||
            dr16.Get_Left_Y() != 0.0f ||
            dr16.Get_Right_X() != 0.0f ||
            dr16.Get_Right_Y() != 0.0f ||
            dr16.Get_Yaw() != 0.0f);
}

static bool DR16_Has_Keyboard_Input(Class_DR16 &dr16)
{
    return (dr16.Get_Mouse_X() != 0.0f ||
            dr16.Get_Mouse_Y() != 0.0f ||
            dr16.Get_Mouse_Z() != 0.0f ||
            dr16.Get_Keyboard_Key_A() != 0 ||
            dr16.Get_Keyboard_Key_D() != 0 ||
            dr16.Get_Keyboard_Key_W() != 0 ||
            dr16.Get_Keyboard_Key_S() != 0);
}

/**
 * @brief 底盘，云台，发射机构初始化
 *
 */
void Class_Chariot::Init(float __Dead_Zone)
{
    DR16.Init(&huart5);
    Orin.Init(&hfdcan2);

    Chassis.Init();

    Dead_Zone = __Dead_Zone;
}

/**
 * @brief 101ms定时任务
 *
 */
void Class_Chariot::TIM_101ms_Alive_PeriodElapsedCallback()
{
    Orin.TIM_101ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 100ms定时任务
 *
 */
void Class_Chariot::TIM_100ms_Alive_PeriodElapsedCallback()
{
    DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
    Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 50ms定时任务
 *
 */
void Class_Chariot::TIM_Unline_Protect_PeriodElapsedCallback()
{
    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        Chassis.Set_Target_Velocity_X(0);
        Chassis.Set_Target_Velocity_Y(0);
        Chassis.Set_Target_Omega(0);
        return;
    }
    if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP &&
        Orin.Get_Status() == Orin_Status_DISABLE)    
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        Chassis.Set_Target_Velocity_X(0);
        Chassis.Set_Target_Velocity_Y(0);
        Chassis.Set_Target_Omega(0);
        return;
    }    
}

/**
 * @brief 50ms定时任务
 *
 */
void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
    Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status_ENABLE);
}
/**
 * @brief 获取当前活动的控制器
 *
 */
void Class_Chariot::Judge_DR16_Control_Type()
{
    if (DR16_Has_Remote_Input(DR16) == true)
    {
        DR16_Control_Type = DR16_Control_Type_REMOTE;
    }
    else if (DR16_Has_Keyboard_Input(DR16) == true)     
    {
        DR16_Control_Type = DR16_Control_Type_KEYBOARD;
    }
    else
    {
        DR16_Control_Type = DR16_Control_Type_NONE;
    }
}

/**
 * @brief 获取当前活动的控制器
 *
 */
void Class_Chariot::Judge_Active_Controller()
{
    // 检查DR16是否有输入
    Judge_DR16_Control_Type();

    // 判断当前活动的控制器
    if (DR16.Get_Right_Switch() != DR16_Switch_Status_UP && 
        DR16.Get_DR16_Status() == DR16_Status_ENABLE)
    {
        Active_Controller = Controller_DR16;
    }
    else if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP &&
             Orin.Get_Status() == Orin_Status_ENABLE)
    {
        Active_Controller = Controller_Orin;
    }    
    else
    {
        Active_Controller = Controller_NONE;
    }
}

/**
 * @brief 底盘，云台，发射机构控制逻辑
 *
 */
void Class_Chariot::Control_Chassis()
{
    // 遥控器摇杆值
    Judge_Active_Controller();

    /************************************上位机控制逻辑*********************************************/
    if (Active_Controller == Controller_Orin)
    {
        float orin_vx = Orin.Get_Target_Velocity_X();
        float orin_vy = Orin.Get_Target_Velocity_Y();
        float orin_omega = Orin.Get_Target_Omega();

        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        Chassis.Set_Target_Velocity_X(orin_vx);
        Chassis.Set_Target_Velocity_Y(orin_vy);
        Chassis.Set_Target_Omega(orin_omega);

    }
    /************************************遥控器控制逻辑*********************************************/

    else if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        float dr16_l_x, dr16_l_y, dr16_yaw;
        float chassis_velocity_x = 0, chassis_velocity_y = 0;
        static float chassis_omega = 0;
        static float chassis_angle = 0;
        // 排除遥控器死区
        dr16_l_x = (Math_Abs(DR16.Get_Left_Y()) > Dead_Zone) ? DR16.Get_Left_Y() : 0;
        dr16_l_y = (Math_Abs(DR16.Get_Left_X()) > Dead_Zone) ? DR16.Get_Left_X()*(-1.0f) : 0;
        // yaw和xy的死区是否相同存疑
        dr16_yaw = (Math_Abs(DR16.Get_Right_X()) > Dead_Zone) ? DR16.Get_Right_X()*(-1.0f) : 0;
        // 设定矩形到圆形映射进行控制
        chassis_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();
        chassis_omega = dr16_yaw * Chassis.Get_Omega_Max();
        chassis_angle += chassis_omega;
        // 键盘遥控器操作逻辑
        if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP) // 上位机用，遥控器不可打断
        {
//            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
//            Chassis.Set_Target_Velocity_X(chassis_velocity_x);
//            Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
//            Chassis.Set_Target_Omega(chassis_omega);
			Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
        else if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN) // 底盘停转
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
        else if (DR16.Get_Right_Switch() == DR16_Switch_Status_MIDDLE) // 底盘随动，遥控器专用
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            Chassis.Set_Target_Velocity_X(chassis_velocity_x);
            Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
            Chassis.Set_Target_Omega(chassis_omega);
        }
    }
}

void Class_Chariot::TIM_Control_Callback()
{
    Judge_Active_Controller();

    // 底盘，云台，发射机构控制逻辑
    Control_Chassis();
}
