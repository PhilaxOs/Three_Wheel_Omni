#ifndef ITA_ROBOT_H
#define ITA_ROBOT_H

#include "crt_chassis.h"
#include "dvc_dr16.h"

/**
 * @brief DR16控制数据来源
 *
 */
enum Enum_DR16_Control_Type
{
    DR16_Control_Type_REMOTE = 0,
    DR16_Control_Type_KEYBOARD,

    DR16_Control_Type_NONE,
};

enum Enum_Control_Source
{
    DR16_Control,
    Control_DISABLE,
};

// 添加活动控制器枚举类型
enum Enum_Active_Controller
{
    Controller_NONE = 0,
    Controller_DR16,
};

class Class_Chariot
{
public:
    Class_DR16 DR16;
    Class_Omni_Chassis Chassis;

    void Init(float __Dead_Zone = 0);
    void TIM_Control_Callback();
    void TIM_Calculate_PeriodElapsedCallback();
    void TIM_Unline_Protect_PeriodElapsedCallback();
    void TIM_100ms_Alive_PeriodElapsedCallback();

    void Judge_DR16_Control_Type();
    void Judge_Active_Controller();

    void Control_Chassis();

protected:
    // DR16控制数据来源
    Enum_DR16_Control_Type DR16_Control_Type = DR16_Control_Type_NONE;

    // 当前活动的控制器
    Enum_Active_Controller Active_Controller = Controller_NONE;

    Enum_Control_Source Control_Source = Control_DISABLE;

    // 遥控器拨动的死区, 0~1
    float Dead_Zone;
};
#endif