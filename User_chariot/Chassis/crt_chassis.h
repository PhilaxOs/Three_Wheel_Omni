#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

#include "dvc_motor_dji.h"
#include "alg_slope.h"

#define SPEED_SLOPE

// 轮组方向
//   [1]
// [2] [3]
// 前x左y，逆时针为正方向

/**
 * @brief 底盘冲刺状态枚举
 *
 */
enum Enum_Sprint_Status : uint8_t
{
    Sprint_Status_DISABLE = 0,
    Sprint_Status_ENABLE,
};
enum Enum_Yaw_Encoder_Control_Status : uint8_t
{
    Yaw_Encoder_Control_Status_Disable = 0,
    Yaw_Encoder_Control_Status_Enable,
};
/**
 * @brief 底盘逻辑方向枚举
 *
 */
enum Enum_Chassis_Logics_Direction : uint8_t
{
    Chassis_Logic_Direction_Positive = 0,
    Chassis_Logic_Direction_Negative,
};
/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type : uint8_t
{
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_FLLOW,
    Chassis_Control_Type_SPIN_Positive,
    Chassis_Control_Type_SPIN_Negative,
    Chassis_Control_Type_Lift,
};

class Class_Omni_Chassis
{
public:
    // 斜坡函数加减速速度X
    Class_Slope Slope_Velocity_X;
    // 斜坡函数加减速速度Y
    Class_Slope Slope_Velocity_Y;
    // 斜坡函数加减速角速度
    Class_Slope Slope_Omega;

    // 底盘速度值PID
    Class_PID PID_Velocity_X;

    // 底盘速度方向PID
    Class_PID PID_Velocity_Y;

    // 底盘角速度PID
    Class_PID PID_Omega;

    // 下方转动电机
    Class_Motor_DJI_C620 Motor_Wheel[3];

    void Init(float __Velocity_X_Max = 3.0f, float __Velocity_Y_Max = 2.0f, float __Omega_Max = 6.0f);

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
    inline float Get_Velocity_X_Max();
    inline float Get_Velocity_Y_Max();
    inline float Get_Omega_Max();

    inline float Get_Now_Velocity_X();
    inline float Get_Now_Velocity_Y();
    inline float Get_Now_Omega();

    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Now_Velocity_X(float __Now_Velocity_X);
    inline void Set_Now_Velocity_Y(float __Now_Velocity_Y);
    inline void Set_Now_Omega(float __Now_Omega);

    inline void Set_Velocity_Y_Max(float __Velocity_Y_Max);
    inline void Set_Velocity_X_Max(float __Velocity_X_Max);

    void TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status);
    void TIM_100ms_Alive_PeriodElapsedCallback();

protected:
    // 速度X限制
    float Velocity_X_Max;
    // 速度Y限制
    float Velocity_Y_Max;
    // 角速度限制
    float Omega_Max;

    // 电机理论上最大输出
    float Wheel_Max_Output = 16384.0f;

    // 转动电机目标值
    float Target_Wheel_Omega[3] = {0.0f};

    // 转动电机电流目标值
    float Target_Wheel_Current[3] = {0.0f};

    // 底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;

    // 目标速度X
    float Target_Velocity_X = 0.0f;
    // 目标速度Y
    float Target_Velocity_Y = 0.0f;
    // 目标角速度
    float Target_Omega = 0.0f;
    // 当前速度X
    float Now_Velocity_X = 0.0f;
    // 当前速度Y
    float Now_Velocity_Y = 0.0f;
    // 当前角速度
    float Now_Omega = 0.0f;

    // 运动学逆解算
    void Kinematics_Inverse_Resolution();

    // 自身解算
    void Self_Resolution();

    // 输出到底盘动力学
    void Output_To_Dynamics();

    // 动力学逆解算
    void Dynamics_Inverse_Resolution();

    void Output_To_Motor();


    // 电机动摩擦阻力电流值
    float Dynamic_Resistance_Wheel_Current[3] = {0.0f,
                                                 0.0f,
                                                 0.0f};
                                                 
    // 电机摩擦阻力连续化的角速度阈值
    float Wheel_Resistance_Omega_Threshold = 0.0f;
    // 防单轮超速系数
    float Wheel_Speed_Limit_Factor = 0.0f;

    // 低电流前馈控制相关参数
    // 低电流死区设置
    float Low_Current_Deadzone = 0.0f;
    float Low_Current_Threshold = 0.0f;        // 低电流阈值
    float Low_Current_Feedforward[3] = {0.0f}; // 低电流前馈值
};

const float WHEEL_TO_CORE_DISTANCE = 0.320f;

// 轮子直径 单位m 待定
const float WHEEL_DIAMETER = 0.1f;

// 线速度转角速度 rad/s
const float VEL2RAD = 1.0f / (WHEEL_DIAMETER / 2.0f);

// 线速度转角速度 单位m/s
const float RAD2VEl = WHEEL_DIAMETER / 2.0f;

const float SQRT3 = 1.7320508;

const int8_t DIRECTION_SIGN = -1; // 方向调整，正负号，视电机安装方式而定

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
Enum_Chassis_Control_Type Class_Omni_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取速度X限制
 *
 * @return float 速度X限制
 */
float Class_Omni_Chassis::Get_Velocity_X_Max()
{
    return (Velocity_X_Max);
}

/**
 * @brief 获取速度Y限制
 *
 * @return float 速度Y限制
 */
float Class_Omni_Chassis::Get_Velocity_Y_Max()
{
    return (Velocity_Y_Max);
}

/**
 * @brief 获取角速度限制
 *
 * @return float 角速度限制
 */
float Class_Omni_Chassis::Get_Omega_Max()
{
    return (Omega_Max);
}

float Class_Omni_Chassis::Get_Now_Velocity_X()
{
    return (Now_Velocity_X);
}

float Class_Omni_Chassis::Get_Now_Velocity_Y()
{
    return (Now_Velocity_Y);
}

float Class_Omni_Chassis::Get_Now_Omega()
{
    return (Now_Omega);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
float Class_Omni_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
float Class_Omni_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标角速度
 *
 * @return float 目标角速度
 */
float Class_Omni_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
void Class_Omni_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
void Class_Omni_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
void Class_Omni_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
void Class_Omni_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定当前速度X
 *
 * @param __Now_Velocity_X 当前速度X
 */
void Class_Omni_Chassis::Set_Now_Velocity_X(float __Now_Velocity_X)
{
    Now_Velocity_X = __Now_Velocity_X;
}

/**
 * @brief 设定当前速度Y
 *
 * @param __Now_Velocity_Y 当前速度Y
 */
void Class_Omni_Chassis::Set_Now_Velocity_Y(float __Now_Velocity_Y)
{
    Now_Velocity_Y = __Now_Velocity_Y;
}

/**
 * @brief 设定当前角速度
 *
 * @param __Now_Omega 当前角速度
 */
void Class_Omni_Chassis::Set_Now_Omega(float __Now_Omega)
{
    Now_Omega = __Now_Omega;
}

/**
 * @brief 设定当前最大X速度
 *
 * @param __Velocity_Y_Max 输入
 */
void Class_Omni_Chassis::Set_Velocity_Y_Max(float __Velocity_Y_Max)
{
    Velocity_Y_Max = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大Y速度
 *
 * @param __Velocity_X_Max 输入
 */
void Class_Omni_Chassis::Set_Velocity_X_Max(float __Velocity_X_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
}

#endif // CRT_CHASSIS_H
