/**
 * @file dvc_motor_robstride.h
 * @author 
 * @brief RobStride (小米CyberGear协议) 电机配置与操作
 * @version 0.1
 * @date 2024-03-18
 *
 * @copyright USTC-RoboWalker (c) 2024
 */

#ifndef DVC_MOTOR_ROBSTRIDE_H
#define DVC_MOTOR_ROBSTRIDE_H

/* Includes ------------------------------------------------------------------*/

#include "drv_can.h" // 假设包含FDCAN_HandleTypeDef或CAN_HandleTypeDef
#include "drv_math.h"

/* Exported macros -----------------------------------------------------------*/

// 控制模式定义
#define ROBSTRIDE_MODE_MOTION  0x00 // 运控模式 (MIT)
#define ROBSTRIDE_MODE_POSITION 0x01 // PP位置模式
#define ROBSTRIDE_MODE_SPEED    0x02 // 速度模式
#define ROBSTRIDE_MODE_CURRENT  0x03 // 电流模式
#define ROBSTRIDE_MODE_ZERO 0x04 //零点模式
#define ROBSTRIDE_MODE_CSP_POSITION 0x05 //CSP位置模式



/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机状态枚举
 */
enum Enum_RobStride_Status
{
    RobStride_Status_DISABLE = 0,
    RobStride_Status_ENABLE,
};

/**
 * @brief RobStride 通信类型 (由扩展帧ID的 24~28位 决定)
 */
enum Enum_RobStride_Communication_Type
{
    RobStride_Comm_GetID = 0x00,                    // 获取设备的ID和64位MCU唯一标识符
    RobStride_Comm_MotionControl = 0x01,            // 运控模式用来向主机发送控制指令
    RobStride_Comm_MotorRequest = 0x02,             // 用来向主机反馈电机运行状态
    RobStride_Comm_MotorEnable = 0x03,              // 电机使能运行
    RobStride_Comm_MotorStop = 0x04,                // 电机停止运行
    RobStride_Comm_SetPosZero = 0x06,               // 设置电机机械零位
    RobStride_Comm_SetCanID = 0x07,                 // 更改当前电机CAN_ID
    RobStride_Comm_GetSingleParam = 0x11,           // 读取单个参数
    RobStride_Comm_SetSingleParam = 0x12,           // 设定单个参数
    RobStride_Comm_ErrorFeedback = 0x15,            // 故障反馈帧
    RobStride_Comm_MotorDataSave = 0x16,            // 电机数据保存帧 (驱动版本 >= 0.13.0)
    RobStride_Comm_BaudRateChange = 0x17,           // 电机波特率修改帧，重新上电生效
    RobStride_Comm_ProactiveEscalationSet = 0x18,   // 电机主动上报
    RobStride_Comm_MotorModeSet = 0x19,             // 电机协议修改帧，重新上电生效
};

/**
 * @brief RobStride 物理参数反馈数据 (解析后)
 */
struct Struct_RobStride_Rx_Data
{
    float Now_Angle;      // rad
    float Now_Omega;      // rad/s
    float Now_Torque;     // Nm
    float Now_Temperature;// ℃
    uint16_t Error_Code;
    
    // 多圈逻辑变量
    uint32_t Pre_Encoder;
    int32_t Total_Round;
};

/**
 * @brief CAN模式枚举
 */
enum Enum_CAN_Mode
{
    CAN_Motor_Mode_Private = 0,
    CAN_Motor_Mode_CANOpen = 1,
    CAN_Motor_Mode_MIT = 2,
};
/**
 * @brief RobStride 电机类
 */
class Class_RobStride_Motor
{
public:

    void Init(FDCAN_HandleTypeDef *hfdcan, uint8_t __CAN_ID, uint8_t __Master_ID = 0x00 , Enum_CAN_Mode __CAN_Mode = CAN_Motor_Mode_Private);

    // Getters
    inline float Get_Now_Angle() { return Rx_Data.Now_Angle; }
    inline float Get_Now_Omega() { return Rx_Data.Now_Omega; }
    inline float Get_Now_Torque() { return Rx_Data.Now_Torque; }
    inline float Get_Now_Temperature() { return Rx_Data.Now_Temperature; }
    inline Enum_RobStride_Status Get_Status() { return Motor_Status; }

    // Setters (用于缓存指令值)
    inline void Set_Target_Angle(float __Angle) { Target_Angle = __Angle; }
    inline void Set_Target_Omega(float __Omega) { Target_Omega = __Omega; }
    inline void Set_Target_Torque(float __Torque) { Target_Torque = __Torque; }
    inline void Set_K_P(float __Kp) { K_P = __Kp; }
    inline void Set_K_D(float __Kd) { K_D = __Kd; }

    /* 发送指令函数 */

    // 1. 运控模式 (类似于达妙MIT模式)
    void CAN_Send_Motion_Control_Private(float Angle, float Omega, float Torque, float Kp, float Kd);
    void CAN_Send_Motion_Control_MIT(float Angle, float Omega, float Torque, float Kp, float Kd);
\
    // 2. 基础控制指令
    void CAN_Send_Enable();
    void CAN_Send_MIT_Enable();
    void CAN_Send_Stop(uint8_t clear_error = 0);
    void CAN_Send_Set_Zero();
    void CAN_Send_Get_ID();
    void Set_Motor_Mode_Private(Enum_CAN_Mode __CAN_Mode);
    void CAN_Config_RunMode(uint8_t mode);
    void CAN_Send_Position_Control(float target_pos, float target_vel);
    // 3. 参数读写 (单次触发)
    void CAN_Send_Set_Param(uint16_t Index, float Value);
    void CAN_Send_Get_Param(uint16_t Index);

    // 回调与周期处理
    void CAN_RxCpltCallback(uint32_t ExtId, uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();

protected:
    // 硬件绑定
    Struct_FDCAN_Manage_Object *FDCAN_Manage_Object;
    uint8_t CAN_ID;        // 电机ID
    uint8_t Master_ID;     // 主机ID
    Enum_CAN_Mode CAN_Mode; // CAN协议模式
    // 内部状态
    Enum_RobStride_Status Motor_Status = RobStride_Status_DISABLE;
    Struct_RobStride_Rx_Data Rx_Data;

    // 目标控制量缓存
    float Target_Angle = 0.0f;
    float Target_Omega = 0.0f;
    float Target_Torque = 0.0f;
    float K_P = 0.0f;
    float K_D = 0.0f;

    // 存活监测
    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    // 内部协议工具
    uint32_t Build_ExtID(uint8_t Comm_Type, uint16_t Data = 0);
    void Data_Process_Private(uint32_t ExtId, uint8_t *Data);
    void Data_Process_MIT(uint8_t *Data);
};

#endif
