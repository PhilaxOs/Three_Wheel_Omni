/**

* @file dvc_motor_rs.h

* @author hzy by yssickjgd

* @brief 灵足电机配置与操作

* @version 0.1

* @date 2026-3-26 0.1 新增

*

* @copyright NEUQ

*

*/

#ifndef DVC_MOTOR_RS_H
#define DVC_MOTOR_RS_H


/* Includes ------------------------------------------------------------------*/


#include "drv_can.h"

#include "drv_math.h"



/* Exported macros -----------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/

/**

* @brief 灵足电机扩展帧通信帧

*

*/
enum Enum_RS_Communication_Type
{
    RS_Comm_GetID = 0x00,                    // 获取设备的ID和64位MCU唯一标识符
    RS_Comm_MotionControl = 0x01,            // 运控模式用来向主机发送控制指令
    RS_Comm_MotorRequest = 0x02,             // 用来向主机反馈电机运行状态
    RS_Comm_MotorEnable = 0x03,              // 电机使能运行
    RS_Comm_MotorStop = 0x04,                // 电机停止运行
    RS_Comm_SetPosZero = 0x06,               // 设置电机机械零位
    RS_Comm_SetCanID = 0x07,                 // 更改当前电机CAN_ID
    RS_Comm_GetSingleParam = 0x11,           // 读取单个参数
    RS_Comm_SetSingleParam = 0x12,           // 设定单个参数
    RS_Comm_ErrorFeedback = 0x15,            // 故障反馈帧
    RS_Comm_MotorDataSave = 0x16,            // 电机数据保存帧 (驱动版本 >= 0.13.0)
    RS_Comm_BaudRateChange = 0x17,           // 电机波特率修改帧，重新上电生效
    RS_Comm_ProactiveEscalationSet = 0x18,   // 电机主动上报
    RS_Comm_MotorModeSet = 0x19,             // 电机协议修改帧，重新上电生效
};
/**

* @brief 灵足电机错误类型

*

*/
enum Enum_Clear_Error_Type
{
    Enum_RUN_NORMAL_Type = 0x00, // 正常运行
    Enum_CLEAR_ERROR_Type = 0x01, // 清除错误
};

/**

* @brief 灵足电机状态

*

*/

enum Enum_Motor_RS_Status

{

    Motor_RS_Status_DISABLE = 0,

    Motor_RS_Status_ENABLE,

};

/**

* @brief 灵足电机控制方式

*

*/

enum Enum_Motor_RS_CAN_Protocol

{
    Motor_RS_Control_Method_Private = 0,

    Motor_RS_Control_Method_CANOPEN,

    Motor_RS_Control_Method_MIT,



};

/**

* @brief 灵足电机控制方式

*

*/

enum Enum_Motor_RS_Control_Method

{
    Motor_RS_Control_Method_NORMAL = 0,

    Motor_RS_Control_Method_CSP,

    Motor_RS_Control_Method_OMEGA,

};
/**
 * @brief 灵足电机MIT模式源数据
 */
struct Struct_Motor_RS_CAN_Rx_Data_MIT
{ 
    uint8_t CAN_ID;

    uint16_t Angle_Reverse;

    uint8_t Omega_11_4;

    uint8_t Omega_3_0_Torque_11_8;

    uint8_t Torque_7_0;

    uint16_t Temp_Reverse;

}__attribute__((packed));

/**

* @brief 灵足电机经过处理的数据, MIT模式有效

*

*/

struct Struct_Motor_RS_Rx_Data_MIT
{
    float Now_Angle;

    float Now_Omega;

    float Now_Torque;

    float Now_Temperature;

    uint32_t Pre_Encoder;

    int32_t Total_Encoder;

    int32_t Total_Round;
};

/**

* @brief 灵足电机常规源数据, MIT控制报文

*

*/
struct Struct_Motor_RS_CAN_Tx_Data_Normal_MIT
{
    uint16_t Control_Angle_Reverse;

    uint8_t Control_Omega_11_4;

    uint8_t Control_Omega_3_0_K_P_11_8;

    uint8_t K_P_7_0;

    uint8_t K_D_11_4;

    uint8_t K_D_3_0_Control_Torque_11_8;

    uint8_t Control_Torque_7_0;
    
}__attribute__((packed));


/**
 * @brief 灵足电机常规源数据, CSP控制报文
 */


struct Struct_Motor_RS_CAN_Tx_Data_CSP_MIT

{
    float Control_Angle;

    float Control_Omega;

} __attribute__((packed));


/**

* @brief 灵足电机常规源数据, 速度控制报文

*

*/

struct Struct_Motor_RS_CAN_Tx_Data_Omega_MIT

{

    float Control_Omega;

    float Control_Current_Limit;

} __attribute__((packed));

/**
 * @brief 灵足电机常规源数据, 私有协议
 */
struct Struct_Motor_RS_CAN_Rx_Data_Private

{

    uint16_t Angle_Reverse;

    uint16_t Omega_Reverse;

    uint16_t Torque_Reverse;

    uint16_t Temp_Reverse;


} __attribute__((packed));


/**
 * @brief RobStride 物理参数反馈数据 (解析后)
 */
struct Struct_Motor_RS_Rx_Data_Private
{
    float Now_Angle;      // rad
    
    float Now_Omega;      // rad/s
   
    float Now_Torque;     // Nm
   
    float Now_Temperature;// ℃
    
    uint16_t Error_Code;
    
    // 多圈逻辑变量

    uint32_t Pre_Encoder;

    int32_t Total_Encoder;

    int32_t Total_Round;
};

/**

* @brief 灵足电机类常规协议

*

*/
struct Struct_Motor_RS_CAN_Tx_Data_Normal_Private
{
    uint16_t Control_Angle;

    uint16_t Control_Omega;

    uint16_t Control_Torque;
    
    uint16_t K_P;

    uint16_t K_D;

};

/**

* @brief 灵足电机类MIT协议

*

*/
class Class_Motor_RS_MIT
{
    public:
    
    void Init(FDCAN_HandleTypeDef *hfdcan, uint8_t __CAN_Tx_ID, uint8_t __CAN_Rx_ID, Enum_Motor_RS_Control_Method __Motor_RS_Control_Method = Motor_RS_Control_Method_NORMAL, float __Angle_Max = 12.57f, float __Omega_Max = 33.0f, float __Torque_Max = 14.0f, float __Current_Max = 10.261194f);
    
    inline float Get_Angle_Max();
    
    inline float Get_Omega_Max();
    
    inline float Get_Torque_Max();
    
    inline float Get_Current_Max();
    
    inline Enum_Motor_RS_Status Get_Status();
    
    inline float Get_Now_Angle();

    inline float Get_Now_Omega();

    inline float Get_Now_Torque();

    inline float Get_Now_Temperature();

    inline Enum_Motor_RS_Control_Method Get_Control_Method();

    inline float Get_Control_Angle();

    inline float Get_Control_Omega();

    inline float Get_Control_Torque();

    inline float Get_Control_Current();

    inline float Get_K_P();

    inline float Get_K_D();

    inline void Set_Control_Angle(float __Control_Angle);

    inline void Set_Control_Omega(float __Control_Omega);

    inline void Set_Control_Torque(float __Control_Torque);

    inline void Set_Control_Current(float __Control_Current);

    inline void Set_K_P(float __K_P);

    inline void Set_K_D(float __K_D);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);

    void CAN_Send_Clear_Error();

    void CAN_Send_Enter();

    void CAN_Send_Exit();

    void CAN_Send_Save_Zero();

    void CAN_Send_Set_Control_Method(Enum_Motor_RS_Control_Method __Control_Method);

    void CAN_Send_Set_Tx_ID(uint8_t __Tx_ID);

    void CAN_Send_Set_Rx_ID(uint8_t __Rx_ID);

    void CAN_Send_Set_CAN_Protocol(Enum_Motor_RS_CAN_Protocol __CAN_Protocol);

    void TIM_Alive_PeriodElapsedCallback();

    void TIM_Send_PeriodElapsedCallback();

    private:


    // 初始化相关变量



    // 绑定的CAN

    Struct_FDCAN_Manage_Object *FDCAN_Manage_Object;

    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致

    uint16_t CAN_Rx_ID;

    // 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量

    uint16_t CAN_Tx_ID;

    // 最大位置, 与上位机控制幅值PMAX保持一致

    float Angle_Max;

    // 最大速度, 与上位机控制幅值VMAX保持一致

    float Omega_Max;

    // 最大扭矩, 与上位机控制幅值TMAX保持一致

    float Torque_Max;

    // 最大电流, 与上位机串口中上电打印电流保持一致

    float Current_Max;



    // 常量



    // 内部变量



    // 当前时刻的电机接收flag

    uint32_t Flag = 0;

    // 前一时刻的电机接收flag

    uint32_t Pre_Flag = 0;



    // 发送缓冲区

    uint8_t Tx_Data[8];


    // 读变量



    // 电机状态

    Enum_Motor_RS_Status Motor_RS_Status = Motor_RS_Status_DISABLE;

    // 电机对外接口信息

    Struct_Motor_RS_Rx_Data_MIT Rx_Data;



    // 写变量



    // 读写变量



    // 电机控制方式

    Enum_Motor_RS_Control_Method Motor_RS_Control_Method = Motor_RS_Control_Method_NORMAL;



    // 角度, rad, 目标角度

    float Control_Angle = 0.0f;

    // 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅

    float Control_Omega = 0.0f;

    // 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅

    float Control_Torque = 0.0f;

    // 电流, A, EMIT模式是限幅, 其余模式无效

    float Control_Current = 0.0f;

    // K_P, 0~500, MIT模式有效

    float K_P = 0.0f;

    // K_D, 0~5, MIT模式有效

    float K_D = 0.0f;



    // 内部函数



    void Data_Process();



    void Output();

};


/**
 * @brief 灵足电机类私有协议
 */

class Class_Motor_RS_Private
{
    public:


    void Init(FDCAN_HandleTypeDef *hfdcan, uint32_t __CAN_Tx_ID, uint32_t __CAN_Rx_ID, Enum_Motor_RS_Control_Method __Motor_RS_Control_Method = Motor_RS_Control_Method_NORMAL, float __Angle_Max = 12.57f, float __Omega_Max = 33.0f, float __Torque_Max = 14.0f, float __Current_Max = 10.261194f);
    

    uint32_t Build_ExtID(uint8_t Comm_Type, uint16_t Data);

    inline float Get_Angle_Max();
    
    inline float Get_Omega_Max();
    
    inline float Get_Torque_Max();
    
    inline float Get_Current_Max();
    
    inline Enum_Motor_RS_Status Get_Status();
    
    inline float Get_Now_Angle();

    inline float Get_Now_Omega();

    inline float Get_Now_Torque();

    inline float Get_Now_Temperature();

    inline Enum_Motor_RS_Control_Method Get_Control_Method();

    inline float Get_Control_Angle();

    inline float Get_Control_Omega();

    inline float Get_Control_Torque();

    inline float Get_Control_Current();

    inline float Get_K_P();

    inline float Get_K_D();

    inline void Set_Control_Angle(float __Control_Angle);

    inline void Set_Control_Omega(float __Control_Omega);

    inline void Set_Control_Torque(float __Control_Torque);

    inline void Set_Control_Current(float __Control_Current);

    inline void Set_K_P(float __K_P);

    inline void Set_K_D(float __K_D);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);

    void CAN_Send_Get_ID();

    void CAN_Send_Enter();

    void CAN_Send_Exit(Enum_Clear_Error_Type __Enum_Clear_Error_Type);

    void CAN_Send_Save_Zero();

    void CAN_Send_Set_Tx_ID(uint8_t __Tx_ID);

    void CAN_Send_Set_CAN_Protocol(Enum_Motor_RS_CAN_Protocol __CAN_Protocol);

    void TIM_Alive_PeriodElapsedCallback();

    void TIM_Send_PeriodElapsedCallback();

    private:    
    
    // 初始化相关变量



    // 绑定的CAN

    Struct_FDCAN_Manage_Object *FDCAN_Manage_Object;

    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致

    uint16_t CAN_Rx_ID;

    // 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量

    uint16_t CAN_Tx_ID;

    // 最大位置, 与上位机控制幅值PMAX保持一致

    float Angle_Max;

    // 最大速度, 与上位机控制幅值VMAX保持一致

    float Omega_Max;

    // 最大扭矩, 与上位机控制幅值TMAX保持一致

    float Torque_Max;

    // 最大电流, 与上位机串口中上电打印电流保持一致

    float Current_Max;



    // 常量



    // 内部变量



    // 当前时刻的电机接收flag

    uint32_t Flag = 0;

    // 前一时刻的电机接收flag

    uint32_t Pre_Flag = 0;



    // 发送缓冲区

    uint8_t Tx_Data[8];


    // 读变量



    // 电机状态

    Enum_Motor_RS_Status Motor_RS_Status = Motor_RS_Status_DISABLE;

    // 电机对外接口信息

    Struct_Motor_RS_Rx_Data_Private Rx_Data;



    // 写变量



    // 读写变量



    // 电机控制方式

    Enum_Motor_RS_Control_Method Motor_RS_Control_Method = Motor_RS_Control_Method_NORMAL;



    // 角度, rad, 目标角度

    float Control_Angle = 0.0f;

    // 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅

    float Control_Omega = 0.0f;

    // 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅

    float Control_Torque = 0.0f;

    // 电流, A, EMIT模式是限幅, 其余模式无效

    float Control_Current = 0.0f;

    // K_P, 0~500, MIT模式有效

    float K_P = 0.0f;

    // K_D, 0~5, MIT模式有效

    float K_D = 0.0f;



    // 内部函数



    void Data_Process();



    void Output();
};


/**

* @brief 获取角度最大值

*

* @return float 角度最大值

*/

inline float Class_Motor_RS_MIT::Get_Angle_Max()
{
    return Angle_Max;
}

/**
 * @brief 获取角速度最大值
 *
 * @return float 角速度最大值
 */

inline float Class_Motor_RS_MIT::Get_Omega_Max()
{
    return Omega_Max;
}

/**
 *  @brief 获取扭矩最大值
 * 
 *  @return float 扭矩最大值
 */

inline float Class_Motor_RS_MIT::Get_Torque_Max()
{
    return Torque_Max;
}

/**
 * @brief 获取电流最大值
 * 
 * @return float 电流最大值
 */

inline float Class_Motor_RS_MIT::Get_Current_Max()
{
    return Current_Max;
}


/**
 * @brief 获取电机状态
 * 
 * @return Enum_Motor_RS_Status 电机状态
 */

 inline Enum_Motor_RS_Status Class_Motor_RS_MIT::Get_Status()
{
    return Motor_RS_Status;
}

/**
 * @brief 获取当前时刻电机角度
 * 
 * @return float 当前时刻电机角度
 */

 inline float Class_Motor_RS_MIT::Get_Now_Angle()
{
    return Rx_Data.Now_Angle;
}


/**
 * @brief 获取当前时刻电机角速度
 * 
 * @return float 当前时刻电机角速度
 */

 inline float Class_Motor_RS_MIT::Get_Now_Omega()
{
    return Rx_Data.Now_Omega;
}


/**
 * @brief 获取当前时刻电机扭矩
 * 
 * @return float 当前时刻电机扭矩
 */

inline float Class_Motor_RS_MIT::Get_Now_Torque()
{
    return Rx_Data.Now_Torque;
}


/**
 * @brief 获取当前时刻电机温度
 * 
 * @return float 当前时刻电机温度
 */

 inline float Class_Motor_RS_MIT::Get_Now_Temperature()
{
    return Rx_Data.Now_Temperature;
}


/**
 * @brief 获取当前电机控制方式
 * 
 * @return Enum_Motor_RS_Control_Method 当前电机控制方式
 */
inline Enum_Motor_RS_Control_Method Class_Motor_RS_MIT::Get_Control_Method()
{
    return Motor_RS_Control_Method;
}


/**
 * @brief 获取当前电机目标角度
 * 
 * @return float 当前电机目标角度
 */

 inline float Class_Motor_RS_MIT::Get_Control_Angle()
{
    return Control_Angle;
}


/**
 * @brief 获取当前电机目标角速度
 * 
 * @return float 当前电机目标角速度
 */

 inline float Class_Motor_RS_MIT::Get_Control_Omega()
{
    return Control_Omega;
}


/**
 * @brief 获取当前电机目标扭矩
 * 
 * @return float 当前电机目标扭矩
 */

 inline float Class_Motor_RS_MIT::Get_Control_Torque()
{
    return Control_Torque;
}


/**
 * @brief 获取当前电机目标电流
 * 
 * @return float 当前电机目标电流
 */

 inline float Class_Motor_RS_MIT::Get_Control_Current()
{
    return Control_Current;
}

/**
 * @brief 获取当前电机目标K_P
 * 
 * @return float 当前电机目标K_P
 */

 inline float Class_Motor_RS_MIT::Get_K_P()
{
    return K_P;
}

/**
 * @brief 获取当前电机目标K_D
 * 
 * @return float 当前电机目标K_D
 */

 inline float Class_Motor_RS_MIT::Get_K_D()
{
    return K_D;
}


/**
 * @brief 设定角度, rad, 目标角度
 * 
 * @param __Control_Angle 角度, rad, 目标角度
 */

 inline void Class_Motor_RS_MIT::Set_Control_Angle(float __Control_Angle)
{
    Control_Angle = __Control_Angle;
}


/**
 * @brief 设定角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 * 
 * @param __Control_Omega 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 */

 inline void Class_Motor_RS_MIT::Set_Control_Omega(float __Control_Omega)
{
    Control_Omega = __Control_Omega;
}


/**
 * @brief 设定扭矩, Nm, MIT模式是目标扭矩,
 * 
 * @param __Control_Torque 扭矩, Nm, MIT模式是目标扭矩
 */


 inline void Class_Motor_RS_MIT::Set_Control_Torque(float __Control_Torque)
{
    Control_Torque = __Control_Torque;
}
/**
 * @brief 设置当前电机目标K_P
 * 
 * @param K_P 当前电机目标K_P
 */

 inline void Class_Motor_RS_MIT::Set_K_P(float __K_P)
{
    K_P = __K_P;
}

/**
 * @brief 设置当前电机目标K_D
 * 
 * @param K_D 当前电机目标K_D
 */

 inline void Class_Motor_RS_MIT::Set_K_D(float __K_D)
{
    K_D = __K_D;
}















/**

* @brief 获取角度最大值

*

* @return float 角度最大值

*/

inline float Class_Motor_RS_Private::Get_Angle_Max()
{
    return Angle_Max;
}

/**
 * @brief 获取角速度最大值
 *
 * @return float 角速度最大值
 */

inline float Class_Motor_RS_Private::Get_Omega_Max()
{
    return Omega_Max;
}

/**
 *  @brief 获取扭矩最大值
 * 
 *  @return float 扭矩最大值
 */

inline float Class_Motor_RS_Private::Get_Torque_Max()
{
    return Torque_Max;
}

/**
 * @brief 获取电流最大值
 * 
 * @return float 电流最大值
 */

inline float Class_Motor_RS_Private::Get_Current_Max()
{
    return Current_Max;
}


/**
 * @brief 获取电机状态
 * 
 * @return Enum_Motor_RS_Status 电机状态
 */

 inline Enum_Motor_RS_Status Class_Motor_RS_Private::Get_Status()
{
    return Motor_RS_Status;
}

/**
 * @brief 获取当前时刻电机角度
 * 
 * @return float 当前时刻电机角度
 */

 inline float Class_Motor_RS_Private::Get_Now_Angle()
{
    return Rx_Data.Now_Angle;
}


/**
 * @brief 获取当前时刻电机角速度
 * 
 * @return float 当前时刻电机角速度
 */

 inline float Class_Motor_RS_Private::Get_Now_Omega()
{
    return Rx_Data.Now_Omega;
}


/**
 * @brief 获取当前时刻电机扭矩
 * 
 * @return float 当前时刻电机扭矩
 */

inline float Class_Motor_RS_Private::Get_Now_Torque()
{
    return Rx_Data.Now_Torque;
}


/**
 * @brief 获取当前时刻电机温度
 * 
 * @return float 当前时刻电机温度
 */

 inline float Class_Motor_RS_Private::Get_Now_Temperature()
{
    return Rx_Data.Now_Temperature;
}


/**
 * @brief 获取当前电机控制方式
 * 
 * @return Enum_Motor_RS_Control_Method 当前电机控制方式
 */
inline Enum_Motor_RS_Control_Method Class_Motor_RS_Private::Get_Control_Method()
{
    return Motor_RS_Control_Method;
}


/**
 * @brief 获取当前电机目标角度
 * 
 * @return float 当前电机目标角度
 */

 inline float Class_Motor_RS_Private::Get_Control_Angle()
{
    return Control_Angle;
}


/**
 * @brief 获取当前电机目标角速度
 * 
 * @return float 当前电机目标角速度
 */

 inline float Class_Motor_RS_Private::Get_Control_Omega()
{
    return Control_Omega;
}


/**
 * @brief 获取当前电机目标扭矩
 * 
 * @return float 当前电机目标扭矩
 */

 inline float Class_Motor_RS_Private::Get_Control_Torque()
{
    return Control_Torque;
}


/**
 * @brief 获取当前电机目标电流
 * 
 * @return float 当前电机目标电流
 */

 inline float Class_Motor_RS_Private::Get_Control_Current()
{
    return Control_Current;
}

/**
 * @brief 获取当前电机目标K_P
 * 
 * @return float 当前电机目标K_P
 */

 inline float Class_Motor_RS_Private::Get_K_P()
{
    return K_P;
}

/**
 * @brief 获取当前电机目标K_D
 * 
 * @return float 当前电机目标K_D
 */

 inline float Class_Motor_RS_Private::Get_K_D()
{
    return K_D;
}


/**
 * @brief 设定角度, rad, 目标角度
 * 
 * @param __Control_Angle 角度, rad, 目标角度
 */

 inline void Class_Motor_RS_Private::Set_Control_Angle(float __Control_Angle)
{
    Control_Angle = __Control_Angle;
}


/**
 * @brief 设定角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 * 
 * @param __Control_Omega 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 */

 inline void Class_Motor_RS_Private::Set_Control_Omega(float __Control_Omega)
{
    Control_Omega = __Control_Omega;
}


/**
 * @brief 设定扭矩, Nm, MIT模式是目标扭矩,
 * 
 * @param __Control_Torque 扭矩, Nm, MIT模式是目标扭矩
 */


 inline void Class_Motor_RS_Private::Set_Control_Torque(float __Control_Torque)
{
    Control_Torque = __Control_Torque;
}
/**
 * @brief 设置当前电机目标K_P
 * 
 * @param K_P 当前电机目标K_P
 */

 inline void Class_Motor_RS_Private::Set_K_P(float __K_P)
{
    K_P = __K_P;
}

/**
 * @brief 设置当前电机目标K_D
 * 
 * @param K_D 当前电机目标K_D
 */

 inline void Class_Motor_RS_Private::Set_K_D(float __K_D)
{
    K_D = __K_D;
}

#endif

