/**
 * @file dvc_motor_rs.cpp
 * @author hzy by yssickjgd
 * @brief 灵足电机配置与操作
 * @version 0.1
 * @date 2026-3-26 0.1 新增
 *
 * @copyright NEUQ (c) 2026
 *
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_rs.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/\
/**
 * @brief RobStride 通信类型 (由扩展帧ID的 24~28位 决定)
 */


// 清除电机错误信息, 传统模式有效
uint8_t RS_Motor_CAN_Message_Clear_Error[8] = {0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xfb};
// 使能电机, 传统模式有效
uint8_t RS_Motor_CAN_Message_Enter[8] = {0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xfc};
// 失能电机, 传统模式有效
uint8_t RS_Motor_CAN_Message_Exit[8] = {0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xfd};
// 保存当前电机位置为零点, 传统模式有效
uint8_t RS_Motor_CAN_Message_Save_Zero[8] = {0xff,
                                             0xff,
                                             0xff,
                                             0xff,
                                             0xff,
                                             0xff,
                                             0xff,
                                             0xfe};
//设置运行模式
uint8_t RS_Motor_CAN_Message_Set_Control_Method[8] = {0xff,
                                                      0xff,
                                                      0xff,
                                                      0xff,
                                                      0xff,
                                                      0xff,
                                                      0x00,
                                                      0xfc};
//设置电机CANID
uint8_t RS_Motor_CAN_Message_Set_CAN_ID[8] = {0xff,
                                              0xff,
                                              0xff,
                                              0xff,
                                              0xff,
                                              0xff,
                                              0x7f,
                                              0xfa};
//设置主机CANID
uint8_t RS_Motor_CAN_Message_Set_Master_ID[8] = {0xff,
                                                 0xff,
                                                 0xff,
                                                 0xff,
                                                 0xff,
                                                 0xff,
                                                 0xfd,
                                                 0x01};
//设置电机CAN协议
uint8_t RS_Motor_CAN_Message_Set_CAN_Protocol[8] = {0xff,
                                                    0xff,
                                                    0xff,
                                                    0xff,
                                                    0xff,
                                                    0xff,
                                                    0x02,
                                                    0xFD};
                            
/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/



/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致, 传统模式有效
 * @param __CAN_Tx_ID 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量, 传统模式有效
 * @param __Motor_DM_Control_Method 电机控制方式
 * @param __Angle_Max 最大位置, 与上位机控制幅值PMAX保持一致, 传统模式有效
 * @param __Omega_Max 最大速度, 与上位机控制幅值VMAX保持一致, 传统模式有效
 * @param __Torque_Max 最大扭矩, 与上位机控制幅值TMAX保持一致, 传统模式有效
 */
void Class_Motor_RS_MIT::Init(FDCAN_HandleTypeDef *hfdcan, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID, Enum_Motor_RS_Control_Method __Motor_RS_Control_Method, float __Angle_Max, float __Omega_Max, float __Torque_Max, float __Current_Max)
{
    if (hfdcan->Instance == FDCAN1)
    {
        FDCAN_Manage_Object = &FDCAN1_Manage_Object;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        FDCAN_Manage_Object = &FDCAN2_Manage_Object;
    }
    else if(hfdcan->Instance == FDCAN3)
    {
        FDCAN_Manage_Object = &FDCAN3_Manage_Object;
    }

    CAN_Rx_ID = __CAN_Rx_ID;
    CAN_Tx_ID = __CAN_Tx_ID;


    Motor_RS_Control_Method = __Motor_RS_Control_Method;
    Angle_Max = __Angle_Max;
    Omega_Max = __Omega_Max;
    Torque_Max = __Torque_Max;
    Current_Max = __Current_Max;
}


/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Motor_RS_MIT::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief 发送清除错误信息
 *
 */
void Class_Motor_RS_MIT::CAN_Send_Clear_Error()
{
    // CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, DM_Motor_CAN_Message_Clear_Error, 8);
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, CAN_Tx_ID, RS_Motor_CAN_Message_Clear_Error, FDCAN_ID_Standard, 8);
}

/**
 * @brief 发送使能电机
 *
 */
void Class_Motor_RS_MIT::CAN_Send_Enter()
{
    // CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, DM_Motor_CAN_Message_Enter, 8);
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, CAN_Tx_ID, RS_Motor_CAN_Message_Enter, FDCAN_ID_Standard, 8);
}

/**
 * @brief 发送失能电机
 *
 */
void Class_Motor_RS_MIT::CAN_Send_Exit()
{
    //CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, DM_Motor_CAN_Message_Exit, 8);
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, CAN_Tx_ID, RS_Motor_CAN_Message_Exit, FDCAN_ID_Standard, 8);
}

/**
 * @brief 发送保存当前位置为零点
 *
 */
void Class_Motor_RS_MIT::CAN_Send_Save_Zero()
{
    //CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, DM_Motor_CAN_Message_Save_Zero, 8);
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, CAN_Tx_ID, RS_Motor_CAN_Message_Save_Zero, FDCAN_ID_Standard, 8);
}

/**
 * @brief 设置运行模式
 *
 */

 void Class_Motor_RS_MIT::CAN_Send_Set_Control_Method(Enum_Motor_RS_Control_Method __Motor_RS_Control_Method)
{
    RS_Motor_CAN_Message_Set_Control_Method[6] = (uint8_t)__Motor_RS_Control_Method;

    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, CAN_Tx_ID, RS_Motor_CAN_Message_Set_Control_Method, FDCAN_ID_Standard, 8);
}


/**
 * @brief 设置电机CANID
 *
 * @param __CAN_ID
 */
void Class_Motor_RS_MIT::CAN_Send_Set_Tx_ID(uint8_t __Tx_ID)
{
    RS_Motor_CAN_Message_Set_CAN_ID[6] = __Tx_ID;

    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, CAN_Tx_ID, RS_Motor_CAN_Message_Set_CAN_ID, FDCAN_ID_Standard, 8);
}

/**
 * @brief 设置主机CANID
 *
 * @param __CAN_ID
 */
void Class_Motor_RS_MIT::CAN_Send_Set_Rx_ID(uint8_t __Rx_ID)
{
    RS_Motor_CAN_Message_Set_Master_ID[6] = __Rx_ID;

    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, CAN_Tx_ID, RS_Motor_CAN_Message_Set_Master_ID, FDCAN_ID_Standard, 8);
}


/**
 * @brief 设置电机CAN协议
 *
 * @param __CAN_Protocol
 */
void Class_Motor_RS_MIT::CAN_Send_Set_CAN_Protocol(Enum_Motor_RS_CAN_Protocol __CAN_Protocol)
{
    RS_Motor_CAN_Message_Set_CAN_Protocol[6] = (uint8_t)__CAN_Protocol;

    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, CAN_Tx_ID, RS_Motor_CAN_Message_Set_CAN_Protocol, FDCAN_ID_Standard, 8);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活, 检测周期取决于电机掉线时长
 *
 */
void Class_Motor_RS_MIT::TIM_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        // 电机断开连接
        Motor_RS_Status = Motor_RS_Status_DISABLE;
    }
    else
    {
        // 电机保持连接
        Motor_RS_Status = Motor_RS_Status_ENABLE;
    }

    Pre_Flag = Flag;

    if(Motor_RS_Status == Motor_RS_Status_DISABLE)
    {
        CAN_Send_Enter();
    }
}

/**
 * @brief TIM定时器中断发送出去的回调函数, 计算周期取决于自主设置的控制周期
 *
 */
void Class_Motor_RS_MIT::TIM_Send_PeriodElapsedCallback()
{
    if (Motor_RS_Status == Motor_RS_Status_ENABLE)
    {
        // 电机在线, 正常控制
        Math_Constrain(&Control_Angle, -Angle_Max, Angle_Max);
        Math_Constrain(&Control_Omega, -Omega_Max, Omega_Max);
        Math_Constrain(&Control_Torque, -Torque_Max, Torque_Max);
        Math_Constrain(&Control_Current, -Current_Max, Current_Max);
        Math_Constrain(&K_P, 0.0f, 500.0f);
        Math_Constrain(&K_D, 0.0f, 5.0f);

        Output();
    }
    else if (Motor_RS_Status == Motor_RS_Status_DISABLE)
    {
        // 电机可能掉线, 使能电机
        CAN_Send_Enter();
    }
    else
    {
        // 电机错误, 发送清除错误帧
        CAN_Send_Clear_Error();
    }
}

/**
 * @brief 数据处理过程
 *
 */
void Class_Motor_RS_MIT::Data_Process()
{
    // 数据处理过程
    int32_t delta_encoder;
    uint16_t tmp_encoder, tmp_omega, tmp_torque, tmp_temperature;
    Struct_Motor_RS_CAN_Rx_Data_MIT *tmp_buffer = (Struct_Motor_RS_CAN_Rx_Data_MIT *)FDCAN_Manage_Object->Rx_Buffer.Data;

    // 电机ID不匹配, 则不进行处理
    if(tmp_buffer->CAN_ID != CAN_Tx_ID )
    {
        return;
    }

    // 处理大小端
    Math_Endian_Reverse_16((void *)&tmp_buffer->Angle_Reverse, &tmp_encoder);
    tmp_omega = (tmp_buffer->Omega_11_4 << 4) | (tmp_buffer->Omega_3_0_Torque_11_8 >> 4);
    tmp_torque = ((tmp_buffer->Omega_3_0_Torque_11_8 & 0x0f) << 8) | (tmp_buffer->Torque_7_0);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Temp_Reverse, &tmp_temperature);



    // 计算电机本身信息
    Rx_Data.Now_Angle = Math_Int_To_Float(tmp_encoder, 0, 65535, - Angle_Max, Angle_Max);
    Rx_Data.Now_Omega = Math_Int_To_Float(tmp_omega, 0, 4096, - Omega_Max, Omega_Max);
    Rx_Data.Now_Torque = Math_Int_To_Float(tmp_torque, 0, 4096, - Torque_Max, Torque_Max);
    Rx_Data.Now_Temperature = tmp_temperature * 0.1f;

    // 存储预备信息
    Rx_Data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief 电机数据输出到CAN总线
 *
 */
void Class_Motor_RS_MIT::Output()
{
    // 电机控制
    switch (Motor_RS_Control_Method)
    {
        case (Motor_RS_Control_Method_NORMAL):
        {
            Struct_Motor_RS_CAN_Tx_Data_Normal_MIT *tmp_buffer = (Struct_Motor_RS_CAN_Tx_Data_Normal_MIT *)Tx_Data;

            uint16_t tmp_angle, tmp_omega, tmp_torque, tmp_k_p, tmp_k_d;

            tmp_angle = Math_Float_To_Int(Control_Angle, -Angle_Max, Angle_Max, 0, 65535);
            tmp_omega = Math_Float_To_Int(Control_Omega, -Omega_Max, Omega_Max, 0, 4095);
            tmp_torque = Math_Float_To_Int(Control_Torque, -Torque_Max, Torque_Max, 0, 4095);
            tmp_k_p = Math_Float_To_Int(K_P, 0, 500.0f, 0, 4095);
            tmp_k_d = Math_Float_To_Int(K_D, 0, 5.0f, 0, 4095);

            tmp_buffer->Control_Angle_Reverse = Math_Endian_Reverse_16(&tmp_angle, nullptr);
            tmp_buffer->Control_Omega_11_4 = tmp_omega >> 4;
            tmp_buffer->Control_Omega_3_0_K_P_11_8 = ((tmp_omega & 0x0f) << 4) | (tmp_k_p >> 8);
            tmp_buffer->K_P_7_0 = tmp_k_p & 0xff;
            tmp_buffer->K_D_11_4 = tmp_k_d >> 4;
            tmp_buffer->K_D_3_0_Control_Torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
            tmp_buffer->Control_Torque_7_0 = tmp_torque & 0xff;

            FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, CAN_Tx_ID, Tx_Data, FDCAN_ID_Standard, 8);

            break;
        }

        case (Motor_RS_Control_Method_CSP):
        {
            Struct_Motor_RS_CAN_Tx_Data_CSP_MIT *tmp_buffer = (Struct_Motor_RS_CAN_Tx_Data_CSP_MIT *)Tx_Data;
            tmp_buffer->Control_Angle = Control_Angle;
            tmp_buffer->Control_Omega = Control_Omega;
            uint16_t send_id = CAN_Tx_ID + 0x100; // CSP模式的CAN ID是在正常模式的基础上加0x100
            FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, send_id, Tx_Data, FDCAN_ID_Standard, 8);
            break;
        }

        case (Motor_RS_Control_Method_OMEGA):
        {
            Struct_Motor_RS_CAN_Tx_Data_Omega_MIT *tmp_buffer = (Struct_Motor_RS_CAN_Tx_Data_Omega_MIT *)Tx_Data;
            tmp_buffer->Control_Omega = Control_Omega;
            tmp_buffer->Control_Current_Limit = Control_Current;
            uint16_t send_id = CAN_Tx_ID + 0x200; // 速度模式的CAN ID是在正常模式的基础上加0x200
            FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, send_id, Tx_Data, FDCAN_ID_Standard, 8);
            break;
        }
    }
}





/**
 * @brief 电机初始化
 * 
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 绑定的CAN ID
 * @param __Motor_DM_Control_Method 电机控制方式, 默认角度
 * @param __Encoder_Offset 编码器偏移, 默认0
 * @param __Current_Max 最大电流
 */
void Class_Motor_RS_Private::Init(FDCAN_HandleTypeDef *hfdcan, uint32_t __CAN_Rx_ID, uint32_t __CAN_Tx_ID, Enum_Motor_RS_Control_Method __Motor_RS_Control_Method, float __Angle_Max, float __Omega_Max, float __Torque_Max, float __Current_Max)
{
    if (hfdcan->Instance == FDCAN1)
    {
        FDCAN_Manage_Object = &FDCAN1_Manage_Object;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        FDCAN_Manage_Object = &FDCAN2_Manage_Object;
    }
    else if(hfdcan->Instance == FDCAN3)
    {
        FDCAN_Manage_Object = &FDCAN3_Manage_Object;
    }
		CAN_Tx_ID = __CAN_Tx_ID;
    CAN_Rx_ID = __CAN_Rx_ID;


    Motor_RS_Control_Method = __Motor_RS_Control_Method;
    Angle_Max = __Angle_Max;
    Omega_Max = __Omega_Max;
    Torque_Max = __Torque_Max;
    Current_Max = __Current_Max;
}

uint32_t Class_Motor_RS_Private::Build_ExtID(uint8_t Comm_Type, uint16_t Data)
{
    uint32_t extid = 0;
    // 使用 |= 是为了在原有的基础上设置对应位, 避免覆盖其他位的数据
    extid |= (uint32_t)(Comm_Type & 0x1F) << 24; 
    extid |= (uint32_t)(Data      & 0xFFFF) << 8; 
    extid |= (uint32_t)(CAN_Tx_ID & 0xFF);       
    return extid;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Motor_RS_Private::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}


/**
 * @brief 获取设备ID和MCU唯一标识符
 */
void Class_Motor_RS_Private::CAN_Send_Get_ID()
{
    uint8_t tx_data[8] = {0};
    uint32_t send_id = Build_ExtID(RS_Comm_GetID, CAN_Rx_ID);
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, send_id, tx_data, FDCAN_ID_Extended, 8);
}
/**
/**
 * @brief 发送使能电机  
 *
 */
void Class_Motor_RS_Private::CAN_Send_Enter()
{
    uint32_t send_id = Build_ExtID(RS_Comm_MotorEnable, CAN_Rx_ID);
    uint8_t tx_data[8] = {0};
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, send_id, tx_data, FDCAN_ID_Extended, 8);
}

/**
 * @brief 发送失能电机
 *
 */
void Class_Motor_RS_Private::CAN_Send_Exit(Enum_Clear_Error_Type __Enum_Clear_Error_Type)
{
    uint32_t send_id = Build_ExtID(RS_Comm_MotorStop, CAN_Rx_ID);
    uint8_t tx_data[8] = {0};
    tx_data[0] = (uint8_t)__Enum_Clear_Error_Type;
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, send_id, tx_data, FDCAN_ID_Extended, 8);;
}

/**
 * @brief 发送保存当前位置为零点
 *
 */
void Class_Motor_RS_Private::CAN_Send_Save_Zero()
{
    uint32_t send_id = Build_ExtID(RS_Comm_SetPosZero, CAN_Rx_ID);
    uint8_t tx_data[8] = {0};
    tx_data[0] = 0x01;
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, send_id, tx_data, FDCAN_ID_Extended, 8);
}





/**
 * @brief 设置电机CANID
 *
 * @param __CAN_ID
 */
void Class_Motor_RS_Private::CAN_Send_Set_Tx_ID(uint8_t __Tx_ID)
{   
    // 直接传入拼好的 16 位 Data，高 8 位是新 ID，低 8 位是主站 ID
    uint16_t data_field = (__Tx_ID << 8) | CAN_Rx_ID; 
    uint32_t send_id = Build_ExtID(RS_Comm_SetCanID, data_field);
    uint8_t tx_data[8] = {0};
    tx_data[0] = 0x01;
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, send_id, tx_data, FDCAN_ID_Extended, 8);
}



/**
 * @brief 设置电机CAN协议
 *
 * @param __CAN_Protocol
 */
void Class_Motor_RS_Private::CAN_Send_Set_CAN_Protocol(Enum_Motor_RS_CAN_Protocol __CAN_Protocol)
{
    uint32_t send_id = Build_ExtID(RS_Comm_MotorModeSet, CAN_Rx_ID); 
    //01 02 03 04 05 06 F_CMD  
    uint8_t tx_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, (uint8_t)__CAN_Protocol,0x00};
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, send_id, tx_data, FDCAN_ID_Extended, 8);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活, 检测周期取决于电机掉线时长
 *
 */
void Class_Motor_RS_Private::TIM_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        // 电机断开连接
        Motor_RS_Status = Motor_RS_Status_DISABLE;
    }
    else
    {
        // 电机保持连接
        Motor_RS_Status = Motor_RS_Status_ENABLE;
    }

    Pre_Flag = Flag;

    if(Motor_RS_Status == Motor_RS_Status_DISABLE)
    {
        CAN_Send_Enter();
    }
}

/**
 * @brief TIM定时器中断发送出去的回调函数, 计算周期取决于自主设置的控制周期
 *
 */
void Class_Motor_RS_Private::TIM_Send_PeriodElapsedCallback()
{
    if (Motor_RS_Status == Motor_RS_Status_ENABLE)
    {
        // 电机在线, 正常控制
        Math_Constrain(&Control_Angle, -Angle_Max, Angle_Max);
        Math_Constrain(&Control_Omega, -Omega_Max, Omega_Max);
        Math_Constrain(&Control_Torque, -Torque_Max, Torque_Max);
        Math_Constrain(&Control_Current, -Current_Max, Current_Max);
        Math_Constrain(&K_P, 0.0f, 500.0f);
        Math_Constrain(&K_D, 0.0f, 5.0f);

        Output();
    }
    else if (Motor_RS_Status == Motor_RS_Status_DISABLE)
    {
        // 电机可能掉线, 使能电机
        CAN_Send_Enter();
    }
    else
    {
        // 电机错误, 发送清除错误帧
        CAN_Send_Exit(Enum_CLEAR_ERROR_Type);
    }
}

/**
 * @brief 数据处理过程
 *
 */
void Class_Motor_RS_Private::Data_Process()
{   
    // 获取扩展帧ID的29位
    uint32_t temp_extid = FDCAN_Manage_Object->Rx_Buffer.Header.Identifier & 0x1FFFFFFF;
     // 数据处理过程
    int32_t delta_encoder;
    uint16_t tmp_encoder, tmp_omega, tmp_torque, tmp_temperature;
    Struct_Motor_RS_CAN_Rx_Data_Private *tmp_buffer = (Struct_Motor_RS_CAN_Rx_Data_Private *)FDCAN_Manage_Object->Rx_Buffer.Data;

    // 电机ID不匹配, 则不进行处理 Bit8~Bit15:当前电机 CAN ID
    uint8_t received_id = (temp_extid >> 8) & 0xFF; // 提取 Bit8~Bit15
    if(received_id != (CAN_Tx_ID & 0xFF))
    {
        return; 
    }

    // 处理大小端
    Math_Endian_Reverse_16((void *)&tmp_buffer->Angle_Reverse, &tmp_encoder);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Omega_Reverse, &tmp_omega);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Torque_Reverse, &tmp_torque);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Temp_Reverse, &tmp_temperature);


    Rx_Data.Now_Angle = Math_Int_To_Float(tmp_encoder, 0, 65535, - Angle_Max, Angle_Max);
    Rx_Data.Now_Omega = Math_Int_To_Float(tmp_omega, 0, 65535, - Omega_Max, Omega_Max);
    Rx_Data.Now_Torque = Math_Int_To_Float(tmp_torque, 0, 65535, - Torque_Max, Torque_Max);
    Rx_Data.Now_Temperature = tmp_temperature * 0.1f;

    // 存储预备信息
    Rx_Data.Pre_Encoder = tmp_encoder;

    // 错误状态解析
    Rx_Data.Error_Code = (uint16_t)((temp_extid >> 16) & 0x3F);
}

/**
 * @brief 电机数据输出到CAN总线
 *
 */
void Class_Motor_RS_Private::Output()
{
    // 电机控制
    switch (Motor_RS_Control_Method)
    {
        case (Motor_RS_Control_Method_NORMAL):
        {
            Struct_Motor_RS_CAN_Tx_Data_Normal_Private *tmp_buffer = (Struct_Motor_RS_CAN_Tx_Data_Normal_Private *)Tx_Data;

            uint16_t tmp_angle, tmp_omega, tmp_torque, tmp_k_p, tmp_k_d;

            tmp_angle = Math_Float_To_Int(Control_Angle, 0, Angle_Max, 0, 65535);
            tmp_omega = Math_Float_To_Int(Control_Omega, 0, Omega_Max, 0, 65535);
            tmp_torque = Math_Float_To_Int(Control_Torque, 0, Torque_Max, 0, 65535);
            tmp_k_p = Math_Float_To_Int(K_P, 0, 500.0f, 0, 65535);
            tmp_k_d = Math_Float_To_Int(K_D, 0, 5.0f, 0, 65535);

            tmp_buffer->Control_Angle = Math_Endian_Reverse_16(&tmp_angle, nullptr);
            tmp_buffer->Control_Omega = Math_Endian_Reverse_16(&tmp_omega, nullptr);
            tmp_buffer->K_P = Math_Endian_Reverse_16(&tmp_k_p, nullptr);
            tmp_buffer->K_D = Math_Endian_Reverse_16(&tmp_k_d, nullptr);

            //tmp_buffer->Control_Torque不能处理成大端
            uint32_t send_id = Build_ExtID(RS_Comm_MotionControl, tmp_torque);

            FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, send_id, Tx_Data, FDCAN_ID_Extended, 8);

            break;
        }
    }
}
