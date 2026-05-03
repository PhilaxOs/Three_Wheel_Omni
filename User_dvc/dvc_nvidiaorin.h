/**
 * @file dvc_nvidiaorin.h
 * @author Lucy (2478427315@qq.com)
 * @brief Orin host communication over FDCAN
 * @version 0.2
 * @date 2026-04-02
 *
 * @copyright ROBOPIONEER (c) 2024
 *
 */

#ifndef DVC_NVIDIAORIN_H
#define DVC_NVIDIAORIN_H

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"
#include "drv_can.h"

/* Exported types ------------------------------------------------------------*/

enum Enum_Orin_Status
{
    Orin_Status_DISABLE = 0,
    Orin_Status_ENABLE,
};

struct Struct_Orin_CAN_Rx_Data
{
    float Target_Velocity_X = 0.0f;
    float Target_Velocity_Y = 0.0f;
    float Target_Omega = 0.0f;
    float Yaw_Target_Omega = 0.0f;
    float Pitch_Target_Omega = 0.0f;
    uint8_t Flag_Todo1 = 0;
    uint8_t Flag_Todo2 = 0;
    uint8_t Flag_Todo3 = 0;
    uint8_t Flag_Todo4 = 0;
    uint8_t Flag_Todo5 = 0;
    uint8_t Flag_Todo6 = 0;
};

struct Struct_Orin_CAN_Tx_Data
{
    float Now_Velocity_X = 0.0f;
    float Now_Velocity_Y = 0.0f;
    float Now_Omega = 0.0f;
    uint8_t Flag_Todo1 = 0;
    uint8_t Flag_Todo2 = 0;
    uint8_t Flag_Todo3 = 0;
    uint8_t Flag_Todo4 = 0;
    uint8_t Flag_Todo5 = 0;
    uint8_t Flag_Todo6 = 0;
};

struct Struct_Orin_CAN_Float2_Frame
{
    float Value_1;
    float Value_2;
} __attribute__((packed));

struct Struct_Orin_CAN_Int16x3_Frame
{
    int16_t Velocity_X;
    int16_t Velocity_Y;
    int16_t Omega;
    int16_t Reserved;
} __attribute__((packed));

struct Struct_Orin_CAN_FloatFlag_Frame
{
    float Value;
    uint16_t Flags;
    uint16_t Sequence;
} __attribute__((packed));

class Class_Orin
{
public:
    static const uint16_t CAN_ID_RX_VELOCITY = 0x310;
    static const uint16_t CAN_ID_RX_OMEGA = 0x311;
    static const uint16_t CAN_ID_RX_PITCH = 0x312;
    static const uint16_t CAN_ID_TX_VELOCITY = 0x320;
    static const uint16_t CAN_ID_TX_OMEGA = 0x321;

    void Init(FDCAN_HandleTypeDef *__hfdcan);

    inline Enum_Orin_Status Get_Status();

    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();
    inline float Get_Pitch_Target_Omega();
    inline float Get_Yaw_Target_Omega();

    inline void Set_Now_Velocity_X(float __Velocity_X);
    inline void Set_Now_Velocity_Y(float __Velocity_Y);
    inline void Set_Now_Omega(float __Now_Omega);

    void FDCAN_RxCpltCallback(Struct_FDCAN_Rx_Buffer *Rx_Message);

    void TIM_101ms_Alive_PeriodElapsedCallback();
    void TIM_10ms_Send_PeriodElapsedCallback();

protected:
    Struct_FDCAN_Manage_Object *FDCAN_Manage_Object = nullptr;

    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;
    uint16_t Tx_Sequence = 0;

    Enum_Orin_Status Orin_Status = Orin_Status_DISABLE;
    Struct_Orin_CAN_Rx_Data Rx_Data;
    Struct_Orin_CAN_Tx_Data Tx_Data;

    void Clear_Rx_Data();
    void Parse_Flag_Mask(uint16_t flag_mask);
    uint16_t Get_Tx_Flag_Mask() const;
    bool Is_Valid_Frame(const Struct_FDCAN_Rx_Buffer *Rx_Message) const;
};

/* Exported function declarations --------------------------------------------*/

inline Enum_Orin_Status Class_Orin::Get_Status()
{
    return Orin_Status;
}

inline float Class_Orin::Get_Target_Velocity_X()
{
    return Rx_Data.Target_Velocity_X;
}

inline float Class_Orin::Get_Target_Velocity_Y()
{
    return Rx_Data.Target_Velocity_Y;
}

inline float Class_Orin::Get_Target_Omega()
{
    return Rx_Data.Target_Omega;
}

inline float Class_Orin::Get_Pitch_Target_Omega()
{
    return Rx_Data.Pitch_Target_Omega;
}

inline float Class_Orin::Get_Yaw_Target_Omega()
{
    return Rx_Data.Yaw_Target_Omega;
}

inline void Class_Orin::Set_Now_Velocity_X(float __Velocity_X)
{
    Tx_Data.Now_Velocity_X = __Velocity_X;
}

inline void Class_Orin::Set_Now_Velocity_Y(float __Velocity_Y)
{
    Tx_Data.Now_Velocity_Y = __Velocity_Y;
}

inline void Class_Orin::Set_Now_Omega(float __Now_Omega)
{
    Tx_Data.Now_Omega = __Now_Omega;
}

#endif

/************************ COPYRIGHT(C) ROBOPIONEER **************************/
