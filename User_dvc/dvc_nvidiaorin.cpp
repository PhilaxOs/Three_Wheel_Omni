/**
 * @file dvc_nvidiaorin.cpp
 * @author Lucy (2478427315@qq.com)
 * @brief Orin host communication over FDCAN
 * @version 0.2
 * @date 2026-04-02
 *
 * @copyright ROBOPIONEER (c) 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_nvidiaorin.h"

#include "drv_math.h"
#include "drv_uart.h"

#include <stdarg.h>
#include <stdio.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
// UART7 now carries the former UART8 output.
static void UART7_Printf_Orin(const char *format, ...)
{
    char tx_buffer[128];
    va_list args;
    int length;

    if (huart7.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    va_start(args, format);
    length = vsnprintf(tx_buffer, sizeof(tx_buffer), format, args);
    va_end(args);

    if (length <= 0)
    {
        return;
    }

    if (length > (int)sizeof(tx_buffer))
    {
        length = (int)sizeof(tx_buffer);
    }

    HAL_UART_Transmit(&huart7, (uint8_t *)tx_buffer, (uint16_t)length, 20);
}

// 新协议量化范围直接绑定当前底盘控制上限，便于上位机和下位机保持同一标定口径。
static constexpr float ORIN_RX_VELOCITY_MIN = -4.0f;
static constexpr float ORIN_RX_VELOCITY_MAX = 4.0f;
static constexpr float ORIN_RX_OMEGA_MIN = -8.0f;
static constexpr float ORIN_RX_OMEGA_MAX = 8.0f;
static constexpr int32_t ORIN_RX_INT16_MIN = -32768;
static constexpr int32_t ORIN_RX_INT16_MAX = 32767;

static int16_t Orin_Parse_Big_Endian_Int16(const uint8_t *data)
{
    return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]));
}

/* Function prototypes -------------------------------------------------------*/

void Class_Orin::Init(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN1)
    {
        FDCAN_Manage_Object = &FDCAN1_Manage_Object;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        FDCAN_Manage_Object = &FDCAN2_Manage_Object;
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        FDCAN_Manage_Object = &FDCAN3_Manage_Object;
    }
    else
    {
        FDCAN_Manage_Object = nullptr;
    }

    Clear_Rx_Data();
    Tx_Data = Struct_Orin_CAN_Tx_Data();
    Flag = 0;
    Pre_Flag = 0;
    Tx_Sequence = 0;
    Orin_Status = Orin_Status_DISABLE;
}

void Class_Orin::FDCAN_RxCpltCallback(Struct_FDCAN_Rx_Buffer *Rx_Message)
{
    if (Is_Valid_Frame(Rx_Message) == false)
    {
//         // 联调时优先确认是不是标准帧/DLC 被校验挡掉了，联调完成可删。
//         if (Rx_Message != nullptr && Rx_Message->Header.Identifier == CAN_ID_RX_VELOCITY)
//         {
//             UART7_Printf_Orin("[ORIN] invalid id=0x%03lX type=%lu dlc=%lu\r\n",
//                               (unsigned long)Rx_Message->Header.Identifier,
//                               (unsigned long)Rx_Message->Header.IdType,
//                               (unsigned long)Rx_Message->Header.DataLength);
//         }
//         return;
    }

    switch (Rx_Message->Header.Identifier)
    {
        case CAN_ID_RX_VELOCITY:
        {
            // 当前协议固定为单帧 3 x int16_t: Vx| Vy | w | reserve
            const int16_t velocity_x = Orin_Parse_Big_Endian_Int16(&Rx_Message->Data[0]);
            const int16_t velocity_y = Orin_Parse_Big_Endian_Int16(&Rx_Message->Data[2]);
            const int16_t omega = Orin_Parse_Big_Endian_Int16(&Rx_Message->Data[4]);

            Rx_Data.Target_Velocity_X = Math_Int_To_Float(velocity_x,
                                                          ORIN_RX_INT16_MIN,
                                                          ORIN_RX_INT16_MAX,
                                                          ORIN_RX_VELOCITY_MIN,
                                                          ORIN_RX_VELOCITY_MAX);
            Rx_Data.Target_Velocity_Y = Math_Int_To_Float(velocity_y,
                                                          ORIN_RX_INT16_MIN,
                                                          ORIN_RX_INT16_MAX,
                                                          ORIN_RX_VELOCITY_MIN,
                                                          ORIN_RX_VELOCITY_MAX);
            Rx_Data.Target_Omega = Math_Int_To_Float(omega,
                                                     ORIN_RX_INT16_MIN,
                                                     ORIN_RX_INT16_MAX,
                                                     ORIN_RX_OMEGA_MIN,
                                                     ORIN_RX_OMEGA_MAX);
            // 这版协议暂不使用 Yaw/Pitch，先在应用层清零，后续需要时再恢复扩展。
            Rx_Data.Yaw_Target_Omega = 0.0f;
            Rx_Data.Pitch_Target_Omega = 0.0f;

//            // 联调时直接打印原始值和映射结果，便于和上位机发值逐项对照，联调完成可删。
//            UART7_Printf_Orin("[ORIN] raw vx=%d vy=%d w=%d -> vx=%.3f vy=%.3f w=%.3f\r\n",
//                              velocity_x,
//                              velocity_y,
//                              omega,
//                              Rx_Data.Target_Velocity_X,
//                              Rx_Data.Target_Velocity_Y,
//                              Rx_Data.Target_Omega);
            break;
        }
        default:
        {
            return;
        }
    }

    Flag += 1;
    Orin_Status = Orin_Status_ENABLE;
}

void Class_Orin::TIM_101ms_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        Orin_Status = Orin_Status_DISABLE;
        Clear_Rx_Data();
    }
    else
    {
        Orin_Status = Orin_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

void Class_Orin::TIM_10ms_Send_PeriodElapsedCallback()
{
    if (FDCAN_Manage_Object == nullptr)
    {
        return;
    }

    Struct_Orin_CAN_Float2_Frame velocity_frame = {Tx_Data.Now_Velocity_X, Tx_Data.Now_Velocity_Y};
    Struct_Orin_CAN_FloatFlag_Frame omega_frame = {Tx_Data.Now_Omega, Get_Tx_Flag_Mask(), Tx_Sequence};

    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler,
                    CAN_ID_TX_VELOCITY,
                    reinterpret_cast<uint8_t *>(&velocity_frame),
                    FDCAN_ID_Standard,
                    8);

    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler,
                    CAN_ID_TX_OMEGA,
                    reinterpret_cast<uint8_t *>(&omega_frame),
                    FDCAN_ID_Standard,
                    8);

    Tx_Sequence += 1;
}

void Class_Orin::Clear_Rx_Data()
{
    Rx_Data = Struct_Orin_CAN_Rx_Data();
}

void Class_Orin::Parse_Flag_Mask(uint16_t flag_mask)
{
    Rx_Data.Flag_Todo1 = (flag_mask & (1U << 0)) ? 1U : 0U;
    Rx_Data.Flag_Todo2 = (flag_mask & (1U << 1)) ? 1U : 0U;
    Rx_Data.Flag_Todo3 = (flag_mask & (1U << 2)) ? 1U : 0U;
    Rx_Data.Flag_Todo4 = (flag_mask & (1U << 3)) ? 1U : 0U;
    Rx_Data.Flag_Todo5 = (flag_mask & (1U << 4)) ? 1U : 0U;
    Rx_Data.Flag_Todo6 = (flag_mask & (1U << 5)) ? 1U : 0U;
}

uint16_t Class_Orin::Get_Tx_Flag_Mask() const
{
    uint16_t flag_mask = 0;

    flag_mask |= (Tx_Data.Flag_Todo1 ? 1U : 0U) << 0;
    flag_mask |= (Tx_Data.Flag_Todo2 ? 1U : 0U) << 1;
    flag_mask |= (Tx_Data.Flag_Todo3 ? 1U : 0U) << 2;
    flag_mask |= (Tx_Data.Flag_Todo4 ? 1U : 0U) << 3;
    flag_mask |= (Tx_Data.Flag_Todo5 ? 1U : 0U) << 4;
    flag_mask |= (Tx_Data.Flag_Todo6 ? 1U : 0U) << 5;

    return flag_mask;
}

bool Class_Orin::Is_Valid_Frame(const Struct_FDCAN_Rx_Buffer *Rx_Message) const
{
    if (Rx_Message == nullptr)
    {
        return false;
    }

    if (Rx_Message->Header.IdType != FDCAN_STANDARD_ID)
    {
        return false;
    }

    if (Rx_Message->Header.DataLength != FDCAN_DLC_BYTES_8)
    {
        return false;
    }

    return true;
}

/************************ COPYRIGHT(C) ROBOPIONEER **************************/
