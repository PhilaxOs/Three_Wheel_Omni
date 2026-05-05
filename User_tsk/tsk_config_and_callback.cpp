#include "tsk_config_and_callback.h"
#include "ita_robot.h"
#include "drv_tim.h"
#include "dvc_dwt.h"
#include "drv_bsp.h"
#include "drv_can.h"

Class_Chariot chariot;
uint32_t flag = 0;
bool init_finished = false;

/**
 * @brief FDCAN1回调函数
 *
 * @param FDCAN_RxMessage FDCAN1收到的消息
 */
uint8_t data[8] = {0};
void Device_FDCAN1_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage)
{

    switch (FDCAN_RxMessage->Header.Identifier)
    {
    // M3508 电机数据反馈
    case 0x201:
    {
        chariot.Chassis.Motor_Wheel[0].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
        break;
    }
    case 0x202:
    {
        chariot.Chassis.Motor_Wheel[1].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
        break;
    }
    case 0x203:
    {
        chariot.Chassis.Motor_Wheel[2].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
        break;
    }

    default:

        break;
    }
}

/**
 * @brief FDCAN2回调函数
 *
 * @param FDCAN_RxMessage FDCAN2收到的消息  
 */
void Device_FDCAN2_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage)
{
    switch (FDCAN_RxMessage->Header.Identifier)
    {
        case Class_Orin::CAN_ID_RX_VELOCITY://现在只看0x310
        {
            chariot.Orin.FDCAN_RxCpltCallback(FDCAN_RxMessage);
            break;
        }
        default:
        {
            break;
        }
    }
}

/**
 * @brief UART5遥控器回调函数
 *
 * @param Buffer UART5收到的消息
 * @param Length 长度
 */
void DR16_UART5_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.DR16.DR16_UART_RxCpltCallback(Buffer,Length);
    // 底盘 云台 发射机构 的控制策略
    chariot.TIM_Control_Callback();
}

/**
 * @brief TIM5任务回调函数
 *
 */
void Task10ms_TIM4_Callback()
{
    if (chariot.DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
    {
        chariot.TIM_Control_Callback();// 右拨杆在UP时，再刷新一次底盘控制决策
    }

    chariot.Orin.Set_Now_Velocity_X(chariot.Chassis.Get_Now_Velocity_X());
    chariot.Orin.Set_Now_Velocity_Y(chariot.Chassis.Get_Now_Velocity_Y());
    chariot.Orin.Set_Now_Omega(chariot.Chassis.Get_Now_Omega());
    chariot.Orin.TIM_10ms_Send_PeriodElapsedCallback();//周期性把当前底盘状态发回 Orin

}

/**
 * @brief TIM5任务回调函数
 *
 */
uint8_t mod100 = 0;
uint8_t mod101 = 0;

void Task1ms_TIM5_Callback()
{
    DWT_Update();
    flag++;

    // 10ms检测存活状态
    mod100++;
    if (mod100 >= 100)
    {
        chariot.TIM_100ms_Alive_PeriodElapsedCallback(); // DR16存活检测,底盘存活检测

        mod100 = 0;
    }
    mod101++;
    if (mod101 >= 101)
    {
        chariot.TIM_101ms_Alive_PeriodElapsedCallback(); // orin存活检测

        mod101 = 0;
    }
    chariot.TIM_Unline_Protect_PeriodElapsedCallback();
    chariot.TIM_Calculate_PeriodElapsedCallback();
    TIM_1ms_CAN_PeriodElapsedCallback();
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    // 驱动层初始化
    DWT_Init();
    // 点俩灯, 开24V
    BSP_Init(BSP_DC24_L_OFF | BSP_DC24_R_OFF | BSP_DC5_ON, 0.0, 0.0);
    // CAN总线初始化
    FDCAN_Init(&hfdcan1, Device_FDCAN1_Callback);
    FDCAN_Init(&hfdcan2, Device_FDCAN2_Callback);

    // UART初始化
    UART_Init(&huart5, DR16_UART5_Callback, 64);
    // 定时器初始化
    TIM_Init(&htim4, Task10ms_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);
    chariot.Init(0.03);

    // 外部中断初始化(舵轮光电门校准)

    // 设备层初始化

    // 战车层初始化

    // 交互层初始化

    // 机器人战车初始化

    // 使能调度时钟
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    // 标记初始化完成
    init_finished = true;
}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{
}