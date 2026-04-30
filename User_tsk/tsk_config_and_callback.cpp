#include "tsk_config_and_callback.h"
#include "ita_robot.h"
#include "drv_tim.h"
#include "dvc_dwt.h"
#include "drv_bsp.h"

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
    // M2006 电机数据反馈
    case 0x201:
    {
        chariot.Chassis.Motor_Wheel[0].FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
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
 * @brief UART5遥控器回调函数
 *
 * @param Buffer UART5收到的消息
 * @param Length 长度
 */
void DR16_UART5_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.DR16.DR16_UART_RxCpltCallback(Buffer);
    // 底盘 云台 发射机构 的控制策略
    chariot.TIM_Control_Callback();
}

/**
 * @brief TIM5任务回调函数
 *
 */
uint8_t mod100 = 0;

void Task1ms_TIM5_Callback()
{
    DWT_Update();
    flag++;

    // 10ms检测存活状态
    mod100++;
    if (mod100 >= 100)
    {
        chariot.TIM_100ms_Alive_PeriodElapsedCallback();

        mod100 = 0;
    }
    //  Weapon_Grab.Motor_Rotate.CAN_Send_Set_Rx_ID(0xF1);
    //  Weapon_Grab.Motor_Rotate.CAN_Send_Set_Tx_ID(0x71);

    // FDCAN_Send_Data(&hfdcan1, 0x01, data, FDCAN_ID_Standard, 8);
    chariot.TIM_Calculate_PeriodElapsedCallback();
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

    // UART初始化
    UART_Init(&huart5, DR16_UART5_Callback, 36);
    // 定时器初始化
    TIM_Init(&htim5, Task1ms_TIM5_Callback);
    chariot.Init(0.03);

    // 外部中断初始化(舵轮光电门校准)

    // 设备层初始化

    // 战车层初始化

    // 交互层初始化

    // 机器人战车初始化

    // 使能调度时钟
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