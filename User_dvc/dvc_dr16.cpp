/**
 * @file dvc_dr16.cpp
 */

#include "dvc_dr16.h"
#include "tsk_config_and_callback.h"
#include "drv_math.h"

namespace
{
constexpr uint8_t SBUS_FRAME_HEADER = 0x0FU;
constexpr uint8_t SBUS_FLAGS_INDEX = 23U;
constexpr uint8_t SBUS_FRAMELOST_MASK = 0x04U;
constexpr uint8_t SBUS_FAILSAFE_MASK = 0x08U;

constexpr uint16_t SBUS_CHANNEL_MIN = 172U;
constexpr uint16_t SBUS_CHANNEL_MID = 992U;
constexpr uint16_t SBUS_CHANNEL_MAX = 1811U;
constexpr uint16_t SBUS_CHANNEL_ABSOLUTE_MAX = 2047U;
constexpr uint16_t SBUS_SWITCH_LOW_THRESHOLD = 700U;
constexpr uint16_t SBUS_SWITCH_HIGH_THRESHOLD = 1300U;

constexpr float SBUS_SIGN_RIGHT_X = 1.0f;
constexpr float SBUS_SIGN_RIGHT_Y = 1.0f;
constexpr float SBUS_SIGN_LEFT_X = 1.0f;
constexpr float SBUS_SIGN_LEFT_Y = 1.0f;
constexpr float SBUS_SIGN_YAW = 1.0f;

bool Is_Likely_SBUS_Footer(uint8_t footer)
{
    return (footer == 0x00U) || ((footer & 0x0FU) == 0x04U);
}

uint16_t Extract_SBUS_Channel(const uint8_t *frame, uint8_t channel_index)
{
    uint32_t bit_index = static_cast<uint32_t>(channel_index) * 11U;
    uint32_t byte_index = 1U + (bit_index / 8U);
    uint32_t shift = bit_index % 8U;
    uint32_t packed = static_cast<uint32_t>(frame[byte_index]) |
                      (static_cast<uint32_t>(frame[byte_index + 1U]) << 8U) |
                      (static_cast<uint32_t>(frame[byte_index + 2U]) << 16U);

    return static_cast<uint16_t>((packed >> shift) & 0x07FFU);
}
}

void Class_DR16::Init(UART_HandleTypeDef *huart_1)
{
    if (huart_1->Instance == USART1)
    {
        UART_Manage_Object_1 = &UART1_Manage_Object;
    }
    else if (huart_1->Instance == USART2)
    {
        UART_Manage_Object_1 = &UART2_Manage_Object;
    }
    else if (huart_1->Instance == USART3)
    {
        UART_Manage_Object_1 = &UART3_Manage_Object;
    }
    else if (huart_1->Instance == UART4)
    {
        UART_Manage_Object_1 = &UART4_Manage_Object;
    }
    else if (huart_1->Instance == UART5)
    {
        UART_Manage_Object_1 = &UART5_Manage_Object;
    }
    else if (huart_1->Instance == USART6)
    {
        UART_Manage_Object_1 = &UART6_Manage_Object;
    }

    Rocker_Offset = static_cast<float>(SBUS_CHANNEL_MID);
    Rocker_Num = static_cast<float>(SBUS_CHANNEL_MAX - SBUS_CHANNEL_MID);
    Reset_Control_Data();
}

void Class_DR16::Reset_Control_Data()
{
    memset(&Now_SBUS_Rx_Data, 0, sizeof(Now_SBUS_Rx_Data));
    memset(&Pre_SBUS_Rx_Data, 0, sizeof(Pre_SBUS_Rx_Data));

    Now_UART_Rx_Data = Struct_DR16_UART_Data();
    Pre_UART_Rx_Data = Struct_DR16_UART_Data();
    Data = Struct_DR16_Data();
    SBUS_Failsafe_Active = false;
}

void Class_DR16::Invalidate_Control_Data()
{
    Now_UART_Rx_Data = Struct_DR16_UART_Data();
    Data = Struct_DR16_Data();
    DR16_Updata_Status = DR16_Status_DisUpdata;
}

bool Class_DR16::Is_Channel_Valid(uint16_t value) const
{
    return (value <= SBUS_CHANNEL_ABSOLUTE_MAX);
}

uint16_t Class_DR16::Get_Channel_Value(uint8_t channel_index) const
{
    if (channel_index == 0U || channel_index > SBUS_MAX_CHANNELS)
    {
        return SBUS_CHANNEL_MID;
    }

    uint16_t value = Now_SBUS_Rx_Data.Channel[channel_index - 1U];
    if (Is_Channel_Valid(value) == false)
    {
        return SBUS_CHANNEL_MID;
    }

    return value;
}

float Class_DR16::Normalize_Channel(uint16_t value, float sign) const
{
    float output = (static_cast<float>(value) - Rocker_Offset) / Rocker_Num;
    Math_Constrain(&output, -1.0f, 1.0f);
    return (output * sign);
}

uint8_t Class_DR16::Decode_Switch_Channel(uint16_t value, uint8_t fallback_status) const
{
    if (Is_Channel_Valid(value) == false)
    {
        return fallback_status;
    }

    if (value >= SBUS_SWITCH_HIGH_THRESHOLD)
    {
        return SWITCH_UP;
    }
    if (value <= SBUS_SWITCH_LOW_THRESHOLD)
    {
        return SWITCH_DOWN;
    }

    return SWITCH_MIDDLE;
}

void Class_DR16::Judge_Switch(Enum_DR16_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status)
{
    switch (Pre_Status)
    {
    case SWITCH_UP:
        switch (Status)
        {
        case SWITCH_UP:
            *Switch = DR16_Switch_Status_UP;
            break;
        case SWITCH_DOWN:
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;
            break;
        case SWITCH_MIDDLE:
            *Switch = DR16_Switch_Status_TRIG_UP_MIDDLE;
            break;
        }
        break;

    case SWITCH_DOWN:
        switch (Status)
        {
        case SWITCH_UP:
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_UP;
            break;
        case SWITCH_DOWN:
            *Switch = DR16_Switch_Status_DOWN;
            break;
        case SWITCH_MIDDLE:
            *Switch = DR16_Switch_Status_TRIG_DOWN_MIDDLE;
            break;
        }
        break;

    case SWITCH_MIDDLE:
    default:
        switch (Status)
        {
        case SWITCH_UP:
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_UP;
            break;
        case SWITCH_DOWN:
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;
            break;
        case SWITCH_MIDDLE:
        default:
            *Switch = DR16_Switch_Status_MIDDLE;
            break;
        }
        break;
    }
}

void Class_DR16::Judge_Key(Enum_DR16_Key_Status *Key, uint8_t Status, uint8_t Pre_Status)
{
    switch (Pre_Status)
    {
    case KEY_FREE:
        *Key = (Status == KEY_PRESSED) ? DR16_Key_Status_TRIG_FREE_PRESSED : DR16_Key_Status_FREE;
        break;
    case KEY_PRESSED:
    default:
        *Key = (Status == KEY_FREE) ? DR16_Key_Status_TRIG_PRESSED_FREE : DR16_Key_Status_PRESSED;
        break;
    }
}

void Class_DR16::Judge_Updata(Struct_DR16_UART_Data Pre_Data, Struct_DR16_UART_Data Now_Data)
{
    if (Pre_Data.Channel_0 == Now_Data.Channel_0 &&
        Pre_Data.Channel_1 == Now_Data.Channel_1 &&
        Pre_Data.Channel_2 == Now_Data.Channel_2 &&
        Pre_Data.Channel_3 == Now_Data.Channel_3 &&
        Pre_Data.Channel_Yaw == Now_Data.Channel_Yaw &&
        Pre_Data.Switch_1 == Now_Data.Switch_1 &&
        Pre_Data.Switch_2 == Now_Data.Switch_2)
    {
        DR16_Updata_Status = DR16_Status_DisUpdata;
    }
    else
    {
        DR16_Updata_Status = DR16_Status_Updata;
    }
}

bool Class_DR16::Decode_SBUS_Frame(uint8_t *Rx_Data, uint16_t Length)
{
    if (Length < SBUS_FRAME_LENGTH)
    {
        return false;
    }

    int latest_valid_offset = -1;

    for (uint16_t offset = 0; offset + SBUS_FRAME_LENGTH <= Length; offset++)
    {
        uint8_t *frame = &Rx_Data[offset];
        if (frame[0] != SBUS_FRAME_HEADER)
        {
            continue;
        }
        if (Is_Likely_SBUS_Footer(frame[24]) == false)
        {
            continue;
        }

        latest_valid_offset = static_cast<int>(offset);
    }

    if (latest_valid_offset < 0)
    {
        return false;
    }

    uint8_t *frame = &Rx_Data[latest_valid_offset];
    for (uint8_t i = 0; i < 16U; i++)
    {
        Now_SBUS_Rx_Data.Channel[i] = Extract_SBUS_Channel(frame, i);
    }
    Now_SBUS_Rx_Data.Flags = frame[SBUS_FLAGS_INDEX];
    Now_SBUS_Rx_Data.Channel[16] = (Now_SBUS_Rx_Data.Flags & 0x01U) ? SBUS_CHANNEL_ABSOLUTE_MAX : 0U;
    Now_SBUS_Rx_Data.Channel[17] = (Now_SBUS_Rx_Data.Flags & 0x02U) ? SBUS_CHANNEL_ABSOLUTE_MAX : 0U;
    Now_SBUS_Rx_Data.FrameLost = ((Now_SBUS_Rx_Data.Flags & SBUS_FRAMELOST_MASK) != 0U);
    Now_SBUS_Rx_Data.Failsafe = ((Now_SBUS_Rx_Data.Flags & SBUS_FAILSAFE_MASK) != 0U);

    return true;
}

void Class_DR16::DR16_Data_Process()
{
    Now_UART_Rx_Data.Channel_0 = Get_Channel_Value(SBUS_CH_RIGHT_X);
    Now_UART_Rx_Data.Channel_1 = Get_Channel_Value(SBUS_CH_RIGHT_Y);
    Now_UART_Rx_Data.Channel_2 = Get_Channel_Value(SBUS_CH_LEFT_X);
    Now_UART_Rx_Data.Channel_3 = Get_Channel_Value(SBUS_CH_LEFT_Y);
    Now_UART_Rx_Data.Channel_Yaw = Get_Channel_Value(SBUS_CH_YAW);
    uint16_t right_switch_raw = Get_Channel_Value(SBUS_CH_RIGHT_SWITCH);

    // FS-i6X default layout uses CH6 as the second knob (VrB), not a 3-state switch.
    Now_UART_Rx_Data.Switch_1 = SWITCH_MIDDLE;
    Now_UART_Rx_Data.Switch_2 = Decode_Switch_Channel(right_switch_raw, Pre_UART_Rx_Data.Switch_2);

    Data.Right_X = Normalize_Channel(Now_UART_Rx_Data.Channel_0, SBUS_SIGN_RIGHT_X);
    Data.Right_Y = Normalize_Channel(Now_UART_Rx_Data.Channel_1, SBUS_SIGN_RIGHT_Y);
    Data.Left_X = Normalize_Channel(Now_UART_Rx_Data.Channel_2, SBUS_SIGN_LEFT_X);
    Data.Left_Y = Normalize_Channel(Now_UART_Rx_Data.Channel_3, SBUS_SIGN_LEFT_Y);
    Data.Yaw = Normalize_Channel(Now_UART_Rx_Data.Channel_Yaw, SBUS_SIGN_YAW);

    Judge_Switch(&Data.Left_Switch, Now_UART_Rx_Data.Switch_1, Pre_UART_Rx_Data.Switch_1);
    Judge_Switch(&Data.Right_Switch, Now_UART_Rx_Data.Switch_2, Pre_UART_Rx_Data.Switch_2);

    Data.Mouse_X = 0.0f;
    Data.Mouse_Y = 0.0f;
    Data.Mouse_Z = 0.0f;
    Data.Mouse_Left_Key = DR16_Key_Status_FREE;
    Data.Mouse_Right_Key = DR16_Key_Status_FREE;
    for (int i = 0; i < 16; i++)
    {
        Data.Keyboard_Key[i] = DR16_Key_Status_FREE;
    }

    Judge_Updata(Pre_UART_Rx_Data, Now_UART_Rx_Data);
}

void Class_DR16::Image_Data_Process(uint8_t *__rx_buffer)
{
    memcpy(&Now_UART_Image_Rx_Data, __rx_buffer, sizeof(Struct_Image_UART_Data));
    Struct_Image_UART_Data *tmp_buffer = (Struct_Image_UART_Data *)__rx_buffer;

    Data.Mouse_X = tmp_buffer->mouse_x / 32768.0f;
    Data.Mouse_Y = tmp_buffer->mouse_y / 32768.0f;
    Data.Mouse_Z = tmp_buffer->mouse_z / 32768.0f;

    Judge_Key(&Data.Mouse_Left_Key, tmp_buffer->mouse_left, Pre_UART_Image_Rx_Data.mouse_left);
    Judge_Key(&Data.Mouse_Right_Key, tmp_buffer->mouse_right, Pre_UART_Image_Rx_Data.mouse_right);

    for (int i = 0; i < 16; i++)
    {
        Judge_Key(&Data.Keyboard_Key[i],
                  (tmp_buffer->key >> i) & 0x1U,
                  (Pre_UART_Image_Rx_Data.key >> i) & 0x1U);
    }
}

void Class_DR16::DR16_UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    if (Decode_SBUS_Frame(Rx_Data, Length) == false)
    {
        Error_Cnt++;
        return;
    }

    if (Now_SBUS_Rx_Data.FrameLost == true || Now_SBUS_Rx_Data.Failsafe == true)
    {
        SBUS_Failsafe_Active = true;
        DR16_Status = DR16_Status_DISABLE;
        DT7_Status = DT7_Status_DISABLE;
        Invalidate_Control_Data();
        Pre_SBUS_Rx_Data = Now_SBUS_Rx_Data;
        Pre_UART_Rx_Data = Now_UART_Rx_Data;
        return;
    }

    SBUS_Failsafe_Active = false;
    DR16_Flag += 1U;
    DR16_Data_Process();

    Pre_SBUS_Rx_Data = Now_SBUS_Rx_Data;
    Pre_UART_Rx_Data = Now_UART_Rx_Data;
}

void Class_DR16::Image_UART_RxCpltCallback(uint8_t *Rx_Data)
{
    if (Rx_Data[0] == 0xA5U)
    {
        uint16_t cmd_id = static_cast<uint16_t>(Rx_Data[6]) << 8U;
        uint16_t data_length = static_cast<uint16_t>(Rx_Data[2]) << 8U;

        cmd_id |= Rx_Data[5];
        data_length |= Rx_Data[1];

        if (cmd_id == 0x0302U && data_length == 30U)
        {
            // Reserved for future image link support.
        }
    }
}

void Class_DR16::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    if (SBUS_Failsafe_Active == true)
    {
        DR16_Status = DR16_Status_DISABLE;
        DT7_Status = DT7_Status_DISABLE;
    }
    else if (DR16_Flag == Pre_DR16_Flag && Image_Flag == Pre_Image_Flag)
    {
        DR16_Status = DR16_Status_DISABLE;
        Unline_Cnt++;
    }
    else
    {
        DR16_Status = DR16_Status_ENABLE;
    }

    if (Image_Flag == Pre_Image_Flag)
    {
        Image_Status = Image_Status_DISABLE;
    }
    else
    {
        Image_Status = Image_Status_ENABLE;
    }

    if (SBUS_Failsafe_Active == true)
    {
        DT7_Status = DT7_Status_DISABLE;
    }
    else if (DR16_Flag == Pre_DR16_Flag)
    {
        DT7_Status = DT7_Status_DISABLE;
    }
    else
    {
        DT7_Status = DT7_Status_ENABLE;
    }

    Pre_DR16_Flag = DR16_Flag;
    Pre_Image_Flag = Image_Flag;
}
