/**
 * @file dvc_dr16.h
 */

#ifndef DVC_DR16_H
#define DVC_DR16_H

#include <limits.h>
#include <string.h>
#include "drv_uart.h"

#ifdef __cplusplus

#define SWITCH_UP (1)
#define SWITCH_DOWN (2)
#define SWITCH_MIDDLE (3)

#define KEY_FREE (0)
#define KEY_PRESSED (1)

#define KEY_W 0
#define KEY_S 1
#define KEY_A 2
#define KEY_D 3
#define KEY_SHIFT 4
#define KEY_CTRL 5
#define KEY_Q 6
#define KEY_E 7
#define KEY_R 8
#define KEY_F 9
#define KEY_G 10
#define KEY_Z 11
#define KEY_X 12
#define KEY_C 13
#define KEY_V 14
#define KEY_B 15

#define SBUS_FRAME_LENGTH (25U)
#define SBUS_MAX_CHANNELS (18U)

#define SBUS_CH_RIGHT_X (1U)
#define SBUS_CH_RIGHT_Y (2U)
#define SBUS_CH_LEFT_Y  (3U)
#define SBUS_CH_LEFT_X  (4U)
#define SBUS_CH_YAW     (5U)
#define SBUS_CH_AUX_KNOB_2   (6U)
#define SBUS_CH_RIGHT_SWITCH (9U)

enum Enum_DR16_Status
{
    DR16_Status_DISABLE = 0,
    DR16_Status_ENABLE,
};

enum Enum_DT7_Status
{
    DT7_Status_DISABLE,
    DT7_Status_ENABLE,
};

enum Enum_Image_Status
{
    Image_Status_DISABLE,
    Image_Status_ENABLE,
};

enum Enum_DR16_Updata_Status
{
    DR16_Status_DisUpdata = 0,
    DR16_Status_Updata,
};

enum Enum_DR16_Switch_Status
{
    DR16_Switch_Status_UP = 0,
    DR16_Switch_Status_TRIG_UP_MIDDLE,
    DR16_Switch_Status_TRIG_MIDDLE_UP,
    DR16_Switch_Status_MIDDLE,
    DR16_Switch_Status_TRIG_MIDDLE_DOWN,
    DR16_Switch_Status_TRIG_DOWN_MIDDLE,
    DR16_Switch_Status_DOWN,
};

enum Enum_DR16_Key_Status
{
    DR16_Key_Status_FREE = 0,
    DR16_Key_Status_PRESSED,
    DR16_Key_Status_TRIG_FREE_PRESSED,
    DR16_Key_Status_TRIG_PRESSED_FREE,
};

struct Struct_Image_UART_Data
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0 : 11;
    uint64_t ch_1 : 11;
    uint64_t ch_2 : 11;
    uint64_t ch_3 : 11;
    uint64_t mode_sw : 2;
    uint64_t pause : 1;
    uint64_t fn_1 : 1;
    uint64_t fn_2 : 1;
    uint64_t wheel : 11;
    uint64_t trigger : 1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left : 2;
    uint8_t mouse_right : 2;
    uint8_t mouse_middle : 2;
    uint16_t key;
    uint16_t crc16;
} __attribute__((packed));


struct Struct_DR16_UART_Data
{
    uint16_t Channel_0 = 1500U;
    uint16_t Channel_1 = 1500U;
    uint16_t Channel_2 = 1500U;
    uint16_t Channel_3 = 1500U;
    uint8_t Switch_2 = SWITCH_DOWN;
    uint8_t Switch_1 = SWITCH_MIDDLE;
    int16_t Mouse_X = 0;
    int16_t Mouse_Y = 0;
    int16_t Mouse_Z = 0;
    uint8_t Mouse_Left_Key = KEY_FREE;
    uint8_t Mouse_Right_Key = KEY_FREE;
    uint16_t Keyboard_Key = 0;
    uint16_t Channel_Yaw = 1500U;
};

struct Struct_SBUS_Data
{
    uint16_t Channel[SBUS_MAX_CHANNELS] = {0};
    uint8_t Flags = 0U;
    bool FrameLost = false;
    bool Failsafe = false;
};

struct Struct_DR16_Data
{
    float Right_X = 0.0f;
    float Right_Y = 0.0f;
    float Left_X = 0.0f;
    float Left_Y = 0.0f;
    Enum_DR16_Switch_Status Left_Switch = DR16_Switch_Status_MIDDLE;
    Enum_DR16_Switch_Status Right_Switch = DR16_Switch_Status_DOWN;
    float Mouse_X = 0.0f;
    float Mouse_Y = 0.0f;
    float Mouse_Z = 0.0f;
    Enum_DR16_Key_Status Mouse_Left_Key = DR16_Key_Status_FREE;
    Enum_DR16_Key_Status Mouse_Right_Key = DR16_Key_Status_FREE;
    Enum_DR16_Key_Status Keyboard_Key[16] = {DR16_Key_Status_FREE};
    float Yaw = 0.0f;
};

class Class_DR16
{
public:
    void Init(UART_HandleTypeDef *huart_1);

    inline Enum_DR16_Status Get_DR16_Status();
    inline Enum_DR16_Updata_Status Get_DR16_Updata_Status();
    inline Enum_Image_Status Get_Image_Status() { return (Image_Status); }
    inline Enum_DT7_Status Get_DT7_Status() { return (DT7_Status); }
    inline uint16_t Get_Circle_Index(uint16_t index)
    {
        return (index % UART_Manage_Object_2->Rx_Buffer_Length);
    };
    inline float Get_Right_X();
    inline float Get_Right_Y();
    inline float Get_Left_X();
    inline float Get_Left_Y();
    inline Enum_DR16_Switch_Status Get_Left_Switch();
    inline Enum_DR16_Switch_Status Get_Right_Switch();
    inline float Get_Mouse_X();
    inline float Get_Mouse_Y();
    inline float Get_Mouse_Z();
    inline Enum_DR16_Key_Status Get_Mouse_Left_Key();
    inline Enum_DR16_Key_Status Get_Mouse_Right_Key();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_W();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_S();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_A();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_D();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_Shift();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_Ctrl();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_Q();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_E();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_R();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_F();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_G();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_Z();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_X();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_C();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_V();
    inline Enum_DR16_Key_Status Get_Keyboard_Key_B();
    inline float Get_Yaw();

    void DR16_UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);
    void Image_UART_RxCpltCallback(uint8_t *Rx_Data);

    void TIM1msMod50_Alive_PeriodElapsedCallback();

protected:
    Struct_UART_Manage_Object *UART_Manage_Object_1 = nullptr;
    Struct_UART_Manage_Object *UART_Manage_Object_2 = nullptr;

    float Rocker_Offset = 1500.0f;
    float Rocker_Num = 500.0f;

    Struct_DR16_UART_Data Now_UART_Rx_Data;
    Struct_DR16_UART_Data Pre_UART_Rx_Data;
    Struct_SBUS_Data Now_SBUS_Rx_Data;
    Struct_SBUS_Data Pre_SBUS_Rx_Data;

    Struct_Image_UART_Data Now_UART_Image_Rx_Data;
    Struct_Image_UART_Data Pre_UART_Image_Rx_Data;

    uint32_t DR16_Flag = 0;
    uint32_t Pre_DR16_Flag = 0;
    uint32_t Image_Flag = 0;
    uint32_t Pre_Image_Flag = 0;

    uint16_t Unline_Cnt = 0;
    uint16_t Error_Cnt = 0;
    bool SBUS_Failsafe_Active = false;

    Enum_DR16_Status DR16_Status = DR16_Status_DISABLE;
    Enum_DR16_Updata_Status DR16_Updata_Status = DR16_Status_DisUpdata;
    Enum_Image_Status Image_Status = Image_Status_DISABLE;
    Enum_DT7_Status DT7_Status = DT7_Status_DISABLE;
    Struct_DR16_Data Data;

    void Reset_Control_Data();
    void Judge_Switch(Enum_DR16_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status);
    void Judge_Key(Enum_DR16_Key_Status *Key, uint8_t Status, uint8_t Pre_Status);
    void Judge_Updata(Struct_DR16_UART_Data Pre_UART_Rx_Data, Struct_DR16_UART_Data Now_UART_Rx_Data);
    void DR16_Data_Process();
    void Image_Data_Process(uint8_t *__rx_buffer);
    void Invalidate_Control_Data();
    bool Decode_SBUS_Frame(uint8_t *Rx_Data, uint16_t Length);
    uint16_t Get_Channel_Value(uint8_t channel_index) const;
    bool Is_Channel_Valid(uint16_t value) const;
    float Normalize_Channel(uint16_t value, float sign) const;
    uint8_t Decode_Switch_Channel(uint16_t value, uint8_t fallback_status) const;
};

inline Enum_DR16_Status Class_DR16::Get_DR16_Status()
{
    return (DR16_Status);
}

inline Enum_DR16_Updata_Status Class_DR16::Get_DR16_Updata_Status()
{
    return (DR16_Updata_Status);
}

inline float Class_DR16::Get_Right_X()
{
    return (Data.Right_X);
}

inline float Class_DR16::Get_Right_Y()
{
    return (Data.Right_Y);
}

inline float Class_DR16::Get_Left_X()
{
    return (Data.Left_X);
}

inline float Class_DR16::Get_Left_Y()
{
    return (Data.Left_Y);
}

inline Enum_DR16_Switch_Status Class_DR16::Get_Left_Switch()
{
    return (Data.Left_Switch);
}

inline Enum_DR16_Switch_Status Class_DR16::Get_Right_Switch()
{
    return (Data.Right_Switch);
}

inline float Class_DR16::Get_Mouse_X()
{
    return (Data.Mouse_X);
}

inline float Class_DR16::Get_Mouse_Y()
{
    return (Data.Mouse_Y);
}

inline float Class_DR16::Get_Mouse_Z()
{
    return (Data.Mouse_Z);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Mouse_Left_Key()
{
    return (Data.Mouse_Left_Key);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Mouse_Right_Key()
{
    return (Data.Mouse_Right_Key);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_W()
{
    return (Data.Keyboard_Key[KEY_W]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_S()
{
    return (Data.Keyboard_Key[KEY_S]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_A()
{
    return (Data.Keyboard_Key[KEY_A]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_D()
{
    return (Data.Keyboard_Key[KEY_D]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_Shift()
{
    return (Data.Keyboard_Key[KEY_SHIFT]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_Ctrl()
{
    return (Data.Keyboard_Key[KEY_CTRL]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_Q()
{
    return (Data.Keyboard_Key[KEY_Q]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_E()
{
    return (Data.Keyboard_Key[KEY_E]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_R()
{
    return (Data.Keyboard_Key[KEY_R]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_F()
{
    return (Data.Keyboard_Key[KEY_F]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_G()
{
    return (Data.Keyboard_Key[KEY_G]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_Z()
{
    return (Data.Keyboard_Key[KEY_Z]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_X()
{
    return (Data.Keyboard_Key[KEY_X]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_C()
{
    return (Data.Keyboard_Key[KEY_C]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_V()
{
    return (Data.Keyboard_Key[KEY_V]);
}

inline Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_B()
{
    return (Data.Keyboard_Key[KEY_B]);
}

inline float Class_DR16::Get_Yaw()
{
    return (Data.Yaw);
}

#endif
#endif
