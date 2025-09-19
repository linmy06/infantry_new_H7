#ifndef FAKE_REFEREE_H
#define FAKE_REFEREE_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define Fake_Referee_Recv_Size 4u // 当前为固定值,36字节  //接收

#pragma pack(1)

typedef struct
{
    uint8_t header;
    uint8_t detect_color;
    uint16_t robot_HP;
} __attribute__((packed)) Fake_Referee_Recv_s;   //假裁判系统接收





#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Fake_Referee_Recv_s *FakeRefereeInit(UART_HandleTypeDef *_handle);


void get_protocol_info_fake_referee(uint8_t *rx_buf, 
                           Fake_Referee_Recv_s *recv_data);


#endif