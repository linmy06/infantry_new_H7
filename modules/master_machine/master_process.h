#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define Fake_Referee_Recv_Size 18u // 当前为固定值,36字节
#define Minipc_Send_Size 20u

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;



typedef struct
{
	struct
    {
		uint8_t header;  // 帧头，固定为0x5A
		float yaw;       // 需要云台转动的相对 yaw 角
		float pitch;     // 需要云台转动的相对 pitch 角
		float deep;     // 物体距离
		int16_t match;     //数据
		uint16_t checksum; // 校验和
	}Vision;

	int32_t TCNT;            //时间戳
	int32_t TCNTLast;         //上一次时间戳
	int32_t FailCNT;
	uint8_t FailFlag;
} __attribute__((packed)) Minipc_Recv_s;

typedef enum
{
	COLOR_BLUE = 1,
	COLOR_RED = 0,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2,
} Vision_Work_Mode_e;

typedef struct
{
	struct
	{
		uint8_t header;  // 帧头，固定为0x5A
		uint8_t detect_color;
		float roll;
		float pitch;
		float yaw;
		int32_t match;  // 上位机时间

		uint16_t checksum; // 校验和
	}Vision;

} __attribute__((packed)) Minipc_Send_s;







#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Minipc_Recv_s *minipcInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
void SendMinipcData();

/*更新发送数据帧，并计算发送数据帧长度*/
void get_protocol_send_Vision_data(Minipc_Send_s *tx_data,          // 待发送的float数据
                            uint8_t *tx_buf) ;   // 待发送的数据帧长度



void get_protocol_info_vision(uint8_t *rx_buf, 
                           Minipc_Recv_s *recv_data);

void VisionSetAltitude(uint8_t color);

#endif // !MASTER_PROCESS_H