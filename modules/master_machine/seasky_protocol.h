#ifndef __SEASKY_PROTOCOL_H
#define __SEASKY_PROTOCOL_H

#include <stdio.h>
#include <stdint.h>
#include "master_process.h"
#define PROTOCOL_CMD_ID 0x5A
#define SEND_VISION_ID 0xA5

#define OFFSET_BYTE 8 // 出数据段外，其他部分所占字节数

uint8_t protocol_header_Check(uint8_t *rx_buf);



#endif
