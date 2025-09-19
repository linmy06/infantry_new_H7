#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "robot_def.h"
#include "bsp_usart.h"

static Minipc_Recv_s minipc_recv_data;
static Minipc_Send_s minipc_send_data;
static DaemonInstance *minipc_daemon_instance;
static USARTInstance *minipc_usart_instance;

#define Minipc_Failed_Count 1000       //重启计时？


void VisionSetFlag(uint8_t color)
{
    minipc_send_data.Vision.detect_color=color;
}


void VisionSetAltitude(uint8_t color)
{
    minipc_send_data.Vision.detect_color = color;   
    minipc_send_data.Vision.pitch = QEKF_INS.Pitch;
    minipc_send_data.Vision.roll = QEKF_INS.Roll;
    minipc_send_data.Vision.yaw = QEKF_INS.Yaw;
    minipc_send_data.Vision.match = minipc_recv_data.TCNTLast;
}


/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(minipc_usart_instance);
#endif // !VISION_USE_UART
}


/*
    此函数用于处理接收数据，
    返回数据内容的id
*/
void get_protocol_info_vision(uint8_t *rx_buf, 
                        Minipc_Recv_s *recv_data)
{

    if (protocol_header_Check(rx_buf)==1) 
    {
            recv_data->Vision.header = rx_buf[0];
            memcpy(&recv_data->Vision.yaw, &rx_buf[1], sizeof(float));
            memcpy(&recv_data->Vision.pitch, &rx_buf[5], sizeof(float));
            memcpy(&recv_data->Vision.deep, &rx_buf[9], sizeof(uint8_t));
            memcpy(&recv_data->Vision.match, &rx_buf[10], sizeof(int32_t));
            recv_data->Vision.checksum = (rx_buf[14] << 8) | rx_buf[15];
    }
       
}


/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeMinpc()
{
    DaemonReload(minipc_daemon_instance); // 喂狗
    get_protocol_info_vision(minipc_usart_instance->recv_buff,&minipc_recv_data);
}

Minipc_Recv_s *minipcInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeMinpc;
    conf.recv_buff_size = Fake_Referee_Recv_Size;
    conf.usart_handle = _handle;
    minipc_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = minipc_usart_instance,
        .reload_count = 10,
    };
    minipc_daemon_instance = DaemonRegister(&daemon_conf);

    return &minipc_recv_data;
}

/*
    此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
    后续调用通信接口的发送函数发送tx_buf中的对应数据
*/
void get_protocol_send_Vision_data(Minipc_Send_s *tx_data,          // 待发送的float数据
                            uint8_t *tx_buf)    // 待发送的数据帧长度
{
    /*帧头部分*/
    tx_buf[0] = SEND_VISION_ID;
    /*数据段*/
    tx_buf[1] =tx_data->Vision.detect_color;
    memcpy(&tx_buf[2], &tx_data->Vision.roll, sizeof(float));
    memcpy(&tx_buf[6], &tx_data->Vision.pitch, sizeof(float));
    memcpy(&tx_buf[10], &tx_data->Vision.yaw, sizeof(float));  
    memcpy(&tx_buf[14], &tx_data->Vision.match, sizeof(int32_t));
}





/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void SendMinipcData()
{
    uint32_t TCNT = 0;   //时间戳
    TCNT = minipc_recv_data.Vision.match;     //获取小电脑接收的数据  
    //TCNT丢失时会一直等于minipc_recv_data.TCNTLast，所以FailCNT会一直加。
    if(TCNT != minipc_recv_data.TCNTLast  )       
    {
        minipc_recv_data.TCNTLast = TCNT; // 记录上次接收的时间戳
        minipc_recv_data.FailCNT = 0;
        minipc_recv_data.FailFlag = 0;
    }
    else if(minipc_recv_data.FailFlag != 1)
    {
        minipc_recv_data.FailCNT +=1;
    }
    //超时
    if(minipc_recv_data.FailCNT > Minipc_Failed_Count)  
    {
        minipc_recv_data.FailCNT =0;
        minipc_recv_data.FailFlag =1;
    }

    static uint8_t send_buff[Minipc_Send_Size];    //发送数据
    get_protocol_send_Vision_data( &minipc_send_data,send_buff);
    VisionSetAltitude(0);
    USARTSend(minipc_usart_instance, send_buff, Minipc_Send_Size, USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突

}
