#include "fake_referee.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "master_process.h"
// #include "robot_def.h"
#include "bsp_usart.h"


static Fake_Referee_Recv_s fake_referee_recv_data;    
static DaemonInstance *fake_referee_daemon_instance;
static USARTInstance *fake_referee_usart_instance;




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
    USARTServiceInit(fake_referee_usart_instance);
    #endif // !VISION_USE_UART
    // ("[vision] vision offline, restart communication.");
}

/*
    此函数用于处理接收数据，
    返回数据内容的id
*/
void get_protocol_info_fake_referee(uint8_t *rx_buf, 
                        Fake_Referee_Recv_s *recv_data)
{

    // if (protocol_header_Check(rx_buf)==1) 
    {
        recv_data->header = rx_buf[0];
        memcpy(&recv_data->detect_color, &rx_buf[1], sizeof(uint8_t));
        memcpy(&recv_data->robot_HP, &rx_buf[2], sizeof(uint16_t));
    }
}

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeMinpc()
{
    DaemonReload(fake_referee_daemon_instance); // 喂狗
    get_protocol_info_fake_referee(fake_referee_usart_instance->recv_buff,&fake_referee_recv_data);
}

Fake_Referee_Recv_s *FakeRefereeInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeMinpc;
    conf.recv_buff_size = Fake_Referee_Recv_Size;
    conf.usart_handle = _handle;
    fake_referee_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = fake_referee_usart_instance,
        .reload_count = 10,
    };
    fake_referee_daemon_instance = DaemonRegister(&daemon_conf);

    return &fake_referee_recv_data;
}