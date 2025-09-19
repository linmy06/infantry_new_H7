#include "daemon.h"
#include "bsp_dwt.h"
#include "stdlib.h"
#include "memory.h"

// 用于保存所有的daemon instance
static DaemonInstance *daemon_instances[DAEMON_MX_CNT] = {NULL};
static uint8_t idx; // 用于记录当前的daemon instance数量,配合回调使用

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    DaemonInstance *instance = (DaemonInstance *)malloc(sizeof(DaemonInstance));
    memset(instance, 0, sizeof(DaemonInstance));

    instance->owner_id = config->owner_id;
    instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count; // 默认值为100
    instance->callback = config->callback;
    instance->temp_count = config->init_count == 0 ? 100 : config->init_count; // 默认值为100,初始计数

    instance->temp_count = config->reload_count;
    daemon_instances[idx++] = instance;
    return instance;
}

/* "喂狗"函数 */
void DaemonReload(DaemonInstance *instance)
{
    instance->temp_count = instance->reload_count;
}



void DaemonTask()
{
    DaemonInstance *dins; // 提高可读性同时降低访存开销
    for (size_t i = 0; i < idx; ++i)
    {

        dins = daemon_instances[i];
        if (dins->temp_count > 0) // 如果计数器还有值,说明上一次喂狗后还没有超时,则计数器减一
            dins->temp_count--;
        else if (dins->callback) // 等于零说明超时了,调用回调函数(如果有的话)
        {
            dins->callback(dins->owner_id); // module内可以将owner_id强制类型转换成自身类型从而调用特定module的offline callback
        }
    }
}