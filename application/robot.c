#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"

#include "robot_cmd.h"
// #include "arm.h"   //机械臂的

void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit();
    RobotCMDInit();
    // ArmInit();   //机械臂
    
    // ChassisInit();
    //GimbalInit();
    //ShootInit();


    OSTaskInit(); // 创建基础任务

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
  RobotCMDTask();
  // ArmTask();
}