//app
#include "shoot.h"     //发射
#include "robot_def.h"


//modules
#include "dji_motor.h"    //大疆电机
#include "message_center.h"   
#include "bsp_dwt.h"
#include "general_def.h" 

/********************双板通信部分*****************************/
//发布者和订阅者
static Publisher_t *shoot_pub;     //发布射击结构的反馈信息
static Subscriber_t *shoot_sub;     //订阅发射的cmd控制信息

static Shoot_Upload_Data_s shoot_feedback_data;  //射击反馈的信息
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;   //射击接收的cmd控制信息

//左摩擦轮，右摩擦轮，拨盘电机
static DJIMotorInstance *friction_l,*friction_r;  //左摩擦轮，右摩擦轮  //发射电机
static DJIMotorInstance *loader;   //拨盘电机  //装弹电机

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;


//云台发射的初始化
void ShootInit()
{
    //摩擦轮电机   //控制发射
    Motor_Init_Config_s friction_config = {
        .can_init_config ={
            .can_handle = &hfdcan2,   //can的句柄
        },
        .controller_param_init_config = {
            //速度环
            .speed_PID = {
                .Kp = 20,
                .Ki = 1,
                .Kd = 0,

                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            //电流环
            .current_PID = {
                .Kp = 0.7,
                .Ki = 0.1,
                .Kd = 0,

                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            //反馈来源
            .angle_feedback_source = MOTOR_FEED,   //反馈来自电机
            .speed_feedback_source = MOTOR_FEED,
            
            //闭环控制类型
            .outer_loop_type  = SPEED_LOOP,  //外环速度环
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,  //内环 速度环 电流环 
        },
        .motor_type = M3508,     //控制电机种类
    };

    //左摩擦轮
    friction_config.can_init_config.tx_id = 2;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;  //电机正转
    friction_l = DJIMotorInit(&friction_config);

    //右摩擦轮
    friction_config.can_init_config.tx_id = 1;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;  //电机反转
    friction_r = DJIMotorInit(&friction_config);


    //拨盘电机   //装弹电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,   //句柄can2
        },
        .controller_param_init_config = {
            //速度环
            .speed_PID = {
                .Kp = 10,
                .Ki = 1,
                .Kd = 0,

                .IntegralLimit = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 10000,
            },
            //电流环
            .current_PID = {
                .Kp = 0.7,
                .Ki = 1,
                .Kd = 0,

                .IntegralLimit = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 10000,
            },
        },
        .controller_setting_init_config = {
            //反馈来源
            .angle_feedback_source = MOTOR_FEED,    //电机反馈
            .speed_feedback_source = MOTOR_FEED,   

            //闭环类型
            .outer_loop_type = SPEED_LOOP,     //外环速度环
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,   //内环 电流和速度环

            //拨盘电机的正反转     //只有一个拨盘电机故可直接在内赋值
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,   // 正转
        },
        .motor_type = M2006,   //电机种类
    };

    //拨盘电机的设置
    loader_config.can_init_config.tx_id = 3;    //canID
    loader = DJIMotorInit(&loader_config);

    //注册射击的发布者和订阅者
    shoot_pub = PubRegister("shoot_feed",sizeof(Shoot_Upload_Data_s));  //注册射击发布者 //发布射击的反馈信息
    shoot_sub = SubRegister("shoot_cmd",sizeof(Shoot_Ctrl_Cmd_s));     //注册射击订阅者 //订阅射击的cmd控制信息
}

//射击的设置      //射击的开启/停止
static void ShootStatueSet()
{
    //射击的模式控制   //根据接收的cmd控制信息

    //关闭射击
    if(shoot_cmd_recv.shoot_mode == SHOOT_OFF)  
    {
        //关闭左摩擦轮，右摩擦轮，拨盘电机
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else
    {
        //使能左摩擦轮，右摩擦轮，拨盘电机
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }
}

//发射的装弹电机电机模式的设置    //拨盘电机,射频设置
static void ShootRateSet()
{
    //根据cmd发送来的控制模式切换模式     
    switch(shoot_cmd_recv.loader_mode)
    {
    //拨盘停止
    case LOAD_STOP:
    {
        DJIMotorOuterLoop(loader,SPEED_LOOP);   //外环-速度环
        DJIMotorSetRef(loader,0);   //设置电机的转速
        break;
    }
    //连发模式，对速度环闭环，射频可变，目前为1HZ
    case LOAD_BURSTFIRE:
    {
        DJIMotorOuterLoop(loader,SPEED_LOOP);  //外环-速度环
        //设置拨盘电机的转速
        DJIMotorSetRef(loader,shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度
        break;
    }
    //拨盘反转，对速度环闭环
    case LOAD_REVERSE:
    {
        //拨盘的外环设为-速度环
        DJIMotorOuterLoop(loader,SPEED_LOOP);
        DJIMotorSetRef(loader, -1000);   //电机反转转速参考值
        break;
    }
    default:
    // while(1)
    // ;     //未知模式，停止运行1，检测指针越界，内存块溢出等问题
    break;
    }
}

//射击-摩擦轮电机转速设置    //射速设置
static void ShootSpeedSet()
{
    //摩擦轮接收的控制模式
    if(shoot_cmd_recv.friction_mode == FRICTION_ON)   //摩擦轮开启
    {
        //设置摩擦轮转速
        DJIMotorSetRef(friction_l,3000);    //左摩擦轮
        DJIMotorSetRef(friction_r,3000);    //右摩擦轮
    }
    else   //关闭摩擦轮
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
    }
}

//发送射击部分的反馈信息
static void SendShootData()
{
    shoot_feedback_data.loader_speed_aps = loader->measure.speed_aps;   //反馈摩擦轮测量到的速度
}

//云台发射的调动任务
void ShootTask()
{
    //射击在cmd中获取（订阅）的控制信息
    SubGetMessage(shoot_sub,(void *)&shoot_cmd_recv);

    //发射的设置    :开启/停止
    ShootStatueSet();

    //拨盘电机 ：射频的设定
    ShootRateSet();

    //摩擦轮电机  ：射速的设定
    ShootSpeedSet();

    //射击部件的(反馈)信息
    SendShootData();

    //(反馈)射击的数据     //反馈射击的数据给cmd控制系统  //用于卡弹反馈(后续加上模块离线)
    PubPushMessage(shoot_pub,(void *)&shoot_feedback_data);
}
