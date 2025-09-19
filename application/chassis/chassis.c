//app
#include "chassis.h"
#include "robot_def.h"

//modules
#include "dji_motor.h" //电机
#include "super_cap.h"  //超级电容
#include "message_center.h"  //订阅，发布
#include "general_def.h"    //计算参数

//bsp
#include "bsp_dwt.h"

#include "arm_math.h"

/* 根据robot_def.h中的macro自动计算的参数 */   //宏定义
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */

/******************底盘的双板通信部分************************/
//底盘的发布者和订阅者
static Publisher_t *chassis_pub;     //底盘发布者 //发布(反馈)底盘的反馈信息给cmd
static Subscriber_t *chassis_sub;    //底盘订阅者  /(订阅)cmd的控制信息

//底盘订阅的cmd控制信息和底盘发布的底盘反馈信息
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;    //底盘(订阅)的cmd控制信息
static Chassis_Upload_Data_s chassis_feedback_data;  //底盘(发布)的反馈信息

//超级电容
static SuperCapInstance *cap;     //超级电容实例
static uint16_t power_data;    //超级电容提供的功率

//底盘的姿态解算
static float sin_theta, cos_theta;//麦轮解算用
static float chassis_rotate_buff;    //底盘旋转

//底盘电机
static DJIMotorInstance *motor_lf,*motor_rf,*motor_lb,*motor_rb; //左前，右前，左后，右后电机

//底盘速度
static float chassis_vx,chassis_vy;    //相对于云台坐标系的移动方向(不是底盘的)

//四个电机速度
static float vt_lf,vt_rf,vt_lb,vt_rb;   //左前，右前，左后，右后


//底盘初始化
void ChassisInit()
{
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config = {
            //待定的
            .can_handle = &hfdcan2,    //can的句柄
        },
        .controller_param_init_config = {
            //速度环
            .speed_PID = {
                //待定
                .Kp = 10,
                .Ki = 1,
                .Kd = 0,

                .IntegralLimit = 3000,  //积分限幅
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 120000,   //最大输出
            },
            //电流环
            .current_PID = {
                //待定
                .Kp = 1,
                .Ki = 2,
                .Kd = 0,
                
                .IntegralLimit = 3000,  //积分限幅
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 120000,   //最大输出
            },
        },
        .controller_setting_init_config = {
            //反馈来源
            .angle_feedback_source = MOTOR_FEED, //反馈源来自电机
            .speed_feedback_source = MOTOR_FEED, 
            //闭环类型
            .outer_loop_type = SPEED_LOOP,    //外环：速度环
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,   // 内环：速度环，电流环
        },
        //电机种类
        .motor_type =  M3508,     
    };

    //四个电机的设置   //四个电机的canID，正反转分开控制

    //左前-电机
    chassis_motor_config.can_init_config.tx_id = 3;    //电机can发送的ID
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE; //电机反转
    motor_lf = DJIMotorInit(&chassis_motor_config);    //初始化左前大疆电机
    //左后
    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL; //z正转
    motor_rf = DJIMotorInit(&chassis_motor_config);
    //右前
    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);
    //右1后
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    /*******************************超级电容*******************************/
    SuperCap_Init_Config_s cap_config = {
        .can_config ={
            .can_handle = &hfdcan1,
            .rx_id = 0x311,          //can接收ID
            .tx_id = 0x310,         //发送ID
        },
        //接收和发送数据长度
        .recv_data_len = sizeof(uint8_t),
        .send_data_len = sizeof(uint8_t),
    };
    cap = SuperCapInit(&cap_config);    //初始化超级电容
}

//用于姿态解算
#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)


//底盘状态（模式）的设置
static void ChassisStatueSet()
{
    //底盘无电流模式
    if(chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)  
    {
        //四个电机都停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_rb);
    }
    else
    {
        //底盘四个电机正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_rb); 
    }
}

//超级电容(反馈)的功率数据   
static void SendPowerData()
{
    power_data = chassis_cmd_recv.power_limit;    
}

/**
 * @brief 底盘四个电机每个的姿态解算和输出
 * 
 */
static void MecanumCalculate()
{
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

    //电机在云台坐标轴的移动速度   //运动姿态解算
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    //四个电机的速度    //左前、右前、左后、右后
    vt_lf = chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = chassis_vx - chassis_vy + chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx + chassis_vy + chassis_cmd_recv.wz * RB_CENTER;
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机的参考值
 * 
 */
static void LimitChassisOutput()
{
    //超级电容的电压信息
    if(cap->cap_msg.vol < 24 && cap->cap_msg.vol > 13)    //超级电容电压    13~24V
    {
        chassis_feedback_data.power_flag = 1;     //功能标志位
    }
    else
    {
        chassis_feedback_data.power_flag = 0;
    }
    
    //功率限制后电机的参数设置
    DJIMotorSetRef(motor_lf,vt_lf);
    DJIMotorSetRef(motor_lb,vt_lb);
    DJIMotorSetRef(motor_rf,vt_rf);
    DJIMotorSetRef(motor_rb,vt_rb);
}



/*机器人底盘核心任务*/
void ChassisTask()
{
     //底盘订阅cmd的控制信息
    SubGetMessage(chassis_sub,(void *)&chassis_cmd_recv);   

     //底盘状态的设置
    ChassisStatueSet();  

    //底盘电机的姿态解算
    MecanumCalculate();

    //底盘超级电容的输出限制
    LimitChassisOutput();

    //发布底盘的反馈信息
    PubPushMessage(chassis_pub,(void *)&chassis_feedback_data);

    //超级电容反馈的功率数据
    SendPowerData();

    //发送给超级电容的控制cmd信息
    SuperCapSend(cap,(uint8_t *)power_data);
}


