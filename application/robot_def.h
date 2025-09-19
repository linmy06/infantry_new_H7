#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H




#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车
#define FAKE_REFEREE

#define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 4140  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 1 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 0.82          // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -0.82           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 49.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 350              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 300             // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 60             // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,   //停止
    ROBOT_READY,      
} Robot_Status_e;     

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,   //关闭
    APP_ONLINE,       //在线
    APP_ERROR,          //报错
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;

//摩擦轮模式
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;     

//发射状态
typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

//自瞄射击的模式
typedef enum
{
    AUTO_OFF=0,
    AUTO_ON,
    FIND_Enermy,
}AutoAim_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
//底盘功率数据
typedef struct
{ // 功率控制
    float chassis_power_mx;  
} Chassis_Power_Data_s;    

//从裁判系统获取的数据
typedef struct
{ // 功率控制
    uint16_t robot_HP;    //血量
} Robot_info_s;


/* ----------------用于记录时间或标志位的结构体---------------- */
typedef struct
{
    float t_shoot;
    float t_pitch;
    float t_cmd_error;
    uint8_t aim_flag;
    uint8_t shoot_flag;
    uint8_t cmd_error_flag;
    uint8_t fire_flag;    //发射标志位
    uint8_t reverse_flag;    //(装弹电机)反向标志位
}DataLebel_t;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布给底盘的控制数据,由chassis订阅    //底盘控制数据
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    float chassis_rotate_buff;
    float chassis_speed_buff;
    float power_limit;
} Chassis_Ctrl_Cmd_s;        

// cmd发布给云台的控制数据,由gimbal订阅    //云台控制数据
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float real_pitch;
    float chassis_rotate_wz;
    AutoAim_mode_e autoaim_mode;
    gimbal_mode_e gimbal_mode;
    float last_deep;
} Gimbal_Ctrl_Cmd_s;

// cmd发布给发射器的控制数据,由shoot订阅    //发射控制数据
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e loader_mode;      //发弹的装载电机 模式
    friction_mode_e friction_mode;   //摩擦轮电机
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */
//底盘上传(返回)的数据 
typedef struct
{
    Enemy_Color_e enemy_color;   // 1 for blue, 0 for red
    uint16_t robot_level;
    uint8_t power_flag;
} Chassis_Upload_Data_s;


//云台上传(返回)的数据
typedef struct
{
    attitude_t gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
    uint8_t cmd_error_flag;
    float init_location;
} Gimbal_Upload_Data_s;


//发射上传(返回)的数据
typedef struct
{
    uint8_t cmd_error_flag;     //控制错误标志位
    int16_t loader_speed_aps;   //摩擦轮速度
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H