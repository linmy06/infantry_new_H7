#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

#include "controller.h"
#include "stdint.h"
#include "lqr.h"
#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/**
 * @brief 闭环类型,如果需要多个闭环,则使用或运算
 *        例如需要速度环和电流环: CURRENT_LOOP|SPEED_LOOP
 */
typedef enum
{
    OPEN_LOOP = 0b0000,        //开环
    CURRENT_LOOP = 0b0001,     //电流环
    SPEED_LOOP = 0b0010,      //速度环
    ANGLE_LOOP = 0b0100,     //角度环/位置环

    // only for checking
    SPEED_AND_CURRENT_LOOP = 0b0011,
    ANGLE_AND_SPEED_LOOP = 0b0110,
    ALL_THREE_LOOP = 0b0111,
} Closeloop_Type_e;      //闭环控制类型


//反馈方式
typedef enum
{
    FEEDFORWARD_NONE = 0b00,
    CURRENT_FEEDFORWARD = 0b01,       //反馈来自电流环
    SPEED_FEEDFORWARD = 0b10,          //反馈来自速度环
    CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} Feedfoward_Type_e;

/* 反馈来源设定,若设为OTHER_FEED则需要指定数据来源指针,详见Motor_Controller_s*/
typedef enum
{
    MOTOR_FEED = 0,   //反馈来自电机
    OTHER_FEED,         //其他
} Feedback_Source_e;

/* 电机正反转标志 */
typedef enum
{
    MOTOR_DIRECTION_NORMAL = 0,   //正转
    MOTOR_DIRECTION_REVERSE = 1   //反转
} Motor_Reverse_Flag_e;

/* 反馈量正反标志 */
typedef enum
{
    FEEDBACK_DIRECTION_NORMAL = 0,    //正反馈
    FEEDBACK_DIRECTION_REVERSE = 1     //负反馈
} Feedback_Reverse_Flag_e;

//电机工作模式
typedef enum
{
    MOTOR_STOP = 0,     //电机停止
    MOTOR_ENALBED = 1,
} Motor_Working_Type_e;

//控制算法选择
typedef enum
{
    PID_MODE=0,    //PID控制算法
    LQR_MODE=1      //LAR控制算法
} Control_Algorithms_e;

/* 电机控制设置,包括“闭环类型”,“反转标志”和“反馈来源” */
typedef struct
{
    Closeloop_Type_e outer_loop_type;              // 最外层的闭环,未设置时默认为最高级的闭环
    Closeloop_Type_e close_loop_type;              // 使用几个闭环(串级)
    Motor_Reverse_Flag_e motor_reverse_flag;       // 是否反转
    Feedback_Reverse_Flag_e feedback_reverse_flag; // 反馈是否反向
    Feedback_Source_e angle_feedback_source;       // 角度反馈类型
    Feedback_Source_e speed_feedback_source;       // 速度反馈类型
    Feedfoward_Type_e feedforward_flag;            // 前馈标志
    Control_Algorithms_e control_algorithm;

} Motor_Control_Setting_s;   //电机控制设置


/* 电机控制器,包括其他来源的反馈数据指针,3环控制器和电机的参考输入*/
typedef struct
{
    float *other_angle_feedback_ptr; // 其他反馈来源的反馈数据指针
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;

    //PID控制算法
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;

    float pid_ref; // 将会作为每个环的输入和输出顺次通过串级闭环
    float lqr_ref;
    LQRInstance lqr;     //lqr算法
} Motor_Controller_s;

/* 电机类型枚举 */
typedef enum
{
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
    LK9025,
    HT04,
    DM8009,
    DM4310,
    
} Motor_Type_e;

/**
 * @brief 电机控制器初始化结构体,包括三环PID的配置以及两个反馈数据来源指针
 *        如果不需要某个控制环,可以不设置对应的pid config
 *        需要其他数据来源进行反馈闭环,不仅要设置这里的指针还需要在Motor_Control_Setting_s启用其他数据来源标志
 */
typedef struct
{
    float *other_angle_feedback_ptr; // 角度反馈数据指针,注意电机使用total_angle
    float *other_speed_feedback_ptr; // 速度反馈数据指针,单位为angle per sec

    float *speed_feedforward_ptr;   // 速度前馈数据指针
    float *current_feedforward_ptr; // 电流前馈数据指针

    PID_Init_Config_s current_PID;    //电流环PID
    PID_Init_Config_s speed_PID;      //速度环PID
    PID_Init_Config_s angle_PID;        //角度环PID

    LQR_Init_Config_s lqr;
    Control_Algorithms_e control_algorithm;

} Motor_Controller_Init_s;

/* 用于初始化CAN电机的结构体,各类电机通用 */
typedef struct
{
    Motor_Controller_Init_s controller_param_init_config;  //电机控制器参数初始化   //运动环控制
    Motor_Control_Setting_s controller_setting_init_config; // 初始化设定
    Motor_Type_e motor_type;                    //电机类别
    CAN_Init_Config_s can_init_config;        //can设置初始化
    uint8_t mit_flag;
} Motor_Init_Config_s;

#endif // !MOTOR_DEF_H
