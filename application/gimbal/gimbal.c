//app
#include "gimbal.h"
#include  "robot_def.h"

//modules
#include "mi_motor.h"     //小米电机
#include "dji_motor.h"      //大疆电机
#include "message_center.h"    //订阅，发布
#include "ins_task.h"        //陀螺仪姿态解算
#include "general_def.h"     


static attitude_t *gimbal_IMU_data;      //  云台的IMU(惯性测量单元)数据
static DJIMotorInstance *yaw_motor;     //yaw(水平转)用大疆电机  //大疆电机实例
static MIMotorInstance  *pitch_motor;    //pitch（抬，低）用小米电机 

/***********************************双板通信****************************** */
static Publisher_t *gimbal_pub;       //云台应用信息发布者   （向cmd反馈自身状态）
static Subscriber_t *gimbal_sub;    //订阅(cmd)的云台控制信息

static Gimbal_Upload_Data_s gimbal_feedback_data;   //云台发布的反馈信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;    //云台订阅的cmd控制信息
/******************************************************************** */

static uint8_t motor_init =0;

//云台初始化
void GimbalInit()
{
    gimbal_IMU_data=INS_Init();  //IMU姿态解算初始化，获取姿态数据指针赋给yaw电机的其他数据来源

    //yaw轴      
    //大疆电机初始化
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            //待定
            .can_handle = &hfdcan1,     //can句柄
            .tx_id = 1,
        },
        .controller_param_init_config = {
            //角度环PID设置
            .angle_PID = {
                //待定
                .Kp = 3,
                .Ki = 20,
                .Kd = 3,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,   //积分限幅
                .MaxOut = 500,   //输出限幅
            },
            //速度环
            .speed_PID = {
                .Ki = 3,
                .Kd = 2,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,   //积分限幅
                .MaxOut = 500,   //输出限幅
            },
            //其他角度反馈  //yaw轴全部角度
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            //其他速度反馈   // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],   //角速度
        },
        .controller_setting_init_config = {
            //反馈来源
            .angle_feedback_source = OTHER_FEED,  
            .speed_feedback_source = OTHER_FEED,
            //外环，内环
            .outer_loop_type = ANGLE_LOOP,      //外环角度环
            .close_loop_type = ANGLE_LOOP  | SPEED_LOOP,   //内环 角度环加速度环
            //电机的正反转标志
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,   //正转
        },
        //电机类型
        .motor_type = GM6020,
    };

    //pitch轴   小米电机
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            //待定
            .can_handle = &hfdcan2,
        },
        .controller_param_init_config = {

            .angle_PID = {
                //待定
                .Kp = 10,
                .Ki = 6,
                .Kd = 7,
            },
            .speed_PID = {
                .Kp = 1,
                .Ki = 2,
                .Kd = 3,
            },
        }
    };

    //初始化大疆电机和小米电机
    yaw_motor = DJIMotorInit(&yaw_config);      //参数为前面的初始化
    pitch_motor = MIMotorInit(&pitch_motor);  


    MIMotorModeSwitch(pitch_motor,1);   //小米电机模式选择
    // //设置小米电机的PID
    // MIMotorSetPid(pitch_motor,pitch_motor->motor_controller.angle_PID.Kp,4,pitch_motor->motor_controller.speed_PID.Kp,pitch_motor->motor_controller.speed_PID.Ki);
    //设置小米电机的机械零位
    MIMotorInstanceetMechPositionToZero(pitch_motor);    

    //注册云台的发布者和订阅者
    gimbal_pub = PubRegister("gimbal_feed",sizeof(Gimbal_Upload_Data_s));   //云台发布云台的反馈信息
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));      //云台订阅cmd的控制信息
}

//设置云台的状态
static void GimbalStateSet()
{
    //cmd控制信息里的云台模式
    switch(gimbal_cmd_recv.gimbal_mode)
    {
        case GIMBAL_ZERO_FORCE:   //电流0输入
        {
            MIMotorInstancestop(pitch_motor);     //pitch小米电机//电机停止运行帧
            DJIMotorStop(yaw_motor);          //yaw轴小米电机停止
            motor_init = 0;
            break;
        }
        case GIMBAL_GYRO_MODE:   //云台陀螺仪反馈
        {
            //yaw轴大疆电机
            DJIMotorEnable(yaw_motor);
            DJIMotorSetRef(yaw_motor,gimbal_cmd_recv.yaw);    //设置电机的参考值
            
            //小米电机位置   //角度环控制
            MI_motor_LocationControl(pitch_motor,gimbal_cmd_recv.pitch,pitch_motor->motor_controller.angle_PID.Kp,pitch_motor->motor_controller.angle_PID.Kd);
            if(motor_init == 0)   //标志位
            {
                MIMotorEnable(pitch_motor); 
                gimbal_feedback_data.init_location = gimbal_IMU_data->Pitch;      //陀螺仪pitcch数据
                  //设置小米电机的PID
                MIMotorSetPid(pitch_motor,pitch_motor->motor_controller.angle_PID.Kp,4,pitch_motor->motor_controller.speed_PID.Kp,pitch_motor->motor_controller.speed_PID.Ki);
                motor_init = 1;
            }
        }
        /*
        else
        MiMotorSetRef(pitch_motor,gimbal_cmd_recv.pitch);
        */
        break;
    default:
        break;
    }
}

//云台发送（反馈）的数据 : 
static void SendGimbalData()
{
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;  //陀螺仪测的数据
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round; //测量的
}



/*机器人云台控制核心任务*/
void GimbalTask()
{
    //获取(订阅)云台的控制信息    //订阅cmd发布的控制信息
    SubGetMessage(gimbal_sub,(void *)&gimbal_cmd_recv);

    //云台状态(模式)的设置
    GimbalStateSet();

    //云台发送的(反馈)信息    //主要是imu和yaw的ecd
    SendGimbalData();

    //发送(发布)云台的反馈信息   //云台发布反馈信息给cmd
    PubPushMessage(gimbal_pub,(void *)&gimbal_feedback_data);
}












