#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_dwt.h"
static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];

static CANInstance sender_assignment[6] = {
    [0] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x01, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x02, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x03, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x04, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [4] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x05, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [5] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x06, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
};




/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    measure->last_torque = measure->torque;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
    if (measure->position - measure->last_position > 12)
    {
        measure->total_round--;
    }
    else if (measure->position - measure->last_position < -12)
    {
        measure->total_round++;
    }
    measure->angle_single_round = ECD_ANGLE_COEF_DM * (float)measure->position;
    //总角度等于圈数*360°+当前角度
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
    if(motor->motor_type ==DM8009)
    {
        measure->real_total_angle =  measure->total_angle*4;
        float temp_angle = measure->angle_single_round * 4;
        measure->real_total_round = (int32_t)(temp_angle / 360.0f);
        measure->real_angle_single_round = temp_angle - measure->real_total_round * 360.0f;
    }
    else if (motor->motor_type ==DM4310)
    {
        measure->real_total_angle =  measure->total_angle*3;
        float temp_angle = measure->angle_single_round * 3;
        measure->real_total_round = (int32_t)(temp_angle / 360.0f);
        measure->real_angle_single_round = temp_angle - measure->real_total_round * 360.0f;
    }



    // 规范化到[-180, 180]范围
    if(measure->real_angle_single_round > 180.0f) {
        measure->real_angle_single_round -= 360.0f;
        measure->real_total_round++;
    } else if(measure->real_angle_single_round < -180.0f) {
        measure->real_angle_single_round += 360.0f;
        measure->real_total_round--;
    }


}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_type = config->motor_type;
    motor->mit_flag=config->mit_flag;
    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);    
    DWT_Delay(0.1);
    DMMotorCaliEncoder(motor);
    DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetPosition(DMMotorInstance *motor, float position)
{
    motor->motor_controller.angle_PID.Ref = position;
}

void DMMotorSetSpeed(DMMotorInstance *motor, float Speed)
{
    motor->motor_controller.speed_PID.Ref = Speed;
}

void DMMotorSetFFTorque(DMMotorInstance *motor, float Torque)
{
    motor->motor_controller.current_PID.Ref = Torque;
}
void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}
void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}



void DMMotorTask(void const *argument)
{
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    DMMotor_Send_s motor_send_mailbox;

    float set;        // 电机控制CAN发送设定值
    Motor_Control_Setting_s *setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    DM_Motor_Measure_s *measure;           // 电机测量值
    float pid_measure, pid_ref;             // 电机PID测量值和设定值

    while (1)
    {
        if(motor->stop_flag == MOTOR_STOP)
        {
            if(motor->set_stop_mode_flag==0)
            {
                DMMotorSetMode(DM_CMD_RESET_MODE, motor);
                DWT_Delay(0.1);
                motor->set_stop_mode_flag=1;
                motor->set_run_mode_flag=0;
            }
        }
        else
        {   
            if(motor->set_run_mode_flag==0)
            {
                DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
                DWT_Delay(0.1);
                motor->set_run_mode_flag=1;
                motor->set_stop_mode_flag=0;
            }
            if(motor->mit_flag==1)
            {
                motor_send_mailbox.position_des = float_to_uint(motor->motor_controller.angle_PID.Ref, DM_P_MIN, DM_P_MAX, 16);
                motor_send_mailbox.velocity_des = float_to_uint(motor->motor_controller.speed_PID.Ref, DM_V_MIN, DM_V_MAX, 12);
                motor_send_mailbox.torque_des   = float_to_uint(motor->motor_controller.current_PID.Ref, DM_T_MIN, DM_T_MAX, 12);
                motor_send_mailbox.Kp = float_to_uint(motor->motor_controller.angle_PID.Kp, DM_KP_MIN, DM_KP_MAX, 12);
                motor_send_mailbox.Kd = float_to_uint(motor->motor_controller.speed_PID.Kd, DM_KD_MIN, DM_KD_MAX, 12);
            }
            else
            {
                setting = &motor->motor_settings;
                motor_controller = &motor->motor_controller;
                measure = &motor->measure;
                pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
                if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
                {
                    if (setting->angle_feedback_source == OTHER_FEED)
                    {
                        pid_measure = *motor_controller->other_angle_feedback_ptr;
                    }    
                    else
                    {
                        pid_measure = measure->real_total_angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
                    }
                    // 更新pid_ref进入下一个环
                    pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
                }
                // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
                if ((setting->close_loop_type & SPEED_LOOP) && (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
                {
                    if (setting->speed_feedback_source == OTHER_FEED)
                    {
                        pid_measure = *motor_controller->other_speed_feedback_ptr;
                    }
                    else // MOTOR_FEED
                    {
                        pid_measure = measure->velocity;
                    }
                    // 更新pid_ref进入下一个环
                    pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
                }
                if (setting->close_loop_type & CURRENT_LOOP)
                {
                    pid_ref = PIDCalculate(&motor_controller->current_PID, measure->torque, pid_ref);
                }

                if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
                {
                    pid_ref *= -1;
                }
                set=pid_ref;
                motor_send_mailbox.torque_des = float_to_uint(set, DM_T_MIN, DM_T_MAX, 12);
                motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
                motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
                motor_send_mailbox.Kp = 0;
                motor_send_mailbox.Kd = 0;
            }
        }
        //设定位置_速度_p_d_力矩
        sender_assignment[idx].tx_buff[0]=(uint8_t)(motor_send_mailbox.position_des >> 8);
        sender_assignment[idx].tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        sender_assignment[idx].tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        sender_assignment[idx].tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        sender_assignment[idx].tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        sender_assignment[idx].tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        sender_assignment[idx].tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        sender_assignment[idx].tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);
        
        for (size_t i = 0; i < 6; ++i)
        {
            CANTransmit(&sender_assignment[i], 1);
        }
        osDelay(2);
    }
}
void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}