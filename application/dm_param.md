# 空载 mit 模式4310 pid

    //这个是mit参数
    // arm_cmd_send.joint0_angle += rc_data[TEMP].rc.rocker_left_x*0.0002f;
    // if(arm_cmd_send.joint0_angle>12)
    // {
    //     arm_cmd_send.joint0_angle=12;
    // }
    // if(arm_cmd_send.joint0_angle<-12)
    // {
    //     arm_cmd_send.joint0_angle=-12;
    // }

    Motor_Init_Config_s joint0_config=
    {
        .can_init_config=
        {
            .can_handle=&hfdcan1,
            .tx_id=1,
        },
        .controller_param_init_config=
        {
            .angle_PID=
            {
                .Kp=80,
            },
            .speed_PID = 
            {
                .Kd =50
            },
        },
        .motor_type=DM,
    };
    joint0->mit_flag=1;
    DMMotorEnable(joint0);
    DMMotorSetPosition(joint0,arm_cmd_recv.joint0_angle) ;
    DMMotorSetFFTorque(joint0,0.8*(joint0->measure.torque-joint0->measure.last_torque));

# 空载 4310电机反馈模式 pid

    arm_cmd_send.joint0_angle += rc_data[TEMP].rc.rocker_left_x*0.001f;
                      .angle_PID=
            {
                .Kp=12,
                .Ki=0.5,
                .Kd=0.48,
                .Kf=1,
                .Improve =PID_ChangingIntegrationRate |  PID_Derivative_On_Measurement,
                .CoefA=2,
                .CoefB=2,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = 
            {
                .Kp=0.11,
                .Ki=0,
                .Kd=0,
                .Kf=0.1,
                .Improve =PID_ChangingIntegrationRate | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 100,
            },
            .current_PID=
            {
                .Kp=1,
                .Kf=2,
                .MaxOut=10,
            },
        DMMotorEnable(joint1);
        DMMotorSetRef(joint1,arm_cmd_recv.joint0_angle);