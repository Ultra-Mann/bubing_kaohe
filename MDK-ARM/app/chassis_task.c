#include "stm32f4xx.h"                  // Device header
#include "chassis_task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "remote_control.h"
#include "main.h"

chassis_move_t chassis_move;
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
pid_type_def pid_yaw_angle;
pid_type_def pid_yaw_speed;
pid_type_def pid_pitch_angle;
pid_type_def pid_pitch_speed;

fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

void chassis_init(chassis_move_t *chassis_move_init)
{
	chassis_move_init->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
	
	chassis_move_init->chassis_RC = get_remote_control_point();

	
	const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
	
	motor_data_Angle_pitch = get_pitch_gimbal_motor_measure_point(); //��ȡ�������ָ����
	motor_data_Angle_yaw = get_yaw_gimbal_motor_measure_point();
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
	
	PID_init(&pid_yaw_angle,PID_POSITION,PID_Angle_yaw,600,150);
	PID_init(&pid_yaw_speed,PID_POSITION,PID_Speed_yaw,20000,2000);//�ṹ�壬����ģʽ�������������������������
	PID_init(&pid_pitch_angle,PID_POSITION,PID_Angle,600,150);
	PID_init(&pid_pitch_speed,PID_POSITION,PID_Speed,20000,2000);
	PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	for(int i=0; i<4; i++)
	{
	    chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		PID_init(&chassis_move_init->motor_speed_pid[i],PID_POSITION,PID_Speed_3508,20000,5000);
	}
	
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
	
	chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	
	chassis_feedback_update(chassis_move_init);
}

void chassis_control(void)
{
	
	for(int i=0; i<4; i++)
	{
		PID_calc(&chassis_move.motor_speed_pid[i],motor_data_Speed[i]->speed_rpm,1000);
	}
	
	CAN_cmd_chassis(chassis_move.motor_speed_pid[0].out,chassis_move.motor_speed_pid[0].out,chassis_move.motor_speed_pid[0].out,chassis_move.motor_speed_pid[0].out);
}

static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
	
    for (i = 0; i < 4; i++)
    {
        //update motor speed, accel is differential of speed PID
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //calculate vertical speed, horizontal speed ,rotation speed, left hand rule 
    //���µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    //���������̬�Ƕ�, �����������������������ⲿ�ִ���
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}



void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
	if(chassis_move_mode->chassis_RC->rc.s[0] == 3)
	{
		chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
	}
	else if(chassis_move_mode->chassis_RC->rc.s[0] == 1)
	{
		chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
	}
	
	if(chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
	}
	else if(chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
	}
}

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	chassis_behaviour_mode_set(chassis_move_mode);
}

static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }
	
	//���������̨ģʽ
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
	//���벻������̨ģʽ
	else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
	
	chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
	int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
	//�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
	
	vx_set_channel = (vx_channel) * CHASSIS_VX_RC_SEN;
    vy_set_channel = (vy_channel) * -CHASSIS_VY_RC_SEN;
	
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
	
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[2];
}

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
	
	//swing angle is generated by sin function, swing_time is the input time of sin
    //ҡ�ڽǶ�������sin�������ɣ�swing_time ��sin����������ֵ
    static fp32 swing_time = 0.0f;
    
    static fp32 swing_angle = 0.0f;
    //max_angle is the max angle that chassis will ratate
    //max_angle ��sin�����ķ�ֵ
    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
    //swing_time  plus the add_time in one control cycle
    //swing_time ��һ�����������ڣ����� add_time
    static fp32 const add_time = PI * 0.5f * configTICK_RATE_HZ / CHASSIS_CONTROL_TIME_MS;
    
    static uint8_t swing_flag = 0;
	
	if (chassis_move_rc_to_vector->chassis_RC->key.v & SWING_KEY)
    {
        if (swing_flag == 0)
        {
            swing_flag = 1;
            swing_time = 0.0f;
        }
    }
	else
	{
		swing_flag = 0;
	}
	
	//judge if keyboard is controlling the chassis, if yes, reduce the max_angle
    //�жϼ��������ǲ����ڿ��Ƶ����˶����������˶���Сҡ�ڽǶ�
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY ||
        chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        max_angle = SWING_MOVE_ANGLE;
    }
    else
    {
        max_angle = SWING_NO_MOVE_ANGLE;
    }
    
    if (swing_flag)
    {
        swing_angle = max_angle * arm_sin_f32(swing_time);
        swing_time += add_time;
    }
    else
    {
        swing_angle = 0.0f;
    }
    //swing_time  range [0, 2*PI]
    //sin����������2pi
    if (swing_time > 2 * PI)
    {
        swing_time -= 2 * PI;
    }


    *angle_set = swing_angle;

}

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
	else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}

static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
	fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
	
	chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
	
	if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //rotate chassis direction, make sure vertial direction follow gimbal 
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //set control relative angle  set-point
        //���ÿ��������̨�Ƕ�
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //calculate ratation speed
        //������תPID���ٶ�
        chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
        //speed limit
        //�ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
	
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //"angle_set" is rotation speed set-point
        //��angle_set�� ����ת�ٶȿ���
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
}

static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
	wheel_speed[0] = -(vx_set) - (vy_set-0.125f) + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (wz_set+1.0f);
    wheel_speed[1] = (vx_set) - (vy_set-0.125f) + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (wz_set+1.0f);
    wheel_speed[2] = (vx_set) + (vy_set-0.125f) + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (wz_set+1.0f);
    wheel_speed[3] = -(vx_set) + (vy_set-0.125f) + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (wz_set+1.0f);

//	wheel_speed[0] = ((vx_set) - (vy_set) + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (wz_set));
//    wheel_speed[1] = (-(vx_set) - (vy_set) + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (wz_set));
//    wheel_speed[2] = (-(vx_set) + (vy_set) + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (wz_set));
//    wheel_speed[3] = ((vx_set) + (vy_set) + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (wz_set));
//	wheel_speed[1] = (-(vx_set) - (vy_set) + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (wz_set));
//    wheel_speed[3] = ((vx_set) - (vy_set) + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (wz_set));
//    wheel_speed[2] = ((vx_set) + (vy_set) + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (wz_set));
//    wheel_speed[0] = (-(vx_set) + (vy_set) + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (wz_set));
}

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
	
	chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);
	
	for(i = 0; i < 4; i++)
	{
		chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
		temp = __fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
		if(max_vector < temp)
		{
			max_vector = temp;
		}
	}
	
	if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }
	
	for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }
	
	 //chassis_power_control(chassis_move_control_loop);
	
	for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}

void chassis_task(void const *pvParameters)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	
	chassis_init(&chassis_move);
	
	while (1)
    {
        //set chassis control mode
        //���õ��̿���ģʽ
        chassis_set_mode(&chassis_move);
        //when mode changes, some data save
        //ģʽ�л����ݱ���
        chassis_mode_change_control_transit(&chassis_move);
        //chassis data update
        //�������ݸ���
        chassis_feedback_update(&chassis_move);
        //set chassis control set-point 
        //���̿���������
        chassis_set_contorl(&chassis_move);
        //chassis control pid calculate
        //���̿���PID����
        chassis_control_loop(&chassis_move);

        //make sure  one motor is online at least, so that the control CAN message can be received
        //ȷ������һ��������ߣ� ����CAN���ư����Ա����յ�
//        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
//        {
//            //when remote control is offline, chassis motor should receive zero current. 
//            //��ң�������ߵ�ʱ�򣬷��͸����̵�������.
//            if (toe_is_error(DBUS_TOE))
//            {
//                CAN_cmd_chassis(0, 0, 0, 0);
//            }
//            else
//            {
                //send control current
                //���Ϳ��Ƶ���
                CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
//            }
//        }
        //os delay
        //ϵͳ��ʱ
		
		seth = seth + 0.0001*(chassis_move.chassis_RC->rc.ch[3]);
		HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
		Angle_Current_pitch=Angle_Change(motor_data_Angle_pitch->ecd);
		Angle_Current_yaw=Angle_Change(motor_data_Angle_yaw->ecd);
		PID_calc(&pid_pitch_angle,Angle_Current_pitch,seth); //�⻷
		PID_calc(&pid_pitch_speed,motor_data_Angle_pitch->speed_rpm,pid_pitch_angle.out); //�ڻ�
		PID_calc(&pid_yaw_angle,Angle_Current_yaw,210); //�⻷
		PID_calc(&pid_yaw_speed,motor_data_Angle_yaw->speed_rpm,pid_yaw_angle.out); //�ڻ�
		CAN_cmd_gimbal(pid_yaw_speed.out,pid_pitch_speed.out,0,0);
		
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	
	
	#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}
