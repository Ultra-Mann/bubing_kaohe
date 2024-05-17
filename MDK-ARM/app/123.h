//#ifndef __123_H
//#define __123_H

//#include "main.h"
//#include "gimbal_task.h"
//#include "arm_math.h"

//#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f
//#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
//#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
//#define CHASSIS_CONTROL_FREQUENCE 500.0f
//#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
//#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
//#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
//#define MOTOR_DISTANCE_TO_CENTER 0.2f
//#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)
//#define INS_YAW_ADDRESS_OFFSET    0
//#define INS_PITCH_ADDRESS_OFFSET  1
//#define INS_ROLL_ADDRESS_OFFSET   2
//#define CHASSIS_WZ_RC_SEN 0.01f
//#define CHASSIS_X_CHANNEL 1
//#define CHASSIS_Y_CHANNEL 0
//#define CHASSIS_RC_DEADLINE 10
////遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
//#define CHASSIS_VX_RC_SEN 0.006f
////遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
//#define CHASSIS_VY_RC_SEN 0.005f
//#define CHASSIS_ACCEL_X_NUM 0.1666666667f
//#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
//#define CHASSIS_CONTROL_TIME 0.002f
//#define CHASSIS_WZ_SET_SCALE 0.1f
//#define MAX_WHEEL_SPEED 4.0f
//#define CHASSIS_TASK_INIT_TIME 357
//#define CHASSIS_CONTROL_TIME_MS 2

//#define CHASSIS_FOLLOW_GIMBAL_PID_KP 40.0f
//#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
//#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
//#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
//#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

////when chassis is not set to move, swing max angle
////摇摆原地不动摇摆最大角度(rad)
//#define SWING_NO_MOVE_ANGLE 0.7f
////when chassis is set to move, swing max angle
////摇摆过程底盘运动最大角度(rad)
//#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f
//#define PI					3.14159265358979f
////底盘任务控制间隔 2ms
//#define CHASSIS_CONTROL_TIME_MS 2
//#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
//#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
//#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
//#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//#define rc_deadband_limit(input, output, dealine)        \
//    {                                                    \
//        if ((input) > (dealine) || (input) < -(dealine)) \
//        {                                                \
//            (output) = (input);                          \
//        }                                                \
//        else                                             \
//        {                                                \
//            (output) = 0;                                \
//        }                                                \
//    }
//	
//typedef __packed struct
//{
//    fp32 input;        //输入数据
//    fp32 out;          //滤波输出的数据
//    fp32 num[1];       //滤波参数
//    fp32 frame_period; //滤波的时间间隔 单位 s
//} first_order_filter_type_t;
//	
//typedef enum
//{
//  CHASSIS_ZERO_FORCE,                   //chassis will be like no power,底盘无力, 跟没上电那样
//  CHASSIS_NO_MOVE,                      //chassis will be stop,底盘保持不动
//  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,   //chassis will follow gimbal, usually in infantry,正常步兵底盘跟随云台
//  CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,  //chassis will follow chassis yaw angle, usually in engineer,
//                                        //because chassis does have gyro sensor, its yaw angle is calculed by gyro in gimbal and gimbal motor angle,
//                                        //if you have a gyro sensor in chassis, please updata yaw, pitch, roll angle in "chassis_feedback_update"  function
//                                        //工程底盘角度控制底盘，由于底盘未有陀螺仪，故而角度是减去云台角度而得到，
//                                        //如果有底盘陀螺仪请更新底盘的yaw，pitch，roll角度 在chassis_feedback_update函数中
//  CHASSIS_NO_FOLLOW_YAW,                //chassis does not follow angle, angle is open-loop,but wheels have closed-loop speed
//                                        //底盘不跟随角度，角度是开环的，但轮子是有速度环
//  CHASSIS_OPEN                          //the value of remote control will mulitiply a value, get current value that will be sent to can bus
//                                        // 遥控器的值乘以比例成电流值 直接发送到can总线上
//} chassis_behaviour_e;

//typedef enum
//{
//  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //chassis will follow yaw gimbal motor relative angle.底盘会跟随云台相对角度
//  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //chassis will have yaw angle(chassis_yaw) close-looped control.底盘有底盘角度控制闭环
//  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //chassis will have rotation speed control. 底盘有旋转速度控制
//  CHASSIS_VECTOR_RAW,                 //control-current will be sent to CAN bus derectly.

//} chassis_mode_e;

//typedef struct
//{
//  const motor_measure_t *chassis_motor_measure;
//  fp32 accel;
//  fp32 speed;
//  fp32 speed_set;
//  int16_t give_current;
//} chassis_motor_t;

//typedef struct
//{
//  const gimbal_motor_t *chassis_yaw_motor;   //will use the relative angle of yaw gimbal motor to calculate the euler angle.底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角.
//  const gimbal_motor_t *chassis_pitch_motor; //will use the relative angle of pitch gimbal motor to calculate the euler angle.底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
//  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针, the point to remote control
//  const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.获取陀螺仪解算出的欧拉角指针
//  chassis_mode_e chassis_mode;               //state machine. 底盘控制状态机
//  chassis_mode_e last_chassis_mode;          //last state machine.底盘上次控制状态机
//  chassis_motor_t motor_chassis[4];          //chassis motor data.底盘电机数据
//  pid_type_def motor_speed_pid[4];             //motor speed PID.底盘电机速度pid
//  pid_type_def chassis_angle_pid;              //follow angle PID.底盘跟随角度pid
//  pid_type_def pid_yaw_angle;
//  pid_type_def pid_yaw_speed;
//  pid_type_def pid_pich_angle;	
//  pid_type_def pid_pich_speed;	
//	
//  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
//  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值	
//	
//  fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. 底盘速度 前进方向 前为正，单位 m/s
//  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.底盘速度 左右方向 左为正  单位 m/s
//  fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.底盘旋转角速度，逆时针为正 单位 rad/s
//  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.底盘设定速度 前进方向 前为正，单位 m/s
//  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.底盘设定速度 左右方向 左为正，单位 m/s
//  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 单位 rad/s
//  fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.底盘与云台的相对角度，单位 rad
//  fp32 chassis_relative_angle_set;  //the set relative angle.设置相对云台控制角度
//  fp32 chassis_yaw_set;             

//  fp32 vx_max_speed;  //max forward speed, unit m/s.前进方向最大速度 单位m/s
//  fp32 vx_min_speed;  //max backward speed, unit m/s.后退方向最大速度 单位m/s
//  fp32 vy_max_speed;  //max letf speed, unit m/s.左方向最大速度 单位m/s
//  fp32 vy_min_speed;  //max right speed, unit m/s.右方向最大速度 单位m/s
//  fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的yaw角度
//  fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的pitch角度
//  fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的roll角度

//} chassis_move_t;

//const motor_measure_t *motor_data_Speed_yaw;
//const motor_measure_t *motor_data_Angle_yaw;
//const motor_measure_t *motor_data_Speed_pitch;
//const motor_measure_t *motor_data_Angle_pitch;
//const motor_measure_t *motor_data_Speed[4];
//static void chassis_feedback_update(chassis_move_t *chassis_move_update);

//fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
//void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//void chassis_init(chassis_move_t *chassis_move_init);
//void chassis_control(void);
//static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);
//static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
//void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);
//static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
//void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
//static void chassis_set_contorl(chassis_move_t *chassis_move_control);
//static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);
//static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

//#endif
