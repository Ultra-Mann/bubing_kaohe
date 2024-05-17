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
////ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
//#define CHASSIS_VX_RC_SEN 0.006f
////ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
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
////ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
//#define SWING_NO_MOVE_ANGLE 0.7f
////when chassis is set to move, swing max angle
////ҡ�ڹ��̵����˶����Ƕ�(rad)
//#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f
//#define PI					3.14159265358979f
////����������Ƽ�� 2ms
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
//    fp32 input;        //��������
//    fp32 out;          //�˲����������
//    fp32 num[1];       //�˲�����
//    fp32 frame_period; //�˲���ʱ���� ��λ s
//} first_order_filter_type_t;
//	
//typedef enum
//{
//  CHASSIS_ZERO_FORCE,                   //chassis will be like no power,��������, ��û�ϵ�����
//  CHASSIS_NO_MOVE,                      //chassis will be stop,���̱��ֲ���
//  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,   //chassis will follow gimbal, usually in infantry,�����������̸�����̨
//  CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,  //chassis will follow chassis yaw angle, usually in engineer,
//                                        //because chassis does have gyro sensor, its yaw angle is calculed by gyro in gimbal and gimbal motor angle,
//                                        //if you have a gyro sensor in chassis, please updata yaw, pitch, roll angle in "chassis_feedback_update"  function
//                                        //���̵��̽Ƕȿ��Ƶ��̣����ڵ���δ�������ǣ��ʶ��Ƕ��Ǽ�ȥ��̨�Ƕȶ��õ���
//                                        //����е�������������µ��̵�yaw��pitch��roll�Ƕ� ��chassis_feedback_update������
//  CHASSIS_NO_FOLLOW_YAW,                //chassis does not follow angle, angle is open-loop,but wheels have closed-loop speed
//                                        //���̲�����Ƕȣ��Ƕ��ǿ����ģ������������ٶȻ�
//  CHASSIS_OPEN                          //the value of remote control will mulitiply a value, get current value that will be sent to can bus
//                                        // ң������ֵ���Ա����ɵ���ֵ ֱ�ӷ��͵�can������
//} chassis_behaviour_e;

//typedef enum
//{
//  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //chassis will follow yaw gimbal motor relative angle.���̻������̨��ԽǶ�
//  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //chassis will have yaw angle(chassis_yaw) close-looped control.�����е��̽Ƕȿ��Ʊջ�
//  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //chassis will have rotation speed control. ��������ת�ٶȿ���
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
//  const gimbal_motor_t *chassis_yaw_motor;   //will use the relative angle of yaw gimbal motor to calculate the euler angle.����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����.
//  const gimbal_motor_t *chassis_pitch_motor; //will use the relative angle of pitch gimbal motor to calculate the euler angle.����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
//  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��, the point to remote control
//  const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.��ȡ�����ǽ������ŷ����ָ��
//  chassis_mode_e chassis_mode;               //state machine. ���̿���״̬��
//  chassis_mode_e last_chassis_mode;          //last state machine.�����ϴο���״̬��
//  chassis_motor_t motor_chassis[4];          //chassis motor data.���̵������
//  pid_type_def motor_speed_pid[4];             //motor speed PID.���̵���ٶ�pid
//  pid_type_def chassis_angle_pid;              //follow angle PID.���̸���Ƕ�pid
//  pid_type_def pid_yaw_angle;
//  pid_type_def pid_yaw_speed;
//  pid_type_def pid_pich_angle;	
//  pid_type_def pid_pich_speed;	
//	
//  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
//  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ	
//	
//  fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. �����ٶ� ǰ������ ǰΪ������λ m/s
//  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
//  fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
//  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.�����趨�ٶ� ǰ������ ǰΪ������λ m/s
//  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
//  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
//  fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.��������̨����ԽǶȣ���λ rad
//  fp32 chassis_relative_angle_set;  //the set relative angle.���������̨���ƽǶ�
//  fp32 chassis_yaw_set;             

//  fp32 vx_max_speed;  //max forward speed, unit m/s.ǰ����������ٶ� ��λm/s
//  fp32 vx_min_speed;  //max backward speed, unit m/s.���˷�������ٶ� ��λm/s
//  fp32 vy_max_speed;  //max letf speed, unit m/s.��������ٶ� ��λm/s
//  fp32 vy_min_speed;  //max right speed, unit m/s.�ҷ�������ٶ� ��λm/s
//  fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�yaw�Ƕ�
//  fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�pitch�Ƕ�
//  fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�roll�Ƕ�

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
