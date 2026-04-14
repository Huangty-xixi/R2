#include "chassis.h"
#include "Motion_Task.h"
#include "lift.h"

Chassis_Module Chassis;


//底盘
DJI_MotorModule chassis_motor1;  // （左前）
DJI_MotorModule chassis_motor2;  // （右前）
DJI_MotorModule chassis_motor3;  // （左后）
DJI_MotorModule chassis_motor4;  // （右后）

//导轮
DJI_MotorModule guide_motor1;  // （左）
DJI_MotorModule guide_motor2;  // （右）



//活动电机伸缩状态机状态
static uint8_t flexible_motor_has_stopped = 0;
static uint8_t flexible_motor_running = 0;

/* 当前主控底盘命令缓存（仅解码预留，后续可接雷达策略） */
static master_chassis_cmd_t g_master_chassis_cmd;

static void chassis_decode_master_cmd(uint8_t action_byte0, uint8_t action_byte1)
{
    g_master_chassis_cmd.speed_level = (chassis_speed_level_t)((action_byte0 >> 6) & 0x03U);
    g_master_chassis_cmd.move_dir = (chassis_move_dir_t)((action_byte0 >> 3) & 0x07U);
    g_master_chassis_cmd.rot_dir = (chassis_rot_dir_t)((action_byte0 >> 1) & 0x03U);
    g_master_chassis_cmd.flexible_extend = (uint8_t)(action_byte0 & 0x01U);
    g_master_chassis_cmd.reserved_byte1 = action_byte1;
}

static uint16_t chassis_get_speed_amp(chassis_speed_level_t level)
{
    const uint16_t span = (uint16_t)(CH2_HIGH - CH2_MID); /* 800 */

    switch (level)
    {
        case CHASSIS_SPEED_LOW:
            return (uint16_t)(span / 3U);         /* 1/3 */
        case CHASSIS_SPEED_MID:
            return (uint16_t)((span * 2U) / 3U);  /* 2/3 */
        case CHASSIS_SPEED_HIGH:
            return span;                          /* 1 */
        case CHASSIS_SPEED_RESERVED:
        default:
            return 0U;
    }
}

static void chassis_apply_master_motion(void)
{
    uint16_t amp = chassis_get_speed_amp(g_master_chassis_cmd.speed_level);

    /* master模式下将底盘三轴通道改为“中位+固定幅值” */
    RCctrl.CH1 = CH1_MID; /* 左右 */
    RCctrl.CH2 = CH2_MID; /* 前后 */
    RCctrl.CH4 = CH4_MID; /* 旋转 */

    /* ACCEL 依赖CH3，固定到高位保证速度幅值由 amp 统一控制 */
    RCctrl.CH3 = CH3_HIGH;

    switch (g_master_chassis_cmd.move_dir)
    {
        case CHASSIS_DIR_FORWARD:
            RCctrl.CH2 = (uint16_t)(CH2_MID + amp);
            break;
        case CHASSIS_DIR_BACKWARD:
            RCctrl.CH2 = (uint16_t)(CH2_MID - amp);
            break;
        case CHASSIS_DIR_LEFT:
            RCctrl.CH1 = (uint16_t)(CH1_MID - amp);
            break;
        case CHASSIS_DIR_RIGHT:
            RCctrl.CH1 = (uint16_t)(CH1_MID + amp);
            break;
        case CHASSIS_DIR_NONE:
        default:
            break;
    }

    switch (g_master_chassis_cmd.rot_dir)
    {
        case CHASSIS_ROT_LEFT:
            RCctrl.CH4 = (uint16_t)(CH4_MID - amp);
            break;
        case CHASSIS_ROT_RIGHT:
            RCctrl.CH4 = (uint16_t)(CH4_MID + amp);
            break;
        case CHASSIS_ROT_NONE:
        case CHASSIS_ROT_RESERVED:
        default:
            break;
    }
}



float chassis_motor1_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.15f,1,500.0f,10000.0f};     //KP,KI,KD,DEADBAND,LIMITINTEGRAL,LIMITOUTPUT
float chassis_motor2_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.25f,1,500.0f,10000.0f};
float chassis_motor3_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.25f,1,500.0f,10000.0f};
float chassis_motor4_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.15f,1,500.0f,10000.0f};

float guide_motor1_pid_param[PID_PARAMETER_NUM] = {3.0f,0.1f,0.2f,1,500.0f,10000.0f};
float guide_motor2_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};

/**
  * @brief 底盘运行逻辑
  */
void manual_chassis_function(void)
{
    /* 主控模式下先做底盘动作字节解码预留，便于后续接入雷达决策 */
    if (control_mode == master_control)
    {
        chassis_decode_master_cmd(master_chassis_action_bits_0, master_chassis_action_bits_1);
        chassis_apply_master_motion();
    }



    if (control_mode == master_control)
    {
        /* master模式：活动电机位 1=伸出，0=收回（映射到高低位边沿命令） */
        flexible_motor_update_command(g_master_chassis_cmd.flexible_extend ? CH5_LOW : CH5_HIGH);
    }
    else if(control_mode == remote_control)
    {
        flexible_motor_update_command(RCctrl.CH5);
    }
flexible_motor_state_machine_step();

///////////////////////////////////////////////////////////////////////


	Chassis.Chassis_Calc(&Chassis);

	chassis_motor1.PID_Calculate(&chassis_motor1, 50*Chassis.param.V_out[0]);
	chassis_motor2.PID_Calculate(&chassis_motor2, 50*Chassis.param.V_out[1]);
	chassis_motor3.PID_Calculate(&chassis_motor3, 50*Chassis.param.V_out[2]);
	chassis_motor4.PID_Calculate(&chassis_motor4, 50*Chassis.param.V_out[3]);
	
	guide_motor1.PID_Calculate(&guide_motor1, 200*Chassis.param.V_out[0]);
	guide_motor2.PID_Calculate(&guide_motor2, 200*Chassis.param.V_out[1]);

	flexible_motor1.PID_Calculate(&flexible_motor1,flexible_motor_PID_input);
	flexible_motor2.PID_Calculate(&flexible_motor2,-flexible_motor_PID_input);			

	DJIset_motor_data(&hfdcan1, 0X200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);
	DJIset_motor_data(&hfdcan2, 0X200, guide_motor1.pid_spd.Output, guide_motor2.pid_spd.Output,flexible_motor1.pid_spd.Output,flexible_motor2.pid_spd.Output);
		
//	DJIset_motor_data(&hfdcan1, 0X200,0,0,0,0);
//	DJIset_motor_data(&hfdcan2, 0X200,0,0,0,0);

}


void Chassis_Calc(Chassis_Module *chassis)
{
    chassis->param.Accel = ACCEL;
    chassis->param.Vw_in = LR_TRANSLATION;
    chassis->param.Vy_in = FB_TRANSLATION;
    chassis->param.Vx_in = ROTATION;
    
    chassis->param.V_out[0] = chassis->param.Vx_in + chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[1] = chassis->param.Vx_in - chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[2] = chassis->param.Vx_in + chassis->param.Vy_in - chassis->param.Vw_in;
    chassis->param.V_out[3] = chassis->param.Vx_in - chassis->param.Vy_in - chassis->param.Vw_in;

}

void Chassis_Stop(Chassis_Module *chassis)
{
    chassis->param.V_out[0] = 0.f;
    chassis->param.V_out[1] = 0.f;
    chassis->param.V_out[2] = 0.f;
    chassis->param.V_out[3] = 0.f;
    
}

void chassis_use()
{
	// 底盘电机正常输出（始终运行）
	DJIset_motor_data(&hfdcan1, 0X200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);
	DJIset_motor_data(&hfdcan2, 0X200, guide_motor1.pid_spd.Output, guide_motor2.pid_spd.Output,flexible_motor1.pid_spd.Output,flexible_motor2.pid_spd.Output);

}
