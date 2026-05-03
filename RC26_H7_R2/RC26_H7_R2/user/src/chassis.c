#include "chassis.h"
#include "Motion_Task.h"
#include "lift.h"
#include "master_control.h"
#include "Sensor_Task.h"
#include "chassis_heading_hold.h"
#include "zone1_process.h"
#include <math.h>

Chassis_Module Chassis;


// 底盘电机
DJI_MotorModule chassis_motor1;  // 左前
DJI_MotorModule chassis_motor2;  // 右前
DJI_MotorModule chassis_motor3;  // 左后
DJI_MotorModule chassis_motor4;  // 右后

// 导轮电机
DJI_MotorModule guide_motor1;  // 左
DJI_MotorModule guide_motor2;  // 右
 
uint16_t switch_state;//光电开关（PE9）

// 全局旋转实例（唯一定义）
Rot90_t rot = {0};

// 角度限制到 [-180, 180]
float rot_wrap_deg(float d)
{
    while (d > 180.0f)
        d -= 360.0f;
    while (d <= -180.0f)
        d += 360.0f;
    return d;
}

static float signf(float x)
{
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}
volatile float ROT_DEAD_ZONE = 1.0f; // 死区：小于此角度直接停机
volatile float ROT_MAX_SPEED = 30.0f; // 最大旋转速度
volatile float ROT_KP = 1.7f; // 比例系数
volatile float ROT_KD = 0.2f; //积分限幅
float target_angle[4] = {-90.0, 0.0, 90.0, 180.0};
uint8_t px = 1;
// 遥控拨码连续触发标志
static uint8_t left_trigger = 0;
static uint8_t right_trigger = 0;

/* 当前底盘指令缓存（用于将位定义转换为速度输入） */
static master_chassis_cmd_t g_master_chassis_cmd;
volatile float g_chassis_rotation_cmd_dbg = 0.0f; /* 临时观测：ROTATION原始值 */
volatile float g_chassis_vx_in_dbg = 0.0f;        /* 临时观测：旋转输入轴 */
volatile float g_chassis_vy_in_dbg = 0.0f;        /* 临时观测：前后输入轴 */
volatile float g_chassis_vw_in_dbg = 0.0f;        /* 临时观测：左右输入轴 */
volatile float g_imu_to_body_yaw_offset_deg = 0.0f;   /* IMU到车头的安装偏角（deg），当前按+9处理 */
volatile float g_chassis_yaw_body_deg_dbg = 0.0f;     /* 临时观测：补偿后的车头航向角（deg） */


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
            return (uint16_t)(span / 10U);        /* 1/10 */
        case CHASSIS_SPEED_NORMAL:
            return (uint16_t)(span / 2U);         /* 1/2 */
        case CHASSIS_SPEED_HIGH:
            return span;                          /* 1 */
        case CHASSIS_SPEED_SUPER_HIGH:
            return span * 1.5f;                      /* 2 */
        default:
            return 0U;
    }
}
static void chassis_apply_master_motion(void)
{
    uint16_t amp = chassis_get_speed_amp(g_master_chassis_cmd.speed_level);
    float trans = ((float)amp / (float)(CH2_HIGH - CH2_MID)) * 100.0f;
    float rot = (40.2814f / 4.0f) * (((float)amp / (float)(CH4_HIGH - CH4_MID)) * 5.0f);

    /* master模式下直接写底盘输入，不再借用RC通道值 */
    Chassis.param.Accel = 100.0f;
    Chassis.param.Vw_in = 0.0f;
    Chassis.param.Vy_in = 0.0f;
    Chassis.param.Vx_in = 0.0f;

    switch (g_master_chassis_cmd.move_dir)
    {
        case CHASSIS_DIR_FORWARD:
            Chassis.param.Vy_in = trans;
            break;
        case CHASSIS_DIR_BACKWARD:
            Chassis.param.Vy_in = -trans;
            break;
        case CHASSIS_DIR_LEFT:
            Chassis.param.Vw_in = -trans;
            break;
        case CHASSIS_DIR_RIGHT:
            Chassis.param.Vw_in = trans;
            break;
        case CHASSIS_DIR_NONE:
        default:
            break;
    }

    switch (g_master_chassis_cmd.rot_dir)
    {
        case CHASSIS_ROT_LEFT:
            Chassis.param.Vx_in = -rot;
            break;
        case CHASSIS_ROT_RIGHT:
            Chassis.param.Vx_in = rot;
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
    if (zone1_state != ZONE1_IDLE)
    {
        Chassis_Calc(&Chassis);
        return;
    }
    
    static MasterLevelGate master_chassis_flex_gate = {0U, 0U};

    /* 主控模式：按上位机动作字节解码并写入底盘输入 */
    if (control_mode == master_control)
    {
        chassis_decode_master_cmd(master_chassis_action_bits_0, master_chassis_action_bits_1);
        chassis_apply_master_motion();
    }



    if (control_mode == master_control)
    {
        uint8_t flex_level = (g_master_chassis_cmd.flexible_extend != 0U) ? 1U : 0U;

        if (master_level_gate_on_change(&master_chassis_flex_gate, flex_level) != 0U)
        {
            flex_cmd = flex_level ? FLEX_CMD_EXTEND : FLEX_CMD_RETRACT;
        }
        else
        {
            flex_cmd = FLEX_CMD_NONE;
        }
    }
		else if(control_mode == remote_control)
		{
				// ==================== CH5三档拨码：一键90°旋转 ====================
        // 连续检测拨码状态
        uint8_t ch5_left  = (RCctrl.CH5 <= 500);
        uint8_t ch5_right = (RCctrl.CH5 >= 1500);

        // 左转：连续切换
        if (ch5_left && !left_trigger)
        {
            left_trigger = 1;
            px = (px - 1 + 4) % 4;
            rot.target_yaw = target_angle[px];
            rot.enable = 1;
        }
        if (!ch5_left) left_trigger = 0;

        // 右转：连续切换
        if (ch5_right && !right_trigger)
        {
            right_trigger = 1;
            px = (px + 1) % 4;
            rot.target_yaw = target_angle[px];
            rot.enable = 1;
        }
        if (!ch5_right) right_trigger = 0;

				// 旋转逻辑
				if(rot.enable)
				{
						float current_yaw = g_imu_yaw_deg;
						rot.error = rot_wrap_deg(rot.target_yaw - current_yaw);
						float abs_err = fabs(rot.error);

						if(abs_err < ROT_DEAD_ZONE)
						{
								rot.enable = 0;
								Chassis.param.Vx_in = 0;
								Chassis.param.Vy_in = 0;
								Chassis.param.Vw_in = 0;
						}
						else
						{
								float spd = ROT_KP * rot.error-ROT_KD*g_imu_gyr_z_dps;
								if(spd > ROT_MAX_SPEED) spd = ROT_MAX_SPEED;
							  if(spd < -ROT_MAX_SPEED) spd = -ROT_MAX_SPEED;
								Chassis.param.Vx_in = -signf(rot.error) * spd;
								Chassis.param.Vy_in = 0;
								Chassis.param.Vw_in = 0;
						}
				}
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
    float yaw_body_deg = 0.0f;
		
    // 仅遥控模式从RC通道读取，master模式输入由chassis_apply_master_motion直接给定
    if (control_mode == remote_control && remote_mode == chassis_mode && rot.enable == 0) {
        chassis->param.Accel = ACCEL;
        chassis->param.Vw_in = LR_TRANSLATION;
        chassis->param.Vy_in = FB_TRANSLATION;
        g_chassis_rotation_cmd_dbg = ROTATION;
        chassis->param.Vx_in = g_chassis_rotation_cmd_dbg;
    }

    g_chassis_vx_in_dbg = chassis->param.Vx_in;
    g_chassis_vy_in_dbg = chassis->param.Vy_in;
    g_chassis_vw_in_dbg = chassis->param.Vw_in;

    /* 平面解耦：减少前后<->左右串扰（结构/负载变化导致） */
    ChassisDecouple_Apply(chassis->param.Vx_in, &chassis->param.Vy_in, &chassis->param.Vw_in);

    /* 航向保持：逻辑在 chassis_heading_hold.c 内部实现 */   
    /* 平移时角度保持：逻辑在 chassis_heading_hold.c 内部实现 */
    yaw_body_deg = g_imu_yaw_deg + g_imu_to_body_yaw_offset_deg;
    g_chassis_yaw_body_deg_dbg = yaw_body_deg;
     
		// 自动旋转模式下，彻底关闭航向保持修正，从根源消灭震荡
		if (rot.enable == 0)  
		{
			/* 平移时角度保持：逻辑在 chassis_heading_hold.c 内部实现 */
			chassis->param.Vx_in += ChassisHeadingHold_TranslationHoldStep((ChassisHeadingHold *)&g_heading_hold,
                                                                  yaw_body_deg,
                                                                  chassis->param.Vx_in,
                                                                  chassis->param.Vy_in,
                                                                  chassis->param.Vw_in);
     }

    /* 起步/停车瞬态补偿：在短时窗口抑制惯量扰动 */
    chassis->param.Vx_in += ChassisTransientComp_Update(chassis->param.Vx_in,
                                                        chassis->param.Vy_in,
                                                        chassis->param.Vw_in);

    /* 逐轴限幅：限制指令变化率，降低起步/变向打滑导致的漂移 */
    chassis->param.Vy_in = ChassisAxisLimiter_Update((ChassisAxisLimiter *)&g_vy_limiter, chassis->param.Vy_in);
    chassis->param.Vw_in = ChassisAxisLimiter_Update((ChassisAxisLimiter *)&g_vw_limiter, chassis->param.Vw_in);
    chassis->param.Vx_in = ChassisAxisLimiter_Update((ChassisAxisLimiter *)&g_vx_limiter, chassis->param.Vx_in);
    
    /* 输出到电机 */
    chassis->param.V_out[0] = chassis->param.Vx_in + chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[1] = chassis->param.Vx_in - chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[2] = chassis->param.Vx_in + chassis->param.Vy_in - chassis->param.Vw_in;
    chassis->param.V_out[3] = chassis->param.Vx_in - chassis->param.Vy_in - chassis->param.Vw_in;
}

/* 停止底盘 */
void Chassis_Stop(Chassis_Module *chassis)
{
    /* 将速度输入与输出清零 */
    chassis->param.Vx_in = 0.0f;
    chassis->param.Vy_in = 0.0f;
    chassis->param.Vw_in = 0.0f;
    chassis->param.V_out[0] = 0.0f;
    chassis->param.V_out[1] = 0.0f;
    chassis->param.V_out[2] = 0.0f;
    chassis->param.V_out[3] = 0.0f;

    /* PID输出清零，防止残留 */
    chassis_motor1.pid_spd.Output = 0.0f;
    chassis_motor2.pid_spd.Output = 0.0f;
    chassis_motor3.pid_spd.Output = 0.0f;
    chassis_motor4.pid_spd.Output = 0.0f;

    guide_motor1.pid_spd.Output = 0.0f;
    guide_motor2.pid_spd.Output = 0.0f;
}




//// 向左旋转90度
//Chassis_Rotate90(ROTATE_LEFT_90);

//// 向右旋转90度
//Chassis_Rotate90(ROTATE_RIGHT_90);

//// 停止旋转
//Chassis_Rotate90(ROTATE_NONE);
