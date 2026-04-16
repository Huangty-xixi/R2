#include "chassis.h"
#include "Motion_Task.h"
#include "lift.h"
#include "master_control.h"

Chassis_Module Chassis;


//ï¿½ï¿½ï¿½ï¿½
DJI_MotorModule chassis_motor1;  // ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ï¿½
DJI_MotorModule chassis_motor2;  // ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ï¿½
DJI_MotorModule chassis_motor3;  // ï¿½ï¿½ï¿½ï¿½ï¿?
DJI_MotorModule chassis_motor4;  // ï¿½ï¿½ï¿½Òºï¿½

//ï¿½ï¿½ï¿½ï¿½
DJI_MotorModule guide_motor1;  // ï¿½ï¿½ï¿½ï¿½
DJI_MotorModule guide_motor2;  // ï¿½ï¿½ï¿½Ò£ï¿½
 


//ï¿½î¶¯ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×´Ì¬ï¿½ï¿½×´Ì?
static uint8_t flexible_motor_has_stopped = 0;
static uint8_t flexible_motor_running = 0;

/* ï¿½ï¿½Ç°ï¿½ï¿½ï¿½Øµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½î»ºï¿½æ£¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ô¤ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½É½ï¿½ï¿½×´ï¿½ï¿½ï¿½Ô£ï¿? */
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

    /* masterÄ£Ê½ÏÂÖ±½ÓÐ´µ×ÅÌÊäÈë£¬²»ÔÙ½èÓÃRCÍ¨µÀÖµ */
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
  * @brief ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ß¼ï¿½
  */
void manual_chassis_function(void)
{
    static MasterLevelGate master_chassis_flex_gate = {0U, 0U};

    /* ï¿½ï¿½ï¿½ï¿½Ä£Ê½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ì¶ï¿½ï¿½ï¿½ï¿½Ö½Ú½ï¿½ï¿½ï¿½Ô¤ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Úºï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×´ï¿½ï¿½ï¿½ï¿? */
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
		
    // ½öÒ£¿ØÄ£Ê½´ÓRCÍ¨µÀ¶ÁÈ¡£¬masterÄ£Ê½ÊäÈëÓÉchassis_apply_master_motionÖ±½Ó¸ø¶¨
    if (control_mode == remote_control && remote_mode == chassis_mode) {
        chassis->param.Accel = ACCEL;
        chassis->param.Vw_in = LR_TRANSLATION;
        chassis->param.Vy_in = FB_TRANSLATION;
        chassis->param.Vx_in = ROTATION;
    }
    
    chassis->param.V_out[0] = chassis->param.Vx_in + chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[1] = chassis->param.Vx_in - chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[2] = chassis->param.Vx_in + chassis->param.Vy_in - chassis->param.Vw_in;
    chassis->param.V_out[3] = chassis->param.Vx_in - chassis->param.Vy_in - chassis->param.Vw_in;

}

void Chassis_Stop(Chassis_Module *chassis)
{
    // 2. ï¿½ï¿½ï¿½Ðµï¿½ï¿½Ä¿ï¿½ï¿½Ç¿ï¿½ï¿? = 0
    chassis->param.Vx_in = 0.0f;
    chassis->param.Vy_in = 0.0f;
    chassis->param.Vw_in = 0.0f;
    chassis->param.V_out[0] = 0.0f;
    chassis->param.V_out[1] = 0.0f;
    chassis->param.V_out[2] = 0.0f;
    chassis->param.V_out[3] = 0.0f;

    // 3. PID ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ã£¨ï¿½ï¿½Ø¼ï¿½ï¿½ï¿½ï¿½ï¿½
    chassis_motor1.pid_spd.Output = 0.0f;
    chassis_motor2.pid_spd.Output = 0.0f;
    chassis_motor3.pid_spd.Output = 0.0f;
    chassis_motor4.pid_spd.Output = 0.0f;

    guide_motor1.pid_spd.Output = 0.0f;
    guide_motor2.pid_spd.Output = 0.0f;
}


