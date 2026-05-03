#include "zone1_process.h"
#include "Sensor_Task.h"
#include "chassis.h"
#include "weapon.h"
#include "sensor.h"
#include "clamp_head.h"
#include "tim.h"
#include <math.h>
#include "new_remote_control.h"

// 目标坐标（根据场地实际测量调整）
static float zone1_target_x = 3.0f;   // I区入口X坐标
static float zone1_target_y = 2.0f;   // I区入口Y坐标
static float zone1_position_threshold = 0.15f; // 位置到达阈值（米）

// ==================== 内部状态变量（封装）====================
Zone1_State_t zone1_state = ZONE1_IDLE;
static Zone1_State_t zone1_next_state = ZONE1_IDLE;
static uint32_t zone1_tick = 0;

// ==================== 调试用的变量导出（只读）====================
uint32_t zone1_tick_debug = 0;
uint8_t zone1_moving_flag_debug = 0;

// ==================== 可调参数 ====================
uint32_t zone1_radar_stop_distance_mm = 150;   // 雷达停止距离
float zone1_forward_speed1 = 0.8f;             // 第一阶段前进速度
float zone1_forward_speed2 = 0.5f;             // 第二阶段前进速度
float zone1_left_right_speed = 0.5f;           // 左右移动速度

// ==================== 辅助函数：复位 ====================
static void Zone1_Internal_Reset(void)
{
    zone1_state = ZONE1_IDLE;
    zone1_next_state = ZONE1_IDLE;
    zone1_tick = 0;
    zone1_tick_debug = 0;
    zone1_moving_flag_debug = 0;
    // 也把自动夹枪模块复位
    clamp_head_init();
}

// ==================== 接口：初始化 ====================
void Zone1_Init(void)
{
    Zone1_Internal_Reset();
}

// ==================== 接口：开始流程 ====================
void Zone1_Start(void)
{
    zone1_state = ZONE1_START_CLAMP;
    zone1_next_state = ZONE1_START_CLAMP;
    zone1_tick = HAL_GetTick();
}

// ==================== 接口：停止/重置 ====================
void Zone1_Stop(void)
{
    Zone1_Internal_Reset();
    Chassis_Stop(&Chassis);
}

// ==================== 辅助：读取底盘总速度（用于限位检测）====================
static float get_chassis_speed(void)
{
    // 用4个电机转速之和来判断底盘是否在动
    float v = fabsf(chassis_motor1.speed_rpm) +
              fabsf(chassis_motor2.speed_rpm) +
              fabsf(chassis_motor3.speed_rpm) +
              fabsf(chassis_motor4.speed_rpm);
    return v;
}

// ==================== 主循环：在Can_Task里调用 ====================
void Zone1_Process(void)
{
    uint32_t now = HAL_GetTick();
    GPIO_PinState photo_switch = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);

    // 更新调试信息
    zone1_tick_debug = now - zone1_tick;

    // 自动夹枪模块先跑（管理夹爪和舵机）
    clamp_head_auto_process();

    switch (zone1_state)
    {
        // ==================== IDLE ====================
        case ZONE1_IDLE:
            // 等待触发，什么也不做
            break;

        // ==================== 步骤1：进入自动夹枪模式 ====================
        case ZONE1_START_CLAMP:
        {
            // 调用clamp_head启动夹爪序列（打开夹爪+舵机水平）
            ClampHead_StartSequence();

            // 等待一小会儿后进入下一步
            if ((now - zone1_tick) > 200)
            {
                zone1_next_state = ZONE1_FORWARD_RADAR;
                zone1_tick = now;
            }
            break;
        }

        // ==================== 步骤2：底盘前进，坐标判断停止 ====================
        case ZONE1_FORWARD_RADAR:
        {
            // 底盘控制
            Chassis.param.Vx_in = zone1_forward_speed1;
            Chassis.param.Vy_in = 0.0f;
            Chassis.param.Vw_in = 0.0f;

            // 停止条件：到达目标坐标
            if (rc_odom_is_valid())
            {
                const rc_odom_t *odom = rc_get_latest_odom();
                float dx = odom->x - zone1_target_x;
                float dy = odom->y - zone1_target_y;
                float dist = sqrtf(dx*dx + dy*dy);
                if (dist < zone1_position_threshold)
                {
                    Chassis.param.Vx_in = 0;
                    zone1_next_state = ZONE1_ROTATE_LEFT_90;
                    zone1_tick = now;
                    break;
                }
            }

            // 超时保护（5秒）
            if ((now - zone1_tick) > 5000)
            {
                Chassis.param.Vx_in = 0;
                zone1_next_state = ZONE1_ERROR;
                break;
            }
            break;
        }

        // ==================== 步骤3：左旋转90度 ====================
        case ZONE1_ROTATE_LEFT_90:
        {
            // 第一次进这个状态时，初始化旋转
            if (rot.enable == 0)
            {
                rot.enable = 1;
                rot.dir = -1;
                rot.start_yaw = g_imu_yaw_deg;
                rot.target_yaw = rot_wrap_deg(rot.start_yaw - 90.0f);
            }

            // 检测是否完成
            float err = rot_wrap_deg(rot.target_yaw - g_imu_yaw_deg);
            if (fabsf(err) < 1.0f)
            {
                rot.enable = 0;
                zone1_next_state = ZONE1_FORWARD_LIMIT;
                zone1_tick = now;
            }

            // 超时保护（3秒）
            if ((now - zone1_tick) > 3000)
            {
                rot.enable = 0;
                Chassis.param.Vx_in = 0;
                zone1_next_state = ZONE1_ERROR;
                break;
            }
            break;
        }

        // ==================== 步骤4：底盘继续前进，限位停 ====================
        case ZONE1_FORWARD_LIMIT:
        {
            // 继续前进
            Chassis.param.Vx_in = zone1_forward_speed2;
            Chassis.param.Vy_in = 0.0f;
            Chassis.param.Vw_in = 0.0f;

            // 限位检测逻辑（类似lift.c中的逻辑）
            float speed = get_chassis_speed();

            // 1. 有速度 -> 标记
            if (speed > 10.0f)
            {
                zone1_moving_flag_debug = 1;
            }

            // 2. 曾经有速度，现在没速度了 -> 碰到限位
            if (zone1_moving_flag_debug && (speed < 3.0f))
            {
                Chassis.param.Vx_in = 0;
                zone1_next_state = ZONE1_LR_PHOTO_SWITCH;
                zone1_tick = now;
                zone1_moving_flag_debug = 0;
                break;
            }

            // 超时保护（5秒）
            if ((now - zone1_tick) > 5000)
            {
                Chassis.param.Vx_in = 0;
                zone1_next_state = ZONE1_ERROR;
                break;
            }
            break;
        }

        // ==================== 步骤5：左右移动，光电触发夹枪 ====================
        case ZONE1_LR_PHOTO_SWITCH:
        {
            // 光电开关触发
            if (photo_switch == 1)
            {
                // 停止底盘
                Chassis.param.Vx_in = 0;
                Chassis.param.Vy_in = 0;

                // 调用clamp_head触发夹爪闭合（会执行：闭合+200ms后舵机直立）
                ClampHead_TriggerClose();

                // 进入等待夹枪完成状态
                zone1_next_state = ZONE1_WAIT_CLAMP_COMPLETE;
                zone1_tick = now;
                break;
            }

            // 左右摆动控制（每500ms换一次方向）
            static int8_t dir = 1;
            if ((now - zone1_tick) > 500)
            {
                dir = -dir;
                zone1_tick = now;
            }

            Chassis.param.Vx_in = 0.0f;
            Chassis.param.Vy_in = (float)dir * zone1_left_right_speed;
            Chassis.param.Vw_in = 0.0f;

            // 超时保护（10秒）
            if ((now - zone1_tick) > 10000)
            {
                Chassis.param.Vx_in = 0;
                Chassis.param.Vy_in = 0;
                zone1_next_state = ZONE1_ERROR;
                break;
            }
            break;
        }

        // ==================== 等待夹枪完成 ====================
        case ZONE1_WAIT_CLAMP_COMPLETE:
        {
            // 等待clamp_head_auto_process完成夹枪流程
            // （它在CLAMP_HEAD_WAIT_CLOSE状态会等待200ms然后舵机直立）
            // 判断是否完成（clamp_head_state回到IDLE）
            if (clamp_head_state == CLAMP_HEAD_IDLE)
            {
                zone1_next_state = ZONE1_COMPLETE;
            }
            break;
        }

        // ==================== 完成/错误 ====================
        case ZONE1_COMPLETE:
        case ZONE1_ERROR:
        {
            // 保持停止
            Chassis.param.Vx_in = 0;
            Chassis.param.Vy_in = 0;
            Chassis.param.Vw_in = 0;
            break;
        }

        default:
            break;
    }

    // ==================== 状态转换 ====================
    if (zone1_next_state != zone1_state)
    {
        zone1_state = zone1_next_state;
        zone1_tick = now;
    }
}
