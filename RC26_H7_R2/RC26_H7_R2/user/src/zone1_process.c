/**
 * @file    zone1_process.c
 * @brief   I区整流程控制模块 - 工业级规范实现（内部实现）
 */

#include "zone1_process.h"
#include "Sensor_Task.h"
#include "chassis.h"
#include "weapon.h"
#include "sensor.h"
#include "clamp_head.h"
#include "tim.h"
#include <math.h>
#include <stdlib.h>
#include "new_remote_control.h"

/* ==================== 内部结构体定义 ==================== */
struct Zone1_Handle_Struct {
    Zone1_Config_t config;
    Zone1_State_t state;
    Zone1_State_t next_state;
    Zone1_State_t last_state;
    uint32_t tick;
    uint8_t initialized;
    uint8_t moving_flag;
    float right_move_start_y;
    Zone1_Status_t status;
};

/* ==================== 静态变量声明 ==================== */
static Zone1_Handle_t g_zone1_handle = NULL;

/* ==================== 辅助函数声明 ==================== */
static float Zone1_GetChassisSpeed(void);
static void Zone1_InternalReset(Zone1_Handle_t handle);

/* ==================== 新接口函数实现（带句柄） ==================== */

Zone1_Error_t Zone1_Init_Handle(const Zone1_Config_t* config, Zone1_Handle_t* handle)
{
    if (config == NULL || handle == NULL) return ZONE1_ERR_NULL_PTR;
    if (config->position_threshold <= 0.0f) return ZONE1_ERR_INVALID_PARAM;
    if (config->forward_speed1 <= 0.0f || config->forward_speed2 <= 0.0f) return ZONE1_ERR_INVALID_PARAM;
    if (config->photo_switch_port == NULL) return ZONE1_ERR_INVALID_PARAM;
    
    Zone1_Handle_t h = (Zone1_Handle_t)malloc(sizeof(struct Zone1_Handle_Struct));
    if (h == NULL) return ZONE1_ERR_HW_FAILURE;
    
    h->config = *config;
    Zone1_InternalReset(h);
    h->initialized = 1;
    *handle = h;
    g_zone1_handle = h;
    
    return ZONE1_OK;
}

Zone1_Error_t Zone1_DeInit_Handle(Zone1_Handle_t handle)
{
    if (handle == NULL) return ZONE1_ERR_NULL_PTR;
    Zone1_Stop_Handle(handle);
    free(handle);
    g_zone1_handle = NULL;
    return ZONE1_OK;
}

Zone1_Error_t Zone1_Start_Handle(Zone1_Handle_t handle)
{
    if (handle == NULL) return ZONE1_ERR_NULL_PTR;
    if (!handle->initialized) return ZONE1_ERR_NOT_INIT;
    if (handle->state != ZONE1_STATE_IDLE && handle->state != ZONE1_STATE_COMPLETE && 
        handle->state != ZONE1_STATE_ERROR) {
        return ZONE1_ERR_ALREADY_RUNNING;
    }
    
    handle->state = ZONE1_STATE_START_CLAMP;
    handle->next_state = ZONE1_STATE_START_CLAMP;
    handle->tick = HAL_GetTick();
    clamp_head_init();
    
    return ZONE1_OK;
}

Zone1_Error_t Zone1_Stop_Handle(Zone1_Handle_t handle)
{
    if (handle == NULL) return ZONE1_ERR_NULL_PTR;
    if (!handle->initialized) return ZONE1_ERR_NOT_INIT;
    
    Chassis_Stop(&Chassis);
    Zone1_InternalReset(handle);
    clamp_head_init();
    
    return ZONE1_OK;
}

Zone1_Error_t Zone1_Process_Handle(Zone1_Handle_t handle)
{
    uint32_t now;
    
    if (handle == NULL) return ZONE1_ERR_NULL_PTR;
    if (!handle->initialized) return ZONE1_ERR_NOT_INIT;
    
    now = HAL_GetTick();
    
    handle->status.current_state = handle->state;
    handle->status.elapsed_ms = now - handle->tick;
    handle->status.photo_switch_state = HAL_GPIO_ReadPin(
        handle->config.photo_switch_port, handle->config.photo_switch_pin);
    handle->status.laser_distance_mm = laser1.distance;
    handle->status.chassis_speed_sum = Zone1_GetChassisSpeed();
    
    if (rc_odom_is_valid()) {
        const rc_odom_t *odom = rc_get_latest_odom();
        handle->status.odom_x = odom->x;
        handle->status.odom_y = odom->y;
        handle->status.odom_yaw = odom->yaw;
    }
    
    clamp_head_auto_process();
    
    switch (handle->state) {
        case ZONE1_STATE_IDLE:
            break;
            
        case ZONE1_STATE_START_CLAMP: {
            ClampHead_StartSequence();
            if ((now - handle->tick) > handle->config.step1_delay_ms) {
                handle->next_state = ZONE1_STATE_FORWARD_RADAR;
                handle->tick = now;
            }
            break;
        }
        
        case ZONE1_STATE_FORWARD_RADAR: {
            Chassis.param.Vx_in = handle->config.forward_speed1;
            Chassis.param.Vy_in = 0.0f;
            Chassis.param.Vw_in = 0.0f;
            
            if (rc_odom_is_valid()) {
                const rc_odom_t *odom = rc_get_latest_odom();
                float dx = odom->x - handle->config.target_x;
                float dy = odom->y - handle->config.target_y;
                float dist = sqrtf(dx*dx + dy*dy);
                if (dist < handle->config.position_threshold) {
                    Chassis.param.Vx_in = 0;
                    handle->next_state = ZONE1_STATE_ROTATE_LEFT_90;
                    handle->tick = now;
                    break;
                }
            }
            
            if ((now - handle->tick) > handle->config.step2_timeout_ms) {
                Chassis.param.Vx_in = 0;
                handle->next_state = ZONE1_STATE_ERROR;
                break;
            }
            break;
        }
        
        case ZONE1_STATE_ROTATE_LEFT_90: {
            if (rot.enable == 0) {
                rot.enable = 1;
                rot.dir = -1;
                rot.start_yaw = g_imu_yaw_deg;
                rot.target_yaw = rot_wrap_deg(rot.start_yaw - 90.0f);
                handle->status.rot_start_yaw = rot.start_yaw;
                handle->status.rot_target_yaw = rot.target_yaw;
            }
            
            float err = rot_wrap_deg(rot.target_yaw - g_imu_yaw_deg);
            handle->status.rot_error = err;
            if (fabsf(err) < 1.0f) {
                rot.enable = 0;
                handle->next_state = ZONE1_STATE_FORWARD_LIMIT;
                handle->tick = now;
            }
            
            if ((now - handle->tick) > handle->config.step3_timeout_ms) {
                rot.enable = 0;
                Chassis.param.Vx_in = 0;
                handle->next_state = ZONE1_STATE_ERROR;
                break;
            }
            break;
        }
        
        case ZONE1_STATE_FORWARD_LIMIT: {
            Chassis.param.Vx_in = handle->config.forward_speed2;
            Chassis.param.Vy_in = 0.0f;
            Chassis.param.Vw_in = 0.0f;
            
            float speed = Zone1_GetChassisSpeed();
            
            if (speed > handle->config.limit_speed_start) {
                handle->moving_flag = 1;
            }
            
            if (handle->moving_flag && (speed < handle->config.limit_speed_stop)) {
                Chassis.param.Vx_in = 0;
                handle->next_state = ZONE1_STATE_RIGHT_MOVE_PHOTO;
                handle->tick = now;
                handle->moving_flag = 0;
                handle->right_move_start_y = handle->status.odom_y;
                break;
            }
            
            if ((now - handle->tick) > handle->config.step4_timeout_ms) {
                Chassis.param.Vx_in = 0;
                handle->next_state = ZONE1_STATE_ERROR;
                break;
            }
            break;
        }
        
        case ZONE1_STATE_RIGHT_MOVE_PHOTO: {
            if (handle->status.photo_switch_state == 1) {
                Chassis.param.Vx_in = 0;
                Chassis.param.Vy_in = 0;
                ClampHead_TriggerClose();
                handle->next_state = ZONE1_STATE_WAIT_CLAMP_COMPLETE;
                handle->tick = now;
                break;
            }
            
            Chassis.param.Vx_in = 0.0f;
            Chassis.param.Vy_in = handle->config.right_speed;
            Chassis.param.Vw_in = 0.0f;
            
            handle->status.right_move_distance = handle->status.odom_y - handle->right_move_start_y;
            
            if ((now - handle->tick) > handle->config.step5_timeout_ms) {
                Chassis.param.Vx_in = 0;
                Chassis.param.Vy_in = 0;
                handle->next_state = ZONE1_STATE_ERROR;
                break;
            }
            break;
        }
        
        case ZONE1_STATE_WAIT_CLAMP_COMPLETE: {
            if (clamp_head_state == CLAMP_HEAD_IDLE) {
                handle->next_state = ZONE1_STATE_COMPLETE;
            }
            break;
        }
        
        case ZONE1_STATE_COMPLETE:
        case ZONE1_STATE_ERROR: {
            Chassis.param.Vx_in = 0;
            Chassis.param.Vy_in = 0;
            Chassis.param.Vw_in = 0;
            break;
        }
        
        default: {
            handle->next_state = ZONE1_STATE_ERROR;
            break;
        }
    }
    
    if (handle->next_state != handle->state) {
        handle->last_state = handle->state;
        handle->status.last_state = handle->last_state;
        handle->state = handle->next_state;
        handle->tick = now;
    }
    
    return ZONE1_OK;
}

Zone1_Error_t Zone1_GetStatus_Handle(Zone1_Handle_t handle, Zone1_Status_t* status)
{
    if (handle == NULL || status == NULL) return ZONE1_ERR_NULL_PTR;
    if (!handle->initialized) return ZONE1_ERR_NOT_INIT;
    *status = handle->status;
    return ZONE1_OK;
}

Zone1_State_t Zone1_GetState_Handle(Zone1_Handle_t handle)
{
    if (handle == NULL || !handle->initialized) return ZONE1_STATE_ERROR;
    return handle->state;
}

Zone1_Error_t Zone1_GetConfig_Handle(Zone1_Handle_t handle, Zone1_Config_t* config)
{
    if (handle == NULL || config == NULL) return ZONE1_ERR_NULL_PTR;
    if (!handle->initialized) return ZONE1_ERR_NOT_INIT;
    
    *config = handle->config;
    return ZONE1_OK;
}

Zone1_Handle_t Zone1_GetGlobalHandle(void)
{
    return g_zone1_handle;
}

Zone1_Error_t Zone1_Init_Default(void)
{
    Zone1_Config_t config = {
        .target_x = 0.4f,
        .target_y = 0.0f,
        .position_threshold = 0.15f,
        .forward_speed1 = 0.8f,
        .forward_speed2 = 0.5f,
        .right_speed = 0.3f,
        .step1_delay_ms = 200,
        .step2_timeout_ms = 5000,
        .step3_timeout_ms = 3000,
        .step4_timeout_ms = 5000,
        .step5_timeout_ms = 10000,
        .limit_speed_start = 10.0f,
        .limit_speed_stop = 3.0f,
        .photo_switch_port = GPIOE,
        .photo_switch_pin = GPIO_PIN_9
    };
    
    Zone1_Error_t err = Zone1_Init_Handle(&config, &g_zone1_handle);
    if (err != ZONE1_OK) {
        return err;
    }
    
    return Zone1_Start_Handle(g_zone1_handle);
}

/* ==================== 辅助函数实现 ==================== */

static float Zone1_GetChassisSpeed(void)
{
    return fabsf(chassis_motor1.speed_rpm) +
           fabsf(chassis_motor2.speed_rpm) +
           fabsf(chassis_motor3.speed_rpm) +
           fabsf(chassis_motor4.speed_rpm);
}

static void Zone1_InternalReset(Zone1_Handle_t handle)
{
    if (handle == NULL) return;
    
    handle->state = ZONE1_STATE_IDLE;
    handle->next_state = ZONE1_STATE_IDLE;
    handle->last_state = ZONE1_STATE_IDLE;
    handle->tick = 0;
    handle->moving_flag = 0;
    handle->right_move_start_y = 0.0f;
    
    handle->status.current_state = ZONE1_STATE_IDLE;
    handle->status.last_state = ZONE1_STATE_IDLE;
    handle->status.elapsed_ms = 0;
    handle->status.laser_distance_mm = 0;
    handle->status.photo_switch_state = 0;
    handle->status.chassis_speed_sum = 0.0f;
    handle->status.odom_x = 0.0f;
    handle->status.odom_y = 0.0f;
    handle->status.odom_yaw = 0.0f;
    handle->status.rot_start_yaw = 0.0f;
    handle->status.rot_target_yaw = 0.0f;
    handle->status.rot_error = 0.0f;
    handle->status.right_move_distance = 0.0f;
}
