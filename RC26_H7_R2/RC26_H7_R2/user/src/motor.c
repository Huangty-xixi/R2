#include "motor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "lift.h"
#include "kfs.h"

#define MOTOR_OVERTEMP_TEST_ENABLE 0

static void MotorModule_Init(baseModule *base) {
    MotorModule *Motor = (MotorModule *)base;
    BaseModule_Init(base); // 调用基类初始化
    
    base->type = motor;
}

void MotorModule_Run(baseModule *base)
{
    MotorModule *Motor = (MotorModule*)base;
}

void MotorModule_Create(MotorModule *obj, uint8_t motor_id, FDCAN_HandleTypeDef *hcan, Motor_Model model, Ctrl_mode mode) {
    obj->model = model;
    obj->mode = mode;
    obj->id = motor_id;
    
    obj->base.Init = MotorModule_Init;
    obj->base.Run = MotorModule_Run;
    obj->base.Stop = BaseModule_Stop;
    obj->base.GetState = BaseModule_GetState;
    obj->base.ClearError = BaseModule_ClearError;
    
    obj->hcan = *hcan;
}

/* 过温阈值（可按实测再调） */
static const uint8_t DJI_TEMP_TRIP = 60U;
static const uint8_t DJI_TEMP_RECOVER = 60U;
static const uint8_t DM_MOS_TEMP_TRIP = 60U;    
static const uint8_t DM_MOS_TEMP_RECOVER = 60U;
static const uint8_t DM_ROTOR_TEMP_TRIP = 80U;
static const uint8_t DM_ROTOR_TEMP_RECOVER = 80U;

static uint8_t g_overtemp_latched = 0U;

static uint8_t dji_overtemp_trip(const DJI_MotorModule *m)
{
    return (m->temp >= DJI_TEMP_TRIP) ? 1U : 0U;
}

static uint8_t dji_overtemp_recover(const DJI_MotorModule *m)
{
    return (m->temp <= DJI_TEMP_RECOVER) ? 1U : 0U;
}

static uint8_t dm_overtemp_trip(const DM_MotorModule *m)
{
    if (m->temp_mos >= DM_MOS_TEMP_TRIP) return 1U;
    if (m->temp_rotor >= DM_ROTOR_TEMP_TRIP) return 1U;
    return 0U;
}

static uint8_t dm_overtemp_recover(const DM_MotorModule *m)
{
    if (m->temp_mos > DM_MOS_TEMP_RECOVER) return 0U;
    if (m->temp_rotor > DM_ROTOR_TEMP_RECOVER) return 0U;
    return 1U;
}

uint8_t Motor_OverTempProtect_Update(void)
{
    uint8_t any_trip = 0U;
    uint8_t all_recover = 1U;

    /* DJI电机 */
    const DJI_MotorModule *dji_list[] = {
        &chassis_motor1, &chassis_motor2, &chassis_motor3, &chassis_motor4,
        &guide_motor1, &guide_motor2,
        &flexible_motor1, &flexible_motor2,
        &kfs_above, &kfs_below
    };
    uint32_t i = 0U;
    for (i = 0U; i < (sizeof(dji_list) / sizeof(dji_list[0])); i++)
    {
        if (dji_overtemp_trip(dji_list[i]) != 0U) any_trip = 1U;
        if (dji_overtemp_recover(dji_list[i]) == 0U) all_recover = 0U;
    }

    /* DM电机 */
    const DM_MotorModule *dm_list[] = {
        &R2_lift_motor_left, &R2_lift_motor_right,
        &main_lift, &kfs_spin, &three_kfs
    };
    for (i = 0U; i < (sizeof(dm_list) / sizeof(dm_list[0])); i++)
    {
        if (dm_overtemp_trip(dm_list[i]) != 0U) any_trip = 1U;
        if (dm_overtemp_recover(dm_list[i]) == 0U) all_recover = 0U;
    }

    if (any_trip != 0U)
    {
        g_overtemp_latched = 1U;
    }
    else if (g_overtemp_latched != 0U && all_recover != 0U)
    {
        g_overtemp_latched = 0U;
    }

    return g_overtemp_latched;
}

volatile uint8_t g_overtemp_test_step = 0U;
volatile uint8_t g_overtemp_test_result = 0U;

void Motor_OverTemp_SimpleTest(void)
{
#if MOTOR_OVERTEMP_TEST_ENABLE
    static uint32_t step_tick_ms = 0U;
    uint32_t now_ms = HAL_GetTick();

    if (step_tick_ms == 0U)
    {
        step_tick_ms = now_ms;
    }

    if (g_overtemp_test_step == 0U)
    {
        chassis_motor1.temp = 60U;
        R2_lift_motor_left.temp_mos = 60U;
        R2_lift_motor_left.temp_rotor = 60U;
        g_overtemp_test_result = Motor_OverTempProtect_Update(); /* expect 0 */
        if ((uint32_t)(now_ms - step_tick_ms) >= 5000U)
        {
            g_overtemp_test_step = 1U;
            step_tick_ms = now_ms;
        }
        return;
    }

    if (g_overtemp_test_step == 1U)
    {
        chassis_motor1.temp = 90U;
        R2_lift_motor_left.temp_mos = 90U;
        R2_lift_motor_left.temp_rotor = 90U;
        g_overtemp_test_result = Motor_OverTempProtect_Update(); /* expect 1 */
        if ((uint32_t)(now_ms - step_tick_ms) >= 5000U)
        {
            g_overtemp_test_step = 2U;
            step_tick_ms = now_ms;
        }
        return;
    }

    if (g_overtemp_test_step == 2U)
    {
        chassis_motor1.temp = 60U;
        R2_lift_motor_left.temp_mos = 60U;
        R2_lift_motor_left.temp_rotor = 60U;
        g_overtemp_test_result = Motor_OverTempProtect_Update(); /* expect 0 */
        if ((uint32_t)(now_ms - step_tick_ms) >= 5000U)
        {
            g_overtemp_test_step = 3U;
            step_tick_ms = now_ms;
        }
        return;
    }
#else
    g_overtemp_test_result = 0U;
#endif
}
