/**
 * @file rc_nav_goto.h
 * @brief 基于上位机/雷达里程计（与 @ref rc_odom_t 同坐标系）的平面点到点导航。
 *
 * 在控制周期内调用 @ref RcNavGoto_Run：内部根据位姿误差计算底盘速度指令，并通过回调
 * @ref RcNavGoto_Config::apply_wheel_inputs 输出到车轮（与工程内 @c Chassis.param 三轴
 * 约定一致：Vx=旋转通道，Vy=前后，Vw=左右平移）。
 *
 * @note 工作区为不透明字节数组，由调用方静态分配并清零；不在本头文件暴露内部状态机成员。
 * @note 所有硬件/底盘接线必须在 @ref RcNavGoto_Config 的函数指针中完成，本模块不直接访问外设。
 *
 * @par 使用方法（集成步骤）
 * 1. 准备一块静态存储：@code
 *    static RcNavGoto_Workspace s_nav_ws;
 *    @endcode
 *    首次使用前对整块内存置零（如 @c memset(&s_nav_ws, 0, sizeof(s_nav_ws)) 或静态区默认零初始化）。
 *
 * 2. 填写 @ref RcNavGoto_Config：\n
 *    - @c platform_user：传给各回调的上下文（可为 NULL）。\n
 *    - @c get_time_ms：单调毫秒，可与 @c HAL_GetTick 或 @c osKernelGetTickCount 一致。\n
 *    - @c read_pose：读当前位姿；典型实现内调用 @c RcOdomSnap_Read(&odom)，再把
 *      @c odom.x / odom.y / odom.yaw（度）写入输出指针；失败返回非 0。\n
 *    - @c apply_wheel_inputs：把三轴指令接到底盘；本工程可在 @c semi_auto_control 且
 *      @c remote_mode == chassis_mode 时，写 @c process_flow_chassis_override（三轴 mask
 *      均置位）使 @c Chassis_Calc 采用覆盖值，字段与 @c Chassis.param 一致（Vx/Vy/Vw）。\n
 *    - @c apply_wheel_stop：停车时清零覆盖或调用 @c Chassis_Stop 等，避免残留速度。\n
 *    - 按实车标定 @c kp_xy/ki_xy/kd_xy、@c kp_yaw/ki_yaw/kd_yaw 与 @c vmax_*、容差、@c timeout_ms。
 *
 * 3. 每控制周期构造 @ref RcNavGoto_Target：\n
 *    - @c x_m / @c y_m：目标点（米），必须与 ODOM 同一世界系。\n
 *    - @c lock_final_yaw 为 0：仅到位判定位置；非 0：到位后仍以 @c yaw_deg 对准航向。\n
 *    - 更换目标或重新发令时递增 @c session_id，以清空内部积分并重启超时计时。
 *
 * 4. 周期调用：@code
 *    RcNavGoto_Status st;
 *    RcNavGoto_Error e = RcNavGoto_Run(&s_nav_ws, &s_nav_cfg, &s_nav_target, &st);
 *    @endcode
 *    - @ref RC_NAV_GOTO_ERR_OK_MOVING：继续下一周期调用。\n
 *    - @ref RC_NAV_GOTO_ERR_OK_ARRIVED：已到达并已执行 @c apply_wheel_stop，可保持调用或停发导航。\n
 *    - @ref RC_NAV_GOTO_ERR_TIMEOUT：超时已停车，需更新 @c session_id 或目标后重试。\n
 *    - 其它错误码：检查空指针、配置、里程计回调返回值。
 *
 * 5. 运行模式：需保证 @c apply_wheel_inputs 写入的指令在当拍不会被其它逻辑覆盖；
 *    若使用流程覆盖底盘，勿与其它模块同时写冲突轴。
 *
 * @date&author 2026/5/4 Hty
 */
#ifndef RC_NAV_GOTO_H
#define RC_NAV_GOTO_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 工作区最小字节数（ sizeof 内部实现，由 .c 中静态断言保证） */
#define RC_NAV_GOTO_WORKSPACE_SIZE 96u

/**
 * @brief 不透明工作区：仅作为字节占位，禁止按业务字段解释其内容。
 */
typedef struct RcNavGoto_Workspace {
    uint8_t opaque[RC_NAV_GOTO_WORKSPACE_SIZE];
} RcNavGoto_Workspace;

/**
 * @brief 模块返回码（所有可失败 API 统一使用）。
 */
typedef enum RcNavGoto_Error {
    /** 已到达目标（本周期已调用 @ref RcNavGoto_Config::apply_wheel_stop） */
    RC_NAV_GOTO_ERR_OK_ARRIVED = 0,
    /** 正常趋近中，下一周期应继续调用 */
    RC_NAV_GOTO_ERR_OK_MOVING = 1,
    RC_NAV_GOTO_ERR_NULL_POINTER = 2,
    RC_NAV_GOTO_ERR_BAD_CONFIG = 3,
    RC_NAV_GOTO_ERR_ODOM_READ = 4,
    RC_NAV_GOTO_ERR_TIMEOUT = 5,
} RcNavGoto_Error;

/**
 * @brief 本次调用时的运行摘要（可选输出，指针可为 NULL）。
 */
typedef struct RcNavGoto_Status {
    float distance_to_target_m;
    float yaw_error_deg;
    uint8_t at_xy;
    uint8_t at_yaw;
} RcNavGoto_Status;

/**
 * @brief 目标点（与里程计同一世界坐标系，单位：米、度）。
 */
typedef struct RcNavGoto_Target {
    float x_m;
    float y_m;
    /** 终端期望航向（度）；仅当 @ref lock_final_yaw 非 0 时在到位后继续对准。 */
    float yaw_deg;
    /** 非 0：到达位置容差后仍以 @ref yaw_deg 为准进行航向精调。 */
    uint8_t lock_final_yaw;
    /**
     * 非 0 且与上次调用不同时：清零内部积分/滤波状态，并重新开始超时计时。
     * 用于更换目标点或重新发令。
     */
    uint32_t session_id;
} RcNavGoto_Target;

/**
 * @brief 平台与调节器配置（每周期可传入相同常量区；不得为 NULL）。
 *
 * 时间基准由 @ref get_time_ms 提供，用于超时与微分。
 */
typedef struct RcNavGoto_Config {
    void *platform_user;

    /** @return 单调毫秒时间戳 */
    uint32_t (*get_time_ms)(void *platform_user);

    /**
     * @brief 读取当前位姿（通常内部调用 @c RcOdomSnap_Read 等）。
     * @return 0 成功；非 0 表示本周期不可用（将返回 @ref RC_NAV_GOTO_ERR_ODOM_READ）。
     */
    int (*read_pose)(void *platform_user, float *x_m, float *y_m, float *yaw_deg);

    /**
     * @brief 写入底盘三轴速度指令（与 @c Chassis_Calc 输入一致）。
     * @param vx_rotation 旋转分量（工程内对应 @c Vx_in）
     * @param vy_forward  前后分量（@c Vy_in）
     * @param vw_strafe   左右平移（@c Vw_in）
     */
    void (*apply_wheel_inputs)(void *platform_user, float vx_rotation, float vy_forward, float vw_strafe);

    /** @brief 急停/清零底盘指令 */
    void (*apply_wheel_stop)(void *platform_user);

    float kp_xy;
    float ki_xy;
    float kd_xy;
    float kp_yaw;
    float ki_yaw;
    float kd_yaw;

    float vmax_forward;
    float vmax_strafe;
    float vmax_rotation;

    float position_tolerance_m;
    float yaw_tolerance_deg;

    uint32_t timeout_ms;
} RcNavGoto_Config;

/**
 * @brief 单周期导航解算与控制输出（本模块对外唯一过程入口）。
 *
 * 典型用法：在 FreeRTOS 任务或主循环中固定周期调用；首次对某工作区使用前将其
 * @ref RcNavGoto_Workspace 内存清零。
 *
 * @param[in,out] ws 不透明工作区
 * @param[in] cfg 配置与平台钩子
 * @param[in] target 目标位姿
 * @param[out] status 可选状态；可为 NULL
 *
 * @retval RC_NAV_GOTO_ERR_OK_MOVING 正在趋近
 * @retval RC_NAV_GOTO_ERR_OK_ARRIVED 已到达（已自动 @ref RcNavGoto_Config::apply_wheel_stop）
 * @retval RC_NAV_GOTO_ERR_TIMEOUT 超过 @ref RcNavGoto_Config::timeout_ms
 * @date&author 2026/5/4 Hty
 */
RcNavGoto_Error RcNavGoto_Run(RcNavGoto_Workspace *ws,
                              const RcNavGoto_Config *cfg,
                              const RcNavGoto_Target *target,
                              RcNavGoto_Status *status);

#ifdef __cplusplus
}
#endif

#endif /* RC_NAV_GOTO_H */
