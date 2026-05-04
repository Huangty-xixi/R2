/**
 * @file    zone1_process.h
 * @brief   I区整流程控制模块 - 工业级规范实现
 *
 * @attention
 * 本模块实现I区自动夹枪头整流程控制，包含以下步骤：
 * 1. 进入自动夹枪模式（打开夹爪、舵机水平）
 * 2. 底盘前进，坐标判断停止
 * 3. 左旋转90度
 * 4. 底盘继续前进，限位检测停止
 * 5. 向右缓慢平移，光电检测物体停止并触发夹枪
 * 6. 等待夹枪完成
 *
 * @author  Embedded Engineer
 * @date    2026
 * @version 1.0.0
 */

#ifndef ZONE1_PROCESS_H
#define ZONE1_PROCESS_H

/* ==================== 包含依赖 ==================== */
#include "main.h"

/* ==================== 错误码枚举 ==================== */
/**
 * @brief Zone1 错误码枚举
 */
typedef enum {
    ZONE1_OK                  = 0x00,    /**< 操作成功 */
    ZONE1_ERR_NULL_PTR        = 0x01,    /**< 空指针错误 */
    ZONE1_ERR_INVALID_PARAM   = 0x02,    /**< 参数无效 */
    ZONE1_ERR_NOT_INIT        = 0x03,    /**< 未初始化 */
    ZONE1_ERR_ALREADY_RUNNING = 0x04,    /**< 流程已在运行中 */
    ZONE1_ERR_TIMEOUT         = 0x05,    /**< 超时错误 */
    ZONE1_ERR_HW_FAILURE      = 0x06,    /**< 硬件故障 */
} Zone1_Error_t;

/* ==================== 状态枚举 ==================== */
/**
 * @brief Zone1 流程状态枚举
 */
typedef enum {
    ZONE1_STATE_IDLE              = 0,    /**< 空闲状态 */
    ZONE1_STATE_START_CLAMP       = 1,    /**< 步骤1：进入自动夹枪模式 */
    ZONE1_STATE_FORWARD_RADAR     = 2,    /**< 步骤2：底盘前进，坐标判断停止 */
    ZONE1_STATE_ROTATE_LEFT_90    = 3,    /**< 步骤3：左旋转90度 */
    ZONE1_STATE_FORWARD_LIMIT     = 4,    /**< 步骤4：底盘继续前进，限位停 */
    ZONE1_STATE_RIGHT_MOVE_PHOTO  = 5,    /**< 步骤5：向右缓慢平移，光电检测物体停止 */
    ZONE1_STATE_WAIT_CLAMP_COMPLETE = 6,  /**< 步骤6：等待夹枪完成 */
    ZONE1_STATE_COMPLETE          = 7,    /**< 流程完成 */
    ZONE1_STATE_ERROR             = 8,    /**< 错误状态 */
} Zone1_State_t;

/* ==================== 配置结构体 ==================== */
/**
 * @brief Zone1 初始化配置结构体
 * @note  所有硬件资源和参数通过此结构体传入，实现配置解耦
 */
typedef struct {
    /* ========== 目标坐标参数 ========== */
    float target_x;                  /**< I区入口X坐标(米) */
    float target_y;                  /**< I区入口Y坐标(米) */
    float position_threshold;        /**< 位置到达阈值(米) */
    
    /* ========== 速度参数 ========== */
    float forward_speed1;            /**< 第一阶段前进速度(m/s) */
    float forward_speed2;            /**< 第二阶段前进速度(m/s) */
    float right_speed;               /**< 向右平移速度(m/s) */
    
    /* ========== 时间参数 ========== */
    uint32_t step1_delay_ms;         /**< 步骤1延时时间(ms) */
    uint32_t step2_timeout_ms;      /**< 步骤2超时时间(ms) */
    uint32_t step3_timeout_ms;      /**< 步骤3超时时间(ms) */
    uint32_t step4_timeout_ms;      /**< 步骤4超时时间(ms) */
    uint32_t step5_timeout_ms;      /**< 步骤5超时时间(ms) */
    
    /* ========== 限位检测参数 ========== */
    float limit_speed_start;         /**< 限位检测：开始移动速度阈值(rpm) */
    float limit_speed_stop;          /**< 限位检测：停止速度阈值(rpm) */
    
    /* ========== 硬件资源配置 ========== */
    GPIO_TypeDef* photo_switch_port; /**< 光电开关GPIO端口 */
    uint16_t photo_switch_pin;      /**< 光电开关GPIO引脚 */
} Zone1_Config_t;

/* ==================== 状态信息结构体（只读） ==================== */
/**
 * @brief Zone1 状态信息结构体
 * @note  用于获取当前状态信息，所有字段为只读
 */
typedef struct {
    Zone1_State_t current_state;     /**< 当前状态 */
    Zone1_State_t last_state;        /**< 上一个状态 */
    uint32_t elapsed_ms;             /**< 当前状态已运行时间(ms) */
    
    /* 传感器数据 */
    uint32_t laser_distance_mm;      /**< 激光雷达距离(mm) */
    uint8_t photo_switch_state;      /**< 光电开关状态(0=未检测到, 1=检测到) */
    float chassis_speed_sum;         /**< 底盘4个电机速度之和(rpm) */
    
    /* 里程计数据 */
    float odom_x;                    /**< 当前X坐标(米) */
    float odom_y;                    /**< 当前Y坐标(米) */
    float odom_yaw;                  /**< 当前航向角(度) */
    
    /* 旋转90度信息 */
    float rot_start_yaw;             /**< 旋转起始角度(度) */
    float rot_target_yaw;            /**< 旋转目标角度(度) */
    float rot_error;                 /**< 旋转误差(度) */
    
    /* 步骤5向右移动信息 */
    float right_move_distance;       /**< 步骤5已移动距离(米) */
} Zone1_Status_t;

/* ==================== 不透明句柄声明 ==================== */
/**
 * @brief Zone1 模块句柄类型（不透明指针）
 */
typedef struct Zone1_Handle_Struct* Zone1_Handle_t;

/* ==================== 接口函数声明 ==================== */

/**
 * @brief 初始化Zone1模块（新接口，带句柄）
 * @param[in]  config  配置结构体指针
 * @param[out] handle  输出句柄指针
 * @return Zone1_Error_t 错误码
 * @note   必须在使用其他接口前调用此函数
 */
Zone1_Error_t Zone1_Init_Handle(const Zone1_Config_t* config, Zone1_Handle_t* handle);

/**
 * @brief 反初始化Zone1模块（新接口，带句柄）
 * @param[in] handle  模块句柄
 * @return Zone1_Error_t 错误码
 */
Zone1_Error_t Zone1_DeInit_Handle(Zone1_Handle_t handle);

/**
 * @brief 启动Zone1流程（新接口，带句柄）
 * @param[in] handle  模块句柄
 * @return Zone1_Error_t 错误码
 */
Zone1_Error_t Zone1_Start_Handle(Zone1_Handle_t handle);

/**
 * @brief 停止Zone1流程并重置（新接口，带句柄）
 * @param[in] handle  模块句柄
 * @return Zone1_Error_t 错误码
 */
Zone1_Error_t Zone1_Stop_Handle(Zone1_Handle_t handle);

/**
 * @brief Zone1主循环处理函数（新接口，带句柄）
 * @param[in] handle  模块句柄
 * @return Zone1_Error_t 错误码
 * @note   需在任务循环中调用，建议调用周期10ms
 */
Zone1_Error_t Zone1_Process_Handle(Zone1_Handle_t handle);

/**
 * @brief 获取Zone1当前状态（新接口，带句柄）
 * @param[in]  handle  模块句柄
 * @param[out] status  状态信息结构体指针
 * @return Zone1_Error_t 错误码
 */
Zone1_Error_t Zone1_GetStatus_Handle(Zone1_Handle_t handle, Zone1_Status_t* status);

/**
 * @brief 获取Zone1当前状态枚举值（新接口，带句柄）
 * @param[in] handle  模块句柄
 * @return Zone1_State_t 当前状态
 */
Zone1_State_t Zone1_GetState_Handle(Zone1_Handle_t handle);

/**
 * @brief 获取Zone1配置参数（新接口，带句柄）
 * @param[in] handle  模块句柄
 * @param[out] config  配置结构体指针
 * @return Zone1_Error_t 错误码
 */
Zone1_Error_t Zone1_GetConfig_Handle(Zone1_Handle_t handle, Zone1_Config_t* config);

/**
 * @brief 获取Zone1全局句柄
 * @return Zone1_Handle_t 全局句柄
 * @note   用于在无法传递句柄的场合获取全局实例
 */
Zone1_Handle_t Zone1_GetGlobalHandle(void);

/**
 * @brief 使用默认配置初始化Zone1模块
 * @return Zone1_Error_t 错误码
 * @note   使用默认参数初始化，简化调用
 */
Zone1_Error_t Zone1_Init_Default(void);

#endif /* ZONE1_PROCESS_H */
