/**
 * @file    new_remote_control.h
 * @brief   R2 机器人与上位机通信协议 - STM32 端接口定义
 *
 * ===================== 协议帧格式 =====================
 * 帧格式: [0xA5 0x5A CMD LEN(2B LE) PAYLOAD CHKSUM]
 *
 * ===================== 上行链路（发到上位机）=====================
 *   CMD 0x01  ODOM         里程计 (x,y,z,roll,pitch,yaw)  float32[6] 单位：米/度
 *   CMD 0x02  PATH         路径点   uint8=n, float32[n*2]
 *   CMD 0x03  KFS          KFS检测  uint8=n, [uint8 id, float32 xyz]*n
 *   CMD 0x05  ZONE_I_PATH  I区路径  uint8 start,end,n, [uint8 block_id]*n
 *
 * ===================== 下行链路（接收上位机）=====================
 *   CMD 0x10  ACK          确认      uint8 cmd, uint8 code(0=OK 1=ERR)
 *   CMD 0x12  STATUS       状态      uint8 state
 *   CMD 0x13  ZONE_I_INFO  I区信息   uint8 n, [uint8 block_id, kfs_type]*n
 *   CMD 0x14  DOCK_OK      R1对接成功  无
 *   CMD 0x15  GO_ZONE_I    前往I区    无
 */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>
#include "main.h"

/* ==================== 帧相关常量 ==================== */
#define RC_SYNC1               0xA5            /* 帧同步字节1 */
#define RC_SYNC2               0x5A            /* 帧同步字节2 */
#define RC_FRAME_HEADER_SIZE   5               /* 帧头大小: SYNC1 SYNC2 CMD LEN_LO LEN_HI */
#define RC_FRAME_MAX_PAYLOAD   64              /* 最大Payload长度 */
#define RC_FRAME_MAX_SIZE      (RC_FRAME_HEADER_SIZE + RC_FRAME_MAX_PAYLOAD + 1)  /* 最大帧大小 +1校验 */
#define RC_ODOM_PAYLOAD_SIZE   24              /* 里程计数据大小: 6*float */

/* ==================== 命令码枚举 ==================== */
/**
 * @brief 上行命令（STM32 -> 上位机）
 */
typedef enum {
    RC_CMD_ODOM        = 0x01,  /**< 里程计数据 */
    RC_CMD_PATH        = 0x02,  /**< 路径点数据 */
    RC_CMD_KFS         = 0x03,  /**< KFS检测数据 */
    RC_CMD_CMD_RSP     = 0x04,  /**< 命令响应 */
    RC_CMD_ZONE_I_PATH = 0x05,  /**< I区路径数据 */
    RC_CMD_ACK         = 0x10,  /**< 确认帧（下行） */
    RC_CMD_STATUS      = 0x12,  /**< 状态帧（下行） */
    RC_CMD_ZONE_I_INFO = 0x13,  /**< I区KFS信息（下行） */
    RC_CMD_DOCK_OK     = 0x14,  /**< 对接成功（下行） */
    RC_CMD_GO_ZONE_I   = 0x15,  /**< 前往I区（下行） */
} rc_cmd_t;

/* ==================== 数据结构定义 ==================== */

/**
 * @brief 里程计数据结构（收到 CMD_ODOM 时使用）
 * @note  包含机器人在全局坐标系中的位置和姿态
 */
typedef struct {
    float x;       /**< X坐标（米），场地坐标系的X方向 */
    float y;       /**< Y坐标（米），场地坐标系的Y方向 */
    float z;       /**< Z坐标（米），通常为0（地面机器人） */
    float roll;    /**< 横滚角（度），绕X轴旋转 */
    float pitch;    /**< 俯仰角（度），绕Y轴旋转 */
    float yaw;      /**< 航向角（度），绕Z轴旋转 ← 最重要！用于判断朝向 */
} rc_odom_t;

/**
 * @brief 单个路径点结构
 */
typedef struct {
    float x;       /**< 路径点X坐标（米） */
    float y;       /**< 路径点Y坐标（米） */
} rc_waypoint_t;

/**
 * @brief 路径点数据（收到 CMD_PATH 时使用）
 */
typedef struct {
    uint8_t       num;                     /**< 路径点数量 */
    rc_waypoint_t points[16];             /**< 路径点数组，最大16个 */
} rc_path_t;

/**
 * @brief 单个KFS检测目标结构
 */
typedef struct {
    uint8_t id;     /**< KFS目标编号 */
    float   x;      /**< KFS在场地坐标系中的X坐标（米） */
    float   y;      /**< KFS在场地坐标系中的Y坐标（米） */
    float   z;      /**< KFS在场地坐标系中的Z坐标（米） */
} rc_kfs_detect_t;

/**
 * @brief KFS检测数据结构（收到 CMD_KFS 时使用）
 */
typedef struct {
    uint8_t          num;                 /**< 检测到的KFS数量 */
    rc_kfs_detect_t  detections[8];       /**< KFS检测数组，最大8个 */
} rc_kfs_t;

/**
 * @brief I区路径数据结构（收到 CMD_ZONE_I_PATH 时使用）
 */
typedef struct {
    uint8_t start_block;                 /**< 起始区块ID */
    uint8_t end_block;                   /**< 目标区块ID */
    uint8_t num_blocks;                  /**< 经过的区块数量 */
    uint8_t block_ids[32];               /**< 途径区块ID列表 */
} rc_zone_i_path_t;

/**
 * @brief I区KFS检测信息结构（发给上位机）
 */
typedef struct {
    uint8_t block_id;    /**< 区块ID */
    uint8_t kfs_type;    /**< KFS类型: 1=R1_KFS, 2=R2_KFS, 3=FAKE(假目标) */
} rc_zone_i_kfs_t;

/**
 * @brief R2 机器人在上位机的状态枚举
 */
typedef enum {
    RC_STATE_IDLE       = 0,  /**< 空闲状态 */
    RC_STATE_MOVING     = 1,  /**< 运动中 */
    RC_STATE_AT_TARGET  = 2,  /**< 到达目标位置 */
    RC_STATE_GRABBING   = 3,  /**< 抓取中 */
    RC_STATE_DONE       = 4,  /**< 任务完成 */
    RC_STATE_ERROR      = 5,  /**< 错误状态 */
} rc_state_t;

/* ==================== 回调函数类型定义 ==================== */
/**
 * @brief 里程计数据回调函数类型
 * @param odom 收到的里程计数据指针
 */
typedef void (*rc_odom_callback_t)(const rc_odom_t *odom);

/**
 * @brief 路径点数据回调函数类型
 * @param path 收到的路径点数据指针
 */
typedef void (*rc_path_callback_t)(const rc_path_t *path);

/**
 * @brief KFS检测数据回调函数类型
 * @param kfs 收到的KFS检测数据指针
 */
typedef void (*rc_kfs_callback_t)(const rc_kfs_t *kfs);

/**
 * @brief I区路径数据回调函数类型
 * @param path 收到的I区路径数据指针
 */
typedef void (*rc_zone_i_path_callback_t)(const rc_zone_i_path_t *path);

/* ==================== 接口变量声明 ==================== */
/**
 * @brief UART句柄指针（用于USART10）
 * @note  供外部模块使用
 */
extern UART_HandleTypeDef *huart_ptr;

/**
 * @brief UART接收缓存（用于USART10）
 * @note  供外部模块使用
 */
extern uint8_t uart_rx_byte;

/* ==================== 接口函数声明 ==================== */

/**
 * @brief 初始化遥控协议栈
 * @param uart_send  发送单个字节的函数指针（如 HAL_UART_Transmit 的包装）
 * @param get_ms     获取当前系统毫秒数的函数指针（如 HAL_GetTick）
 * @note  在使用其他函数之前必须先调用此函数进行初始化
 */
void rc_init(void (*uart_send)(uint8_t byte), uint32_t (*get_ms)(void));

/**
 * @brief 使用指定UART初始化遥控协议栈（推荐使用）
 * @param huart  指向UART句柄的指针（如 &huart10）
 * @note  此函数会初始化协议栈、开启USART10中断接收
 *        只需在初始化时调用一次即可
 */
void rc_init_with_uart(UART_HandleTypeDef *huart);

/**
 * @brief 注册里程计数据回调
 * @param cb 回调函数，当收到有效的里程计数据时会被调用
 */
void rc_set_odom_callback(rc_odom_callback_t cb);

/**
 * @brief 注册路径点数据回调
 * @param cb 回调函数，当收到有效的路径点数据时会被调用
 */
void rc_set_path_callback(rc_path_callback_t cb);

/**
 * @brief 注册KFS检测数据回调
 * @param cb 回调函数，当收到有效的KFS数据时会被调用
 */
void rc_set_kfs_callback(rc_kfs_callback_t cb);

/**
 * @brief 注册I区路径数据回调
 * @param cb 回调函数，当收到有效的I区路径数据时会被调用
 */
void rc_set_zone_i_path_callback(rc_zone_i_path_callback_t cb);

/**
 * @brief 喂入接收到的字节（在UART中断或轮询中调用）
 * @param byte 接收到的单个字节
 * @note  通常在UART_RX_IRQHandler或DMA完成回调中调用此函数
 */
void rc_feed_byte(uint8_t byte);

/**
 * @brief 轮询处理函数（在主循环中调用）
 * @note  用于处理超时等时间相关任务，当前为空实现
 */
void rc_poll(void);

/* ==================== 发送函数（STM32 -> 上位机）==================== */

/**
 * @brief 发送确认帧
 * @param cmd  被确认的命令码
 * @param code 确认码: 0=成功, 1=失败
 */
void rc_send_ack(uint8_t cmd, uint8_t code);

/**
 * @brief 发送机器人状态到上位机
 * @param state 当前状态（参见 rc_state_t）
 */
void rc_send_status(rc_state_t state);

/**
 * @brief 发送I区KFS检测信息到上位机（R1需要此信息做路径规划）
 * @param num      KFS数量
 * @param kfs_list KFS信息数组
 */
void rc_send_zone_i_info(uint8_t num, const rc_zone_i_kfs_t *kfs_list);

/**
 * @brief 发送对接成功消息到上位机
 * @note  当R1成功对接后调用此函数通知上位机
 */
void rc_send_dock_ok(void);

/**
 * @brief 发送前往I区请求到上位机
 * @note  当需要上位机规划到I区的路径时调用
 */
void rc_send_go_zone_i(void);

/* ==================== 查询函数 ==================== */

/**
 * @brief 获取最近一次收到的里程计数据
 * @return 指向最近里程计数据的常量指针
 * @note  返回的是内部缓存的指针，不要修改其内容
 */
const rc_odom_t *rc_get_latest_odom(void);

/**
 * @brief 检查里程计数据是否有效（未超时）
 * @return 1=有效, 0=无效（数据超时）
 * @note  默认超时时间为 2000ms，超过此时间未收到新数据则认为无效
 */
uint8_t rc_odom_is_valid(void);

#endif /* REMOTE_CONTROL_H */
