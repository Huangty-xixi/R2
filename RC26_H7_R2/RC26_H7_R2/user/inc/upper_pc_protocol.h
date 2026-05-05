/**
 * @file    upper_pc_protocol.h
 * @brief   R2 上下位机串口通信协议 — STM32 端解包（工程内文件名：上位机协议）
 *
 * 帧格式: [0xA5 0x5A CMD LEN(2B LE) PAYLOAD CHKSUM]
 *
 * 上位机 → 下位机 (Uplink):
 *   CMD 0x01  ODOM         里程计 (x,y,z,roll,pitch,yaw)  float32[6] 米/度
 *   CMD 0x02  PATH         路径点   uint8=n, float32[n*2]
 *   CMD 0x03  KFS          KFS检测  uint8=n, [uint8 id, float32 xyz]*n
 *   CMD 0x05  ZONE_I_PATH  I区路径  uint8 start,end,n, [uint8 block_id]*n
 *
 * 下位机 → 上位机 (Downlink):
 *   CMD 0x10  ACK          确认      uint8 cmd, uint8 code(0=OK 1=ERR)
 *   CMD 0x12  STATUS       状态      uint8 state
 *   CMD 0x13  ZONE_I_INFO  I区信息   uint8 n, [uint8 block_id, kfs_type]*n
 *   CMD 0x14  DOCK_OK      R1对接成功  空
 *   CMD 0x15  GO_ZONE_I    请求入I区   空
 *
 * 说明：与 SBUS 遥控器用的 remote_control.h 重名冲突，故本文件用 upper_pc_protocol.h。
 */
#ifndef UPPER_PC_PROTOCOL_H
#define UPPER_PC_PROTOCOL_H

#include <stdint.h>

/* ---------- 帧常量 ---------- */
#define RC_SYNC1               0xA5
#define RC_SYNC2               0x5A
#define RC_FRAME_HEADER_SIZE   5   /* SYNC1 SYNC2 CMD LEN_LO LEN_HI */
#define RC_FRAME_MAX_PAYLOAD   64
#define RC_FRAME_MAX_SIZE      (RC_FRAME_HEADER_SIZE + RC_FRAME_MAX_PAYLOAD + 1)  /* +1 CHKSUM */
#define RC_ODOM_PAYLOAD_SIZE   24  /* 6*float */

/* ---------- 命令码 ---------- */
typedef enum {
    RC_CMD_ODOM        = 0x01,  /* 上→下: 里程计 */
    RC_CMD_PATH        = 0x02,  /* 上→下: 路径 */
    RC_CMD_KFS         = 0x03,  /* 上→下: KFS检测 */
    RC_CMD_CMD_RSP     = 0x04,  /* 上→下: 指令响应 */
    RC_CMD_ZONE_I_PATH = 0x05,  /* 上→下: I区路径 */
    RC_CMD_ACK         = 0x10,  /* 下→上: 确认 */
    RC_CMD_STATUS      = 0x12,  /* 下→上: 状态 */
    RC_CMD_ZONE_I_INFO = 0x13,  /* 下→上: I区KFS布局 */
    RC_CMD_DOCK_OK     = 0x14,  /* 下→上: R1对接成功 */
    RC_CMD_GO_ZONE_I   = 0x15,  /* 下→上: 请求入I区 */
} rc_cmd_t;

/* ---------- 数据结构 ---------- */

/** 里程计数据 (收到 CMD_ODOM 时填充) */
typedef struct {
    float x;      /* 米 */
    float y;      /* 米 */
    float z;      /* 米 */
    float roll;   /* 度 */
    float pitch;  /* 度 */
    float yaw;    /* 度 */
} rc_odom_t;

/** 单个路径点 */
typedef struct {
    float x;
    float y;
} rc_waypoint_t;

/** 路径数据 */
typedef struct {
    uint8_t       num;//路径点数量
    rc_waypoint_t points[16];//路径点数组
} rc_path_t;

/** 单个 KFS 检测结果 */
typedef struct {
    uint8_t id;//KFS编号
    float   x, y, z; /* 相机坐标系, 米 */
} rc_kfs_detect_t;

/** KFS 检测数据 */
typedef struct {
    uint8_t          num;//KFS检测数量
    rc_kfs_detect_t  detections[8];//KFS检测数组
} rc_kfs_t;

/** I区树林路径 */
typedef struct {
    uint8_t start_block;//起始块ID
    uint8_t end_block;//结束块ID
    uint8_t num_blocks;//块数量
    uint8_t block_ids[32];//块ID数组
} rc_zone_i_path_t;

/** I区 KFS 布局信息 (发给上位机) */
typedef struct {
    uint8_t block_id;//块ID
    uint8_t kfs_type;   /* 1=R1_KFS, 2=R2_KFS, 3=FAKE */
} rc_zone_i_kfs_t;

/** R2 下位机状态 */
typedef enum {
    RC_STATE_IDLE       = 0,
    RC_STATE_MOVING     = 1,
    RC_STATE_AT_TARGET  = 2,
    RC_STATE_GRABBING   = 3,
    RC_STATE_DONE       = 4,
    RC_STATE_ERROR      = 5,
} rc_state_t;

/* ---------- 回调函数类型 ---------- */
typedef void (*rc_odom_callback_t)(const rc_odom_t *odom);
typedef void (*rc_path_callback_t)(const rc_path_t *path);
typedef void (*rc_kfs_callback_t)(const rc_kfs_t *kfs);
typedef void (*rc_zone_i_path_callback_t)(const rc_zone_i_path_t *path);

/* ---------- 解析器 ---------- */

/**
 * @brief 初始化解析器（ODOM 帧写入内部 latest_odom；读数用 rc_get_latest_odom / rc_odom_is_valid）
 * @param uart_send  发送单字节函数 (如 HAL_UART_Transmit 的封装)
 * @param get_ms     获取毫秒时间戳函数 (用于看门狗)
 */
void rc_init(void (*uart_send)(uint8_t byte), uint32_t (*get_ms)(void));

/** 注册回调 */
void rc_set_odom_callback(rc_odom_callback_t cb);
void rc_set_path_callback(rc_path_callback_t cb);
void rc_set_kfs_callback(rc_kfs_callback_t cb);
void rc_set_zone_i_path_callback(rc_zone_i_path_callback_t cb);

/**
 * @brief 喂入接收到的字节 (在 UART RX 中断/回调中调用)
 * @param byte 接收到的单字节
 */
void rc_feed_byte(uint8_t byte);

/** 主循环中调用，检查数据超时 */
void rc_poll(void);

/* ---------- 发送 (下位机→上位机) ---------- */

/** 发送 ACK */
void rc_send_ack(uint8_t cmd, uint8_t code);

/** 发送下位机状态 */
void rc_send_status(rc_state_t state);

/** 发送 I区 KFS 布局信息 (R1 红外转发的数据) */
void rc_send_zone_i_info(uint8_t num, const rc_zone_i_kfs_t *kfs_list);

/** 发送 R1 对接成功 */
void rc_send_dock_ok(void);

/** 发送请求进入 I区 */
void rc_send_go_zone_i(void);

/* ---------- 工具 ---------- */

/** 获取最近一次收到的里程计数据 */
const rc_odom_t *rc_get_latest_odom(void);

/** ODOM 数据是否在有效期内 (默认 2 秒超时) */
uint8_t rc_odom_is_valid(void);

#endif /* UPPER_PC_PROTOCOL_H */
