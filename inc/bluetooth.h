/**
 * @file bluetooth.h
 * @brief 蓝牙模块接口定义
 *
 * 功能：
 * 1. 作为从设备与手机APP通信
 * 2. 发送DTU心跳、4G网络状态、蓝牙连接状态
 * 3. 接收手机端指令，选择数据发送路径（北斗/4G）
 */

#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include <stdint.h>

// ============== 发送路径定义 ==============
typedef enum
{
    SEND_PATH_AUTO = 0, // 自动选择（优先4G，失败切北斗）
    SEND_PATH_4G_ONLY,  // 仅4G
    SEND_PATH_BD_ONLY,  // 仅北斗
    SEND_PATH_4G_FIRST, // 4G优先
    SEND_PATH_BD_FIRST  // 北斗优先
} send_path_t;

// ============== 蓝牙命令定义 ==============
// 手机APP发送的命令格式: $CMD,<cmd_id>,<param>*<checksum>\r\n
#define BT_CMD_SET_PATH 0x01   // 设置发送路径
#define BT_CMD_GET_STATUS 0x02 // 获取设备状态
#define BT_CMD_REBOOT 0x03     // 重启设备
#define BT_CMD_GET_CONFIG 0x04 // 获取配置

// ============== 蓝牙响应定义 ==============
// DTU发送的响应格式: $RSP,<cmd_id>,<result>,<data>*<checksum>\r\n
#define BT_RSP_OK 0x00      // 成功
#define BT_RSP_ERR 0x01     // 失败
#define BT_RSP_INVALID 0x02 // 无效命令

// ============== 心跳包定义 ==============
// 心跳格式: $HB,<4g_status>,<bt_status>,<send_path>,<uptime>*<checksum>\r\n
#define BT_HEARTBEAT_INTERVAL 5 // 心跳间隔（秒）

// ============== 状态结构体 ==============
typedef struct
{
    uint8_t eg_connected;     // 4G连接状态: 0=断开, 1=已连接
    uint8_t eg_signal;        // 4G信号强度 (0-31, 99=未知)
    uint8_t bt_connected;     // 蓝牙连接状态: 0=断开, 1=已连接
    uint8_t send_path;        // 当前发送路径
    uint32_t uptime;          // 运行时间（秒）
    uint32_t send_count_4g;   // 4G发送计数
    uint32_t send_count_bd;   // 北斗发送计数
    uint32_t send_fail_count; // 发送失败计数
} dtu_status_t;

// ============== 函数声明 ==============

/**
 * @brief 初始化蓝牙模块
 * @return 0=成功, -1=失败
 */
int bt_init(void);

/**
 * @brief 发送心跳包
 * @param status DTU状态
 * @return 0=成功, -1=失败
 */
int bt_send_heartbeat(const dtu_status_t *status);

/**
 * @brief 发送状态响应
 * @param status DTU状态
 * @return 0=成功, -1=失败
 */
int bt_send_status(const dtu_status_t *status);

/**
 * @brief 发送命令响应
 * @param cmd_id 命令ID
 * @param result 结果码
 * @param data 附加数据（可为NULL）
 * @return 0=成功, -1=失败
 */
int bt_send_response(uint8_t cmd_id, uint8_t result, const char *data);

/**
 * @brief 解析蓝牙命令
 * @param buf 接收缓冲区
 * @param len 数据长度
 * @param cmd_id 输出命令ID
 * @param param 输出参数
 * @return 0=解析成功, -1=解析失败
 */
int bt_parse_command(const char *buf, int len, uint8_t *cmd_id, uint8_t *param);

/**
 * @brief 检查蓝牙连接状态（通过GPIO）
 * @return 1=已连接, 0=未连接
 */
int bt_is_connected(void);

/**
 * @brief 获取当前发送路径
 * @return 发送路径枚举值
 */
send_path_t bt_get_send_path(void);

/**
 * @brief 设置发送路径
 * @param path 发送路径
 */
void bt_set_send_path(send_path_t path);

/**
 * @brief 蓝牙通信线程函数
 * @param arg 线程参数（未使用）
 * @return NULL
 */
void *bt_comm_thread(void *arg);

/**
 * @brief 更新DTU状态（供其他模块调用）
 * @param eg_connected 4G连接状态
 * @param eg_signal 4G信号强度
 * @param send_count_4g 4G发送计数
 * @param send_count_bd 北斗发送计数
 * @param send_fail_count 发送失败计数
 */
void bt_update_status(uint8_t eg_connected, uint8_t eg_signal,
                      uint32_t send_count_4g, uint32_t send_count_bd,
                      uint32_t send_fail_count);
send_path_t bt_get_send_path(void);
void bt_set_send_path(send_path_t path);
void bt_update_status(uint8_t eg_connected, uint8_t eg_signal,
                      uint32_t send_count_4g, uint32_t send_count_bd,
                      uint32_t send_fail_count);
int bt_send_heartbeat_simple(uint8_t hb);
void bt_handle_simple_command(const char *cmd);
#endif /* __BLUETOOTH_H__ */
