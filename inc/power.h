/**
 * @file power.h
 * @brief 低功耗管理模块
 * 
 * 实现功能:
 * 1. 系统休眠 (suspend-to-RAM)
 * 2. RTC 定时唤醒
 * 3. 外设电源控制
 * 4. 低功耗工作流程管理
 */

#ifndef __POWER_H__
#define __POWER_H__

#include <stdint.h>

// ============== 低功耗模式定义 ==============
typedef enum {
    POWER_MODE_NORMAL = 0,      // 正常运行模式
    POWER_MODE_IDLE,            // 空闲模式 (CPU 降频)
    POWER_MODE_SUSPEND,         // 休眠模式 (suspend-to-RAM)
    POWER_MODE_DEEP_SLEEP       // 深度睡眠 (需要硬件支持)
} power_mode_t;

// ============== 唤醒源定义 ==============
typedef enum {
    WAKEUP_SOURCE_NONE = 0,
    WAKEUP_SOURCE_RTC,          // RTC 定时器唤醒
    WAKEUP_SOURCE_GPIO,         // GPIO 中断唤醒
    WAKEUP_SOURCE_UART,         // 串口数据唤醒
    WAKEUP_SOURCE_USER          // 用户操作唤醒
} wakeup_source_t;

// ============== 低功耗配置 ==============
typedef struct {
    int wakeup_interval;        // 唤醒间隔 (秒)
    int enable_4g_sleep;        // 是否让 4G 模块休眠
    int enable_bt_sleep;        // 是否让蓝牙模块休眠
    int enable_lora_sleep;      // 是否让 LoRa 模块休眠
    int check_data_before_sleep;// 休眠前检查是否有待发送数据
} power_config_t;

// ============== 低功耗状态 ==============
typedef struct {
    power_mode_t current_mode;  // 当前电源模式
    wakeup_source_t last_wakeup;// 上次唤醒源
    uint32_t suspend_count;     // 休眠次数
    uint32_t wakeup_count;      // 唤醒次数
    uint32_t total_sleep_time;  // 总休眠时间 (秒)
} power_status_t;

// ============== 函数声明 ==============

/**
 * @brief 初始化低功耗管理模块
 * @param config 配置参数
 * @return 0=成功, -1=失败
 */
int power_init(const power_config_t *config);

/**
 * @brief 进入休眠模式
 * @param wakeup_seconds RTC 唤醒时间 (秒)
 * @return 唤醒源
 */
wakeup_source_t power_suspend(int wakeup_seconds);

/**
 * @brief 准备进入休眠 (关闭外设)
 * @return 0=成功, -1=失败
 */
int power_prepare_suspend(void);

/**
 * @brief 从休眠恢复 (重新初始化外设)
 * @return 0=成功, -1=失败
 */
int power_resume(void);

/**
 * @brief 设置 RTC 唤醒定时器
 * @param seconds 唤醒时间 (秒)
 * @return 0=成功, -1=失败
 */
// int power_set_rtc_wakeup(int seconds);

/**
 * @brief 清除 RTC 唤醒定时器
 */
// void power_clear_rtc_wakeup(void);

/**
 * @brief 控制 4G 模块电源/休眠
 * @param enable 1=开启, 0=休眠
 */
void power_4g_control(int enable);

/**
 * @brief 控制蓝牙模块电源
 * @param enable 1=开启, 0=关闭
 */
void power_bt_control(int enable);

/**
 * @brief 控制北斗模块电源
 * @param enable 1=开启, 0=关闭
 */
void power_bd_control(int enable);

/**
 * @brief 打开所有通信模块电源（4G和北斗）
 * @return 0=成功, -1=失败
 */
int power_comm_modules_on(void);

/**
 * @brief 关闭所有通信模块电源（4G和北斗）
 */
void power_comm_modules_off(void);

/**
 * @brief 获取低功耗状态
 * @param status 输出状态
 */
void power_get_status(power_status_t *status);

/**
 * @brief 低功耗工作线程
 * @param arg 配置参数 (power_config_t*)
 * @return NULL
 */
void *power_manager_thread(void *arg);

/**
 * @brief 检查是否有待发送数据
 * @return 1=有数据, 0=无数据
 */
int power_check_pending_data(void);

/**
 * @brief 执行一次数据发送周期
 * @return 0=成功, -1=失败
 */
int power_do_send_cycle(void);

#endif /* __POWER_H__ */
