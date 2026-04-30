/**
 * @file power.c
 * @brief 低功耗管理模块实现
 * 
 * 工作流程:
 * 1. 设置 RTC 唤醒定时器
 * 2. 关闭/休眠外设 (4G, BT, LoRa)
 * 3. 写入 "mem" 到 /sys/power/state 进入 suspend-to-RAM
 * 4. RTC 中断唤醒系统
 * 5. 恢复外设
 * 6. 检查并发送数据
 * 7. 重复步骤 1
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <sys/stat.h>
#include <stdatomic.h>
#include <gpiod.h>
#include "../inc/power.h"
#include "../inc/usart.h"
#include "../inc/universal.h"
#include "../inc/eg800k.h"
#include "../inc/bluetooth.h"
#include "../inc/gpio.h"
#include "../app/thread_summary.h"

// ============== 外部变量 ==============
extern volatile sig_atomic_t stop_flag;
extern struct gpiod_line *line_4g_pow;
extern struct gpiod_line *line_4g_sleep;
extern struct gpiod_line *line_4g_boot;
extern struct gpiod_line *line_bt_pow;
extern struct gpiod_line *line_bd_en;
extern struct gpiod_line *line_bd_pow;
extern int eg_fd;
extern int bt_fd;

// ============== 全局变量 ==============
static power_config_t g_power_config = {
    .wakeup_interval = 10,
    .enable_4g_sleep = 1,
    .enable_bt_sleep = 1,
    .enable_lora_sleep = 1,
    .check_data_before_sleep = 1
};

static power_status_t g_power_status = {
    .current_mode = POWER_MODE_NORMAL,
    .last_wakeup = WAKEUP_SOURCE_NONE,
    .suspend_count = 0,
    .wakeup_count = 0,
    .total_sleep_time = 0
};

// 数据发送相关
static const char *log_paths[] = {"/home/cat/rs485_data.log", "/home/cat/rs232_data.log"};
static off_t log_offsets[2] = {0, 0};
#define OFFSET_FILE_POWER "/home/cat/power_offset.dat"

// ============== 内部函数 ==============

/**
 * @brief 写入字符串到 sysfs 文件
 */
static int write_sysfs(const char *path, const char *value)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror(path);
        return -1;
    }
    
    int ret = write(fd, value, strlen(value));
    close(fd);
    
    return (ret > 0) ? 0 : -1;
}

/**
 * @brief 读取 sysfs 文件内容
 */
static int read_sysfs(const char *path, char *buf, int len)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        return -1;
    }
    
    int n = read(fd, buf, len - 1);
    close(fd);
    
    if (n > 0) {
        buf[n] = '\0';
        return n;
    }
    return -1;
}

// ============== 公共函数实现 ==============

int power_init(const power_config_t *config)
{
    if (config) {
        memcpy(&g_power_config, config, sizeof(power_config_t));
    }
    
    // 加载偏移量
    load_offsets(OFFSET_FILE_POWER, log_offsets, 2);
    
    // 检查系统是否支持 suspend
    char buf[64];
    if (read_sysfs("/sys/power/state", buf, sizeof(buf)) > 0) {
        if (strstr(buf, "mem")) {
            printf("[POWER] System supports suspend-to-RAM\n");
        } else {
            printf("[POWER] Warning: suspend-to-RAM not supported\n");
            printf("[POWER] Available states: %s", buf);
        }
    }
    
    // 配置 RTC 为唤醒源
    // 通常需要在设备树或 sysfs 中配置
    // echo enabled > /sys/class/rtc/rtc0/device/power/wakeup
    write_sysfs("/sys/class/rtc/rtc0/device/power/wakeup", "enabled");
    
    printf("[POWER] Power manager initialized, wakeup_interval=%d sec\n", 
           g_power_config.wakeup_interval);
    
    return 0;
}

// int power_set_rtc_wakeup(int seconds)
// {
//     // 方法1: 使用 RX8010SJ 定时器
//     if (rx8010_set_timer_wakeup(seconds) == 0) {
//         printf("[POWER] RTC wakeup set via RX8010SJ: %d seconds\n", seconds);
//         return 0;
//     }
    
//     // 方法2: 使用 /sys/class/rtc/rtc0/wakealarm
//     char cmd[64];
//     time_t now = time(NULL);
//     time_t wakeup_time = now + seconds;
    
//     // 先清除旧的 wakealarm
//     write_sysfs("/sys/class/rtc/rtc0/wakealarm", "0");
    
//     // 设置新的 wakealarm (Unix 时间戳)
//     snprintf(cmd, sizeof(cmd), "%ld", wakeup_time);
//     if (write_sysfs("/sys/class/rtc/rtc0/wakealarm", cmd) == 0) {
//         printf("[POWER] RTC wakeup set via sysfs: %d seconds\n", seconds);
//         return 0;
//     }
    
//     // 方法3: 使用 rtcwake 命令 (备用)
//     // snprintf(cmd, sizeof(cmd), "rtcwake -m no -s %d", seconds);
//     // system(cmd);
    
//     printf("[POWER] Failed to set RTC wakeup\n");
//     return -1;
// }

// void power_clear_rtc_wakeup(void)
// {
//     rx8010_disable_irq();
//     write_sysfs("/sys/class/rtc/rtc0/wakealarm", "0");
// }

void power_4g_control(int enable)
{
    if (enable) {
        // 唤醒 4G 模块
        if (line_4g_sleep) {
            gpio_set_value(0, line_4g_sleep);  // DTR 低电平唤醒
        }
        printf("[POWER] 4G module wakeup\n");
    } else {
        // 让 4G 模块进入休眠
        if (line_4g_sleep) {
            gpio_set_value(1, line_4g_sleep);  // DTR 高电平休眠
        }
        printf("[POWER] 4G module sleep\n");
    }
}

void power_bt_control(int enable)
{
    if (line_bt_pow) {
        gpio_set_value(enable ? 0 : 1, line_bt_pow);
        printf("[POWER] Bluetooth %s\n", enable ? "enabled" : "disabled");
    }
}

void power_bd_control(int enable)
{
    if (enable) {
        // 开启北斗模块电源
        if (line_bd_pow) {
            gpio_set_value(1, line_bd_pow);
        }
        if (line_bd_en) {
            gpio_set_value(1, line_bd_en);
        }
        printf("[POWER] BeiDou module enabled\n");
    } else {
        // 关闭北斗模块电源
        if (line_bd_en) {
            gpio_set_value(0, line_bd_en);
        }
        if (line_bd_pow) {
            gpio_set_value(0, line_bd_pow);
        }
        printf("[POWER] BeiDou module disabled\n");
    }
}

int power_comm_modules_on(void)
{
    printf("[POWER] Powering on communication modules...\n");
    
    // 1. 开启4G模块电源
    if (line_4g_pow) {
        gpio_set_value(1, line_4g_pow);
    }
    if (line_4g_boot) {
        gpio_set_value(0, line_4g_boot);
        usleep(100000);  // 100ms
        gpio_set_value(1, line_4g_boot);
    }
    power_4g_control(1);  // 唤醒4G模块
    
    // 2. 开启北斗模块电源
    power_bd_control(1);
    
    // 3. 等待模块启动稳定
    printf("[POWER] Waiting for modules to stabilize...\n");
    sleep(3);  // 等待3秒让模块稳定
    
    printf("[POWER] Communication modules powered on\n");
    return 0;
}

void power_comm_modules_off(void)
{
    printf("[POWER] Powering off communication modules...\n");
    
    // 1. 关闭4G模块
    power_4g_control(0);  // 先让4G进入休眠
    usleep(100000);
    if (line_4g_pow) {
        gpio_set_value(0, line_4g_pow);
    }
    
    // 2. 关闭北斗模块
    power_bd_control(0);
    
    printf("[POWER] Communication modules powered off\n");
}

int power_prepare_suspend(void)
{
    printf("[POWER] Preparing for suspend...\n");
    
    // 1. 保存偏移量
    save_offsets(OFFSET_FILE_POWER, log_offsets, 2);
    
    // 2. 确保通信模块已关闭（正常情况下应该已经关闭）
    // 这里只是做双重保险
    if (g_power_config.enable_4g_sleep) {
        power_4g_control(0);
    }
    
    // 3. 关闭蓝牙
    if (g_power_config.enable_bt_sleep) {
        power_bt_control(0);
    }
    
    // 4. 刷新所有文件缓冲
    sync();
    
    printf("[POWER] Suspend preparation complete\n");
    return 0;
}

int power_resume(void)
{
    printf("[POWER] Resuming from suspend...\n");
    
    // 1. 开启蓝牙（如果需要）
    if (g_power_config.enable_bt_sleep) {
        power_bt_control(1);
    }
    
    // 2. 清除 RTC 中断标志
    // rx8010_clear_irq();
    
    // 注意：通信模块（4G和北斗）不在这里开启
    // 只有在有数据需要发送时才会在 power_do_send_cycle 中开启
    
    g_power_status.wakeup_count++;
    printf("[POWER] Resume complete (wakeup #%u)\n", g_power_status.wakeup_count);
    
    return 0;
}

/**
 * @brief 使用 sysfs 直接设置 RTC wakealarm 并进入休眠
 * @return 0 成功, -1 失败
 */
static int suspend_via_sysfs(int wakeup_seconds)
{
    char buf[64];
    time_t wakeup_time = time(NULL) + wakeup_seconds;
    
    // 1. 清除旧的 wakealarm
    if (write_sysfs("/sys/class/rtc/rtc0/wakealarm", "0") < 0) {
        printf("[POWER] Failed to clear wakealarm\n");
        return -1;
    }
    
    // 2. 设置新的 wakealarm (Unix 时间戳)
    snprintf(buf, sizeof(buf), "%ld", wakeup_time);
    if (write_sysfs("/sys/class/rtc/rtc0/wakealarm", buf) < 0) {
        printf("[POWER] Failed to set wakealarm\n");
        return -1;
    }
    
    // 3. 验证 wakealarm 已设置
    if (read_sysfs("/sys/class/rtc/rtc0/wakealarm", buf, sizeof(buf)) > 0) {
        printf("[POWER] Wakealarm set to: %s", buf);
    }
    
    // 4. 同步文件系统
    sync();
    usleep(100000);  // 100ms
    
    // 5. 写入 mem 进入休眠
    printf("[POWER] Writing 'mem' to /sys/power/state...\n");
    fflush(stdout);
    
    if (write_sysfs("/sys/power/state", "mem") < 0) {
        printf("[POWER] Failed to enter suspend\n");
        return -1;
    }
    
    // === 系统唤醒后从这里继续 ===
    return 0;
}

wakeup_source_t power_suspend(int wakeup_seconds)
{
    time_t sleep_start = time(NULL);
    
    // 准备休眠
    power_prepare_suspend();
    
    printf("[POWER] Entering suspend for %d seconds...\n", wakeup_seconds);
    g_power_status.current_mode = POWER_MODE_SUSPEND;
    g_power_status.suspend_count++;
    
    int ret;
    
    // 方法1: 直接使用 sysfs (推荐)
    printf("[POWER] Trying sysfs wakealarm method...\n");
    ret = suspend_via_sysfs(wakeup_seconds);
    
    if (ret != 0) {
        // 方法2: 使用 rtcwake 命令作为备用
        char cmd[256];
        printf("[POWER] Sysfs failed, trying rtcwake...\n");
        
        snprintf(cmd, sizeof(cmd), "rtcwake -d /dev/rtc0 -m mem -s %d 2>&1", wakeup_seconds);
        printf("[POWER] Executing: %s\n", cmd);
        fflush(stdout);
        sync();
        
        ret = system(cmd);
        
        if (ret != 0) {
            // 方法3: 尝试 freeze 模式 (s2idle)
            printf("[POWER] rtcwake mem failed (ret=%d), trying freeze...\n", ret);
            
            snprintf(cmd, sizeof(cmd), "rtcwake -d /dev/rtc0 -m freeze -s %d 2>&1", wakeup_seconds);
            printf("[POWER] Executing: %s\n", cmd);
            fflush(stdout);
            sync();
            
            ret = system(cmd);
            
            if (ret != 0) {
                printf("[POWER] All suspend modes failed, using sleep fallback\n");
                // 使用可中断的 sleep
                interruptible_sleep(wakeup_seconds);
            }
        }
    }
    
    // === 系统被唤醒后从这里继续执行 ===
    
    time_t sleep_end = time(NULL);
    int actual_sleep = (int)(sleep_end - sleep_start);
    g_power_status.total_sleep_time += actual_sleep;
    g_power_status.current_mode = POWER_MODE_NORMAL;
    
    printf("[POWER] Woke up! ret=%d, actual_sleep=%d sec\n", ret, actual_sleep);
    
    // 恢复
    power_resume();
    
    g_power_status.last_wakeup = WAKEUP_SOURCE_RTC;
    return WAKEUP_SOURCE_RTC;
}

int power_check_pending_data(void)
{
    // 检查日志文件是否有新数据
    for (int i = 0; i < 2; i++) {
        struct stat st;
        if (stat(log_paths[i], &st) == 0) {
            if (st.st_size > log_offsets[i]) {
                printf("[POWER] Pending data found in %s (%ld bytes)\n", 
                       log_paths[i], st.st_size - log_offsets[i]);
                return 1;
            }
        }
    }
    return 0;
}

int power_do_send_cycle(void)
{
    char hex_buf[256];
    int hex_len, entry_count;
    static int eg_connected = 0;  // 记录连接状态
    static int eg_initialized = 0;  // 记录4G模块是否已初始化
    
    // 打包数据
    if (pack_data_from_files(log_paths, log_offsets, 2, 200,
                             hex_buf, &hex_len, &entry_count) != 0) {
        return -1;
    }
    
    if (entry_count == 0) {
        printf("[POWER] No data to send\n");
        return 0;
    }
    
    // 转换为二进制
    unsigned char raw_data[256];
    int raw_len = hex_to_bytes(hex_buf, raw_data, sizeof(raw_data));
    if (raw_len <= 0) {
        return -1;
    }
    
    printf("[POWER] Sending %d bytes...\n", raw_len);
    
    // ========== 有数据要发送，先打开通信模块电源 ==========
    printf("[POWER] Data ready, powering on communication modules...\n");
    power_comm_modules_on();
    
    // 如果4G模块未初始化，先初始化
    if (!eg_initialized) {
        printf("[POWER] Initializing 4G module...\n");
        if (eg_init() == 0) {
            eg_initialized = 1;
            printf("[POWER] 4G module initialized\n");
        } else {
            printf("[POWER] 4G module init failed\n");
        }
    }
    
    // 获取发送路径
    send_path_t path = bt_get_send_path();
    int send_ok = 0;
    
    // 根据路径发送
    if (path == SEND_PATH_4G_ONLY || path == SEND_PATH_4G_FIRST || path == SEND_PATH_AUTO) {
        // 如果未连接，先尝试连接
        if (!eg_connected && eg_initialized) {
            printf("[POWER] TCP not connected, trying to connect...\n");
            if (eg_connect() == 0) {
                eg_connected = 1;
                printf("[POWER] TCP connected\n");
            }
        }
        
        // 尝试 4G 发送
        if (eg_connected) {
            if (eg_send_data(raw_data, raw_len) == 0) {
                send_ok = 1;
                printf("[POWER] Sent via 4G\n");
            } else {
                // 发送失败，可能连接断开，尝试重连
                printf("[POWER] 4G send failed, reconnecting...\n");
                eg_send_cmd("AT+QICLOSE=0\r\n", "OK", 5);
                eg_connected = 0;
                
                if (eg_connect() == 0) {
                    eg_connected = 1;
                    if (eg_send_data(raw_data, raw_len) == 0) {
                        send_ok = 1;
                        printf("[POWER] Sent via 4G after reconnect\n");
                    }
                }
            }
        }
    }
    
    if (!send_ok && (path == SEND_PATH_BD_ONLY || path == SEND_PATH_BD_FIRST || path == SEND_PATH_AUTO)) {
        // 尝试北斗发送
        if (bd_send_packet(raw_data, raw_len) == 0) {
            send_ok = 1;
            printf("[POWER] Sent via BD\n");
        }
    }
    
    // ========== 发送完成后关闭通信模块电源 ==========
    printf("[POWER] Send cycle complete, powering off communication modules...\n");
    
    // 关闭TCP连接
    if (eg_connected) {
        eg_send_cmd("AT+QICLOSE=0\r\n", "OK", 5);
        eg_connected = 0;
    }
    eg_initialized = 0;  // 下次需要重新初始化
    
    power_comm_modules_off();
    
    if (send_ok) {
        save_offsets(OFFSET_FILE_POWER, log_offsets, 2);
        return 0;
    }
    
    printf("[POWER] Send failed\n");
    return -1;
}

void power_get_status(power_status_t *status)
{
    if (status) {
        memcpy(status, &g_power_status, sizeof(power_status_t));
    }
}

// ============== 低功耗工作线程 ==============

void *power_manager_thread(void *arg)
{
    power_config_t *config = (power_config_t *)arg;
    
    if (config) {
        memcpy(&g_power_config, config, sizeof(power_config_t));
    }
    
    printf("[POWER] Power manager thread started\n");
    printf("[POWER] Wakeup interval: %d seconds\n", g_power_config.wakeup_interval);
    
    // 初始化
    power_init(NULL);
    
    // 初始状态：关闭通信模块电源（省电）
    printf("[POWER] Initial state: powering off communication modules...\n");
    power_comm_modules_off();
    
    // 等待其他模块初始化完成
    interruptible_sleep(2);
    
    while (!stop_flag) {
        // 1. 检查是否有待发送数据
        if (power_check_pending_data()) {
            printf("[POWER] Data pending, starting send cycle...\n");
            
            // power_do_send_cycle 内部会自动开关电源
            power_do_send_cycle();
        } else {
            printf("[POWER] No pending data, modules remain off\n");
        }
        
        // 2. 进入休眠（通信模块已关闭）
        if (!stop_flag) {
            printf("[POWER] Entering low power mode for %d seconds...\n", 
                   g_power_config.wakeup_interval);
            
            wakeup_source_t source = power_suspend(g_power_config.wakeup_interval);
            
            printf("[POWER] Woke up from %s\n", 
                   source == WAKEUP_SOURCE_RTC ? "RTC" : "other source");
        }
    }
    
    // 清理
    // power_clear_rtc_wakeup();
    save_offsets(OFFSET_FILE_POWER, log_offsets, 2);
    
    printf("[POWER] Power manager thread stopped\n");
    printf("[POWER] Stats: suspend=%u, wakeup=%u, total_sleep=%u sec\n",
           g_power_status.suspend_count, g_power_status.wakeup_count,
           g_power_status.total_sleep_time);
    
    return NULL;
}
