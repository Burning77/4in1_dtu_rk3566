/**
 * @file bluetooth.c
 * @brief 蓝牙模块实现
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <gpiod.h>
#include <stdatomic.h>
#include "../inc/bluetooth.h"
#include "../inc/usart.h"
#include <sys/select.h>
#include <signal.h>
#include "../inc/llcc68.h"
#include "../inc/universal.h"

// ============== 外部变量 ==============
extern int bt_fd;
extern volatile sig_atomic_t stop_flag;
extern struct gpiod_line *line_bt_status;

// ============== 全局变量 ==============
static send_path_t g_send_path = SEND_PATH_AUTO; // 默认自动选择
static pthread_mutex_t g_path_mutex = PTHREAD_MUTEX_INITIALIZER;

// DTU状态（由其他模块更新）
static dtu_status_t g_dtu_status = {0};
static pthread_mutex_t g_status_mutex = PTHREAD_MUTEX_INITIALIZER;

// 启动时间
static time_t g_start_time = 0;
static int bt_send_lora_cfg(void)
{
    loRa_Para_t cfg;
    char msg[128];

    lora_cfg_get(&cfg);

    snprintf(msg, sizeof(msg),
             "LL_LORA_ROOT_0x%02X,MESH_0x%02X,NET_0x%02X,DEV_0x%02X\r\n",
             cfg.is_root, cfg.mesh_type, cfg.net_id, cfg.dev_id);

    return bt_send_text(msg);
}
static int bt_set_name(void)
{
    loRa_Para_t cfg;
    char cmd[64];
    char resp[128] = {0};
    char name[32];
    int n;

    lora_cfg_get(&cfg);

    snprintf(name, sizeof(name), "LLXT_%02X", cfg.dev_id);
    snprintf(cmd, sizeof(cmd), "AT+NAME=%s\r\n", name);

    tcflush(bt_fd, TCIFLUSH);

    if (data_send((unsigned char *)cmd, strlen(cmd), BT_DEV) <= 0)
    {
        printf("[BT] set name send failed: %s\n", name);
        return -1;
    }

    usleep(200 * 1000);

    n = data_recv((unsigned char *)resp, sizeof(resp) - 1, BT_DEV);
    if (n > 0)
    {
        resp[n] = '\0';
        printf("[BT] set name response: %s\n", resp);

        if (strstr(resp, "OK") != NULL)
        {
            printf("[BT] name set to %s, reboot required to take effect\n", name);
            return 0;
        }

        if (strstr(resp, "ERROR") != NULL)
        {
            printf("[BT] set name error: %s\n", name);
            return -1;
        }
    }

    printf("[BT] set name no response: %s\n", name);
    return -1;
}
// ============== 内部函数 ==============

/**
 * @brief 计算校验和（XOR）
 */
// static unsigned char calc_checksum(const char *s)
// {
//     unsigned char checksum = 0;
//     const char *p = strchr(s, '$');
//     if (p)
//         p++;
//     while (p && *p && *p != '*')
//     {
//         checksum ^= *p;
//         p++;
//     }
//     return checksum;
// }

/**
 * @brief 验证校验和
 */
// static int verify_checksum(const char *buf)
// {
//     const char *star = strchr(buf, '*');
//     if (!star)
//         return 0;

//     unsigned char expected = (unsigned char)strtol(star + 1, NULL, 16);
//     unsigned char calculated = calc_checksum(buf);

//     return (expected == calculated);
// }

// ============== 公共函数实现 ==============

int bt_init(void)
{
    char buf[64] = {0};

    // 先不要恢复出厂，避免把透传/波特率/角色改乱
    // char *default_cmd = "AT+DEFAULT=1\r\n";
    bt_set_name();
    char *sleep_cmd = "AT+SLEEPEN=0\r\n"; // 如果模块支持，用这个关闭睡眠
    char *uvar_cmd = "AT+TS=1\r\n";       // 透传模式

    data_send((unsigned char *)sleep_cmd, strlen(sleep_cmd), BT_DEV);
    usleep(200 * 1000);
    int n1 = data_recv((unsigned char *)buf, sizeof(buf) - 1, BT_DEV);
    if (n1 > 0)
    {
        buf[n1] = '\0';
        printf("[BT] SLEEPEN response: %s\n", buf);
    }

    memset(buf, 0, sizeof(buf));
    data_send((unsigned char *)uvar_cmd, strlen(uvar_cmd), BT_DEV);
    usleep(200 * 1000);
    int n2 = data_recv((unsigned char *)buf, sizeof(buf) - 1, BT_DEV);
    if (n2 > 0)
    {
        buf[n2] = '\0';
        printf("[BT] TS response: %s\n", buf);
    }

    printf("[BT] Bluetooth module initialized\n");
    return 0;
}

int bt_is_connected(void)
{
    if (line_bt_status == NULL)
    {
        return 0;
    }
    // BT_STATUS GPIO: 高电平表示已连接
    int value = gpiod_line_get_value(line_bt_status);
    return (value == 1) ? 1 : 0;
}

send_path_t bt_get_send_path(void)
{
    send_path_t path;
    pthread_mutex_lock(&g_path_mutex);
    path = g_send_path;
    pthread_mutex_unlock(&g_path_mutex);
    return path;
}

void bt_set_send_path(send_path_t path)
{
    pthread_mutex_lock(&g_path_mutex);
    g_send_path = path;
    pthread_mutex_unlock(&g_path_mutex);

    const char *path_names[] = {"AUTO", "4G_ONLY", "BD_ONLY", "4G_FIRST", "BD_FIRST"};
    printf("[BT] Send path changed to: %s\n", path_names[path]);
}

int bt_send_heartbeat(const dtu_status_t *status)
{
    char msg[128];
    uint32_t uptime = (uint32_t)(time(NULL) - g_start_time);

    // 格式: $HB,<4g_status>,<4g_signal>,<bt_status>,<send_path>,<uptime>*<checksum>\r\n
    int len = snprintf(msg, sizeof(msg) - 5,
                       "$HB,%d,%d,%d,%d,%u*",
                       status->eg_connected,
                       status->eg_signal,
                       status->bt_connected,
                       status->send_path,
                       uptime);

    unsigned char cs = calc_checksum(msg);
    snprintf(msg + len, sizeof(msg) - len, "%02X\r\n", cs);

    int ret = data_send((unsigned char *)msg, strlen(msg), BT_DEV);
    if (ret > 0)
    {
        // printf("[BT] Heartbeat sent: %s", msg);
        return 0;
    }
    return -1;
}
int bt_send_text(const char *text)
{
    if (text == NULL)
        return -1;

    int ret = data_send((unsigned char *)text, strlen(text), BT_DEV);
    if (ret > 0)
    {
        printf("[BT] Sent: %s", text);
        return 0;
    }
    return -1;
}
int bt_send_heartbeat_simple(uint8_t hb)
{
    char msg[32];
    snprintf(msg, sizeof(msg), "LL_HB_%02X\r\n", hb);
    return bt_send_text(msg);
}
int bt_send_status(const dtu_status_t *status)
{
    char msg[256];
    uint32_t uptime = (uint32_t)(time(NULL) - g_start_time);

    // 格式: $STATUS,<4g>,<4g_sig>,<bt>,<path>,<uptime>,<cnt_4g>,<cnt_bd>,<fail>*<cs>\r\n
    int len = snprintf(msg, sizeof(msg) - 5,
                       "$STATUS,%d,%d,%d,%d,%u,%u,%u,%u*",
                       status->eg_connected,
                       status->eg_signal,
                       status->bt_connected,
                       status->send_path,
                       uptime,
                       status->send_count_4g,
                       status->send_count_bd,
                       status->send_fail_count);

    unsigned char cs = calc_checksum(msg);
    snprintf(msg + len, sizeof(msg) - len, "%02X\r\n", cs);

    int ret = data_send((unsigned char *)msg, strlen(msg), BT_DEV);
    if (ret > 0)
    {
        printf("[BT] Status sent: %s", msg);
        return 0;
    }
    return -1;
}

int bt_send_response(uint8_t cmd_id, uint8_t result, const char *data)
{
    char msg[128];
    int len;

    if (data && strlen(data) > 0)
    {
        len = snprintf(msg, sizeof(msg) - 5, "$RSP,%02X,%02X,%s*", cmd_id, result, data);
    }
    else
    {
        len = snprintf(msg, sizeof(msg) - 5, "$RSP,%02X,%02X*", cmd_id, result);
    }

    unsigned char cs = calc_checksum(msg);
    snprintf(msg + len, sizeof(msg) - len, "%02X\r\n", cs);

    int ret = data_send((unsigned char *)msg, strlen(msg), BT_DEV);
    if (ret > 0)
    {
        printf("[BT] Response sent: %s", msg);
        return 0;
    }
    return -1;
}

// int bt_parse_command(const char *buf, int len, uint8_t *cmd_id, uint8_t *param)
// {
//     // 命令格式: $CMD,<cmd_id>,<param>*<checksum>\r\n
//     if (len < 10 || buf[0] != '$')
//     {
//         return -1;
//     }

//     // 验证校验和
//     if (!verify_checksum(buf))
//     {
//         printf("[BT] Checksum verification failed\n");
//         return -1;
//     }

//     // 解析命令
//     unsigned int id, p;
//     if (sscanf(buf, "$CMD,%x,%x*", &id, &p) == 2)
//     {
//         *cmd_id = (uint8_t)id;
//         *param = (uint8_t)p;
//         return 0;
//     }

//     // 尝试无参数格式
//     if (sscanf(buf, "$CMD,%x*", &id) == 1)
//     {
//         *cmd_id = (uint8_t)id;
//         *param = 0;
//         return 0;
//     }

//     return -1;
// }

/**
 * @brief 处理蓝牙命令
 */
// static void bt_handle_command(uint8_t cmd_id, uint8_t param)
// {
//     printf("[BT] Received command: 0x%02X, param: 0x%02X\n", cmd_id, param);

//     switch (cmd_id)
//     {
//     case BT_CMD_SET_PATH:
//         if (param <= SEND_PATH_BD_FIRST)
//         {
//             bt_set_send_path((send_path_t)param);
//             bt_send_response(cmd_id, BT_RSP_OK, NULL);
//         }
//         else
//         {
//             bt_send_response(cmd_id, BT_RSP_ERR, "Invalid path");
//         }
//         break;

//     case BT_CMD_GET_STATUS:
//     {
//         dtu_status_t status;
//         pthread_mutex_lock(&g_status_mutex);
//         memcpy(&status, &g_dtu_status, sizeof(status));
//         pthread_mutex_unlock(&g_status_mutex);
//         bt_send_status(&status);
//     }
//     break;

//     case BT_CMD_GET_CONFIG:
//     {
//         char config[64];
//         snprintf(config, sizeof(config), "PATH=%d", bt_get_send_path());
//         bt_send_response(cmd_id, BT_RSP_OK, config);
//     }
//     break;

//     case BT_CMD_REBOOT:
//         bt_send_response(cmd_id, BT_RSP_OK, "Rebooting...");
//         sleep(1);
//         // system("reboot");  // 取消注释以启用重启功能
//         printf("[BT] Reboot command received (disabled)\n");
//         break;

//     default:
//         bt_send_response(cmd_id, BT_RSP_INVALID, NULL);
//         break;
//     }
// }

static int bt_send_4g_status_simple(void)
{
    dtu_status_t status;

    pthread_mutex_lock(&g_status_mutex);
    memcpy(&status, &g_dtu_status, sizeof(status));
    pthread_mutex_unlock(&g_status_mutex);

    if (status.eg_connected)
        return bt_send_text("LL_4G_0x01\r\n");
    else
        return bt_send_text("LL_4G_0x00\r\n");
}
/**
 * @brief 更新DTU状态（供其他模块调用）
 */
void bt_update_status(uint8_t eg_connected, uint8_t eg_signal,
                      uint32_t send_count_4g, uint32_t send_count_bd,
                      uint32_t send_fail_count)
{
    pthread_mutex_lock(&g_status_mutex);
    g_dtu_status.eg_connected = eg_connected;
    g_dtu_status.eg_signal = eg_signal;
    g_dtu_status.bt_connected = bt_is_connected();
    g_dtu_status.send_path = bt_get_send_path();
    g_dtu_status.uptime = (uint32_t)(time(NULL) - g_start_time);
    g_dtu_status.send_count_4g = send_count_4g;
    g_dtu_status.send_count_bd = send_count_bd;
    g_dtu_status.send_fail_count = send_fail_count;
    pthread_mutex_unlock(&g_status_mutex);
}
void bt_handle_simple_command(const char *cmd)
{
    unsigned int val;

    if (cmd == NULL || cmd[0] == '\0')
        return;

    printf("[BT] Received cmd: %s\n", cmd);

    if (strcmp(cmd, "LL_00") == 0)
    {
        bt_set_send_path(SEND_PATH_AUTO);
        bt_send_text("LL_ACK_00\r\n");
    }
    else if (strcmp(cmd, "LL_01") == 0)
    {
        bt_set_send_path(SEND_PATH_4G_ONLY);
        bt_send_text("LL_ACK_01\r\n");
    }
    else if (strcmp(cmd, "LL_02") == 0)
    {
        bt_set_send_path(SEND_PATH_BD_ONLY);
        bt_send_text("LL_ACK_02\r\n");
    }
    else if (strcmp(cmd, "LL_03") == 0)
    {
        bt_set_send_path(SEND_PATH_4G_FIRST);
        bt_send_text("LL_ACK_03\r\n");
    }
    else if (strcmp(cmd, "LL_04") == 0)
    {
        bt_set_send_path(SEND_PATH_BD_FIRST);
        bt_send_text("LL_ACK_04\r\n");
    }
    // 获取4g连接状态
    else if (strcmp(cmd, "LL_0a") == 0 || strcmp(cmd, "LL_0A") == 0)
    {
        bt_send_4g_status_simple();
    }
    else if (strcmp(cmd, "LL_LORA_GET") == 0)
    {
        bt_send_lora_cfg();
    }
    else if (sscanf(cmd, "LL_ROOT_%x", &val) == 1)
    {
        if (val == LORA_MESH_ROOT || val == LORA_MESH_NOTROOT)
        {
            lora_cfg_set(0, (uint8_t)val);
            if (lora_cfg_save_persist() == 0)
                bt_send_text("LL_ACK_ROOT\r\n");
            else
                bt_send_text("LL_ACK_SAVE_ERR\r\n");
        }
        else
        {
            bt_send_text("LL_ACK_ERR\r\n");
        }
    }
    else if (sscanf(cmd, "LL_MESH_%x", &val) == 1)
    {
        if (val == LORA_MESH_GATEWAY || val == LORA_MESH_NODE)
        {
            lora_cfg_set(1, (uint8_t)val);
            if (lora_cfg_save_persist() == 0)
                bt_send_text("LL_ACK_MESH\r\n");
            else
                bt_send_text("LL_ACK_SAVE_ERR\r\n");
        }
        else
        {
            bt_send_text("LL_ACK_ERR\r\n");
        }
    }
    else if (sscanf(cmd, "LL_NET_%x", &val) == 1)
    {
        lora_cfg_set(2, (uint8_t)val);
        if (lora_cfg_save_persist() == 0)
            bt_send_text("LL_ACK_NET\r\n");
        else
            bt_send_text("LL_ACK_SAVE_ERR\r\n");
    }
    else if (sscanf(cmd, "LL_DEV_%x", &val) == 1)
    {
        lora_cfg_set(3, (uint8_t)val);
        if (lora_cfg_save_persist() == 0)
            bt_send_text("LL_ACK_DEV\r\n");
        else
            bt_send_text("LL_ACK_SAVE_ERR\r\n");
    }
    else
    {
        bt_send_text("LL_ACK_ERR\r\n");
    }
}
