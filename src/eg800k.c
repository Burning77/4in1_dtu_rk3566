#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdatomic.h>
#include "../inc/universal.h"
#include "../inc/eg800k.h"
#include "../inc/usart.h"

static int eg_connect_id = 0; // Socket 连接 ID
static int eg_context_id = 1; // PDP 场景 ID
extern int eg_fd;
extern volatile sig_atomic_t stop_flag;

// 发送 AT 命令并等待指定响应（带超时，可被 stop_flag 中断）
// int eg_send_cmd(const char *cmd, const char *expected_resp, int timeout_sec)
// {
//     char buf[512];
//     data_send((unsigned char *)cmd, strlen(cmd), EG_DEV);

//     fd_set fds;
//     struct timeval tv;
//     int ret;
//     int elapsed = 0;

//     // 使用 1 秒超时循环，检查 stop_flag
//     while (elapsed < timeout_sec && !stop_flag)
//     {
//         FD_ZERO(&fds);
//         FD_SET(eg_fd, &fds);
//         tv.tv_sec = 1;
//         tv.tv_usec = 0;

//         ret = select(eg_fd + 1, &fds, NULL, NULL, &tv);
//         if (ret > 0)
//         {
//             int n = data_recv(buf, sizeof(buf) - 1, EG_DEV);
//             if (n <= 0)
//                 return -1;
//             buf[n] = '\0';

//             if (expected_resp && strstr(buf, expected_resp) == NULL)
//             {
//                 printf("[EG] Unexpected response: %s\r\n", buf);
//                 return -1;
//             }
//             return 0;
//         }
//         elapsed++;
//     }

//     if (stop_flag)
//     {
//         printf("[EG] Command interrupted by stop_flag\n");
//         return -1;
//     }

//     printf("[EG] Timeout waiting for response to: %s\n", cmd);
//     return -1;
// }
int eg_send_cmd(const char *cmd, const char *expected_resp, int timeout_sec)
{
    char buf[512] = {0};
    int total = 0;
    fd_set fds;
    struct timeval tv, start, now;

    // 发送命令前清空接收缓冲区（可选，避免残留数据干扰）
    tcflush(eg_fd, TCIFLUSH);
    data_send((unsigned char *)cmd, strlen(cmd), EG_DEV);
    printf("[EG] Sent: %s", cmd);

    gettimeofday(&start, NULL);
    while (!stop_flag)
    {
        // 计算是否超时
        gettimeofday(&now, NULL);
        long elapsed = (now.tv_sec - start.tv_sec) * 1000 +
                       (now.tv_usec - start.tv_usec) / 1000;
        if (elapsed >= timeout_sec * 1000)
        {
            printf("[EG] Timeout: %s", cmd);
            return -1;
        }

        FD_ZERO(&fds);
        FD_SET(eg_fd, &fds);
        tv.tv_sec = 0;
        tv.tv_usec = 200000; // 200ms 等待新数据

        int ret = select(eg_fd + 1, &fds, NULL, NULL, &tv);
        if (ret < 0)
        {
            if (stop_flag)
                break;
            continue;
        }
        if (ret == 0)
            continue;

        // 读取新数据并追加
        int n = data_recv(buf + total, sizeof(buf) - 1 - total, EG_DEV);
        if (n > 0)
        {
            total += n;
            buf[total] = '\0';
            printf("[EG] Recv: %s\n", buf);

            // 如果收到预期响应则成功
            if (expected_resp && strstr(buf, expected_resp))
                return 0;
            // 如果收到 ERROR 则提前失败
            if (strstr(buf, "ERROR"))
                return -1;
        }
    }

    if (stop_flag)
    {
        printf("[EG] Command interrupted by stop_flag\n");
        return -1;
    }
    return -1;
}
// 初始化 4G 模块（仅执行一次）
int eg_init(void)
{
    printf("[EG] Starting initialization...\n");

    // 1. AT 同步
    if (eg_send_cmd("AT\r\n", "OK", 2) != 0)
        return -1;
    if (eg_send_cmd("ATI\r\n", "Revision", 8) != 0)
    {
        printf("[EG] Module not responding correctly to ATI\n");
        return -1;
    }
    // 2. 查询 SIM 卡
    if (eg_send_cmd("AT+CPIN?\r\n", "READY", 5) != 0)
    {
        printf("[EG] SIM card not ready\n");
        return -1;
    }

    // 3. 查询信号质量（仅打印，不强制要求）
    eg_send_cmd("AT+CSQ\r\n", NULL, 2);

    // 4. 查询网络注册（LTE）
    if (eg_send_cmd("AT+CEREG?\r\n", "1", 10) != 0)
    {
        printf("[EG] Network registration failed\n");
        return -1;
    }

    // 5. 配置 APN（请根据 SIM 卡运营商修改）
    // 中国联通: "UNINET", 中国移动: "CMNET", 中国电信: "CTNET"
    if (eg_send_cmd("AT+QICSGP=1,1,\"CMNET\",\"\",\"\",1\r\n", "OK", 5) != 0)
    {
        printf("[EG] APN configuration failed\n");
        return -1;
    }

    // 6. 激活 PDP 场景（最长 150 秒）
    if (eg_send_cmd("AT+QIACT=1\r\n", "OK", 150) != 0)
    {
        printf("[EG] PDP activation failed\n");
        return -1;
    }

    // 7. 查询 IP 地址（可选）
    eg_send_cmd("AT+QIACT?\r\n", NULL, 2);

    printf("[EG] Initialization complete\n");
    return 0;
}
// 发送数据（通过已建立的 TCP 连接）
int eg_send_data(const unsigned char *data, int len)
{
    char cmd[32];
    char buf[256] = {0};
    int total = 0;
    fd_set fds;
    struct timeval tv, start, now;

    // 清空接收缓冲区
    tcflush(eg_fd, TCIFLUSH);

    // 发送 AT+QISEND 命令
    snprintf(cmd, sizeof(cmd), "AT+QISEND=%d,%d\r\n", eg_connect_id, len);
    data_send((unsigned char *)cmd, strlen(cmd), EG_DEV);
    printf("[EG] Sent: %s", cmd);

    // 等待 '>' 提示符（最多 5 秒）
    gettimeofday(&start, NULL);
    while (!stop_flag)
    {
        gettimeofday(&now, NULL);
        long elapsed = (now.tv_sec - start.tv_sec) * 1000 +
                       (now.tv_usec - start.tv_usec) / 1000;
        if (elapsed >= 5000)
        {
            printf("[EG] No '>' prompt timeout\n");
            return -1;
        }

        FD_ZERO(&fds);
        FD_SET(eg_fd, &fds);
        tv.tv_sec = 0;
        tv.tv_usec = 200000;

        int ret = select(eg_fd + 1, &fds, NULL, NULL, &tv);
        if (ret <= 0)
            continue;

        int n = data_recv(buf + total, sizeof(buf) - 1 - total, EG_DEV);
        if (n > 0)
        {
            total += n;
            buf[total] = '\0';
            if (strstr(buf, ">"))
            {
                printf("[EG] Got '>' prompt\n");
                break;
            }
            if (strstr(buf, "ERROR"))
            {
                printf("[EG] QISEND error: %s\n", buf);
                return -1;
            }
        }
    }

    // 发送原始数据（不加 Ctrl+Z，使用固定长度模式）
    data_send((unsigned char *)data, len, EG_DEV);
    printf("[EG] Sent %d bytes data\n", len);

    // 等待 SEND OK（最多 5 秒，累积所有响应）
    memset(buf, 0, sizeof(buf));
    total = 0;
    gettimeofday(&start, NULL);
    while (!stop_flag)
    {
        gettimeofday(&now, NULL);
        long elapsed = (now.tv_sec - start.tv_sec) * 1000 +
                       (now.tv_usec - start.tv_usec) / 1000;
        if (elapsed >= 5000)
        {
            printf("[EG] SEND OK timeout, received so far: [%s]\n", buf);
            // 即使超时，如果服务器收到了数据，也算成功
            // 检查是否有部分成功的迹象
            return -1;
        }

        FD_ZERO(&fds);
        FD_SET(eg_fd, &fds);
        tv.tv_sec = 0;
        tv.tv_usec = 500000; // 500ms

        int ret = select(eg_fd + 1, &fds, NULL, NULL, &tv);
        if (ret <= 0)
            continue;

        int n = data_recv(buf + total, sizeof(buf) - 1 - total, EG_DEV);
        if (n > 0)
        {
            total += n;
            buf[total] = '\0';
            // 只打印非空响应
            if (n > 0 && buf[total - n] != '\0')
            {
                printf("[EG] Recv (%d bytes): [%s]\n", n, buf + total - n);
            }

            if (strstr(buf, "SEND OK"))
            {
                printf("[EG] Data sent successfully (%d bytes)\n", len);
                return 0;
            }
            if (strstr(buf, "ERROR") || strstr(buf, "SEND FAIL") || strstr(buf, "CLOSED"))
            {
                printf("[EG] Send failed: %s\n", buf);
                return -1;
            }
        }
    }
    return -1;  // stop_flag 中断
}

// 检查 4G 网络是否可用（信号正常 + 已注册）
int eg_is_network_available(void)
{
    char buf[64];
    int rssi, stat;

    // 1. 查询信号质量
    data_send((unsigned char *)"AT+CSQ\r\n", strlen("AT+CSQ\r\n"), EG_DEV);
    fd_set fds;
    struct timeval tv;
    FD_ZERO(&fds);
    FD_SET(eg_fd, &fds);
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    int ret = select(eg_fd + 1, &fds, NULL, NULL, &tv);
    if (ret <= 0)
        return 0;
    int n = data_recv(buf, sizeof(buf) - 1, EG_DEV);
    if (n <= 0)
        return 0;
    buf[n] = '\0';
    if (sscanf(buf, "+CSQ: %d,%d", &rssi, &stat) != 2)
        return 0;
    if (rssi == 99)
        return 0; // 未知信号

    // 2. 查询 LTE 注册状态
    data_send((unsigned char *)"AT+CEREG?\r\n", strlen("AT+CEREG?\r\n"), EG_DEV);
    FD_ZERO(&fds);
    FD_SET(eg_fd, &fds);
    tv.tv_sec = 2;
    ret = select(eg_fd + 1, &fds, NULL, NULL, &tv);
    if (ret <= 0)
        return 0;
    n = data_recv(buf, sizeof(buf) - 1, EG_DEV);
    if (n <= 0)
        return 0;
    buf[n] = '\0';
    int n_val, reg_stat;
    if (sscanf(buf, "+CEREG: %d,%d", &n_val, &reg_stat) == 2)
    {
        if (reg_stat == 1 || reg_stat == 5)
            return 1; // 已注册或漫游
    }
    return 0;
}
int server_port = SERVER_PORT;
int eg_connect(void)
{
    const char *server_ip = SERVER_IP;
    int port = server_port;
    char tcp_cmd[128];
    snprintf(tcp_cmd, sizeof(tcp_cmd), 
             "AT+QIOPEN=1,0,\"TCP\",\"%s\",%d,0,0\r\n", server_ip, port);
    char buf[256];

    // 先关闭可能存在的旧连接
    eg_send_cmd("AT+QICLOSE=0\r\n", "OK", 3);
    tcflush(eg_fd, TCIFLUSH);

    printf("[EG] Connecting to %s:%d...\n", server_ip, port);
    data_send((unsigned char *)tcp_cmd, strlen(tcp_cmd), EG_DEV);

    // 等待 +QIOPEN: 0,x （最长 60 秒，可被 stop_flag 中断）
    // AT+QIOPEN 会先返回 OK，然后异步返回 +QIOPEN: <connectID>,<err>
    // err=0 表示成功，其他值表示失败
    fd_set fds;
    struct timeval tv;
    int elapsed = 0;
    const int timeout_sec = 60;

    while (elapsed < timeout_sec && !stop_flag)
    {
        FD_ZERO(&fds);
        FD_SET(eg_fd, &fds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select(eg_fd + 1, &fds, NULL, NULL, &tv);
        if (ret > 0)
        {
            int n = data_recv(buf, sizeof(buf) - 1, EG_DEV);
            if (n > 0)
            {
                buf[n] = '\0';
                // 累积响应
                // if (total_len + n < (int)sizeof(total_buf) - 1) {
                //     memcpy(total_buf + total_len, buf, n);
                //     total_len += n;
                //     total_buf[total_len] = '\0';
                // }
                // printf("[EG] Recv: %s\n", buf);

                // // 检查是否收到 +QIOPEN 响应
                // if (strstr(total_buf, "+QIOPEN: 0,0") != NULL)
                // {
                //     printf("[EG] TCP connected successfully\n");
                //     return 0;
                // }
                // // 检查连接失败（+QIOPEN: 0,非零错误码）
                // char *qiopen = strstr(total_buf, "+QIOPEN: 0,");
                // if (qiopen != NULL)
                // {
                //     int err_code = atoi(qiopen + 11);
                //     if (err_code != 0) {
                //         printf("[EG] TCP connect failed, error code: %d\n", err_code);
                //         return -1;
                //     }
                // }
                // // 检查 ERROR 响应
                // if (strstr(total_buf, "ERROR") != NULL)
                // {
                //     printf("[EG] TCP connect error\n");
                //     return -1;
                // }
                // // 收到 OK 只是命令确认，继续等待 +QIOPEN
                char *line = strtok(buf, "\r\n");
                while (line)
                {
                    printf("[EG LINE] %s\n", line);

                    // 1️⃣ 处理 QIOPEN
                    if (strncmp(line, "+QIOPEN:", 8) == 0)
                    {
                        int id, err;
                        sscanf(line, "+QIOPEN: %d,%d", &id, &err);

                        if (err == 0)
                        {
                            printf("[EG] TCP connected successfully\n");
                            return 0;
                        }
                        else
                        {
                            printf("[EG] TCP connect failed, error code: %d\n", err);
                            return -1;
                        }
                    }
                    // 2️⃣ ERROR
                    if (strcmp(line, "ERROR") == 0)
                    {
                        printf("[EG] TCP connect error\n");
                        return -1;
                    }
                    // 3️⃣ OK（忽略，不代表成功）
                    if (strcmp(line, "OK") == 0)
                    {
                        // 只是确认命令发送成功，不做处理
                    }
                    line = strtok(NULL, "\r\n");
                }
            }
        }
        elapsed++;
    }

    if (stop_flag)
    {
        printf("[EG] TCP connect interrupted by stop_flag\n");
        return -1;
    }

    printf("[EG] TCP connect timeout (no +QIOPEN response)\n");
    return -1;
}

int eg_reinit_pdp(void)
{
    printf("[EG] Re-initializing PDP...\n");
    // 先去激活（清理本地状态）
    eg_send_cmd("AT+QIDEACT=1\r\n", "OK", 10);
    // 重新激活 PDP
    if (eg_send_cmd("AT+QIACT=1\r\n", "OK", 150) != 0)
    {
        printf("[EG] PDP reactivation failed\n");
        return -1;
    }
    // 重新建立 TCP 连接
    if (eg_connect() != 0)
    {
        printf("[EG] Reconnect after PDP reactivation failed\n");
        return -1;
    }
    printf("[EG] PDP re-initialization successful\n");
    return 0;
}