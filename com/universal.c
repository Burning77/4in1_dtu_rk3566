#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "../inc/universal.h"
#include "string.h"
#include "../inc/usart.h"
#include "../inc/kfifo.h"
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <string.h>

#define FRAME_TIMEOUT_MS 10
#define BUFFER_SIZE 1024
#define LOG_COMPACT_MAX_LINE 1024
#define LOG_COMPACT_MAX_KEEP_FRAMES 32
typedef struct
{
    uint32_t id;
    int valid;
} kept_frame_t;
static pthread_mutex_t g_log_frame_id_lock = PTHREAD_MUTEX_INITIALIZER;
static uint32_t g_log_frame_id = 0;
uint16_t crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}
int hex_to_bytes(const char *hex_str, unsigned char *out_bytes, int max_len)
{
    if (!hex_str || !out_bytes || max_len <= 0)
        return -1;
    int byte_count = 0;
    const char *p = hex_str;
    while (*p && byte_count < max_len)
    {
        if (*p == ',')
        {
            p++;
            continue;
        }
        unsigned int byte;
        if (sscanf(p, "%2x", &byte) != 1)
            break;
        out_bytes[byte_count++] = (unsigned char)byte;
        p += 2;
    }
    return byte_count;
}
unsigned char calc_checksum(const char *s)
{
    unsigned char checksum = 0;
    // ��$��ĵ�һ���ַ����?
    const char *p = strchr(s, '$');
    if (p)
        p++; // ����$
    while (p && *p && *p != '*')
    {
        checksum ^= *p;
        p++;
    }
    return checksum;
}

int parse_log_line(const char *line, unsigned char *out_data, int max_len)
{
    // 跳过时间戳部分：找到 ']' 后的空格
    char *data_start = strchr(line, ']');
    if (!data_start)
        return 0;
    data_start++; // 跳过 ']'
    while (*data_start == ' ')
        data_start++;
    while (strncmp(data_start, "F=", 2) == 0 ||
           strncmp(data_start, "P=", 2) == 0)
    {
        while (*data_start && *data_start != ' ')
            data_start++;
        while (*data_start == ' ')
            data_start++;
    }
    // 解析十六进制字节
    int count = 0;
    char *p = data_start;
    while (*p && count < max_len)
    {
        // 跳过空格
        while (*p == ' ')
            p++;
        if (!*p)
            break;
        // 提取两个十六进制字符
        unsigned int byte;
        if (sscanf(p, "%2x", &byte) != 1)
            break;
        out_data[count++] = (unsigned char)byte;
        p += 2;
    }
    return count;
}

// 静态函数：检查是否为垃圾数据
static int is_garbage_data(serial_state_t *state)
{
    // 过滤条件1：单字节 0x00 数据（RS485 空闲噪声）
    if (state->frame_len == 1 && state->frame_buf[0] == 0x00)
    {
        return 1;
    }

    // 过滤条件2：全零数据
    int all_zero = 1;
    for (int i = 0; i < state->frame_len; i++)
    {
        if (state->frame_buf[i] != 0x00)
        {
            all_zero = 0;
            break;
        }
    }
    if (all_zero && state->frame_len > 0)
    {
        return 1;
    }

    return 0;
}

// 静态函数：提交一帧数据
static void submit_frame(serial_state_t *state, frame_processor_ctx_t *ctx, const char *reason)
{
    (void)reason; // 当前 reason 参数未使用，保留以备未来扩展（如日志记录）
    if (state == NULL || ctx == NULL)
    {
        return;
    }

    if (state->frame_len <= 0)
    {
        return;
    }

    if (is_garbage_data(state))
    {
        printf("[%s FILTERED] Garbage data (len=%d, first_byte=0x%02X), skipping\n",
               state->tag, state->frame_len, state->frame_buf[0]);
        return;
    }
    int offset = 0;
    uint32_t frame_id = log_next_frame_id();
    int part_count = (state->frame_len + (int)sizeof(((fifo_message_t *)0)->data) - 1) /
                     (int)sizeof(((fifo_message_t *)0)->data);
    int part_index = 0;
    while (offset < state->frame_len)
    {
        fifo_message_t msg;
        memset(&msg, 0, sizeof(msg));

        int chunk_len = state->frame_len - offset;
        if (chunk_len > (int)sizeof(msg.data))
        {
            chunk_len = sizeof(msg.data);
        }

        msg.type = state->data_type;
        msg.len = chunk_len;
        msg.frame_id = frame_id;
        msg.part_index = part_index++;
        msg.part_count = part_count;
        memcpy(msg.data, state->frame_buf + offset, chunk_len);
        pthread_mutex_lock(ctx->fifo_lock);
        // memcpy(msg.data, state->frame_buf, copy_len);
        if (kfifo_left(ctx->fifo) < sizeof(fifo_message_t))
        {
            pthread_mutex_unlock(ctx->fifo_lock);
            printf("[%s WARN] fifo full, drop remaining %d bytes\n",
                   state->tag, state->frame_len - offset);
            break;
        }

        kfifo_put(ctx->fifo, (unsigned char *)&msg, sizeof(fifo_message_t));
        pthread_cond_signal(ctx->fifo_not_empty);
        pthread_mutex_unlock(ctx->fifo_lock);

        offset += chunk_len;
    }

    // 加锁推入 kfifo
    // pthread_mutex_lock(ctx->fifo_lock);
    // kfifo_put(ctx->fifo, (unsigned char *)&msg, sizeof(fifo_message_t));
    // pthread_cond_signal(ctx->fifo_not_empty);
    // pthread_mutex_unlock(ctx->fifo_lock);

    // 打印调试信息
    // printf("[%s COMPLETE %d bytes (%s)]: ", state->tag, state->frame_len, reason);
    // for (int i = 0; i < state->frame_len && i < 32; i++)
    // { // 限制打印长度
    //     printf("%02X ", state->frame_buf[i]);
    // }
    // if (state->frame_len > 32)
    // {
    //     printf("... (+%d more bytes)", state->frame_len - 32);
    // }
    // printf("\n");
}

// 初始化串口状态
void serial_state_init(serial_state_t *state, int data_type, const char *tag)
{
    if (!state)
        return;
    memset(state->frame_buf, 0, BUFFER_SIZE); // 防残留数据
    state->frame_len = 0;
    state->last_recv = (struct timeval){0};
    memset(state->frame_buf, 0, sizeof(state->frame_buf));
    state->frame_len = 0;
    state->last_recv.tv_sec = 0;
    state->last_recv.tv_usec = 0;
    state->data_type = data_type;
    state->tag = tag;
}

void process_serial_data(serial_state_t *state, unsigned char *read_buf, int read_len, int is_timeout_triggered, frame_processor_ctx_t *ctx)
{
    // 1. 仅校验指针，绝不因 frame_len == 0 拦截新数据
    if (!state || !ctx)
        return;

    struct timeval now;
    gettimeofday(&now, NULL);

    // 2. 安全计算时间差（处理微秒借位）
    long diff_ms = 0;
    if (state->frame_len > 0)
    {
        long sec_diff = now.tv_sec - state->last_recv.tv_sec;
        long usec_diff = now.tv_usec - state->last_recv.tv_usec;
        if (usec_diff < 0)
        {
            sec_diff--;
            usec_diff += 1000000;
        }
        diff_ms = sec_diff * 1000 + usec_diff / 1000;
    }

    // 3. 超时处理：提交旧帧后清空缓冲，但绝对不能 return！
    if (state->frame_len > 0 && diff_ms > FRAME_TIMEOUT_MS)
    {
        submit_frame(state, ctx, "timeout");
        state->frame_len = 0; // 释放空间，准备承接本次可能带来的新数据
    }

    // 4. 处理本次到达的新数据（无论是否刚发生过超时）
    if (read_len > 0)
    {
        // 防驱动异常返回超大值导致越界
        if (read_len > BUFFER_SIZE)
            read_len = BUFFER_SIZE;

        if (state->frame_len + read_len <= BUFFER_SIZE)
        {
            memcpy(state->frame_buf + state->frame_len, read_buf, read_len);
            state->frame_len += read_len;
            state->last_recv = now; // 更新活跃时间戳

            // 达到最大逻辑帧长，立即提交
            if (state->frame_len >= MAX_LOG_LEN)
            {
                submit_frame(state, ctx, "max_len");
                state->frame_len = 0;
            }
        }
        else
        {
            // 缓冲区不足：先提交已累积帧
            submit_frame(state, ctx, "overflow");
            state->frame_len = 0;
            // 策略：保留新数据防丢（若协议允许分包）
            memcpy(state->frame_buf, read_buf, read_len);
            state->frame_len = read_len;
            state->last_recv = now;
        }
    }
}
int push_lora_to_fifo(frame_processor_ctx_t *ctx, const uint8_t *data, uint8_t len)
{
    fifo_message_t msg;

    if (ctx == NULL || ctx->fifo == NULL ||
        ctx->fifo_lock == NULL || ctx->fifo_not_empty == NULL ||
        data == NULL || len == 0)
    {
        return -1;
    }

    if (len > sizeof(msg.data))
    {
        printf("[LORA RX] invalid len=%u, max=%zu\n", len, sizeof(msg.data));
        return -1;
    }

    memset(&msg, 0, sizeof(msg));
    msg.type = LORA_DATA;
    msg.len = len;
    memcpy(msg.data, data, len);

    pthread_mutex_lock(ctx->fifo_lock);

    if (kfifo_left(ctx->fifo) < sizeof(fifo_message_t))
    {
        pthread_mutex_unlock(ctx->fifo_lock);
        printf("[LORA RX] fifo full, drop %u bytes\n", len);
        return -1;
    }

    kfifo_put(ctx->fifo, (unsigned char *)&msg, sizeof(fifo_message_t));
    pthread_cond_signal(ctx->fifo_not_empty);
    pthread_mutex_unlock(ctx->fifo_lock);

    return 0;
}
// // 处理串口数据
// void process_serial_data(serial_state_t *state,
//                          unsigned char *read_buf, int read_len,
//                          int is_timeout_triggered,
//                          frame_processor_ctx_t *ctx)
// {
//     if (!state || !ctx)
//     {
//         return;
//     }

//     struct timeval now;
//     gettimeofday(&now, NULL);

//     // 计算距离上次接收的时间差（毫秒）
//     long diff_ms = 0;
//     if (state->frame_len > 0)
//     {
//         diff_ms = (now.tv_sec - state->last_recv.tv_sec) * 1000 +
//                   (now.tv_usec - state->last_recv.tv_usec) / 1000;
//     }
//     if (state->frame_len > 0 && diff_ms > FRAME_TIMEOUT_MS)
//     {
//         submit_frame(state, ctx, "timeout");
//         state->frame_len = 0;
//         // 提交后，如果本次有 read_len > 0，下面会处理新数据
//     }
//     if (state->frame_len >= MAX_LOG_LEN)
//     {
//         submit_frame(state, ctx, "max_len");
//         state->frame_len = 0;
//     }

//     // 情况A: 有新数据到达
//     if (read_len > 0)
//     {
//         // 检查缓冲区是否有足够空间
//         if (state->frame_len + read_len < BUFFER_SIZE)
//         {
//             memcpy(state->frame_buf + state->frame_len, read_buf, read_len);
//             state->frame_len += read_len;
//             state->last_recv = now; // 更新最后接收时间
//             if (state->frame_len >= MAX_LOG_LEN)
//             {
//                 submit_frame(state, ctx, "max_len");
//                 state->frame_len = 0;
//             }
//         }
//         else
//         {
//             // 缓冲区溢出，提交当前帧
//             submit_frame(state, ctx, "overflow");
//             state->frame_len = 0;
//             // 尝试存储新数据
//             if (read_len < BUFFER_SIZE)
//             {
//                 memcpy(state->frame_buf, read_buf, read_len);
//                 state->frame_len = read_len;
//                 state->last_recv = now;
//             }
//         }
//         return;
//     }
//     return;
// }

// 强制提交缓冲区中的残留帧
void flush_serial_state(serial_state_t *state, frame_processor_ctx_t *ctx)
{
    if (!state || !ctx)
    {
        return;
    }

    if (state->frame_len > 0)
    {
        submit_frame(state, ctx, "flush");
        state->frame_len = 0;
    }
}
int load_offsets(const char *path, off_t *offsets, int count)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0)
    {
        if (errno == ENOENT)
        {
            memset(offsets, 0, sizeof(off_t) * count);
            return 0; // 首次运行，无历史数据
        }
        perror("load_offsets open");
        return -1;
    }
    ssize_t n = read(fd, offsets, sizeof(off_t) * count);
    close(fd);
    if (n != sizeof(off_t) * count)
    {
        // 文件内容不完整，重置为0
        memset(offsets, 0, sizeof(off_t) * count);
    }
    return 0;
}

int save_offsets(const char *path, const off_t *offsets, int count)
{
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0)
    {
        perror("save_offsets open");
        return -1;
    }
    ssize_t n = write(fd, offsets, sizeof(off_t) * count);
    close(fd);
    if (n != sizeof(off_t) * count)
    {
        perror("save_offsets write");
        return -1;
    }
    return 0;
}
// 将原始数据打包成北斗协议并发送

/**
 * 从多个日志文件中读取新行，解析原始数据并打包成十六进制字符串（以逗号分隔）。
 * 每个线程独立维护自己的文件偏移量，互不干扰。
 *
 * @param paths         文件路径数组
 * @param offsets       对应文件的偏移量指针数组（每个线程独立维护）
 * @param file_count    文件数量
 * @param max_hex_len   打包后十六进制字符串的最大长度（不含结尾 '\0'）
 * @param out_hex_buf   输出缓冲区，用于存放打包后的十六进制字符串（调用者分配，建议大小为 max_hex_len + 64）
 * @param out_hex_len   输出实际十六进制字符数
 * @param out_entry_count 输出打包的原始数据条目数
 * @return              0 表示成功（即使无数据也返回0，通过 out_entry_count 判断是否有数据），负数表示错误（如文件打开失败）
 */
int pack_data_from_files(const char **paths, off_t *offsets, int file_count,
                         int max_hex_len, char *out_hex_buf,
                         int *out_hex_len, int *out_entry_count)
{
    if (!paths || !offsets || file_count <= 0 || !out_hex_buf || !out_hex_len || !out_entry_count)
        return -1;

    out_hex_buf[0] = '\0';
    *out_hex_len = 0;
    *out_entry_count = 0;

    for (int i = 0; i < file_count; i++)
    {
        FILE *fp = fopen(paths[i], "r");
        if (!fp)
        {
            // 文件可能不存在，跳过（不视为错误）
            continue;
        }

        // 定位到上次发送位置
        fseeko(fp, offsets[i], SEEK_SET);

        char line[512];
        while (fgets(line, sizeof(line), fp))
        {
            unsigned char raw[256];
            int raw_len = parse_log_line(line, raw, sizeof(raw));
            if (raw_len < 2) // 跳过无效行（至少1字节有效数据）
                continue;

            int need_hex = raw_len * 2; // 需要的十六进制字符数
            int total_need = *out_hex_len + (*out_entry_count > 0 ? 1 : 0) + need_hex;

            if (total_need > max_hex_len)
            {
                // 空间不足，回退一行，留待下次处理
                fseeko(fp, -(off_t)strlen(line), SEEK_CUR);
                break;
            }

            // 添加逗号分隔（除了第一个条目）
            if (*out_entry_count > 0)
            {
                strcat(out_hex_buf, ",");
                (*out_hex_len)++;
            }

            // 将原始数据转换为大写十六进制追加到 out_hex_buf
            char hex_part[512];
            char *p = hex_part;
            for (int j = 0; j < raw_len; j++)
            {
                p += sprintf(p, "%02X", raw[j]);
            }
            strcat(out_hex_buf, hex_part);
            *out_hex_len += need_hex;
            (*out_entry_count)++;

            // 更新偏移量
            offsets[i] = ftello(fp);
        }

        fclose(fp);
    }

    return 0;
}
uint32_t log_next_frame_id(void)
{
    uint32_t id;

    pthread_mutex_lock(&g_log_frame_id_lock);
    id = ++g_log_frame_id;
    if (g_log_frame_id == 0)
    {
        g_log_frame_id = 1;
        id = 1;
    }
    pthread_mutex_unlock(&g_log_frame_id_lock);

    return id;
}
static int parse_log_frame_id(const char *line, uint32_t *frame_id)
{
    const char *p;

    if (!line || !frame_id)
        return 0;

    p = strstr(line, "] F=");
    if (!p)
        return 0;

    p += 4;

    if (sscanf(p, "%u", frame_id) != 1)
        return 0;

    return 1;
}

static int frame_id_is_kept(kept_frame_t *frames, int count, uint32_t frame_id)
{
    for (int i = 0; i < count; i++)
    {
        if (frames[i].valid && frames[i].id == frame_id)
            return 1;
    }

    return 0;
}

static void keep_frame_id(kept_frame_t *frames, int keep_frames, uint32_t frame_id)
{
    for (int i = 0; i < keep_frames; i++)
    {
        if (frames[i].valid && frames[i].id == frame_id)
            return;
    }

    for (int i = 0; i < keep_frames - 1; i++)
        frames[i] = frames[i + 1];

    frames[keep_frames - 1].id = frame_id;
    frames[keep_frames - 1].valid = 1;
}

int compact_log_file(const char *path, off_t *offset, int keep_frames)
{
    FILE *fp = NULL;
    FILE *out = NULL;
    char tmp_path[256];
    char line[LOG_COMPACT_MAX_LINE];
    kept_frame_t kept[LOG_COMPACT_MAX_KEEP_FRAMES] = {0};
    off_t sent_end;
    off_t new_offset = 0;
    uint32_t legacy_frame_id = 0;

    if (!path || !offset || *offset <= 0)
        return 0;

    if (keep_frames <= 0)
        return 0;

    if (keep_frames > LOG_COMPACT_MAX_KEEP_FRAMES)
        keep_frames = LOG_COMPACT_MAX_KEEP_FRAMES;

    fp = fopen(path, "r");
    if (!fp)
        return 0;

    sent_end = *offset;

    while (ftello(fp) < sent_end && fgets(line, sizeof(line), fp))
    {
        uint32_t frame_id;

        if (!parse_log_frame_id(line, &frame_id))
        {
            frame_id = ++legacy_frame_id;
        }

        keep_frame_id(kept, keep_frames, frame_id);
    }

    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);

    out = fopen(tmp_path, "w");
    if (!out)
    {
        fclose(fp);
        return -1;
    }

    rewind(fp);
    legacy_frame_id = 0;

    while (ftello(fp) < sent_end && fgets(line, sizeof(line), fp))
    {
        uint32_t frame_id;

        if (!parse_log_frame_id(line, &frame_id))
        {
            frame_id = ++legacy_frame_id;
        }

        if (frame_id_is_kept(kept, keep_frames, frame_id))
        {
            fputs(line, out);
            new_offset += strlen(line);
        }
    }

    fseeko(fp, sent_end, SEEK_SET);

    while (fgets(line, sizeof(line), fp))
    {
        fputs(line, out);
    }

    fflush(out);
    fsync(fileno(out));
    fclose(out);
    fclose(fp);

    if (rename(tmp_path, path) != 0)
    {
        remove(tmp_path);
        return -1;
    }

    *offset = new_offset;
    return 0;
}

time_t monotonic_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec;
}