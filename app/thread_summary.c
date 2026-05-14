#include "../app/thread_summary.h"
#include "../inc/eg800k.h"
#include "../inc/watch_dog.h"
#include "../inc/bluetooth.h"
#include <sys/ioctl.h>
#include "../inc/power.h"
#include "../inc/bd3.h"
#define DEBUG

#define MAIN_IDLE_WAKE_POLL_SEC 60
#define MAIN_SEND_RETRY_WAIT_SEC 10
extern struct kfifo data_fifo;

static pthread_mutex_t fifo_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t fifo_not_empty = PTHREAD_COND_INITIALIZER;
extern volatile sig_atomic_t stop_flag;
extern int rtc_fd;
int data_num = 1;
static uint8_t rs485_frame_buf[1024];
static int rs485_frame_len = 0;
static struct timeval rs485_last_recv = {0};
static uint8_t rs232_frame_buf[1024];
static int rs232_frame_len = 0;
static struct timeval rs232_last_recv = {0};
static uint8_t bd_frame_buf[256];
static int bd_frame_len = 0;
static struct timeval bd_last_recv = {0};
static eg_state_t eg_state = EG_STATE_INIT;
// 4G 网络可用性标志（由监测线程更新）
volatile int g_4g_available = 0; // 0=不可用, 1=可用
pthread_mutex_t g_4g_mutex = PTHREAD_MUTEX_INITIALIZER;
static volatile int g_pdp_deact = 0;
static pthread_mutex_t g_pdp_mutex = PTHREAD_MUTEX_INITIALIZER;
// 4G 初始化完成标志（防止 receive_thread 抢占 eg_fd 数据）
static volatile int g_eg_init_done = 0;
static pthread_mutex_t g_eg_init_mutex = PTHREAD_MUTEX_INITIALIZER;
// 发送统计
static uint32_t g_send_count_4g = 0;
static uint32_t g_send_count_bd = 0;
static uint32_t g_send_fail_count = 0;
static send_path_t g_send_path = SEND_PATH_AUTO;
static pthread_mutex_t g_path_mutex = PTHREAD_MUTEX_INITIALIZER;

static dtu_status_t g_dtu_status = {0};
static pthread_mutex_t g_status_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t g_lora_cfg_mutex = PTHREAD_MUTEX_INITIALIZER;
static time_t g_bt_start_time = 0;
static pthread_mutex_t g_debug_bt_bd_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_data_wakeup_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_data_wakeup_cond = PTHREAD_COND_INITIALIZER;
static unsigned int g_data_wakeup_seq = 0;
extern int rs485_fd;
extern int rs232_fd;
extern int bd_fd;
extern int bt_fd;
extern int eg_fd;
extern int watchdog_fd;
extern loRa_Para_t my_lora_config;
static int offsets_reached(const off_t *offsets, const off_t *end_offsets)
{
    for (int i = 0; i < MAIN_PATH_COUNT; i++)
    {
        if (offsets[i] < end_offsets[i])
            return 0;
    }

    return 1;
}
static int main_pack_pending_until(const char **paths,
                                   const off_t *offsets,
                                   const off_t *end_offsets,
                                   off_t *pending_offsets,
                                   int max_hex_len,
                                   char *hex_buf,
                                   unsigned char *raw_buf,
                                   int raw_buf_size,
                                   int *raw_len,
                                   int *entry_count)
{
    int hex_len = 0;

    memcpy(pending_offsets, offsets, sizeof(off_t) * MAIN_PATH_COUNT);

    hex_buf[0] = '\0';
    *raw_len = 0;
    *entry_count = 0;

    for (int i = 0; i < MAIN_PATH_COUNT; i++)
    {
        FILE *fp = fopen(paths[i], "r");
        if (!fp)
            continue;

        if (pending_offsets[i] >= end_offsets[i])
        {
            fclose(fp);
            continue;
        }

        if (fseeko(fp, pending_offsets[i], SEEK_SET) != 0)
        {
            fclose(fp);
            return -1;
        }

        char line[1024];

        while (ftello(fp) < end_offsets[i] && fgets(line, sizeof(line), fp))
        {
            off_t line_end = ftello(fp);

            if (line_end > end_offsets[i])
                break;

            unsigned char raw[256];
            int line_raw_len = parse_log_line(line, raw, sizeof(raw));

            if (line_raw_len < 2)
            {
                pending_offsets[i] = line_end;
                continue;
            }

            int comma_len = (*entry_count > 0) ? 1 : 0;
            int need_hex = line_raw_len * 2;

            if (hex_len + comma_len + need_hex > max_hex_len)
                break;

            if (*entry_count > 0)
                hex_buf[hex_len++] = ',';

            for (int j = 0; j < line_raw_len; j++)
            {
                snprintf(hex_buf + hex_len,
                         max_hex_len + 1 - hex_len,
                         "%02X",
                         raw[j]);
                hex_len += 2;
            }

            hex_buf[hex_len] = '\0';
            (*entry_count)++;
            pending_offsets[i] = line_end;
        }

        fclose(fp);
    }

    if (*entry_count == 0)
        return 0;

    *raw_len = hex_to_bytes(hex_buf, raw_buf, raw_buf_size);
    return (*raw_len > 0) ? 0 : -1;
}
static int main_pack_pending(const char **paths,
                             const off_t *offsets,
                             off_t *pending_offsets,
                             int max_hex_len,
                             char *hex_buf,
                             unsigned char *raw_buf,
                             int raw_buf_size,
                             int *raw_len,
                             int *entry_count)
{
    memcpy(pending_offsets, offsets, sizeof(off_t) * MAIN_PATH_COUNT);

    if (pack_data_from_files(paths, pending_offsets, MAIN_PATH_COUNT,
                             max_hex_len, hex_buf, raw_len, entry_count) != 0)
    {
        return -1;
    }

    if (*entry_count == 0)
    {
        *raw_len = 0;
        return 0;
    }

    *raw_len = hex_to_bytes(hex_buf, raw_buf, raw_buf_size);
    return (*raw_len > 0) ? 0 : -1;
}
static void main_wait_for_data_wakeup(unsigned int *seen_seq, int timeout_sec)
{
    struct timespec ts;

    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout_sec;

    pthread_mutex_lock(&g_data_wakeup_mutex);

    while (!stop_flag && g_data_wakeup_seq == *seen_seq)
    {
        int ret = pthread_cond_timedwait(&g_data_wakeup_cond,
                                         &g_data_wakeup_mutex,
                                         &ts);
        if (ret == ETIMEDOUT)
            break;
    }

    if (g_data_wakeup_seq != *seen_seq)
    {
        printf("[MAIN] Data wakeup received, seq=%u -> %u\n",
               *seen_seq, g_data_wakeup_seq);
    }

    *seen_seq = g_data_wakeup_seq;
    pthread_mutex_unlock(&g_data_wakeup_mutex);
}

#ifdef DEBUG
static volatile int g_debug_eg_connected = 0;
static pthread_mutex_t g_debug_eg_mutex = PTHREAD_MUTEX_INITIALIZER;

#define DEBUG_BT_BD_INTERVAL_SEC 60
#define DEBUG_BT_BD_QUEUE_DEPTH 8
#define DEBUG_BT_MAX_RAW_LEN 128

typedef struct
{
    uint8_t data[DEBUG_BT_MAX_RAW_LEN];
    int len;
} debug_bt_bd_item_t;

static debug_bt_bd_item_t g_debug_bt_bd_queue[DEBUG_BT_BD_QUEUE_DEPTH];
static int g_debug_bt_bd_head = 0;
static int g_debug_bt_bd_tail = 0;
static int g_debug_bt_bd_count = 0;
static time_t g_debug_bt_last_bd_send_time = 0;

static int debug_bt_push_to_fifo(const uint8_t *data, int len)
{
    fifo_message_t msg;

    if (data == NULL || len <= 0 || len > (int)sizeof(msg.data))
        return -1;

    memset(&msg, 0, sizeof(msg));
    msg.type = BT_DATA;
    msg.len = len;
    memcpy(msg.data, data, len);

    pthread_mutex_lock(&fifo_lock);

    if (kfifo_left(&data_fifo) < sizeof(fifo_message_t))
    {
        pthread_mutex_unlock(&fifo_lock);
        printf("[DEBUG BT] fifo full, drop BT data len=%d\n", len);
        return -1;
    }

    kfifo_put(&data_fifo, (unsigned char *)&msg, sizeof(fifo_message_t));
    pthread_cond_signal(&fifo_not_empty);

    pthread_mutex_unlock(&fifo_lock);
    return 0;
}
static void notify_data_wakeup(const char *source)
{
    pthread_mutex_lock(&g_data_wakeup_mutex);
    g_data_wakeup_seq++;
    pthread_cond_signal(&g_data_wakeup_cond);
    pthread_mutex_unlock(&g_data_wakeup_mutex);

    printf("[POWER] Wakeup requested by %s data\n", source);
}
static int debug_is_hex_char(char c)
{
    return ((c >= '0' && c <= '9') ||
            (c >= 'a' && c <= 'f') ||
            (c >= 'A' && c <= 'F'));
}

static int debug_hex_char_to_val(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

/*
 * 手机发送纯16进制字符串：
 * 0102030a0b0c -> 01 02 03 0A 0B 0C
 * 支持中间带空格；不支持 0x 前缀。
 */
static int debug_parse_plain_hex(const char *str, uint8_t *out, int out_max)
{
    char clean[512];
    int clean_len = 0;

    if (str == NULL || out == NULL || out_max <= 0)
        return -1;

    for (int i = 0; str[i] != '\0' && clean_len < (int)sizeof(clean) - 1; i++)
    {
        char c = str[i];

        if (c == ' ' || c == '\r' || c == '\n' || c == '\t')
            continue;

        if (!debug_is_hex_char(c))
            return -1;

        clean[clean_len++] = c;
    }

    clean[clean_len] = '\0';

    if (clean_len == 0 || (clean_len % 2) != 0)
        return -1;

    int byte_len = clean_len / 2;
    if (byte_len > out_max)
        return -1;

    for (int i = 0; i < byte_len; i++)
    {
        int high = debug_hex_char_to_val(clean[i * 2]);
        int low = debug_hex_char_to_val(clean[i * 2 + 1]);

        if (high < 0 || low < 0)
            return -1;

        out[i] = (uint8_t)((high << 4) | low);
    }

    return byte_len;
}

static void debug_dump_hex(const char *tag, const uint8_t *data, int len)
{
    printf("%s len=%d: ", tag, len);
    for (int i = 0; i < len; i++)
    {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

static int debug_bt_bd_enqueue(const uint8_t *data, int len)
{
    if (data == NULL || len <= 0)
        return -1;

    /*
     * bd_send_packet() 会把原始数据转成十六进制字符串。
     * BD_MAX_RAW_LEN 是十六进制最大长度，所以原始数据不能太长。
     */
    if ((len * 2) > (BD_MAX_RAW_LEN - 2))
    {
        printf("[DEBUG BT] BD payload too long: raw_len=%d\n", len);
        return -2;
    }

    pthread_mutex_lock(&g_debug_bt_bd_mutex);

    if (g_debug_bt_bd_count >= DEBUG_BT_BD_QUEUE_DEPTH)
    {
        pthread_mutex_unlock(&g_debug_bt_bd_mutex);
        printf("[DEBUG BT] BD queue full\n");
        return -3;
    }

    memcpy(g_debug_bt_bd_queue[g_debug_bt_bd_tail].data, data, len);
    g_debug_bt_bd_queue[g_debug_bt_bd_tail].len = len;

    g_debug_bt_bd_tail = (g_debug_bt_bd_tail + 1) % DEBUG_BT_BD_QUEUE_DEPTH;
    g_debug_bt_bd_count++;

    printf("[DEBUG BT] BD queued, count=%d\n", g_debug_bt_bd_count);

    pthread_mutex_unlock(&g_debug_bt_bd_mutex);
    return 0;
}

/*
 * 每次只尝试发送队列头部一包。
 * 严格遵守 60 秒间隔。
 *
 * 返回：
 *  1 = 本次发送了一包
 *  0 = 没到60秒，或者队列为空
 * -1 = 发送失败
 */
static int debug_bt_bd_flush_once(void)
{
    uint8_t data[DEBUG_BT_MAX_RAW_LEN];
    int len = 0;
    time_t now = time(NULL);

    pthread_mutex_lock(&g_debug_bt_bd_mutex);

    if (g_debug_bt_bd_count <= 0)
    {
        pthread_mutex_unlock(&g_debug_bt_bd_mutex);
        return 0;
    }

    if (g_debug_bt_last_bd_send_time != 0 &&
        now - g_debug_bt_last_bd_send_time < DEBUG_BT_BD_INTERVAL_SEC)
    {
        long left = DEBUG_BT_BD_INTERVAL_SEC - (now - g_debug_bt_last_bd_send_time);
        pthread_mutex_unlock(&g_debug_bt_bd_mutex);
        printf("[DEBUG BT] BD interval not reached, %ld sec left\n", left);
        return 0;
    }

    len = g_debug_bt_bd_queue[g_debug_bt_bd_head].len;
    memcpy(data, g_debug_bt_bd_queue[g_debug_bt_bd_head].data, len);

    g_debug_bt_bd_head = (g_debug_bt_bd_head + 1) % DEBUG_BT_BD_QUEUE_DEPTH;
    g_debug_bt_bd_count--;

    /*
     * 无论本次 bd_send_packet 成功还是失败，都更新 last time。
     * 这样可以保证测试通道不会在北斗异常时疯狂重发。
     */
    g_debug_bt_last_bd_send_time = now;

    pthread_mutex_unlock(&g_debug_bt_bd_mutex);

    printf("[DEBUG BT] BD send now, len=%d\n", len);
    if (bd_send_packet(data, len) == 0)
    {
        printf("[DEBUG BT] BD send ok\n");
        return 1;
    }

    printf("[DEBUG BT] BD send failed\n");
    return -1;
}

static int debug_bt_send_4g_now(const uint8_t *data, int len)
{
    int connected = 0;

    if (data == NULL || len <= 0)
        return -1;

    pthread_mutex_lock(&g_debug_eg_mutex);
    connected = g_debug_eg_connected;
    pthread_mutex_unlock(&g_debug_eg_mutex);

    if (!connected)
    {
        printf("[DEBUG BT] 4G not connected, skip\n");
        return -2;
    }

    if (eg_send_data((unsigned char *)data, len) == 0)
    {
        printf("[DEBUG BT] 4G send ok, len=%d\n", len);
        return 0;
    }

    printf("[DEBUG BT] 4G send failed\n");
    return -1;
}

static int debug_bt_send_lora_by_rule(const uint8_t *data, int len)
{
    loRa_Para_t cfg;

    lora_cfg_get(&cfg);

    if (cfg.mesh_type != LORA_MESH_NODE)
    {
        printf("[DEBUG BT] LoRa skip: not node, mesh=0x%02X\n", cfg.mesh_type);
        return -2;
    }

    if (cfg.is_root == LORA_MESH_ROOT)
    {
        printf("[DEBUG BT] LoRa skip: root node\n");
        return -3;
    }

    if (len > cfg.payload_size)
    {
        printf("[DEBUG BT] LoRa payload too long: len=%d max=%u\n",
               len, cfg.payload_size);
        return -4;
    }

    int ret = Lora_send_packet(cfg.net_id, cfg.dev_id, (uint8_t *)data, len);
    if (ret == 0)
    {
        printf("[DEBUG BT] LoRa send ok, net=0x%02X dev=0x%02X len=%d\n",
               cfg.net_id, cfg.dev_id, len);
        return 0;
    }

    printf("[DEBUG BT] LoRa send failed, net=0x%02X dev=0x%02X len=%d\n",
           cfg.net_id, cfg.dev_id, len);

    return -1;
}

/*
 * 返回：
 * 1 = 已经作为 DEBUG 纯16进制数据处理
 * 0 = 不是 DEBUG 数据，继续走原来的 bt_handle_simple_command()
 */
static int debug_bt_handle_hex_payload_line(const char *line)
{
    uint8_t payload[DEBUG_BT_MAX_RAW_LEN];
    int payload_len;
    int bd_queue_ret;
    int bd_flush_ret;
    int eg_ret;
    int lora_ret;
    char ack[128];

    if (line == NULL || line[0] == '\0')
        return 0;

    /*
     * 保留原来的 BL_ 控制命令：
     * BL_0x00 / BL_0x01 / BL_0x02 / BL_0x03 / BL_0x04 / BL_0x0a
     * 以及你后面加的 LoRa 配置命令。
     */
    if (strncmp(line, "BL_", 3) == 0)
        return 0;

    payload_len = debug_parse_plain_hex(line, payload, sizeof(payload));
    if (payload_len <= 0)
    {
        printf("[DEBUG BT] invalid pure hex payload: %s\n", line);
        bt_send_text("BL_DEBUG_DATA_ERR\r\n");
        return 1;
    }

    debug_dump_hex("[DEBUG BT RX]", payload, payload_len);
    int fifo_ret = debug_bt_push_to_fifo(payload, payload_len);
    printf("[DEBUG BT] queued to main fifo ret=%d\n", fifo_ret);
    notify_data_wakeup("BT");
    /*
     * 北斗：只入队，由 debug_bt_bd_flush_once() 按60秒间隔发送。
     */
    bd_queue_ret = debug_bt_bd_enqueue(payload, payload_len);
    bd_flush_ret = debug_bt_bd_flush_once();

    /*
     * 4G：测试直发。
     * 注意：这是 DEBUG 测试通道，如果 main_send_thread 正在发送4G，
     * 可能会与原来的4G发送逻辑抢 AT 通道。
     */
    eg_ret = debug_bt_send_4g_now(payload, payload_len);

    /*
     * LoRa：遵守原来的根节点/非根节点规则。
     */
    lora_ret = debug_bt_send_lora_by_rule(payload, payload_len);

    snprintf(ack, sizeof(ack),
             "BL_DEBUG_ACK,BD_Q=%d,BD_FLUSH=%d,4G=%d,LORA=%d\r\n",
             bd_queue_ret, bd_flush_ret, eg_ret, lora_ret);
    bt_send_text(ack);

    return 1;
}

#endif
static int eg_reopen_init_connect(int *eg_connected)
{
    if (eg_connected)
        *eg_connected = 0;

    pthread_mutex_lock(&g_4g_mutex);
    g_4g_available = 0;
    pthread_mutex_unlock(&g_4g_mutex);

    if (uart_reopen_eg() != 0)
    {
        printf("[MAIN] EG UART reopen failed\n");
        return -1;
    }

    printf("[MAIN] Re-initializing 4G module after power cycle\n");
    if (eg_init() != 0)
    {
        printf("[MAIN] 4G init failed after UART reopen\n");
        return -1;
    }

    if (eg_connect() != 0)
    {
        printf("[MAIN] TCP connect failed after 4G re-init\n");
        return -1;
    }

    if (eg_connected)
        *eg_connected = 1;

    pthread_mutex_lock(&g_4g_mutex);
    g_4g_available = 1;
    pthread_mutex_unlock(&g_4g_mutex);

    printf("[MAIN] 4G ready after power cycle\n");
    return 0;
}
/**
 * @brief 从 RTC 设备读取当前时间
 * @param fd RTC 设备文件描述符
 * @param tm 输出时间结构体
 * @return 0=成功, -1=参数错误, -2=ioctl失败
 */
int rtc_get_time(int fd, struct rtc_time *tm)
{
    // 参数有效性检查
    if (fd < 0)
    {
        fprintf(stderr, "[RTC] Invalid file descriptor: %d\n", fd);
        return -1;
    }
    if (tm == NULL)
    {
        fprintf(stderr, "[RTC] NULL pointer for rtc_time\n");
        return -1;
    }

    // 清零结构体，避免未初始化数据
    memset(tm, 0, sizeof(struct rtc_time));

    // 通过 ioctl 读取 RTC 时间
    if (ioctl(fd, RTC_RD_TIME, tm) < 0)
    {
        fprintf(stderr, "[RTC] ioctl RTC_RD_TIME failed: %s\n", strerror(errno));
        return -2;
    }

    // 基本的时间有效性校验
    if (tm->tm_year < 0 || tm->tm_mon < 0 || tm->tm_mon > 11 ||
        tm->tm_mday < 1 || tm->tm_mday > 31 ||
        tm->tm_hour < 0 || tm->tm_hour > 23 ||
        tm->tm_min < 0 || tm->tm_min > 59 ||
        tm->tm_sec < 0 || tm->tm_sec > 59)
    {
        fprintf(stderr, "[RTC] Invalid time values read from RTC\n");
        return -2;
    }

    return 0;
}

void handle_signal(int sig)
{
    stop_flag = 1;
    pthread_cond_broadcast(&fifo_not_empty);
}

// 可被 stop_flag 中断的 sleep
int interruptible_sleep(int seconds)
{
    for (int i = 0; i < seconds && !stop_flag; i++)
    {
        sleep(1);
    }
    return stop_flag ? -1 : 0;
}

// 接收线程
void *receive_thread(void *arg)
{
    unsigned char buf[256];
    fd_set read_fds;
    struct timeval tv;
    int rs485_fd = get_fd(RS485_DEV);
    int rs232_fd = get_fd(RS232_DEV);
    int bd_fd = get_fd(BD_DEV);
    int max_fd;

    // 初始化串口状态
    serial_state_t rs485_state, rs232_state, bd_state, bt_state, eg_state;
    serial_state_init(&rs485_state, RS485_DATA, "RS485");
    serial_state_init(&rs232_state, RS232_DATA, "RS232");
    serial_state_init(&bd_state, BD_DATA, "BD");
    serial_state_init(&eg_state, EG_DATA, "EG");

    // 初始化帧处理器上下文
    frame_processor_ctx_t ctx = {
        .fifo = &data_fifo,
        .fifo_lock = &fifo_lock,
        .fifo_not_empty = &fifo_not_empty};

    while (!stop_flag)
    {
        int cur_rs485_fd = get_fd(RS485_DEV);
        int cur_rs232_fd = get_fd(RS232_DEV);
        int cur_bd_fd = get_fd(BD_DEV);
        // int cur_eg_fd = eg_fd;

        FD_ZERO(&read_fds);
        max_fd = -1;

        if (cur_rs485_fd >= 0)
        {
            FD_SET(cur_rs485_fd, &read_fds);
            if (cur_rs485_fd > max_fd)
                max_fd = cur_rs485_fd;
        }

        if (cur_rs232_fd >= 0)
        {
            FD_SET(cur_rs232_fd, &read_fds);
            if (cur_rs232_fd > max_fd)
                max_fd = cur_rs232_fd;
        }

        if (cur_bd_fd >= 0)
        {
            FD_SET(cur_bd_fd, &read_fds);
            if (cur_bd_fd > max_fd)
                max_fd = cur_bd_fd;
        }

        // if (cur_eg_fd >= 0)
        // {
        //     FD_SET(cur_eg_fd, &read_fds);
        //     if (cur_eg_fd > max_fd)
        //         max_fd = cur_eg_fd;
        // }

        if (max_fd < 0)
        {
            usleep(200 * 1000);
            continue;
        }

        tv.tv_sec = 0;
        tv.tv_usec = 200000;

        int ret = select(max_fd + 1, &read_fds, NULL, NULL, &tv);

        if (ret < 0)
        {
            if (errno == EINTR)
                continue;

            if (errno == EBADF)
            {
                printf("[RECV] select EBADF, fd changed, retry\n");
                usleep(200 * 1000);
                continue;
            }

            perror("select");
            usleep(200 * 1000);
            continue;
        }

        int rs485_read_len = 0;
        if (cur_rs485_fd >= 0 && FD_ISSET(cur_rs485_fd, &read_fds))
        {
            rs485_read_len = data_recv(buf, sizeof(buf) - 1, RS485_DEV);
            if (rs485_read_len > 0)
            {
                printf("[RS485 RX] len=%d: ", rs485_read_len);
                for (int i = 0; i < rs485_read_len; i++)
                    printf("%02X ", buf[i]);
                printf("\n");
            }
        }
        process_serial_data(&rs485_state, buf, rs485_read_len, (ret == 0), &ctx);

        int rs232_read_len = 0;
        if (cur_rs232_fd >= 0 && FD_ISSET(cur_rs232_fd, &read_fds))
        {
            rs232_read_len = data_recv(buf, sizeof(buf) - 1, RS232_DEV);
        }
        process_serial_data(&rs232_state, buf, rs232_read_len, (ret == 0), &ctx);

        int bd_read_len = 0;
        if (cur_bd_fd >= 0 && FD_ISSET(cur_bd_fd, &read_fds))
        {
            bd_read_len = data_recv(buf, sizeof(buf) - 1, BD_DEV);
        }
        process_serial_data(&bd_state, buf, bd_read_len, (ret == 0), &ctx);

        // int eg_read_len = 0;
        // if (cur_eg_fd >= 0 && FD_ISSET(cur_eg_fd, &read_fds))
        // {
        //     eg_read_len = data_recv(buf, sizeof(buf) - 1, EG_DEV);

        //     if (eg_read_len > 0)
        //     {
        //         buf[eg_read_len] = '\0';
        //         printf("[EG DEBUG] Received: [%s]\n", buf);

        //         char *urc = strstr((char *)buf, "+QIURC: \"pdpdeact\"");
        //         if (urc != NULL)
        //         {
        //             pthread_mutex_lock(&g_pdp_mutex);
        //             g_pdp_deact = 1;
        //             pthread_mutex_unlock(&g_pdp_mutex);
        //             printf("[EG] Received +QIURC: pdpdeact, will re-init PDP\n");
        //         }
        //     }
        // }
        // process_serial_data(&eg_state, buf, eg_read_len, (ret == 0), &ctx);
    }

    // 退出前处理残留帧
    flush_serial_state(&rs485_state, &ctx);
    flush_serial_state(&rs232_state, &ctx);
    flush_serial_state(&bd_state, &ctx);

    return NULL;
}
void *serial_send_thread(void *arg)
{
    const char *test = "Hello from rs232!\r\n";
    // 注意：不再从此线程发送 EG 命令，避免与 main_send_thread 的 eg_init() 冲突
    // while (!stop_flag)
    // {
    //     if (interruptible_sleep(SEND_INTERVAL) < 0)
    //         break;

    //     printf("[SEND] Sending command...\n");
    //     int resualt = data_send(test, strlen(test), RS232_DEV);
    //     int ret = data_send(CMD_STRING, strlen(CMD_STRING), RS485_DEV);
    //     int res_bd = data_send(BD_CARD, strlen(BD_CARD), BD_DEV);
    //     if (resualt < 0)
    //     {
    //         perror("rs232_send");
    //     }
    //     if (ret < 0)
    //     {
    //         perror("rs485_send");
    //     }
    //     if (res_bd < 0)
    //     {
    //         perror("bd_send");
    //     }
    // }
    return NULL;
}
void *read_rtc_thread(void *arg)
{
    int year, month, day, hour, min, sec;
    struct rtc_time get_time;
    while (!stop_flag)
    {
        if (interruptible_sleep(SEND_INTERVAL) < 0)
            break;
        if (rtc_get_time(rtc_fd, &get_time) == 0)
        {
            printf("Current RTC time: %04d-%02d-%02d %02d:%02d:%02d\n",
                   get_time.tm_year + 1900, get_time.tm_mon + 1, get_time.tm_mday,
                   get_time.tm_hour, get_time.tm_min, get_time.tm_sec);
        }
        else
        {
            perror("rtc_get_time");
        }
    }
    return NULL;
}
void *write_file_thread(void *arg)
{
    fifo_message_t msg;
    char log_line[1024];

    while (!stop_flag)
    {
        pthread_mutex_lock(&fifo_lock);
        // 等待直到至少有一个完整消息可读
        while (kfifo_len(&data_fifo) < sizeof(fifo_message_t) && !stop_flag)
        {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 0; // 200ms 超时
            ts.tv_nsec += 200 * 1000000;
            if (ts.tv_nsec >= 1000000000)
            {
                ts.tv_sec++;
                ts.tv_nsec -= 1000000000;
            }
            pthread_cond_timedwait(&fifo_not_empty, &fifo_lock, &ts);
        }
        if (stop_flag)
        {
            pthread_mutex_unlock(&fifo_lock);
            break;
        }
        // 读取消息
        unsigned int read_len = kfifo_get(&data_fifo, (unsigned char *)&msg, sizeof(msg));
        pthread_mutex_unlock(&fifo_lock);

        if (read_len != sizeof(msg))
            continue; // 数据不完整，跳过

        // 选择文件 - 根据数据类型选择对应的日志文件
        const char *path = NULL;
        switch (msg.type)
        {
        case RS485_DATA:
            path = RS485_LOG_PATH;
            break;
        case RS232_DATA:
            path = RS232_LOG_PATH;
            break;
        case BD_DATA:
            path = BD_LOG_PATH;
            break;
        case LORA_DATA:
            path = LORA_LOG_PATH;
            break;
#ifdef DEBUG
        case BT_DATA:
            path = BT_LOG_PATH;
            break;
#endif
        default:
            printf("[WARN] Unknown data type: %d, skipping\n", msg.type);
            continue;
        }
        // 构造时间戳行
        time_t now = time(NULL);
        struct tm *tm_info = localtime(&now);
        if (msg.frame_id == 0)
        {
            msg.frame_id = log_next_frame_id();
            msg.part_index = 0;
            msg.part_count = 1;
        }

        int pos = snprintf(log_line, sizeof(log_line),
                           "[%04d-%02d-%02d %02d:%02d:%02d] F=%u P=%u/%u ",
                           tm_info->tm_year + 1900,
                           tm_info->tm_mon + 1,
                           tm_info->tm_mday,
                           tm_info->tm_hour,
                           tm_info->tm_min,
                           tm_info->tm_sec,
                           msg.frame_id,
                           msg.part_index,
                           msg.part_count);
        for (int i = 0; i < msg.len && pos < (int)sizeof(log_line) - 5; i++)
        {
            pos += snprintf(log_line + pos, sizeof(log_line) - pos, "%02X ", msg.data[i]);
        }
        log_line[pos] = '\n';
        log_line[pos + 1] = '\0';

        FILE *fp = fopen(path, "a");
        if (!fp)
        {
            perror("fopen");
            continue;
        }

        if (fputs(log_line, fp) == EOF)
        {
            perror("fputs");
            fclose(fp);
            continue;
        }

        fflush(fp);
        fsync(fileno(fp));
        fclose(fp);

        if (msg.type == RS485_DATA || msg.type == BT_DATA)
        {
            notify_data_wakeup(msg.type == RS485_DATA ? "RS485" : "BT");
        }
    }
    return NULL;
}
// void *bd_send_thread(void *arg)
// {
//     char hex_buf[BD_MAX_RAW_LEN];
//     char msg[512];
//     const char *paths[] = {RS485_LOG_PATH, RS232_LOG_PATH};
//     off_t offsets[2] = {0, 0};
//     int hex_len, entry_count;
//     load_offsets(OFFSET_FILE_BD, offsets, 2);

//     // 等待日志文件被创建（最多等待30秒）
//     while (!stop_flag)
//     {
//         if (pack_data_from_files(paths, offsets, 2, BD_MAX_RAW_LEN,
//                                  hex_buf, &hex_len, &entry_count) == 0)
//         {

//             // 构造并发送报文
//             if (entry_count > 0)
//             {
//                 snprintf(msg, sizeof(msg), "$CCTCQ,4314513,2,1,2,,%s*", hex_buf);
//                 unsigned char cs = calc_checksum(msg);
//                 size_t len = strlen(msg);
//                 snprintf(msg + len, sizeof(msg) - len, "%02X\r\n", cs);
//                 data_send((unsigned char *)msg, strlen(msg), BD_DEV);
//                 printf("[SEND BDTX] %s", msg);
//                 save_offsets(OFFSET_FILE_BD, offsets, 2);
//             }
//             else
//             {
//                 // 文件存在但没有有效数据，不发送任何报文
//                 printf("[INFO] No valid data in log files, skipping BD transmission.\n");
//             }
//         }
//         else
//         {
//             printf("[ERROR] pack_data_from_files failed\n");
//         }
//         // 等待60秒
//         for (int i = 0; i < 60 && !stop_flag; i++)
//         {
//             sleep(1);
//         }
//     }
//     save_offsets(OFFSET_FILE_BD, offsets, 2);
//     return NULL;
// }
void *lora_transform_thread(void *arg)
{
    const char *paths[] = {RS485_LOG_PATH, RS232_LOG_PATH};
    off_t offsets[] = {0, 0};
    char hex_buf[LORA_MAX_HEX_LEN + 64];
    int hex_len, entry_count;
    while (!stop_flag)
    {
        loRa_Para_t cfg;
        lora_cfg_get(&cfg);

        if (cfg.mesh_type != LORA_MESH_NODE)
        {
            sleep(1);
            continue;
        }

        if (pack_data_from_files(paths, offsets, 2, LORA_MAX_HEX_LEN,
                                 hex_buf, &hex_len, &entry_count) == 0)
        {
            if (entry_count > 0)
            {
                uint8_t payload[256];
                int payload_len = 0;
                for (int i = 0; i < hex_len && payload_len < sizeof(payload); i += 2)
                {
                    unsigned int byte;
                    sscanf(hex_buf + i, "%2x", &byte);
                    payload[payload_len++] = (uint8_t)byte;
                }
                Lora_send_packet(cfg.net_id, cfg.dev_id, payload, payload_len);
                printf("[SEND LORA] net=0x%02X dev=0x%02X sent %d bytes\n",
                       cfg.net_id, cfg.dev_id, payload_len);

                save_offsets(OFFSET_FILE_LORA, offsets, 2);
            }
        }

        for (int i = 0; i < 30 && !stop_flag; i++)
            sleep(1);
    }

    save_offsets(OFFSET_FILE_LORA, offsets, 2);
    return NULL;
}
void *lora_receive_thread(void *arg)
{
    loRa_Para_t *lora_para = (loRa_Para_t *)arg;
    uint8_t rx_buf[256];
    uint8_t rx_len = 0;

    frame_processor_ctx_t ctx = {
        .fifo = &data_fifo,
        .fifo_lock = &fifo_lock,
        .fifo_not_empty = &fifo_not_empty};

    if (lora_para == NULL)
    {
        printf("[LORA RX] lora_para is NULL\n");
        return NULL;
    }

    RxInit();
    printf("[LORA RX] thread started, root=0x%02X mesh=0x%02X net=0x%02X dev=0x%02X\n",
           my_lora_config.is_root,
           my_lora_config.mesh_type,
           my_lora_config.net_id,
           my_lora_config.dev_id);
    while (!stop_flag)
    {
        rx_len = 0;
        int ret = Lora_recv_packet(rx_buf, &rx_len);
        if (ret != 0)
        {
            printf("[LORA RX DEBUG] ret=%d len=%u\n", ret, rx_len);
        }
        if (ret == 1)
        {
#ifdef DEBUG
            printf("[LORA RX RAW] ");
            for (int i = 0; i < rx_len; i++)
            {
                printf("%02X ", rx_buf[i]);
            }
            printf("\n");
#endif
            loRa_Para_t cfg;
            lora_cfg_get(&cfg);

            if (cfg.mesh_type == LORA_MESH_GATEWAY)
            {
                if (push_lora_to_fifo(&ctx, rx_buf, rx_len) == 0)
                {
                    printf("[LORA RX][GW] queued %u bytes\n", rx_len);
                }
            }
            else if (cfg.mesh_type == LORA_MESH_NODE)
            {
                if (cfg.is_root == LORA_MESH_ROOT)
                {
                    printf("[LORA RX][NODE][ROOT] recv %u bytes, no relay\n", rx_len);
                    continue;
                }

                printf("[LORA RX][NODE] relay %u bytes via LoRa\n", rx_len);
                Lora_send(rx_buf, rx_len);
            }
        }
        else if (ret < 0)
        {
            RxInit();
            usleep(100 * 1000);
        }
        else
        {
            usleep(20 * 1000);
        }
    }

    return NULL;
}

void *bt_comm_thread(void *arg)
{
    (void)arg;

    char recv_buf[256] = {0};
    int recv_len = 0;
    uint8_t heartbeat = 0;
    time_t last_heartbeat = 0;
    fd_set fds;
    struct timeval tv;

    printf("[BT] Communication thread started\n");

    bt_init();
    g_bt_start_time = time(NULL);
    bt_set_send_path(SEND_PATH_AUTO);

    while (!stop_flag)
    {
        time_t now = time(NULL);

        if (now != last_heartbeat)
        {
            bt_send_heartbeat_simple(heartbeat);
            heartbeat++;
            last_heartbeat = now;
        }
#ifdef DEBUG
        debug_bt_bd_flush_once();
#endif
        FD_ZERO(&fds);
        FD_SET(bt_fd, &fds);
        tv.tv_sec = 0;
        tv.tv_usec = 200000;

        int ret = select(bt_fd + 1, &fds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(bt_fd, &fds))
        {
            int n = read(bt_fd, recv_buf + recv_len, sizeof(recv_buf) - 1 - recv_len);
            if (n > 0)
            {
                // notify_data_wakeup("BT");

                if (!rf_power_is_on())
                {
                    rf_power_on();
                }
                recv_len += n;
                recv_buf[recv_len] = '\0';

                char *end = strstr(recv_buf, "\r\n");
                while (end != NULL)
                {
                    *end = '\0';
#ifdef DEBUG
                    if (!debug_bt_handle_hex_payload_line(recv_buf))
#endif
                    {
                        bt_handle_simple_command(recv_buf);
                    }

                    int consumed = (end - recv_buf) + 2;
                    int remaining = recv_len - consumed;

                    if (remaining > 0)
                    {
                        memmove(recv_buf, recv_buf + consumed, remaining);
                        recv_len = remaining;
                    }
                    else
                    {
                        recv_len = 0;
                    }

                    recv_buf[recv_len] = '\0';
                    end = strstr(recv_buf, "\r\n");
                }

                if (recv_len > 200)
                {
                    recv_len = 0;
                    recv_buf[0] = '\0';
                }
            }
        }
    }

    printf("[BT] Communication thread stopped\n");
    return NULL;
}
// 主发送线程（从文件读取数据并发送）
// void *eg_send_thread(void *arg)
// {
//     // 1. 初始化模块
//     if (eg_init() != 0)
//     {
//         printf("[EG] Initialization failed, thread exiting\n");
//         return NULL;
//     }

//     // 2. 建立 TCP 连接
//     if (eg_connect() != 0)
//     {
//         printf("[EG] TCP connect failed, thread exiting\n");
//         return NULL;
//     }
//     eg_state = EG_STATE_CONNECTED;

//     // 3. 准备文件读取和偏移量管理
//     const char *paths[] = {RS485_LOG_PATH, RS232_LOG_PATH};
//     off_t offsets[2] = {0, 0}; // ✅ 真正的 off_t 数组
//     int hex_len, entry_count;
//     char hex_buf[EG_MSG_LEN + 64]; // ✅ 定义 EG_MAX_HEX_LEN（例如 512）

//     // 加载持久化偏移量
//     load_offsets(OFFSET_FILE_4G, offsets, 2);

//     while (!stop_flag)
//     {
//         // 打包数据
//         if (pack_data_from_files(paths, offsets, 2, EG_MSG_LEN,
//                                  hex_buf, &hex_len, &entry_count) == 0)
//         {
//             if (entry_count > 0)
//             {
//                 // ✅ 将十六进制字符串转换为原始二进制数据
//                 unsigned char raw_data[256];
//                 int raw_len = hex_to_bytes(hex_buf, raw_data, sizeof(raw_data));
//                 if (raw_len > 0)
//                 {
//                     if (eg_send_data(raw_data, raw_len) == 0)
//                     {
//                         save_offsets(OFFSET_FILE_4G, offsets, 2);
//                         printf("[EG SEND] Sent %d bytes from logs.\n", raw_len);
//                     }
//                     else
//                     {
//                         // 发送失败，尝试重连
//                         printf("[EG] Send failed, attempting reconnect...\n");
//                         eg_send_cmd("AT+QICLOSE=0\r\n", "OK", 5);
//                         if (eg_connect() == 0)
//                         {
//                             // 重连成功，重试发送当前数据
//                             if (eg_send_data(raw_data, raw_len) == 0)
//                             {
//                                 save_offsets(OFFSET_FILE_4G, offsets, 2);
//                                 printf("[EG SEND] Sent %d bytes after reconnect.\n", raw_len);
//                             }
//                             else
//                             {
//                                 printf("[EG] Retry send still failed, skip this data\n");
//                                 // 注意：这里不更新偏移量，下次会重试同一数据
//                             }
//                         }
//                         else
//                         {
//                             printf("[EG] Reconnect failed, will retry later\n");
//                             // 等待一段时间后继续循环
//                             sleep(30);
//                         }
//                     }
//                 }
//                 else
//                 {
//                     printf("[EG] Failed to convert hex string to bytes\n");
//                 }
//             }
//             else
//             {
//                 printf("[INFO] No valid data for EG, sleeping...\n");
//             }
//         }
//         else
//         {
//             printf("[ERROR] pack_data_from_files failed\n");
//         }

//         // 等待一段时间再检查新数据（避免空转）
//         for (int i = 0; i < 10 && !stop_flag; i++)
//         {
//             sleep(1);
//         }
//     }

//     // 清理
//     eg_send_cmd("AT+QICLOSE=0\r\n", "OK", 5);
//     eg_send_cmd("AT+QIDEACT=1\r\n", "OK", 40);
//     save_offsets(OFFSET_FILE_4G, offsets, 2);
//     return NULL;
// }

void *eg_monitor_thread(void *arg)
{
    // 等待 4G 初始化完成
    while (!stop_flag)
    {
        pthread_mutex_lock(&g_eg_init_mutex);
        int init_done = g_eg_init_done;
        pthread_mutex_unlock(&g_eg_init_mutex);
        if (init_done)
            break;
        sleep(1);
    }

    // TCP 连接成功后，不再主动检测网络状态
    // 因为 eg_is_network_available() 会发送 AT 命令，与数据发送产生冲突
    // 网络状态由 main_send_thread 根据发送结果自行判断
    while (!stop_flag)
    {
        // 仅监控 PDP 去激活事件（由 receive_thread 通过 URC 检测）
        // 不再主动发送 AT 命令检测网络
        if (interruptible_sleep(30) < 0)
            break;
    }
    return NULL;
}
void *main_send_thread(void *arg)
{
    (void)arg;

    const char *paths[MAIN_PATH_COUNT] = {
        RS485_LOG_PATH,
        RS232_LOG_PATH,
        LORA_LOG_PATH,
        BT_LOG_PATH};

    off_t offsets[MAIN_PATH_COUNT] = {0};
    off_t pending_offsets[MAIN_PATH_COUNT];

    char hex_buf[EG_MAX_HEX_LEN + 1];
    char bd_hex_buf[BD_MAX_HEX_LEN + 1];

    unsigned char raw_data[(EG_MAX_HEX_LEN / 2) + 1];
    unsigned char bd_raw_data[BD_MAX_RAW_LEN];

    int hex_len = 0;
    int entry_count = 0;
    int raw_len = 0;

    int bd_hex_len = 0;
    int bd_entry_count = 0;
    int bd_raw_len = 0;

    time_t next_bd_send_time = 0;
    time_t next_4g_try_time = 0;

    int eg_initialized = 0;
    int eg_connected = 0;
    int eg_ready_fail_count = 0;
    int bd_fail_count = 0;
    int bd_fallback_active = 0;

    unsigned int eg_power_generation_seen = rf_power_get_generation();
    unsigned int wakeup_seq_seen = 0;

    main_send_state_t state = MAIN_ST_IDLE;
    send_path_t send_path = SEND_PATH_AUTO;

    load_offsets(OFFSET_FILE_MAIN, offsets, MAIN_PATH_COUNT);

    pthread_mutex_lock(&g_data_wakeup_mutex);
    wakeup_seq_seen = g_data_wakeup_seq;
    pthread_mutex_unlock(&g_data_wakeup_mutex);
    struct timespec ts;
    off_t bd_batch_end_offsets[MAIN_PATH_COUNT] = {0};
    int bd_batch_active = 0;
    while (!stop_flag)
    {
        switch (state)
        {
        case MAIN_ST_IDLE:
            usleep(300 * 1000);
            if (main_pack_pending(paths, offsets, pending_offsets,
                                  EG_MAX_HEX_LEN, hex_buf,
                                  raw_data, sizeof(raw_data),
                                  &raw_len, &entry_count) != 0)
            {
                printf("[MAIN] pack failed\n");
                main_wait_for_data_wakeup(&wakeup_seq_seen, MAIN_SEND_RETRY_WAIT_SEC);
                break;
            }

            if (entry_count == 0)
            {
                bd_fallback_active = 0;
                state = MAIN_ST_POWER_DOWN;
                break;
            }

            send_path = bt_get_send_path();

            if (bd_fallback_active ||
                send_path == SEND_PATH_BD_ONLY ||
                send_path == SEND_PATH_BD_FIRST)
            {
                state = MAIN_ST_TRY_BD;
            }
            else
            {
                state = MAIN_ST_TRY_4G;
            }
            break;

        case MAIN_ST_TRY_4G:
        {
            time_t now = time(NULL);

            if (send_path == SEND_PATH_BD_ONLY)
            {
                state = MAIN_ST_TRY_BD;
                break;
            }

            if (now < next_4g_try_time)
            {
                state = (send_path == SEND_PATH_4G_ONLY) ? MAIN_ST_IDLE : MAIN_ST_TRY_BD;
                break;
            }

            int eg_ready_ret = main_ensure_eg_ready(&eg_initialized,
                                                    &eg_connected,
                                                    &eg_power_generation_seen,
                                                    1);

            if (eg_ready_ret != 0)
            {
                eg_ready_fail_count++;
                next_4g_try_time = time(NULL) + MAIN_4G_COOLDOWN_SEC;
                eg_connected = 0;
                main_set_4g_available(0);

                printf("[MAIN] 4G init/connect failed %d/10, ret=%d\n",
                       eg_ready_fail_count, eg_ready_ret);

                if (eg_ready_fail_count >= 10)
                {
                    state = MAIN_ST_POWER_DOWN;
                    break;
                }
                if (send_path != SEND_PATH_4G_ONLY && !bd_batch_active)
                {
                    memcpy(bd_batch_end_offsets, pending_offsets, sizeof(bd_batch_end_offsets));
                    bd_batch_active = 1;
                }
                state = (send_path == SEND_PATH_4G_ONLY) ? MAIN_ST_IDLE : MAIN_ST_TRY_BD;
                break;
            }

            eg_ready_fail_count = 0;
            next_4g_try_time = 0;

            if (eg_send_data(raw_data, raw_len) == 0)
            {
                memcpy(offsets, pending_offsets, sizeof(offsets));

                for (int i = 0; i < MAIN_PATH_COUNT; i++)
                    compact_log_file(paths[i], &offsets[i], 10);

                save_offsets(OFFSET_FILE_MAIN, offsets, MAIN_PATH_COUNT);

                g_send_count_4g++;
                printf("[MAIN] Sent %d bytes via 4G\n", raw_len);
                state = MAIN_ST_IDLE;
            }
            else
            {
                eg_connected = 0;
                main_set_4g_available(0);
                printf("[MAIN] 4G send failed\n");
                if (send_path != SEND_PATH_4G_ONLY && !bd_batch_active)
                {
                    memcpy(bd_batch_end_offsets, pending_offsets, sizeof(bd_batch_end_offsets));
                    bd_batch_active = 1;
                }
                state = (send_path == SEND_PATH_4G_ONLY) ? MAIN_ST_IDLE : MAIN_ST_TRY_BD;
            }
            break;
        }

        case MAIN_ST_TRY_BD:
        {
            bd_fallback_active = 1;
            if (eg_power_is_on() || eg_connected || eg_initialized)
            {
                printf("[MAIN] BD mode active, powering down EG\n");

                main_eg_power_down(&eg_initialized,
                                   &eg_connected,
                                   &eg_power_generation_seen);
            }
            if (main_pack_pending_until(paths,
                                        offsets,
                                        bd_batch_end_offsets,
                                        pending_offsets,
                                        BD_MAX_HEX_LEN,
                                        bd_hex_buf,
                                        bd_raw_data,
                                        sizeof(bd_raw_data),
                                        &bd_raw_len,
                                        &bd_entry_count) != 0)
            {
                printf("[MAIN] BD pack failed\n");
                state = MAIN_ST_BD_WAIT;
                break;
            }

            if (bd_entry_count == 0)
            {
                bd_fallback_active = 0;
                bd_fail_count = 0;
                state = MAIN_ST_IDLE;
                break;
            }
            time_t now = monotonic_sec();
            if (next_bd_send_time != 0 &&
                now < next_bd_send_time)
            {
                state = MAIN_ST_BD_WAIT;
                break;
            }

            main_ensure_bd_ready();

            if (bd_send_packet(bd_raw_data, bd_raw_len) == 0)
            {
                memcpy(offsets, pending_offsets, sizeof(offsets));
                save_offsets(OFFSET_FILE_MAIN, offsets, MAIN_PATH_COUNT);

                next_bd_send_time = monotonic_sec() + BD_SEND_INTERVAL_SEC;
                bd_fail_count = 0;
                g_send_count_bd++;

                printf("[MAIN] Sent %d bytes via BD segment, offsets advanced\n",
                       bd_raw_len);

                if (bd_batch_active && offsets_reached(offsets, bd_batch_end_offsets))
                {
                    for (int i = 0; i < MAIN_PATH_COUNT; i++)
                        compact_log_file(paths[i], &offsets[i], 10);

                    save_offsets(OFFSET_FILE_MAIN, offsets, MAIN_PATH_COUNT);

                    bd_batch_active = 0;
                    bd_fallback_active = 0;
                    next_4g_try_time = 0;

                    printf("[MAIN] BD batch complete, return to 4G-first mode\n");

                    state = MAIN_ST_IDLE;
                }
                else
                {
                    state = MAIN_ST_BD_WAIT;
                }
            }
            else
            {
                bd_fail_count++;
                g_send_fail_count++;

                printf("[MAIN] BD segment failed %d/%d\n",
                       bd_fail_count, BD_FAIL_MAX);

                if (bd_fail_count >= BD_FAIL_MAX)
                {
                    state = MAIN_ST_POWER_DOWN;
                }
                else
                {
                    state = MAIN_ST_BD_WAIT;
                }
            }
            break;
        }

        case MAIN_ST_BD_WAIT:
        {
            time_t now = monotonic_sec();

            if (next_bd_send_time == 0 || now >= next_bd_send_time)
            {
                state = MAIN_ST_TRY_BD;
                break;
            }

            int wait_sec = (int)(next_bd_send_time - now);

            if (wait_sec > MAIN_SEND_RETRY_WAIT_SEC)
                wait_sec = MAIN_SEND_RETRY_WAIT_SEC;

            main_wait_for_data_wakeup(&wakeup_seq_seen, wait_sec);

            state = MAIN_ST_BD_WAIT;
            break;
        }

        case MAIN_ST_POWER_DOWN:
        {
            printf("[MAIN] Power down and wait for new data\n");

            main_all_power_down(&eg_initialized,
                                &eg_connected,
                                &eg_power_generation_seen);

            bd_fallback_active = 0;
            bd_fail_count = 0;
            eg_ready_fail_count = 0;
            next_4g_try_time = 0;

            main_wait_for_data_wakeup(&wakeup_seq_seen, MAIN_SEND_RETRY_WAIT_SEC);
            usleep(1000 * 1000);

            state = MAIN_ST_IDLE;
            break;
        }
        }
    }

    main_all_power_down(&eg_initialized,
                        &eg_connected,
                        &eg_power_generation_seen);

    save_offsets(OFFSET_FILE_MAIN, offsets, MAIN_PATH_COUNT);
    return NULL;
}

void *watchdog_feed_thread(void *arg)
{
    // while (!stop_flag)
    // {
    //     // if (watchdog_fd != -1)
    //     // {
    //     //     if (write(watchdog_fd, "\0", 1) != 1)
    //     //     {
    //     //         perror("watchdog write");
    //     //     }
    //     // }
    //     // if (interruptible_sleep(5) < 0)
    //     //     break;
    // }
    return NULL;
}