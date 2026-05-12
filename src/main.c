#include "../app/thread_summary.h"
#include "../inc/watch_dog.h"
#include "../inc/bluetooth.h"
#include <stdatomic.h>
#include "../inc/power.h"
#define THREAD_COUNT 9
typedef void *(*thread_func_t)(void *);
volatile sig_atomic_t stop_flag = 0;
struct kfifo data_fifo;
static unsigned char fifo_buffer[FIFO_SIZE];
// RTC 唤醒间隔（秒）
static int rtc_wakeup_interval = 10;
int rtc_fd;
extern int rs485_fd;
extern int rs232_fd;
extern int bd_fd;
extern int bt_fd;
extern int eg_fd;
extern int watchdog_fd;
extern int spi_fd;
loRa_Para_t my_lora_config = {
    .is_root = LORA_MESH_ROOT,      // 是根节点;非根节点 0x00
    .mesh_type = LORA_MESH_GATEWAY, // 网关；节点：0x00
    .net_id = 0x01,
    .dev_id = 0x0a,
    .rf_freq = 433000000, // 中心频率：433 MHz
    .tx_power = 14,       // 发射功率：14 dBm
    .lora_sf = 7,         // 扩频因子：SF7 (需与 llcc68.h 中定义匹配，如 LORA_SF7)
    .band_width = 0x04,   // 带宽：125 kHz (对应 LORA_BW_125，值0x04)
    .code_rate = 0x01,    // 编码率：4/5 (对应 LORA_CR_4_5，值0x01)
    .payload_size = 64    // 预期接收负载的最大长度
};
// int check_hardware_flow_control(int fd)
// {
//     struct termios options;
//     if (tcgetattr(fd, &options) != 0)
//     {
//         perror("tcgetattr");
//         return -1;
//     }

//     if (options.c_cflag & CRTSCTS)
//     {
//         printf("Hardware flow control (CRTSCTS) is ENABLED.\n");
//         return 1;
//     }
//     else
//     {
//         printf("Hardware flow control (CRTSCTS) is DISABLED.\n");
//         return 0;
//     }
// }
int main(int argc, char *argv[])
{
    // 关闭 stdout/stderr 缓冲，printf/perror 实时写入 journal
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    // 注册信号处理
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    kfifo_init(&data_fifo, fifo_buffer, FIFO_SIZE);
    // 初始化GPIO
    if (gpio_init() < 0)
    {
        return 1;
    }
    rf_power_on();

    // 等待 4G 模块 USB 枚举完成（EG800K 上电后需要较长时间）
    printf("Waiting for 4G module USB enumeration...\n");
    int uart_retry = 0;
    const int max_uart_retry = 10;
    while (uart_init_gather() != 0)
    {
        uart_retry++;
        if (uart_retry >= max_uart_retry)
        {
            fprintf(stderr, "UART init failed after %d retries, exiting\n", max_uart_retry);
            gpio_cleanup();
            return 1;
        }
        printf("UART init failed, retry %d/%d in 2 seconds...\n", uart_retry, max_uart_retry);
        sleep(2);
    }
    printf("All UARTs initialized successfully.\n");
    // // 发送读卡号指令
    // char cmd[] = "$CCICR,0,00 * 68\r\n";
    // write(bd_fd, cmd, strlen(cmd));

    // // 等待并读取回复
    // sleep(2);
    // char buf[256];
    // int n = read(bd_fd, buf, sizeof(buf));
    // printf("Received %d bytes\n", n);
    // for (int i = 0; i < n; i++) {
    //     printf("%c", buf[i]);
    // }
    // printf("flow check\n");
    // check_hardware_flow_control(eg_fd);
    // printf("check over\r\n");
    // eg_init();
    // char cmd[] = "AT+CEREG?\r\n";
    // printf("write cmd to eg\r\n");
    // write(eg_fd, cmd, strlen(cmd));
    // printf("write cmd to eg over\r\n");
    // sleep(2);
    // char buf[256];
    // printf("read data from eg\r\n");
    // int n = read(eg_fd, buf, sizeof(buf));
    // printf("Received %s bytes\n", buf);
    struct rtc_time set_time = {
        .tm_year = 2026 - 1900,
        .tm_mon = 4 - 1, // 4月
        .tm_mday = 22,
        .tm_hour = 17,
        .tm_min = 0,
        .tm_sec = 0,
        .tm_isdst = 0};

    // 初始化 RX8010SJ RTC (通过 I2C)
    // printf("Initializing RX8010SJ RTC...\n");
    // if (rx8010_init() == 0)
    // {
    //     rx8010_set_time(&set_time);
    //     printf("RX8010SJ RTC initialized successfully.\n");
    // }
    // else
    // {
    //     printf("Warning: RX8010SJ RTC init failed, using system RTC\n");
    //     rtc_set_time(&set_time);
    // }

    // 保留 /dev/rtc0 用于兼容
    rtc_fd = open("/dev/rtc0", O_RDONLY);
    printf("Loading LoRa config from file...\n");
    lora_cfg_load_persist(&my_lora_config);
    printf("Initializing LoRa module...\n");
    if (!Lora_init(&my_lora_config))
    { // 将配置参数传递给驱动
        fprintf(stderr, "LoRa module initialization failed!\n");
        return -1;
    }
    printf("LoRa module initialized successfully.\n");
    // init_watchdog();
    // 创建线程
    thread_func_t thread_funcs[THREAD_COUNT] = {
        receive_thread,
        read_rtc_thread,
        write_file_thread,
        main_send_thread,
        lora_transform_thread,
        eg_monitor_thread,
        watchdog_feed_thread,
        bt_comm_thread,
        lora_receive_thread,
    };
    void *thread_args[THREAD_COUNT] = {
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &my_lora_config};
    pthread_t tids[THREAD_COUNT];
    int created_count = 0;
    int ret = 1; // 默认失败
    for (int i = 0; i < THREAD_COUNT; i++)
    {
        if (pthread_create(&tids[i], NULL, thread_funcs[i], thread_args[i]) != 0)
        {
            perror("pthread_create failed");
            atomic_store(&stop_flag, 1);
            goto cleanup;
        }
        created_count++;
    }
    for (int i = 0; i < THREAD_COUNT; i++)
    {
        pthread_join(tids[i], NULL);
    }
    ret = 0;

cleanup:
    // 如果是异常退出，等待已创建的线程
    if (ret != 0)
    {
        const char *thread_names[THREAD_COUNT] = {
            "receive_thread",
            "read_rtc_thread",
            "write_file_thread",
            "main_send_thread",
            "lora_transform_thread",
            "eg_monitor_thread",
            "watchdog_feed_thread",
            "bt_comm_thread",
            "lora_receive_thread",
        };

        for (int i = 0; i < created_count; i++)
        {
            printf("[MAIN] joining %s...\n", thread_names[i]);
            pthread_join(tids[i], NULL);
            printf("[MAIN] joined %s\n", thread_names[i]);
        }
    }
    rf_power_off();
    uart_close(rs232_fd);
    uart_close(rs485_fd);
    uart_close(bd_fd);
    uart_close(eg_fd);
    uart_close(bt_fd);
    gpio_cleanup();
    close(rtc_fd);
    close(spi_fd);
    if (watchdog_fd >= 0)
    {
        cleanup_watchdog();
    }
    return ret;
}