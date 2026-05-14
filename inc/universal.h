#ifndef DATA_SAVE_H
#define DATA_SAVE_H
#include <stdint.h>
#include <sys/time.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>

#define RING_BUFFER_SIZE 1024
#define MAX_LOG_LEN 256
#define CMD_STRING "AT\r\n"
#define BD_CARD "$CCICR,0,00*68\r\n"
#define SEND_INTERVAL 2 // 秒
#define RS485_LOG_PATH "/home/cat/rs485_data.log"
#define RS232_LOG_PATH "/home/cat/rs232_data.log"
#define BD_LOG_PATH "/home/cat/bd_data.log"
#define LORA_LOG_PATH "/home/cat/lora_data.log"
#define LORA_CFG_PATH "/home/cat/lora_cfg.ini"
#define LORA_CFG_TMP_PATH "/home/cat/lora_cfg.ini.tmp"
#define BT_LOG_PATH "/home/cat/bt_data.log"
#define RS485_DATA 0
#define RS232_DATA 1
#define BD_DATA 2
#define BT_DATA 3
#define EG_DATA 4
#define LORA_DATA 5
#define FIFO_SIZE (16 * 1024) // 必须是2的幂次方

#define EG_MSG_LEN 2048
#define EG_MAX_HEX_LEN (EG_MSG_LEN * 2 + 256)
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define LORA_MESH_GATEWAY 0x01
#define LORA_MESH_NODE 0x00
#define LORA_MESH_ROOT 0xff
#define LORA_MESH_NOTROOT 0x00
#define DEBUG
typedef struct
{
    int type;                  // RS485_DATA 或 RS232_DATA
    unsigned int len;          // 数据长度
    unsigned char data[256];   // 原始数据
    unsigned int frame_id;     // 原始帧ID
    unsigned short part_index; // 当前分片序号
    unsigned short part_count; // 总分片数
} fifo_message_t;

// 前向声明
struct kfifo;

// 定义串口状态结构
typedef struct
{
    uint8_t frame_buf[1024];  // 帧缓冲区
    int frame_len;            // 当前帧长度
    struct timeval last_recv; // 上次接收时间
    int data_type;            // 数据类型 (RS485_DATA 或 RS232_DATA)
    const char *tag;          // 调试标签，如 "RS485" 或 "RS232"
} serial_state_t;

// 帧处理器上下文
typedef struct
{
    struct kfifo *fifo;
    pthread_mutex_t *fifo_lock;
    pthread_cond_t *fifo_not_empty;
} frame_processor_ctx_t;
int hex_to_bytes(const char *hex_str, unsigned char *out_bytes, int max_len);
unsigned char calc_checksum(const char *s);
int parse_log_line(const char *line, unsigned char *out_data, int max_len);

// 初始化串口状态
void serial_state_init(serial_state_t *state, int data_type, const char *tag);

// 处理串口数据
void process_serial_data(serial_state_t *state,
                         unsigned char *read_buf, int read_len,
                         int is_timeout_triggered,
                         frame_processor_ctx_t *ctx);

// 强制提交缓冲区中的残留帧
void flush_serial_state(serial_state_t *state, frame_processor_ctx_t *ctx);
int pack_data_from_files(const char **paths, off_t *offsets, int file_count,
                         int max_hex_len, char *out_hex_buf,
                         int *out_hex_len, int *out_entry_count);
int load_offsets(const char *path, off_t *offsets, int count);
int save_offsets(const char *path, const off_t *offsets, int count);
uint16_t crc16(const uint8_t *data, uint16_t len);
int push_lora_to_fifo(frame_processor_ctx_t *ctx, const uint8_t *data, uint8_t len);
uint32_t log_next_frame_id(void);
int compact_log_file(const char *path, off_t *offset, int keep_frames);
time_t monotonic_sec(void);
#endif