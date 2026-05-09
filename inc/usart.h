#ifndef USART_H
#define USART_H

#include <termios.h> // 包含 speed_t 类型
#include <stddef.h>

#define RS485_DEV "/dev/ttyS1"
#define RS485_BAUD B9600
#define RS232_DEV "/dev/ttyS3"
#define RS232_BAUD B115200
#define BD_DEV "/dev/ttyS7"
#define BD_BAUD B115200
#define BT_DEV "/dev/ttyS4"
#define BT_BAUD B115200
#define EG_DEV "/dev/ttyUSB2"
#define EG_BAUD B115200
// 初始化串口，设置为阻塞模式
// 返回文件描述符，失败返回-1
int uart_init(const char *dev, speed_t baud);
int uart_init_gather();
// 切换到发送模式（高电平）
void rs485_set_tx_mode(void);

// 切换到接收模式（低电平）
void rs485_set_rx_mode(void);

int data_send(const void *buf, size_t len, const char *dev);
int data_recv(void *buf, size_t len, const char *dev);
int get_fd(const char *dev);
void uart_close(int fd);
void uart_close_eg(void);
int uart_reopen_eg(void);
#endif