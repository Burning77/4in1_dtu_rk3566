#define _GNU_SOURCE
#include "../inc/usart.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include "string.h"
#include "../inc/gpio.h"
static pthread_mutex_t dir_mutex = PTHREAD_MUTEX_INITIALIZER;
extern struct gpiod_line *line_rs485;
int rs485_fd = -1;
int rs232_fd = -1;
int bd_fd = -1;
int bt_fd = -1;
int eg_fd = -1;
int uart_init(const char *dev, speed_t baud)
{
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror("open uart");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;                             // 8位数据
    options.c_cflag &= ~PARENB;                         // 无校验
    options.c_cflag &= ~CSTOPB;                         // 1停止位
    options.c_cflag &= ~CRTSCTS;                        // 无硬件流控
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始模式
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // 无软件流控
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~OPOST;

    // 阻塞模式：至少读一个字节才返回
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;

    tcsetattr(fd, TCSANOW, &options);

    // 确保阻塞模式（清除可能的非阻塞标志）
    fcntl(fd, F_SETFL, 0);
    if (!strcmp(dev, RS485_DEV))
    {
        rs485_fd = fd;
    }
    if (!strcmp(dev, RS232_DEV))
    {
        rs232_fd = fd;
    }
    if (!strcmp(dev, BD_DEV))
    {
        bd_fd = fd;
    }
    return fd;
}
int uart_init_gather()
{
    // 初始化rs485
    rs485_fd = uart_init(RS485_DEV, RS485_BAUD);
    if (rs485_fd < 0)
    {
        return 1;
    }
    // 初始化rs232
    rs232_fd = uart_init(RS232_DEV, RS232_BAUD);
    if (rs232_fd < 0)
    {
        return 1;
    }
    bd_fd = uart_init(BD_DEV, BD_BAUD);
    if (bd_fd < 0)
    {
        return 1;
    }
    bt_fd = uart_init(BT_DEV, BT_BAUD);
    if (bt_fd < 0)
    {
        return 1;
    }
    eg_fd = uart_init(EG_DEV, EG_BAUD);
    if (eg_fd < 0)
        return 1;
    return 0;
}

void rs485_set_tx_mode(void)
{
    pthread_mutex_lock(&dir_mutex);
    gpio_set_value(1, line_rs485);
    pthread_mutex_unlock(&dir_mutex);
}

void rs485_set_rx_mode(void)
{
    pthread_mutex_lock(&dir_mutex);
    gpio_set_value(0, line_rs485);
    pthread_mutex_unlock(&dir_mutex);
}

int data_send(const void *buf, size_t len, const char *dev)
{
    if (!strcmp(dev, RS485_DEV))
    {
        rs485_set_tx_mode();
        usleep(10); // 等待收发器切换
        int ret = write(rs485_fd, buf, len);
        tcdrain(rs485_fd); // 等待数据发送完成
        rs485_set_rx_mode();
        return ret;
    }
    else if (!strcmp(dev, RS232_DEV))
    {
        int ret = write(rs232_fd, buf, len);
        tcdrain(rs232_fd);
        return ret;
    }
    else if (!strcmp(dev, BD_DEV))
    {
        int ret = write(bd_fd, buf, len);
        tcdrain(bd_fd);
        return ret;
    }
    else if (!strcmp(dev, BT_DEV))
    {
        int ret = write(bt_fd, buf, len);
        tcdrain(bt_fd);
        return ret;
    }
    else if (!strcmp(dev, EG_DEV))
    {
        int ret = write(eg_fd, buf, len);
        tcdrain(eg_fd);
        return ret;
    }
    return -1;
}
int data_recv(void *buf, size_t len, const char *dev)
{
    if (!strcmp(dev, RS485_DEV))
    {
        return read(rs485_fd, buf, len);
    }
    else if (!strcmp(dev, RS232_DEV))
    {
        return read(rs232_fd, buf, len);
    }
    else if (!strcmp(dev, BD_DEV))
    {
        return read(bd_fd, buf, len);
    }
    else if (!strcmp(dev, BT_DEV))
    {
        return read(bt_fd, buf, len);
    }
    else if (!strcmp(dev, EG_DEV))
    {
        return read(eg_fd, buf, len);
    }
    return -1;
}
int get_fd(const char *dev)
{
    if (!strcmp(dev, RS232_DEV))
    {
        return rs232_fd;
    }
    else if (!strcmp(dev, RS485_DEV))
    {
        return rs485_fd;
    }
    else if (!strcmp(dev, BD_DEV))
    {
        return bd_fd;
    }
    else if (!strcmp(dev, BT_DEV))
    {
        return bt_fd;
    }
    else if (!strcmp(dev, EG_DEV))
    {
        return eg_fd;
    }
    return -1;
}
void uart_close(int fd)
{
    if (fd > 0)
        close(fd);
    fd = -1;
}
