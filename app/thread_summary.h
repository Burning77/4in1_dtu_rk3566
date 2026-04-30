#ifndef __THREAD_SUMMARY_H__
#define __THREAD_SUMMARY_H__
#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/select.h>
#include <errno.h>
#include "../inc/gpio.h"
#include "../inc/usart.h"
#include <linux/rtc.h>
#include <fcntl.h>
#include "../inc/universal.h"
#include "../inc/kfifo.h"
#include <time.h>
#include "llcc68.h"
#include "../inc/bluetooth.h"

#define OFFSET_FILE_MAIN   "/home/cat/send_offset.dat"
#define OFFSET_FILE_LORA "/home/cat/send_offset_lora.dat"
#define OFFSET_FILE_LORA_RECV "/home/cat/send_offset_lora_recv.dat"

#define LORA_MAX_HEX_LEN 256

void handle_signal(int sig);
// 可被 stop_flag 中断的 sleep，返回 0 表示正常完成，-1 表示被中断
int interruptible_sleep(int seconds);
void *receive_thread(void *arg);
void *serial_send_thread(void *arg);
void *read_rtc_thread(void *arg);
void *write_file_thread(void *arg);
void *main_send_thread(void *arg);
void *lora_transform_thread(void *arg);
void *eg_monitor_thread(void *arg);
void *watchdog_feed_thread(void *arg);
void *lora_receive_thread(void *arg);
#endif