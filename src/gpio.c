#include <stdio.h>
#include <gpiod.h>
#include "../inc/gpio.h"
#define RS485_GPIO_CHIP "/dev/gpiochip2"
#define RS485_GPIO_LINE 13
#define BD_EN_CHIP "/dev/gpiochip2"
#define BD_EN_LINE 22
#define BD_POW_CHIP "/dev/gpiochip3"
#define BD_POW_LINE 17
#define BT_POW_CHIP "/dev/gpiochip0"
#define BT_POW_LINE 29
#define EG_POW_CHIP "/dev/gpiochip3"
#define EG_POW_LINE 19
#define EG_BOOT_CHIP "/dev/gpiochip3"
#define EG_BOOT_LINE 3
#define EG_STATE_CHIP "/dev/gpiochip3"
#define EG_STATE_LINE 1
#define EG_SLEEP_CHIP "/dev/gpiochip3"
#define EG_SLEEP_LINE 27
#define BT_IDLE_CHIP "/dev/gpiochip4"
#define BT_IDLE_LINE 12
#define BT_STATUS_CHIP "/dev/gpiochip4"
#define BT_STATUS_LINE 10
#define USB_POW_CHIP "/dev/gpiochip3"
#define USB_POW_LINE 2

struct gpiod_chip *chip_rs485 = NULL;
struct gpiod_line *line_rs485 = NULL;
struct gpiod_chip *chip_bd_en = NULL;
struct gpiod_line *line_bd_en = NULL;
struct gpiod_chip *chip_bd_pow = NULL;
struct gpiod_line *line_bd_pow = NULL;
struct gpiod_chip *chip_bt_pow = NULL;
struct gpiod_line *line_bt_pow = NULL;
struct gpiod_chip *chip_4g_pow = NULL;
struct gpiod_line *line_4g_pow = NULL;
struct gpiod_chip *chip_4g_boot = NULL;
struct gpiod_line *line_4g_boot = NULL;
struct gpiod_chip *chip_4g_state = NULL;
struct gpiod_line *line_4g_state = NULL;
struct gpiod_chip *chip_4g_sleep = NULL;
struct gpiod_line *line_4g_sleep = NULL;
struct gpiod_chip *chip_bt_idle = NULL;
struct gpiod_line *line_bt_idle = NULL;
struct gpiod_chip *chip_bt_status = NULL;
struct gpiod_line *line_bt_status = NULL;
struct gpiod_chip *chip_usb_pow = NULL;
struct gpiod_line *line_usb_pow = NULL;
int gpio_init()
{
    chip_rs485 = gpiod_chip_open(RS485_GPIO_CHIP);
    if (!chip_rs485)
    {
        perror("gpiod_chip_open485");
        return -1;
    }
    line_rs485 = gpiod_chip_get_line(chip_rs485, RS485_GPIO_LINE);
    if (!line_rs485)
    {
        perror("gpiod_chip_get_line 485");
        gpiod_chip_close(chip_rs485);
        return -1;
    }
    // 请求为输出，初始值为0（接收模式）
    if (gpiod_line_request_output(line_rs485, "rs485", 0) < 0)
    {
        perror("gpiod_line_request_output485");
        gpiod_chip_close(chip_rs485);
        return -1;
    }

    chip_bd_en = gpiod_chip_open(BD_EN_CHIP);
    if (!chip_bd_en)
    {
        perror("gpiod_chip_open bd_en");
        return -1;
    }
    line_bd_en = gpiod_chip_get_line(chip_bd_en, BD_EN_LINE);
    if (!line_bd_en)
    {
        perror("gpiod_chip_get_line bd_en");
        gpiod_chip_close(chip_bd_en);
        return -1;
    }
    // 请求为输出，初始值为0
    if (gpiod_line_request_output(line_bd_en, "bd_en", 0) < 0)
    {
        perror("gpiod_line_request_output bd_en");
        gpiod_chip_close(chip_bd_en);
        return -1;
    }

    // 北斗电源控制（可选，如果被占用则跳过）
    chip_bd_pow = gpiod_chip_open(BD_POW_CHIP);
    if (chip_bd_pow)
    {
        line_bd_pow = gpiod_chip_get_line(chip_bd_pow, BD_POW_LINE);
        if (line_bd_pow)
        {
            // 请求为输出，初始值为0（关闭状态，按需开启）
            if (gpiod_line_request_output(line_bd_pow, "bd_pow_en", 0) < 0)
            {
                // 如果被占用，打印警告但继续运行
                perror("gpiod_line_request_output bd_pow_en (optional, continuing)");
                line_bd_pow = NULL;
                gpiod_chip_close(chip_bd_pow);
                chip_bd_pow = NULL;
            }
        }
        else
        {
            gpiod_chip_close(chip_bd_pow);
            chip_bd_pow = NULL;
        }
    }
    // 北斗电源控制是可选的，即使失败也继续运行

    chip_bt_pow = gpiod_chip_open(BT_POW_CHIP);
    line_bt_pow = gpiod_chip_get_line(chip_bt_pow, BT_POW_LINE);
    // 请求为输出，初始值为1
    if (gpiod_line_request_output(line_bt_pow, "bt_pow_en", 1) < 0)
    {
        perror("gpiod_line_request_output bt_pow_en");
        gpiod_chip_close(chip_bt_pow);
        return -1;
    }

    chip_4g_pow = gpiod_chip_open(EG_POW_CHIP);
    line_4g_pow = gpiod_chip_get_line(chip_4g_pow, EG_POW_LINE);
    // 请求为输出，初始值为1
    if (gpiod_line_request_output(line_4g_pow, "4g_pow_en", 1) < 0)
    {
        perror("gpiod_line_request_output 4g_pow_en");
        gpiod_chip_close(chip_4g_pow);
        return -1;
    }

    chip_bt_idle = gpiod_chip_open(BT_IDLE_CHIP);
    line_bt_idle = gpiod_chip_get_line(chip_bt_idle, BT_IDLE_LINE);
    // 请求为输出，初始值为1
    if (gpiod_line_request_output(line_bt_idle, "bt_idle", 0) < 0)
    {
        perror("gpiod_line_request_output bt_idle");
        gpiod_chip_close(chip_bt_idle);
        return -1;
    }

    chip_bt_status = gpiod_chip_open(BT_STATUS_CHIP);
    line_bt_status = gpiod_chip_get_line(chip_bt_status, BT_STATUS_LINE);
    // 请求为输入
    if (gpiod_line_request_input(line_bt_status, "bt_status") < 0)
    {
        perror("gpiod_line_request_input bt_status");
        gpiod_chip_close(chip_bt_status);
        return -1;
    }

    chip_4g_boot = gpiod_chip_open(EG_BOOT_CHIP);
    line_4g_boot = gpiod_chip_get_line(chip_4g_boot, EG_BOOT_LINE);
    // 请求为输出，默认初始为1
    if (gpiod_line_request_output(line_4g_boot, "4g_boot", 1) < 0)
    {
        perror("gpiod_line_request_output 4g_boot");
        gpiod_chip_close(chip_4g_boot);
        return -1;
    }

    chip_4g_state = gpiod_chip_open(EG_STATE_CHIP);
    line_4g_state = gpiod_chip_get_line(chip_4g_state, EG_STATE_LINE);
    // 请求为输入
    if (gpiod_line_request_input(line_4g_state, "4g_state") < 0)
    {
        perror("gpiod_line_request_input 4g_state");
        gpiod_chip_close(chip_4g_state);
        return -1;
    }
    chip_4g_sleep = gpiod_chip_open(EG_SLEEP_CHIP);
    line_4g_sleep = gpiod_chip_get_line(chip_4g_sleep, EG_SLEEP_LINE);
    // 请求为输出，默认初始为1
    if (gpiod_line_request_output(line_4g_sleep, "4g_sleep", 0) < 0)
    {
        perror("gpiod_line_request_output 4g_sleep");
        gpiod_chip_close(chip_4g_sleep);
        return -1;
    }
    chip_usb_pow = gpiod_chip_open(USB_POW_CHIP);
    line_usb_pow = gpiod_chip_get_line(chip_usb_pow, USB_POW_LINE);
    // 请求为输出，默认初始为1
    if (gpiod_line_request_output(line_usb_pow, "4g_usb_pow", 1) < 0)
    {
        perror("gpiod_line_request_output usb_pow");
        gpiod_chip_close(chip_usb_pow);
        return -1;
    }
    return 0;
}

void gpio_set_value(int value, struct gpiod_line *line)
{
    if (line)
        gpiod_line_set_value(line, value);
}

int gpio_get_value(struct gpiod_line *line)
{
    if (line)
        return gpiod_line_get_value(line);
    return -1;
}

void gpio_cleanup(void)
{
    if (chip_rs485)
        gpiod_chip_close(chip_rs485);
    if (chip_bd_en)
        gpiod_chip_close(chip_bd_en);
}