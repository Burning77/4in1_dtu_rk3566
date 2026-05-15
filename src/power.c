#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <gpiod.h>
#include "../inc/power.h"
#include "../inc/gpio.h"
#include "../inc/bd3.h"
extern struct gpiod_line *line_bd_en;
extern struct gpiod_line *line_bd_pow;
extern struct gpiod_line *line_bt_pow;
extern struct gpiod_line *line_4g_pow;
extern struct gpiod_line *line_4g_boot;
extern struct gpiod_line *line_usb_pow;
extern pthread_mutex_t g_4g_mutex;
extern volatile int g_4g_available;
extern int eg_fd;
static pthread_mutex_t g_rf_power_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_rf_power_enabled = 0;

static int g_bd_power_enabled = 0;
static int g_eg_power_enabled = 0;
static unsigned int g_rf_power_generation = 0;

void bd_power_on(void)
{
    pthread_mutex_lock(&g_rf_power_mutex);

    if (!g_bd_power_enabled)
    {
        printf("[POWER] BD power on\n");
        gpio_set_value(1, line_bd_pow);
        gpio_set_value(1, line_bd_en);
        g_bd_power_enabled = 1;
        g_rf_power_generation++;
    }

    pthread_mutex_unlock(&g_rf_power_mutex);
}

void bd_power_off(void)
{
    pthread_mutex_lock(&g_rf_power_mutex);

    if (g_bd_power_enabled)
    {
        printf("[POWER] BD power off\n");
        gpio_set_value(0, line_bd_en);
        gpio_set_value(0, line_bd_pow);
        g_bd_power_enabled = 0;
        g_rf_power_generation++;
    }

    pthread_mutex_unlock(&g_rf_power_mutex);
}

int bd_power_is_on(void)
{
    int enabled;

    pthread_mutex_lock(&g_rf_power_mutex);
    enabled = g_bd_power_enabled;
    pthread_mutex_unlock(&g_rf_power_mutex);

    return enabled;
}

void eg_power_on(void)
{
    pthread_mutex_lock(&g_rf_power_mutex);

    if (!g_eg_power_enabled)
    {
        printf("[POWER] EG power on\n");
        gpio_set_value(1, line_usb_pow);
        gpio_set_value(1, line_4g_pow);
        gpio_set_value(0, line_4g_boot);
        sleep(1);
        gpio_set_value(1, line_4g_boot);

        g_eg_power_enabled = 1;
        g_rf_power_generation++;
    }

    pthread_mutex_unlock(&g_rf_power_mutex);
}

void eg_power_off(void)
{
    pthread_mutex_lock(&g_rf_power_mutex);

    if (g_eg_power_enabled)
    {
        printf("[POWER] EG power off\n");

        gpio_set_value(0, line_4g_pow);
        gpio_set_value(0, line_usb_pow);

        gpio_set_value(1, line_4g_boot);
        sleep(1);
        gpio_set_value(0, line_4g_boot);
        sleep(1);
        gpio_set_value(1, line_4g_boot);

        g_eg_power_enabled = 0;
        g_rf_power_generation++;
    }

    pthread_mutex_unlock(&g_rf_power_mutex);
}

int eg_power_is_on(void)
{
    int enabled;

    pthread_mutex_lock(&g_rf_power_mutex);
    enabled = g_eg_power_enabled;
    pthread_mutex_unlock(&g_rf_power_mutex);

    return enabled;
}

void rf_power_on(void)
{
    bd_power_on();
    eg_power_on();
    gpio_set_value(0, line_bt_pow);
}

void rf_power_off(void)
{
    bd_power_off();
    eg_power_off();
    gpio_set_value(0, line_bt_pow);
}

int rf_power_is_on(void)
{
    int enabled;

    pthread_mutex_lock(&g_rf_power_mutex);
    enabled = g_bd_power_enabled || g_eg_power_enabled;
    pthread_mutex_unlock(&g_rf_power_mutex);

    return enabled;
}

unsigned int rf_power_get_generation(void)
{
    unsigned int generation;

    pthread_mutex_lock(&g_rf_power_mutex);
    generation = g_rf_power_generation;
    pthread_mutex_unlock(&g_rf_power_mutex);

    return generation;
}
void main_set_4g_available(int available)
{
    pthread_mutex_lock(&g_4g_mutex);
    g_4g_available = available;
    pthread_mutex_unlock(&g_4g_mutex);
}
void main_eg_power_down(int *eg_initialized,
                        int *eg_connected,
                        unsigned int *eg_power_generation_seen)
{
    if (eg_fd >= 0)
    {
        if (*eg_connected)
        {
            eg_send_cmd("AT+QICLOSE=0\r\n", "OK", 5);
        }

        eg_send_cmd("AT+QIDEACT=1\r\n", "OK", 10);
    }

    *eg_connected = 0;
    *eg_initialized = 0;
    main_set_4g_available(0);

    eg_power_off();
    uart_close_eg();

    *eg_power_generation_seen = rf_power_get_generation();
}
void main_all_power_down(int *eg_initialized,
                         int *eg_connected,
                         unsigned int *eg_power_generation_seen)
{
    printf("[MAIN] No pending data, powering off communication modules\n");

    main_eg_power_down(eg_initialized,
                       eg_connected,
                       eg_power_generation_seen);

    bd_power_off();
}
int main_ensure_bd_ready(void)
{
    if (!bd_power_is_on())
    {
        printf("[MAIN] Powering on BD module\n");
        bd_status_reset();
        bd_power_on();
        sleep(14);
    }

    return 0;
}

// void rf_power_on(void)
// {
//     pthread_mutex_lock(&g_rf_power_mutex);

//     if (g_rf_power_enabled)
//     {
//         pthread_mutex_unlock(&g_rf_power_mutex);
//         return;
//     }

//     printf("[POWER] RF modules on (BT kept on)\n");

//     gpio_set_value(1, line_bd_pow);
//     gpio_set_value(1, line_bd_en);
//     gpio_set_value(1, line_usb_pow);
//     gpio_set_value(1, line_4g_pow);
//     gpio_set_value(0, line_bt_pow); // 蓝牙保持上电
//     gpio_set_value(0, line_4g_boot);

//     sleep(1);
//     gpio_set_value(1, line_4g_boot);

//     g_rf_power_enabled = 1;
//     g_rf_power_generation++;

//     pthread_mutex_unlock(&g_rf_power_mutex);
// }
// void rf_power_off(void)
// {
//     pthread_mutex_lock(&g_rf_power_mutex);

//     if (!g_rf_power_enabled)
//     {
//         pthread_mutex_unlock(&g_rf_power_mutex);
//         return;
//     }

//     printf("[POWER] RF modules off (BT kept on, RS485 unaffected)\n");

//     gpio_set_value(0, line_bd_en);
//     gpio_set_value(0, line_bd_pow);
//     gpio_set_value(0, line_4g_pow);
//     gpio_set_value(0, line_usb_pow);
//     // gpio_set_value(0, line_bt_pow); // 蓝牙不关，低电平为开启
//     gpio_set_value(1, line_4g_boot);

//     sleep(1);
//     printf("Powering off 4G module...\n");
//     gpio_set_value(0, line_4g_boot);
//     sleep(1);
//     gpio_set_value(1, line_4g_boot);

//     g_rf_power_enabled = 0;
//     g_rf_power_generation++;

//     pthread_mutex_unlock(&g_rf_power_mutex);
// }
// int rf_power_is_on(void)
// {
//     int enabled;

//     pthread_mutex_lock(&g_rf_power_mutex);
//     enabled = g_rf_power_enabled;
//     pthread_mutex_unlock(&g_rf_power_mutex);

//     return enabled;
// }

// unsigned int rf_power_get_generation(void)
// {
//     unsigned int generation;

//     pthread_mutex_lock(&g_rf_power_mutex);
//     generation = g_rf_power_generation;
//     pthread_mutex_unlock(&g_rf_power_mutex);

//     return generation;
// }