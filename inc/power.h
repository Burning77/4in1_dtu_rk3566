/**
 * @file power.h
 * @brief 低功耗管理模块
 *
 * 实现功能:
 * 1. 系统休眠 (suspend-to-RAM)
 * 2. RTC 定时唤醒
 * 3. 外设电源控制
 * 4. 低功耗工作流程管理
 */

#ifndef __POWER_H__
#define __POWER_H__
void main_set_4g_available(int available);
void main_power_down_rf(int *eg_initialized,
                        int *eg_connected,
                        unsigned int *eg_power_generation_seen);
void main_all_power_down(int *eg_initialized,
                         int *eg_connected,
                         unsigned int *eg_power_generation_seen);
int main_ensure_bd_ready(void);
void bd_power_on(void);
void bd_power_off(void);
int bd_power_is_on(void);

void eg_power_on(void);
void eg_power_off(void);
int eg_power_is_on(void);

void rf_power_on(void);
void rf_power_off(void);
int rf_power_is_on(void);
unsigned int rf_power_get_generation(void);

#endif /* __POWER_H__ */
