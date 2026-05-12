#ifndef __EG800K_H__
#define __EG800K_H__
#define SERVER_IP "115.120.239.161"
#define SERVER_PORT 23738
#define EG_CONNECT_MAX_RETRY 10
// 定义模块状态
typedef enum
{
    EG_STATE_INIT,
    EG_STATE_READY,
    EG_STATE_CONNECTED,
    EG_STATE_ERROR
} eg_state_t;

int eg_send_cmd(const char *cmd, const char *expected_resp, int timeout_sec);
int eg_init(void);
int eg_send_data(const unsigned char *data, int len);
int bd_send_packet(const unsigned char *data, int len);
int eg_is_network_available(void);
int eg_reinit_pdp(void);
int eg_connect(void);
int main_ensure_eg_ready(int *eg_initialized,
                         int *eg_connected,
                         unsigned int *eg_power_generation_seen,
                         int connect_max_retry);
#endif