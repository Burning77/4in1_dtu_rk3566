#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#include "../inc/bd3.h"
#include "../inc/usart.h"
#include "../inc/universal.h"
static pthread_mutex_t g_bd_status_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_bd_status_cond = PTHREAD_COND_INITIALIZER;

static int g_bd_ready_seen = 0;
static int g_bd_pwi_valid = 0;
static int g_bd_pwi_signal = 0;
static time_t g_bd_pwi_seen_time = 0;

static unsigned int g_bd_tcq_seq = 0;
static int g_bd_tcq_result = 0;
static int g_bd_tcq_code = -1;
extern volatile sig_atomic_t stop_flag;
int bd_send_packet(const unsigned char *data, int len)
{
    char hex_buf[BD_MAX_HEX_LEN + 1];
    char msg[640];

    if (!data || len <= 0)
        return -1;

    if (len > BD_MAX_RAW_LEN)
    {
        printf("[BD] payload too large: len=%d max=%d\n", len, BD_MAX_RAW_LEN);
        return -1;
    }

    char *p = hex_buf;
    for (int i = 0; i < len; i++)
    {
        p += sprintf(p, "%02X", data[i]);
    }
    *p = '\0';

    snprintf(msg, sizeof(msg), "$CCTCQ,%s,2,1,2,%s,0*",
             BD_RECEIVER_ID, hex_buf);

    unsigned char cs = calc_checksum(msg);
    size_t msg_len = strlen(msg);

    snprintf(msg + msg_len, sizeof(msg) - msg_len, "%02X\r\n", cs);

    int send_len = strlen(msg);

    printf("[BD TX] %s", msg);

    int ret = data_send((unsigned char *)msg, send_len, BD_DEV);
    if (ret == send_len)
        return 0;

    printf("[BD] send failed or partial, ret=%d expected=%d\n", ret, send_len);
    return -1;
}

void bd_task_clear(bd_segment_task_t *task)
{
    if (!task)
        return;

    memset(task, 0, sizeof(*task));
}

int bd_task_start(bd_segment_task_t *task, const unsigned char *data, int len)
{
    if (!task || !data || len <= 0 || len > EG_MSG_LEN)
        return -1;

    memset(task, 0, sizeof(*task));

    memcpy(task->data, data, len);
    task->len = len;
    task->offset = 0;
    task->active = 1;

    printf("[BD TASK] start, total=%d\n", len);
    return 0;
}

int bd_task_send_one_segment(bd_segment_task_t *task, time_t *last_bd_send_time)
{
    if (!task || !last_bd_send_time)
        return BD_TASK_FAILED;

    if (!task->active)
        return BD_TASK_DONE;

    if (*last_bd_send_time != 0)
    {
        time_t now = time(NULL);
        int wait_sec = BD_SEND_INTERVAL_SEC - (int)(now - *last_bd_send_time);

        if (wait_sec > 0)
        {
            printf("[BD TASK] wait %d sec before next segment\n", wait_sec);
            return BD_TASK_PENDING;
        }
    }

    int left = task->len - task->offset;
    int chunk_len = left;

    if (chunk_len > BD_MAX_RAW_LEN)
        chunk_len = BD_MAX_RAW_LEN;

    printf("[BD TASK] send segment offset=%d len=%d total=%d\n",
           task->offset, chunk_len, task->len);

    if (bd_send_packet(task->data + task->offset, chunk_len) != 0)
    {
        printf("[BD TASK] segment failed offset=%d len=%d\n",
               task->offset, chunk_len);
        return BD_TASK_FAILED;
    }

    *last_bd_send_time = time(NULL);
    task->offset += chunk_len;

    if (task->offset >= task->len)
    {
        printf("[BD TASK] complete total=%d\n", task->len);
        task->active = 0;
        return BD_TASK_DONE;
    }

    return BD_TASK_PENDING;
}
static void bd_status_parse_line(const char *line)
{
    if (!line || line[0] == '\0')
        return;

    pthread_mutex_lock(&g_bd_status_mutex);

    if (strstr(line, "READY FOR TRANSMISSION") != NULL ||
        strncmp(line, "$BDICP,", 7) == 0)
    {
        g_bd_ready_seen = 1;
        pthread_cond_broadcast(&g_bd_status_cond);
    }
    else if (strncmp(line, "$BDPWI,", 7) == 0)
    {
        char utc[32] = {0};
        int a = 0, valid = 0, c = 0, sig1 = 0, sig2 = 0;

        if (sscanf(line, "$BDPWI,%31[^,],%d,%d,%d,%d,%d",
                   utc, &a, &valid, &c, &sig1, &sig2) >= 6 &&
            valid == 1 && sig1 > 0 && sig2 > 0)
        {
            g_bd_pwi_valid = 1;
            g_bd_pwi_signal = sig1;
            g_bd_pwi_seen_time = monotonic_sec();
            pthread_cond_broadcast(&g_bd_status_cond);
        }
        else
        {
            g_bd_pwi_valid = 0;
            g_bd_pwi_signal = 0;
        }
    }
    else if (strncmp(line, "$BDFKI,", 7) == 0)
    {
        char cmd[16] = {0};
        char yn = 0;
        int code = -1;

        if (sscanf(line, "$BDFKI,%*[^,],%15[^,],%c,%d",
                   cmd, &yn, &code) == 3 &&
            strcmp(cmd, "TCQ") == 0)
        {
            g_bd_tcq_seq++;
            g_bd_tcq_code = code;
            g_bd_tcq_result = (yn == 'Y' && code == 0) ? 1 : -1;
            pthread_cond_broadcast(&g_bd_status_cond);
        }
    }

    pthread_mutex_unlock(&g_bd_status_mutex);
}

void bd_status_feed(const unsigned char *data, int len)
{
    static char line[256];
    static int pos = 0;

    for (int i = 0; i < len; i++)
    {
        unsigned char ch = data[i];

        if (ch == '\r')
            continue;

        if (ch == '\n')
        {
            line[pos] = '\0';
            if (pos > 0)
                bd_status_parse_line(line);
            pos = 0;
            continue;
        }

        if (ch < 0x20 || ch > 0x7E)
            continue;

        if (pos < (int)sizeof(line) - 1)
            line[pos++] = (char)ch;
        else
            pos = 0;
    }
}

unsigned int bd_get_tcq_seq(void)
{
    unsigned int seq;

    pthread_mutex_lock(&g_bd_status_mutex);
    seq = g_bd_tcq_seq;
    pthread_mutex_unlock(&g_bd_status_mutex);

    return seq;
}

int bd_wait_ready_and_pwi(int timeout_sec)
{
    struct timespec ts;

    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout_sec;

    pthread_mutex_lock(&g_bd_status_mutex);

    while (!stop_flag)
    {
        if (g_bd_ready_seen)
        {
            printf("[BD] ready, pwi_valid=%d signal=%d\n",
                   g_bd_pwi_valid, g_bd_pwi_signal);

            pthread_mutex_unlock(&g_bd_status_mutex);
            return 0;
        }

        int ret = pthread_cond_timedwait(&g_bd_status_cond,
                                         &g_bd_status_mutex,
                                         &ts);
        if (ret == ETIMEDOUT)
            break;
    }

    printf("[BD] wait ready timeout, ready=%d pwi=%d signal=%d\n",
           g_bd_ready_seen, g_bd_pwi_valid, g_bd_pwi_signal);

    pthread_mutex_unlock(&g_bd_status_mutex);
    return -1;
}

int bd_wait_tcq_success(unsigned int seq_before, int timeout_sec)
{
    struct timespec ts;

    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout_sec;

    pthread_mutex_lock(&g_bd_status_mutex);

    while (!stop_flag && g_bd_tcq_seq == seq_before)
    {
        int ret = pthread_cond_timedwait(&g_bd_status_cond,
                                         &g_bd_status_mutex,
                                         &ts);
        if (ret == ETIMEDOUT)
        {
            break;
        }
    }

    if (g_bd_tcq_seq != seq_before && g_bd_tcq_result == 1)
    {
        printf("[BD] TCQ success, code=%d\n", g_bd_tcq_code);
        pthread_mutex_unlock(&g_bd_status_mutex);
        return 0;
    }

    printf("[BD] TCQ failed/timeout, seq_before=%u seq_now=%u result=%d code=%d\n",
           seq_before, g_bd_tcq_seq, g_bd_tcq_result, g_bd_tcq_code);

    pthread_mutex_unlock(&g_bd_status_mutex);
    return -1;
}

void bd_status_reset(void)
{
    pthread_mutex_lock(&g_bd_status_mutex);

    g_bd_ready_seen = 0;
    g_bd_pwi_valid = 0;
    g_bd_pwi_signal = 0;
    g_bd_pwi_seen_time = 0;

    g_bd_tcq_seq = 0;
    g_bd_tcq_result = 0;
    g_bd_tcq_code = -1;

    pthread_mutex_unlock(&g_bd_status_mutex);
}