#include "../inc/bd3.h"
#include "../inc/usart.h"

#include <stdio.h>
#include <string.h>

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