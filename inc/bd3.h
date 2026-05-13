 #ifndef __BD3_H__
  #define __BD3_H__

  #include <stdint.h>
  #include <time.h>
  #include "universal.h"

  #define BD_RECEIVER_ID "4314513"
  #define BD_MAX_RAW_LEN 229
  #define BD_MAX_HEX_LEN (BD_MAX_RAW_LEN * 2)
  #define BD_SEND_INTERVAL_SEC 60

  #define BD_TASK_DONE 0
  #define BD_TASK_PENDING 1
  #define BD_TASK_FAILED -1

  typedef struct
  {
      int active;
      unsigned char data[EG_MSG_LEN];
      int len;
      int offset;
  } bd_segment_task_t;

  int bd_send_packet(const unsigned char *data, int len);

  void bd_task_clear(bd_segment_task_t *task);
  int bd_task_start(bd_segment_task_t *task, const unsigned char *data, int len);
  int bd_task_send_one_segment(bd_segment_task_t *task, time_t *last_bd_send_time);

  #endif