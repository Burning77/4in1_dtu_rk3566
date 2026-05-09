#ifndef LLCC68_PORT_H
#define LLCC68_PORT_H

#include <stdint.h>
#include <stdbool.h>
#include <linux/spi/spidev.h>

#define SPI_DEV_PATH "/dev/spidev2.0" // 修改为你的 SPI 设备
#define GPIO_CHIP_NAME "gpiochip3"	  // 修改为 gpiodetect 查到的名称

#define PIN_RESET 22 // GPIO3_PC6 (复位脚，输出)
#define PIN_BUSY 23	 // GPIO3_PC7 (忙信号，输入)
#define PIN_DIO1 24	 // GPIO3_PD0 (中断脚，输入)

#define SET_SLEEP 0x84
#define SET_STANDBY 0x80
#define SET_TX 0x83
#define SET_RX 0x82
#define SET_PACKET_TYPE 0x8A
#define SET_RF_FREQUENCY 0x86
#define SET_TX_PARAMS 0x8E
#define SET_BUF_BASE_ADDR 0x8F

#define SET_RAMP_10U 0x00
#define SET_RAMP_20U 0x01
#define SET_RAMP_40U 0x02
#define SET_RAMP_80U 0x03
#define SET_RAMP_200U 0x04
#define SET_RAMP_800U 0x05
#define SET_RAMP_1700U 0x06
#define SET_RAMP_3400U 0x07

#define LDRO_ON 0x01
#define LDRO_OFF 0x00

#define RxDone_IRQ 0x02
#define TxDone_IRQ 0x01

uint8_t *rxbuf_pt;
uint16_t *rxcnt_pt;

// -----------------------------------
typedef struct
{
	uint8_t is_root;
	uint8_t mesh_type;
	uint8_t net_id;
	uint8_t dev_id;
	uint32_t rf_freq;
	int8_t tx_power;
	uint8_t lora_sf;
	uint8_t band_width;
	uint8_t code_rate;
	uint8_t payload_size;
} loRa_Para_t;

bool Lora_init(loRa_Para_t *lp_pt);

// 底层 SPI 传输
void spi_transfer(uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len);
int Lora_send(uint8_t *payload, uint8_t size);
void Lora_receive(uint8_t *payload, uint8_t size);
int Lora_send_packet(uint8_t netid, uint8_t devid,
					 uint8_t *payload, uint8_t size);
int Lora_recv_packet(uint8_t *payload, uint8_t *out_len);
void lora_cfg_get(loRa_Para_t *out);
void lora_cfg_set(uint8_t field, uint8_t value);
int lora_cfg_save_persist(void);
int lora_cfg_load_persist(loRa_Para_t *cfg);
#endif // LLCC68_PORT_H