#include "llcc68.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <gpiod.h>
#include <pthread.h>
#include "../inc/universal.h"

#define FREQ_STEP 0.953674
#define MAX_FRAME_SIZE 256
#define DIO3_3_3V 0x07
#define LORA_SPI_SPEED 1000000
#ifdef DEBUG
#define IRQ_TX_DONE 0x0001
#define IRQ_RX_DONE 0x0002
#define IRQ_PREAMBLE_DETECTED 0x0004
#define IRQ_SYNCWORD_VALID 0x0008
#define IRQ_HEADER_VALID 0x0010
#define IRQ_HEADER_ERR 0x0020
#define IRQ_CRC_ERR 0x0040
#define IRQ_TIMEOUT 0x0200

#define IRQ_RX_DEBUG_MASK (IRQ_RX_DONE | IRQ_PREAMBLE_DETECTED |   \
						   IRQ_SYNCWORD_VALID | IRQ_HEADER_VALID | \
						   IRQ_HEADER_ERR | IRQ_CRC_ERR | IRQ_TIMEOUT)
#endif
// 全局句柄
int spi_fd = -1;
static struct gpiod_chip *gpio_chip = NULL;
static struct gpiod_line *line_nss = NULL;
static struct gpiod_line *line_reset = NULL;
static struct gpiod_line *line_busy = NULL;
static struct gpiod_line *line_dio1 = NULL;
static uint8_t regModeParam = 0x01; // 0: LDO, 1: DC-DC
static loRa_Para_t *lora_para_pt;
static uint8_t last_frame[MAX_FRAME_SIZE];
static int last_len = 0;
static uint8_t tx_buf[260];
static uint8_t rx_buf[260];
static pthread_mutex_t g_lora_lock = PTHREAD_MUTEX_INITIALIZER;
extern pthread_mutex_t g_lora_cfg_mutex;
extern loRa_Para_t my_lora_config;
int CheckBusy(int timeout_ms);
uint16_t GetIrqStatus(void);
static void dump_lora_irq(uint16_t irq)
{
	printf("[LORA IRQ] irq=0x%04X", irq);

	if (irq & IRQ_TX_DONE)
		printf(" TX_DONE");
	if (irq & IRQ_RX_DONE)
		printf(" RX_DONE");
	if (irq & IRQ_PREAMBLE_DETECTED)
		printf(" PREAMBLE");
	if (irq & IRQ_SYNCWORD_VALID)
		printf(" SYNCWORD");
	if (irq & IRQ_HEADER_VALID)
		printf(" HEADER_VALID");
	if (irq & IRQ_HEADER_ERR)
		printf(" HEADER_ERR");
	if (irq & IRQ_CRC_ERR)
		printf(" CRC_ERR");
	if (irq & IRQ_TIMEOUT)
		printf(" TIMEOUT");

	printf("\n");
}
uint8_t GetStatusRaw(void)
{
	uint8_t tx[2] = {0xC0, 0x00};
	uint8_t rx[2] = {0};

	spi_transfer(tx, rx, 2);

	// printf("[LORA STATUS RAW] rx=%02X %02X\n", rx[0], rx[1]);

	return rx[1];
}
static int lora_write_cmd(uint8_t opcode, const uint8_t *params, uint8_t len)
{
	uint8_t buf[16];

	if (len + 1 > sizeof(buf))
		return -1;

	if (CheckBusy(100) != 0)
		return -1;

	buf[0] = opcode;

	if (len > 0 && params != NULL)
		memcpy(&buf[1], params, len);

	spi_transfer(buf, NULL, len + 1);
	return 0;
}
void SetDIO2AsRfSwitchCtrl(void)
{
	uint8_t enable = 0x01;
	lora_write_cmd(0x9D, &enable, 1);
}

void SetDIO3AsTCXOCtrl(uint8_t tcxoVoltage)
{
	uint8_t params[4];

	params[0] = tcxoVoltage; // 0x07 = 3.3V
	params[1] = 0x00;
	params[2] = 0x00;
	params[3] = 0x64; // 官方例程用 0x000064

	lora_write_cmd(0x97, params, 4);
}
void lora_cfg_get(loRa_Para_t *out)
{
	if (out == NULL)
		return;

	pthread_mutex_lock(&g_lora_cfg_mutex);
	*out = my_lora_config;
	pthread_mutex_unlock(&g_lora_cfg_mutex);
}

void lora_cfg_set(uint8_t field, uint8_t value)
{
	pthread_mutex_lock(&g_lora_cfg_mutex);

	switch (field)
	{
	case 0: // is_root
		my_lora_config.is_root = value;
		break;
	case 1: // mesh_type
		my_lora_config.mesh_type = value;
		break;
	case 2: // net_id
		my_lora_config.net_id = value;
		break;
	case 3: // dev_id
		my_lora_config.dev_id = value;
		break;
	default:
		break;
	}

	pthread_mutex_unlock(&g_lora_cfg_mutex);
}
int lora_cfg_save_persist(void)
{
	loRa_Para_t cfg;
	int fd, dirfd;
	char buf[128];
	int len;

	lora_cfg_get(&cfg);

	fd = open(LORA_CFG_TMP_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (fd < 0)
	{
		perror("open LORA_CFG_TMP_PATH");
		return -1;
	}

	len = snprintf(buf, sizeof(buf),
				   "is_root=0x%02X\n"
				   "mesh_type=0x%02X\n"
				   "net_id=0x%02X\n"
				   "dev_id=0x%02X\n",
				   cfg.is_root, cfg.mesh_type, cfg.net_id, cfg.dev_id);

	if (write(fd, buf, len) != len)
	{
		perror("write lora cfg");
		close(fd);
		return -1;
	}

	if (fsync(fd) != 0)
	{
		perror("fsync lora cfg");
		close(fd);
		return -1;
	}

	close(fd);

	if (rename(LORA_CFG_TMP_PATH, LORA_CFG_PATH) != 0)
	{
		perror("rename lora cfg");
		return -1;
	}

	dirfd = open("/home/cat", O_RDONLY | O_DIRECTORY);
	if (dirfd >= 0)
	{
		fsync(dirfd);
		close(dirfd);
	}

	printf("[LORA CFG] saved: root=0x%02X mesh=0x%02X net=0x%02X dev=0x%02X\n",
		   cfg.is_root, cfg.mesh_type, cfg.net_id, cfg.dev_id);
	return 0;
}
int lora_cfg_load_persist(loRa_Para_t *cfg)
{
	FILE *fp;
	char line[64];
	unsigned int val;

	if (cfg == NULL)
		return -1;

	fp = fopen(LORA_CFG_PATH, "r");
	if (fp == NULL)
	{
		printf("[LORA CFG] no cfg file, using default config\n");
		return 0;
	}

	while (fgets(line, sizeof(line), fp))
	{
		if (sscanf(line, "is_root=0x%x", &val) == 1)
		{
			if (val == LORA_MESH_ROOT || val == LORA_MESH_NOTROOT)
				cfg->is_root = (uint8_t)val;
		}
		else if (sscanf(line, "mesh_type=0x%x", &val) == 1)
		{
			if (val == LORA_MESH_GATEWAY || val == LORA_MESH_NODE)
				cfg->mesh_type = (uint8_t)val;
		}
		else if (sscanf(line, "net_id=0x%x", &val) == 1)
		{
			cfg->net_id = (uint8_t)val;
		}
		else if (sscanf(line, "dev_id=0x%x", &val) == 1)
		{
			cfg->dev_id = (uint8_t)val;
		}
	}

	fclose(fp);

	printf("[LORA CFG] loaded: root=0x%02X mesh=0x%02X net=0x%02X dev=0x%02X\n",
		   cfg->is_root, cfg->mesh_type, cfg->net_id, cfg->dev_id);
	return 0;
}
int hw_init(void)
{
	spi_fd = open(SPI_DEV_PATH, O_RDWR);
	if (spi_fd < 0)
	{
		perror("Failed to open SPI");
		return -1;
	}
	uint8_t mode = SPI_MODE_0;
	uint8_t bits = 8;
	uint32_t speed = LORA_SPI_SPEED;

	if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0)
		perror("SPI_IOC_WR_MODE");
	if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
		perror("SPI_IOC_WR_BITS_PER_WORD");
	if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
		perror("SPI_IOC_WR_MAX_SPEED_HZ");

	gpio_chip = gpiod_chip_open_by_name(GPIO_CHIP_NAME);
	if (!gpio_chip)
	{
		perror("Failed to open GPIO chip");
		close(spi_fd);
		return -1;
	}

	line_reset = gpiod_chip_get_line(gpio_chip, PIN_RESET);
	line_busy = gpiod_chip_get_line(gpio_chip, PIN_BUSY);
	line_dio1 = gpiod_chip_get_line(gpio_chip, PIN_DIO1);

	if (!line_reset || !line_busy || !line_dio1)
	{
		perror("gpiod_chip_get_line lora");
		gpiod_chip_close(gpio_chip);
		close(spi_fd);
		return -1;
	}

	if (gpiod_line_request_output(line_reset, "llcc68_reset", 1) < 0)
	{
		perror("gpiod_line_request_output reset");
		return -1;
	}

	if (gpiod_line_request_input(line_busy, "llcc68_busy") < 0)
	{
		perror("gpiod_line_request_input busy");
		return -1;
	}

	if (gpiod_line_request_input(line_dio1, "llcc68_dio1") < 0)
	{
		perror("gpiod_line_request_input dio1");
		return -1;
	}

	// printf("[LORA GPIO] chip=%s reset=%d busy=%d dio1=%d\n",
	// 	   GPIO_CHIP_NAME, PIN_RESET, PIN_BUSY, PIN_DIO1);

	// printf("[LORA GPIO] initial busy=%d dio1=%d\n",
	// 	   gpiod_line_get_value(line_busy),
	// 	   gpiod_line_get_value(line_dio1));

	return 0;
}
// void spi_init()
// {
// 	// 1. 打开SPI设备
// 	int fd = open(SPI_DEV_PATH, O_RDWR);
// 	if (fd < 0)
// 	{
// 		perror("无法打开SPI设备");
// 		return 1;
// 	}

// 	// 2. 配置SPI参数
// 	uint8_t mode = SPI_MODE_0; // CPOL=0, CPHA=0
// 	uint8_t bits = 8;		   // 8位数据
// 	uint32_t speed = 1000000;  // 1MHz（可根据需要调整）

// 	ioctl(fd, SPI_IOC_WR_MODE, &mode);
// 	ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
// 	ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
// }
void reset_llcc68()
{
	gpiod_line_set_value(line_reset, 0); // 拉低复位
	usleep(2 * 1000);
	gpiod_line_set_value(line_reset, 1);
	usleep(20 * 1000);
}
// 核心 SPI 传输函数 (对应 Arduino 的 SPI.transfer)
void spi_transfer(uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len)
{
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx_buf,
		.rx_buf = (unsigned long)rx_buf,
		.len = len,
		.delay_usecs = 0,
		.speed_hz = LORA_SPI_SPEED,
		.bits_per_word = 8,
	};

	int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
	{
		perror("[LORA SPI] SPI_IOC_MESSAGE");
	}
}
void SetStandby(uint8_t StdbyConfig)
{
	if (CheckBusy(100) != 0)
	{
		return;
	}
	lora_write_cmd(SET_STANDBY, &StdbyConfig, 1);
}
int CheckBusy(int timeout_ms)
{
	int cnt = 0;

	while (gpiod_line_get_value(line_busy))
	{
		usleep(1000);
		cnt++;

		if (cnt >= timeout_ms)
		{
			printf("[LORA] BUSY timeout, busy still high\n");
			return -1;
		}
	}

	return 0;
}
void SetRegulatorMode(void)
{
	if (CheckBusy(100) != 0)
	{
		return;
	}

	lora_write_cmd(0x96, &regModeParam, 1);
}

void SetPaConfig(void)
{
	uint8_t tx_cmd[4];
	if (CheckBusy(100) != 0)
	{
		return;
	}
	tx_cmd[0] = 0x04;
	tx_cmd[1] = 0x07;
	tx_cmd[2] = 0x00;
	tx_cmd[3] = regModeParam;
	lora_write_cmd(0x95, tx_cmd, 4);
}

void SetPacketType(uint8_t PacketType)
{
	lora_write_cmd(SET_PACKET_TYPE, &PacketType, 1);
}
void SetRfFrequency(uint32_t frequency)
{
	uint8_t params[4];
	uint32_t rfFreq = (uint32_t)((double)frequency / (double)FREQ_STEP);

	params[0] = (rfFreq >> 24) & 0xFF;
	params[1] = (rfFreq >> 16) & 0xFF;
	params[2] = (rfFreq >> 8) & 0xFF;
	params[3] = rfFreq & 0xFF;

	lora_write_cmd(SET_RF_FREQUENCY, params, 4);
}
void SetTxParams(int8_t power, uint8_t RampTime)
{
	uint8_t params[2];

	params[0] = (uint8_t)power;
	params[1] = RampTime;

	lora_write_cmd(SET_TX_PARAMS, params, 2);
}

void SetModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
	uint8_t params[8];
	if (CheckBusy(100) != 0)
	{
		return;
	}
	params[0] = sf;	  // SF=5~12
	params[1] = bw;	  // BW
	params[2] = cr;	  // CR
	params[3] = ldro; // LDRO LowDataRateOptimize 0:OFF; 1:ON;
	params[4] = 0xFF;
	params[5] = 0xFF;
	params[6] = 0xFF;
	params[7] = 0xFF;
	lora_write_cmd(0x8B, params, 8);
}

void SetPacketParams(uint8_t payload_len)
{
	uint8_t Opcode;
	uint16_t prea_len = 8;
	uint8_t params[9];
	if (CheckBusy(100) != 0)
	{
		return;
	}

	params[0] = prea_len >> 8;
	params[1] = prea_len & 0xFF;
	params[2] = 0x00;
	params[3] = payload_len;
	params[4] = 0x01;
	params[5] = 0x00;
	params[6] = 0xFF;
	params[7] = 0xFF;
	params[8] = 0xFF;

	lora_write_cmd(0x8C, params, 9);
}
void LLCC68_Config()
{
	uint32_t rf_freq_temp;
	int8_t power_temp;
	uint8_t sf_temp;
	uint8_t bw_temp;
	uint8_t cr_temp;
	uint8_t size_temp;

	rf_freq_temp = lora_para_pt->rf_freq;
	power_temp = lora_para_pt->tx_power;
	sf_temp = lora_para_pt->lora_sf;
	bw_temp = lora_para_pt->band_width;
	cr_temp = lora_para_pt->code_rate;
	size_temp = lora_para_pt->payload_size;

	SetStandby(0); // 0:STDBY_RC; 1:STDBY_XOSC
	SetRegulatorMode();
	SetPaConfig();
	// SetDIO3AsTCXOCtrl(DIO3_3_3V);
	SetDIO2AsRfSwitchCtrl();
	SetPacketType(1);					   // 0:GFSK; 1:LORA
	SetRfFrequency(rf_freq_temp);		   // RF_Freq = freq_reg*32M/(2^25)
	SetTxParams(power_temp, SET_RAMP_10U); // set power and ramp_time

	SetModulationParams(sf_temp, bw_temp, cr_temp, LDRO_ON);

	SetPacketParams(size_temp); // PreambleLength;HeaderType;PayloadLength;CRCType;InvertIQ
}

void SetSleep(void)
{
	uint8_t sleepConfig;

	if (CheckBusy(100) != 0)
	{
		return;
	}
	sleepConfig = 0x00; // 0x04;	//bit2: 1:warm start; bit0:0: RTC timeout disable
	lora_write_cmd(SET_SLEEP, &sleepConfig, 1);
}
void SetBufferBaseAddress(uint8_t TX_base_addr, uint8_t RX_base_addr)
{
	uint8_t params[2];
	if (CheckBusy(100) != 0)
	{
		return;
	}
	params[0] = TX_base_addr;
	params[1] = RX_base_addr;

	lora_write_cmd(SET_BUF_BASE_ADDR, params, 2);
}
void SetDioIrqParams(uint16_t irq)
{

	uint8_t params[8];
	params[0] = irq >> 8;
	params[1] = irq & 0xFF;
	params[2] = irq >> 8;
	params[3] = irq & 0xFF;
	params[4] = 0;
	params[5] = 0;
	params[6] = 0;
	params[7] = 0;

	if (CheckBusy(100) != 0)
	{
		return;
	}

	lora_write_cmd(0x08, params, 8);
}
void SetRx(uint32_t timeout)
{
	uint8_t time_out[3];

	if (CheckBusy(100) != 0)
	{
		return;
	}
	time_out[0] = (timeout >> 16) & 0xFF; // MSB
	time_out[1] = (timeout >> 8) & 0xFF;
	time_out[2] = timeout & 0xFF; // LSB

	lora_write_cmd(SET_RX, time_out, 3);
}
void RxInit(void)
{
	SetStandby(0);
	SetPacketParams(lora_para_pt->payload_size);
	SetBufferBaseAddress(0, 0);
	ClearIrqStatus(0xFFFF);
	SetDioIrqParams(IRQ_RX_DEBUG_MASK);
	SetRx(0xFFFFFF);
	// printf("[LORA RX] RxInit done, DIO1=%d BUSY=%d IRQ=0x%04X\n",
	// 	   gpiod_line_get_value(line_dio1),
	// 	   gpiod_line_get_value(line_busy),
	// 	   GetIrqStatus());
}
void SetTx(uint32_t timeout)
{
	uint8_t time_out[3];

	if (CheckBusy(100) != 0)
	{
		return;
	} // 0x83
	time_out[0] = (timeout >> 16) & 0xFF; // MSB
	time_out[1] = (timeout >> 8) & 0xFF;
	time_out[2] = timeout & 0xFF; // LSB

	lora_write_cmd(SET_TX, time_out, 3);
}
bool Lora_init(loRa_Para_t *lp_pt)
{
	lora_para_pt = lp_pt;

	if (hw_init() != 0)
	{
		printf("[LORA] hw_init failed\n");
		return false;
	}

	reset_llcc68();
	// printf("[LORA] status after reset=0x%02X\n", GetStatusRaw());
	LLCC68_Config();
	// printf("[LORA] status after config=0x%02X\n", GetStatusRaw());
	RxInit();

	return true;
}
uint16_t GetIrqStatus(void)
{
	uint8_t tx_buf[4] = {0x12, 0xFF, 0xFF, 0xFF};
	uint8_t rx_buf[4] = {0};
	uint16_t irq_status = 0;

	if (CheckBusy(100) != 0)
	{
		printf("[LORA] GetIrqStatus BUSY timeout\n");
		return 0xFFFF;
	}

	spi_transfer(tx_buf, rx_buf, 4);

	// printf("[LORA IRQ RAW] rx=%02X %02X %02X %02X\n",
	// 	   rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

	/*
	 * rx_buf[1] 是 Status
	 * rx_buf[2] 是 IRQ MSB
	 * rx_buf[3] 是 IRQ LSB
	 */
	irq_status = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];

	return irq_status;
}
void GetRxBufferStatus(uint8_t *payload_len, uint8_t *buf_pointer)
{
	uint8_t tx_buf[4] = {0x13, 0xFF, 0xFF, 0xFF};
	uint8_t rx_buf[4] = {0};

	if (payload_len == NULL || buf_pointer == NULL)
		return;

	if (CheckBusy(100) != 0)
	{
		printf("[LORA] GetRxBufferStatus BUSY timeout\n");
		*payload_len = 0;
		*buf_pointer = 0;
		return;
	}

	spi_transfer(tx_buf, rx_buf, 4);

	printf("[LORA RXBUF RAW] rx=%02X %02X %02X %02X\n",
		   rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

	/*
	 * rx_buf[1] 是 Status
	 * rx_buf[2] 是 payload_len
	 * rx_buf[3] 是 buf_pointer
	 */
	*payload_len = rx_buf[2];
	*buf_pointer = rx_buf[3];
}
void ReadBuffer(uint8_t offset, uint8_t *data, uint8_t length)
{
	if (data == NULL || length < 1)
		return;

	if ((3 + length) > sizeof(tx_buf) || (3 + length) > sizeof(rx_buf))
		return;

	if (CheckBusy(100) != 0)
	{
		return;
	}

	uint32_t total_len = 3 + length;

	tx_buf[0] = 0x1E;
	tx_buf[1] = offset;
	tx_buf[2] = 0xFF;
	memset(&tx_buf[3], 0xFF, length);
	memset(rx_buf, 0, total_len);

	spi_transfer(tx_buf, rx_buf, total_len);

	memcpy(data, &rx_buf[3], length);
}
void ClearIrqStatus(uint16_t irq)
{
	uint8_t params[2];
	if (CheckBusy(100) != 0)
	{
		return;
	}
	params[0] = (irq >> 8);	  // 中断掩码高字节
	params[1] = (irq & 0xFF); // 中断掩码低字节
	lora_write_cmd(0x02, params, 2);
}
uint8_t WaitForIRQ_RxDone(void)
{
	uint16_t Irq_Status;
	uint8_t packet_size;
	uint8_t buf_offset;
	uint8_t RF_DIO1;
	if (gpiod_line_get_value(line_dio1)) // if IRQ check
	{
		Irq_Status = GetIrqStatus(); // read Irq Status
		if ((Irq_Status & 0x02) == RxDone_IRQ)
		{
			GetRxBufferStatus(&packet_size, &buf_offset);
			if (packet_size == 0 || packet_size > lora_para_pt->payload_size)
			{
				ClearIrqStatus(RxDone_IRQ);
				RxInit();
				return 0;
			}
			ReadBuffer(buf_offset, rxbuf_pt, packet_size);
			*rxcnt_pt = packet_size;
			ClearIrqStatus(RxDone_IRQ); // Clear the IRQ RxDone flag
			RxInit();
			return 1;
		}
	}
	return 0;
}

uint8_t WaitForIRQ_TxDone(void)
{
	int timeout_ms = 3000;

	while (timeout_ms-- > 0)
	{
		if (gpiod_line_get_value(line_dio1))
		{
			uint16_t irq = GetIrqStatus();

			if (irq & TxDone_IRQ)
			{
				ClearIrqStatus(TxDone_IRQ);
				return 1;
			}

			if (irq != 0)
				ClearIrqStatus(irq);
		}

		usleep(1000);
	}

	printf("[LORA] TxDone timeout, DIO1=%d IRQ=0x%04X\n",
		   gpiod_line_get_value(line_dio1),
		   GetIrqStatus());

	ClearIrqStatus(0xFFFF);
	return 0;
}
void WriteBuffer(uint8_t offset, uint8_t *data, uint8_t length)
{
	uint8_t buf[260];

	if (data == NULL || length < 1)
		return;

	if ((2 + length) > sizeof(buf))
		return;

	if (CheckBusy(100) != 0)
		return;

	buf[0] = 0x0E;
	buf[1] = offset;
	memcpy(&buf[2], data, length);

	spi_transfer(buf, NULL, 2 + length);
}
int Lora_send(uint8_t *payload, uint8_t size)
{
	int ok;

	if (payload == NULL || size == 0)
		return -1;

	pthread_mutex_lock(&g_lora_lock);

	SetStandby(0);
	SetBufferBaseAddress(0, 0);
	WriteBuffer(0, payload, size);
	SetPacketParams(size);
	ClearIrqStatus(0xFFFF);
	SetDioIrqParams(TxDone_IRQ);
	SetTx(0);

	ok = WaitForIRQ_TxDone();

	if (ok)
	{
		printf("Data sent successfully.\n");
		RxInit();
		pthread_mutex_unlock(&g_lora_lock);
		return 0;
	}

	printf("Data send failed or timeout.\n");
	RxInit();
	pthread_mutex_unlock(&g_lora_lock);
	return -1;
}
int lora_unpack(uint8_t *buf, int len,
				uint8_t expect_netid,
				uint8_t expect_devid,
				uint8_t *out_payload,
				int *out_len)
{
	if (len < 4)
		return -1; // 至少 NETID+DEVID+CRC

	// 1. CRC校验
	uint16_t recv_crc = buf[len - 2] | (buf[len - 1] << 8);
	uint16_t calc_crc = crc16(buf, len - 2);

	if (recv_crc != calc_crc)
	{
		printf("[LORA] CRC error\n");
		return -2;
	}

	// 2. NETID / DEVID 校验
	if (buf[0] != expect_netid || buf[1] != expect_devid)
	{
		printf("[LORA] ID mismatch\n");
		return -3;
	}

	// 3. 去重（完全一样才丢）
	if (len == last_len && memcmp(buf, last_frame, len) == 0)
	{
		printf("[LORA] Duplicate packet dropped\n");
		return -4;
	}

	// 保存为最新帧
	memcpy(last_frame, buf, len);
	last_len = len;

	// 4. 提取payload
	int payload_len = len - 4;
	memcpy(out_payload, &buf[2], payload_len);
	*out_len = payload_len;

	return 0;
}
void Lora_receive(uint8_t *payload, uint8_t size)
{
	rxbuf_pt = payload;
	rxcnt_pt = &size;
	SetBufferBaseAddress(0, 0);
	SetDioIrqParams(RxDone_IRQ);
	SetRx(0);
	if (WaitForIRQ_RxDone())
	{
		uint8_t payload[256];
		int payload_len;

		int ret = lora_unpack(rxbuf_pt, *rxcnt_pt,
							  0x01, 0x02, // 你的NETID/DEVID
							  payload, &payload_len);

		if (ret == 0)
		{
			printf("[LORA] Valid packet (%d bytes): ", payload_len);
			for (int i = 0; i < payload_len; i++)
				printf("%02X ", payload[i]);
			printf("\n");
		}
	}
}
int lora_pack(uint8_t netid, uint8_t devid,
			  uint8_t *payload, uint8_t payload_len,
			  uint8_t *out_buf)
{
	int len = 0;

	out_buf[len++] = netid;
	out_buf[len++] = devid;

	memcpy(&out_buf[len], payload, payload_len);
	len += payload_len;

	uint16_t crc = crc16(out_buf, len);

	out_buf[len++] = crc & 0xFF;		// 低字节
	out_buf[len++] = (crc >> 8) & 0xFF; // 高字节

	return len;
}
int Lora_send_packet(uint8_t netid, uint8_t devid,
					 uint8_t *payload, uint8_t size)
{
	uint8_t buf[256];
	int len;

	if (payload == NULL || size == 0)
		return -1;

	len = lora_pack(netid, devid, payload, size, buf);
	return Lora_send(buf, len);
}

int Lora_recv_packet(uint8_t *payload, uint8_t *out_len)
{
	uint16_t irq_status;
	uint8_t packet_size = 0;
	uint8_t buf_offset = 0;
	static time_t last_print = 0;
	time_t now = time(NULL);

	if (now != last_print)
	{
		printf("[LORA GPIO] DIO1=%d BUSY=%d IRQ=0x%04X\n",
			   gpiod_line_get_value(line_dio1),
			   gpiod_line_get_value(line_busy),
			   GetIrqStatus());
		last_print = now;
	}
	if (payload == NULL || out_len == NULL)
		return -1;

	pthread_mutex_lock(&g_lora_lock);

	// 没有中断，表示暂时没有收到数据
	if (!gpiod_line_get_value(line_dio1))
	{
		pthread_mutex_unlock(&g_lora_lock);
		return 0;
	}

	irq_status = GetIrqStatus();
	dump_lora_irq(irq_status);
	if ((irq_status & RxDone_IRQ) != RxDone_IRQ)
	{
		ClearIrqStatus(irq_status);
		pthread_mutex_unlock(&g_lora_lock);
		return 0;
	}

	GetRxBufferStatus(&packet_size, &buf_offset);

	if (packet_size == 0 || packet_size > lora_para_pt->payload_size)
	{
		ClearIrqStatus(RxDone_IRQ);
		RxInit();
		pthread_mutex_unlock(&g_lora_lock);
		return -2;
	}

	ReadBuffer(buf_offset, payload, packet_size);
	*out_len = packet_size;

	ClearIrqStatus(RxDone_IRQ);
	RxInit(); // 收完立刻回到接收态

	pthread_mutex_unlock(&g_lora_lock);
	return 1;
}