#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#ifdef ARCH_CV182X
#include <linux/cvi_vip_snsr.h>
#include "cvi_comm_video.h"
#else
#include <linux/vi_snsr.h>
#include <linux/cvi_comm_video.h>
#endif
#include "cvi_sns_ctrl.h"
#include "sc3336_1L_cmos_ex.h"

#define SC3336_1L_CHIP_ID_HI_ADDR		0x3107
#define SC3336_1L_CHIP_ID_LO_ADDR		0x3108
#define SC3336_1L_CHIP_ID			0xcc41

static void sc3336_1l_linear_1296P20_init(VI_PIPE ViPipe);

CVI_U8 sc3336_1l_i2c_addr = 0x30;        /* I2C Address of SC3336_1L */
const CVI_U32 sc3336_1l_addr_byte = 2;
const CVI_U32 sc3336_1l_data_byte = 1;
static int g_fd[VI_MAX_PIPE_NUM] = {[0 ... (VI_MAX_PIPE_NUM - 1)] = -1};

int sc3336_1l_i2c_init(VI_PIPE ViPipe)
{
	char acDevFile[16] = {0};
	CVI_U8 u8DevNum;

	if (g_fd[ViPipe] >= 0)
		return CVI_SUCCESS;
	int ret;

	u8DevNum = g_aunSC3336_1L_BusInfo[ViPipe].s8I2cDev;
	snprintf(acDevFile, sizeof(acDevFile),  "/dev/i2c-%u", u8DevNum);

	g_fd[ViPipe] = open(acDevFile, O_RDWR, 0600);

	if (g_fd[ViPipe] < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Open /dev/cvi_i2c_drv-%u error!\n", u8DevNum);
		return CVI_FAILURE;
	}

	ret = ioctl(g_fd[ViPipe], I2C_SLAVE_FORCE, sc3336_1l_i2c_addr);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_SLAVE_FORCE error!\n");
		close(g_fd[ViPipe]);
		g_fd[ViPipe] = -1;
		return ret;
	}

	return CVI_SUCCESS;
}

int sc3336_1l_i2c_exit(VI_PIPE ViPipe)
{
	if (g_fd[ViPipe] >= 0) {
		close(g_fd[ViPipe]);
		g_fd[ViPipe] = -1;
		return CVI_SUCCESS;
	}
	return CVI_FAILURE;
}

int sc3336_1l_read_register(VI_PIPE ViPipe, int addr)
{
	int ret, data;
	CVI_U8 buf[8];
	CVI_U8 idx = 0;

	if (g_fd[ViPipe] < 0)
		return CVI_FAILURE;

	if (sc3336_1l_addr_byte == 2)
		buf[idx++] = (addr >> 8) & 0xff;

	// add address byte 0
	buf[idx++] = addr & 0xff;

	ret = write(g_fd[ViPipe], buf, sc3336_1l_addr_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_WRITE error!\n");
		return ret;
	}

	buf[0] = 0;
	buf[1] = 0;
	ret = read(g_fd[ViPipe], buf, sc3336_1l_data_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_READ error!\n");
		return ret;
	}

	// pack read back data
	data = 0;
	if (sc3336_1l_data_byte == 2) {
		data = buf[0] << 8;
		data += buf[1];
	} else {
		data = buf[0];
	}

	syslog(LOG_DEBUG, "i2c r 0x%x = 0x%x\n", addr, data);
	return data;
}

int sc3336_1l_write_register(VI_PIPE ViPipe, int addr, int data)
{
	CVI_U8 idx = 0;
	int ret;
	CVI_U8 buf[8];

	if (g_fd[ViPipe] < 0)
		return CVI_SUCCESS;

	if (sc3336_1l_addr_byte == 2) {
		buf[idx] = (addr >> 8) & 0xff;
		idx++;
		buf[idx] = addr & 0xff;
		idx++;
	}

	if (sc3336_1l_data_byte == 1) {
		buf[idx] = data & 0xff;
		idx++;
	}

	ret = write(g_fd[ViPipe], buf, sc3336_1l_addr_byte + sc3336_1l_data_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_WRITE error!\n");
		return CVI_FAILURE;
	}
	syslog(LOG_DEBUG, "i2c w 0x%x 0x%x\n", addr, data);
	return CVI_SUCCESS;
}

static void delay_ms(int ms)
{
	usleep(ms * 1000);
}

void sc3336_1l_standby(VI_PIPE ViPipe)
{
	sc3336_1l_write_register(ViPipe, 0x0100, 0x00);
}

void sc3336_1l_restart(VI_PIPE ViPipe)
{
	sc3336_1l_write_register(ViPipe, 0x0100, 0x00);
	delay_ms(20);
	sc3336_1l_write_register(ViPipe, 0x0100, 0x01);
}

void sc3336_1l_default_reg_init(VI_PIPE ViPipe)
{
	CVI_U32 i;

	for (i = 0; i < g_pastSC3336_1L[ViPipe]->astSyncInfo[0].snsCfg.u32RegNum; i++) {
		if (g_pastSC3336_1L[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].bUpdate == CVI_TRUE) {
			sc3336_1l_write_register(ViPipe,
				g_pastSC3336_1L[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32RegAddr,
				g_pastSC3336_1L[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32Data);
		}
	}
}

void sc3336_1l_mirror_flip(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip)
{
	CVI_U8 val = 0;

	switch (eSnsMirrorFlip) {
	case ISP_SNS_NORMAL:
		break;
	case ISP_SNS_MIRROR:
		val |= 0x6;
		break;
	case ISP_SNS_FLIP:
		val |= 0x60;
		break;
	case ISP_SNS_MIRROR_FLIP:
		val |= 0x66;
		break;
	default:
		return;
	}

	sc3336_1l_write_register(ViPipe, 0x3221, val);
}

int sc3336_1l_probe(VI_PIPE ViPipe)
{
	int nVal;
	CVI_U16 chip_id;

	delay_ms(4);
	if (sc3336_1l_i2c_init(ViPipe) != CVI_SUCCESS)
		return CVI_FAILURE;

	nVal = sc3336_1l_read_register(ViPipe, SC3336_1L_CHIP_ID_HI_ADDR);
	if (nVal < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "read sensor id error.\n");
		return nVal;
	}
	chip_id = (nVal & 0xFF) << 8;
	nVal = sc3336_1l_read_register(ViPipe, SC3336_1L_CHIP_ID_LO_ADDR);
	if (nVal < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "read sensor id error.\n");
		return nVal;
	}
	chip_id |= (nVal & 0xFF);

	if (chip_id != SC3336_1L_CHIP_ID) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Sensor ID Mismatch! Use the wrong sensor??\n");
		return CVI_FAILURE;
	}

	return CVI_SUCCESS;
}

void sc3336_1l_init(VI_PIPE ViPipe)
{
	sc3336_1l_i2c_init(ViPipe);

	//linear mode only
	sc3336_1l_linear_1296P20_init(ViPipe);

	g_pastSC3336_1L[ViPipe]->bInit = CVI_TRUE;
}

void sc3336_1l_exit(VI_PIPE ViPipe)
{
	sc3336_1l_i2c_exit(ViPipe);
}

/* 1296P20 and 1296P25 */
static void sc3336_1l_linear_1296P20_init(VI_PIPE ViPipe)
{
	sc3336_1l_write_register(ViPipe, 0x0103, 0x01);
	sc3336_1l_write_register(ViPipe, 0x36e9, 0x80);
	sc3336_1l_write_register(ViPipe, 0x37f9, 0x80);
	sc3336_1l_write_register(ViPipe, 0x3018, 0x1a);
	sc3336_1l_write_register(ViPipe, 0x3019, 0x0e);
	sc3336_1l_write_register(ViPipe, 0x301f, 0x14);
	sc3336_1l_write_register(ViPipe, 0x30b8, 0x33);
	sc3336_1l_write_register(ViPipe, 0x3253, 0x10);
	sc3336_1l_write_register(ViPipe, 0x325f, 0x20);
	sc3336_1l_write_register(ViPipe, 0x3301, 0x04);
	sc3336_1l_write_register(ViPipe, 0x3306, 0x50);
	sc3336_1l_write_register(ViPipe, 0x3309, 0xa8);
	sc3336_1l_write_register(ViPipe, 0x330a, 0x00);
	sc3336_1l_write_register(ViPipe, 0x330b, 0xd8);
	sc3336_1l_write_register(ViPipe, 0x3314, 0x13);
	sc3336_1l_write_register(ViPipe, 0x331f, 0x99);
	sc3336_1l_write_register(ViPipe, 0x3333, 0x10);
	sc3336_1l_write_register(ViPipe, 0x3334, 0x40);
	sc3336_1l_write_register(ViPipe, 0x335e, 0x06);
	sc3336_1l_write_register(ViPipe, 0x335f, 0x0a);
	sc3336_1l_write_register(ViPipe, 0x3364, 0x5e);
	sc3336_1l_write_register(ViPipe, 0x337c, 0x02);
	sc3336_1l_write_register(ViPipe, 0x337d, 0x0e);
	sc3336_1l_write_register(ViPipe, 0x3390, 0x01);
	sc3336_1l_write_register(ViPipe, 0x3391, 0x03);
	sc3336_1l_write_register(ViPipe, 0x3392, 0x07);
	sc3336_1l_write_register(ViPipe, 0x3393, 0x04);
	sc3336_1l_write_register(ViPipe, 0x3394, 0x04);
	sc3336_1l_write_register(ViPipe, 0x3395, 0x04);
	sc3336_1l_write_register(ViPipe, 0x3396, 0x08);
	sc3336_1l_write_register(ViPipe, 0x3397, 0x0b);
	sc3336_1l_write_register(ViPipe, 0x3398, 0x1f);
	sc3336_1l_write_register(ViPipe, 0x3399, 0x04);
	sc3336_1l_write_register(ViPipe, 0x339a, 0x0a);
	sc3336_1l_write_register(ViPipe, 0x339b, 0x3a);
	sc3336_1l_write_register(ViPipe, 0x339c, 0xa0);
	sc3336_1l_write_register(ViPipe, 0x33a2, 0x04);
	sc3336_1l_write_register(ViPipe, 0x33ac, 0x08);
	sc3336_1l_write_register(ViPipe, 0x33ad, 0x1c);
	sc3336_1l_write_register(ViPipe, 0x33ae, 0x10);
	sc3336_1l_write_register(ViPipe, 0x33af, 0x30);
	sc3336_1l_write_register(ViPipe, 0x33b1, 0x80);
	sc3336_1l_write_register(ViPipe, 0x33b3, 0x48);
	sc3336_1l_write_register(ViPipe, 0x33f9, 0x60);
	sc3336_1l_write_register(ViPipe, 0x33fb, 0x74);
	sc3336_1l_write_register(ViPipe, 0x33fc, 0x4b);
	sc3336_1l_write_register(ViPipe, 0x33fd, 0x5f);
	sc3336_1l_write_register(ViPipe, 0x349f, 0x03);
	sc3336_1l_write_register(ViPipe, 0x34a6, 0x4b);
	sc3336_1l_write_register(ViPipe, 0x34a7, 0x5f);
	sc3336_1l_write_register(ViPipe, 0x34a8, 0x20);
	sc3336_1l_write_register(ViPipe, 0x34a9, 0x18);
	sc3336_1l_write_register(ViPipe, 0x34ab, 0xe8);
	sc3336_1l_write_register(ViPipe, 0x34ac, 0x01);
	sc3336_1l_write_register(ViPipe, 0x34ad, 0x00);
	sc3336_1l_write_register(ViPipe, 0x34f8, 0x5f);
	sc3336_1l_write_register(ViPipe, 0x34f9, 0x18);
	sc3336_1l_write_register(ViPipe, 0x3630, 0xc0);
	sc3336_1l_write_register(ViPipe, 0x3631, 0x84);
	sc3336_1l_write_register(ViPipe, 0x3632, 0x64);
	sc3336_1l_write_register(ViPipe, 0x3633, 0x32);
	sc3336_1l_write_register(ViPipe, 0x363b, 0x03);
	sc3336_1l_write_register(ViPipe, 0x363c, 0x08);
	sc3336_1l_write_register(ViPipe, 0x3641, 0x38);
	sc3336_1l_write_register(ViPipe, 0x3670, 0x4e);
	sc3336_1l_write_register(ViPipe, 0x3674, 0xc0);
	sc3336_1l_write_register(ViPipe, 0x3675, 0xc0);
	sc3336_1l_write_register(ViPipe, 0x3676, 0xc0);
	sc3336_1l_write_register(ViPipe, 0x3677, 0x86);
	sc3336_1l_write_register(ViPipe, 0x3678, 0x86);
	sc3336_1l_write_register(ViPipe, 0x3679, 0x86);
	sc3336_1l_write_register(ViPipe, 0x367c, 0x48);
	sc3336_1l_write_register(ViPipe, 0x367d, 0x49);
	sc3336_1l_write_register(ViPipe, 0x367e, 0x4b);
	sc3336_1l_write_register(ViPipe, 0x367f, 0x5f);
	sc3336_1l_write_register(ViPipe, 0x3690, 0x32);
	sc3336_1l_write_register(ViPipe, 0x3691, 0x32);
	sc3336_1l_write_register(ViPipe, 0x3692, 0x42);
	sc3336_1l_write_register(ViPipe, 0x369c, 0x4b);
	sc3336_1l_write_register(ViPipe, 0x369d, 0x5f);
	sc3336_1l_write_register(ViPipe, 0x36b0, 0x87);
	sc3336_1l_write_register(ViPipe, 0x36b1, 0x90);
	sc3336_1l_write_register(ViPipe, 0x36b2, 0xa1);
	sc3336_1l_write_register(ViPipe, 0x36b3, 0xd8);
	sc3336_1l_write_register(ViPipe, 0x36b4, 0x49);
	sc3336_1l_write_register(ViPipe, 0x36b5, 0x4b);
	sc3336_1l_write_register(ViPipe, 0x36b6, 0x4f);
	sc3336_1l_write_register(ViPipe, 0x36ea, 0x0a);
	sc3336_1l_write_register(ViPipe, 0x36eb, 0x0d);
	sc3336_1l_write_register(ViPipe, 0x36ec, 0x0c);
	sc3336_1l_write_register(ViPipe, 0x36ed, 0x26);
	sc3336_1l_write_register(ViPipe, 0x370f, 0x01);
	sc3336_1l_write_register(ViPipe, 0x3722, 0x09);
	sc3336_1l_write_register(ViPipe, 0x3724, 0x41);
	sc3336_1l_write_register(ViPipe, 0x3725, 0xc1);
	sc3336_1l_write_register(ViPipe, 0x3771, 0x09);
	sc3336_1l_write_register(ViPipe, 0x3772, 0x09);
	sc3336_1l_write_register(ViPipe, 0x3773, 0x05);
	sc3336_1l_write_register(ViPipe, 0x377a, 0x48);
	sc3336_1l_write_register(ViPipe, 0x377b, 0x5f);
	sc3336_1l_write_register(ViPipe, 0x37fa, 0x0a);
	sc3336_1l_write_register(ViPipe, 0x37fb, 0x33);
	sc3336_1l_write_register(ViPipe, 0x37fc, 0x11);
	sc3336_1l_write_register(ViPipe, 0x37fd, 0x18);
	sc3336_1l_write_register(ViPipe, 0x3904, 0x04);
	sc3336_1l_write_register(ViPipe, 0x3905, 0x8c);
	sc3336_1l_write_register(ViPipe, 0x391d, 0x04);
	sc3336_1l_write_register(ViPipe, 0x3921, 0x20);
	sc3336_1l_write_register(ViPipe, 0x3926, 0x21);
	sc3336_1l_write_register(ViPipe, 0x3933, 0x80);
	sc3336_1l_write_register(ViPipe, 0x3934, 0x0a);
	sc3336_1l_write_register(ViPipe, 0x3935, 0x00);
	sc3336_1l_write_register(ViPipe, 0x3936, 0x2a);
	sc3336_1l_write_register(ViPipe, 0x3937, 0x6a);
	sc3336_1l_write_register(ViPipe, 0x3938, 0x6a);
	sc3336_1l_write_register(ViPipe, 0x39dc, 0x02);
	sc3336_1l_write_register(ViPipe, 0x3e01, 0x53);
	sc3336_1l_write_register(ViPipe, 0x3e02, 0xe0);
	sc3336_1l_write_register(ViPipe, 0x3e09, 0x00);
	sc3336_1l_write_register(ViPipe, 0x440d, 0x10);
	sc3336_1l_write_register(ViPipe, 0x440e, 0x01);
	sc3336_1l_write_register(ViPipe, 0x4509, 0x20);
	sc3336_1l_write_register(ViPipe, 0x4819, 0x09);
	sc3336_1l_write_register(ViPipe, 0x481b, 0x05);
	sc3336_1l_write_register(ViPipe, 0x481d, 0x12);
	sc3336_1l_write_register(ViPipe, 0x481f, 0x04);
	sc3336_1l_write_register(ViPipe, 0x4821, 0x0a);
	sc3336_1l_write_register(ViPipe, 0x4823, 0x05);
	sc3336_1l_write_register(ViPipe, 0x4825, 0x04);
	sc3336_1l_write_register(ViPipe, 0x4827, 0x04);
	sc3336_1l_write_register(ViPipe, 0x4829, 0x07);
	sc3336_1l_write_register(ViPipe, 0x5ae0, 0xfe);
	sc3336_1l_write_register(ViPipe, 0x5ae1, 0x40);
	sc3336_1l_write_register(ViPipe, 0x5ae2, 0x38);
	sc3336_1l_write_register(ViPipe, 0x5ae3, 0x30);
	sc3336_1l_write_register(ViPipe, 0x5ae4, 0x28);
	sc3336_1l_write_register(ViPipe, 0x5ae5, 0x38);
	sc3336_1l_write_register(ViPipe, 0x5ae6, 0x30);
	sc3336_1l_write_register(ViPipe, 0x5ae7, 0x28);
	sc3336_1l_write_register(ViPipe, 0x5ae8, 0x3f);
	sc3336_1l_write_register(ViPipe, 0x5ae9, 0x34);
	sc3336_1l_write_register(ViPipe, 0x5aea, 0x2c);
	sc3336_1l_write_register(ViPipe, 0x5aeb, 0x3f);
	sc3336_1l_write_register(ViPipe, 0x5aec, 0x34);
	sc3336_1l_write_register(ViPipe, 0x5aed, 0x2c);
	sc3336_1l_write_register(ViPipe, 0x36e9, 0x20);
	sc3336_1l_write_register(ViPipe, 0x37f9, 0x20);

	sc3336_1l_default_reg_init(ViPipe);

	sc3336_1l_write_register(ViPipe, 0x0100, 0x01);

	printf("ViPipe:%d,===SC3336_1L 1296P 20fps 10bit LINE Init OK!===\n", ViPipe);
}
