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
#include "tc358743_cmos_ex.h"

#define TC358743_I2C_DEV 2
#define TC358743_I2C_BANK_ADDR 0xff

// https://github.com/jaesc/tc358743_i2c/blob/master/tc358743_regs.h


#define CC_RGB_PASSTHROUGH      1
#define CC_RGB_YUV422           2
#define CC_RGB_YUV444           3
#define CC_YUV444_YUV422        4
#define CC_YUV422_YUV444        5
#define COLOR_CONVERSION CC_RGB_YUV422

#if COLOR_CONVERSION == CC_RGB_PASSTHROUGH
#define r8576  0x00 // 0000 0000 -- RGB full // RGB through mode
#define r8573  0x00 // 00000000 -- RGB through
#define r8574  0x00
#define r0004  0x0e24
#elif COLOR_CONVERSION == CC_RGB_YUV422
#define r8574  0x08
#define r8573  /* 11000001 */ 0xC1
#define r8576  0x60
#define r0004  0x0ee4
#elif COLOR_CONVERSION == CC_RGB_YUV444
#define r8574  0x08
#define r8573  /* 00000001 */ 0x01
#define r8576  0x60
#define r0004  0x0e24
#elif COLOR_CONVERSION == CC_YUV444_YUV422
#define r8574  0x08
#define r8573  /* 00000001 */ 0x80
#define r8576  0x00
#define r0004  0x0ee4
#elif COLOR_CONVERSION == CC_YUV422_YUV444
#define r8574  0x08
#define r8573  /* 00000001 */ 0x00
#define r8576  0x00
#define r0004  0x0e24
#endif

struct cmds_t {
	uint16_t addr;
	uint32_t value;
	uint8_t num_bytes;
};

#define TWOLANES
#ifdef TWOLANES
static unsigned char TOSHH2C_DEFAULT_EDID[] =
    "00ffffffffffff005262888800888888"
    "1c150103800000780aEE91A3544C9926"
    "0F505400000001010101010101010101"
    "010101010101011d007251d01e206e28"
    "5500c48e2100001e8c0ad08a20e02d10"
    "103e9600138e2100001e000000"
                              "fc0054"   //FC, 00, Device name ("Toshiba-H2C")
    "6f73686962612d4832430a20000000"
                                  "FD"
    "003b3d0f2e0f1e0a2020202020200100"

    "0203"                                //CEA EDID, V3
        "20"                              //offset to DTDs,
          "42"                            //num of native DTDs | 0x40 (basic audio support)
            "4d841303021211012021223c"    //(Length|0x40), then list of VICs.
    "3d3e"
        "23090707"                        //(Length|0x20), then audio data block
                "66030c00300080"          //(Length|0x60), then vendor specific block
                              "E3007F"    //?? Reserved DCB type 7, length 3
    "8c0ad08a20e02d10103e9600c48e2100"    //DTD #1
    "0018"
        "8c0ad08a20e02d10103e9600138e"    //DTD #2
    "21000018"
            "8c0aa01451f01600267c4300"    //DTD #3
    "138e21000098"
                "00000000000000000000"    //End. 0 padded.
    "00000000000000000000000000000000"
    "00000000000000000000000000000000";

#else
//4 LANES
static unsigned char TOSHH2C_DEFAULT_EDID[] =
    "00ffffffffffff005262888800888888"
    "1c150103800000780aEE91A3544C9926"
    "0F505400000001010101010101010101"
    "010101010101011d007251d01e206e28"
    "5500c48e2100001e8c0ad08a20e02d10"
    "103e9600138e2100001e000000"
                              "fc0054"   //FC, 00, Device name ("Toshiba-H2C")
    "6f73686962612d4832430a20000000"
                                  "FD"
    "003b3d0f2e0f1e0a2020202020200100"

    "0203"                                //CEA EDID, V3
        "22"                              //offset to DTDs,
          "42"                            //num of native DTDs | 0x40 (basic audio support)
            "4f841303021211012021223c"    //(Length|0x40), then list of VICs.
    "3d3e101f"
            "2309070766030c00300080"      //Audio. 2-chan LPCM, 32/44/48kHz, 16/20/24 bit.
                                  "E3"    //?? Reserved DCB type 7, length 3
    "007F"
        "8c0ad08a20e02d10103e9600c48e"    //DTD #1
    "21000018"
            "8c0ad08a20e02d10103e9600"    //DTD #2
    "138e21000018"
                "8c0aa01451f01600267c"    //DTD #3
    "4300138e21000098"
                    "0000000000000000"    //End. 0 padded.
    "00000000000000000000000000000000"
    "00000000000000000000000000000000";

#endif

CVI_U8 tc358743_i2c_addr = 0xf;
const CVI_U32 tc358743_addr_byte = 2;
const CVI_U32 tc358743_data_byte = 1;


static int g_fd[VI_MAX_PIPE_NUM] = {[0 ... (VI_MAX_PIPE_NUM - 1)] = -1};

void start_hdmi_streaming(VI_PIPE ViPipe);
void stop_hdmi_streaming(VI_PIPE ViPipe);

int tc358743_i2c_init(VI_PIPE ViPipe)
{
	char acDevFile[16] = {0};
	// CVI_U8 u8DevNum;

	if (g_fd[ViPipe] >= 0)
		return CVI_SUCCESS;
	int ret;

	// u8DevNum = g_aunTC358743_BusInfo[ViPipe].s8I2cDev;
	snprintf(acDevFile, sizeof(acDevFile),  "/dev/i2c-%u", TC358743_I2C_DEV);

	g_fd[ViPipe] = open(acDevFile, O_RDWR, 0600);

	if (g_fd[ViPipe] < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Open /dev/i2c-%u error!\n", TC358743_I2C_DEV);
		return CVI_FAILURE;
	}

	ret = ioctl(g_fd[ViPipe], I2C_SLAVE_FORCE, tc358743_i2c_addr);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_SLAVE_FORCE error!\n");
		close(g_fd[ViPipe]);
		g_fd[ViPipe] = -1;
		return ret;
	}

	return CVI_SUCCESS;
}

int tc358743_i2c_exit(VI_PIPE ViPipe)
{
	if (g_fd[ViPipe] >= 0) {
		close(g_fd[ViPipe]);
		g_fd[ViPipe] = -1;
		return CVI_SUCCESS;
	}
	return CVI_FAILURE;
}

// static void delay_ms(int ms)
// {
// 	usleep(ms * 1000);
// }


int tc358743_read_register(VI_PIPE ViPipe, int addr)
{
	int ret, data;
	char buf[8];
	int idx = 0;

	if (g_fd[ViPipe] < 0)
		return CVI_FAILURE;

	if (tc358743_addr_byte == 2)
		buf[idx++] = (addr >> 8) & 0xff;

	// add address byte 0
	buf[idx++] = addr & 0xff;

	ret = write(g_fd[ViPipe], buf, tc358743_addr_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_WRITE error!\n");
		return ret;
	}

	buf[0] = 0;
	buf[1] = 0;
	ret = read(g_fd[ViPipe], buf, tc358743_data_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_READ error!\n");
		return ret;
	}

	// pack read back data
	data = 0;
	if (tc358743_data_byte == 2) {
		data = buf[0] << 8;
		data += buf[1];
	} else {
		data = buf[0];
	}

	syslog(LOG_DEBUG, "vipipe:%d i2c r 0x%x = 0x%x\n", ViPipe, addr, data);
	printf("vipipe:%d i2c r 0x%x = 0x%x\n", ViPipe, addr, data);
	return data;
}

int tc358743_write_register(VI_PIPE ViPipe, int addr, int data)
{
	int idx = 0;
	int ret;
	char buf[8];

	if (g_fd[ViPipe] < 0)
		return CVI_SUCCESS;

	if (tc358743_addr_byte == 2) {
		buf[idx] = (addr >> 8) & 0xff;
		idx++;
	}
	buf[idx] = addr & 0xff;
	idx++;


	if (tc358743_data_byte == 2) {
		buf[idx] = (data >> 8) & 0xff;
		idx++;
	}
	buf[idx] = data & 0xff;
	idx++;

	ret = write(g_fd[ViPipe], buf, tc358743_addr_byte + tc358743_data_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_WRITE error!\n");
		return CVI_FAILURE;
	}

	syslog(LOG_DEBUG, "ViPipe:%d i2c w 0x%x 0x%x\n", ViPipe, addr, data);
	return CVI_SUCCESS;
}

int tc358743_write_bytes(VI_PIPE ViPipe, uint16_t addr, uint32_t data, uint8_t len)
{
	int idx = 0;
	int ret;
	uint8_t buf[8];

	if (g_fd[ViPipe] < 0)
		return CVI_SUCCESS;

	if (tc358743_addr_byte == 2) {
		buf[idx] = (addr >> 8) & 0xff;
		idx++;
	}
	buf[idx] = addr & 0xff;
	idx++;


	for(int i =0; i < len; i++)
	{
		buf[idx] = (data >> (i*8) ) & 0xff;
		idx++;
	}

	// if (tc358743_data_byte == 2) {
	// 	buf[idx] = (data >> 8) & 0xff;
	// 	idx++;
	// }
	// buf[idx] = data & 0xff;
	// idx++;

	ret = write(g_fd[ViPipe], buf, tc358743_addr_byte + len);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_WRITE error!\n");
		return CVI_FAILURE;
	}

	// syslog(LOG_DEBUG, "ViPipe:%d i2c w 0x%x 0x%x\n", ViPipe, addr, data);
	return CVI_SUCCESS;
}

void tc358743_default_reg_init(VI_PIPE ViPipe)
{
	CVI_U32 i;

	for (i = 0; i < g_pastTC358743[ViPipe]->astSyncInfo[0].snsCfg.u32RegNum; i++) {
		tc358743_write_register(ViPipe,
				g_pastTC358743[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32RegAddr,
				g_pastTC358743[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32Data);
	}
}

void tc358743_standby(VI_PIPE ViPipe)
{
	(void) ViPipe;
	CVI_TRACE_SNS(CVI_DBG_NOTICE, "unsupport standby.\n");
}

void tc358743_restart(VI_PIPE ViPipe)
{
	(void) ViPipe;
	CVI_TRACE_SNS(CVI_DBG_NOTICE, "unsupport restart.\n");
}

#define TC358743_CHIP_ID_ADDR	0x0000
#define TC358743_CHIP_ID    0x0000

int  tc358743_probe(VI_PIPE ViPipe)
{
	int nVal;

	usleep(50);
	if (tc358743_i2c_init(ViPipe) != CVI_SUCCESS)
		return CVI_FAILURE;

	nVal  = tc358743_read_register(ViPipe, TC358743_CHIP_ID_ADDR);
	if (nVal < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "read sensor id error.\n");
		return nVal;
	}
	printf("update data:%04x\n", nVal);
	// if ((((nVal & 0xFF) << 8) | (nVal & 0xFF)) != TC358743_CHIP_ID) {
	// 	CVI_TRACE_SNS(CVI_DBG_ERR, "Sensor ID Mismatch! Use the wrong sensor??\n");
	// 	return CVI_FAILURE;
	// }

	return CVI_SUCCESS;
}

void tc358743_init(VI_PIPE ViPipe)
{
	tc358743_i2c_init(ViPipe);

	start_hdmi_streaming(ViPipe);

	g_pastTC358743[ViPipe]->bInit = CVI_TRUE;
}

void tc358743_exit(VI_PIPE ViPipe)
{
	stop_hdmi_streaming(ViPipe);

	tc358743_i2c_exit(ViPipe);
}


struct cmds_t cmds2[] = {
	/* HDMI specification requires HPD to be pulsed low for 100ms when EDID changed */
	{0x8544, 0x01, 1},      // DDC5V detection interlock -- disable
	{0x8544, 0x00, 1},      // DDC5V detection interlock -- pulse low

	// {0x0000, 100, 0xFFFF},  // sleep
	{0x8544, 0x10, 1},      // DDC5V detection interlock -- enable

	{0x85D1, 0x01, 1},         // Key loading command
	{0x8560, 0x24, 1},         // KSV Auto Clear Mode
	{0x8563, 0x11, 1},         // EESS_Err auto-unAuth
	{0x8564, 0x0F, 1},      // DI_Err (Data Island Error) auto-unAuth

	// RGB888 to YUV422 conversion (must)
	{0x8574, r8574, 1},
	{0x8573, r8573, 1},        // OUT YUV444[7]
	// 1010 0001
	{0x8576, r8576, 1},        // [7:5] = YCbCr601 Limited ? 3'b011 : 3'b101 (YCbCr 709 Limited)

	{0x8600, 0x00, 1},      // Forced Mute Off, Set Auto Mute On
	{0x8602, 0xF3, 1},      // AUTO Mute (AB_sel, PCM/NLPCM_chg, FS_chg, PX_chg, PX_off, DVI)
	{0x8603, 0x02, 1},         // AUTO Mute (AVMUTE)
	{0x8604, 0x0C, 1},      // AUTO Play (mute-->BufInit-->play)
	{0x8606, 0x05, 1},         // BufInit start time = 0.5sec
	{0x8607, 0x00, 1},         // Disable mute
	{0x8620, 0x00, 1},      // [5] = 0: LPCM/NLPCMinformation extraction from Cbit
	{0x8640, 0x01, 1},         // CTS adjustment=ON
	{0x8641, 0x65, 1},      // Adjustment level 1=1000ppm, Adjustment level 2=2000ppm
	{0x8642, 0x07, 1},         // Adjustment level 3=4000ppm
	{0x8652, 0x02, 1},      // Data Output Format: [6:4] = 0, 16-bit, [1:0] = 2, I2S Format
	{0x8665, 0x10, 1},      // [7:4] 128 Fs Clock divider  Delay 1 * 0.1 s, [0] = 0: 44.1/48 KHz Auto switch setting

	{0x8709, 0xFF, 1},      // ""FF"": Updated secondary Pkts even if there are errors received
	{0x870B, 0x2C, 1},      // [7:4]: ACP packet Intervals before interrupt, [3:0] AVI packet Intervals, [] * 80 ms
	{0x870C, 0x53, 1},      // [6:0]: PKT receive interrupt is detected,storage register automatic clear, video signal with RGB and no repeat are set
	{0x870D, 0x01, 1},      // InFo Pkt: [7]: Correctable Error is included, [6:0] # of Errors before assert interrupt
	{0x870E, 0x30, 1},      // [7:4]: VS packet Intervals before interrupt, [3:0] SPD packet Intervals, [] * 80 ms
	{0x9007, 0x10, 1},      // [5:0]  Auto clear by not receiving 16V GBD
	{0x854A, 0x01, 1},      // HDMIRx Initialization Completed, THIS MUST BE SET AT THE LAST!

};
#define NUM_REGS_CMD2 (sizeof(cmds2)/sizeof(cmds2[0]))

struct cmds_t cmds3[] = {
	{0x0004, r0004 | 0x03, 2},        // Enable tx buffer_size
};
#define NUM_REGS_CMD3 (sizeof(cmds3)/sizeof(cmds3[0]))

#define ENABLE_DATALANE_1 0x0
#define DISABLE_DATALANE_1 0x1


unsigned char ascii_to_hex(unsigned char c)
{
	if(c>='0' && c<='9')
		return (c-'0');
	else if (c>='A' && c<='F')
		return ((c-'A') + 10);
	else if (c>='a' && c<='f')
		return ((c-'a') + 10);
	return 0;
}

static void i2c_wr(VI_PIPE ViPipe, uint16_t reg, uint8_t *values, uint32_t n)
{
	uint8_t data[1024];
	int err, i;
	struct i2c_msg msg;
	struct i2c_rdwr_ioctl_data msgset;

	if ((2 + n) > sizeof(data))
		printf("i2c wr reg=%04x: len=%d is too big!\n",
				reg, 2 + n);

	msg.addr = 0xF;
	msg.buf = data;
	msg.len = 2 + n;
	msg.flags = 0;

	data[0] = reg >> 8;
	data[1] = reg & 0xff;

	for (i = 0; i < (int)n; i++)
		data[2 + i] = values[i];

	msgset.msgs = &msg;
	msgset.nmsgs = 1;

	err = ioctl(g_fd[ViPipe], I2C_RDWR, &msgset);
	if (err != 1) {
		printf("%s: writing register 0x%x from 0xf failed\n",
				__func__, reg);
		return;
	}
}

void start_hdmi_streaming(VI_PIPE ViPipe)
{
	int val;
	int index;
	uint16_t r0006;
	uint8_t r0148;
	uint32_t r0500;

	val = tc358743_read_register(ViPipe, 0x8521) & 0x0F;
	printf("VI_STATUS to select cfg.data_lanes: %u\n", val);

	// if (val < 12)
	// {
	// 	r0006 = 0x0080;
	// 	r0148 = DISABLE_DATALANE_1;
	// 	r0500 = 0xA3008080;
	// 	printf("Selected Sub 720p registers\n");
	// }
	// else
	{
		r0006 = 0x0008;
		r0148 = ENABLE_DATALANE_1;
		r0500 = 0xA3008082;
		printf("Selected 720p+ registers\n");
	}

	struct cmds_t cmds[] = {
		{0x0004, 0x0000, 2}, // Disable video TX Buffer
		// Turn off power and put in reset
		// handle.first_boot = VC_FALSE;
		{0x0002, 0x0F00, 2}, // Assert Reset, [0] = 0: Exit Sleep, wait
		// {0x0000, 1, 0xFFFF}, // V054 requires us to wait 1ms for PLL to lock
		{0x0002, 0x0000, 2},		 // Release Reset, Exit Sleep
		{0x0006, r0006, 2},			 // FIFO level
		{0x0008, 0x005f, 2},		 // Audio buffer level -- 96 bytes = 0x5F + 1
		{0x0014, 0xFFFF, 2},		 // Clear HDMI Rx, CSI Tx and System Interrupt Status
		{0x0016, 0x051f, 2},		 // Enable HDMI-Rx Interrupt (bit 9), Sys interrupt (bit 5). Disable others. 11-15, 6-7 reserved
		{0x0020, 0x8111, 2},		 // PRD[15:12], FBD[8:0]
		{0x0022, 0x0213, 2},		 // FRS[11:10], LBWS[9:8]= 2, Clock Enable[4] = 1,  ResetB[1] = 1,  PLL En[0]
		{0x0004, r0004, 2},			 // PwrIso[15], 422 output, send infoframe
		{0x0140, 0x0, 4},			 // Enable CSI-2 Clock lane
		{0x0144, 0x0, 4},			 // Enable CSI-2 Data lane 0
		{0x0148, r0148, 4},			 // Enable CSI-2 Data lane 1
		{0x014C, 0x1, 4},			 // Disable CSI-2 Data lane 2
		{0x0150, 0x1, 4},			 // Disable CSI-2 Data lane 3
		{0x0210, 0x00002988, 4},	 // LP11 = 100 us for D-PHY Rx Init
		{0x0214, 0x00000005, 4},	 // LP Tx Count[10:0]
		{0x0218, 0x00001d04, 4},	 // TxClk_Zero[15:8]
		{0x021C, 0x00000002, 4},	 // TClk_Trail =
		{0x0220, 0x00000504, 4},	 // HS_Zero[14:8] =
		{0x0224, 0x00004600, 4},	 // TWAKEUP Counter[15:0]
		{0x0228, 0x0000000A, 4},	 // TxCLk_PostCnt[10:0]
		{0x022C, 0x00000004, 4},	 // THS_Trail =
		{0x0234, 0x0000001F, 4},	 // Enable Voltage Regulator for CSI (4 Data + Clk) Lanes
		{0x0204, 0x00000001, 4},	 // Start PPI
		{0x0518, 0x00000001, 4},	 // Start CSI-2 Tx
		{0x0500, r0500, 4},			 // SetBit[31:29]
		{0x8502, 0x01, 1},			 // Enable HPD DDC Power Interrupt
		{0x8512, 0xFE, 1},			 // Disable HPD DDC Power Interrupt Mask
		{0x8513, (uint8_t)~0x20, 1}, // Receive interrupts for video format change (bit 5)
		{0x8515, (uint8_t)~0x02, 1}, // Receive interrupts for format change (bit 1)
		{0x8531, 0x01, 1},			 // [1] = 1: RefClk 42 MHz, [0] = 1, DDC5V Auto detection
		{0x8540, 0x0A8C, 2},		 // SysClk Freq count with RefClk = 27 MHz (0x1068 for 42 MHz, default)
		{0x8630, 0x00041eb0, 4},	 // Audio FS Lock Detect Control [19:0]: 041EB0 for 27 MHz, 0668A0 for 42 MHz (default)
		{0x8670, 0x01, 1},			 // SysClk 27/42 MHz: 00:= 42 MHz
		{0x8532, 0x80, 1},			 // PHY_AUTO_RST[7:4] = 1600 us, PHY_Range_Mode = 12.5 us
		{0x8536, 0x40, 1},			 // [7:4] Ibias: TBD, [3:0] BGR_CNT: Default
		{0x853F, 0x0A, 1},			 // [3:0] = 0x0a: PHY TMDS CLK line squelch level: 50 uA
		{0x8543, 0x32, 1},			 // [5:4] = 2'b11: 5V Comp, [1:0] = 10, DDC 5V active detect delay setting: 100 ms
		{0x8544, 0x10, 1},			 // DDC5V detection interlock -- enable
		{0x8545, 0x31, 1},			 //  [5:4] = 2'b11: Audio PLL charge pump setting to Normal, [0] = 1: DAC/PLL Power On
		{0x8546, 0x2D, 1},			 // [7:0] = 0x2D: AVMUTE automatic clear setting (when in MUTE and no AVMUTE CMD received) 45 * 100 ms
		{0x85C7, 0x01, 1},			 // [6:4] EDID_SPEED: 100 KHz, [1:0] EDID_MODE: Internal EDID-RAM & DDC2B mode
		{0x85CB, 0x01, 1},			 // EDID Data size read from EEPROM EDID_LEN[10:8] = 0x01, 256-Byte
	};

#define NUM_REGS_CMD (sizeof(cmds)/sizeof(cmds[0]))

	for (index = 0; index < (int)NUM_REGS_CMD; index++)
	{
		tc358743_write_bytes(ViPipe, cmds[index].addr, cmds[index].value, cmds[index].num_bytes);
	}

	{
		uint8_t edid[256];
		int i, j;
		unsigned char checksum = 0;
		for (i = 0; i < (int)(sizeof(TOSHH2C_DEFAULT_EDID) / 2); i += 16)
		{
			for (j = 0; j < 16; j++)
			{
				edid[i + j] = (ascii_to_hex(TOSHH2C_DEFAULT_EDID[(i + j) * 2]) << 4) +
							  ascii_to_hex(TOSHH2C_DEFAULT_EDID[(i + j) * 2 + 1]);
				checksum -= edid[i + j];
			}
			// if checksum byte
			if (i == (7 * 16) || i == (15 * 16))
			{
				edid[i + 15] = checksum;
				checksum = 0;
			}
			i2c_wr(ViPipe, 0x8C00 + i, &edid[i], 16);

		}
	}

	for (index = 0; index < (int)NUM_REGS_CMD2; index++)
	{
		tc358743_write_bytes(ViPipe, cmds2[index].addr, cmds2[index].value, cmds2[index].num_bytes);
	}

	for (index = 0; index < (int)NUM_REGS_CMD3; index++)
	{
		tc358743_write_bytes(ViPipe, cmds3[index].addr, cmds3[index].value, cmds3[index].num_bytes);
	}

	tc358743_default_reg_init(ViPipe);

}

void stop_hdmi_streaming(VI_PIPE ViPipe)
{
	// delay_ms(10);
	tc358743_write_register(ViPipe, 0x8544, 0x01);
	tc358743_write_register(ViPipe, 0x8544, 0x00);
}