#ifndef __TC358743_CMOS_PARAM_H_
#define __TC358743_CMOS_PARAM_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#ifdef ARCH_CV182X
#include <linux/cvi_vip_cif.h>
#include <linux/cvi_vip_snsr.h>
#include "cvi_type.h"
#else
#include <linux/cif_uapi.h>
#include <linux/vi_snsr.h>
#include <linux/cvi_type.h>
#endif
#include "cvi_sns_ctrl.h"
#include "tc358743_cmos_ex.h"

// not real time resolution
#define WIDTH   1920
#define HEIGHT  1080

static TC358743_MODE_S g_astTC358743_mode[TC358743_MODE_NUM] = {
	[TC358743_MODE_NORMAL] = {
		.name = "tc358743",
		.astImg[0] = {
			.stSnsSize = {
				.u32Width = WIDTH,
				.u32Height = HEIGHT,
			},
			.stWndRect = {
				.s32X = 0,
				.s32Y = 0,
				.u32Width = WIDTH,
				.u32Height = HEIGHT,
			},
			.stMaxSize = {
				.u32Width = WIDTH,
				.u32Height = HEIGHT,
			},
		},
	},
};

struct combo_dev_attr_s tc358743_rx_attr = {
	.input_mode = INPUT_MODE_MIPI,
	.mac_clk = RX_MAC_CLK_400M,
	.mipi_attr = {
		.raw_data_type = YUV422_8BIT,
		.lane_id = {5, 3, 4, -1, -1},
		.pn_swap = {0, 0, 0, 0, 0},
		.wdr_mode = CVI_MIPI_WDR_MODE_NONE,
		.dphy = {
			.enable = 1,
			.hs_settle = 8,
		},
	},
	.mclk = {
		.cam = 0,
		.freq = CAMPLL_FREQ_24M,
	},
	.devno = 0,
};

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */


#endif /* __TC358743_CMOS_PARAM_H_ */