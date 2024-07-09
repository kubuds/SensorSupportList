#ifndef __TC358743_CMOS_EX_H_
#define __TC358743_CMOS_EX_H_

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

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif


enum tc358743_linear_regs_e {
	LINEAR_REGS_NUM
};


typedef enum _TC358743_MODE_E {
	TC358743_MODE_NONE,
	TC358743_MODE_NORMAL,
	TC358743_MODE_NUM
} TC358743_MODE_E;


typedef struct _TC358743_MODE_S {
	ISP_WDR_SIZE_S astImg[2];
	char name[64];
} TC358743_MODE_S;

extern ISP_SNS_STATE_S *g_pastTC358743[VI_MAX_PIPE_NUM];
extern ISP_SNS_COMMBUS_U g_aunTC358743_BusInfo[];
extern CVI_U8 tc358743_i2c_addr;
extern const CVI_U32 tc358743_addr_byte;
extern const CVI_U32 tc358743_data_byte;
extern void tc358743_init(VI_PIPE ViPipe);
extern void tc358743_exit(VI_PIPE ViPipe);
extern void tc358743_standby(VI_PIPE ViPipe);
extern void tc358743_restart(VI_PIPE ViPipe);
extern int  tc358743_write_register(VI_PIPE ViPipe, int addr, int data);
extern int  tc358743_read_register(VI_PIPE ViPipe, int addr);
extern void tc358743_mirror_flip(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip);
extern int  tc358743_probe(VI_PIPE ViPipe);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */


#endif /* __TC358743_CMOS_EX_H_ */