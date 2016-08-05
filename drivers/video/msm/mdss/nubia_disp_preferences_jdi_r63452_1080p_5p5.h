#ifndef __NUBIA_DISP_PREFERENCES_JDI_R63452_1080P_5P5__
#define __NUBIA_DISP_PREFERENCES_JDI_R63452_1080P_5P5__

struct mdp_pcc_cfg_data jdi_r63452_1080p_5p5_pcc_cfg_warm = {
	.block = 0x10,
	.ops = 0x5,
	{
		.c = 0,
		.r = 0x8000,
		.g = 0,
		.b = 0,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
	{
		.c = 0,
		.r = 0,
		.g = 0x7cff,
		.b = 0,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
	{
		.c = 0,
		.r = 0,
		.g = 0,
		.b = 0x7c7c,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
};

struct mdp_pcc_cfg_data jdi_r63452_1080p_5p5_pcc_cfg_natural = {
	.block = 0x10,
	.ops = 0x5,
	{
		.c = 0,
		.r = 0x8000,
		.g = 0,
		.b = 0,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
	{
		.c = 0,
		.r = 0,
		.g = 0x8000,
		.b = 0,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
	{
		.c = 0,
		.r = 0,
		.g = 0,
		.b = 0x8000,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
};

struct mdp_pcc_cfg_data jdi_r63452_1080p_5p5_pcc_cfg_cool = {
	.block = 0x10,
	.ops = 0x5,
	{
		.c = 0,
		.r = 0x7d80,
		.g = 0,
		.b = 0,
	},
	{
		.c = 0,
		.r = 0,
		.g = 0x7d76,
		.b = 0,
	},
	{
		.c = 0,
		.r = 0,
		.g = 0,
		.b = 0x8000,
	},
};

static char r63452_B000[] = {0xB0, 0x00};
static char r63452_D601[] = {0xD6, 0x01};
static char r63452_CA_STD[] = {
	0xCA, 0x00, 0xFC, 0xFC, 0xFC, 0x00, 0xD1, 0xD1,
	0xA8, 0x00, 0xA8, 0xCE, 0xCE, 0x00, 0x9C, 0x00,
	0x00, 0x00, 0xBA, 0x00, 0x00, 0x00, 0x9C, 0xFF,
	0x9F, 0x83, 0xFF, 0x9F, 0x83, 0x7D, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};

static char r63452_CA_SOFT[] = {
	0xCA, 0x1D, 0xFC, 0xFC, 0xFC, 0x00, 0xD1, 0xD1,
	0xA8, 0x00, 0xA8, 0xCE, 0xCE, 0x00, 0x9C, 0x00,
	0x00, 0x00, 0xBA, 0x00, 0x00, 0x00, 0x9C, 0xFF,
	0x9F, 0x83, 0xFF, 0x9F, 0x83, 0x7D, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};

static char r63452_CA_GLOW[] = {
	0xCA, 0x3D, 0xFC, 0xFC, 0xFC, 0x00, 0xD5, 0xD5,
	0xAA, 0x00, 0xAA, 0xD1, 0xD1, 0x00, 0x99, 0x00,
	0x00, 0x00, 0xB4, 0x00, 0x00, 0x00, 0x99, 0xFF,
	0x9F, 0x83, 0xFF, 0x9F, 0x83, 0x7D, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};

static struct dsi_cmd_desc jdi_r63452_1080p_5p5_glow[] = {
	{{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(r63452_B000)}, r63452_B000},
	{{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(r63452_D601)}, r63452_D601},
	{{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(r63452_CA_GLOW)}, r63452_CA_GLOW},
};
static struct dsi_cmd_desc jdi_r63452_1080p_5p5_soft[] = {
	{{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(r63452_B000)}, r63452_B000},
	{{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(r63452_D601)}, r63452_D601},
	{{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(r63452_CA_SOFT)}, r63452_CA_SOFT},
};
static struct dsi_cmd_desc jdi_r63452_1080p_5p5_standard[] = {
	{{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(r63452_B000)}, r63452_B000},
	{{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(r63452_D601)}, r63452_D601},
	{{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(r63452_CA_STD)}, r63452_CA_STD},
};

#endif
