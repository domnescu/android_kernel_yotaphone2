/*
 *-----------------------------------------------------------------------------
 * linux/include/video/s1d13522fb.h
 *
 * This is the function header for the Epson S1D13522 linux frame buffer driver.
 *
 * Copyright(C) SEIKO EPSON CORPORATION 2011-2013. All rights reserved.
 *
 * This driver software is distributed as is, without any warranty of any kind,
 * either express or implied as further specified in the GNU Public License. This
 * software may be used and distributed according to the terms of the GNU Public
 * License, version 2. See the file COPYING in the main directory of this archive
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *----------------------------------------------------------------------------
 *
 * NOTE: All chip-specific features, such as hardware information needed
 *       to generate proper timings, color depth, screen resolution, etc.,
 *       are placed in chip-specific header files and C modules.
 *
 *-----------------------------------------------------------------------------
 */

#ifndef __S1D13522FB_H__
#define __S1D13522FB_H__

#ifndef FALSE
#define FALSE	(0)
#endif

#ifndef TRUE
#define TRUE	(!FALSE)
#endif

/* S1D13522 command defines */
#define S1D13522_CMD_INIT_CMD_SET			0x00
#define S1D13522_CMD_INIT_PLL				0x01
#define S1D13522_CMD_RUN_SYS				0x02
#define S1D13522_CMD_STBY				0x04
#define S1D13522_CMD_SLP				0x05
#define S1D13522_CMD_INIT_SYS_RUN			0x06
#define S1D13522_CMD_INIT_DSPE_CFG			0x09
#define S1D13522_CMD_INIT_DSPE_TMG			0x0A
#define S1D13522_CMD_INIT_ROTMODE			0x0B
#define S1D13522_CMD_INIT_WAVE_DEV			0x0C
#define S1D13522_CMD_INIT_DSPE_TMG_ADV			0x0D
#define S1D13522_CMD_RD_REG				0x10
#define S1D13522_CMD_WR_REG				0x11
#define S1D13522_CMD_PIP_DISABLE			0x14
#define S1D13522_CMD_PIP_ENABLE				0x15
#define S1D13522_CMD_PIP_ADRCFG				0x16
#define S1D13522_CMD_PIP_XYSETUP			0x17
#define S1D13522_CMD_CSR_MAINCFG			0x18
#define S1D13522_CMD_CSR_XYSETUP			0x19
#define S1D13522_CMD_CSR_ADRCFG				0x1A
#define S1D13522_CMD_BST_RD_SDR				0x1C
#define S1D13522_CMD_BST_WR_SDR				0x1D
#define S1D13522_CMD_BST_END_SDR			0x1E
#define S1D13522_CMD_LD_IMG				0x20
#define S1D13522_CMD_LD_IMG_AREA			0x22
#define S1D13522_CMD_LD_IMG_END				0x23
#define S1D13522_CMD_LD_IMG_SETADR			0x25
#define S1D13522_CMD_LD_IMG_DSPEADR			0x26
#define S1D13522_CMD_WAIT_DSPE_TRG			0x28
#define S1D13522_CMD_WAIT_DSPE_FREND			0x29
#define S1D13522_CMD_WAIT_DSPE_LUTFREE			0x2A
#define S1D13522_CMD_WAIT_DSPE_MLUTFREE			0x2B
#define S1D13522_CMD_RD_WFM_INFO			0x30
#define S1D13522_CMD_UPD_INIT				0x32
#define S1D13522_CMD_UPD_FULL				0x33
#define S1D13522_CMD_UPD_FULL_AREA			0x34
#define S1D13522_CMD_UPD_PART				0x35
#define S1D13522_CMD_UPD_PART_AREA			0x36
#define S1D13522_CMD_UPD_GDRV_CLR			0x37
#define S1D13522_CMD_UPD_SET_IMGADR			0x38
#define S1D13522_CMD_PEN_DRAW				0x3A
#define S1D13522_CMD_PEN_MENU				0x3B
#define S1D13522_CMD_PEN_LINE				0x3C


/* S1D13522 register interface defines */

/* Revision Code register */
#define S1D13522_REG0000_REV_CODE			0x0000
/* Product Code register */
#define S1D13522_REG0002_PROD_CODE			0x0002
/* Power Save Mode register */
#define S1D13522_REG0006_POWER_SAVE_MODE		0x0006
/* Software Reset register [Reserved!!!] */
#define S1D13522_REG0008_SOFTWARE_RESET			0x0008
/* System Status register */
#define S1D13522_REG000A_SYSTEM_STATUS			0x000A
/* Pll Configuration register 0 */
#define S1D13522_REG0010_PLL_CONFIG0			0x0010
/* Pll Configuration register 1 */
#define S1D13522_REG0012_PLL_CONFIG1			0x0012
/* Pll Configuration register 2 */
#define S1D13522_REG0014_PLL_CONFIG2			0x0014
/* Clock Configuration register */
#define S1D13522_REG0016_CLOCK_CONFIG			0x0016
/* I2C Clock Configuration register */
#define S1D13522_REG001A_I2C_CLOCK_CONFIG		0x001A
/* Host Memory Access Configuration and Status Register */
#define S1D13522_REG0140_MEM_ACCESS_CONFIG		0x0140
/* SPI Read Data Register */
#define S1D13522_REG0200_SPI_READ			0x0200
/* SPI Write Data Register */
#define S1D13522_REG0202_SPI_WRITE			0x0202
/* SPI Control Register */
#define S1D13522_REG0204_SPI_CONTROL			0x0204
/* SPI Status Register */
#define S1D13522_REG0206_SPI_STATUS			0x0206
/* SPI Chip Select Control Register */
#define S1D13522_REG0208_SPI_CS_SELECT			0x0208
/* I2C Command Register */
#define S1D13522_REG021A_I2C_COMMAND			0x021A
/* I2C Read Data Register */
#define S1D13522_REG021C_I2C_READ_DATA			0x021C
/* I2C Write Data Register */
#define S1D13522_REG021E_I2C_WRITE_DATA			0x021E
/* GPIO Configuration Register */
#define S1D13522_REG0250_GPIO_CONFIG			0x0250
/* GPIO Status/Control register */
#define S1D13522_REG0252_GPIO_CONTROL			0x0252
/* Image Buffer Start Adress register 0 */
#define S1D13522_REG0310_IMG_BUFFER_SA0			0x0310
/* Image Buffer Start Adress register 1 */
#define S1D13522_REG0312_IMG_BUFFER_SA1			0x0312
/* Temperature Device Select register */
#define S1D13522_REG0320_TEMPERATURE_DEVICE		0x0320
/* Temperature Value Register */
#define S1D13522_REG0322_TEMPERATURE_VALUE		0x0322
/* Power Control Configuration register */
#define S1D13522_REG032A_POWER_CONTROL_CONFIG		0x032A
/* General Configuration Register */
#define S1D13522_REG032C_GENERAL_CONFIG			0x032C
/* Panel Update Buffer Configuration register */
#define S1D13522_REG0330_UPDATE_BUFFER_CONFIG		0x0330
/* Display Engine Control/Trigger register */
#define S1D13522_REG0334_CONTROL_TRIGGER		0x0334
/* Display Engine Busy Status register */
#define S1D13522_REG0338_DISPLAY_STATUS			0x0338

enum wf_mode {
	WF_MODE_INIT	= 0,
	WF_MODE_DU	= 1,
	WF_MODE_GC16	= 2,
	WF_MODE_GL16	= 3,
	WF_MODE_GLR16	= 4,
	WF_MODE_GLD16	= 5,
	WF_MODE_A2	= 6,
	WF_MODE_DU4	= 7,
	WF_MODE_AUTO	= 8
};

enum upd_type {
	UPD_INIT	= 0x32,
	UPD_FULL	= 0x33,
	UPD_FULL_AREA	= 0x34,
	UPD_PART	= 0x35,
	UPD_PART_AREA	= 0x36,
	UPD_GDRV_CLR	= 0x37
};

static inline char *waveform_to_str(enum wf_mode waveform)
{
	switch (waveform) {
	case WF_MODE_INIT:
		return "INIT";
	case WF_MODE_DU:
		return "DU";
	case WF_MODE_GC16:
		return "GC16";
	case WF_MODE_GL16:
		return "GL16";
	case WF_MODE_GLR16:
		return "GLR16";
	case WF_MODE_GLD16:
		return "GLD16";
	case WF_MODE_A2:
		return "A2";
	case WF_MODE_DU4:
		return "DU4";
	case WF_MODE_AUTO:
		return "AUTO";
	default:
		return "unknown";
	}
}

static inline char *mode_to_str(enum upd_type mode)
{
	switch (mode) {
	case UPD_INIT:
		return "INIT";
	case UPD_FULL:
		return "FULL";
	case UPD_FULL_AREA:
		return "FULL_AREA";
	case UPD_PART:
		return "PART";
	case UPD_PART_AREA:
		return "PART_AREA";
	case UPD_GDRV_CLR:
		return "GDRV_CLR";
	default:
		return "unknown";
	}
}

struct s1d13522fb_status;

#define TEMP_SCALE_INTERVAL	10
#define TEMP_POINTS		5
#define TEMP_UPD_INTERVAL	5 // (min)

#define TEMP_MAX		((TEMP_SCALE_INTERVAL * TEMP_POINTS) - 6)
#define CHECK_TEMP(x)		((x < 0) ? 0 : ((x > TEMP_MAX) ? TEMP_MAX : x))
#define TEMP_TO_INDEX(x)	((unsigned)((CHECK_TEMP(x) + (TEMP_SCALE_INTERVAL/2)) / TEMP_SCALE_INTERVAL))

struct s1d13522fb_stat {
	/* update counters */
	atomic_t bw_upd[TEMP_POINTS];
	atomic_t gl_full_upd[TEMP_POINTS];
	atomic_t gl_part_upd[TEMP_POINTS];
	atomic_t region_upd[TEMP_POINTS];
};

static inline void s1d13522fb_statistic_reset(struct s1d13522fb_stat *stat)
{
	int i;

	for (i = 0; i < TEMP_POINTS; i++) {
		atomic_set(&stat->bw_upd[i], 0);
		atomic_set(&stat->gl_full_upd[i], 0);
		atomic_set(&stat->gl_part_upd[i], 0);
		atomic_set(&stat->region_upd[i], 0);
	}
}

#define s1d13522fb_statistic_inc(par, name)	\
	atomic_inc(&par->statistic.name[TEMP_TO_INDEX(par->status->curent_temp)]);

/* struct used by S1D13522 */
struct s1d13522fb_par {
	struct fb_info *info;

	wait_queue_head_t waitq;
	void (*write_reg)(struct s1d13522fb_par *, u16, u16);
	u16  (*read_reg)(struct s1d13522fb_par *, u16);
	int (*set_temp)(struct s1d13522fb_par *, int);

	struct s1d13522fb_board *board;
	struct s1d13522fb_status *status;
	struct s1d13522fb_stat statistic;

	void (*power_acquire)(struct s1d13522fb_status *);
	void (*power_release)(struct s1d13522fb_status *);
	struct delayed_work temp_worker;

	/* gray splash timeout */
	atomic_t splash_timeout;
	struct delayed_work splash_worker;

	/* io_lock protect: io sequence, epd buffer */
	struct mutex io_lock;
	unsigned char dthr_num;
	unsigned char draw_off;
	void *epd_buffer;
	enum wf_mode waveform;
	enum upd_type update_mode;
};

/* common function prototypes */
void s1d13522fb_screen_update(struct s1d13522fb_par *par, u16 dl_par[5]);
int s1d13522fb_register_sysfs(struct fb_info *info);
void s1d13522fb_unregister_sysfs(struct fb_info *info);


/* board specific routines */
struct s1d13522fb_board {
	struct s1d13522fb_par *fb_par;
	struct module *owner;
	int  (*get_panel_config) (const char *);
	int  (*init)(struct s1d13522fb_par *);
	int  (*wait_for_rdy)(struct s1d13522fb_par *, int);
	void (*cleanup)(struct s1d13522fb_par *);
	int  (*send_command)(struct s1d13522fb_par *, u16);
	int  (*send_cmdargs)(struct s1d13522fb_par *, u16, int, u16 *);
	void (*burst_read)(struct s1d13522fb_par *, int, u16*);
	void (*burst_write)(struct s1d13522fb_par *, int, u16*);
	void (*write_data)(struct s1d13522fb_par *, u16);
	u16  (*read_data)(struct s1d13522fb_par *);
	int  (*setup_irq)(struct fb_info *);
	int  (*get_temp)(struct s1d13522fb_par *, int *);
};

#endif	/* __S1D13522FB_H__ */
