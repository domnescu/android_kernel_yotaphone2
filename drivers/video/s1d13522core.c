/* drivers/video/s1d13522core.c
 *
 * (c) 2016 Yota Devices
 *
 * Driver framebuffer for Epson S1D13522 e-ink chip
 *
 * Adapted from
 *  linux/drivers/video/broadsheetfb.c
 *  linux/drivers/video/s1d13522fb.c (1.1 driver by Epson)
 *
 * TODO:
 * - add dithering algorithms
 * - add black and white mode
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/mman.h>
#include <linux/uaccess.h>
#include <linux/fb.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/bug.h>
#include <linux/atomic.h>
#include <linux/dithering.h>

#include <video/s1d13522fb.h>
#include <linux/epd_sflash.h>

#ifdef CONFIG_EPD_NOTIFIER
#include <linux/epd_notifier.h>
#endif

static int s1d13522fb_init_panel(struct s1d13522fb_par *par);

union s1d13522fb_addr {
	u32 addr;
	struct {
		u16 addr_l;
		u16 addr_h;
	};
};
typedef union s1d13522fb_addr s1d13522fb_addr_t;

/* describe current framebuffer status */
struct s1d13522fb_status {
	struct s1d13522fb_par *par;
	dthr_helper helper;

	struct rw_semaphore pwr_lock;
	unsigned char is_sleep;
	struct delayed_work pwr_worker;

	int curent_temp;
	atomic_t vsync_en;
	atomic_t gray_wq_skip;
};

static struct fb_var_screeninfo s1d13522fb_var = {
	.activate = FB_ACTIVATE_NOW,
	.bits_per_pixel	= 16,
	.grayscale	= 0,
	.red =		{11, 5, 0 },
	.green =	{ 5, 6, 0 },
	.blue =		{ 0, 5, 0 },
	.transp =	{ 0, 0, 0 },
};

static struct fb_fix_screeninfo s1d13522fb_fix = {
	.id		= "s1d13522fb",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_TRUECOLOR,
	.type_aux	= 0,
	.xpanstep	= 0,
	.ypanstep	= 0,
	.ywrapstep	= 0,
	.accel		= FB_ACCEL_NONE,
};

static u32 s1d13522fb_pseudo_palette[16] = {
	0x00000000, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};

static u16 s1d13522fb_read_reg(struct s1d13522fb_par *par, u16 reg)
{
	par->board->send_cmdargs(par, S1D13522_CMD_RD_REG, 1, &reg);

	return par->board->read_data(par);
}

static void s1d13522fb_write_reg(struct s1d13522fb_par *par, u16 reg,
					u16 data)
{
	par->board->send_cmdargs(par, S1D13522_CMD_WR_REG, 1, &reg);
	par->board->write_data(par, data);
}

static void s1d13522fb_turn_low_power(struct s1d13522fb_par *par)
{
	if (par->board->wait_for_rdy(par, 1000)) {
		dev_err(par->info->dev, "EPD controller is hang - reboot it\n");

		if (par->board->cleanup)
			par->board->cleanup(par);

		if (!par->board->setup_irq || par->board->setup_irq(par->info))
			goto reinit_fail;

		if (!par->board->init || par->board->init(par))
			goto reinit_fail;

		if (s1d13522fb_init_panel(par))
			goto reinit_fail;

		/* after reseting first update should be  FULL on INIT */
		par->update_mode = UPD_FULL;
	}

	par->board->send_command(par, S1D13522_CMD_SLP);
	return;

reinit_fail:
	dev_WARN(par->info->dev, "EPD controller reinitialization failed\n");
}

static inline void s1d13522fb_exit_low_power(struct s1d13522fb_par *par)
{
	u16 tmp;

	tmp = s1d13522fb_read_reg(par, S1D13522_REG000A_SYSTEM_STATUS);
	/* Set OSC Enable Bypass */
	tmp |= BIT(12);
	s1d13522fb_write_reg(par, S1D13522_REG000A_SYSTEM_STATUS, tmp);

	par->board->send_command(par, S1D13522_CMD_RUN_SYS);
}


static void s1d13522fb_power_acquire(struct s1d13522fb_status *status)
{
	struct s1d13522fb_par *par = status->par;

	cancel_delayed_work_sync(&status->pwr_worker);

	/* keep pwr_lock read-locked till power release */
	down_read(&status->pwr_lock);
	if (!status->is_sleep)
		return;

	up_read(&status->pwr_lock);

	/* wake up controller if it sleep */
	down_write(&status->pwr_lock);
	if (!status->is_sleep) {
		downgrade_write(&status->pwr_lock);
		return;
	}

	dev_dbg(par->info->dev, "wakeup epd controller\n");
	mutex_lock(&par->io_lock);
	s1d13522fb_exit_low_power(par);
	mutex_unlock(&par->io_lock);

	status->is_sleep = 0;

	downgrade_write(&status->pwr_lock);
}

static void s1d13522fb_power_work(struct work_struct *work)
{
	struct s1d13522fb_status *status;
	struct s1d13522fb_par *par;

	status = container_of(work, struct s1d13522fb_status, pwr_worker.work);
	par = status->par;

	down_write(&status->pwr_lock);

	if (!status->is_sleep) {
		dev_dbg(par->info->dev, "epd controller going sleep\n");
		mutex_lock(&par->io_lock);
		s1d13522fb_turn_low_power(par);
		mutex_unlock(&par->io_lock);
		status->is_sleep = 1;
	}

	up_write(&status->pwr_lock);
}

static void s1d13522fb_power_release(struct s1d13522fb_status *status)
{
	up_read(&status->pwr_lock);
	schedule_delayed_work(&status->pwr_worker, HZ * 5);
}

static void s1d13522fb_splash_work(struct work_struct *work)
{
	struct s1d13522fb_par *par = container_of(work, struct s1d13522fb_par,
							splash_worker.work);
	struct s1d13522fb_status *status = par->status;
	struct fb_info *info = par->info;
	dthr_prop buf_prop;
	u16 args[1];
	unsigned bytes;

	/* gray_wq_skip flag allows to fore stop splash work */
	atomic_set(&status->gray_wq_skip, 0);

	/* get exclusive access to enable controller if it went sleep */
	down_write(&status->pwr_lock);
	mutex_lock(&par->io_lock);

	/*
	 * splash worker could abort execution in next cases:
	 * - current picture already is gray
	 * - drawing on EPD was disabled
	 * - gray_wq_skip was set in drawing threads
	 *   while gray worker wait on locks
	 */
	if ((par->waveform == WF_MODE_GC16) ||
	    par->draw_off ||
	    atomic_read(&status->gray_wq_skip)) {

		mutex_unlock(&par->io_lock);
		up_write(&status->pwr_lock);
		return;
	}

	if (status->is_sleep)
		s1d13522fb_exit_low_power(par);

	buf_prop.width = info->var.xres_virtual;
	buf_prop.height = info->var.yres_virtual;

	buf_prop.left = 0;
	buf_prop.top = info->var.yoffset;
	buf_prop.right = info->var.xres;
	buf_prop.bottom = info->var.yoffset + info->var.yres;

	/* use default-dummy(zero) algorithm to get gay buffer */
	bytes = dither_list[0].fn(&buf_prop, &status->helper,
				  info->screen_base, par->epd_buffer);

	args[0] = 0x3 << 4;
	par->board->send_cmdargs(par, S1D13522_CMD_LD_IMG, 1, args);

	args[0] = 0x154;
	par->board->send_cmdargs(par, S1D13522_CMD_WR_REG, 1, args);

	par->board->burst_write(par, bytes, par->epd_buffer);

	par->board->send_command(par, S1D13522_CMD_LD_IMG_END);

	args[0] = WF_MODE_GC16 << 8;
	par->board->send_cmdargs(par, UPD_PART, 1, args);
	s1d13522fb_statistic_inc(par, gl_part_upd);

	par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_TRG);

	/* always wait end of drawing since next update could be not sync */
	par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_FREND);

	if (status->is_sleep)
		s1d13522fb_turn_low_power(par);

	/* after gray update next update should be DU */
	par->waveform = WF_MODE_DU;

	mutex_unlock(&par->io_lock);
	up_write(&status->pwr_lock);
}

void s1d13522fb_screen_update(struct s1d13522fb_par *par, u16 dl_par[5])
{
	/* TODO: handle GLR update */
	enum upd_type upd_code;
	u16 tmp[2];
	u16 *args;
	int ms;
#ifdef CONFIG_EPD_NOTIFIER
	bool send_event = 0;
	struct epd_event event;
#endif

	if (unlikely(par->draw_off))
		return;

	// Apply software workaround for FULL (A2 and DU) updates
	if ((par->update_mode == UPD_FULL) &&
	    (par->waveform == WF_MODE_A2 || par->waveform == WF_MODE_DU)) {

		if (dl_par) {
			par->waveform = WF_MODE_GC16;
			s1d13522fb_screen_update(par, dl_par);
			par->waveform = WF_MODE_DU;
			return;
		}

		par->update_mode = UPD_PART;

		tmp[0] = 0;
		tmp[1] = 0;
		par->board->send_cmdargs(par, S1D13522_CMD_PIP_XYSETUP, 2, tmp);

		s1d13522fb_screen_update(par, NULL);

		par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_TRG);

		tmp[0] = par->info->var.xres;
		tmp[1] = par->info->var.yres;
		par->board->send_cmdargs(par, S1D13522_CMD_PIP_XYSETUP, 2, tmp);

		// Intermediate update should be always synchronous
		par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_FREND);
	}

	if (dl_par) {
		args = dl_par;
		upd_code = (par->update_mode == UPD_FULL) ? UPD_FULL_AREA :
							    UPD_PART_AREA;
		s1d13522fb_statistic_inc(par, region_upd);
	} else {
		args = tmp;
		upd_code = par->update_mode;
	}

	/*
	 * In case enabled optimized vsync mechanism
	 * wait frame end before trigger new screen update.
	 * This trick allow to save time required to send image via SPI
	 */
	if (atomic_xchg(&par->status->vsync_en, 0))
		par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_FREND);
	else
		par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_LUTFREE);

#ifdef CONFIG_EPD_NOTIFIER
	if ((par->waveform == WF_MODE_GC16) && (upd_code == UPD_FULL))
		send_event = true;

	if (send_event) {
		event.type = EPD_RADIO_NOISE_START;
		epd_notifier_call_chain(EPD_EVENT_RADIO_NOISE, &event);
	}
#endif

	args[0] = par->waveform << 8;
	par->board->send_cmdargs(par, upd_code, (dl_par) ? 5 : 1, args);

	switch (par->waveform) {
	case WF_MODE_DU:
	case WF_MODE_A2:
		s1d13522fb_statistic_inc(par, bw_upd);
		break;
	default:
		if ((upd_code == UPD_FULL) || (upd_code == UPD_FULL_AREA)) {
			s1d13522fb_statistic_inc(par, gl_full_upd);
		} else {
			s1d13522fb_statistic_inc(par, gl_part_upd);
		}
		break;
	}

	dev_dbg(par->info->dev, "update screen mode: %s, waveform: %s\n",
		mode_to_str(upd_code), waveform_to_str(par->waveform));

#ifdef CONFIG_EPD_NOTIFIER
	if (send_event) {
		/*
		 * Always wait frame drawing end
		 * to be sure that radio noise is gone
		 */
		par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_TRG);
		par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_FREND);
		par->board->wait_for_rdy(par, 1000);

		event.type = EPD_RADIO_NOISE_STOP;
		epd_notifier_call_chain(EPD_EVENT_RADIO_NOISE, &event);
	}
#endif

	/* from gray to black-white mode throughout DU: GC16->DU->A2 */
	if (par->waveform == WF_MODE_DU)
		par->waveform = WF_MODE_A2;

	/*
	 * trigger gray splash timer
	 * if update is not gray and splash option is enabled
	 */
	ms = atomic_read(&par->splash_timeout);

	if ((par->waveform != WF_MODE_GC16) && (ms > 0)) {
		atomic_set(&par->status->gray_wq_skip, 1);
		__cancel_delayed_work(&par->splash_worker);
		schedule_delayed_work(&par->splash_worker, msecs_to_jiffies(ms));
	}

	/* reset update type back to partial */
	par->update_mode = UPD_PART;
}

static void s1d13522fb_dpy_update_pages(struct s1d13522fb_par *par,
						short y1, short y2)
{
	struct fb_info *info = par->info;
	unsigned bytes;
	dthr_prop buf_prop;
	u16 args[5];

	s1d13522fb_power_acquire(par->status);
	mutex_lock(&par->io_lock);

	/* y1 must be a multiple of 4 so drop the lower bits */
	y1 &= 0xFFFC;
	/* y2 must be a multiple of 4 , but - 1 so up the lower bits */
	y2 |= 0x0003;

	preempt_disable();
	buf_prop.width = info->var.xres_virtual;
	buf_prop.height = info->var.yres_virtual;

	buf_prop.left = 0;
	buf_prop.top = info->var.yoffset + y1;
	buf_prop.right = info->var.xres;
	buf_prop.bottom = info->var.yoffset + y2;

	bytes = dither_list[par->dthr_num].fn(&buf_prop, &par->status->helper,
					      info->screen_base,
					      par->epd_buffer);
	preempt_enable();

	args[0] = 0x3 << 4;
	args[1] = 0;
	args[2] = y1;
	args[3] = cpu_to_le16(par->info->var.xres);
	args[4] = y2;
	par->board->send_cmdargs(par, S1D13522_CMD_LD_IMG_AREA, 5, args);

	args[0] = 0x154;
	par->board->send_cmdargs(par, S1D13522_CMD_WR_REG, 1, args);

	par->board->burst_write(par, bytes, par->epd_buffer);

	par->board->send_command(par, S1D13522_CMD_LD_IMG_END);

	s1d13522fb_screen_update(par, args);

	par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_TRG);

	mutex_unlock(&par->io_lock);
	s1d13522fb_power_release(par->status);
}

static void s1d13522fb_dpy_update(struct s1d13522fb_par *par)
{
	struct fb_info *info = par->info;
	unsigned bytes;
	dthr_prop buf_prop;
	u16 args[1];

	s1d13522fb_power_acquire(par->status);
	mutex_lock(&par->io_lock);

	preempt_disable();
	buf_prop.width = info->var.xres_virtual;
	buf_prop.height = info->var.yres_virtual;

	buf_prop.left = 0;
	buf_prop.top = info->var.yoffset;
	buf_prop.right = info->var.xres;
	buf_prop.bottom = info->var.yoffset + info->var.yres;

	bytes = dither_list[par->dthr_num].fn(&buf_prop, &par->status->helper,
					      info->screen_base,
					      par->epd_buffer);
	preempt_enable();

	WARN_ON(bytes != (info->var.xres*info->var.yres));

	args[0] = 0x3 << 4;
	par->board->send_cmdargs(par, S1D13522_CMD_LD_IMG, 1, args);

	args[0] = 0x154;
	par->board->send_cmdargs(par, S1D13522_CMD_WR_REG, 1, args);

	par->board->burst_write(par, bytes, par->epd_buffer);

	par->board->send_command(par, S1D13522_CMD_LD_IMG_END);

	s1d13522fb_screen_update(par, NULL);

	par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_TRG);

	mutex_unlock(&par->io_lock);
	s1d13522fb_power_release(par->status);
}

static void s1d13522fb_dpy_deferred_io(struct fb_info *info,
					struct list_head *pagelist)
{
	int yres = info->var.yres;
	int prev_index = -1;
	int y1 = 0, h = 0;
	int p_start, p_end;
	struct page *cur;
	int h_inc;

	/* height increment is fixed per page (lines per page) */
	h_inc = DIV_ROUND_UP(PAGE_SIZE , info->fix.line_length);

	/* get valid active area offset */
	if (info->var.yoffset == 0) {
		p_start = 0;
		p_end = (info->screen_size / PAGE_SIZE) / 2;
	} else {
		p_start = (info->screen_size / PAGE_SIZE) / 2;
		p_end = info->screen_size / PAGE_SIZE;

		if (unlikely(info->var.yoffset != (info->var.yres_virtual / 2)))
			dev_err(info->dev, "invalid yoffset: %u  0 or %d\n",
				 info->var.yoffset,
				 info->var.yres_virtual/2);
	}

	if (list_empty(pagelist)) {
		s1d13522fb_dpy_update(info->par);
		return;
	}

	/* walk the written page list and swizzle the data */
	list_for_each_entry(cur, pagelist, lru) {
		/* skip non active area */
		if ((cur->index > p_end) || (cur->index < p_start))
			continue;

		if (prev_index < 0) {
			/* just starting so assign first page */
			y1 = (cur->index << PAGE_SHIFT) / info->fix.line_length;
			h = h_inc;
		} else if ((prev_index + 1) == cur->index) {
			/* this page is consecutive so increase our height */
			h += h_inc;
		} else {
			/* page not consecutive, issue previous update first */
			s1d13522fb_dpy_update_pages(info->par, y1, y1 + h);
			/* start over with our non consecutive page */
			y1 = (cur->index << PAGE_SHIFT) / info->fix.line_length;
			h = h_inc;
		}
		prev_index = cur->index;
	}

	/* if we still have any pages to update we do so now */
	if (h >= yres) {
		/* its a full screen update, just do it */
		s1d13522fb_dpy_update(info->par);
	} else {
		s1d13522fb_dpy_update_pages(info->par, y1,
						min((y1 + h), yres));
	}
}

static struct fb_deferred_io s1d13522fb_defio = {
	.delay		= HZ/20,			/* 50 ms */
	.deferred_io	= s1d13522fb_dpy_deferred_io,
};

static int s1d13522fb_pan_display(struct fb_var_screeninfo *var,
				  struct fb_info *info)
{
	if ((var->xoffset != 0) || (var->yoffset != (info->var.yres_virtual/2)))
		return -EINVAL;

	return 0;
}

static void s1d13522fb_fillrect(struct fb_info *info,
				   const struct fb_fillrect *rect)
{
	sys_fillrect(info, rect);

	cancel_delayed_work(&info->deferred_work);
	schedule_delayed_work(&info->deferred_work, HZ/10);
}

static void s1d13522fb_copyarea(struct fb_info *info,
				   const struct fb_copyarea *area)
{
	sys_copyarea(info, area);

	cancel_delayed_work(&info->deferred_work);
	schedule_delayed_work(&info->deferred_work, HZ/10);
}

static void s1d13522fb_imageblit(struct fb_info *info,
				const struct fb_image *image)
{
	sys_imageblit(info, image);

	cancel_delayed_work(&info->deferred_work);
	schedule_delayed_work(&info->deferred_work, HZ/10);
}

static int s1d13522fb_check_var(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	struct s1d13522fb_par *par = info->par;

	/* check that real image is equal to panel size */
	if (par->board && par->board->get_panel_config) {
		if (var->xres != par->board->get_panel_config("width"))
			return -EINVAL;

		if (var->yres != par->board->get_panel_config("height"))
			return -EINVAL;
	}

	/* Do not allow to have real line length larger than virtual */
	if (var->xres > var->xres_virtual)
		var->xres_virtual = var->xres;

	/* allow only one and double buffering */
	if ((var->yres != var->yres_virtual) &&
	    ((var->yres * 2) != var->yres_virtual))
		var->yres_virtual = var->yres * 2;

	var->red.msb_right = var->green.msb_right = var->blue.msb_right = 0;

	switch (var->bits_per_pixel) {
	case 16:
		var->red.offset = 11;
		var->red.length = 5;

		var->green.offset = 5;
		var->green.length = 6;

		var->blue.offset = 0;
		var->blue.length = 5;

		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 32:
		var->transp.offset = 0;
		var->transp.length = 8;

		var->red.offset = 8;
		var->red.length = 8;

		var->green.offset = 16;
		var->green.length = 8;

		var->blue.offset = 24;
		var->blue.length = 8;
		break;
	default:
		return -EINVAL;
	}

	var->xoffset = var->yoffset = 0;
	var->transp.msb_right = 0;

	return 0;
}

static void s1d13522fb_gen_epd_cmap(struct fb_cmap *cmap, unsigned bpp)
{
	int i;

	switch (bpp) {
	case 32:
		for (i = 0; i < 256; i++) {
			/* red channel weight 0.30 * 65536 */
			cmap->red[i] = ((i * 19661) >> 16) & 0xFF;
			/* green channel weight 0.59 * 65536 */
			cmap->green[i] = ((i * 38666) >> 16) & 0xFF;
			/* blue channel weight 0.11 * 65536*/
			cmap->blue[i] = ((i * 7209) >> 16) & 0xFF;
		}

		i = 0;
		i += cmap->red[255];
		i += cmap->green[255];
		i += cmap->blue[255];
		break;
	case 16:
		/* set default color mar for gray mode (red and blue have 5 bits)*/
		for (i = 0; i < 32; i++) {
			/* red channel weight 0.22 * 65536 */
			cmap->red[i] = (((i << 3) * 14417) >> 16) & 0xFF;
			/* blue channel weight 0.075 * 65536 */
			cmap->blue[i] = (((i << 3) * 4915) >> 16) & 0xFF;
		}

		/* green channel have 6 bit */
		for (i = 0; i < 64; i++)
			/* green channel weight 0.73 * 65536 */
			cmap->green[i] = (((i << 2) * 47841) >> 16) & 0xFF;

		i = 0;
		i += cmap->red[31];
		i += cmap->green[63];
		i += cmap->blue[31];
		break;
	default:
		BUG();
	}

	WARN(i > 255, "Invalid color map\n");
}

static int s1d13522fb_set_par(struct fb_info *info)
{
	struct s1d13522fb_par *par = info->par;
	struct fb_cmap cmap_old;
	unsigned vm_size;
	unsigned byte_pp;
	void *p_mem;

	if (info->fbdefio)
		cancel_delayed_work_sync(&info->deferred_work);

	mutex_lock(&par->io_lock);

	byte_pp = info->var.bits_per_pixel / 8;
	info->fix.line_length  = info->var.xres_virtual * byte_pp;
	/* get size of one screen */
	vm_size = roundup((info->fix.line_length * info->var.yres), PAGE_SIZE);

	/* multiply for double buffering */
	if (info->var.yres != info->var.yres_virtual)
		vm_size = vm_size * 2;

	info->screen_size = info->fix.smem_len = vm_size;

	p_mem = vzalloc(vm_size);
	if (!p_mem) {
		mutex_unlock(&par->io_lock);
		return -ENOMEM;
	}

	vfree(info->screen_base);
	info->screen_base = p_mem;

	cmap_old = info->cmap;
	if (info->var.bits_per_pixel == 16)
		fb_alloc_cmap(&info->cmap, 64, 0);
	else
		fb_alloc_cmap(&info->cmap, 256, 0);

	if (info->cmap.len == 0) {
		info->cmap = cmap_old;
		mutex_unlock(&par->io_lock);
		return -EINVAL;
	}

	s1d13522fb_gen_epd_cmap(&info->cmap, info->var.bits_per_pixel);

	if (par->status)
		par->status->helper.pix_sz = byte_pp;

	dev_dbg(info->dev, "set new video mode [%ux%u] %ubpp\n",
		info->var.xres_virtual,
		info->var.yres_virtual,
		info->var.bits_per_pixel);

	mutex_unlock(&par->io_lock);

	return 0;
}

static int s1d13522fb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	if (cmap->transp)
		return -EINVAL;

	if (info->var.bits_per_pixel == 16) {
		if (cmap->len != 64)
			return -EINVAL;

		if ((cmap->red[31] + cmap->blue[31] + cmap->green[63]) > 255)
			return -EINVAL;
	} else {
		if (cmap->len != 256)
			return -EINVAL;

		if ((cmap->red[255] + cmap->blue[255] + cmap->green[255]) > 255)
			return -EINVAL;
	}

	return 0;
}

static u8 __attribute__ ((hot, optimize(Ofast)))
s1d13522fb_input_sampler(void *pix, void *priv)
{
	struct fb_info *info = priv;
	struct fb_cmap *cmap = &info->cmap;
	u8 r,g,b;
	union {
		union {
			u16	raw;
			struct {
				u8 blue:5;
				u8 green:6;
				u8 red:5;
			};
		} rgb565;
		union {
			u32	raw;
			struct {
				u8 blue;
				u8 green;
				u8 red;
				u8 alpha;
			};
		} argb8888;
	} *cpix = pix;

	if (info->var.bits_per_pixel == 16) {
		r = cpix->rgb565.red;
		g = cpix->rgb565.green;
		b = cpix->rgb565.blue;
	} else {
		r = cpix->argb8888.red;
		g = cpix->argb8888.green;
		b = cpix->argb8888.blue;
	}

	return cmap->red[r] + cmap->green[g] + cmap->blue[b];
}

/*
 * this is the slow path from userspace. they can seek and write to
 * the fb. it's inefficient to do anything less than a full screen draw
 */
static ssize_t s1d13522fb_write(struct fb_info *info, const char __user *buf,
				size_t count, loff_t *ppos)
{
	size_t ret;
	ret = fb_sys_write(info, buf, count, ppos);

	s1d13522fb_dpy_update(info->par);
	return ret;
}

static int s1d13522fb_wait_for_vsync(struct s1d13522fb_par *par, u32 crtc)
{
	struct s1d13522fb_status *status = par->status;

	down_read(&status->pwr_lock);
	if (status->is_sleep) {
		up_read(&status->pwr_lock);
		return 0;
	}

	/*
	 * reuse crtc parameter to signal vsync implementation
	 * use optimized vsync mechanism or really wait frame end
	 */
	if (likely(crtc != 0)) {
		atomic_set(&status->vsync_en, 1);
	} else {
		mutex_lock(&par->io_lock);
		par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_FREND);
		par->board->wait_for_rdy(par, 1000);
		mutex_unlock(&par->io_lock);
	}

	up_read(&status->pwr_lock);

	return 0;
}

static int s1d13522fb_ioctl(struct fb_info *info, unsigned int cmd,
			    unsigned long arg)
{
	struct s1d13522fb_par *par = info->par;
	int ret;
	u32 crtc;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		if (get_user(crtc, (u32 __user *)arg)) {
			ret = -EFAULT;
			break;
		}
		ret = s1d13522fb_wait_for_vsync(par, crtc);
		break;
	default:
		ret = -ENOTTY;
	}

	return ret;
}

static struct fb_ops s1d13522fb_ops = {
	.owner		= THIS_MODULE,
	.fb_read	= fb_sys_read,
	.fb_write	= s1d13522fb_write,
	.fb_fillrect	= s1d13522fb_fillrect,
	.fb_copyarea	= s1d13522fb_copyarea,
	.fb_imageblit	= s1d13522fb_imageblit,
	.fb_pan_display	= s1d13522fb_pan_display,
	.fb_check_var	= s1d13522fb_check_var,
	.fb_set_par	= s1d13522fb_set_par,
	.fb_setcmap	= s1d13522fb_setcmap,
	.fb_ioctl	= s1d13522fb_ioctl,
};

static int s1d13522fb_set_temp(struct s1d13522fb_par *par, int degree)
{
	u16 t;

	dev_dbg(par->info->dev, "set new temperature [%d C]\n", degree);
	par->status->curent_temp = degree;

	if (degree < 0)
		t = 255 + degree + 1;
	else if (degree > 50)
		t = 50;
	else
		t = degree;

	mutex_lock(&par->io_lock);

	s1d13522fb_write_reg(par, S1D13522_REG0322_TEMPERATURE_VALUE, t);

	mutex_unlock(&par->io_lock);

	return 0;
}

static void s1d13522fb_temperature_work(struct work_struct *work)
{
	struct s1d13522fb_par *par = container_of(work, struct s1d13522fb_par,
							temp_worker.work);
	int temp;

	s1d13522fb_power_acquire(par->status);

	if (par->board->get_temp && !par->board->get_temp(par, &temp))
		s1d13522fb_set_temp(par, temp);

	s1d13522fb_power_release(par->status);

	schedule_delayed_work(&par->temp_worker, HZ * 60 * TEMP_UPD_INTERVAL);
}

static int s1d13522fb_init_panel(struct s1d13522fb_par *par)
{
	int ret;
	int pip_size;
	u16 cmd_param[4];
	s1d13522fb_addr_t pip_mem;
	u16 data;

	/* set default temperature */
	s1d13522fb_write_reg(par, S1D13522_REG0320_TEMPERATURE_DEVICE, BIT(0));
	s1d13522fb_write_reg(par, S1D13522_REG0322_TEMPERATURE_VALUE, 20);

	schedule_delayed_work(&par->temp_worker, HZ);

	/* set default rotation mode */
	cmd_param[0] = 0x100;
	ret = par->board->send_cmdargs(par, S1D13522_CMD_INIT_ROTMODE,
				       1, cmd_param);
	if (ret) {
		dev_err(par->info->dev, "failed to set rotation mode\n");
		return ret;
	}

	/*
	 * copy rotation mode to general configuration register
	 * for properly regional updates and switch to absolute
	 * region coordinates instead horizontal/vertical sizes
	 * REG032C_GENERAL_CONFIG:
	 * 12 bit - Area Coordinate End Size Select (should be zero)
	 * 8-9 bits Area Coordinate Rotation Select (same as REG0140)
	 */
	data = s1d13522fb_read_reg(par, S1D13522_REG0140_MEM_ACCESS_CONFIG);
	data &= (BIT(9) | BIT(8));
	s1d13522fb_write_reg(par, S1D13522_REG032C_GENERAL_CONFIG, data);


	/* Switch update rectangle mode to "use host X/Y Start/End position"*/
	data = s1d13522fb_read_reg(par, S1D13522_REG0334_CONTROL_TRIGGER);
	data &= ~BIT(13);
	data |= BIT(12);
	s1d13522fb_write_reg(par, S1D13522_REG0334_CONTROL_TRIGGER, data);

	/* Enable PIP engine */
	cmd_param[0] = (0x3 << 4);
	cmd_param[1] = par->info->var.xres;
	cmd_param[2] = par->info->var.yres;
	cmd_param[3] = 0;
	ret |= par->board->send_cmdargs(par, S1D13522_CMD_PIP_ENABLE,
					4, cmd_param);

	pip_mem.addr_h = s1d13522fb_read_reg(par, S1D13522_REG0312_IMG_BUFFER_SA1);
	pip_mem.addr_l = s1d13522fb_read_reg(par, S1D13522_REG0310_IMG_BUFFER_SA0);

	/*
	 * place PIP buffer after main window buffer
	 * note that real pixel size in controller memory is 4 bits
	 * and address should be aligned on 64 bit (8 byte)
	 */
	pip_mem.addr += (par->info->var.xres * par->info->var.yres) / 2;
	pip_mem.addr = roundup(pip_mem.addr, 8);

	cmd_param[0] = pip_mem.addr_l;
	cmd_param[1] = pip_mem.addr_h;
	ret |= par->board->send_cmdargs(par, S1D13522_CMD_PIP_ADRCFG,
					2, cmd_param);
	if (ret) {
		dev_err(par->info->dev, "failed to enable PIP mode\n");
		return ret;
	}

	/* prepare white PIP area for intermediate A2 updates */
	pip_size = par->info->var.xres * par->info->var.yres;
	memset(par->epd_buffer, 0xFF, pip_size);

	cmd_param[0] = (0x3 << 4) | BIT(10);
	par->board->send_cmdargs(par, S1D13522_CMD_LD_IMG, 1, cmd_param);

	cmd_param[0] = 0x154;
	par->board->send_cmdargs(par, S1D13522_CMD_WR_REG, 1, cmd_param);

	par->board->burst_write(par, pip_size, par->epd_buffer);

	par->board->send_command(par, S1D13522_CMD_LD_IMG_END);

	cmd_param[0] = par->info->var.xres;
	cmd_param[1] = par->info->var.yres;
	ret = par->board->send_cmdargs(par, S1D13522_CMD_PIP_XYSETUP,
				       2, cmd_param);

	return ret;
}

static int __devinit
s1d13522fb_identify(struct s1d13522fb_par *par)
{
	u16 rev, prc;
	struct device *dev = par->info->device;
#ifdef CONFIG_EPD_SFLASH
	int vcom, ret;
#endif

	rev = s1d13522fb_read_reg(par, S1D13522_REG0000_REV_CODE);
	prc = s1d13522fb_read_reg(par, S1D13522_REG0002_PROD_CODE);
	dev_info(dev, "Epson s1d13522 Rev 0x%x, Product Code 0x%x\n", rev, prc);

#ifdef CONFIG_EPD_SFLASH
	ret = epd_sflash_get_vcom(par, &vcom);
	if (ret) {
		dev_err(dev, "%s: failed to get VCOM value, "
			"panel is not connected\n", __func__);
		return ret;
	} else {
		dev_info(dev, "Panel detected, VCOM = %d\n", vcom);
	}

	return ret;
#else
	return 0;
#endif
}

static int __devinit
s1d13522fb_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	struct s1d13522fb_board *board;
	struct s1d13522fb_par *def_par;
	struct s1d13522fb_status *status;
	int dpyw = 0, dpyh = 0;

	/* pick up board specific routines */
	board = pdev->dev.platform_data;
	if (!board)
		return -EINVAL;

	/* try to count device specific driver, if can't, platform recalls */
	if (!try_module_get(board->owner))
		return -ENODEV;

	info = framebuffer_alloc(sizeof(*def_par), &pdev->dev);
	if (!info)
		goto err;

	if (board->get_panel_config) {
		dpyw = board->get_panel_config("width");
		dpyh = board->get_panel_config("height");
	}

	if ((dpyh <= 0) || (dpyw <= 0)) {
		dev_err(&pdev->dev, "wrong panel dimensions h: %d, w: %d",
				    dpyh, dpyw);
		goto err_fb_rel;
	}

	s1d13522fb_var.xres = dpyw;
	s1d13522fb_var.yres = dpyh;

	info->fbops = &s1d13522fb_ops;
	info->pseudo_palette = s1d13522fb_pseudo_palette;
	info->var = s1d13522fb_var;
	info->fix = s1d13522fb_fix;

	def_par = info->par;
	def_par->info = info;
	def_par->board = board;
	def_par->write_reg = s1d13522fb_write_reg;
	def_par->read_reg = s1d13522fb_read_reg;
	def_par->set_temp = s1d13522fb_set_temp;

	init_waitqueue_head(&def_par->waitq);
	mutex_init(&def_par->io_lock);
	atomic_set(&def_par->splash_timeout, 0);

	if (s1d13522fb_check_var(&info->var, info) ||
	    s1d13522fb_set_par(info))
		goto err_fb_rel;

	INIT_DELAYED_WORK(&def_par->temp_worker, s1d13522fb_temperature_work);
	INIT_DELAYED_WORK(&def_par->splash_worker, s1d13522fb_splash_work);

	/* init framebuffer status struct */
	status = kmalloc(sizeof(struct s1d13522fb_status), GFP_KERNEL);
	if (!status)
		goto err_vfree;

	status->par = def_par;

	/* Reset update counters */
	s1d13522fb_statistic_reset(&def_par->statistic);

	atomic_set(&status->vsync_en, 0);
	init_rwsem(&status->pwr_lock);
	INIT_DELAYED_WORK(&status->pwr_worker, s1d13522fb_power_work);
	status->is_sleep = 0;

	def_par->status = status;
	def_par->power_acquire = s1d13522fb_power_acquire;
	def_par->power_release = s1d13522fb_power_release;

	/* allocate two extra lines for dithering purposes */
	def_par->epd_buffer = kmalloc(dpyw * (dpyh + 2), GFP_KERNEL | GFP_DMA);
	if (!def_par->epd_buffer)
		goto err_stat_free;

	def_par->draw_off = 0;
	def_par->waveform = WF_MODE_GC16;
	def_par->update_mode = UPD_FULL;

	info->flags = FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;
	/* use pan only for double-buffering */
	info->flags |= FBINFO_PARTIAL_PAN_OK;

	info->fbdefio = &s1d13522fb_defio;
	fb_deferred_io_init(info);

	if (!board->setup_irq || board->setup_irq(info)) {
		dev_err(&pdev->dev, "failed to setup irq\n");
		goto err_kfree;
	}

	if (!board->init || board->init(def_par)) {
		dev_err(&pdev->dev, "failed to init board\n");
		goto err_free_irq;
	}

	if(s1d13522fb_identify(def_par)) {
		dev_err(&pdev->dev, "failed to identify panel\n");
		goto err_free_irq;
	}

	if (s1d13522fb_init_panel(def_par)) {
		dev_err(&pdev->dev, "failed to init panel\n");
		goto err_free_irq;
	}

	/* setup dithering environment */
	def_par->dthr_num = 0;
	status->helper.priv = info;
	status->helper.pix_sz = info->var.bits_per_pixel/8;
	status->helper.input_sampler = s1d13522fb_input_sampler;

	if (register_framebuffer(info) < 0)
		goto err_stop_worker;

	platform_set_drvdata(pdev, info);

	if (s1d13522fb_register_sysfs(info))
		goto err_unreg_fb;

	printk(KERN_INFO
	       "fb%d: Epson s1d13522 frame buffer, using %luK of memory\n",
	       info->node, (info->screen_size + (dpyw*dpyh)) >> 10);

#ifdef CONFIG_LOGO
	if (fb_prepare_logo(info, FB_ROTATE_UR)) {
		/* show logo on boot */
		fb_show_logo(info, FB_ROTATE_UR);
	}
#else
	memset(info->screen_base,
#ifdef CONFIG_EPD_SFLASH
	       epd_sflash_is_case_white(def_par) ? 0x00 : 0xFF,
#else
	       0xFF,
#endif /* CONFIG_EPD_SFLASH */
	       info->screen_size);
	schedule_delayed_work(&info->deferred_work, HZ/10);
#endif /* CONFIG_LOGO */

	return 0;

err_unreg_fb:
	unregister_framebuffer(info);
err_stop_worker:
	cancel_delayed_work_sync(&def_par->temp_worker);
err_free_irq:
	if (board->cleanup)
		board->cleanup(def_par);
err_kfree:
	kfree(def_par->epd_buffer);
err_stat_free:
	kfree(def_par->status);
err_vfree:
	vfree(info->screen_base);
	fb_dealloc_cmap(&info->cmap);
err_fb_rel:
	framebuffer_release(info);
err:
	module_put(board->owner);
	return -EINVAL;
}

static int __devexit
s1d13522fb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);

	if (info) {
		struct s1d13522fb_par *par = info->par;

		s1d13522fb_unregister_sysfs(info);

		unregister_framebuffer(info);
		fb_deferred_io_cleanup(info);

		cancel_delayed_work_sync(&par->temp_worker);
		cancel_delayed_work_sync(&par->status->pwr_worker);

		if (par->board->cleanup)
			par->board->cleanup(par);
		fb_dealloc_cmap(&info->cmap);
		kfree(par->status);
		kfree(par->epd_buffer);
		vfree((void *)info->screen_base);
		module_put(par->board->owner);
		framebuffer_release(info);
	}

	return 0;
}

#ifdef CONFIG_PM
static int s1d13522fb_suspend(struct platform_device *dev, pm_message_t msg)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct s1d13522fb_par *par = info->par;
	struct s1d13522fb_status *status = par->status;

	cancel_delayed_work_sync(&status->pwr_worker);
	flush_delayed_work_sync(&par->splash_worker);

	down_write(&status->pwr_lock);

	if (!status->is_sleep) {
		mutex_lock(&par->io_lock);
		s1d13522fb_turn_low_power(par);
		mutex_unlock(&par->io_lock);
		status->is_sleep = 1;
		dev_dbg(par->info->dev, "epd going sleep on suspend\n");
	}

	up_write(&status->pwr_lock);

	return 0;
}
#endif /* CONFIG_PM */

static struct platform_driver s1d13522fb_driver = {
	.probe		= s1d13522fb_probe,
	.remove		= s1d13522fb_remove,
#ifdef CONFIG_PM
	.suspend	= s1d13522fb_suspend,
#endif
	.driver = {
		.name	= "s1d13522fb",
	},
};

static int __init
s1d13522fb_init(void)
{
	return platform_driver_register(&s1d13522fb_driver);
}


static void __exit
s1d13522fb_exit(void)
{
	platform_driver_unregister(&s1d13522fb_driver);
}

module_init(s1d13522fb_init);
module_exit(s1d13522fb_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Framebuffer driver for S1D13522 devices");
MODULE_AUTHOR("Dmitry Ivanov <dmitriy.ivanov@yotadevices.com>");
