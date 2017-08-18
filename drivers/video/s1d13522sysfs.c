/* drivers/video/s1d13522sysfs.c
 *
 * (c) 2016 Yota Devices
 *
 * sysfs interface for S1D13522 framebuffer driver
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/dithering.h>

#include <video/s1d13522fb.h>

static ssize_t s1d13522fb_set_full_upd(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct s1d13522fb_par *par = info->par;
	int r, val;

	r = kstrtoint(buf, 10, &val);
	if (r)
		return r;

	if (val == 1) {
		mutex_lock(&par->io_lock);
		par->update_mode = UPD_FULL;
		mutex_unlock(&par->io_lock);
	} else if (val == 0) {
		mutex_lock(&par->io_lock);
		par->update_mode = UPD_PART;
		mutex_unlock(&par->io_lock);
	} else
		return -EINVAL;

	return size;
}

static ssize_t s1d13522fb_update_disable(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct s1d13522fb_par *par = info->par;
	int r, val;

	r = kstrtoint(buf, 10, &val);
	if (r)
		return r;

	if (val == 1) {
		mutex_lock(&par->io_lock);
		par->draw_off = 1;
		mutex_unlock(&par->io_lock);
	} else if (val == 0) {
		mutex_lock(&par->io_lock);
		par->draw_off = 0;
		mutex_unlock(&par->io_lock);
	} else
		return -EINVAL;

	return size;
}

static ssize_t s1d13522fb_set_update_area(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct s1d13522fb_par *par = info->par;
	unsigned x1, x2, y1, y2;
	unsigned char tmp;
	u16 args[5];

	if (sscanf(buf, "%ux%u %ux%u", &x1, &y1, &x2, &y2) != 4)
		return -EINVAL;

	if ((x1 >= x2) || (x1 > info->var.xres) || (x2 > info->var.xres) ||
	    (y1 >= y2) || (y1 > info->var.yres) || (y2 > info->var.yres))
		return -EINVAL;

	if (par->power_acquire)
		par->power_acquire(par->status);
	mutex_lock(&par->io_lock);

	args[1] = cpu_to_le16(x1);
	args[2] = cpu_to_le16(y1);
	args[3] = cpu_to_le16(x2);
	args[4] = cpu_to_le16(y2);

	/* temporary enable drawing even if it was disabled */
	tmp = par->draw_off;
	par->draw_off = 0;

	s1d13522fb_screen_update(par, args);
	par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_TRG);

	par->draw_off = tmp;
	mutex_unlock(&par->io_lock);

	if (par->power_release)
		par->power_release(par->status);

	return size;
}

static ssize_t s1d13522fb_store_temp(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct s1d13522fb_par *par = info->par;
	int r, val;
	char *auto_mode = "auto";

	if (!strncmp(buf, auto_mode, strlen(auto_mode))) {
		schedule_delayed_work(&par->temp_worker, 0);
		return size;
	}

	r = kstrtoint(buf, 10, &val);
	if (r)
		return r;

	if ((val > TEMP_MAX) || (val < -20))
		return -EINVAL;

	cancel_delayed_work_sync(&par->temp_worker);

	if (par->power_acquire)
		par->power_acquire(par->status);

	r = par->set_temp(par, val);

	if (par->power_release)
		par->power_release(par->status);

	return r ? r : size;
}

static ssize_t s1d13522fb_read_temp(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct s1d13522fb_par *par = info->par;
	int r, val;


	if (!par->board->get_temp)
		return -ENODEV;

	if (par->power_acquire)
		par->power_acquire(par->status);

	r = par->board->get_temp(par, &val);

	if (par->power_release)
		par->power_release(par->status);

	return r ? r : snprintf(buf, PAGE_SIZE, "%d\n", val);
}


static ssize_t s1d13522fb_read_splash(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct s1d13522fb_par *par = info->par;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			atomic_read(&par->splash_timeout));
}

static ssize_t s1d13522fb_store_splash(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct s1d13522fb_par *par = info->par;
	int r, val;

	r = kstrtoint(buf, 10, &val);
	if (r)
		return r;

	if (val <= 0) {
		cancel_delayed_work(&par->splash_worker);
		atomic_set(&par->splash_timeout, 0);
	} else {
		atomic_set(&par->splash_timeout, val);
		schedule_delayed_work(&par->splash_worker, msecs_to_jiffies(val));
	}

	return size;
}

static ssize_t s1d13522fb_show_dithering(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct s1d13522fb_par *par = info->par;
	unsigned i;
	ssize_t sz = 0;

	for (i = 0; i < DITH_CNT; i++)
		if (par->dthr_num == i)
			sz += scnprintf(buf + sz, PAGE_SIZE - sz - 2,
					"[%s] ", dither_list[i].name);
		else
			sz += scnprintf(buf + sz, PAGE_SIZE - sz - 2,
					"%s ", dither_list[i].name);

	sz += scnprintf(buf + sz, PAGE_SIZE - sz, "\n");

	return sz;
}

static ssize_t s1d13522fb_set_dithering(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct s1d13522fb_par *par = info->par;
	char *cur;
	unsigned i;

	for (i = 0; i < DITH_CNT; i++) {

		cur = dither_list[i].name;

		if (strncmp(buf, cur, strlen(cur)))
			continue;

		/*
		 * Check is it required to change dithering.
		 * If dithering already same no need to
		 * take lock and change mode.
		 * P.S. Not necessary to take io_lock here for
		 * reading par->dthr_num because only one place (this fn)
		 * write value, another threads only read it.
		 */
		wmb();
		if (par->dthr_num == i)
			return size;

		mutex_lock(&par->io_lock);

		par->dthr_num = i;

		if (dither_list[i].is_binary)
			par->waveform = (par->waveform == WF_MODE_GC16) ? \
						WF_MODE_DU : par->waveform;
		else
			par->waveform = WF_MODE_GC16;

		dev_dbg(dev, "Select new dithering algorithm [%s]\n",
			dither_list[i].name);

		mutex_unlock(&par->io_lock);

		return size;
	}

	return -EINVAL;
}

static ssize_t s1d13522fb_show_statistic(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct s1d13522fb_par *par = info->par;
	unsigned i;
	ssize_t sz = 0;

	sz += scnprintf(buf + sz, PAGE_SIZE - sz - 2,
			"\n| Temp\t| B/W\t| GL-F\t| GL\t| Region\n");

	for (i = 0; i < 5; i++)
		sz += scnprintf(buf + sz, PAGE_SIZE - sz - 2, "+-------");

	for (i = 0; i < TEMP_POINTS; i++)
		sz += scnprintf(buf + sz, PAGE_SIZE - sz - 2,
				"\n| %2d\t| %d\t| %d\t| %d\t| %d",
				i * TEMP_SCALE_INTERVAL,
				atomic_read(&par->statistic.bw_upd[i]),
				atomic_read(&par->statistic.gl_full_upd[i]),
				atomic_read(&par->statistic.gl_part_upd[i]),
				atomic_read(&par->statistic.region_upd[i]));

	sz += scnprintf(buf + sz, PAGE_SIZE - sz, "\n");
	return sz;
}

static ssize_t s1d13522fb_raw_data_read(
			struct file *filp,
			struct kobject *kobj, struct bin_attribute *a,
			char *buf, loff_t off, size_t count)
{
	struct device *fbdev = container_of(kobj, struct device, kobj);
	struct fb_info *info = dev_get_drvdata(fbdev);
	struct s1d13522fb_par *par = info->par;
	char *src = par->epd_buffer;

	if ((off >= a->size) || (count > a->size))
		return 0;

	if (off + count > a->size)
		count = a->size - off;

	src += off;

	mutex_lock(&par->io_lock);
	memcpy(buf, src, count);
	mutex_unlock(&par->io_lock);

	return count;
}

static ssize_t s1d13522fb_raw_data_write(
			struct file *filp,
			struct kobject *kobj, struct bin_attribute *a,
			char *src, loff_t src_off, size_t src_size)
{
	struct device *fbdev = container_of(kobj, struct device, kobj);
	struct fb_info *info = dev_get_drvdata(fbdev);
	struct s1d13522fb_par *par = info->par;
	char *dst = par->epd_buffer;
	u16 args[1];

	if ((src_size > a->size) || (src_off >= a->size))
		return -ENOSPC;

	if ((src_off + src_size) > a->size)
		src_size = a->size - src_off;

	dst += src_off;

	if (par->power_acquire)
		par->power_acquire(par->status);
	mutex_lock(&par->io_lock);

	memcpy(dst, src, src_size);

	args[0] = 0x3 << 4;
	par->board->send_cmdargs(par, S1D13522_CMD_LD_IMG, 1, args);

	args[0] = 0x154;
	par->board->send_cmdargs(par, S1D13522_CMD_WR_REG, 1, args);

	par->board->burst_write(par, info->var.xres*info->var.yres,
				par->epd_buffer);

	par->board->send_command(par, S1D13522_CMD_LD_IMG_END);

	/* update screen with current waveform */
	args[0] = par->waveform << 8;
	par->board->send_cmdargs(par, par->update_mode, 1, args);

	par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_TRG);

	mutex_unlock(&par->io_lock);
	if (par->power_release)
		par->power_release(par->status);

	return src_size;
}

static DEVICE_ATTR(clear_artifacts, 0664, NULL, s1d13522fb_set_full_upd);
static DEVICE_ATTR(update_disable, 0664, NULL, s1d13522fb_update_disable);
static DEVICE_ATTR(update_area, 0664, NULL, s1d13522fb_set_update_area);
static DEVICE_ATTR(temperature, 0664, s1d13522fb_read_temp,
				      s1d13522fb_store_temp);
static DEVICE_ATTR(splash_timeout, 0664, s1d13522fb_read_splash,
					 s1d13522fb_store_splash);
static DEVICE_ATTR(dithering_select, 0664, s1d13522fb_show_dithering,
					   s1d13522fb_set_dithering);
static DEVICE_ATTR(statistic, 0664, s1d13522fb_show_statistic, NULL);

static struct bin_attribute dev_attr_rw_raw_data = {
	.attr.name = "raw_data",
	.attr.mode = 0664,
	.read = s1d13522fb_raw_data_read,
	.write = s1d13522fb_raw_data_write
};


int s1d13522fb_register_sysfs(struct fb_info *info)
{
	int r;

	r = device_create_file(info->dev, &dev_attr_clear_artifacts);
	if (r)
		goto fail0;

	r = device_create_file(info->dev, &dev_attr_update_disable);
	if (r)
		goto fail1;

	r = device_create_file(info->dev, &dev_attr_update_area);
	if (r)
		goto fail2;

	r = device_create_file(info->dev, &dev_attr_temperature);
	if (r)
		goto fail3;

	r = device_create_file(info->dev, &dev_attr_splash_timeout);
	if (r)
		goto fail4;

	r = device_create_file(info->dev, &dev_attr_dithering_select);
	if (r)
		goto fail5;

	r = device_create_file(info->dev, &dev_attr_statistic);
	if (r)
		goto fail6;

	dev_attr_rw_raw_data.size = info->var.xres * info->var.yres;
	r = device_create_bin_file(info->dev, &dev_attr_rw_raw_data);
	if (r)
		goto fail7;

	return 0;
fail7:
	device_remove_file(info->dev, &dev_attr_statistic);
fail6:
	device_remove_file(info->dev, &dev_attr_dithering_select);
fail5:
	device_remove_file(info->dev, &dev_attr_splash_timeout);
fail4:
	device_remove_file(info->dev, &dev_attr_temperature);
fail3:
	device_remove_file(info->dev, &dev_attr_update_area);
fail2:
	device_remove_file(info->dev, &dev_attr_update_disable);
fail1:
	device_remove_file(info->dev, &dev_attr_clear_artifacts);
fail0:
	dev_err(info->dev, "unable to register sysfs interface\n");
	return r;
}


void s1d13522fb_unregister_sysfs(struct fb_info *info)
{
	device_remove_file(info->dev, &dev_attr_clear_artifacts);
	device_remove_file(info->dev, &dev_attr_update_disable);
	device_remove_file(info->dev, &dev_attr_update_area);
	device_remove_file(info->dev, &dev_attr_temperature);
	device_remove_file(info->dev, &dev_attr_splash_timeout);
	device_remove_file(info->dev, &dev_attr_dithering_select);
	device_remove_file(info->dev, &dev_attr_statistic);
	device_remove_bin_file(info->dev, &dev_attr_rw_raw_data);
}
