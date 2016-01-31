#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#define VI_CONTROL_REG		0x00
#define VI_DRAM_ADDR_REG	0x04
#define VI_H_WIDTH_REG		0x08
#define VI_V_INTR_REG		0x0C
#define VI_V_CURRENT_LINE_REG	0x10
#define VI_TIMING_REG		0x14
#define VI_V_SYNC_REG		0x18
#define VI_H_SYNC_REG		0x1C
#define VI_H_SYNC_LEAP_REG	0x20
#define VI_H_VIDEO_REG		0x24
#define VI_V_VIDEO_REG		0x28
#define VI_V_BURST_REG		0x2C
#define VI_X_SCALE_REG		0x30
#define VI_Y_SCALE_REG		0x34

struct nus_par {
	u32 pseudo_palette[16];
	void __iomem *base;
};

static struct fb_fix_screeninfo nusfb_fix = {
	.id = "nusfb",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.accel = FB_ACCEL_NONE,
};

#define FB_SIZE			(640 * 480 * 2)

static struct fb_var_screeninfo nusfb_var = {
	.xres = 640,
	.yres = 480,
	.xres_virtual = 640,
	.yres_virtual = 480,
	.bits_per_pixel = 16,
	.red = {11, 5, 0},
	.green = {6, 5, 0},
	.blue = {1, 5, 0},
	.activate = FB_ACTIVATE_NOW,
	.height = -1,
	.width = -1,
	.vmode = FB_VMODE_INTERLACED
};

static inline unsigned int chan_to_field(unsigned int chan,
		struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int nusfb_setcolreg(unsigned regno, unsigned red, unsigned green,
		unsigned blue, unsigned transp, struct fb_info *info)
{
	u32 *pal = info->pseudo_palette;
	unsigned int val;
	val = chan_to_field(red, &info->var.red);
	val |= chan_to_field(green, &info->var.green);
	val |= chan_to_field(blue, &info->var.blue);
	pal[regno] = val;
	return 0;
}

static struct fb_ops nusfb_ops = {
	.owner = THIS_MODULE,
	//.fb_open = nusfb_open,
	//.fb_read = nusfb_read,
	//.fb_write = nusfb_write,
	//.fb_release = nusfb_release,
	//.fb_check_var = nusfb_check_var,
	//.fb_set_par = nusfb_set_par,
	.fb_setcolreg = nusfb_setcolreg,
	//.fb_blank = nusfb_blank,
	//.fb_pan_display = nusfb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	//.fb_cursor = nusfb_cursor,
	//.fb_rotate = nusfb_rotate,
	//.fb_sync = nusfb_sync,
	//.fb_ioctl = nusfb_ioctl,
	//.fb_mmap = nusfb_mmap,
};

static int nusfb_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	struct nus_par *par;
	struct device *device = &pdev->dev;
	int error;

	//struct resource *res;
	void __iomem *base;
	void *fb_virt;
	phys_addr_t fb_phys;

	/*res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);*/
	base = ioremap(0x04400000, 0x40);
	if (IS_ERR(base)) {
		error = PTR_ERR(base);
		goto err_ioremap_resource;
	}

	info = framebuffer_alloc(sizeof(struct nus_par), device);
	if (!info) {
		error = -ENOMEM;
		goto err_framebuffer_alloc;
	}

	par = info->par;
	par->base = base;

	// FIXME: Do other video standards and modes

	// NTSC 640x480
	__raw_writel(0x00013252, base + VI_CONTROL_REG);
	__raw_writel(0x00000280, base + VI_H_WIDTH_REG);
	__raw_writel(0x00000200, base + VI_V_INTR_REG);
	__raw_writel(0x03e52239, base + VI_TIMING_REG);
	__raw_writel(0x0000020c, base + VI_V_SYNC_REG);
	__raw_writel(0x00000c15, base + VI_H_SYNC_REG);
	__raw_writel(0x0c150c15, base + VI_H_SYNC_LEAP_REG);
	__raw_writel(0x006c02ec, base + VI_H_VIDEO_REG);
	__raw_writel(0x002301fd, base + VI_V_VIDEO_REG);
	__raw_writel(0x000e0204, base + VI_V_BURST_REG);
	__raw_writel(0x00000400, base + VI_X_SCALE_REG);
	__raw_writel(0x02000800, base + VI_Y_SCALE_REG);

	fb_virt = alloc_pages_exact(FB_SIZE, GFP_KERNEL | __GFP_ZERO);
	if (!fb_virt) {
		error = -ENOMEM;
		goto err_screen_alloc;
	}
	fb_phys = virt_to_phys(fb_virt);

	__raw_writel(fb_phys | 0xa0000000, base + VI_DRAM_ADDR_REG);

	info->screen_base = fb_virt;
	info->screen_size = FB_SIZE;
	info->fbops = &nusfb_ops;
	info->fix = nusfb_fix;
	info->fix.smem_start = fb_phys;
	info->fix.smem_len = FB_SIZE;
	info->fix.line_length = nusfb_var.xres_virtual * 2;
	info->pseudo_palette = par->pseudo_palette;
	info->flags = FBINFO_DEFAULT;
	info->var = nusfb_var;

	info->apertures = alloc_apertures(1);
	if (!info->apertures) {
		error = -ENOMEM;
		goto err_aperture_alloc;
	}
	info->apertures->ranges[0].base = fb_phys;
	info->apertures->ranges[0].size = FB_SIZE;

	if (fb_alloc_cmap(&info->cmap, 256, 0) < 0) {
		error = -ENOMEM;
		goto err_cmap_alloc;
	}

	if (register_framebuffer(info) < 0) {
		error = -EINVAL;
		goto err_framebuffer_register;
	}

	fb_info(info, "%s frame buffer device\n", info->fix.id);
	platform_set_drvdata(pdev, info);

	return 0;

err_framebuffer_register:
	fb_dealloc_cmap(&info->cmap);
err_cmap_alloc:
err_aperture_alloc:
	free_pages_exact(fb_virt, FB_SIZE);
err_screen_alloc:
	framebuffer_release(info);
err_framebuffer_alloc:
err_ioremap_resource:
	return error;
}

static int nusfb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);

	if (info) {
		unregister_framebuffer(info);
		free_pages_exact(info->screen_base, info->screen_size);
		framebuffer_release(info);
	}

	return 0;
}

static struct platform_driver nusfb_driver = {
	.probe = nusfb_probe,
	.remove = nusfb_remove,
	.driver = {
		.name = "nusfb",
	},
};

static struct platform_device *nusfb_device;

#ifndef MODULE
static int __init nusfb_setup(char *options)
{
	return 0;
}
#endif

static int __init nusfb_init(void)
{
	int ret;
#ifndef MODULE
	char *option = NULL;

	if (fb_get_options("nusfb", &option))
		return -ENODEV;
	nusfb_setup(option);
#endif
	ret = platform_driver_register(&nusfb_driver);

	if (!ret) {
		nusfb_device = platform_device_register_simple("nusfb", 0,
								NULL, 0);

		if (IS_ERR(nusfb_device)) {
			platform_driver_unregister(&nusfb_driver);
			ret = PTR_ERR(nusfb_device);
		}
	}

	return ret;
}

static void __exit nusfb_exit(void)
{
	platform_device_unregister(nusfb_device);
	platform_driver_unregister(&nusfb_driver);
}

module_init(nusfb_init);
module_exit(nusfb_exit);

MODULE_LICENSE("GPL");
