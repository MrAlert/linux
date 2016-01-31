#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/bootinfo.h>
#include <asm/io.h>
#include <asm/irq_cpu.h>
#include <asm/time.h>

const char *get_system_type(void)
{
	return "Nintendo 64";
}


void __init arch_init_irq(void)
{
	mips_cpu_irq_init();
}

void __init prom_init(void)
{
}

void prom_free_prom_memory(void)
{
}


void __init plat_mem_setup(void)
{
	/* FIXME: actually detect memory */
	add_memory_region(0x0, 0x800000, BOOT_MEM_RAM);
}

void __init plat_time_init(void)
{
	mips_hpt_frequency = 93750000 / 2;
	/* clear the screen */
	//writel(0, ioremap(0x04400000, 0x4));
}

/*static struct resource nusfb_resource = {
	.start = 0x04400000,
	.end = 0x044fffff,
	.flags = IORESOURCE_MEM
};

static struct platform_device nusfb_device = {
	.name = "nusfb",
	.id = 0,
	.num_resources = 1,
	.resource = &nusfb_resource
};

static struct platform_device *nus_devices[] __initdata = {
	&nusfb_device
};

static int __init nus_dev_init(void)
{
	return platform_add_devices(nus_devices, ARRAY_SIZE(nus_devices));
}
device_initcall(nus_dev_init);*/
