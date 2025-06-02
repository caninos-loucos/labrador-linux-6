
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pm_runtime.h>


#include <asm/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/suspend.h>

#include "aotg_hcd.h"

static void aotg_hcd_init(struct aotg_hcd *acthcd)
{
	int i;
	
	/*init software state */
	spin_lock_init(&acthcd->lock);
	spin_lock_init(&acthcd->tasklet_lock);
	acthcd->tasklet_retry = 0;
	
	acthcd->port = 0;
	acthcd->rhstate = AOTG_RH_POWEROFF;
	acthcd->inserted = 0;
	
	INIT_LIST_HEAD(&acthcd->hcd_enqueue_list);
	INIT_LIST_HEAD(&acthcd->hcd_dequeue_list);
	INIT_LIST_HEAD(&acthcd->hcd_finished_list);
	
	acthcd->active_ep0 = NULL;
	
	aotg_hcep_pool_init(acthcd);
	
	acthcd->fifo_map[0] = BIT(31);
	acthcd->fifo_map[1] = BIT(31) | ALLOC_FIFO_UNIT;
	
	for (i = 2; i < AOTG_MAX_FIFO_MAP_CNT; i++) {
		acthcd->fifo_map[i] = 0;
	}
	
	for (i = 0; i < AOTG_QUEUE_POOL_CNT; i++) {
		acthcd->queue_pool[i] = NULL;
	}
	
	acthcd->put_aout_msg = 0;
	acthcd->discon_happened = 0;
	
	tasklet_init(&acthcd->urb_tasklet, urb_tasklet_func, (unsigned long)acthcd);
	timer_setup(&acthcd->trans_wait_timer, aotg_hub_trans_wait_timer, 0);
	timer_setup(&acthcd->check_trb_timer, aotg_check_trb_timer, 0);
	
	hrtimer_init(&acthcd->hotplug_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	acthcd->hotplug_timer.function = aotg_hub_hotplug_timer;
}

static int caninos_hcd_probe(struct platform_device *pdev)
{
	enum caninos_hw_model model;
	struct aotg_hcd *acthcd;
	struct resource *res;
	struct usb_hcd *hcd;
	int retval;
	
	if (usb_disabled()) {
		dev_err(&pdev->dev, "usb is disabled, hcd probe canceled\n");
		return -ENODEV;
	}
	
	model = (enum caninos_hw_model)of_device_get_match_data(&pdev->dev);
	
	if (model != CANINOS_HW_MODEL_K7 && model != CANINOS_HW_MODEL_K5) {
		dev_err(&pdev->dev, "could not get hardware model\n");
		return -ENODEV;
	}
	
	retval = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	
	if (retval) {
		dev_err(&pdev->dev, "could not set dma mask, %d\n", retval);
		return retval;
	}
	
	hcd = caninos_usb_create_hcd(&pdev->dev);
	
	if (!hcd) {
		dev_err(&pdev->dev, "usb create hcd failed\n");
		return -ENOMEM;
	}
	
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "base");
	
	if (!res)
	{
		dev_err(&pdev->dev, "failed to get base resource\n");
		usb_put_hcd(hcd);
		return -EINVAL;
	}
	
	acthcd = hcd_to_aotg(hcd);
	aotg_hcd_init(acthcd);
	
	acthcd->hcd = hcd;
	acthcd->model = model;
	acthcd->dev = &pdev->dev;
	acthcd->hcd_exiting = 0;
	acthcd->rsrc_start = res->start;
	acthcd->rsrc_len = resource_size(res);
	
	acthcd->base = devm_ioremap_resource(&pdev->dev, res);
	
	if (IS_ERR(acthcd->base))
	{
		retval = PTR_ERR(acthcd->base);
		dev_err(&pdev->dev, "failed to ioremap base resource, %d\n", retval);
		usb_put_hcd(hcd);
		return retval;
	}
	
	acthcd->uhc_irq = platform_get_irq(pdev, 0);
	
	if (acthcd->uhc_irq <= 0) {
		dev_err(&pdev->dev, "could not get irq\n");
		usb_put_hcd(hcd);
		return -ENODEV;
	}
	
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "usbecs");
	
	if (!res)
	{
		dev_err(&pdev->dev, "failed to get usbecs resource\n");
		usb_put_hcd(hcd);
		return -EINVAL;
	}
	
	acthcd->usbecs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	
	if (!acthcd->usbecs)
	{
		dev_err(&pdev->dev, "failed to ioremap usbecs resource\n");
		usb_put_hcd(hcd);
		return -ENOMEM;
	}
	
	acthcd->rst = devm_reset_control_get(&pdev->dev, NULL);
	
	if (!acthcd->rst)
	{
		dev_err(&pdev->dev, "could not get device reset control.\n");
		usb_put_hcd(hcd);
		return -ENODEV;
	}
	
	acthcd->clk_usbh_pllen = devm_clk_get(&pdev->dev, "pllen");
	
	if (IS_ERR(acthcd->clk_usbh_pllen))
	{
		retval = PTR_ERR(acthcd->clk_usbh_pllen);
		dev_err(&pdev->dev, "could not get pllen clk, %d\n", retval);
		usb_put_hcd(hcd);
		return retval;
	}
	
	acthcd->clk_usbh_phy = devm_clk_get(&pdev->dev, "phy");
	
	if (IS_ERR(acthcd->clk_usbh_phy))
	{
		retval = PTR_ERR(acthcd->clk_usbh_phy);
		dev_err(&pdev->dev, "could not get phy clk, %d\n", retval);
		usb_put_hcd(hcd);
		return retval;
	}
	
	acthcd->clk_usbh_cce = devm_clk_get(&pdev->dev, "cce");
	
	if (IS_ERR(acthcd->clk_usbh_cce))
	{
		retval = PTR_ERR(acthcd->clk_usbh_cce);
		dev_err(&pdev->dev, "could not get cce clk, %d\n", retval);
		usb_put_hcd(hcd);
		return retval;
	}
	
	retval = caninos_usb_add_hcd(hcd);
	
	if (retval)
	{
		usb_put_hcd(hcd);
		return retval;
	}
	return 0;
}

static int caninos_hcd_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	
	caninos_usb_hcd_remove(hcd);
	
	return 0;
}

static int __maybe_unused caninos_suspend(struct device *dev)
{
	dev_info(dev, "caninos_suspend() called\n");
	return 0;
}

static int __maybe_unused caninos_resume(struct device *dev)
{
	dev_info(dev, "caninos_resume() called\n");
	return 0;
}

static SIMPLE_DEV_PM_OPS(caninos_pm_ops, caninos_suspend, caninos_resume);

static const struct of_device_id caninos_hcd_dt_id[] = {
	{.compatible = "caninos,k7-usb2.0-0", .data = (void*)CANINOS_HW_MODEL_K7},
	{.compatible = "caninos,k7-usb2.0-1", .data = (void*)CANINOS_HW_MODEL_K7},
	{.compatible = "caninos,k5-usb2.0-0", .data = (void*)CANINOS_HW_MODEL_K5},
	{.compatible = "caninos,k5-usb2.0-1", .data = (void*)CANINOS_HW_MODEL_K5},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, caninos_hcd_dt_id);

static struct platform_driver caninos_hcd_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = caninos_hcd_dt_id,
		.owner = THIS_MODULE,
		.pm = &caninos_pm_ops,
	},
	.probe = caninos_hcd_probe,
	.remove = caninos_hcd_remove,
	.shutdown = usb_hcd_platform_shutdown,
};

static int __init caninos_usb_module_init(void)
{
	int ret = aotg_kmem_cache_create();
	
	if (ret) {
		return ret;
	}
	
	ret = platform_driver_register(&caninos_hcd_driver);
	
	if (ret) {
		aotg_kmem_cache_destroy();
	}
	return ret;
}

module_init(caninos_usb_module_init);

static void __exit caninos_usb_module_exit(void)
{
	platform_driver_unregister(&caninos_hcd_driver);
	
	aotg_kmem_cache_destroy();
}

module_exit(caninos_usb_module_exit);

MODULE_AUTHOR("Edgar Bernardi Righi <edgar.righi@lsitec.org.br>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
