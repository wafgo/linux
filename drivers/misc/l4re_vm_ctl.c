#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/l4re_vm_ctl.h>
#include <linux/printk.h>

struct vmctl {
	struct Mcm_cmd_header __iomem *header;
	struct Mcm_cmd_client __iomem *hv_data;
	struct Mcm_cmd_client __iomem *alpha_data;
};

static void write_status(struct vmctl *vmctl_data, uint32_t val)
{
	BUILD_BUG_ON(sizeof(vmctl_data->hv_data->status) != 4);
	writel(val, &vmctl_data->hv_data->status);
}

static void write_command_request(struct vmctl *vmctl_data, uint32_t val, uint32_t flag)
{
	BUILD_BUG_ON(sizeof(vmctl_data->hv_data->command) != 4);
	BUILD_BUG_ON(sizeof(vmctl_data->alpha_data->command) != 4);

	if (flag == ALPHA_VM_FLAG) {
		writel(val, &vmctl_data->alpha_data->command);
	}
	else if (flag == HV_FLAG) {
		writel(val, &vmctl_data->hv_data->command);
	}
	pr_info(" write_command_request val: '%d'\n", val);
}

static uint32_t read_command_request(struct vmctl *vmctl_data)
{
	return readl(&vmctl_data->hv_data->command);
}

static ssize_t set_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct vmctl *vmctl_data = platform_get_drvdata(pdev);
	vmctl_data->header->api_version = 1;
	strcpy(vmctl_data->header->magic, MCM_CMD_HEADER_MAGIC);

	if (!strncmp("HV_STATUS", buf, HV_STATUS_LEN))
		write_status(vmctl_data, MCM_HV_STATUS_UNKNOWN);
	else
		pr_err("Unknown command '%.*s'\n", (int)count, buf);

	return count;
}

static DEVICE_ATTR_WO(set_status);

static ssize_t command_request_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct vmctl *vmctl_data = platform_get_drvdata(pdev);
	vmctl_data->header->api_version = 1;
	strcpy(vmctl_data->header->magic, MCM_CMD_HEADER_MAGIC);

	if (!strncmp("EMMC_STOP", buf, EMMC_STOP_LEN)){
		write_command_request(
			vmctl_data, MCM_CMD_EMMCSTOP,HV_FLAG);
	}
	else if (!strncmp("KILL_VMS", buf, KILL_VMS_LEN)){
		write_command_request(
                        vmctl_data, MCM_CMD_KILL,HV_FLAG);
	}
	else if (!strncmp("ALPHA", buf, ALPHA_LEN)){
		write_command_request(
                        vmctl_data, MCM_CMD_KILL, ALPHA_VM_FLAG);
	}
        else if (!strncmp("LAUNCH", buf, LAUNCH_LEN)){
                write_command_request(
                        vmctl_data, MCM_CMD_LAUNCH, ALPHA_VM_FLAG);
        }	
	else{
		pr_err("Unknown command '%.*s'\n", (int)count, buf);
		pr_err("VM's test\n");
	}

	return count;
}

static DEVICE_ATTR_WO(command_request);

static struct attribute *attrs[] = {
	&dev_attr_set_status.attr,
	&dev_attr_command_request.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
	NULL,
};

static int vmctl_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct vmctl *vms;
	int retval;
	dev_warn(dev, "Probe called for vmctl \n");
	vms = devm_kzalloc(dev, sizeof(*vms), GFP_KERNEL);
	if (!vms)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		retval = -ENODEV;
		goto out_kfree;
	}

	if (!devm_request_mem_region(dev, res->start, resource_size(res),
				     dev_name(dev))) {
		dev_warn(dev, "could not reserve region %pR\n", res);
		retval = -EBUSY;
		goto out_kfree;
	}

	vms->header = ioremap_cache(res->start, resource_size(res));
	if (IS_ERR(vms->header)) {
		pr_err("ioremap_cache Err: header '%lld','%lld' \n", res->start, resource_size(res));
		retval = PTR_ERR(vms->header);
		goto out_release;
	}
	/* Mcm_cmd_client structures, which are spaced by 0x20 */
	vms->hv_data = ioremap_cache(res->start + 0x20, resource_size(res));
	if (IS_ERR(vms->hv_data)) {
		pr_err("ioremap_cache Err: data '%lld'\n", res->start);
		retval = PTR_ERR(vms->hv_data);
		goto out_release;
	}
	/* Mcm_cmd_client structures, which are spaced by 0x20 */
        vms->alpha_data = ioremap_cache(res->start + 0x40, resource_size(res));
        if (IS_ERR(vms->alpha_data)) {
                pr_err("ioremap_cache Err: data '%lld'\n", res->start);
                retval = PTR_ERR(vms->alpha_data);
                goto out_release;
        }	
	retval = sysfs_create_group(&pdev->dev.kobj, &attr_group);
	if (retval) {
		dev_err(dev, "Unable to export interface, error: %d\n", retval);
		goto out_iounmap;
	}

	// Add a convenience link
	retval = sysfs_create_link(NULL, &pdev->dev.kobj, "vm_ctl");
	if (retval)
		dev_warn(dev, "Could not create 'vmctl' sysfs link\n");

	platform_set_drvdata(pdev, vms);
	dev_warn(dev, "Probe finished for vmctl \n");

	return 0;

out_iounmap:
	iounmap(vms->header);
	iounmap(vms->hv_data);
	iounmap(vms->alpha_data);
out_release:
	devm_release_mem_region(dev, res->start, resource_size(res));
out_kfree:
	devm_kfree(dev, vms);
	return retval;
}

static int vmctl_remove(struct platform_device *pdev)
{
	struct vmctl *vmctl_data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res) {
		devm_kfree(dev, vmctl_data);
		return -ENODEV;
	}
	iounmap(vmctl_data->header);
	iounmap(vmctl_data->hv_data);
	iounmap(vmctl_data->alpha_data);
	devm_release_mem_region(dev, res->start, resource_size(res));
	devm_kfree(dev, vmctl_data);

	return 0;
}


static const struct of_device_id vmctl_of_match[] = {
	{ .compatible = "l4,psm" },
	{},
};
MODULE_DEVICE_TABLE(of, vmctl_of_match);

static struct platform_driver vmctl_driver = {
	.probe  = vmctl_probe,
	.remove = vmctl_remove,
	.driver = {
		.name = "vm_ctl",
		.of_match_table = vmctl_of_match,
	},
};
module_platform_driver(vmctl_driver);

MODULE_AUTHOR("Adam Lackorzynski <adam@l4re.org>");
MODULE_DESCRIPTION("VM-Ctl Device");
MODULE_LICENSE("GPL v2");
