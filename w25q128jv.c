#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>

#include "w25q128jv.h"

static int w25q128jv_open(struct block_device *dev, fmode_t mode);
static void w25q128jv_release(struct gendisk *gdisk, fmode_t mode);
int w25q128jv_ioctl(struct block_device *bdev, fmode_t mode, unsigned cmd, unsigned long arg);

static blk_status_t queue_request(struct blk_mq_hw_ctx *ctx, const struct blk_mq_queue_data *data);

static int w25q128jv_probe(struct spi_device *client);
static void w25q128jv_remove(struct spi_device *client);

static struct of_device_id w25q128jv_driver_of_ids[] = {
    {
        .compatible = "mr,custom_w25q128jv",
    },
    {},
};
MODULE_DEVICE_TABLE(of, w25q128jv_driver_of_ids);

static struct spi_device_id w25q128jv_ids[] = {
    {"custom_w25q128jv", 0},
    {},
};
MODULE_DEVICE_TABLE(spi, w25q128jv_ids);

static struct spi_driver w25q128jv_spi_driver = {
    .probe = w25q128jv_probe,
    .remove = w25q128jv_remove,
    .driver = {
        .name = "custom_w25q128jv",
        .of_match_table = of_match_ptr(w25q128jv_driver_of_ids),
    },
};

static struct block_device_operations block_dev_ops = {
    .owner = THIS_MODULE,
    .open = w25q128jv_open,
    .release = w25q128jv_release,
};

static struct blk_mq_ops block_mq_ops = {
    .queue_rq = queue_request,
};

struct driver_data drv_data;

static int w25q128jv_open(struct block_device *dev, fmode_t mode)
{
    return 0;
}

static void w25q128jv_release(struct gendisk *gdisk, fmode_t mode)
{
}

int w25q128jv_ioctl(struct block_device *bdev, fmode_t mode, unsigned cmd, unsigned long arg)
{
    return -ENOTTY;
}

static blk_status_t queue_request(struct blk_mq_hw_ctx *ctx, const struct blk_mq_queue_data *data)
{
    return BLK_STS_OK;
}

static int w25q128jv_probe(struct spi_device *client)
{
    int status;
    struct device_data *dev_data;

    dev_data = devm_kzalloc(&client->dev, sizeof(struct device_data), GFP_KERNEL);
    if (!dev_data)
        return -ENOMEM;

    spi_set_drvdata(client, dev_data);
    dev_data->client = client;

    dev_data->tag_set.ops = &block_mq_ops;
	dev_data->tag_set.nr_hw_queues = 1;
	dev_data->tag_set.nr_maps = 1;
	dev_data->tag_set.queue_depth = 16;
	dev_data->tag_set.numa_node = NUMA_NO_NODE;
	dev_data->tag_set.flags = BLK_MQ_F_SHOULD_MERGE;
    status = blk_mq_alloc_tag_set(&dev_data->tag_set);
	if (status)
        return status;

    dev_data->disk = blk_mq_alloc_disk(&dev_data->tag_set, NULL);
    if (IS_ERR(dev_data->disk))
    {
        blk_mq_free_tag_set(&dev_data->tag_set);
		return PTR_ERR(dev_data->disk);
    }

    dev_data->disk->flags |= GENHD_FL_NO_PART;
    dev_data->disk->major = drv_data.major;
    dev_data->disk->first_minor = drv_data.device_no;
    dev_data->disk->minors = 1;
    dev_data->disk->fops = &block_dev_ops;
    dev_data->disk->private_data = dev_data;
    sprintf(dev_data->disk->disk_name, W25Q128JV_DISK_NAME);
    set_capacity(dev_data->disk, W25Q128JV_CAPACITY);

    status = add_disk(dev_data->disk);
    if(status)
    {
        blk_mq_free_tag_set(&dev_data->tag_set);
        put_disk(dev_data->disk);
        return status;
    }

    drv_data.device_no++;
    dev_info(&client->dev, "device inserted");
    return 0;
}

static void w25q128jv_remove(struct spi_device *client)
{
    struct device_data *dev_data;
	dev_data = spi_get_drvdata(client);

    if(dev_data->disk)
    {
        del_gendisk(dev_data->disk);
        blk_mq_free_tag_set(&dev_data->tag_set);
    }
    dev_info(&client->dev, "device removed");
}

static int __init w25q128jv_driver_init(void)
{
    int status;
    drv_data.device_no = 0;
    drv_data.major = register_blkdev(0, W25Q128JV_DISK_NAME);

    status = spi_register_driver(&w25q128jv_spi_driver);
    if (status < 0)
        return status;

    return status;
}

static void __exit w25q128jv_driver_exit(void)
{
    spi_unregister_driver(&w25q128jv_spi_driver);
    unregister_blkdev(drv_data.major, W25Q128JV_DISK_NAME);
}

module_init(w25q128jv_driver_init);
module_exit(w25q128jv_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maciej Rutkowski");
MODULE_DESCRIPTION("SPI driver for ILI9341 display");