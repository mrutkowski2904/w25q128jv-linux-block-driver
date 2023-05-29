#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>
#include <linux/blk-mq.h>
#include <linux/hdreg.h>

/* register map */
#define W25Q128JV_REG_ID 0x90

#define W25Q128JV_REG_ID_MANUFACTURER_EXPECTED_VAL 0xEF

#define W25Q128JV_DISK_NAME "w25q128jv"
#define W25Q128JV_SECTOR_SIZE 4096UL

/* 1MiB */
#define W25Q128JV_SECTORS 256UL

struct device_data
{
    u8 *buff;
    struct spi_device *client;

    struct gendisk *disk;
    struct blk_mq_tag_set tag_set;
    struct request_queue *req_queue;
};

struct driver_data
{
    int major;
    int device_no;
};

/* hardware io */
static bool w25q128jv_is_id_ok(struct device_data *dev_data);

static int w25q128jv_open(struct block_device *dev, fmode_t mode);
static void w25q128jv_release(struct gendisk *gdisk, fmode_t mode);

static blk_status_t handle_request(struct request *req, unsigned int *bytes_processed);
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

/* driver wide data */
struct driver_data drv_data;

static bool w25q128jv_is_id_ok(struct device_data *dev_data)
{
    struct spi_message id_read_message;
    struct spi_transfer transfers[3];
    u8 cmd;
    u8 addr[3];
    u8 in[2];

    cmd = W25Q128JV_REG_ID;
    memset(addr, 0, 3);
    memset(transfers, 0, 3 * sizeof(struct spi_transfer));

    transfers[0].tx_buf = &cmd;
    transfers[0].len = 1;

    transfers[1].tx_buf = addr;
    transfers[1].len = 3;

    transfers[2].rx_buf = in;
    transfers[2].len = 2;

    spi_message_init(&id_read_message);
    for(int i = 0; i < 3; i++)
        spi_message_add_tail(&transfers[i], &id_read_message);

    if(spi_sync(dev_data->client, &id_read_message))
    {
        dev_err(&dev_data->client->dev, "spi transfer error, status: %d\n", id_read_message.status);
        return false;
    }

    return (in[0] == W25Q128JV_REG_ID_MANUFACTURER_EXPECTED_VAL);
}

static int w25q128jv_open(struct block_device *dev, fmode_t mode)
{
    return 0;
}

static void w25q128jv_release(struct gendisk *gdisk, fmode_t mode)
{
}

static blk_status_t handle_request(struct request *req, unsigned int *bytes_processed)
{
    blk_status_t status = BLK_STS_OK;
    struct bio_vec vector;
    struct req_iterator iterator;
    struct device_data *dev_data = req->q->queuedata;
    unsigned long seg_len;
    void *seg_data;
    loff_t pos;
    loff_t dev_size = (loff_t)(W25Q128JV_SECTORS * W25Q128JV_SECTOR_SIZE);

    /* blk_rq_pos(req) - 512 byte sector of request */
    pos = blk_rq_pos(req) << SECTOR_SHIFT;

    /* non FS request */
    if (blk_rq_is_passthrough(req))
    {
        status = BLK_STS_IOERR;
        return status;
    }

    /*
    pr_info("------------------\nw25q128jv: request, sector: %llu, addr: %llu, all bytes: %llu\n", blk_rq_pos(req), pos, blk_rq_bytes(req));
    */

    /* go through all segments of request */
    rq_for_each_segment(vector, req, iterator)
    {
        seg_data = page_address(vector.bv_page) + vector.bv_offset;
        seg_len = vector.bv_len;

        if ((pos + seg_len) > dev_size)
            seg_len = dev_size - pos;

        if (rq_data_dir(req) == WRITE)
        {
            /* pr_info("w25q128jv: seg write, pos: %llu, size: %llu\n", pos, seg_len); */
            memcpy(dev_data->buff + pos, seg_data, seg_len);
        }
        else
        {
            /* pr_info("w25q128jv: seg read, pos: %llu, size: %llu\n", pos, seg_len); */
            memcpy(seg_data, dev_data->buff + pos, seg_len);
        }

        pos += seg_len;
        *bytes_processed += seg_len;
    }

    return status;
}

static blk_status_t queue_request(struct blk_mq_hw_ctx *ctx, const struct blk_mq_queue_data *data)
{
    blk_status_t status;
    unsigned int bytes_processed = 0;
    struct request *req = data->rq;

    blk_mq_start_request(req);
    status = handle_request(req, &bytes_processed);
    blk_update_request(req, status, bytes_processed);
    blk_mq_end_request(req, status);
    return status;
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

    /* temporary buffer for testing */
    dev_data->buff = devm_kzalloc(&client->dev, W25Q128JV_SECTORS * W25Q128JV_SECTOR_SIZE, GFP_KERNEL);
    if (!dev_data->buff)
        return -ENOMEM;

    if(!w25q128jv_is_id_ok(dev_data))
    {
        dev_err(&client->dev, "invalid manufacturer id\n");
        return -EINVAL;
    }

    dev_data->tag_set.ops = &block_mq_ops;
    dev_data->tag_set.nr_hw_queues = 1;
    dev_data->tag_set.nr_maps = 1;
    dev_data->tag_set.queue_depth = 16;
    dev_data->tag_set.numa_node = NUMA_NO_NODE;
    dev_data->tag_set.flags = BLK_MQ_F_SHOULD_MERGE | BLK_MQ_F_BLOCKING;
    status = blk_mq_alloc_tag_set(&dev_data->tag_set);
    if (status)
        return status;

    dev_data->disk = blk_mq_alloc_disk(&dev_data->tag_set, dev_data);
    if (IS_ERR(dev_data->disk))
    {
        blk_mq_free_tag_set(&dev_data->tag_set);
        return PTR_ERR(dev_data->disk);
    }
    blk_queue_physical_block_size(dev_data->disk->queue, W25Q128JV_SECTOR_SIZE);
    blk_queue_max_segments(dev_data->disk->queue, 1);
    blk_queue_io_min(dev_data->disk->queue, W25Q128JV_SECTOR_SIZE);

    dev_data->disk->flags |= GENHD_FL_NO_PART;
    dev_data->disk->major = drv_data.major;
    dev_data->disk->first_minor = drv_data.device_no;
    dev_data->disk->minors = 1;
    dev_data->disk->fops = &block_dev_ops;
    dev_data->disk->private_data = dev_data;
    sprintf(dev_data->disk->disk_name, W25Q128JV_DISK_NAME);
    set_capacity(dev_data->disk, W25Q128JV_SECTORS * (W25Q128JV_SECTOR_SIZE / SECTOR_SIZE));

    status = add_disk(dev_data->disk);
    if (status)
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

    if (dev_data->disk)
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