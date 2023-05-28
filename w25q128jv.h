#ifndef _W25Q128JV_H
#define _W25Q128JV_H

#include <linux/types.h>
#include <linux/spi/spi.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>

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

#endif /* _W25Q128JV_H */