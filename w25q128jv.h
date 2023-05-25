#ifndef _W25Q128JV_H
#define _W25Q128JV_H

#include <linux/types.h>
#include <linux/spi/spi.h>

struct device_data
{
    struct spi_device *client;
};

#endif /* _W25Q128JV_H */