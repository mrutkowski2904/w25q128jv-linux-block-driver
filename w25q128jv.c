#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of.h>

#include "w25q128jv.h"

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

static int w25q128jv_probe(struct spi_device *client)
{
    int status;
    struct device_data *dev_data;

    dev_data = devm_kzalloc(&client->dev, sizeof(struct device_data), GFP_KERNEL);
    if (!dev_data)
        return -ENOMEM;

    spi_set_drvdata(client, dev_data);
    dev_data->client = client;

    dev_info(&client->dev, "probe called");
    return 0;
}

static void w25q128jv_remove(struct spi_device *client)
{
    dev_info(&client->dev, "remove called");
}

module_spi_driver(w25q128jv_spi_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maciej Rutkowski");
MODULE_DESCRIPTION("SPI driver for ILI9341 display");