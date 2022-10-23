#ifndef W25Q128_H
#define W25Q128_H

#include <stdint.h>
#include "spi.h"

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *nss_port;
    uint16_t nss_pin;

    uint8_t manu_id;
    uint8_t dev_id;

    uint32_t page_size;
    uint32_t sector_size;
    uint32_t block_size;
    uint32_t flash_size;
} w25q128jv_t;

typedef enum {
    W25Q128JV_OK = 0,
    W25Q128JV_ERR = -1,
} w25q128jv_status_t;

w25q128jv_status_t w25q128jv_init(w25q128jv_t *w, SPI_HandleTypeDef *hspi, GPIO_TypeDef *nss_port, uint16_t nss_pin);

w25q128jv_status_t w25q128jv_read(w25q128jv_t *w, uint32_t addr, uint8_t *buf, uint32_t len);
w25q128jv_status_t w25q128jv_write(w25q128jv_t *w, uint32_t addr, uint8_t *buf, uint32_t len);
w25q128jv_status_t w25q128jv_erase_sector(w25q128jv_t *w, uint32_t addr, uint32_t len);

#endif
