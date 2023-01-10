#include <string.h>
#include "w25q128jv.h"
#include "spi.h"
#include "util.h"

#define MANU_ID_WINBOND  0xEF
#define DEV_ID_W25Q128JV 0x17

#define PAGE_SIZE   256
#define SECTOR_SIZE 4096
#define BLOCK_SIZE  65536
#define FLASH_SIZE  16777216

#define PAGE_MASK   0xFFFF00
#define SECTOR_MASK 0xFFF000
#define BLOCK_MASK  0xFF0000

#define INST_WRITE_ENABLE    0x06
#define INST_MANU_DEV_ID     0x90

#define INST_READ            0x03
#define INST_PAGE_PROG       0x02
#define INST_SECTOR_ERASE    0x20

#define INST_READ_STAT_REG_1 0x05
#define INST_READ_STAT_REG_2 0x35
#define INST_READ_STAT_REG_3 0x15

#define STAT_REG_1_BUSY 0x01
#define STAT_REG_1_WEL  0x02
#define STAT_REG_1_BP0  0x04
#define STAT_REG_1_BP1  0x08
#define STAT_REG_1_BP2  0x10
#define STAT_REG_1_TB   0x20
#define STAT_REG_1_SEC  0x40
#define STAT_REG_1_SRP0 0x80

static w25q128jv_status_t w25q128jv_transmit(w25q128jv_t *w, uint8_t *buf, uint32_t len);
static w25q128jv_status_t w25q128jv_receive(w25q128jv_t *w, uint8_t *buf, uint32_t len);
static w25q128jv_status_t w25q128jv_transmit_receive(w25q128jv_t *w, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len);

static w25q128jv_status_t w25q128jv_read_id(w25q128jv_t *w);
static w25q128jv_status_t w25q128jv_write_enable(w25q128jv_t *w);
static w25q128jv_status_t w25q128jv_read_status_reg_1(w25q128jv_t *w, uint8_t *reg);
static w25q128jv_status_t w25q128jv_wait_busy(w25q128jv_t *w);

static void nss_low(w25q128jv_t *w);
static void nss_high(w25q128jv_t *w);

w25q128jv_status_t w25q128jv_init(w25q128jv_t *w, SPI_HandleTypeDef *hspi, GPIO_TypeDef *nss_port, uint16_t nss_pin) {
    w25q128jv_status_t res;

    w->hspi = hspi;
    w->nss_port = nss_port;
    w->nss_pin = nss_pin;

    w->page_size = PAGE_SIZE;
    w->sector_size = SECTOR_SIZE;
    w->block_size = BLOCK_SIZE;
    w->flash_size = FLASH_SIZE;

    res = w25q128jv_read_id(w);
    if (res == W25Q128JV_OK) {
        switch (w->manu_id) {
        case MANU_ID_WINBOND:
            switch (w->dev_id) {
            case DEV_ID_W25Q128JV:
                break;
            default:
                /* Unknown device. */
                res = W25Q128JV_ERR;
            }
            break;
        default:
            /* Unknown manufacturer. */
            res = W25Q128JV_ERR;
        }
    }

    return res;
}

w25q128jv_status_t w25q128jv_read(w25q128jv_t *w, uint32_t addr, uint8_t *buf, uint32_t len) {
    uint8_t tx_buf[4] = {INST_READ, BYTE_2(addr), BYTE_1(addr), BYTE_0(addr)};
    w25q128jv_status_t res = W25Q128JV_OK;

    if (w25q128jv_wait_busy(w) == W25Q128JV_OK) {
        nss_low(w);
        if (w25q128jv_transmit(w, tx_buf, sizeof(tx_buf)) != W25Q128JV_OK ||
            w25q128jv_receive(w, buf, len) != W25Q128JV_OK)
            res = W25Q128JV_ERR;
        nss_high(w);
    } else
        res = W25Q128JV_ERR;

    return res;
}

w25q128jv_status_t w25q128jv_write(w25q128jv_t *w, uint32_t addr, uint8_t *buf, uint32_t len) {
    uint32_t cur_addr, cur_len, remain_len, page_end_addr;
    w25q128jv_status_t res;

    cur_addr = addr;
    remain_len = len;
    res = W25Q128JV_OK;

    while (remain_len > 0) {
        page_end_addr = (cur_addr & PAGE_MASK) + PAGE_SIZE;
        cur_len = MIN(remain_len, page_end_addr - cur_addr);

        if (w25q128jv_wait_busy(w) == W25Q128JV_OK && w25q128jv_write_enable(w) == W25Q128JV_OK) {
            uint8_t tx_buf[4] = {INST_PAGE_PROG, BYTE_2(cur_addr), BYTE_1(cur_addr), BYTE_0(cur_addr)};

            nss_low(w);
            if (w25q128jv_transmit(w, tx_buf, sizeof(tx_buf)) != W25Q128JV_OK ||
                w25q128jv_transmit(w, buf + (len - remain_len), cur_len) != W25Q128JV_OK)
                res = W25Q128JV_ERR;
            nss_high(w);
        } else
            res = W25Q128JV_ERR;

        if (res != W25Q128JV_OK)
            break;

        cur_addr += cur_len;
        remain_len -= cur_len;
    }

    return res;
}

w25q128jv_status_t w25q128jv_erase_sector(w25q128jv_t *w, uint32_t addr, uint32_t len) {
    w25q128jv_status_t res = W25Q128JV_OK;

    if ((addr & SECTOR_MASK) == 0 && (len % SECTOR_SIZE) == 0)
        for (uint32_t i = 0; i < len / SECTOR_SIZE; i++) {
            if (w25q128jv_wait_busy(w) == W25Q128JV_OK && w25q128jv_write_enable(w) == W25Q128JV_OK) {
                uint32_t cur_addr = addr + i * SECTOR_SIZE;
                uint8_t tx_buf[4] = {INST_SECTOR_ERASE, BYTE_2(cur_addr), BYTE_1(cur_addr), BYTE_0(cur_addr)};

                nss_low(w);
                if (w25q128jv_transmit(w, tx_buf, sizeof(tx_buf)) != W25Q128JV_OK)
                    res = W25Q128JV_ERR;
                nss_high(w);
            } else
                res = W25Q128JV_ERR;

            if (res != W25Q128JV_OK)
                break;
        }
    else
        res = W25Q128JV_ERR;

    return res;
}

static w25q128jv_status_t w25q128jv_transmit(w25q128jv_t *w, uint8_t *buf, uint32_t len) {
    return HAL_SPI_Transmit(w->hspi, buf, len, HAL_MAX_DELAY) == HAL_OK ? W25Q128JV_OK : W25Q128JV_ERR;
}

static w25q128jv_status_t w25q128jv_receive(w25q128jv_t *w, uint8_t *buf, uint32_t len) {
    return HAL_SPI_Receive(w->hspi, buf, len, HAL_MAX_DELAY) == HAL_OK ? W25Q128JV_OK : W25Q128JV_ERR;
}

static w25q128jv_status_t w25q128jv_transmit_receive(w25q128jv_t *w, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len) {
    return HAL_SPI_TransmitReceive(w->hspi, tx_buf, rx_buf, len, HAL_MAX_DELAY) == HAL_OK ?
           W25Q128JV_OK :
           W25Q128JV_ERR;
}

static w25q128jv_status_t w25q128jv_read_id(w25q128jv_t *w) {
    uint8_t tx_buf[6] = {INST_MANU_DEV_ID, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_buf[6];
    w25q128jv_status_t res;

    nss_low(w);
    res = w25q128jv_transmit_receive(w, tx_buf, rx_buf, sizeof(tx_buf));
    nss_high(w);

    if (res == W25Q128JV_OK) {
        w->manu_id = rx_buf[4];
        w->dev_id = rx_buf[5];
    }

    return res;
}

static w25q128jv_status_t w25q128jv_write_enable(w25q128jv_t *w) {
    uint8_t tx_buf[1] = {INST_WRITE_ENABLE};
    w25q128jv_status_t res;

    nss_low(w);
    res = w25q128jv_transmit(w, tx_buf, sizeof(tx_buf));
    nss_high(w);

    return res;
}

static w25q128jv_status_t w25q128jv_read_status_reg_1(w25q128jv_t *w, uint8_t *reg) {
    uint8_t tx_buf[2] = {INST_READ_STAT_REG_1};
    uint8_t rx_buf[2];
    w25q128jv_status_t res;

    nss_low(w);
    res = w25q128jv_transmit_receive(w, tx_buf, rx_buf, sizeof(tx_buf));
    nss_high(w);

    if (res == W25Q128JV_OK)
        *reg = rx_buf[1];

    return res;
}

static w25q128jv_status_t w25q128jv_wait_busy(w25q128jv_t *w) {
    uint8_t stat_reg_1;
    w25q128jv_status_t res;

    do {
        HAL_Delay(1);
        res = w25q128jv_read_status_reg_1(w, &stat_reg_1);
    } while (res == W25Q128JV_OK && (stat_reg_1 & STAT_REG_1_BUSY) == STAT_REG_1_BUSY);

    return res;
}

static void nss_low(w25q128jv_t *w) {
    HAL_GPIO_WritePin(w->nss_port, w->nss_pin, GPIO_PIN_RESET);
}

static void nss_high(w25q128jv_t *w) {
    HAL_GPIO_WritePin(w->nss_port, w->nss_pin, GPIO_PIN_SET);
}
