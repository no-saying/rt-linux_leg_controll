#include <rtthread.h>
#include <rthw.h>
#include <stdint.h>
#include <string.h>
#include "amp_periph.h"
#include "amp_dma.h"
#include "bsp_dwt.h"
#ifndef REG32
#define REG32(addr) (*(volatile uint32_t *)(addr))
#endif

#define GPIO4_BASE              0xfe770000U
#define GPIO4_DR_H              REG32(GPIO4_BASE + 0x04)
#define GPIO4_DDR_H             REG32(GPIO4_BASE + 0x0c)
#define GPIO4_PC4_BIT           (1U << 4)
#define GPIO4_PC6_BIT           (1U << 6)

#define SPI3_BASE               0xfe640000U
#define SPI_CTRLR0              REG32(SPI3_BASE + 0x0000)
#define SPI_CTRLR1              REG32(SPI3_BASE + 0x0004)
#define SPI_SSIENR              REG32(SPI3_BASE + 0x0008)
#define SPI_SER                 REG32(SPI3_BASE + 0x000c)
#define SPI_BAUDR               REG32(SPI3_BASE + 0x0010)
#define SPI_TXFTLR              REG32(SPI3_BASE + 0x0014)
#define SPI_RXFTLR              REG32(SPI3_BASE + 0x0018)
#define SPI_TXFLR               REG32(SPI3_BASE + 0x001c)
#define SPI_RXFLR               REG32(SPI3_BASE + 0x0020)
#define SPI_SR                  REG32(SPI3_BASE + 0x0024)
#define SPI_IMR                 REG32(SPI3_BASE + 0x002c)
#define SPI_ICR                 REG32(SPI3_BASE + 0x0038)
#define SPI_DMACR               REG32(SPI3_BASE + 0x003c)
#define SPI_DMATDLR             REG32(SPI3_BASE + 0x0040)
#define SPI_DMARDLR             REG32(SPI3_BASE + 0x0044)
#define SPI_TXDR                REG32(SPI3_BASE + 0x0400)
#define SPI_RXDR                REG32(SPI3_BASE + 0x0800)

#define SPI_SR_TF_FULL          (1U << 1)
#define SPI_SR_RF_EMPTY         (1U << 3)
#define SPI_CR0_8BIT            (1U << 0)
#define SPI_CR0_CPOL_HIGH       (1U << 7)
#define SPI_CR0_CPHA_START      (1U << 6)
#define SPI_CR0_CSM_KEEP        (0U << 8)
#define SPI_CR0_SSD_ONE         (1U << 10)
#define SPI_CR0_EM_BIG          (1U << 11)
#define SPI_CR0_BHT_8BIT        (1U << 13)
#define SPI_CR0_FRF_SPI         (0U << 16)
#define SPI_CR0_XFM_TR          (0U << 18)

#define BMI088_ACC_CHIP_ID      0x00
#define BMI088_ACC_ID_VALUE     0x1e
#define BMI088_ACC_XOUT_L       0x12
#define BMI088_TEMP_M           0x22
#define BMI088_ACC_CONF         0x40
#define BMI088_ACC_RANGE        0x41
#define BMI088_INT1_IO_CTRL     0x53
#define BMI088_INT_MAP_DATA     0x58
#define BMI088_ACC_PWR_CONF     0x7c
#define BMI088_ACC_PWR_CTRL     0x7d
#define BMI088_ACC_SOFTRESET    0x7e
#define BMI088_GYRO_CHIP_ID     0x00
#define BMI088_GYRO_ID_VALUE    0x0f
#define BMI088_GYRO_RANGE       0x0f
#define BMI088_GYRO_BANDWIDTH   0x10
#define BMI088_GYRO_LPM1        0x11
#define BMI088_GYRO_SOFTRESET   0x14
#define BMI088_GYRO_CTRL        0x15
#define BMI088_GYRO_IO_CONF     0x16
#define BMI088_GYRO_IO_MAP      0x18

#define BMI088_ACC_SEN_6G       0.00179443359375f
#define BMI088_GYRO_SEN_2000    0.00106526443603f
#define BMI088_TEMP_FACTOR      0.125f
#define BMI088_TEMP_OFFSET      23.0f

struct bmi088_reg_cfg {
    uint8_t reg;
    uint8_t val;
};

static uint8_t _bmi088_error = 0xff;
static uint8_t _bmi088_acc_id_raw = 0xff;
static uint8_t _bmi088_gyro_id_raw = 0xff;
static uint32_t _spi3_timeouts = 0;
static uint32_t _spi3_ok = 0;
static struct rt_mutex spi_lock;
static int spi_lock_ready;
static uint8_t spi_tx[AMP_DMA_MAX_BYTES] __attribute__((aligned(64)));
static uint8_t spi_rx[AMP_DMA_MAX_BYTES] __attribute__((aligned(64)));
static struct amp_spi3_diagnostic spi_diagnostic;

static void gpio4_set(uint32_t bit, int high)
{
    GPIO4_DR_H = (bit << 16) | (high ? bit : 0);
}

static void gpio4_output(uint32_t bit, int high)
{
    gpio4_set(bit, high);
    GPIO4_DDR_H = (bit << 16) | bit;
}

static void spi3_cs(int cs, int active)
{
    if (cs == 1)
        gpio4_set(GPIO4_PC4_BIT, !active);
    else if (cs == 2)
        gpio4_set(GPIO4_PC6_BIT, !active);
}

void amp_spi3_hw_init(void)
{
    gpio4_output(GPIO4_PC4_BIT, 1);
    gpio4_output(GPIO4_PC6_BIT, 1);

    SPI_SSIENR = 0;
    SPI_IMR = 0;
    SPI_DMACR = 0;
    SPI_ICR = 0xffffffffU;
    SPI_TXFTLR = 16;
    SPI_RXFTLR = 0;
    SPI_BAUDR = 10; /* 50 MHz assigned clock / 10 = 5 MHz. */
    /* SPI mode 0 (CPOL=0, CPHA=0) — most IMU boards are wired for mode 0. */
    SPI_CTRLR0 = SPI_CR0_8BIT |
                 SPI_CR0_CSM_KEEP | SPI_CR0_SSD_ONE | SPI_CR0_EM_BIG |
                 SPI_CR0_BHT_8BIT | SPI_CR0_FRF_SPI | SPI_CR0_XFM_TR;
    SPI_SER = 1;
}

int amp_spi3_transfer(int cs, const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    uint32_t sent = 0, received = 0;
    uint64_t deadline, begin;
    int rc = RT_EOK, ready, dma;

    if ((cs != 1 && cs != 2) || !len || len > 65536U)
        return -RT_EINVAL;
    rt_mutex_take(&spi_lock, RT_WAITING_FOREVER);
    ready = amp_dma_available();
    if (ready < 0) { rc = -RT_ERROR; goto unlock; }
    dma = ready && len >= 8 && len <= AMP_DMA_MAX_BYTES;
    begin = BSPTimeCounter();
    deadline = begin + ((uint64_t)len * 2U + 2000U) * 24U;
    SPI_SSIENR = 0;
    SPI_DMACR = 0;
    SPI_CTRLR1 = len - 1;
    SPI_ICR = 0xffffffffU;
    SPI_SER = 1;
    if (dma) {
        if (tx) memcpy(spi_tx, tx, len); else memset(spi_tx, 0xff, len);
        rc = amp_dma_prepare(AMP_DMA_SPI_RX, spi_rx, len);
        if (!rc) rc = amp_dma_prepare(AMP_DMA_SPI_TX, spi_tx, len);
        if (rc) goto stop;
        SPI_DMATDLR = 0; SPI_DMARDLR = 0;
        /* Arm RX before TX; clocks cannot begin until SSIENR is set. */
        rc = amp_dma_start(AMP_DMA_SPI_RX);
        if (!rc) rc = amp_dma_start(AMP_DMA_SPI_TX);
        if (rc) goto stop;
        SPI_DMACR = 3;
    }
    spi3_cs(cs, 1);
    SPI_SSIENR = 1;
    if (dma) {
        ++spi_diagnostic.dma;
        rc = amp_dma_wait(AMP_DMA_SPI_RX, deadline);
        if (!rc) rc = amp_dma_wait(AMP_DMA_SPI_TX, deadline);
        if (!rc && rx) memcpy(rx, spi_rx, len);
    } else {
        ++spi_diagnostic.pio;
        /* Pipeline the 64-entry FIFO, rather than waiting after every byte. */
        while (received < len) {
            while (sent < len && sent - received < 64U && !(SPI_SR & SPI_SR_TF_FULL)) {
                SPI_TXDR = tx ? tx[sent] : 0xff;
                ++sent;
            }
            while (received < sent && !(SPI_SR & SPI_SR_RF_EMPTY)) {
                uint8_t byte = SPI_RXDR;
                if (rx) rx[received] = byte;
                ++received;
            }
            if (BSPTimeCounter() >= deadline) { rc = -RT_ETIMEOUT; break; }
        }
    }
    while (!rc && (SPI_SR & 1U)) {
        if (BSPTimeCounter() >= deadline) rc = -RT_ETIMEOUT;
    }
stop:
    SPI_DMACR = 0;
    if (dma) {
        if (amp_dma_stop(AMP_DMA_SPI_TX)) rc = -RT_ERROR;
        if (amp_dma_stop(AMP_DMA_SPI_RX)) rc = -RT_ERROR;
    }
    SPI_SSIENR = 0;
    spi3_cs(cs, 0);
    if (BSPTimeCounter() - begin > spi_diagnostic.max_ticks)
        spi_diagnostic.max_ticks = (uint32_t)(BSPTimeCounter() - begin);
    if (!rc) {
        _spi3_ok++;
    } else if (rc == -RT_ETIMEOUT) ++_spi3_timeouts;
    else ++spi_diagnostic.errors;
unlock:
    rt_mutex_release(&spi_lock);
    return rc ? rc : (int)len;
}

static void bmi088_write(int cs, uint8_t reg, uint8_t val)
{
    uint8_t tx[2];

    tx[0] = reg & 0x7f;
    tx[1] = val;
    if (amp_spi3_transfer(cs, tx, RT_NULL, sizeof(tx)) != sizeof(tx))
        _bmi088_error |= 0x80;
}

static int bmi088_read_acc(uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint8_t tx[10];
    uint8_t rx[10];
    uint32_t n = len + 2;

    if (n > sizeof(tx))
        return -RT_EINVAL;

    memset(tx, 0xff, n);
    tx[0] = reg | 0x80;
    if (amp_spi3_transfer(1, tx, rx, n) != (int)n) return -RT_ERROR;
    memcpy(buf, &rx[2], len);
    return RT_EOK;
}

static int bmi088_read_gyro(uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint8_t tx[10];
    uint8_t rx[10];
    uint32_t n = len + 1;

    if (n > sizeof(tx))
        return -RT_EINVAL;

    memset(tx, 0xff, n);
    tx[0] = reg | 0x80;
    if (amp_spi3_transfer(2, tx, rx, n) != (int)n) return -RT_ERROR;
    memcpy(buf, &rx[1], len);
    return RT_EOK;
}

static uint8_t bmi088_read_acc_reg(uint8_t reg)
{
    uint8_t val = 0xff;

    bmi088_read_acc(reg, &val, 1);
    return val;
}

static uint8_t bmi088_read_gyro_reg(uint8_t reg)
{
    uint8_t val = 0xff;

    bmi088_read_gyro(reg, &val, 1);
    return val;
}

static int bmi088_init(void)
{
    static const struct bmi088_reg_cfg acc_cfg[] = {
        { BMI088_ACC_PWR_CTRL, 0x04 },
        { BMI088_ACC_PWR_CONF, 0x00 },
        { BMI088_ACC_CONF, 0x80 | 0x20 | 0x0b },
        { BMI088_ACC_RANGE, 0x01 },
        { BMI088_INT1_IO_CTRL, 0x08 },
        { BMI088_INT_MAP_DATA, 0x04 },
    };
    static const struct bmi088_reg_cfg gyro_cfg[] = {
        { BMI088_GYRO_RANGE, 0x00 },
        { BMI088_GYRO_BANDWIDTH, 0x80 | 0x01 },
        { BMI088_GYRO_LPM1, 0x00 },
        { BMI088_GYRO_CTRL, 0x80 },
        { BMI088_GYRO_IO_CONF, 0x00 },
        { BMI088_GYRO_IO_MAP, 0x01 },
    };
    uint8_t id;
    uint32_t i;

    _bmi088_error = 0;

    id = bmi088_read_acc_reg(BMI088_ACC_CHIP_ID);
    rt_thread_mdelay(1);
    id = bmi088_read_acc_reg(BMI088_ACC_CHIP_ID);
    _bmi088_acc_id_raw = id;
    rt_kprintf("[BMI088] acc chip_id raw=0x%02x (want 0x1e)\n", id);
    if (id != BMI088_ACC_ID_VALUE)
        _bmi088_error |= 0x01;

    bmi088_write(1, BMI088_ACC_SOFTRESET, 0xb6);
    rt_thread_mdelay(80);

    id = bmi088_read_acc_reg(BMI088_ACC_CHIP_ID);
    if (id != BMI088_ACC_ID_VALUE)
        _bmi088_error |= 0x02;

    for (i = 0; i < sizeof(acc_cfg) / sizeof(acc_cfg[0]); i++) {
        bmi088_write(1, acc_cfg[i].reg, acc_cfg[i].val);
        rt_thread_mdelay(1);
    }

    id = bmi088_read_gyro_reg(BMI088_GYRO_CHIP_ID);
    rt_thread_mdelay(1);
    id = bmi088_read_gyro_reg(BMI088_GYRO_CHIP_ID);
    _bmi088_gyro_id_raw = id;
    rt_kprintf("[BMI088] gyro chip_id raw=0x%02x (want 0x0f)\n", id);
    if (id != BMI088_GYRO_ID_VALUE)
        _bmi088_error |= 0x10;

    bmi088_write(2, BMI088_GYRO_SOFTRESET, 0xb6);
    rt_thread_mdelay(80);

    id = bmi088_read_gyro_reg(BMI088_GYRO_CHIP_ID);
    if (id != BMI088_GYRO_ID_VALUE)
        _bmi088_error |= 0x20;

    for (i = 0; i < sizeof(gyro_cfg) / sizeof(gyro_cfg[0]); i++) {
        bmi088_write(2, gyro_cfg[i].reg, gyro_cfg[i].val);
        rt_thread_mdelay(1);
    }

    return _bmi088_error ? -RT_ERROR : RT_EOK;
}

uint8_t amp_bmi088_last_error(void)
{
    return _bmi088_error;
}

void amp_bmi088_raw_ids(uint8_t *acc_id, uint8_t *gyro_id)
{
    if (acc_id)
        *acc_id = _bmi088_acc_id_raw;
    if (gyro_id)
        *gyro_id = _bmi088_gyro_id_raw;
}

void amp_spi3_stats(uint32_t *ok, uint32_t *timeouts)
{
    if (ok)
        *ok = _spi3_ok;
    if (timeouts)
        *timeouts = _spi3_timeouts;
}

int amp_bmi088_read(struct amp_imu_sample *sample)
{
    uint8_t buf[8];
    int16_t raw;

    if (!sample)
        return -RT_EINVAL;
    if (_bmi088_error)
        return -RT_ERROR;

    if (bmi088_read_acc(BMI088_ACC_XOUT_L, buf, 6)) return -RT_ERROR;
    raw = (int16_t)((buf[1] << 8) | buf[0]);
    sample->accel[0] = raw * BMI088_ACC_SEN_6G;
    raw = (int16_t)((buf[3] << 8) | buf[2]);
    sample->accel[1] = raw * BMI088_ACC_SEN_6G;
    raw = (int16_t)((buf[5] << 8) | buf[4]);
    sample->accel[2] = raw * BMI088_ACC_SEN_6G;

    if (bmi088_read_gyro(BMI088_GYRO_CHIP_ID, buf, 8)) return -RT_ERROR;
    if (buf[0] != BMI088_GYRO_ID_VALUE)
        return -RT_ERROR;
    raw = (int16_t)((buf[3] << 8) | buf[2]);
    sample->gyro[0] = raw * BMI088_GYRO_SEN_2000;
    raw = (int16_t)((buf[5] << 8) | buf[4]);
    sample->gyro[1] = raw * BMI088_GYRO_SEN_2000;
    raw = (int16_t)((buf[7] << 8) | buf[6]);
    sample->gyro[2] = raw * BMI088_GYRO_SEN_2000;

    if (bmi088_read_acc(BMI088_TEMP_M, buf, 2)) return -RT_ERROR;
    raw = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (raw > 1023)
        raw -= 2048;
    sample->temperature = raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    return RT_EOK;
}

void amp_spi3_get_diagnostic(struct amp_spi3_diagnostic *diagnostic)
{
    if (diagnostic) *diagnostic = spi_diagnostic;
}
int amp_spi3_init(void)
{
    if (!spi_lock_ready) {
        rt_mutex_init(&spi_lock, "spi3", RT_IPC_FLAG_PRIO);
        spi_lock_ready = 1;
    }
    rt_mutex_take(&spi_lock, RT_WAITING_FOREVER);
    amp_spi3_hw_init();
    rt_mutex_release(&spi_lock);
    return bmi088_init();
}
