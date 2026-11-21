#ifndef __AMP_PERIPH_H__
#define __AMP_PERIPH_H__

#include <rtthread.h>
#include <stddef.h>
#include <stdint.h>

/* HAL-style board bring-up for the peripherals owned by CPU3. */
void amp_hw_init(void);
void amp_spi3_hw_init(void);
int amp_spi3_init(void);

struct amp_can_frame {
    uint32_t id;
    uint8_t ide;
    uint8_t rtr;
    uint8_t len;
    /* The ARM libc copy may use word accesses even with the MMU enabled.
     * Keep payloads aligned when dispatching them to registered callbacks. */
    _Alignas(uint32_t) uint8_t data[8];
};

_Static_assert(offsetof(struct amp_can_frame, data) % sizeof(uint32_t) == 0,
               "CAN payload must support aligned word copies");

struct amp_can_diagnostic {
    uint32_t sample_ms;
    uint32_t mode, cmd, state, irq, error, rx_error, tx_error;
    uint32_t nbtp, tx_id, tx_fic;
    uint32_t tx_submitted, tx_completed, tx_busy;
    uint32_t error_events, bus_off_events, recovery_attempts;
    uint32_t last_error, last_state, last_rx_error, last_tx_error;
};

struct amp_imu_sample {
    float accel[3];
    float gyro[3];
    float temperature;
};

int amp_periph_init(void);
int amp_can_send(int bus, uint32_t id, const uint8_t *data, uint8_t len, int ide);
int amp_can_try_send(int bus, uint32_t id, const uint8_t *data, uint8_t len, int ide);
int amp_can_recv(int bus, struct amp_can_frame *frame);
/* Copies CPU3's cached snapshot; does not access or acknowledge hardware. */
int amp_can_get_diagnostic(int bus, struct amp_can_diagnostic *diagnostic);
#ifdef AMP_CAN_LOOPBACK_TEST
struct amp_can_loopback_result {
    uint32_t sent, received, mismatched, timeouts, normal_mode, rxfc;
    struct amp_can_diagnostic during;
};
/* Boot-time only: run before starting the motor service. Always restores CAN. */
int amp_can_loopback_test(int bus, struct amp_can_loopback_result *result);
#endif
int amp_spi3_transfer(int cs, const uint8_t *tx, uint8_t *rx, uint32_t len);
int amp_pwm14_set(uint32_t duty_permille);
int amp_bmi088_read(struct amp_imu_sample *sample);
uint8_t amp_bmi088_last_error(void);
void amp_bmi088_raw_ids(uint8_t *acc_id, uint8_t *gyro_id);
void amp_spi3_stats(uint32_t *ok, uint32_t *timeouts);
struct amp_spi3_diagnostic { uint32_t pio, dma, errors, max_ticks; };
void amp_spi3_get_diagnostic(struct amp_spi3_diagnostic *diagnostic);

#endif
