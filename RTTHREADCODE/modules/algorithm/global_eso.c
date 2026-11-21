#include "global_eso.h"

#include <math.h>
#include <string.h>

#define GLOBAL_ESO_PI 3.14159265358979323846

static int finite3(const double values[GLOBAL_ESO_AXES])
{
    unsigned int i;
    for (i = 0; i < GLOBAL_ESO_AXES; ++i) {
        if (!isfinite(values[i]))
            return 0;
    }
    return 1;
}

static double wrap_angle(double value)
{
    return remainder(value, 2.0 * GLOBAL_ESO_PI);
}

static double fal(double error, double alpha, double delta)
{
    double magnitude = fabs(error);
    if (magnitude <= delta)
        return error / pow(delta, 1.0 - alpha);
    return pow(magnitude, alpha) * (error < 0.0 ? -1.0 : 1.0);
}

static int valid_config(const struct global_eso_config *config)
{
    unsigned int i;
    if (!config || !finite3(config->bandwidth) ||
        !finite3(config->disturbance_limit) ||
        !isfinite(config->nonlinear_alpha) ||
        !isfinite(config->nonlinear_delta) ||
        config->nonlinear_alpha <= 0.0 || config->nonlinear_alpha >= 1.0 ||
        config->nonlinear_delta <= 0.0)
        return 0;
    for (i = 0; i < GLOBAL_ESO_AXES; ++i) {
        if (config->bandwidth[i] <= 0.0 ||
            config->disturbance_limit[i] <= 0.0)
            return 0;
    }
    return 1;
}

void global_eso_config_default(struct global_eso_config *config)
{
    if (!config)
        return;
    config->bandwidth[0] = 35.0;
    config->bandwidth[1] = 35.0;
    config->bandwidth[2] = 25.0;
    config->disturbance_limit[0] = 30.0;
    config->disturbance_limit[1] = 30.0;
    config->disturbance_limit[2] = 20.0;
    config->nonlinear_alpha = 0.5;
    config->nonlinear_delta = 0.01;
}

int global_eso_init(struct global_eso *eso,
                    const struct global_eso_config *config)
{
    struct global_eso_config defaults;
    if (!eso)
        return 0;
    if (!config) {
        global_eso_config_default(&defaults);
        config = &defaults;
    }
    if (!valid_config(config))
        return 0;
    memset(eso, 0, sizeof(*eso));
    eso->config = *config;
    return 1;
}

void global_eso_reset(struct global_eso *eso,
                      const double angle[GLOBAL_ESO_AXES],
                      const double rate[GLOBAL_ESO_AXES])
{
    if (!eso)
        return;
    memset(&eso->state, 0, sizeof(eso->state));
    if (angle && finite3(angle))
        memcpy(eso->state.angle, angle, sizeof(eso->state.angle));
    if (rate && finite3(rate))
        memcpy(eso->state.rate, rate, sizeof(eso->state.rate));
    eso->initialized = 0U;
}

int global_eso_update(struct global_eso *eso,
                      const struct global_eso_measurement *measurement,
                      double dt, struct global_eso_state *output)
{
    struct global_eso_state next;
    unsigned int i;

    if (!eso || !measurement || !output || !valid_config(&eso->config) ||
        !finite3(measurement->angle) || !finite3(measurement->rate) ||
        !finite3(measurement->nominal_acceleration) || !isfinite(dt) ||
        dt <= 0.0 || dt > 0.02)
        return 0;

    if (!eso->initialized) {
        memcpy(eso->state.angle, measurement->angle, sizeof(eso->state.angle));
        memcpy(eso->state.rate, measurement->rate, sizeof(eso->state.rate));
        memset(eso->state.disturbance, 0, sizeof(eso->state.disturbance));
        eso->initialized = 1U;
        *output = eso->state;
        return 1;
    }

    next = eso->state;
    for (i = 0; i < GLOBAL_ESO_AXES; ++i) {
        double bandwidth = eso->config.bandwidth[i];
        double innovation = wrap_angle(measurement->angle[i] - eso->state.angle[i]) +
                            (measurement->rate[i] - eso->state.rate[i]) /
                                fmax(bandwidth, 1e-6);
        double correction = fal(innovation, eso->config.nonlinear_alpha,
                                eso->config.nonlinear_delta);
        double b1 = 3.0 * bandwidth;
        double b2 = 3.0 * bandwidth * bandwidth;
        double b3 = bandwidth * bandwidth * bandwidth;

        next.angle[i] += dt * (eso->state.rate[i] + b1 * innovation);
        next.rate[i] += dt * (measurement->nominal_acceleration[i] +
                             eso->state.disturbance[i] + b2 * correction);
        next.disturbance[i] += dt * b3 * correction;
        if (next.disturbance[i] > eso->config.disturbance_limit[i])
            next.disturbance[i] = eso->config.disturbance_limit[i];
        else if (next.disturbance[i] < -eso->config.disturbance_limit[i])
            next.disturbance[i] = -eso->config.disturbance_limit[i];
        next.angle[i] = wrap_angle(next.angle[i]);
    }

    if (!finite3(next.angle) || !finite3(next.rate) ||
        !finite3(next.disturbance))
        return 0;
    eso->state = next;
    *output = next;
    return 1;
}
