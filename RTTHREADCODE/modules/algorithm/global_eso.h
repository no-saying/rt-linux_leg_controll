#ifndef GLOBAL_ESO_H
#define GLOBAL_ESO_H

#include <stdint.h>

#define GLOBAL_ESO_AXES 3U

struct global_eso_config {
    double bandwidth[GLOBAL_ESO_AXES];
    double disturbance_limit[GLOBAL_ESO_AXES];
    double nonlinear_alpha;
    double nonlinear_delta;
};

struct global_eso_measurement {
    double angle[GLOBAL_ESO_AXES];
    double rate[GLOBAL_ESO_AXES];
    double nominal_acceleration[GLOBAL_ESO_AXES];
};

struct global_eso_state {
    double angle[GLOBAL_ESO_AXES];
    double rate[GLOBAL_ESO_AXES];
    double disturbance[GLOBAL_ESO_AXES];
};

struct global_eso {
    struct global_eso_config config;
    struct global_eso_state state;
    uint8_t initialized;
};

void global_eso_config_default(struct global_eso_config *config);
int global_eso_init(struct global_eso *eso,
                    const struct global_eso_config *config);
void global_eso_reset(struct global_eso *eso, const double angle[GLOBAL_ESO_AXES],
                      const double rate[GLOBAL_ESO_AXES]);
int global_eso_update(struct global_eso *eso,
                      const struct global_eso_measurement *measurement,
                      double dt, struct global_eso_state *output);

#endif
