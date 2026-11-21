/* Ported from cup_car; see modules/LICENSE.cup_car. */
#include "daemon.h"
#include <rtthread.h>

static DaemonInstance *instances[DAEMON_MX_CNT];

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    unsigned int i;
    DaemonInstance *instance;
    if (!config)
        return RT_NULL;
    for (i = 0; i < DAEMON_MX_CNT && instances[i]; i++)
        ;
    if (i == DAEMON_MX_CNT)
        return RT_NULL;
    instance = rt_calloc(1, sizeof(*instance));
    if (!instance)
        return RT_NULL;
    instance->owner_id = config->owner_id;
    instance->callback = config->callback;
    instance->reload_count = config->reload_count ? config->reload_count : 100;
    instance->temp_count = config->init_count ? config->init_count : instance->reload_count;
    instances[i] = instance;
    return instance;
}

void DaemonReload(DaemonInstance *instance)
{
    if (instance)
        instance->temp_count = instance->reload_count;
}

uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    return instance && instance->temp_count > 0;
}

void DaemonTask(void)
{
    unsigned int i;
    for (i = 0; i < DAEMON_MX_CNT; i++) {
        DaemonInstance *instance = instances[i];
        if (instance && instance->temp_count && --instance->temp_count == 0 && instance->callback)
            instance->callback(instance->owner_id);
    }
}
