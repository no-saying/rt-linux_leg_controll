#ifndef BSP_LOG_H
#define BSP_LOG_H

#include <rtthread.h>

#define LOGINFO(...) do { rt_kprintf("[I] "); rt_kprintf(__VA_ARGS__); rt_kprintf("\n"); } while (0)
#define LOGWARNING(...) do { rt_kprintf("[W] "); rt_kprintf(__VA_ARGS__); rt_kprintf("\n"); } while (0)
#define LOGERROR(...) do { rt_kprintf("[E] "); rt_kprintf(__VA_ARGS__); rt_kprintf("\n"); } while (0)

#endif
