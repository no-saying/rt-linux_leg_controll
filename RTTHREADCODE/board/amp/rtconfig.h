#ifndef RT_CONFIG_H__
#define __RT_CONFIG_H__

/* =============== RT-Thread Kernel =============== */
#define RT_NAME_MAX 8
#define RT_ALIGN_SIZE 8
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000

/* IPC */
#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE

/* Memory */
#define RT_USING_MEMPOOL
#define RT_USING_MEMHEAP
#define RT_USING_HEAP
#define RT_USING_SMALL_MEM
/* Select the small memory allocator as the system heap backend. Without one
 * of the *_AS_HEAP selectors, rt_system_heap_init() is a no-op and rt_malloc()
 * always returns RT_NULL -> every thread/semaphore allocation fails. */
#define RT_USING_SMALL_MEM_AS_HEAP

/* Device */
#define RT_USING_DEVICE
#define RT_USING_DEVICE_OPS
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 256

/* Components */
#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 8192
#define RT_MAIN_THREAD_PRIORITY 10

/* =============== ARM Architecture =============== */
#define ARCH_ARM
#define ARCH_ARM_CORTEX_A
#define MAX_HANDLERS 256
#define RT_USING_CACHE
#define RT_CPU_CACHE_LINE_SZ 64
#define RT_USING_GIC_V3
#define RT_GIC_V3_ITS
#define RT_GIC_MAX_NR 1

/* =============== Board =============== */
#define SOC_RK3568
#define RT_USING_AMP_BAREMETAL
#define RT_CPUS_NR 1
#define RT_BACKTRACE_LEVEL_MAX_NR 8

#endif
/* =============== IPC: Mailbox + RPMsg =============== */
