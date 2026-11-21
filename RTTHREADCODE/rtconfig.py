import os

# toolchains options
ARCH        = 'arm'
CPU         = 'cortex-a'
CROSS_TOOL  = 'gcc'

_project_dir = os.path.dirname(os.path.abspath(__file__))
_sdk_root = os.path.abspath(os.path.join(_project_dir, '..', '..', 'rk356x-linux-2026051515'))
RTT_ROOT = os.path.abspath(os.getenv('RTT_ROOT') or os.path.join(_sdk_root, 'rtos'))

if os.getenv('RTT_CC'):
    CROSS_TOOL = os.getenv('RTT_CC')

PLATFORM    = 'gcc'
_default_exec = os.path.join(_sdk_root, 'prebuilts', 'gcc', 'linux-x86', 'arm', 'gcc-arm-none-eabi', 'bin')
EXEC_PATH   = os.getenv('RTT_EXEC_PATH') or (_default_exec if os.path.isdir(_default_exec) else '/usr/bin')

BUILD = os.getenv('AMP_BUILD', 'release')
if BUILD not in ('debug', 'release'):
    raise ValueError('AMP_BUILD must be debug or release')

if PLATFORM == 'gcc':
    # toolchains - ARM32 bare-metal
    # CROSS_COMPILE is set by mk-rtos.sh via environment
    PREFIX  = os.getenv('RTT_CC_PREFIX') or 'arm-none-eabi-'
    # Resolve the SDK toolchain even when callers invoke scons directly without
    # exporting PATH. An explicit absolute prefix still wins.
    if not os.path.isabs(PREFIX):
        candidate = os.path.join(EXEC_PATH, PREFIX)
        if os.path.isfile(candidate + 'gcc'):
            PREFIX = candidate
    CC      = PREFIX + 'gcc'
    CXX     = PREFIX + 'g++'
    AS      = PREFIX + 'gcc'
    AR      = PREFIX + 'ar'
    LINK    = PREFIX + 'gcc'
    TARGET_EXT = 'elf'
    SIZE    = PREFIX + 'size'
    OBJDUMP = PREFIX + 'objdump'
    OBJCPY  = PREFIX + 'objcopy'

    # Cortex-A55, ARMv8-A AArch32, bare-metal
    DEVICE = ' -g -march=armv8-a -mtune=cortex-a55 -marm -fdiagnostics-color=always'
    CFLAGS = DEVICE + ' -Wall -Wno-cpp -ffreestanding'
    AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp -D__ASSEMBLY__'
    # Use the toolchain's newlib for the small libc surface pulled in by
    # RT-Thread's klibc (memcpy, strtoll, ctype and floating conversion).
    # Keep libgcc after libc so EABI helpers resolve correctly.
    LIBGCC_PATH = os.popen(PREFIX + 'gcc -print-libgcc-file-name').read().strip()
    LIBGCC_DIR = os.path.dirname(LIBGCC_PATH)
    LIBS = ' -L' + LIBGCC_DIR + ' -lc -lm -lgcc'
    LFLAGS = DEVICE + ' -nostartfiles -nostdlib -Wl,--gc-sections,-Map=rtthread.map,-cref,-u,system_vectors -T link.lds'
    CPPFLAGS= ' -E -P -x assembler-with-cpp'
    CPATH   = ''
    LPATH   = ''

    if BUILD == 'debug':
        CFLAGS += ' -O0 -ggdb'
        AFLAGS += ' -ggdb'
    else:
        CFLAGS += ' -O2 -ggdb'

    if os.getenv('AMP_CAN_LOOPBACK_TEST') == '1':
        CFLAGS += ' -DAMP_CAN_LOOPBACK_TEST'
    if os.getenv('AMP_CACHE', '1') == '0':
        CFLAGS += ' -DAMP_PRIVATE_CACHE=0'

    CXXFLAGS = CFLAGS

DUMP_ACTION = OBJDUMP + ' -D -S $TARGET > rtt.asm\n'
POST_ACTION = OBJCPY + ' -O binary $TARGET rtthread.bin\n' + SIZE + ' $TARGET \n'
