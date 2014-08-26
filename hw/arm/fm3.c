/*
 * Fujitsu FM3 MCU emulator
 *
 * Copyright (c) 2012 Pylone, Inc.
 * Written by Masashi YOKOTA <yokota@pylone.jp>
 *
 * This code is licensed under the GNU GPL v2.
 */

#include "hw/sysbus.h"
#include "hw/arm/arm.h"
#include "hw/devices.h"
#include "hw/boards.h"
#include "fm3.h"
#include "fm3_board_config.h"
#include "exec/address-spaces.h"


struct fm3_board_info_t {
    hwaddr sram_start;
    uint32_t sram_size;
    uint32_t flash_size;
    uint32_t ex_sram_size;
    uint32_t main_clk_hz;
    uint32_t sub_clk_hz;
} cqfm3_base_info = {
    .sram_start = 0x1FFF0000,
    .sram_size = 128 * 1024,
    .flash_size = 1024 * 1024,
    .ex_sram_size = 0,
};

static void fm3_init(const char *kernel_filename, const char *cpu_model,
                     const struct fm3_board_info_t *bi)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *ex_sram;
    DeviceState *dev = NULL;
    qemu_irq *nvic;
    qemu_irq irq[FM3_IRQ_NUM];
    int n;

    /* Cortex-M3 */
    nvic = armv7m_init(sysmem, bi->flash_size/1024, bi->sram_size/1024,
                      kernel_filename, cpu_model);
    if (!nvic)
        return;

    /* Interrupt sources */
    dev = sysbus_create_simple("fm3.int", 0x40031000, NULL);
    if (!dev)
        return;

	/* Connect to NVIC */
    for (n = 0; n < FM3_IRQ_NUM; n++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), n, nvic[n]);
        irq[n] = qdev_get_gpio_in(dev, n);
    }
    /* Clock */
    sysbus_create_simple("fm3.cr", 0x40010000, NULL);

    /* Watchdog timers */
    sysbus_create_simple("fm3.wdt", 0x40011000, NULL);

    /* External interrupts */
    sysbus_create_varargs("fm3.exti", 0x40030000, irq[4], irq[5], NULL);

    /* GPIO */
    sysbus_create_simple("fm3.gpio", 0x40033000, NULL);

    /* UART(MFS) */
    sysbus_create_varargs("fm3.uart", 0x40038000,
                          irq[7],  irq[8],  /* ch0 */
                          irq[13], irq[14], /* ch3 */
                          irq[15], irq[16], /* ch4 */
                          NULL);

	/* REG Sample */
    sysbus_create_varargs("fm3.RegSample", 0x41000000,
                          irq[21],
                          NULL);

    /* External memory */
    if (bi->ex_sram_size) {
        ex_sram = g_new(MemoryRegion, 1);
        memory_region_init_ram(ex_sram, NULL, "fm3.ex-ram", bi->ex_sram_size);
        memory_region_add_subregion(sysmem, 0x60000000, ex_sram);
    }
}

static void cq_fm3_init(QEMUMachineInitArgs *args)
{
    fm3_init(args->kernel_filename, args->cpu_model, &cqfm3_base_info);
}

static void cq_fm3_ex_init(QEMUMachineInitArgs *args)
{
    cqfm3_base_info.flash_size = 1024 * 1024 * 256;
    cqfm3_base_info.ex_sram_size = 1024 * 1024 * 256;
    fm3_init(args->kernel_filename, args->cpu_model, &cqfm3_base_info);
}


static QEMUMachine fm3_machines[] = {
    {
        .name = "cq-frk-fm3",
        .desc = "Interface CQ-FRK-FM3 board",
        .init = cq_fm3_init,
    },
    {
        .name = "cq-frk-fm3-ex",
        .desc = "Interface CQ-FRK-FM3 board (with expanded Flash and external SRAM)",
        .init = cq_fm3_ex_init,
    },
    {},
};

static void fm3_machine_init(void)
{
    int i;

    for (i = 0; fm3_machines[i].name; i++)
        qemu_register_machine(&fm3_machines[i]);
}

machine_init(fm3_machine_init);
