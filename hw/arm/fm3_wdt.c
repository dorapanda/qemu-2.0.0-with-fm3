/*
 * Fujitsu FM3 MCU emulator
 *
 * Copyright (c) 2012 Pylone, Inc.
 * Written by Masashi YOKOTA <yokota@pylone.jp>
 *
 * This code is licensed under the GNU GPL v2.
 */

#include "hw/sysbus.h"
#include "hw/devices.h"

#define TYPE_FM3_WDT	"fm3.wdt"

/* register offsets */
#define FM3_WDT_OFFSET_HW_LDR       (0x0000)
#define FM3_WDT_OFFSET_HW_VLR       (0x0004)
#define FM3_WDT_OFFSET_HW_CTL       (0x0008)
#define FM3_WDT_OFFSET_HW_ICL       (0x000C)
#define FM3_WDT_OFFSET_HW_RIS       (0x0010)
#define FM3_WDT_OFFSET_HW_LCK       (0x0C00)
#define FM3_WDT_OFFSET_SW_LDR       (0x1000)
#define FM3_WDT_OFFSET_SW_VLR       (0x1004)
#define FM3_WDT_OFFSET_SW_CTL       (0x1008)
#define FM3_WDT_OFFSET_SW_ICL       (0x100C)
#define FM3_WDT_OFFSET_SW_RIS       (0x1010)
#define FM3_WDT_OFFSET_SW_LCK       (0x1C00)

/* unlock codes */
#define FM3_WDT_UNLOCK              (0x1ACCE551)
#define FM3_WDT_UNLOCK_CTL          (0xE5331AAE)

/* states for unlocking */
enum FM3_WDT_STATE {
    FM3_WDT_STATE_LOCK_ALL,
    FM3_WDT_STATE_LOCK_CTL,
    FM3_WDT_STATE_UNLOCK,
};

typedef struct {
    enum FM3_WDT_STATE state;
    uint32_t control;
} Fm3WatchdogTimer;

typedef struct {
    SysBusDevice busdev;
    MemoryRegion mmio;
    Fm3WatchdogTimer sw;
    Fm3WatchdogTimer hw;
} Fm3WdtState;
#define FM3_WDT(obj) \
    OBJECT_CHECK(Fm3WdtState, (obj), TYPE_FM3_WDT)

static enum FM3_WDT_STATE 
fm3_wdt_unlock_state(enum FM3_WDT_STATE state, uint32_t unlock_code)
{
    enum FM3_WDT_STATE ret = FM3_WDT_STATE_LOCK_ALL; 

    switch (state) {
    case FM3_WDT_STATE_LOCK_ALL:
        if (unlock_code == FM3_WDT_UNLOCK)
            ret = FM3_WDT_STATE_LOCK_CTL; 
        break;
    case FM3_WDT_STATE_LOCK_CTL:
        if (unlock_code == FM3_WDT_UNLOCK_CTL)
            ret = FM3_WDT_STATE_UNLOCK; 
        break;
    default:
        break;
    }

    return ret;
}

static uint64_t fm3_wdt_read(void *opaque, hwaddr offset,
                             unsigned size)
{
    Fm3WdtState *s = (Fm3WdtState *)opaque;
    uint64_t retval = 0;

    switch (offset) {
    case FM3_WDT_OFFSET_HW_CTL:
        retval = s->hw.control & 3;
        break;
    case FM3_WDT_OFFSET_HW_LCK:
        if (s->hw.state != FM3_WDT_STATE_UNLOCK)
            retval = 1;
        break;
    default:
        break;
    }

    return retval;
}

static void fm3_wdt_write(void *opaque, hwaddr offset,
                          uint64_t value, unsigned size)
{
    Fm3WdtState *s = (Fm3WdtState *)opaque;

    switch (offset) {
    case FM3_WDT_OFFSET_HW_CTL:
        if (s->hw.state != FM3_WDT_STATE_UNLOCK)
            break;

        printf("FM3_WDT: H/W watchdog timer is %s\n",
               (value & 2) ? "enabled" : "disabled");

        s->hw.control = value & 3;
        break;
    case FM3_WDT_OFFSET_HW_LCK:
        s->hw.state = fm3_wdt_unlock_state(s->hw.state, value);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps fm3_wdt_mem_ops = {
    .read = fm3_wdt_read,
    .write = fm3_wdt_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int fm3_wdt_init(SysBusDevice *dev)
{
	DeviceState		*devs	= DEVICE(dev);
    Fm3WdtState *s = FM3_WDT(devs);

    memory_region_init_io(&s->mmio, OBJECT(s), &fm3_wdt_mem_ops, s, 
                          TYPE_FM3_WDT, 0x2000);
    sysbus_init_mmio(dev, &s->mmio);

    s->sw.state = FM3_WDT_STATE_LOCK_ALL;
    s->hw.state = FM3_WDT_STATE_LOCK_ALL;

    return 0;
}

static Property fm3_wdt_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void fm3_wdt_class_init(ObjectClass *klass, void *data)
{
	DeviceClass			*dc	= DEVICE_CLASS(klass);
	SysBusDeviceClass	*k	= SYS_BUS_DEVICE_CLASS(klass);

	k->init		= fm3_wdt_init;			/* 初期化関数を登録		*/
	dc->desc	= TYPE_FM3_WDT;		/* ハードウェア名称		*/
	dc->props	= fm3_wdt_properties;	/* 内部情報				*/
}

static const TypeInfo fm3_wdt_info = {
	.name			= TYPE_FM3_WDT,			/* ハードウェア名称		*/
	.parent			= TYPE_SYS_BUS_DEVICE,		/* 接続バス				*/
	.instance_size	= sizeof(Fm3WdtState),	/* インスタンスサイズ	*/
	.class_init		= fm3_wdt_class_init,		/* 初期化関数のポインタ	*/
};

static void fm3_wdt_register_device(void)
{
    type_register_static(&fm3_wdt_info);
}

type_init(fm3_wdt_register_device)
