/*
 * Fujitsu FM3 External Interrupts
 *
 * Copyright (c) 2012 Pylone, Inc.
 * Written by Masashi YOKOTA <yokota@pylone.jp>
 *
 * This code is licensed under the GNU GPL v2.
 */

#include "hw/sysbus.h"
#include "sysemu/sysemu.h"
#include "qapi/qmp/qerror.h"
#include "fm3.h"
#include "fm3_board_config.h"

//#define FM3_DEBUG_EXTI
#define TYPE_FM3_EXTI	"fm3.exti"


#ifdef FM3_DEBUG_EXTI
#define DPRINTF(fmt, ...)                                       \
    do { printf(fmt, ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif

#define FM3_EXTI_IRQ_NUM    (2)
typedef struct {
    SysBusDevice busdev;
    MemoryRegion mmio;
    qemu_irq irq[FM3_EXTI_IRQ_NUM];
    uint32_t enable;
    uint8_t signal[FM3_EXTI_NUM];
    uint32_t request_latch;
    uint32_t mode_0;
    uint32_t mode_1;
    int port_no[FM3_EXTI_NUM];
    int irq_flag[FM3_EXTI_IRQ_NUM];
} Fm3ExtiState;
#define FM3_EXTI(obj) \
    OBJECT_CHECK(Fm3ExtiState, (obj), TYPE_FM3_EXTI)


#define FM3_EXTI_ENIR           (0x00)
#define FM3_EXTI_EIRR           (0x04)
#define FM3_EXTI_EICL           (0x08)
#define FM3_EXTI_ELVR           (0x0C)
#define FM3_EXTI_ELVR1          (0x10)
#define FM3_EXTI_NMIRR          (0x14)
#define FM3_EXTI_NMICL          (0x18)

static Fm3ExtiState *fm3_exti_state;

static int fm3_exti_get_mode(Fm3ExtiState *s, int exti_no);
static void fm3_exti_request_latch(Fm3ExtiState *s, int exti_no, int request);
static void fm3_exti_update_irq(Fm3ExtiState *s);

enum {
    FM3_EXTI_ENIR_0_7,
    FM3_EXTI_ENIR_8_31,
};

enum {
    FM3_EXTI_ELVR_LEVEL_LOW,
    FM3_EXTI_ELVR_LEVEL_HIGH,
    FM3_EXTI_ELVR_EDGE_RISING,
    FM3_EXTI_ELVR_EDGE_FALLING,
};

uint32_t fm3_exti_get_irq_stat(int ch)
{
    Fm3ExtiState *s = fm3_exti_state;
    if (ch < 0 || FM3_EXTI_NUM <= ch)
        return 0;

    return ((s->request_latch >> ch) & (s->enable >> ch )) & 1;
}

static int fm3_exti_get_mode(Fm3ExtiState *s, int exti_no)
{
    int mode;

    if (exti_no < 16)
       mode = s->mode_0;
    else 
       mode = s->mode_1;

    return (mode >> exti_no * 2) & 3;
}

static int fm3_exti_get_signal(Fm3ExtiState *s, int exti_no, int request)
{
    int mode = fm3_exti_get_mode(s, exti_no);

    switch (mode) {
    case FM3_EXTI_ELVR_LEVEL_LOW:
    case FM3_EXTI_ELVR_LEVEL_HIGH:
        return request? FM3_EXTI_ELVR_LEVEL_HIGH:
                        FM3_EXTI_ELVR_LEVEL_LOW;
    case FM3_EXTI_ELVR_EDGE_FALLING:
    case FM3_EXTI_ELVR_EDGE_RISING:
        return request? FM3_EXTI_ELVR_EDGE_RISING:
                        FM3_EXTI_ELVR_EDGE_FALLING;
    default:
        printf("%s: Unknown level mode = %d\n", __func__, mode);
        return -1;
    }
}

static void fm3_exti_request_latch(Fm3ExtiState *s, int exti_no, int signal)
{
    if (fm3_exti_get_mode(s, exti_no) == signal) {
        s->request_latch |= (1 << exti_no);
    } else {
        s->request_latch &= ~(1 << exti_no);
    }
}

static int fm3_exti_check_port(int exti_no, int port_no)
{
    int export, export_setting;
    int port_type = fm3_board_get_port_info(port_no);
    int port_setting = fm3_gpio_get_port_setting(port_no);

    if (port_type != port_setting) {
        DPRINTF("FM3_EXTI: [P%02X] port_type = %d, port_setting=%d\n", 
                port_no, port_type, port_setting);
        return false;
    }

    export = (exti_no < 16)? FM3_PORT_EPFR06_EINT0:
                             FM3_PORT_EPFR15_EINT1;
    export_setting = fm3_gpio_get_extport_setting(export);
    return fm3_board_check_extport_exti(exti_no, export_setting);
}

void fm3_exti_set_request(int exti_no, int port_val)
{
    Fm3ExtiState *s = fm3_exti_state;

    if (exti_no < 0 || FM3_EXTI_NUM <= exti_no)
        return;

    if (fm3_exti_check_port(exti_no, s->port_no[exti_no])) {
        s->signal[exti_no] = fm3_exti_get_signal(s, exti_no, port_val);
        fm3_exti_request_latch(s, exti_no, s->signal[exti_no]);
        fm3_exti_update_irq(s); 
    } else {
        DPRINTF("FM3_EXTI: INT%02d=%d Ignored\n", exti_no, port_val);
    }
}

static inline void fm3_exti_set_irq(Fm3ExtiState *s, int irq_no, int irq_flag)
{
    if (FM3_EXTI_IRQ_NUM <= irq_no)
        return;

    if (s->irq_flag[irq_no] != irq_flag) {
        qemu_set_irq(s->irq[irq_no], irq_flag);
        s->irq_flag[irq_no] = irq_flag;
    }
}

static void fm3_exti_update_irq(Fm3ExtiState *s)
{
    uint32_t request;
    int irq_0 = 0, irq_1 = 0;

    request = s->request_latch & s->enable;
    irq_0 = (request & 0x7f) != 0;  /* EINT#0 - 7 */
    irq_1 = (request & ~0x7f) != 0; /* EINT#8 - 31 */

    fm3_exti_set_irq(s, 0, irq_0);
    fm3_exti_set_irq(s, 1, irq_1);
}

/* --- ハードウェア制御情報 --- */

static uint64_t fm3_exti_read(void *opaque, hwaddr offset,
                              unsigned size)
{
    Fm3ExtiState *s = (Fm3ExtiState *)opaque;
    uint64_t retval = 0;

    switch (offset & 0xff) {
    case FM3_EXTI_ENIR:
        retval = s->enable;
        break;
    case FM3_EXTI_EIRR:
        retval = s->request_latch;
        break;
    case FM3_EXTI_EICL:
        retval = 0xffffffff;
        break;
    case FM3_EXTI_ELVR:
        retval = s->mode_0;
        break;
    case FM3_EXTI_ELVR1:
        retval = s->mode_1;
        break;
    case FM3_EXTI_NMIRR:
        retval = 0;
        break;
    case FM3_EXTI_NMICL:
        retval = 1;
        break;
    default:
        goto out;
    }

out:
    DPRINTF("%s : 0x%08x ---> 0x%08x\n", __func__, offset, retval);
    return retval;
}

static void fm3_exti_write(void *opaque, hwaddr offset,
                           uint64_t value, unsigned size)
{
    Fm3ExtiState *s = (Fm3ExtiState *)opaque;
    int i;
    uint32_t mode, re_request;

    DPRINTF("%s: 0x%08x <--- 0x%08x \n", __func__, offset, value);

    switch (offset & 0xff) {
    case FM3_EXTI_ENIR:
        s->enable = value;
        fm3_exti_update_irq(s); 
        break;
    case FM3_EXTI_EIRR:
        break;
    case FM3_EXTI_EICL:
        s->request_latch &= value;

        re_request = 0;
        for (i = 0; i < FM3_EXTI_NUM; i++) {
            mode = fm3_exti_get_mode(s, i);
            if ((s->signal[i] == mode) &&
                (mode == FM3_EXTI_ELVR_LEVEL_LOW ||
                 mode == FM3_EXTI_ELVR_LEVEL_HIGH)) {
                re_request |= (1 << i);
            }
        }

        if (re_request)
            s->request_latch |= re_request;
        
        fm3_exti_update_irq(s); 
        break;
    case FM3_EXTI_ELVR:
        s->mode_0 = value;
        break;
    case FM3_EXTI_ELVR1:
        s->mode_1 = value;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps fm3_exti_mem_ops = {
    .read = fm3_exti_read,
    .write = fm3_exti_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int fm3_exti_get_port_no(int exti_no,  enum FM3_PINPACKAGE pkg)
{
    int pin_no;
    pin_no = fm3_board_get_exti_pin(exti_no);
    return fm3_gpio_get_port_from_pin(pin_no, pkg);
}

static int fm3_exti_init(SysBusDevice *dev)
{
	DeviceState		*devs	= DEVICE(dev);
    Fm3ExtiState	*s		= FM3_EXTI(devs);
    enum FM3_PINPACKAGE pkg = FM3_PINPACKAGE_LQFP176;
    int i;
    
    sysbus_init_irq(dev, &s->irq[0]);		/* 割り込みを登録 */
    sysbus_init_irq(dev, &s->irq[1]);		/* 割り込みを登録 */

    memory_region_init_io(&s->mmio, OBJECT(s), &fm3_exti_mem_ops, s, TYPE_FM3_EXTI, 0x1000);
    sysbus_init_mmio(dev, &s->mmio);

    fm3_exti_state = s;
    s->enable = 0;
    s->request_latch = 0;
    s->mode_0 = 0;
    s->mode_1 = 0;
    s->irq_flag[0] = 0;
    s->irq_flag[1] = 0;
    for (i = 0; i < FM3_EXTI_NUM; i++)
        s->signal[i] = -1;

    s->port_no[12] = fm3_exti_get_port_no(12, pkg);
    s->port_no[13] = fm3_exti_get_port_no(13, pkg);
    s->port_no[14] = fm3_exti_get_port_no(14, pkg);
    s->port_no[15] = fm3_exti_get_port_no(15, pkg);

    return 0;
}

/* --- QEMUへの登録関連 --- */
static Property fm3_exti_properties[] = {
    DEFINE_PROP_UINT32("mode_0"			, Fm3ExtiState	, mode_0			, 0),
    DEFINE_PROP_UINT32("mode_1"			, Fm3ExtiState	, mode_1			, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void fm3_exti_class_init(ObjectClass *klass, void *data)
{
	DeviceClass			*dc	= DEVICE_CLASS(klass);
	SysBusDeviceClass	*k	= SYS_BUS_DEVICE_CLASS(klass);

	k->init		= fm3_exti_init;			/* 初期化関数を登録		*/
	dc->desc	= TYPE_FM3_EXTI;	/* ハードウェア名称		*/
	dc->props	= fm3_exti_properties;	/* 内部情報				*/
}

static const TypeInfo fm3_exti_info = {
	.name			= TYPE_FM3_EXTI,		/* ハードウェア名称		*/
	.parent			= TYPE_SYS_BUS_DEVICE,	/* 接続バス				*/
	.instance_size	= sizeof(Fm3ExtiState),	/* インスタンスサイズ	*/
	.class_init		= fm3_exti_class_init,	/* 初期化関数のポインタ	*/
};

static void fm3_register_devices(void)
{
    type_register_static(&fm3_exti_info);
}

type_init(fm3_register_devices)
