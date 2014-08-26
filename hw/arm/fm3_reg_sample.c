#include "hw/sysbus.h"
#include "sysemu/char.h"
#include "qapi/qmp/qerror.h"
#include "hw/devices.h"
#include "hw/arm/arm.h"

#include <stdlib.h>
/*
 ***
  このモジュールがサポートするハードウェア情報の定義
  */

/* ハードウェア名称 */
#define TYPE_FM3_REG_SAMPLE	"fm3.RegSample"
#define FM3_REG_SAMPLE(obj) \
    OBJECT_CHECK(Fm3RegState, (obj), TYPE_FM3_REG_SAMPLE)	/* objをFm3RegStateでキャストしても大丈夫か確認する。 */

#define TYPE_FM3_CHRDEV_REG_SAMPLE	"fm3-ChrDev-RegSample"
#define FM3_CHRDEV_REG_SAMPLE(obj) \
    OBJECT_CHECK(Fm3ChrDevRegState, (obj), TYPE_FM3_CHRDEV_REG_SAMPLE)	/* objをFm3ChrDevRegStateでキャストしても大丈夫か確認する。 */


/* 内部的なハードウェア情報構造体 */
typedef struct {
	/* 必須事項 */
	SysBusDevice parent_obj;
	MemoryRegion iomem;

	/* ハードウェア定義 */
	qemu_irq	irq0;
	uint32_t	reg0;
	uint32_t	reg1;
} Fm3RegState;

typedef struct {
	/* 必須事項 */
	SysBusDevice busdev;
	CharDriverState *chr;

	/* ハードウェア定義 */
	uint32_t	reg0;
	uint32_t	reg1;
} Fm3ChrDevRegState;
Fm3ChrDevRegState	*CheDevState;

/* 内部処理関数 */

/*
 -- リードした時に呼ばれる関数 --
 *opaque	: memory_region_init_ioの第4引数
 offset		: アドレス
 size		: アクセスサイズ
 return		: 読み出したデータ
 */
static uint64_t fm3_reg_read(void *opaque, hwaddr offset,
                            unsigned size)
{
	Fm3RegState *s = (Fm3RegState *)opaque;
	printf("read offset = %x, size = %x\n", offset, size);
	uint64_t retval = 0;

	switch (offset) {
	case 0:
		retval = s->reg0;
		break;
	case 1:
		retval = s->reg1;
		break;
	default:
		retval = 0xCCCCCCCC;
		break;
	}
	return retval;
}

/*
 -- ライトした時に呼ばれる関数 --
 *opaque	: memory_region_init_ioの第4引数
 offset		: アドレス
 value		: データ
 size		: アクセスサイズ
 */
static void fm3_reg_write(void *opaque, hwaddr offset,
                         uint64_t value, unsigned size)
{
	Fm3RegState *s = (Fm3RegState *)opaque;
	printf("write:offset = %x, value = %x, size = %x\n", offset, (unsigned long)value, size);
	switch (offset) {
	case 0:
		s->reg0 = value;
		break;
	case 4:
		s->reg1 = value;
		printf("setirq\n");
		qemu_set_irq(s->irq0, value);
		break;
	default:
		break;
	}
}

/*
 -- ChrDevからの読み出し可能かを確認する（外部からの書き込み）関数 --
 *opaque	: memory_region_init_ioの第4引数
 */
static int fm3_chrdev_can_read(void *opaque)
{
    //Fm3GpioState *s = opaque;

    if (CheDevState)
        return 255;
    else
        return 0;
}

/*
 -- ChrDevからの読み出し（外部からの書き込み）関数 --
 *opaque	: memory_region_init_ioの第4引数
 buf		: データバッファのポインタ
 size		: サイズ
 */
static void fm3_chrdev_read(void *opaque, const uint8_t *buf, int size)
{
    Fm3ChrDevRegState *s = opaque;
	
	char recv_string[256];
	int i;
	
	memset(recv_string, 0, 256);
	for (i = 0; i < size; i++) {
		recv_string[i] = *(buf + i);
		if (i > 253) {
			break;
		}
	}
	qemu_chr_fe_write(s->chr, (const uint8_t *)buf, size);
}

/*
 -- ChrDevのイベント発生時に呼ばれる関数 --
 *opaque	: memory_region_init_ioの第4引数
 event		: イベント
 */
static void fm3_chrdev_event(void *opaque, int event)
{
    //Fm3GpioControlState *s = opaque;

    switch (event) {
    case CHR_EVENT_OPENED:
    case CHR_EVENT_CLOSED:
        break;
    }
}


/* ここで定義された関数がQEMUのCPUから呼ばれる。 */
static const MemoryRegionOps fm3_reg_mem_ops = {
    .read = fm3_reg_read,				/* リードした時に呼ばれる */
    .write = fm3_reg_write,				/* ライトした時に呼ばれる */
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* リセット関数 */
static void fm3_reg_reset(DeviceState *d)
{
    Fm3RegState *s = FM3_REG_SAMPLE(d);
	s->reg0 = 0;
	s->reg1 = 0;
}

static void fm3_chrdev_reg_reset(DeviceState *d)
{
    Fm3ChrDevRegState *s = FM3_CHRDEV_REG_SAMPLE(d);
	s->reg0 = 0;
	s->reg1 = 0;
}

/* 初期化。ハードウェアが生成されたときに呼ばれる。 */
static int fm3_reg_init(SysBusDevice *dev)
{
	DeviceState	*devs	= DEVICE(dev);
    Fm3RegState	*s		= FM3_REG_SAMPLE(devs);

	sysbus_init_irq(dev, &s->irq0);

    memory_region_init_io(&s->iomem, OBJECT(s), &fm3_reg_mem_ops, s, TYPE_FM3_REG_SAMPLE, 0x10);		// レジスタは2個しかないが、適当に0x10バイトにする
    sysbus_init_mmio(dev, &s->iomem);

    return 0;
}

static int fm3_chrdev_reg_init(SysBusDevice *dev)
{
    Fm3ChrDevRegState	*s	= FM3_CHRDEV_REG_SAMPLE(dev);
	if (s->chr == NULL) {
		qerror_report(QERR_MISSING_PARAMETER, "chardev");
		return -1;
	}
    qemu_chr_add_handlers(s->chr, fm3_chrdev_can_read,
                          fm3_chrdev_read, fm3_chrdev_event, s);
	CheDevState = s;

    return 0;
}


/* 内部情報 */
static Property fm3_reg_properties[] = {
    DEFINE_PROP_UINT32("reg0"			, Fm3RegState	, reg0			, 0),
    DEFINE_PROP_UINT32("reg1"			, Fm3RegState	, reg1			, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static Property fm3_chrdev_reg_properties[] = {
    DEFINE_PROP_CHR("chardev"			, Fm3ChrDevRegState	, chr),
    DEFINE_PROP_UINT32("reg0"			, Fm3ChrDevRegState	, reg0			, 0),
    DEFINE_PROP_UINT32("reg1"			, Fm3ChrDevRegState	, reg1			, 0),
    DEFINE_PROP_END_OF_LIST(),
};


/* 初期化。ハードウェア登録時にQEMUから呼ばれる。 */
static void fm3_reg_class_init(ObjectClass *klass, void *data)
{
	DeviceClass			*dc	= DEVICE_CLASS(klass);
	SysBusDeviceClass	*k	= SYS_BUS_DEVICE_CLASS(klass);

	k->init		= fm3_reg_init;			/* 初期化関数を登録		*/
	dc->desc	= TYPE_FM3_REG_SAMPLE;	/* ハードウェア名称		*/
	dc->reset	= fm3_reg_reset;		/* リセット時に呼ばれる */
	dc->props	= fm3_reg_properties;	/* 内部情報				*/
}

static void fm3_chrdev_reg_class_init(ObjectClass *klass, void *data)
{
	DeviceClass			*dc	= DEVICE_CLASS(klass);
	SysBusDeviceClass	*k	= SYS_BUS_DEVICE_CLASS(klass);

	k->init		= fm3_chrdev_reg_init;			/* 初期化関数を登録		*/
	dc->desc	= TYPE_FM3_CHRDEV_REG_SAMPLE;	/* ハードウェア名称		*/
	dc->reset	= fm3_chrdev_reg_reset;		/* リセット時に呼ばれる */
	dc->props	= fm3_chrdev_reg_properties;	/* 内部情報				*/
	dc->cannot_instantiate_with_device_add_yet = false;
}

/* ハードウェア情報。 */
static const TypeInfo fm3_reg_info = {
	.name			= TYPE_FM3_REG_SAMPLE,	/* ハードウェア名称		*/
	.parent			= TYPE_SYS_BUS_DEVICE,	/* 接続バス				*/
	.instance_size	= sizeof(Fm3RegState),	/* インスタンスサイズ	*/
	.class_init		= fm3_reg_class_init,	/* 初期化関数のポインタ	*/
};

static const TypeInfo fm3_chrdev_reg_info = {
	.name			= TYPE_FM3_CHRDEV_REG_SAMPLE,	/* ハードウェア名称		*/
	.parent			= TYPE_SYS_BUS_DEVICE,	/* 接続バス				*/
	.instance_size	= sizeof(Fm3ChrDevRegState),	/* インスタンスサイズ	*/
	.class_init		= fm3_chrdev_reg_class_init,	/* 初期化関数のポインタ	*/
};

/* ハードウェア情報をQEMUに登録。 */
static void fm3_register_devices(void)
{
    type_register_static(&fm3_reg_info);
    type_register_static(&fm3_chrdev_reg_info);
}

type_init(fm3_register_devices)
