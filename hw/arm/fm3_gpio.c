/*
 * Fujitsu FM3 GPIO
 *
 * Copyright (c) 2012 Pylone, Inc.
 * Written by Masashi YOKOTA <yokota@pylone.jp>
 *
 * This code is licensed under the GNU GPL v2.
 */

#include <ctype.h>
#include "hw/sysbus.h"
#include "sysemu/sysemu.h"
#include "qapi/qmp/qerror.h"
#include "sysemu/char.h"
#include "fm3.h"
#include "fm3_board_config.h"
#include "exec/gdbstub.h"

//#define FM3_DEBUG_GPIO
#define TYPE_FM3_GPIO		"fm3.gpio"
#define TYPE_FM3_GPIO_CTRL	"fm3-gpio-control"

#ifdef FM3_DEBUG_GPIO
#define DPRINTF(fmt, ...)                                       \
    do { printf(fmt, ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif

#define FM3_GPIO_BLOCK_NUM 16

typedef struct {
    SysBusDevice busdev;
    MemoryRegion mmio;
    uint32_t mode[FM3_GPIO_BLOCK_NUM];
    uint32_t ext_mode[FM3_GPIO_BLOCK_NUM];
    uint32_t dir[FM3_GPIO_BLOCK_NUM];
    uint32_t in[FM3_GPIO_BLOCK_NUM];
    uint32_t out[FM3_GPIO_BLOCK_NUM];
} Fm3GpioState;
#define FM3_GPIO(obj) \
    OBJECT_CHECK(Fm3GpioState, (obj), TYPE_FM3_GPIO)

typedef struct {
    SysBusDevice busdev;
    CharDriverState *chr;
} Fm3GpioControlState;
#define FM3_GPIO_CTRL(obj) \
    OBJECT_CHECK(Fm3GpioControlState, (obj), TYPE_FM3_GPIO_CTRL)
    
#define FM3_GPIO_REG_PFR_BASE           (0x000)
#define FM3_GPIO_REG_PCR_BASE           (0x100)
#define FM3_GPIO_REG_DDR_BASE           (0x200)
#define FM3_GPIO_REG_PDIR_BASE          (0x300)
#define FM3_GPIO_REG_PDOR_BASE          (0x400)
#define FM3_GPIO_REG_ADE_BASE           (0x500)
#define FM3_GPIO_REG_EPFR_BASE          (0x600)

#define FM3_GPIO_REG_PFR_GPIO           (0)
#define FM3_GPIO_REG_PFR_PERIPHERAL     (1)
#define FM3_GPIO_REG_DDR_IN             (0)
#define FM3_GPIO_REG_DDR_OUT            (1)

static Fm3GpioState *fm3_gpio_state;
static Fm3GpioControlState *fm3_gpio_ctrl_state;

static const int fm3_gpio_pin_table[][FM3_PINPACKAGE_MAX] = {
    { 134,  110, }, /* P00 */
    { 135,  111, }, /* P01 */
    { 136,  112, }, /* P02 */
    { 137,  113, }, /* P03 */
    { 138,  114, }, /* P04 */
    { 8  ,  8  , }, /* P05 */
    { 9  ,  9  , }, /* P06 */
    { 10 ,  10 , }, /* P07 */
    { 11 ,  11 , }, /* P08 */
    { 12 ,  12 , }, /* P09 */
    { -1 ,  -1 , }, /* P0A */
    { -1 ,  -1 , }, /* P0B */
    { -1 ,  -1 , }, /* P0C */
    { -1 ,  -1 , }, /* P0D */
    { -1 ,  -1 , }, /* P0E */
    { -1 ,  -1 , }, /* P0F */
    { 90 ,  74 , }, /* P10 */
    { 91 ,  75 , }, /* P11 */
    { 92 ,  76 , }, /* P12 */
    { 93 ,  77 , }, /* P13 */
    { 94 ,  78 , }, /* P14 */
    { 95 ,  79 , }, /* P15 */
    { 96 ,  80 , }, /* P16 */
    { 97 ,  81 , }, /* P17 */
    { 98 ,  82 , }, /* P18 */
    { 99 ,  83 , }, /* P19 */
    { 100,  84 , }, /* P1A */
    { 101,  85 , }, /* P1B */
    { 102,  86 , }, /* P1C */
    { 103,  87 , }, /* P1D */
    { 104,  88 , }, /* P1E */
    { 105,  89 , }, /* P1F */
    { 127,  103, }, /* P20 */
    { 126,  102, }, /* P21 */
    { 125,  101, }, /* P22 */
    { 124,  100, }, /* P23 */
    { 123,  99 , }, /* P24 */
    { 122,  98 , }, /* P25 */
    { 121,  97 , }, /* P26 */
    { 120,  96 , }, /* P27 */
    { 119,  95 , }, /* P28 */
    { 118,  94 , }, /* P29 */
    { -1 ,  -1 , }, /* P2A */
    { -1 ,  -1 , }, /* P2B */
    { -1 ,  -1 , }, /* P2C */
    { -1 ,  -1 , }, /* P2D */
    { -1 ,  -1 , }, /* P2E */
    { -1 ,  -1 , }, /* P2F */
    { 28 ,  -1 , }, /* P30 */
    { 29 ,  -1 , }, /* P31 */
    { 30 ,  -1 , }, /* P32 */
    { 31 ,  -1 , }, /* P33 */
    { 32 ,  -1 , }, /* P34 */
    { 33 ,  -1 , }, /* P35 */
    { 34 ,  26 , }, /* P36 */
    { 35 ,  27 , }, /* P37 */
    { 36 ,  28 , }, /* P38 */
    { 37 ,  29 , }, /* P39 */
    { 38 ,  30 , }, /* P3A */
    { 39 ,  31 , }, /* P3B */
    { 40 ,  32 , }, /* P3C */
    { 41 ,  33 , }, /* P3D */
    { 42 ,  34 , }, /* P3E */
    { 43 ,  35 , }, /* P3F */
    { 46 ,  38 , }, /* P40 */
    { 47 ,  39 , }, /* P41 */
    { 48 ,  40 , }, /* P42 */
    { 49 ,  41 , }, /* P43 */
    { 50 ,  42 , }, /* P44 */
    { 51 ,  43 , }, /* P45 */
    { 55 ,  47 , }, /* P46 */
    { 56 ,  48 , }, /* P47 */
    { 58 ,  50 , }, /* P48 */
    { 59 ,  51 , }, /* P49 */
    { 60 ,  52 , }, /* P4A */
    { 61 ,  53 , }, /* P4B */
    { 62 ,  54 , }, /* P4C */
    { 63 ,  55 , }, /* P4D */
    { 64 ,  56 , }, /* P4E */
    { -1 ,  -1 , }, /* P4F */
    { 13 ,  13 , }, /* P50 */
    { 14 ,  14 , }, /* P51 */
    { 15 ,  15 , }, /* P52 */
    { 16 ,  16 , }, /* P53 */
    { 17 ,  17 , }, /* P54 */
    { 18 ,  18 , }, /* P55 */
    { 19 ,  19 , }, /* P56 */
    { 20 ,  20 , }, /* P57 */
    { 21 ,  21 , }, /* P58 */
    { 22 ,  22 , }, /* P59 */
    { 23 ,  23 , }, /* P5A */
    { 24 ,  24 , }, /* P5B */
    { 25 ,  -1 , }, /* P5C */
    { 26 ,  -1 , }, /* P5D */
    { -1 ,  -1 , }, /* P5E */
    { -1 ,  -1 , }, /* P5F */
    { 169,  139, }, /* P60 */
    { 168,  138, }, /* P61 */
    { 167,  137, }, /* P62 */
    { -1 ,  -1 , }, /* P63 */
    { -1 ,  -1 , }, /* P64 */
    { -1 ,  -1 , }, /* P65 */
    { -1 ,  -1 , }, /* P66 */
    { -1 ,  -1 , }, /* P67 */
    { -1 ,  -1 , }, /* P68 */
    { -1 ,  -1 , }, /* P69 */
    { -1 ,  -1 , }, /* P6A */
    { -1 ,  -1 , }, /* P6B */
    { -1 ,  -1 , }, /* P6C */
    { -1 ,  -1 , }, /* P6D */
    { -1 ,  -1 , }, /* P6E */
    { -1 ,  -1 , }, /* P6F */
    { 65 ,  57 , }, /* P70 */
    { 66 ,  58 , }, /* P71 */
    { 67 ,  59 , }, /* P72 */
    { 68 ,  60 , }, /* P73 */
    { 69 ,  61 , }, /* P74 */
    { 70 ,  62 , }, /* P75 */
    { 71 ,  63 , }, /* P76 */
    { 72 ,  64 , }, /* P77 */
    { 73 ,  65 , }, /* P78 */
    { 74 ,  66 , }, /* P79 */
    { 75 ,  67 , }, /* P7A */
    { 76 ,  -1 , }, /* P7B */
    { 77 ,  -1 , }, /* P7C */
    { 78 ,  -1 , }, /* P7D */
    { 79 ,  -1 , }, /* P7E */
    { 80 ,  -1 , }, /* P7F */
    { 174,  142, }, /* P80 */
    { 175,  143, }, /* P81 */
    { 130,  106, }, /* P82 */
    { 131,  107, }, /* P83 */
    { -1 ,  -1 , }, /* P84 */
    { -1 ,  -1 , }, /* P85 */
    { -1 ,  -1 , }, /* P86 */
    { -1 ,  -1 , }, /* P87 */
    { -1 ,  -1 , }, /* P88 */
    { -1 ,  -1 , }, /* P89 */
    { -1 ,  -1 , }, /* P8A */
    { -1 ,  -1 , }, /* P8B */
    { -1 ,  -1 , }, /* P8C */
    { -1 ,  -1 , }, /* P8D */
    { -1 ,  -1 , }, /* P8E */
    { -1 ,  -1 , }, /* P8F */
    { 139,  -1 , }, /* P90 */
    { 140,  -1 , }, /* P91 */
    { 141,  -1 , }, /* P92 */
    { 142,  -1 , }, /* P93 */
    { 143,  -1 , }, /* P94 */
    { 144,  -1 , }, /* P95 */
    { -1 ,  -1 , }, /* P96 */
    { -1 ,  -1 , }, /* P97 */
    { -1 ,  -1 , }, /* P98 */
    { -1 ,  -1 , }, /* P99 */
    { -1 ,  -1 , }, /* P9A */
    { -1 ,  -1 , }, /* P9B */
    { -1 ,  -1 , }, /* P9C */
    { -1 ,  -1 , }, /* P9D */
    { -1 ,  -1 , }, /* P9E */
    { -1 ,  -1 , }, /* P9F */
    {  2 ,   2 , }, /* PA0 */
    {  3 ,   3 , }, /* PA1 */
    {  4 ,   4 , }, /* PA2 */
    {  5 ,   5 , }, /* PA3 */
    {  6 ,   6 , }, /* PA4 */
    {  7 ,   7 , }, /* PA5 */
    { -1 ,  -1 , }, /* PA6 */
    { -1 ,  -1 , }, /* PA7 */
    { -1 ,  -1 , }, /* PA8 */
    { -1 ,  -1 , }, /* PA9 */
    { -1 ,  -1 , }, /* PAA */
    { -1 ,  -1 , }, /* PAB */
    { -1 ,  -1 , }, /* PAC */
    { -1 ,  -1 , }, /* PAD */
    { -1 ,  -1 , }, /* PAE */
    { -1 ,  -1 , }, /* PAF */
    { 110,  -1 , }, /* PB0 */
    { 111,  -1 , }, /* PB1 */
    { 112,  -1 , }, /* PB2 */
    { 113,  -1 , }, /* PB3 */
    { 114,  -1 , }, /* PB4 */
    { 115,  -1 , }, /* PB5 */
    { 116,  -1 , }, /* PB6 */
    { 117,  -1 , }, /* PB7 */
    { -1 ,  -1 , }, /* PB8 */
    { -1 ,  -1 , }, /* PB9 */
    { -1 ,  -1 , }, /* PBA */
    { -1 ,  -1 , }, /* PBB */
    { -1 ,  -1 , }, /* PBC */
    { -1 ,  -1 , }, /* PBD */
    { -1 ,  -1 , }, /* PBE */
    { -1 ,  -1 , }, /* PBF */
    { 145,  115, }, /* PC0 */
    { 146,  116, }, /* PC1 */
    { 147,  117, }, /* PC2 */
    { 148,  118, }, /* PC3 */
    { 149,  119, }, /* PC4 */
    { 150,  120, }, /* PC5 */
    { 151,  121, }, /* PC6 */
    { 152,  122, }, /* PC7 */
    { 153,  123, }, /* PC8 */
    { 154,  124, }, /* PC9 */
    { 155,  125, }, /* PCA */
    { 158,  128, }, /* PCB */
    { 159,  129, }, /* PCC */
    { 160,  130, }, /* PCD */
    { 161,  131, }, /* PCE */
    { 162,  132, }, /* PCF */
    { 163,  133, }, /* PD0 */
    { 164,  134, }, /* PD1 */
    { 165,  135, }, /* PD2 */
    { 166,  136, }, /* PD3 */
    { -1 ,  -1 , }, /* PD4 */
    { -1 ,  -1 , }, /* PD5 */
    { -1 ,  -1 , }, /* PD6 */
    { -1 ,  -1 , }, /* PD7 */
    { -1 ,  -1 , }, /* PD8 */
    { -1 ,  -1 , }, /* PD9 */
    { -1 ,  -1 , }, /* PDA */
    { -1 ,  -1 , }, /* PDB */
    { -1 ,  -1 , }, /* PDC */
    { -1 ,  -1 , }, /* PDD */
    { -1 ,  -1 , }, /* PDE */
    { -1 ,  -1 , }, /* PDF */
    { 84 ,  68 , }, /* PE0 */
    { -1 ,  -1 , }, /* PE1 */
    { 86 ,  70 , }, /* PE2 */
    { 87 ,  71 , }, /* PE3 */
    { -1 ,  -1 , }, /* PE4 */
    { -1 ,  -1 , }, /* PE5 */
    { -1 ,  -1 , }, /* PE6 */
    { -1 ,  -1 , }, /* PE7 */
    { -1 ,  -1 , }, /* PE8 */
    { -1 ,  -1 , }, /* PE9 */
    { -1 ,  -1 , }, /* PEA */
    { -1 ,  -1 , }, /* PEB */
    { -1 ,  -1 , }, /* PEC */
    { -1 ,  -1 , }, /* PED */
    { -1 ,  -1 , }, /* PEE */
    { -1 ,  -1 , }, /* PEF */
    { 81 ,  -1 , }, /* PF0 */
    { 82 ,  -1 , }, /* PF1 */
    { 83 ,  -1 , }, /* PF2 */
    { 170,  -1 , }, /* PF3 */
    { 171,  -1 , }, /* PF4 */
    { 172,  140, }, /* PF5 */
    { 128,  104, }, /* PF6 */
    { -1 ,  -1 , }, /* PF7 */
    { -1 ,  -1 , }, /* PF8 */
    { -1 ,  -1 , }, /* PF9 */
    { -1 ,  -1 , }, /* PFA */
    { -1 ,  -1 , }, /* PFB */
    { -1 ,  -1 , }, /* PFC */
    { -1 ,  -1 , }, /* PFD */
    { -1 ,  -1 , }, /* PFE */
    { -1 ,  -1 , }, /* PFF */
    {  0 ,   0 , }, /* --- */
};

int fm3_gpio_get_port_from_pin(int pin_no, enum FM3_PINPACKAGE pkg)
{
    int i; 

    if (FM3_PINPACKAGE_MAX <= pkg || pin_no < 0) {
        printf("%s: Invalid pin-package type(%d) or pin_no(%d)\n", 
               __func__, pkg, pin_no);
        return -1;
    }

    for (i = 0; fm3_gpio_pin_table[i][pkg] != 0; i++) {
        if (fm3_gpio_pin_table[i][pkg] == pin_no) {
            return i;
        }
    }

    return -1;
}

uint32_t fm3_gpio_get_port_setting(uint32_t port_no)
{
    Fm3GpioState *s = fm3_gpio_state;
    uint32_t retval = 0;
    uint32_t block_no = FM3_PORT_TO_BLOCKNO(port_no);
    uint32_t bit_pos = FM3_PORT_TO_BITPOS(port_no);
    assert(block_no < FM3_GPIO_BLOCK_NUM);

    if (s) {
        retval = (s->mode[block_no] >> bit_pos) & 1;
    }

    return retval;
}

uint32_t fm3_gpio_get_extport_setting(uint32_t block_no)
{
    Fm3GpioState *s = fm3_gpio_state;
    uint32_t retval = 0;

    assert(block_no < FM3_GPIO_BLOCK_NUM);

    if (s) {
        retval = s->ext_mode[block_no];
    }

    return retval;
}

static inline uint32_t fm3_gpio_make_port_no(uint32_t block_no, uint32_t bit_pos)
{
    return ((block_no & 0xf) << 4) | (bit_pos & 0xf);
}

static int fm3_gpio_send_msg(Fm3GpioControlState *s, char *buf, int len)
{
    Fm3GpioControlState *state = s;
    if (!state)
        state = fm3_gpio_ctrl_state;

    return qemu_chr_fe_write(state->chr, (const uint8_t *)buf, len);
}

static uint64_t fm3_gpio_read(void *opaque, hwaddr offset, 
                              unsigned size)
{
    Fm3GpioState *s = (Fm3GpioState *)opaque;
    uint32_t *p = NULL;
    uint32_t block_no = (offset & 0xff) >> 2;
    uint32_t reg = offset & ~0xff;
    uint64_t retval = 0;

#if 0
    if (FM3_GPIO_NUM <= block_no)
        hw_error("%s: Invalid port No = %d", __func__, block_no);
#endif
    switch (reg) {
    case FM3_GPIO_REG_PFR_BASE:
        p = &s->mode[block_no];
        break;
    case FM3_GPIO_REG_DDR_BASE:
        p = &s->dir[block_no];
        break;
    case FM3_GPIO_REG_PDIR_BASE:
        p = &s->in[block_no];
        break;
    case FM3_GPIO_REG_PDOR_BASE:
        p = &s->out[block_no];
        break;
    case FM3_GPIO_REG_EPFR_BASE:
        p = &s->ext_mode[block_no];
        break;
    default:
        goto out;
    }

    switch (size) {
    case 1:
        retval = (*p >> ((offset & 3)*8)) & 0xff;
        break;
    case 2:
        if ((offset & 2)) {
            retval = (*p >> 16) & 0xffff;
        } else {
            retval = *p & 0xffff;
        }
        break;
    case 4:
        retval = *p;
        break;
    default:
        printf("%s: Invalid access size = %d\n", __func__, size);
        goto out;
    }
out:
    DPRINTF("%s : 0x%08x ---> 0x%08x (block_no=%d)\n", __func__, offset, retval, block_no);
    return retval;
}

static char fm3_gpio_get_io_char(int get_dir, uint32_t block_no, uint32_t bit_pos)
{
    Fm3GpioState *s = fm3_gpio_state; 
    uint32_t mode = (s->mode[block_no] >> bit_pos) & 1;
    uint32_t dir = (s->dir[block_no] >> bit_pos) & 1;
    uint32_t *p;
    char ret = '-';

    if ((mode == FM3_GPIO_REG_PFR_GPIO) && (dir == get_dir)){
        if (get_dir == FM3_GPIO_REG_DDR_OUT) {
            p = &s->out[block_no];
        } else {
            p = &s->in[block_no];
        }
        ret = ((*p >> bit_pos) & 1)? 'H':'L';
    }

    return ret;
}

#if 0
static char fm3_gpio_get_input_char(uint32_t block_no, uint32_t bit_pos)
{
    return fm3_gpio_get_io_char(FM3_GPIO_REG_DDR_IN, block_no, bit_pos);
}

static char fm3_gpio_get_output_char(uint32_t block_no, uint32_t bit_pos)
{
    return fm3_gpio_get_io_char(FM3_GPIO_REG_DDR_OUT, block_no, bit_pos);
}
#endif

static void fm3_gpio_make_io_str(uint32_t block_no, int dir, char *buf, 
                                 int32_t start, uint32_t width)
{
    uint32_t i, pos = start;
    char *q;
    for (i = 0, q = buf + (width - 1); i < width; i++, q--) {
        *q = fm3_gpio_get_io_char(dir, block_no, pos++);
    }
}

#if 0
static void fm3_gpio_make_input_str(uint32_t block_no,
        char *buf, uint32_t width)
{
    fm3_gpio_make_io_str(block_no, FM3_GPIO_REG_DDR_IN, buf, width);
}
#endif

static void fm3_gpio_make_output_str(uint32_t block_no, char *buf, 
                                     int32_t start, uint32_t width)
{
    fm3_gpio_make_io_str(block_no, FM3_GPIO_REG_DDR_OUT, buf, 
            start, width);
}

static char* fm3_gpio_make_port_block_msg(char *p, uint32_t block_no)
{
    const size_t size = 3 + 16 + 2 + 1;  /* max size of string */

    snprintf(p, size, "%X*:%016d\r\n", block_no, 0);
    fm3_gpio_make_output_str(block_no, &p[3], 0, 16);
    return p + qemu_strnlen(p, size);
}

static char* fm3_gpio_make_port_bit_msg(char *p, uint32_t block_no,
                                        uint32_t bit_pos)
{
    const size_t size = 3 + 1 + 2 + 1;  /* max size of string */

    snprintf(p, size, "%X%X:-\r\n", block_no, bit_pos);
    fm3_gpio_make_output_str(block_no, &p[3], bit_pos, 1);
    return p + qemu_strnlen(p, size);
}


static void fm3_gpio_write(void *opaque, hwaddr offset,
                           uint64_t value, unsigned size)
{
    Fm3GpioState *s = (Fm3GpioState *)opaque;
    uint32_t block_no = (offset & 0xff) >> 2;
    uint32_t *p = NULL;
    uint32_t tmp;
    uint32_t out = s->out[block_no];
    uint32_t dir = s->dir[block_no];
    uint32_t reg = offset & ~0xff;
    char msg[32];

    DPRINTF("%s: 0x%08x <--- 0x%08x (block_no=%d)\n", __func__, offset, value, block_no);
#if 0
    if (FM3_GPIO_NUM <= block_no)
        hw_error("%s: Invalid port No = %d", __func__, block_no);
#endif

    switch (reg) {
    case FM3_GPIO_REG_PFR_BASE:
        p = &s->mode[block_no];
        break;
    case FM3_GPIO_REG_DDR_BASE:
        p = &s->dir[block_no];
        break;
    case FM3_GPIO_REG_PDOR_BASE:
        p = &s->out[block_no];
        break;
    case FM3_GPIO_REG_EPFR_BASE:
        p = &s->ext_mode[block_no];
        break;
    default:
        return;
    }

    switch (size) {
    case 1:
        tmp = *p;
        tmp &= ~(0xff << ((offset & 3)*8));
        tmp |= value << ((offset & 3)*8);
        *p = tmp;
        break;
    case 2:
        tmp = *p;
        if ((offset & 2)) {
            tmp = (tmp & 0xffff) | (value << 16);
        } else {
            tmp = (tmp & 0xffff0000) | value;
        }
        *p = tmp;
        break;
    case 4:
        *p = value;
        break;
    default:
        printf("%s: Invalid access size = %d\n", __func__, size);
        return;
    }

    if (out != s->out[block_no] || dir != s->dir[block_no]) {
        /* update the input data reg when the port direction is output */
        s->in[block_no] = (s->in[block_no] & ~s->dir[block_no]) | 
                          (s->out[block_no] & s->dir[block_no]);

        /* send the message which informs that the port is changed */
        fm3_gpio_make_port_block_msg(msg, block_no);
        fm3_gpio_send_msg(NULL, msg, strlen(msg));
    }
}

static const MemoryRegionOps fm3_gpio_mem_ops = {
    .read = fm3_gpio_read,
    .write = fm3_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int fm3_gpio_init(SysBusDevice *dev)
{
	DeviceState		*devs	= DEVICE(dev);
    Fm3GpioState	*s		= FM3_GPIO(devs);

    memory_region_init_io(&s->mmio, OBJECT(s), &fm3_gpio_mem_ops, s, TYPE_FM3_GPIO, 0x1000);
    sysbus_init_mmio(dev, &s->mmio);

    fm3_gpio_state = s;
    return 0;
}

/*
 * chardev callbacks
 */

static int fm3_gpio_chardev_can_read(void *opaque)
{
    //Fm3GpioState *s = opaque;

    if (fm3_gpio_state)
        return 32;
    else
        return 0;
}

static bool fm3_gpio_check_port(Fm3GpioState *s, uint32_t block_no, uint32_t bit_pos, int dir)
{
    if (FM3_GPIO_BLOCK_NUM < block_no)
        return false;
    if (((s->mode[block_no] >> bit_pos) & 1) != FM3_GPIO_REG_PFR_GPIO)
        return false;
    if (((s->dir[block_no] >> bit_pos) & 1) != dir)
        return false;
    return true;
}

static void fm3_gpio_ctrl_port_gpio(Fm3GpioState *s, int set, uint32_t block_no, uint32_t bit_pos)
{
    assert(s);
   
    switch (set) {
    case 'H':
        s->in[block_no] |= (1 << bit_pos);
        break;
    case 'L':
        s->in[block_no] &= ~(1 << bit_pos);
        break;
    default:
        break;
    }
}

static void fm3_gpio_ctrl_port_exti(Fm3GpioState *s, int set, int port_no)
{
    int exti_no;

    exti_no = fm3_board_port_to_extint(port_no);
    switch (set) {
    case 'H':
        fm3_exti_set_request(exti_no, 1);
        break;
    case 'L':
        fm3_exti_set_request(exti_no, 0);
        break;
    default:
        break;
    }
}

static void fm3_gpio_ctrl_port(int set, uint32_t block_no, uint32_t bit_pos)
{
    Fm3GpioState *s = fm3_gpio_state;
    uint32_t port_no = fm3_gpio_make_port_no(block_no, bit_pos);

    if (0 <= fm3_board_port_to_uart(port_no)) {
        printf("FM3_GPIO_CTRL: P%02X is assigned for MFS.\n", port_no);
        return;
    }

    if (fm3_gpio_check_port(s, block_no, bit_pos, FM3_GPIO_REG_DDR_OUT)) {
        printf("FM3_GPIO_CTRL: P%X%X is set for GPIO/OUT - Ignored\n", block_no, bit_pos);
        return;
    }

    fm3_gpio_ctrl_port_gpio(s, set, block_no, bit_pos);
    fm3_gpio_ctrl_port_exti(s, set, port_no);
}

#if 0
static int fm3_gpio_read_port(uint32_t block_no, uint32_t bit_pos)
{
    Fm3GpioState *s = fm3_gpio_state;

    if (!s || !fm3_gpio_check_port(s, block_no, bit_pos, FM3_GPIO_REG_DDR_OUT))
        return -1;

    DPRINTF("%s: block_no=%d bit_pos=%d value=%08x\n", __func__,
            block_no, bit_pos, s->out[block_no]);
    return (s->out[block_no] >> bit_pos) & 1;
}
#endif

static int fm3_gpio_check_cmd_str(char *val, int len)
{
    int i;
    char *p = val;
    for (i = 0; i < len; i++, p++) {
        switch (toupper(*p)) {
        case 'H':
        case 'L':
        case '-':
            continue;
        default:
            return -1;
        }
    }
    return 0;
}

static int fm3_gpio_cmdstr_to_hex(char val)
{
    int ret = -1;
    char str[2] = {0};
    if (val == '*') {
        ret = 16;
    } else if (qemu_isxdigit(val)) {
        str[0] = val;
        ret = strtol(str, NULL, 16);
    }
    return ret;
}

static int fm3_gpio_misc_cmd(char *cmd)
{
    int ret = 0;
    int len;
    char *p;

    for (p = cmd; *p; p++)
        if (!qemu_isprint(*p))
            break;
    *p = 0;
    len = strlen(cmd);
    if ((len == 1 && *cmd == 'c') ||
        strcasecmp(cmd, "cont") == 0) {
        vm_start();
        ret = 1;
    }
    return ret;
}

/* --- CharDev関連 --- */
static void fm3_gpio_chardev_read(void *opaque, const uint8_t *buf, int size)
{
    Fm3GpioControlState *s = opaque;
    int i;
    char *p = (char *)buf;
    uint32_t block_no = 0;
    uint32_t bit_pos = 0;
    int scaned = 0;
    char port[4] = {0};
    char val[32];
    char res_str[512] = {0};

    *(p+size) = 0;
    scaned = sscanf(p, "%2[^\r\n]=%s", port, val);
    if (scaned <= 0)
        return;

    if (size < 3 || *(p+2) != '=') {
        /* special case */
        if (fm3_gpio_misc_cmd(p))
            goto out;
        else
            goto err;
    }

    if ((i = fm3_gpio_cmdstr_to_hex(port[0])) < 0)
        goto err;
    block_no = i;

    if ((i = fm3_gpio_cmdstr_to_hex(port[1])) < 0)
        goto err;
    bit_pos = i;

    DPRINTF("%X%X=%s\n", block_no, bit_pos, val);

    if (scaned < 2) {
        /* read */
        char *tmp = res_str;
        if (16 <= block_no) {
            /* read the all ports */
            for (i = 0; i < 16; i++)
                tmp = fm3_gpio_make_port_block_msg(tmp, i);
        } else if (16 <= bit_pos) {
            /* read the port block */
            fm3_gpio_make_port_block_msg(tmp, block_no);
        } else {
            /* read the port bit */
            fm3_gpio_make_port_bit_msg(tmp, block_no, bit_pos);
        }
    } else {
        if (fm3_gpio_check_cmd_str(val, strlen(val)) < 0)
            goto err;

        /* write */
        if (16 <= block_no) {
            goto err;
        } else if (16 <= bit_pos) {
            if (strlen(val) < 16)
                goto err;
            for (i = 0; i < 16; i++)
                fm3_gpio_ctrl_port(toupper(val[15-i]), block_no, i);
        } else {
            fm3_gpio_ctrl_port(toupper(val[0]), block_no, bit_pos);
        }
    }
out:
    pstrcat(res_str, sizeof(res_str), "OK\r\n");
    fm3_gpio_send_msg(s, res_str, strlen(res_str));
    return;
err:
    fm3_gpio_send_msg(s, (char *)"NG\r\n", 4);
}

static void fm3_gpio_chardev_event(void *opaque, int event)
{
    //Fm3GpioControlState *s = opaque;

    switch (event) {
    case CHR_EVENT_OPENED:
    case CHR_EVENT_CLOSED:
        break;
    }
}

static int fm3_gpio_chardev_init(SysBusDevice *dev)
{
    Fm3GpioControlState *s = FM3_GPIO_CTRL(dev);
    if (s->chr == NULL) {
        qerror_report(QERR_MISSING_PARAMETER, "chardev");
        return -1;
    }

    qemu_chr_add_handlers(s->chr, fm3_gpio_chardev_can_read,
                          fm3_gpio_chardev_read, fm3_gpio_chardev_event, s);

    fm3_gpio_ctrl_state = s;
    return 0;
}

/* 通常版定義 */
static Property fm3_gpio_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void fm3_gpio_class_init(ObjectClass *klass, void *data)
{
	DeviceClass			*dc	= DEVICE_CLASS(klass);
	SysBusDeviceClass	*k	= SYS_BUS_DEVICE_CLASS(klass);

	k->init		= fm3_gpio_init;			/* 初期化関数を登録		*/
	dc->desc	= TYPE_FM3_GPIO;			/* ハードウェア名称		*/
	dc->props	= fm3_gpio_properties;		/* 内部情報				*/
}

static const TypeInfo fm3_gpio_info = {
	.name			= TYPE_FM3_GPIO,		/* ハードウェア名称		*/
	.parent			= TYPE_SYS_BUS_DEVICE,	/* 接続バス				*/
	.instance_size	= sizeof(Fm3GpioState),	/* インスタンスサイズ	*/
	.class_init		= fm3_gpio_class_init,	/* 初期化関数のポインタ	*/
};


/* CharDev定義 */
static Property fm3_gpio_ctrl_properties[] = {
    DEFINE_PROP_CHR("chardev"			, Fm3GpioControlState	, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void fm3_gpio_ctrl_class_init(ObjectClass *klass, void *data)
{
	DeviceClass			*dc	= DEVICE_CLASS(klass);
	SysBusDeviceClass	*k	= SYS_BUS_DEVICE_CLASS(klass);

	k->init		= fm3_gpio_chardev_init;	/* 初期化関数を登録		*/
	dc->desc	= TYPE_FM3_GPIO_CTRL;		/* ハードウェア名称		*/
	dc->props	= fm3_gpio_ctrl_properties;	/* 内部情報				*/
	dc->cannot_instantiate_with_device_add_yet = false;
}

static const TypeInfo fm3_gpio_ctrl_info = {
	.name			= TYPE_FM3_GPIO_CTRL,			/* ハードウェア名称		*/
	.parent			= TYPE_SYS_BUS_DEVICE,			/* 接続バス				*/
	.instance_size	= sizeof(Fm3GpioControlState),	/* インスタンスサイズ	*/
	.class_init		= fm3_gpio_ctrl_class_init,		/* 初期化関数のポインタ	*/
};

static void fm3_register_devices(void)
{
    type_register_static(&fm3_gpio_info);
    type_register_static(&fm3_gpio_ctrl_info);
}

type_init(fm3_register_devices)
