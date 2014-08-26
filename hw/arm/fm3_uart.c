/*
 * Fujitsu FM3 UART (MFS)
 *
 * Copyright (c) 2012 Pylone, Inc.
 * Written by Masashi YOKOTA <yokota@pylone.jp>
 *
 * This code is licensed under the GNU GPL v2.
 */

#include "hw/sysbus.h"
#include "hw/devices.h"
#include "sysemu/char.h"
#include "fm3.h"
#include "fm3_board_config.h"

//#define FM3_DEBUG_UART
#define TYPE_FM3_UART	"fm3.uart"

#ifdef FM3_DEBUG_UART
#define DPRINTF(fmt, ...)                                       \
    do { printf(fmt, ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif

#define FM3_UART_REG_SMR_OFFSET     (0x000)
#define FM3_UART_REG_SMR_MD2        (1 << 7)
#define FM3_UART_REG_SMR_MD1        (1 << 6)
#define FM3_UART_REG_SMR_MD0        (1 << 5)
#define FM3_UART_REG_SMR_WUCR       (1 << 4)
#define FM3_UART_REG_SMR_SBL        (1 << 3)
#define FM3_UART_REG_SMR_BDS        (1 << 2)
#define FM3_UART_REG_SMR_SOE        (1 << 0)
#define FM3_UART_REG_SCR_OFFSET     (0x001)
#define FM3_UART_REG_SCR_UPCL       (1 << 7)
#define FM3_UART_REG_SCR_RIE        (1 << 4)
#define FM3_UART_REG_SCR_TIE        (1 << 3)
#define FM3_UART_REG_SCR_TBIE       (1 << 2)
#define FM3_UART_REG_SCR_RXE        (1 << 1)
#define FM3_UART_REG_SCR_TXE        (1 << 0)
#define FM3_UART_REG_ESCR_OFFSET    (0x004)
#define FM3_UART_REG_ESCR_FLWEN     (1 << 7)
#define FM3_UART_REG_ESCR_ESBL      (1 << 6)
#define FM3_UART_REG_ESCR_INV       (1 << 5)
#define FM3_UART_REG_ESCR_PEN       (1 << 4)
#define FM3_UART_REG_ESCR_P         (1 << 3)
#define FM3_UART_REG_ESCR_L2        (1 << 2)
#define FM3_UART_REG_ESCR_L1        (1 << 1)
#define FM3_UART_REG_ESCR_L0        (1 << 0)
#define FM3_UART_REG_SSR_OFFSET     (0x005)
#define FM3_UART_REG_SSR_REC        (1 << 7)
#define FM3_UART_REG_SSR_PE         (1 << 5)
#define FM3_UART_REG_SSR_FRE        (1 << 4)
#define FM3_UART_REG_SSR_ORE        (1 << 3)
#define FM3_UART_REG_SSR_RDRF       (1 << 2)
#define FM3_UART_REG_SSR_TDRE       (1 << 1)
#define FM3_UART_REG_SSR_TBI        (1 << 0)
#define FM3_UART_REG_RDR_OFFSET     (0x008)
#define FM3_UART_REG_TDR_OFFSET     (0x008)
#define FM3_UART_REG_BGR0_OFFSET    (0x00C)
#define FM3_UART_REG_BGR1_OFFSET    (0x00D)
#define FM3_UART_REG_BGR1_EXT       (1 << 7)
#define FM3_UART_REG_ISBA_OFFSET    (0x010)
#define FM3_UART_REG_ISMK_OFFSET    (0x011)
#define FM3_UART_REG_FCR0_OFFSET    (0x014)
#define FM3_UART_REG_FCR0_FLST      (1 << 6)
#define FM3_UART_REG_FCR0_FLD       (1 << 5)
#define FM3_UART_REG_FCR0_FSET      (1 << 4)
#define FM3_UART_REG_FCR0_FCL2      (1 << 3)
#define FM3_UART_REG_FCR0_FCL1      (1 << 2)
#define FM3_UART_REG_FCR0_FE2       (1 << 1)
#define FM3_UART_REG_FCR0_FE1       (1 << 0)
#define FM3_UART_REG_FCR1_OFFSET    (0x015)
#define FM3_UART_REG_FCR1_FLSTE     (1 << 4)
#define FM3_UART_REG_FCR1_FRIIE     (1 << 3)
#define FM3_UART_REG_FCR1_FDRQ      (1 << 2)
#define FM3_UART_REG_FCR1_FTIE      (1 << 1)
#define FM3_UART_REG_FCR1_FSEL      (1 << 0)
#define FM3_UART_REG_FBYTE1_OFFSET  (0x018)
#define FM3_UART_REG_FBYTE2_OFFSET  (0x019)

enum {
    FM3_UART_MODE_NORMAL = 0,
    FM3_UART_MODE_MULTI,
};

#define unsupported(reg)            DPRINTF("FM3_UART: *WARNING* %s is not supported\n", reg)
#define get_ch_no(addr)             ((offset >> 8) & 0xf)
#define get_smr_mode(value)         ((value >> 5) & 7)
#define get_escr_len(value)         (value & 7)
#define is_error(s)                 ((s->ssr & (FM3_UART_REG_SSR_PE|FM3_UART_REG_SSR_FRE|FM3_UART_REG_SSR_ORE)) != 0)
#define is_fifo(s)                  ((s->fcr0 & (FM3_UART_REG_FCR0_FE2|FM3_UART_REG_FCR0_FE1)) != 0)

typedef struct {
    uint8_t data[FM3_UART_FIFO_MAX_LENGTH];
    uint32_t size;
    uint32_t count;
    uint32_t put;
    uint32_t get;
    uint32_t saved_get;
    uint32_t trigger;
} Fm3UartFifo;

typedef struct {
    uint32_t ch_no;
    CharDriverState *chr;
    uint32_t scr;
    uint32_t smr;
    uint32_t ssr;
    uint32_t escr;
    uint32_t bgr1;
    uint32_t bgr0;
    uint32_t fcr1;
    uint32_t fcr0;
    Fm3UartFifo fifo1;
    Fm3UartFifo fifo2;
    Fm3UartFifo *tx_fifo;
    Fm3UartFifo *rx_fifo;
    qemu_irq irq_rx;
    qemu_irq irq_tx;
    int irq_rx_level;
    int irq_tx_level;
    int tx_port;
    int rx_port;
} Fm3UartChState;
typedef struct {
    SysBusDevice busdev;
    MemoryRegion mmio;
    Fm3UartChState ch[FM3_MFS_NUM];
} Fm3UartState;
#define FM3_UART(obj) \
    OBJECT_CHECK(Fm3UartState, (obj), TYPE_FM3_UART)


static Fm3UartState *fm3_uart_state;

static inline bool fm3_uart_ch_has_fifo(int ch)
{
    if (ch < 4) {
        DPRINTF("FM3_UART: ch%d has no FIFO\n", ch);
        return false;
    } else
        return true;
}

static inline uint32_t fm3_uart_get_max_fifo(int ch)
{
    if (fm3_uart_ch_has_fifo(ch))
        return FM3_UART_FIFO_MAX_LENGTH;
    else
        return 1;
}

#if 0
static inline uint32_t fm3_uart_get_max_trigger(int ch, uint32_t req_size)
{
    if (!fm3_uart_ch_has_fifo(ch))
        return 1;

    if (FM3_UART_FIFO_MAX_LENGTH < req_size) {
        printf("FM3_UART: Requested FBYTE is too big(=%d)\n", req_size);
        return FM3_UART_FIFO_MAX_LENGTH;
    }

    return req_size;
}
#else
static inline uint32_t fm3_uart_get_max_trigger(int ch, uint32_t req_size)
{
    uint32_t retval = fm3_uart_get_max_fifo(ch);

    if (retval < req_size) {
        return retval;
    }

    return req_size;
}
#endif

static inline uint32_t fm3_uart_get_irq_stat(int level, int current_ch, 
                                                                int ch)
{
    if (ch < 0 || FM3_MFS_NUM <= ch || ch != current_ch)
        return 0;

    return (level != 0);
}

uint32_t fm3_uart_get_rx_irq_stat(int ch)
{
    Fm3UartChState *s = &fm3_uart_state->ch[ch];
    return fm3_uart_get_irq_stat(s->irq_rx_level, s->ch_no, ch);
}

uint32_t fm3_uart_get_tx_irq_stat(int ch)
{
    Fm3UartChState *s = &fm3_uart_state->ch[ch];
    return fm3_uart_get_irq_stat(s->irq_tx_level, s->ch_no, ch);
}

uint32_t fm3_uart_get_stat_irq_stat(int ch)
{
    return 0; /* not supported */
}

static inline void fm3_uart_clear_tx_irq_flags(Fm3UartChState *s)
{
    s->ssr &= ~(FM3_UART_REG_SSR_TDRE | 
                FM3_UART_REG_SSR_TBI);
}

static inline void fm3_uart_set_tx_irq_flags(Fm3UartChState *s)
{
    s->ssr |= (FM3_UART_REG_SSR_TDRE | 
               FM3_UART_REG_SSR_TBI);
    s->fcr1 |= FM3_UART_REG_FCR1_FDRQ;
}

static inline void fm3_uart_clear_fifo(Fm3UartFifo *f, uint32_t trigger)
{
    f->count = 0;
    f->put = 0;
    f->get = 0;
    f->saved_get = 0;
    f->trigger = trigger;
}

static inline Fm3UartFifo * fm3_uart_get_online_tx_fifo(Fm3UartChState *s)
{
    if (s->fcr0 & FM3_UART_REG_FCR0_FE2) {
        return (&s->fifo2 == s->tx_fifo)? &s->fifo2 : NULL;
    } else if (s->fcr0 & FM3_UART_REG_FCR0_FE1) {
        return (&s->fifo1 == s->tx_fifo)? &s->fifo1 : NULL;
    }
    return NULL;
}

static inline Fm3UartFifo * fm3_uart_get_online_rx_fifo(Fm3UartChState *s)
{
    if (s->fcr0 & FM3_UART_REG_FCR0_FE2) {
        return (&s->fifo2 == s->rx_fifo)? &s->fifo2 : NULL;
    } else if (s->fcr0 & FM3_UART_REG_FCR0_FE1) {
        return (&s->fifo1 == s->rx_fifo)? &s->fifo1 : NULL;
    }
    return NULL;
}


static int fm3_uart_check_port(int ch, int tx, int port_no)
{
    int export, export_setting;
    int port_type = fm3_board_get_port_info(port_no);
    int port_setting = fm3_gpio_get_port_setting(port_no);

    if (port_type != port_setting)
        return false;

    export = (ch < 4)? FM3_PORT_EPFR07_MFS0:
                       FM3_PORT_EPFR08_MFS1;
    export_setting = fm3_gpio_get_extport_setting(export);
    return fm3_board_check_extport_uart(ch, tx, export_setting);
}

static void fm3_uart_chr_write(Fm3UartChState *s, const uint8_t *buf, int len)
{
    if ((s->smr & FM3_UART_REG_SMR_SOE) && 
        fm3_uart_check_port(s->ch_no, 1, s->tx_port))
        qemu_chr_fe_write(s->chr, buf, len); 
}


static void fm3_uart_send_fifo(Fm3UartChState *s) 
{
    Fm3UartFifo *f = fm3_uart_get_online_tx_fifo(s);
    uint8_t *p;
    uint32_t len;
    uint32_t max_fifo = fm3_uart_get_max_fifo(s->ch_no);

    if (!f)
        return;

    p = &f->data[f->get];
    len = f->put - f->get;

    if (f->put < f->get) {
        len = max_fifo - f->get;
        if (len)
            fm3_uart_chr_write(s, p, len); 
        p = &f->data[0];
        len = f->put;
    }

    if (len) 
        fm3_uart_chr_write(s, p, len);

    fm3_uart_clear_fifo(f, f->trigger);
    fm3_uart_set_tx_irq_flags(s);
}

static void fm3_uart_update_tx_irq(Fm3UartChState *s)
{
    int level = 0;

    if (s->scr & FM3_UART_REG_SCR_TIE) {
        level |= !!(s->ssr & FM3_UART_REG_SSR_TDRE);
    }

    if (s->scr & FM3_UART_REG_SCR_TBIE) {
        level |= !!(s->ssr & FM3_UART_REG_SSR_TBI);
    }

    if (s->fcr1 & FM3_UART_REG_FCR1_FTIE) {
        level |= !!(s->fcr1 & FM3_UART_REG_FCR1_FDRQ);
    }

    if (s->irq_tx_level != level) {
        qemu_set_irq(s->irq_tx, level);
        s->irq_tx_level = level;
    }
}

static void fm3_uart_update_rx_irq(Fm3UartChState *s)
{
    int level = 0;
    
    if (s->scr & FM3_UART_REG_SCR_RIE) {
        level |= !!(s->ssr & (FM3_UART_REG_SSR_RDRF|
                              FM3_UART_REG_SSR_ORE |
                              FM3_UART_REG_SSR_FRE |
                              FM3_UART_REG_SSR_PE));
    }

    if (s->irq_rx_level != level) {
        qemu_set_irq(s->irq_rx, level);
        s->irq_rx_level = level;
    }
}

static void fm3_uart_update_irq(Fm3UartChState *s)
{
    fm3_uart_update_tx_irq(s);
    fm3_uart_update_rx_irq(s);
}

#if 0
static inline void fm3_uart_check_trigger(Fm3UartChState *s)
{
    DPRINTF("FM3_UART: Tx-FBYTE = %d\n", s->tx_fifo->trigger);
    DPRINTF("FM3_UART: Rx-FBYTE = %d\n", s->rx_fifo->trigger);
    if (FM3_UART_FIFO_MAX_LENGTH < s->tx_fifo->trigger) {
        DPRINTF("FM3_UART: *WARNING* Tx-FBYTE is too big (%d)\n", 
                s->tx_fifo->trigger);
    }

    if (FM3_UART_FIFO_MAX_LENGTH < s->rx_fifo->trigger) {
        DPRINTF("FM3_UART: *WARNING* Rx-FBYTE is too big (%d)\n", 
                s->rx_fifo->trigger);
    }
}
#endif


static uint64_t fm3_uart_read(void *opaque, hwaddr offset,
                              unsigned size)
{
    int ch = get_ch_no(offset);
    Fm3UartChState *s = &((Fm3UartState *)opaque)->ch[ch];
    Fm3UartFifo *rx_fifo = fm3_uart_get_online_rx_fifo(s);
    uint32_t max_fifo = fm3_uart_get_max_fifo(s->ch_no);
    uint64_t retval = 0;

    offset &= 0xff;
    switch (offset) {
    case FM3_UART_REG_SCR_OFFSET:
        retval = s->scr & ~FM3_UART_REG_SCR_UPCL;
        break;

    case FM3_UART_REG_SMR_OFFSET:
        retval = s->smr;
        break;

    case FM3_UART_REG_SSR_OFFSET:
        retval = s->ssr & ~FM3_UART_REG_SSR_REC;
        break;

    case FM3_UART_REG_ESCR_OFFSET:
        retval = s->escr;
        break;

    case FM3_UART_REG_RDR_OFFSET:
        if (rx_fifo) {
            if (rx_fifo->count) {
                retval = rx_fifo->data[rx_fifo->get++];
                rx_fifo->count--;
                if (max_fifo <= rx_fifo->get)
                    rx_fifo->get = 0;
                if (rx_fifo->count == 0)
                    s->ssr &= ~FM3_UART_REG_SSR_RDRF;
            }
        } else {
            retval = s->rx_fifo->data[0];
            s->rx_fifo->count = 0;
            s->ssr &= ~FM3_UART_REG_SSR_RDRF;
        }
        break;
    case FM3_UART_REG_RDR_OFFSET+1:
        /* ignore */
        break;

    case FM3_UART_REG_BGR1_OFFSET:
        retval = s->bgr1;
        break;

    case FM3_UART_REG_BGR0_OFFSET:
        retval = s->bgr0;
        break;

    case FM3_UART_REG_FCR1_OFFSET:
        retval = s->fcr1;
        break;

    case FM3_UART_REG_FCR0_OFFSET:
        retval = s->fcr0;
        break;

    case FM3_UART_REG_FBYTE2_OFFSET:
        retval = (s->fcr1 & FM3_UART_REG_FCR1_FSEL)?
                    s->tx_fifo->count : s->rx_fifo->count;
        break;

    case FM3_UART_REG_FBYTE1_OFFSET:
        retval = (s->fcr1 & FM3_UART_REG_FCR1_FSEL)?
                    s->rx_fifo->count : s->tx_fifo->count;
        break;

    default:
        printf("FM3_UART: Unknown regster (offset: 0x%x)", offset);
        goto out;
    }
    fm3_uart_update_irq(s);
out:
    DPRINTF("%s [%d]: 0x%08x ---> 0x%08x\n", __func__, ch, offset, retval);
    return retval;
}

static void fm3_uart_write(void *opaque, hwaddr offset,
                           uint64_t value, unsigned size)
{
    int ch = get_ch_no(offset);
    Fm3UartChState *s = &((Fm3UartState *)opaque)->ch[ch];
    Fm3UartFifo *tx_fifo = fm3_uart_get_online_tx_fifo(s);
    uint8_t data = value;
    uint32_t max_fifo = fm3_uart_get_max_fifo(s->ch_no);
    uint32_t mode = 0;

    DPRINTF("%s[%d]: 0x%08x <--- 0x%08x\n", __func__, ch, offset, value);
    offset &= 0xff;
    value &= 0xff;
    switch (offset) {
    case FM3_UART_REG_SCR_OFFSET:
        if (value & FM3_UART_REG_SCR_UPCL) {
            fm3_uart_clear_fifo(&s->fifo1, s->fifo1.trigger);
            fm3_uart_clear_fifo(&s->fifo2, s->fifo2.trigger);
        }
        s->scr = value & ~FM3_UART_REG_SCR_UPCL;
        break;

    case FM3_UART_REG_SMR_OFFSET:
        mode = get_smr_mode(value);
        if ((FM3_UART_MODE_NORMAL != mode) &&
                (FM3_UART_MODE_MULTI != mode)) {
            printf("FM3_UART: Invalid mode (MD2-0 = %d)", mode);
        }
        if (value & FM3_UART_REG_SMR_WUCR)
            unsupported("SMR WUCR");
        if (value & FM3_UART_REG_SMR_SBL)
            unsupported("SMR SBL");
        if (value & FM3_UART_REG_SMR_BDS)
            unsupported("SMR BDS");
        s->smr = value;
        break;

    case FM3_UART_REG_SSR_OFFSET:
        if (value & FM3_UART_REG_SSR_REC) {
            s->ssr &= ~(FM3_UART_REG_SSR_PE |
                        FM3_UART_REG_SSR_FRE |
                        FM3_UART_REG_SSR_ORE);
        }
        break;

    case FM3_UART_REG_ESCR_OFFSET:
        if (value & FM3_UART_REG_ESCR_FLWEN)
            unsupported("ESCR FLWEN");
        if (value & FM3_UART_REG_ESCR_ESBL)
            unsupported("ESCR FLWEN");
        if (value & FM3_UART_REG_ESCR_INV)
            unsupported("ESCR INV");
        if (value & FM3_UART_REG_ESCR_PEN)
            unsupported("ESCR PEN");
        if (value & FM3_UART_REG_ESCR_P)
            unsupported("ESCR P");
        if (get_escr_len(value))
            unsupported("ESCR L2-0");
        s->escr = value;
        break;
    case FM3_UART_REG_TDR_OFFSET:
        if (s->scr & FM3_UART_REG_SCR_TXE) {
            if (tx_fifo) {
                if (tx_fifo->count < max_fifo) {
                    tx_fifo->data[tx_fifo->put++] = data;
                    if (max_fifo <= tx_fifo->put) {
                        tx_fifo->put = 0;
                    }
                    tx_fifo->count++;
                    if (tx_fifo->count < max_fifo) {
                        s->fcr1 &= ~FM3_UART_REG_FCR1_FDRQ;
                    } 
                    if (tx_fifo->trigger <= tx_fifo->count) {
                        fm3_uart_send_fifo(s); 
                    }
                }
            } else {
#if 0
                fm3_uart_clear_tx_irq_flags(s);
#endif
                fm3_uart_chr_write(s, &data, 1);
                fm3_uart_set_tx_irq_flags(s);
            }
        }
        break;
    case FM3_UART_REG_TDR_OFFSET+1:
        /* ignore */
        break;

    case FM3_UART_REG_BGR1_OFFSET:
        if (value & FM3_UART_REG_BGR1_EXT)
            unsupported("BGR1 EXT");
        s->bgr1 = value;
        break;

    case FM3_UART_REG_BGR0_OFFSET:
        s->bgr0 = value;
        break;

    case FM3_UART_REG_FCR1_OFFSET:
        if (value & FM3_UART_REG_FCR1_FRIIE)
            unsupported("FCR1 FRIIE");
#if 0
        if ((!(value & FM3_UART_REG_FCR1_FDRQ)) &&
            (s->fcr1 & FM3_UART_REG_FCR1_FDRQ)) {
            value &= ~FM3_UART_REG_FCR1_FDRQ;
        }
#else
        value |= FM3_UART_REG_FCR1_FDRQ;
#endif
        if (value & FM3_UART_REG_FCR1_FSEL) {
            s->tx_fifo = &s->fifo2;
            s->rx_fifo = &s->fifo1;
        } else {
            s->tx_fifo = &s->fifo1;
            s->rx_fifo = &s->fifo2;
        }
        s->fcr1 = value;
        break;

    case FM3_UART_REG_FCR0_OFFSET:
        value &= ~(FM3_UART_REG_FCR0_FLST |
                   FM3_UART_REG_FCR0_FLD  |
                   FM3_UART_REG_FCR0_FSET |
                   FM3_UART_REG_FCR0_FCL1 |
                   FM3_UART_REG_FCR0_FCL2 );

        if (value & FM3_UART_REG_FCR0_FLD) {
            s->tx_fifo->get = s->tx_fifo->saved_get;
        }

        if (value & FM3_UART_REG_FCR0_FSET) {
            s->tx_fifo->saved_get = s->tx_fifo->get;
        }

        if (value & FM3_UART_REG_FCR0_FCL2) {
            fm3_uart_clear_fifo(&s->fifo2, s->fifo2.trigger);
            if (s->tx_fifo == &s->fifo2)
                s->fcr0 &= ~FM3_UART_REG_FCR0_FLST;
        }

        if (value & FM3_UART_REG_FCR0_FCL1) {
            fm3_uart_clear_fifo(&s->fifo1, s->fifo1.trigger);
            if (s->tx_fifo == &s->fifo1)
                s->fcr0 &= ~FM3_UART_REG_FCR0_FLST;
        }

        if (value & FM3_UART_REG_FCR0_FE2) {
            if ((s->rx_fifo == &s->fifo2) && is_error(s)) {
                value &= ~FM3_UART_REG_FCR0_FE2;
            } else if ((s->tx_fifo == &s->fifo2) && s->tx_fifo->count &&
                 (s->scr & FM3_UART_REG_SCR_TXE)) {
                    fm3_uart_send_fifo(s);
            }
        }

        if (value & FM3_UART_REG_FCR0_FE1) {
            if ((s->rx_fifo == &s->fifo1) && is_error(s)) {
                value &= ~FM3_UART_REG_FCR0_FE1;
            } else if ((s->tx_fifo == &s->fifo1) && s->tx_fifo->count &&
                 (s->scr & FM3_UART_REG_SCR_TXE)) {
                    fm3_uart_send_fifo(s);
            }
        }

        s->fcr0 |= value;
        break;
#if 0
    case FM3_UART_REG_FBYTE2_OFFSET:
        if (s->fcr1 & FM3_UART_REG_FCR1_FSEL)
            s->tx_fifo->trigger = value;
        else
            s->rx_fifo->trigger = value;

        fm3_uart_check_trigger(s);
        break;

    case FM3_UART_REG_FBYTE1_OFFSET:
        if (s->fcr1 & FM3_UART_REG_FCR1_FSEL)
            s->rx_fifo->trigger = value;
        else
            s->tx_fifo->trigger = value;

        fm3_uart_check_trigger(s);
        break;
#else
    case FM3_UART_REG_FBYTE2_OFFSET:
        if (s->fcr1 & FM3_UART_REG_FCR1_FSEL)
            s->tx_fifo->trigger = 1;
        else
            s->rx_fifo->trigger = fm3_uart_get_max_trigger(ch, value);
        break;

    case FM3_UART_REG_FBYTE1_OFFSET:
        if (s->fcr1 & FM3_UART_REG_FCR1_FSEL)
            s->rx_fifo->trigger = fm3_uart_get_max_trigger(ch, value);
        else
            s->tx_fifo->trigger = 1;
        break;
#endif
    default:
        printf("FM3_UART: Unknown regster (offset: 0x%x)", offset);
        goto out;
    }
    fm3_uart_update_irq(s);
out:
    return;
}

static const MemoryRegionOps fm3_uart_mem_ops = {
    .read = fm3_uart_read,
    .write = fm3_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int fm3_uart_can_receive(void *opaque)
{
    Fm3UartChState *s = opaque;
    Fm3UartFifo *fifo = fm3_uart_get_online_rx_fifo(s);
    int retval = 0;

    if (fm3_uart_check_port(s->ch_no, 0, s->rx_port) && 
        ((s->scr & FM3_UART_REG_SCR_RXE) != 0)) {
        if (fifo && fm3_uart_ch_has_fifo(s->ch_no)) {
            retval = FM3_UART_FIFO_MAX_LENGTH - fifo->count;
        } else {
            retval = 1;
        }
    }

    return retval;
}

static void fm3_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    Fm3UartChState *s = opaque;
    Fm3UartFifo *fifo = fm3_uart_get_online_rx_fifo(s);
    uint32_t max_fifo = fm3_uart_get_max_fifo(s->ch_no);
    int i;

    if (fifo) {
        for (i = 0; i < size; i++) {
            if (fifo->count < max_fifo) {
                fifo->data[fifo->put++] = buf[i];
                if (max_fifo <= fifo->put) {
                    fifo->put = 0;
                }
                fifo->count++;
                if (fifo->count >= fifo->trigger) {
                    s->ssr |= FM3_UART_REG_SSR_RDRF;
                }
            } else
                break;
        }
    } else {
        s->rx_fifo->data[0] = buf[0];
        s->rx_fifo->count = 1;
        s->ssr |= FM3_UART_REG_SSR_RDRF;
    }

    fm3_uart_update_rx_irq(s);
}

static void fm3_uart_event(void *opaque, int event)
{
}

static void fm3_uart_reset(DeviceState *d)
{
	Fm3UartState *s = FM3_UART(d);
    Fm3UartChState *ch;
    int i;

    for (i = 0; i < 8; i++) {
        ch = &s->ch[i];
        ch->scr = 0;
        ch->smr = 0;
        ch->ssr = (FM3_UART_REG_SSR_TDRE | 
                   FM3_UART_REG_SSR_TBI);
        ch->escr = 0;
        ch->bgr1 = 0;
        ch->bgr0 = 0;
        ch->fcr1 = FM3_UART_REG_FCR1_FDRQ;
        ch->fcr0 = 0;

        ch->irq_rx_level = 0;
        ch->irq_tx_level = 0;
        ch->tx_fifo = &ch->fifo1;
        ch->rx_fifo = &ch->fifo2;
        fm3_uart_clear_fifo(ch->tx_fifo, 1);
        fm3_uart_clear_fifo(ch->rx_fifo, 1);
    }
}

static void fm3_uart_ch_init(Fm3UartChState *ch, 
                             SysBusDevice *dev,
                             uint32_t ch_no)
{
    int pin_no;

    ch->ch_no = ch_no;
    ch->chr = qemu_char_get_next_serial();
    if (ch->chr) {
        qemu_chr_add_handlers(ch->chr, fm3_uart_can_receive, 
                              fm3_uart_receive, fm3_uart_event, 
                              ch);
    } else {
        printf("FM3_UART: could not get chardev\n");
    }

    pin_no = fm3_board_get_uart_rx_pin(ch->ch_no);
    ch->rx_port = fm3_gpio_get_port_from_pin(pin_no, 
                            FM3_PINPACKAGE_LQFP176);
    pin_no = fm3_board_get_uart_tx_pin(ch->ch_no);
    ch->tx_port = fm3_gpio_get_port_from_pin(pin_no, 
                            FM3_PINPACKAGE_LQFP176);

    sysbus_init_irq(dev, &ch->irq_rx); /* Rx */
    sysbus_init_irq(dev, &ch->irq_tx); /* Tx */
}

static int fm3_uart_init(SysBusDevice *dev)
{
	DeviceState		*devs	= DEVICE(dev);
    Fm3UartState	*s		= FM3_UART(devs);

    fm3_uart_ch_init(&s->ch[0], dev, 0);
    fm3_uart_ch_init(&s->ch[3], dev, 3);
    fm3_uart_ch_init(&s->ch[4], dev, 4);

    memory_region_init_io(&s->mmio, OBJECT(s), &fm3_uart_mem_ops, s, 
                          TYPE_FM3_UART, 0x1000);
    sysbus_init_mmio(dev, &s->mmio);

    fm3_uart_state = s;
    return 0;
}

static Property fm3_uart_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void fm3_uart_class_init(ObjectClass *klass, void *data)
{
	DeviceClass			*dc	= DEVICE_CLASS(klass);
	SysBusDeviceClass	*k	= SYS_BUS_DEVICE_CLASS(klass);

	k->init		= fm3_uart_init;			/* 初期化関数を登録		*/
	dc->desc	= TYPE_FM3_UART;			/* ハードウェア名称		*/
	dc->props	= fm3_uart_properties;		/* 内部情報				*/
	dc->reset	= fm3_uart_reset;
}

static const TypeInfo fm3_uart_info = {
	.name			= TYPE_FM3_UART,			/* ハードウェア名称		*/
	.parent			= TYPE_SYS_BUS_DEVICE,		/* 接続バス				*/
	.instance_size	= sizeof(Fm3UartState),	/* インスタンスサイズ	*/
	.class_init		= fm3_uart_class_init,		/* 初期化関数のポインタ	*/
};

static void fm3_register_devices(void)
{
    type_register_static(&fm3_uart_info);
}

type_init(fm3_register_devices)
