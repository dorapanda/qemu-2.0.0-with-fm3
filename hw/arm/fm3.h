#ifndef FM3_H
#define FM3_H
/*
 * Fujitsu FM3 MCU emulator
 *
 * Copyright (c) 2012 Pylone, Inc.
 * Written by Masashi YOKOTA <yokota@pylone.jp>
 *
 * This code is licensed under the GNU GPL v2.
 */

enum FM3_PINPACKAGE {
    FM3_PINPACKAGE_LQFP176 = 0,
    FM3_PINPACKAGE_LQFP144,
    FM3_PINPACKAGE_MAX,
};

/* External port block No. */
enum FM3_PORT_EPFR {
    FM3_PORT_EPFR00_SYSTEM = 0,
    FM3_PORT_EPFR01_MFT0,
    FM3_PORT_EPFR02_MFT1,
    FM3_PORT_EPFR03_MFT2,
    FM3_PORT_EPFR04_BT0,
    FM3_PORT_EPFR05_BT1,
    FM3_PORT_EPFR06_EINT0,
    FM3_PORT_EPFR07_MFS0,
    FM3_PORT_EPFR08_MFS1,
    FM3_PORT_EPFR09_CAN_ADC_QPRC,
    FM3_PORT_EPFR10_EXBUS0,
    FM3_PORT_EPFR11_EXBUS1,
    FM3_PORT_EPFR12_BT2,
    FM3_PORT_EPFR13_BT3,
    FM3_PORT_EPFR14_QPRC_ETHER,
    FM3_PORT_EPFR15_EINT1,         /* 15 */
};

enum FM3_PORT_TYPE {
    FM3_PORT_TYPE_GPIO = 0,
    FM3_PORT_TYPE_PERIPHERAL,
};

#define FM3_IRQ_NUM         48
#define FM3_MFS_NUM         8
#define FM3_EXTI_NUM        32

#define FM3_PORT_TO_BLOCKNO(port_no)	((port_no >> 4) & 0xf)
#define FM3_PORT_TO_BITPOS(port_no)     (port_no & 0xf)

/* functions checking irq status for IRQxxMON registers */
uint32_t fm3_uart_get_rx_irq_stat(int ch);
uint32_t fm3_uart_get_tx_irq_stat(int ch);
uint32_t fm3_uart_get_stat_irq_stat(int ch);
uint32_t fm3_exti_get_irq_stat(int ch);

void fm3_exti_set_request(int int_ch, int level);
int fm3_gpio_get_port_from_pin(int pin_no, enum FM3_PINPACKAGE pkg);
uint32_t fm3_gpio_get_port_setting(uint32_t port_no);
uint32_t fm3_gpio_get_extport_setting(uint32_t block_no);

#endif
