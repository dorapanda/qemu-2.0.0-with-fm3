/*
 * Fujitsu FM3 MCU emulator
 *
 * Copyright (c) 2012 Pylone, Inc.
 * Written by Masashi YOKOTA <yokota@pylone.jp>
 *
 * This code is licensed under the GNU GPL v2.
 */

#include "qemu-common.h"
#include "fm3.h"
#include "fm3_board_config.h"


int fm3_board_get_uart_pin(int uart_ch, int tx)
{
    static const int pin_no[FM3_MFS_NUM][2] = {
    /*  { rx,  tx}  */
        {126, 125}, /* ch0(SIN0_0, SOT0_0) */
        { -1,  -1},
        { -1,  -1},
        { 58,  59}, /* ch3(SIN3_2, SOT3_2) */
        {  8,   9}, /* ch4(SIN4_2, SOT4_2) */
        { -1,  -1}, 
        { -1,  -1},
        { -1,  -1}, 
    };

    if (7 < uart_ch)
        return -1;

    return pin_no[uart_ch][(tx != 0)];
}

int fm3_board_get_exti_pin(int exti_no)
{
    static const int pin_no[FM3_EXTI_NUM] = {
        [0 ... 11] = -1,
        [12] = 78, /* INT12_0 */
        [13] = 81, /* INT13_0 */
        [14] = 82, /* INT14_0 */
        [15] = 83, /* INT15_0 */
        [16 ... 31] = -1,
    };

    if (31 < exti_no)
        return -1;

    return pin_no[exti_no];
}

int fm3_board_port_to_uart(int port_no)
{
    switch (port_no) {
    case 0x05: /* SIN4_2 */
    case 0x06: /* SOT4_2 */
        return 4;
    case 0x21: /* SIN0_0 */
    case 0x22: /* SOT0_0 */
        return 0;
    case 0x48: /* SIN3_2 */
    case 0x49: /* SOT3_2 */
        return 3;
    default:
        return -1;
    }
}

int fm3_board_port_to_extint(int port_no)
{
    switch (port_no) {
    case 0x7D: /* INT12_0 */
        return 12;
    case 0xF0: /* INT13_0 */
        return 13;
    case 0xF1: /* INT14_0 */ 
        return 14;
    case 0xF2: /* INT15_0 */
        return 15;
    default:
        return -1;
    }
}

int fm3_board_get_port_info(int port_no)
{
    switch (port_no) {
    case 0x05: /* SIN4_2 */
    case 0x06: /* SOT4_2 */
    case 0x21: /* SIN0_0 */
    case 0x22: /* SOT0_0 */
    case 0x48: /* SIN3_2 */
    case 0x49: /* SOT3_2 */
    case 0x7D: /* INT12_0 */
    case 0xF0: /* INT13_0 */
    case 0xF1: /* INT14_0 */ 
    case 0xF2: /* INT15_0 */
        return FM3_PORT_TYPE_PERIPHERAL;

    default:
        return FM3_PORT_TYPE_GPIO;
    }
}

#define fm3_board_extport_cmp(val1, val2, mask) \
            (((val1) & mask) == ((val2) & mask))

bool fm3_board_check_extport_exti(int ch, int setting)
{
    int ex_index = ch & 0xf;
    int bits, i;
    int mask = 0, val = 0;

    static const int ex_port_exti_bits[][2] = {
        [0 ... 11] = { -1},
        [12] = { 0, 1 },       /* INT12_0 */
        [13] = { 0, 1 },       /* INT13_0 */ 
        [14] = { 0, 1 },       /* INT14_0 */ 
        [15] = { 0, 1 },       /* INT15_0 */
        [16 ... 31] = { -1},
    };
    if (31 < ch)
        return false;

    ex_index <<= 1;
    mask = 3 << ex_index;
    for (i = 0; i < 2; i++) { 
        bits = ex_port_exti_bits[ch][i];
        if (bits == -1)
            return false;

        val = bits << ex_index;
        if (fm3_board_extport_cmp(setting, val, mask))
            return true;
    }
    return false;
}

bool fm3_board_check_extport_uart(int ch, int tx, int setting)
{
    if (7 < ch)
        return false;

    int tmp;
    if (tx) {
        /* SOT */
        switch (ch) {
        case 0:         /* SOT0_0 */
            if (((setting >> 6) & 0x3) == 1)
                return true;
            break;
        case 3:         /* SOT3_2 */
            if (((setting >> 24) & 0x3) == 3)
                return true;
            break;
        case 4:         /* SOT4_2 */
            if (((setting >> 6) & 0x3) == 3)
                return true;
            break;
        }
    } else {
        /* SIN */
        switch (ch) {
        case 0:         /* SIN0_0 */
            tmp = ((setting >> 4) & 0x3); 
            if (tmp == 0 || tmp == 1)
                return true;
            break;
        case 3:         /* SIN3_2 */
            if (((setting >> 22) & 0x3) == 3)
                return true;
            break;
        case 4:         /* SIN4_2 */
            if (((setting >> 4) & 0x3) == 3)
                return true;
            break;
        }
    }

    return false;
}
