#ifndef FM3_BOARD_CONFIG_H
#define FM3_BOARD_CONFIG_H
/*
 * Fujitsu FM3 MCU emulator
 *
 * Copyright (c) 2012 Pylone, Inc.
 * Written by Masashi YOKOTA <yokota@pylone.jp>
 *
 * This code is licensed under the GNU GPL v2.
 */

int fm3_board_get_uart_pin(int uart_ch, int tx);

#define fm3_board_get_uart_rx_pin(uart_ch) fm3_board_get_uart_pin(uart_ch, 0)
#define fm3_board_get_uart_tx_pin(uart_ch) fm3_board_get_uart_pin(uart_ch, 1)

int fm3_board_get_exti_pin(int exti_no);
int fm3_board_port_to_uart(int port_no);
int fm3_board_port_to_extint(int port_no);
int fm3_board_get_port_info(int port_no);

#define fm3_board_extport_cmp(val1, val2, mask) \
            (((val1) & mask) == ((val2) & mask))

bool fm3_board_check_extport_exti(int ch, int setting);
bool fm3_board_check_extport_uart(int ch, int tx, int setting);

#define FM3_UART_FIFO_MAX_LENGTH    (16)

#endif
