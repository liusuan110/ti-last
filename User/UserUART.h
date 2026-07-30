#ifndef __USER_UART_H__
#define __USER_UART_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "ti_msp_dl_config.h"

#define UART_TX_BUF_SIZE 256
#define USER_UART_TX_QUEUE_SIZE 512
#define USER_UART_RX_LINE_SIZE 64

/*
 * 调试串口选择：0 = UART0(PA10 TX / PA11 RX)，2 = UART2(PB17 TX / PB18 RX)。
 * 只在确认USB-TTL接线后修改此宏。
 */
#ifndef USER_UART_DEBUG_PORT
#define USER_UART_DEBUG_PORT 0
#endif

int UART0_sendStr(const char* str);
int UART2_sendStr(const char* str);
void UART0_printf(const char* fmt, ...);
void UART2_printf(const char* fmt, ...);

void UserUART_task(void);
int UserUART_write(const char* str);
int UserUART_printf(const char* fmt, ...);
bool UserUART_readLine(char* dst, size_t dstSize);

#endif /* #ifndef __USER_UART_H__ */
