#include "UserUART.h"
#include <string.h>

#if USER_UART_DEBUG_PORT == 0
#define USER_UART_INST UART_0_INST
#elif USER_UART_DEBUG_PORT == 2
#define USER_UART_INST UART_2_INST
#else
#error "USER_UART_DEBUG_PORT must be 0 or 2"
#endif

static char UserUART_TxQueue[USER_UART_TX_QUEUE_SIZE];
static uint16_t UserUART_TxHead = 0U;
static uint16_t UserUART_TxTail = 0U;
static char UserUART_RxLine[USER_UART_RX_LINE_SIZE];
static uint16_t UserUART_RxLen = 0U;

static uint16_t txQueueUsed(void) {
    if (UserUART_TxHead >= UserUART_TxTail) {
        return UserUART_TxHead - UserUART_TxTail;
    }
    return USER_UART_TX_QUEUE_SIZE - UserUART_TxTail + UserUART_TxHead;
}

int UART0_sendStr(const char* str) {
    int count = 0;
    while (*str) {
        DL_UART_transmitDataBlocking(UART_0_INST, (uint8_t)*str);
        str++;
        count++;
    }
    return count;
}

int UART2_sendStr(const char* str) {
    int count = 0;
    while (*str) {
        DL_UART_transmitDataBlocking(UART_2_INST, (uint8_t)*str);
        str++;
        count++;
    }
    return count;
}

void UART0_printf(const char* fmt, ...) {
    static char buf[UART_TX_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    UART0_sendStr(buf);
}

void UART2_printf(const char* fmt, ...) {
    static char buf[UART_TX_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    UART2_sendStr(buf);
}

void UserUART_task(void) {
    while ((UserUART_TxTail != UserUART_TxHead) &&
           !DL_UART_Main_isTXFIFOFull(USER_UART_INST)) {
        DL_UART_Main_transmitData(USER_UART_INST,
                                  (uint8_t)UserUART_TxQueue[UserUART_TxTail]);
        UserUART_TxTail++;
        if (UserUART_TxTail >= USER_UART_TX_QUEUE_SIZE) UserUART_TxTail = 0U;
    }
}

int UserUART_write(const char* str) {
    size_t len;
    size_t i;
    uint16_t freeSpace;

    if (str == NULL) return -1;
    len = strlen(str);
    if (len >= USER_UART_TX_QUEUE_SIZE) return -1;

    freeSpace = (uint16_t)(USER_UART_TX_QUEUE_SIZE - 1U - txQueueUsed());
    if (len > freeSpace) return -1;

    for (i = 0U; i < len; i++) {
        UserUART_TxQueue[UserUART_TxHead] = str[i];
        UserUART_TxHead++;
        if (UserUART_TxHead >= USER_UART_TX_QUEUE_SIZE) UserUART_TxHead = 0U;
    }
    return (int)len;
}

int UserUART_printf(const char* fmt, ...) {
    char buf[UART_TX_BUF_SIZE];
    int len;
    va_list args;

    if (fmt == NULL) return -1;
    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0) return -1;
    if ((size_t)len >= sizeof(buf)) {
        buf[sizeof(buf) - 1U] = '\0';
    }
    return UserUART_write(buf);
}

bool UserUART_readLine(char* dst, size_t dstSize) {
    uint8_t ch;

    if ((dst == NULL) || (dstSize == 0U)) return false;

    while (DL_UART_Main_receiveDataCheck(USER_UART_INST, &ch)) {
        if (ch == '\r') continue;
        if (ch == '\n') {
            if (UserUART_RxLen == 0U) continue;
            UserUART_RxLine[UserUART_RxLen] = '\0';
            if ((size_t)UserUART_RxLen >= dstSize) {
                UserUART_RxLen = 0U;
                return false;
            }
            memcpy(dst, UserUART_RxLine, (size_t)UserUART_RxLen + 1U);
            UserUART_RxLen = 0U;
            return true;
        }

        if (UserUART_RxLen < (USER_UART_RX_LINE_SIZE - 1U)) {
            UserUART_RxLine[UserUART_RxLen++] = (char)ch;
        }
        else {
            UserUART_RxLen = 0U;
        }
    }
    return false;
}
