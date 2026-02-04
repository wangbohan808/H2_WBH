#include "hal_uart.h"
#include "cw32l010.h"
#include "cw32l010_uart.h"

/* ===== 重写逐字符处理、字符批量处理的函数 ===== */

int fputc(int ch, FILE *f)
{
    /* 将字符转为 8 位无符号整数，发送一个字节到 UART */
    UART_SendData_8bit(CW_UART1, (uint8_t)ch);
    /* UART_FLAG_TXE是发送缓冲器的数据标志位 */
    while (UART_GetFlagStatus(CW_UART1, UART_FLAG_TXE) == RESET);
    return ch;
}

size_t __write(int handle, const unsigned char * buffer, size_t size)
{
    size_t nChars = 0;

    if (buffer == 0)
    {
        /*
         * This means that we should flush internal buffers.  Since we
         * don't we just return.  (Remember, "handle" == -1 means that all
         * handles should be flushed.)
         */
        return 0;
    }


    for (/* Empty */; size != 0; --size)
    {
        UART_SendData_8bit(CW_UART1, *buffer++);
        while (UART_GetFlagStatus(CW_UART1, UART_FLAG_TXE) == RESET);
        ++nChars;
    }

    return nChars;
}
