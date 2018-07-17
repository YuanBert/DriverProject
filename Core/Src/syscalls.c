#include "stdio.h"
#include "string.h"
#include "stm32f1xx_hal.h"

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0x000f);
	return ch;
}
int _write(int file,char *ptr, int len)
{
	int DataIdx;
	for(DataIdx = 0; DataIdx < len;DataIdx++)
	{
		__io_putchar(*ptr++);
	}
	return len;
}



