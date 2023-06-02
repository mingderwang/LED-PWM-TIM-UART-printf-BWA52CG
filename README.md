# LED-PWM-TIM-UART-printf-BWA52CG

# example main.c

```
#include "stm32wbaxx.h"
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
void MX_USART1_UART_Init(void);

PUTCHAR_PROTOTYPE
{
    // Send character to USART or other output device
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

int main(void)
{
    HAL_Init();
    MX_USART1_UART_Init();

    // Redirect stdout to USART1
    setbuf(stdout, NULL);

    printf("Hello, world!\r\n");

    while (1) {
        // Your application code here
    }
}

void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
```
