//TODO:
//Measure nominal current with current code
//Set MEM_SPI cs pins to high starting condition
//Measure current (verify no shorts)
//Finish sram test
//Do mem test

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_usart.h"
#include <string.h>

extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern UART_HandleTypeDef huart1;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void uart_tx(char*msg){
	HAL_UART_Transmit(&huart3, msg, strlen(msg), 1000);
}
char rx_in;
int rx_bytes = 0;

int main(int argc, char* argv[])
{

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); //Set MS5611 cs high
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); //Set Mag cs high
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); //Set A/G cs high
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); //Set H3LI cs high
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); //Set SRAM cs high
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);//Set Flash cs high

  HAL_Delay(1000);

  HAL_UART_Receive_IT(&huart3, rx_in, 1);

  while (1)
    {
	  char update[100];
	  sprintf(update, "Rx bytes = %d\n", rx_bytes);
	  uart_tx(update);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
      //HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 0xFFFF);
      HAL_Delay(500);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      //HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 0xFFFF);
      HAL_Delay(1000);

          //HAL_UART_Transmit(&huart3, (uint8_t*)xmsg, strlen(xmsg), 0xFFFF);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uart_tx(rx_in);
	rx_bytes++;
	HAL_UART_Receive_IT(&huart3, rx_in, 1);

}

#pragma GCC diagnostic pop


