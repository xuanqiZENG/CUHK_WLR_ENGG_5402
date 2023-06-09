#include "bsp_rc.h"
#include "usart.h"
#include "dma.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	SET_BIT(huart1.Instance->CR3,USART_CR3_DMAR);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	
	__HAL_DMA_DISABLE(&hdma_usart1_rx);
	while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
	{
		//断掉串口保护数据
		__HAL_DMA_DISABLE(&hdma_usart1_rx);
	}
	
	hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
  hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);

  hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);

  hdma_usart1_rx.Instance->NDTR = dma_buf_num;

  SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

  __HAL_DMA_ENABLE(&hdma_usart1_rx);
	
}
