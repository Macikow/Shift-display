/*
 * shift_display.c
 *
 *  Created on: 17 mar 2023
 *      Author: kowma
 */

#include "main.h"
#include "font_24x32.h"
#include "shift_display.h"
#include "string.h"
#include "stdio.h"
#include "SSD1309.h"


#define SHIF_DISPAY_MAX_CHAR 32
#define DISPAY_BUFFER_SIZE 512
/** SSD1309_shift_buffer
 * 								     [768]
 * 		[0]	[1]	[2]	[3]... [24]... [32 * 24]
 * [0]	|	--------------    |
 * [1]	|	  1 x  FONT		  |
 * [2]	|   --------------	  |
 * [3]	|	--------------	  |
 *
 */



uint8_t SSD1309_shift_buffer[BIG_FONT_HIGHT/8][BIG_FONT_WIDTH*SHIF_DISPAY_MAX_CHAR];
uint8_t SSD1309_display_buffer[512];

uint8_t character_cnt=0;
uint16_t shift_cnt=0;

void configure_dma(void)
{
	DMA1_Channel6->CCR &= ~DMA_CCR_EN;
	DMA1_Channel6->CMAR = (uint32_t) & (SSD1309_display_buffer);
	DMA1_Channel6->CNDTR = BUFFER_SIZE;
	I2C1->CR1 |= I2C_CR1_PE | I2C_CR1_TXDMAEN | I2C_CR1_TCIE;
	I2C2->CR2     &= ~( I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_AUTOEND );
	I2C1->CR2 |=  (0x3C << 1); //slave address
	I2C1->CR2 |= (255 << I2C_CR2_NBYTES_Pos ) |  I2C_CR2_RELOAD;
}

void shift_display_fill_buffer(const char* text)
{
	configure_dma();
	uint8_t i = 0;
	while (*(text+i) != '\0' && i < SHIF_DISPAY_MAX_CHAR)
	{
		memcpy(&SSD1309_shift_buffer[0][i*24],&font24_32[(*(text+i)-0x20)*FONT_BYTE_LENGHT],24);
		memcpy(&SSD1309_shift_buffer[1][i*24],&font24_32[((*(text+i)-0x20)*FONT_BYTE_LENGHT)+24],24);
		memcpy(&SSD1309_shift_buffer[2][i*24],&font24_32[((*(text+i)-0x20)*FONT_BYTE_LENGHT)+48],24);
		memcpy(&SSD1309_shift_buffer[3][i*24],&font24_32[((*(text+i)-0x20)*FONT_BYTE_LENGHT)+72],24);
		i++;
	}
	character_cnt = i;
}
void copy_shitf_buffer_2_display(uint16_t shift)
{
	memcpy(&SSD1309_display_buffer[0], 	 &SSD1309_shift_buffer[0][shift], 128);
	memcpy(&SSD1309_display_buffer[128], &SSD1309_shift_buffer[1][shift], 128);
	memcpy(&SSD1309_display_buffer[256], &SSD1309_shift_buffer[2][shift], 128);
	memcpy(&SSD1309_display_buffer[384], &SSD1309_shift_buffer[3][shift], 128);
}

void shift_display_handler()
{


	copy_shitf_buffer_2_display(shift_cnt++);
	if( shift_cnt > ((character_cnt-4)*24) )
	{
		shift_cnt =0;
	}
	I2C1->CR2 &= ~(I2C_CR2_NBYTES);
	I2C1->CR2 |= (255 << I2C_CR2_NBYTES_Pos );
	DMA1_Channel6->CCR &= ~DMA_CCR_EN;
	I2C1->CR1 &= ~(I2C_CR1_PE);
	I2C1->TXDR=0x40;
	DMA1_Channel6->CNDTR = BUFFER_SIZE+1;
	I2C1->CR1 |= (I2C_CR1_PE);
	I2C1->CR2 |= I2C_CR2_START;
	I2C1->TXDR=0x40;
	DMA1_Channel6->CCR |= DMA_CCR_EN;
}





