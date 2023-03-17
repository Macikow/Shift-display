/*
 * SSD1309.c
 *
 *  Created on: 10 mar 2023
 *      Author: kowma
 */

#include "main.h"
#include "SSD1309.h"
#include <stdio.h>
#include <string.h>


#define TABLE_SIZE 19

#define DISPALY_LENGTH (128/8)
#define DISPLAY_HIGTH 64





uint8_t SSD1309_tz_buff[BUFFER_SIZE];

const uint8_t table[TABLE_SIZE][4]={
					{1, 0x00, 0x00, 0x00},
					{1, SSD1306_DISPLAY_OFF,0,0},  								//[0] 0xAE  Set Display OFF
					{2, SSD1306_SET_MUX_RATIO,63,0},							//[1] 0xA8 	64MUX for 128 x 64
																				//			32MUX for 128 x 32
					{2, SSD1306_MEMORY_ADDR_MODE,0x00,0x00},					//[2] 0x20	0x00 - horisontal addressing mode
																				//			0x01 - Vertical adressing mode
																				// 			0x02 - Page addressing mode (RESET)

					{3, SSD1306_COLUMNADDR,START_COLUMN_ADDR,END_COLUMN_ADDR},	//[3] 	0x21 Set Column Address
					{3, SSD1306_PAGEADDR,START_PAGE_ADDR,END_PAGE_ADDR},		//[4]	0x22 Set Page Address


					{1, SSD1306_SETSTARTLINE,0x00,0x00},						//[5] 0x40
					{2, SSD1306_DISPLAY_OFFSET,0x00,0x00},						//[6] 0xD3
					{1, SSD1306_SEG_REMAP_OP,0x00,0x00},						//[7] 0xA0
					{1, SSD1306_COM_SCAN_DIR_OP,0x00,0x00},						//[8] 0xC0
					{2, SSD1306_COM_PIN_CONF,0x12,0x00},						//[9] 0xDA, 0x12 - Disable COM Left/Right Remap, Alterantive VOM pin configuration
																				//			0x12 - for 128 x 64 version
																				// 			0x02 - for 128 x 32 version

					{2, SSD1306_SETCONTRAST,0xFF,0},							//[10] 0x81 , 0x7F - reset value , max 0xff
					{1, SSD1306_DISPLAYALLON_RESUME,0x00,0},					//[11] 0xA4
					{1, SSD1306_NORMALDISPLAY,0x00,0},							//[12] 0xA6
					{2, SSD1306_SET_OSC_FREQ,0x80,0},							//[13] 0xD5
					{2, SSD1306_SET_PRECHARGE,0xc2,0},							//[14] 0xD9
					{2, SSD1306_VCOM_DESELECT,0x20,0},											//[15] 0xDB
					{2, SSD1306_CHARGEPUMP,0x14,0}, 							//[16] 0x8D Enable charge pump
					{1, SSD1306_DISPLAYON,0,0}, 								//[17] 0xAF Set Display on
					};

void SSD1309_configure_reg_for_init(uint8_t data_amout)
{
	DMA1_Channel6->CCR &= ~(DMA_CCR_EN); // turn off DMA for register values changes
	DMA1_Channel6->CMAR = (uint32_t) & (SSD1309_tz_buff);
	DMA1_Channel6->CNDTR = (uint32_t) data_amout;
	// I2C configuration
	I2C1->CR1 |= I2C_CR1_PE | I2C_CR1_TXDMAEN | I2C_CR1_TCIE;
	I2C2->CR2     &= ~( I2C_CR2_SADD | I2C_CR2_NBYTES );
	I2C1->CR2 |=  (0x3C << 1); //slave address
	I2C1->CR2 |=  I2C_CR2_AUTOEND;
	DMA1_Channel6->CCR |= (DMA_CCR_EN); // turn on DMA for register values changes
}

void SSD1309_configure_reg_for_stream(void)
{
	DMA1_Channel6->CCR &= ~(DMA_CCR_EN); // turn of DMA for register values changes
	DMA1_Channel6->CMAR = (uint32_t) & (SSD1309_tz_buff);
	DMA1_Channel6->CNDTR = BUFFER_SIZE;
	//I2C interface configuration
	I2C1->CR1 |= I2C_CR1_PE | I2C_CR1_TXDMAEN | I2C_CR1_TCIE;
	I2C2->CR2     &= ~( I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_AUTOEND );
	I2C1->CR2 |=  (0x3C << 1); //slave address
	I2C1->CR2 |= (255 << I2C_CR2_NBYTES_Pos ) |  I2C_CR2_RELOAD;
 }



//24x32

//1- [0][2][3] ... 	 	[23]
//2- [24][25][26] ...	[47]
//3- [48][49][50] ...	[71]
//4- [72][73][74] ...	[95]
void test_put_char()
{
//	memcpy (&SSD1309_tz_buff[0], &transformed_font_B[0], 24);
//	memcpy (&SSD1309_tz_buff[128], &transformed_font_B[24], 24);
//	memcpy (&SSD1309_tz_buff[2*128], &transformed_font_B[2*24], 24);
//	memcpy (&SSD1309_tz_buff[3*128], &transformed_font_B[3*24], 24);
}

void SSD1309_init(void)
{
	uint8_t init_stream_inc=0;

	for(uint8_t table_row_inc=0; table_row_inc < TABLE_SIZE; table_row_inc++ )
	{
		for(uint8_t tab_col_inc = 1; tab_col_inc <= table[table_row_inc][0]; tab_col_inc++ )
		{
			SSD1309_tz_buff[init_stream_inc++] = table[table_row_inc][tab_col_inc];
		}
	}
	I2C1->CR2 &= ~ (255 << I2C_CR2_NBYTES_Pos );
	I2C1->CR2 |= (init_stream_inc) << I2C_CR2_NBYTES_Pos;
	SSD1309_configure_reg_for_init(init_stream_inc+1);
	I2C1->CR2 |= I2C_CR2_START;

	while(!(I2C1->ISR & I2C_ISR_STOPF))
	{

	}
	SSD1309_configure_reg_for_stream();
	NVIC_SetPriority(I2C1_EV_IRQn, 0x03);
	NVIC_EnableIRQ( I2C1_ER_IRQn );

//	for(uint16_t i = 0; i< BUFFER_SIZE ; i++)
//	{
//		SSD1309_tz_buff[i] = 0x00;
//	}
	//test_put_char();

	//I2C1->CR2 |= I2C_CR2_START;
	//I2C1->TXDR=0x40;
	//DMA1_Channel6->CCR |= DMA_CCR_EN;

}
