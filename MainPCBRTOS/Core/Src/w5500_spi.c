/*
 * w5500_spi.c
 *
 *  Created on: 30 Jun 2022
 *      Author: Q6Q
 */

/*includes*/
#include "stm32f4xx_hal.h"
#include "wizchip_conf.h"
#include "stdio.h"

/*Type Defines*/
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

/*Defines*/
#define TRUE 1
#define FALSE 0

uint8_t wizchip1_status = FALSE;
uint8_t wizchip2_status = FALSE;

/*Functions*/
void wizchip1_select(void){
	wizchip1_status = TRUE;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //Makes CS pin LOW
}

void wizchip1_deselect(void){
	wizchip1_status = FALSE;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //Makes CS pin HIGH
}

void wizchip2_select(void){
	wizchip2_status = TRUE;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //Makes CS pin LOW
}

void wizchip2_deselect(void){
	wizchip2_status = FALSE;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //Makes CS pin HIGH
}

//Pointers has to be added here to differentiate from both received data packages
uint8_t SPIRead(void){
	uint8_t rb = 0;
	if(wizchip1_status == TRUE){
		HAL_SPI_Receive(&hspi1, &rb, 1, HAL_MAX_DELAY);
	}else if(wizchip2_status == TRUE){
		HAL_SPI_Receive(&hspi2, &rb, 1, HAL_MAX_DELAY);
	}
	return rb;
}

//Pointers has to be added here to differentiate from both transmitted data packages
void SPIWrite(uint8_t wb){
	if(wizchip1_status == TRUE){
		HAL_SPI_Transmit(&hspi1, &wb, 1, HAL_MAX_DELAY);
	}else if(wizchip2_status == TRUE){
		HAL_SPI_Transmit(&hspi2, &wb, 1, HAL_MAX_DELAY);
	}
}

uint8_t wizchip_read(void){
	uint8_t readByte;
	readByte = SPIRead();
	return readByte;
}

void wizchip_write(uint8_t writeByte){
	SPIWrite(writeByte);
}

void wizchip_readBurst(uint8_t* pBuf, uint16_t len){
	for(uint16_t i = 0; i < len; i++){
		*pBuf = SPIRead();
		pBuf++;
	}
}

void wizchip_writeBurst(uint8_t *pBuf, uint16_t len){
	for(uint16_t i = 0; i < len; i++){
		SPIWrite(*pBuf);
		pBuf++;
	}
}

void wizchip1_settings(void){
	  wiz_NetInfo netInfo = { .mac 	= {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},	// Mac address
	                          .ip 	= {169, 254, 2, 1},					    // IP address
	                          .sn 	= {255, 255, 0, 0},						// Subnet mask
	                          .gw 	= {169, 254, 1, 1}};					// Gateway address
	  wizchip_setnetinfo(&netInfo);
}

void wizchip2_settings(void){
	  wiz_NetInfo netInfo = { .mac 	= {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},	// Mac address
	                          .ip 	= {10, 16, 6, 189},					    // IP address
	                          .sn 	= {255, 255, 0, 0},					    // Subnet mask
	                          .gw 	= {255, 255, 255, 127}};					// Gateway address
	  wizchip_setnetinfo(&netInfo);

	  /* Set interrupt */
	  setSn_IMR(1, Sn_IR_RECV);
	  setSIMR(0x02);
	  setSIR(0x00);
}

void W5500Init(){
	uint8_t tmp;
	uint8_t memsize[2][8]= {{4, 4, 2, 2, 2, 2, 0, 0},{4, 4, 2, 2, 2, 2, 0, 0}};

	wizchip1_deselect(); //CS HIGH by default
	wizchip2_deselect(); //CS HIGH by default

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	tmp = 0xFF;
	while(tmp--);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

	reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
	reg_wizchip_spiburst_cbfunc(wizchip_readBurst, wizchip_writeBurst);

	/* Initialise first WIZchip on RSCS */
	reg_wizchip_cs_cbfunc(wizchip1_select, wizchip1_deselect);
	if(ctlwizchip(CW_INIT_WIZCHIP, (void*)memsize) == -1){
		while(1); // Initialisation failed!
	}
	/*Initialise MAC, IP, and the rest for wizchip1 here*/
	wizchip1_settings();

	/* Initialise second WIZchip on RSCS */
	reg_wizchip_cs_cbfunc(wizchip2_select, wizchip2_deselect);
	if(ctlwizchip(CW_INIT_WIZCHIP, (void*)memsize) == -1){
		while(1); // Initialisation failed!
	}
	/*Initialise MAC, IP, and the rest for wizchip2 here*/
	wizchip2_settings();
}
