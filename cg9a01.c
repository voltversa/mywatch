/*
 *
 *  Created on: 05/4/2025
 *  Author: MOSTAFA
 */

#include "cg9a01.h"
#include "main.h"
//#include "spi.h"
#include "font.h"

#include "font8x8_basic.h"
#include "font8x16.h"

;extern SPI_HandleTypeDef hspi1;

#define LCD_RST_1 HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_SET)			// LCD_RST = 1 , LCD RESET pin
#define LCD_RST_0 HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_RESET)		// LCD_RST = 0 , LCD RESET pin

#define LCD_CS_1 HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_SET)			// LCD_CS = 1, LCD select pin
#define LCD_CS_0 HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_RESET)			// LCD_CS = 0, LCD select pin

#define LCD_DC_1 HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_SET)			// LCD_DC = 1, LCD Data/Command pin
#define LCD_DC_0 HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_RESET)			// LCD_DC = 0，LCD Data/Command pin

HAL_StatusTypeDef SPI_Write(uint8_t* pbuff, uint16_t size);
HAL_StatusTypeDef SPI_Read(uint8_t* pbuff, uint16_t size);

unsigned char String6_12[];
const unsigned short pic[];
#define LCD_WIDTH 240
#define LCD_HEIGHT 240
#define TEXT_COLOR 0xFFFF // White
#define BG_COLOR    0x0000 // Black


extern const uint8_t font8x16[][16];

//===============================================================
//write parameter

void  Write_Cmd_Data (unsigned char CMDP)
{
    LCD_CS_0;
   	LCD_DC_1;

   	SPI_Write(&CMDP, 1);

   	LCD_CS_1;
}

//=============================================================
//write command

void Write_Cmd(unsigned char CMD)
{
    LCD_CS_0;
   	LCD_DC_0;

   	SPI_Write(&CMD, 1);

   	LCD_CS_1;
}

//==============================================================
//write  data word

void  Write_Data_U16(unsigned int y)
{
	unsigned char m,n;
	m=y>>8;
	n=y;
	Write_Data(m,n);
}


//===================================================================
//write data byte

void Write_Data(unsigned char DH,unsigned char DL)
{
    LCD_CS_0;
   	LCD_DC_1;

   	SPI_Write(&DH, 1);
   	SPI_Write(&DL, 1);

   	LCD_CS_1;
}
void Write_Data2(uint8_t data)
{
    LCD_DC_1;      // Data mode
    LCD_CS_0;
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
    LCD_CS_1;
}


void Write_Bytes(unsigned char * pbuff, unsigned short size)
{
    LCD_CS_0;
   	LCD_DC_1;

   	SPI_Write(pbuff, size);

   	LCD_CS_1;
}


HAL_StatusTypeDef SPI_Write(uint8_t* pbuff, uint16_t size)
{
	//DMA, use HAL_SPI_Transmit_DMA() function
    HAL_StatusTypeDef status =  HAL_SPI_Transmit_DMA(&hspi1, pbuff, size);
    while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){;}
    return status;

    //no DMA, use HAL_SPI_Transmit() function
    //return HAL_SPI_Transmit(&hspi1, pbuff, size, 100);
}

HAL_StatusTypeDef SPI_Read(uint8_t* pbuff, uint16_t size)
{
	//DMA, use HAL_SPI_Receive_DMA() function
   HAL_StatusTypeDef status = HAL_SPI_Receive_DMA(&hspi1, pbuff, size);
   while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){;}
   return status;

   //no DMA, use HAL_SPI_Receive() function
   //return HAL_SPI_Receive(&hspi1, pbuff, size, 100);
}


//=============================================================
//LCD Initial

void GC9A01_Initial(void)
{
  	LCD_CS_1;
	HAL_Delay(5);
	LCD_RST_0;
	HAL_Delay(10);
	LCD_RST_1;
	HAL_Delay(120);


 //************* Start Initial Sequence **********//
	Write_Cmd(0xEF);

	Write_Cmd(0xEB);
	Write_Cmd_Data(0x14);

    Write_Cmd(0xFE);
	Write_Cmd(0xEF);

	Write_Cmd(0xEB);
	Write_Cmd_Data(0x14);

	Write_Cmd(0x84);
	Write_Cmd_Data(0x40);

	Write_Cmd(0x85);
	Write_Cmd_Data(0xFF);

	Write_Cmd(0x86);
	Write_Cmd_Data(0xFF);

	Write_Cmd(0x87);
	Write_Cmd_Data(0xFF);

	Write_Cmd(0x88);
	Write_Cmd_Data(0x0A);

	Write_Cmd(0x89);
	Write_Cmd_Data(0x21);

	Write_Cmd(0x8A);
	Write_Cmd_Data(0x00);

	Write_Cmd(0x8B);
	Write_Cmd_Data(0x80);

	Write_Cmd(0x8C);
	Write_Cmd_Data(0x01);

	Write_Cmd(0x8D);
	Write_Cmd_Data(0x01);

	Write_Cmd(0x8E);
	Write_Cmd_Data(0xFF);

	Write_Cmd(0x8F);
	Write_Cmd_Data(0xFF);


	Write_Cmd(0xB6);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x00);

	Write_Cmd(0x36);

	if(USE_HORIZONTAL==0)Write_Cmd_Data(0x18);
	else if(USE_HORIZONTAL==1)Write_Cmd_Data(0x28);
	else if(USE_HORIZONTAL==2)Write_Cmd_Data(0x48);
	else Write_Cmd_Data(0x88);



	Write_Cmd(0x3A);
	Write_Cmd_Data(0x05);


	Write_Cmd(0x90);
	Write_Cmd_Data(0x08);
	Write_Cmd_Data(0x08);
	Write_Cmd_Data(0x08);
	Write_Cmd_Data(0x08);

	Write_Cmd(0xBD);
	Write_Cmd_Data(0x06);

	Write_Cmd(0xBC);
	Write_Cmd_Data(0x00);

	Write_Cmd(0xFF);
	Write_Cmd_Data(0x60);
	Write_Cmd_Data(0x01);
	Write_Cmd_Data(0x04);

	Write_Cmd(0xC3);
	Write_Cmd_Data(0x13);
	Write_Cmd(0xC4);
	Write_Cmd_Data(0x13);

	Write_Cmd(0xC9);
	Write_Cmd_Data(0x22);

	Write_Cmd(0xBE);
	Write_Cmd_Data(0x11);

	Write_Cmd(0xE1);
	Write_Cmd_Data(0x10);
	Write_Cmd_Data(0x0E);

	Write_Cmd(0xDF);
	Write_Cmd_Data(0x21);
	Write_Cmd_Data(0x0c);
	Write_Cmd_Data(0x02);

	Write_Cmd(0xF0);
    Write_Cmd_Data(0x45);
    Write_Cmd_Data(0x09);
 	Write_Cmd_Data(0x08);
  	Write_Cmd_Data(0x08);
 	Write_Cmd_Data(0x26);
 	Write_Cmd_Data(0x2A);

 	Write_Cmd(0xF1);
 	Write_Cmd_Data(0x43);
 	Write_Cmd_Data(0x70);
 	Write_Cmd_Data(0x72);
 	Write_Cmd_Data(0x36);
 	Write_Cmd_Data(0x37);
 	Write_Cmd_Data(0x6F);


 	Write_Cmd(0xF2);
 	Write_Cmd_Data(0x45);
 	Write_Cmd_Data(0x09);
 	Write_Cmd_Data(0x08);
 	Write_Cmd_Data(0x08);
 	Write_Cmd_Data(0x26);
 	Write_Cmd_Data(0x2A);

 	Write_Cmd(0xF3);
 	Write_Cmd_Data(0x43);
 	Write_Cmd_Data(0x70);
 	Write_Cmd_Data(0x72);
 	Write_Cmd_Data(0x36);
 	Write_Cmd_Data(0x37);
 	Write_Cmd_Data(0x6F);

	Write_Cmd(0xED);
	Write_Cmd_Data(0x1B);
	Write_Cmd_Data(0x0B);

	Write_Cmd(0xAE);
	Write_Cmd_Data(0x77);

	Write_Cmd(0xCD);
	Write_Cmd_Data(0x63);


	Write_Cmd(0x70);
	Write_Cmd_Data(0x07);
	Write_Cmd_Data(0x07);
	Write_Cmd_Data(0x04);
	Write_Cmd_Data(0x0E);
	Write_Cmd_Data(0x0F);
	Write_Cmd_Data(0x09);
	Write_Cmd_Data(0x07);
	Write_Cmd_Data(0x08);
	Write_Cmd_Data(0x03);

	Write_Cmd(0xE8);
	Write_Cmd_Data(0x34);

	Write_Cmd(0x62);
	Write_Cmd_Data(0x18);
	Write_Cmd_Data(0x0D);
	Write_Cmd_Data(0x71);
	Write_Cmd_Data(0xED);
	Write_Cmd_Data(0x70);
	Write_Cmd_Data(0x70);
	Write_Cmd_Data(0x18);
	Write_Cmd_Data(0x0F);
	Write_Cmd_Data(0x71);
	Write_Cmd_Data(0xEF);
	Write_Cmd_Data(0x70);
	Write_Cmd_Data(0x70);

	Write_Cmd(0x63);
	Write_Cmd_Data(0x18);
	Write_Cmd_Data(0x11);
	Write_Cmd_Data(0x71);
	Write_Cmd_Data(0xF1);
	Write_Cmd_Data(0x70);
	Write_Cmd_Data(0x70);
	Write_Cmd_Data(0x18);
	Write_Cmd_Data(0x13);
	Write_Cmd_Data(0x71);
	Write_Cmd_Data(0xF3);
	Write_Cmd_Data(0x70);
	Write_Cmd_Data(0x70);

	Write_Cmd(0x64);
	Write_Cmd_Data(0x28);
	Write_Cmd_Data(0x29);
	Write_Cmd_Data(0xF1);
	Write_Cmd_Data(0x01);
	Write_Cmd_Data(0xF1);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x07);

	Write_Cmd(0x66);
	Write_Cmd_Data(0x3C);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0xCD);
	Write_Cmd_Data(0x67);
	Write_Cmd_Data(0x45);
	Write_Cmd_Data(0x45);
	Write_Cmd_Data(0x10);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x00);

	Write_Cmd(0x67);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x3C);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x01);
	Write_Cmd_Data(0x54);
	Write_Cmd_Data(0x10);
	Write_Cmd_Data(0x32);
	Write_Cmd_Data(0x98);

	Write_Cmd(0x74);
	Write_Cmd_Data(0x10);
	Write_Cmd_Data(0x85);
	Write_Cmd_Data(0x80);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x00);
	Write_Cmd_Data(0x4E);
	Write_Cmd_Data(0x00);

    Write_Cmd(0x98);
	Write_Cmd_Data(0x3e);
	Write_Cmd_Data(0x07);

	Write_Cmd(0x35);
	Write_Cmd(0x21);

	Write_Cmd(0x11);
	HAL_Delay(120);
	Write_Cmd(0x29);
	HAL_Delay(20);
}


//===============================================================
//clear screen
//point to point clear , slowly
void ClearScreen(unsigned int bColor)
{
 unsigned int i,j;

 LCD_SetPos(0,0,GC9A01_TFTWIDTH-1,GC9A01_TFTHEIGHT-1);//240x240

 for (i=0;i<GC9A01_TFTWIDTH;i++)
 {
	 for (j=0;j<GC9A01_TFTHEIGHT;j++)
		 Write_Data_U16(bColor);
 }

}

//===============================================================
//clear screen2
//Use DMA，faster
void ClearScreen2(unsigned int bColor)
{
	ClearWindow(0,0,GC9A01_TFTWIDTH,GC9A01_TFTHEIGHT,bColor);
}

void ClearWindow(unsigned int startX, unsigned int startY, unsigned int endX, unsigned int endY, unsigned int bColor)
{
 unsigned int i;

 //Exchange high 8bit and low 8bit of bColor for DMA batch transmit
 unsigned char hb = (bColor&0xFFFF) >> 8;
 unsigned char lb = bColor & 0xFF;
 unsigned short tempColor = lb * 256 + hb;

 unsigned int totalSize = (endX-startX) * (endY - startY) * 2; // total clear window data size
 unsigned int bufSize = 512;  // define bufSize, need less than DMA transmit size

 unsigned int loopNum = (totalSize - (totalSize % bufSize)) / bufSize; // transmit loop times
 unsigned int modNum = totalSize % bufSize;  // remainder data bytes


 //use a tempBuf to initial bColor data, bufSize < DMA transmit size
 unsigned short tempBuf[bufSize];
 unsigned char * ptempBuf;

 //init tempBuf data to tempColor( Exchange high 8bit and low 8bit of bColor )
 for(i=0; i<bufSize; i++){
	 tempBuf[i] = tempColor;
 }

 // Clear window size: from (startX, startY) to (endX, endY)
 LCD_SetPos(startX,startY,endX-1,endY-1);// (endX-startX) * (endY - startY)

 // transmit bufSize byte one time, loopNum loops
 ptempBuf = (unsigned char *)tempBuf;
 for(i=0; i<loopNum; i++){
	 Write_Bytes(ptempBuf, bufSize);
 }

 // transmit remainder data, modNum bytes
 Write_Bytes(ptempBuf, modNum);

}


//===============================================================
 void LCD_SetPos(unsigned int Xstart,unsigned int Ystart,unsigned int Xend,unsigned int Yend)
{
	Write_Cmd(0x2a);
	Write_Cmd_Data(Xstart>>8);
	Write_Cmd_Data(Xstart);
 	Write_Cmd_Data(Xend>>8);
	Write_Cmd_Data(Xend);

	Write_Cmd(0x2b);
	Write_Cmd_Data(Ystart>>8);
	Write_Cmd_Data(Ystart);
	Write_Cmd_Data(Yend>>8);
	Write_Cmd_Data(Yend);

  	Write_Cmd(0x2c);//LCD_WriteCMD(GRAMWR);
}


 //show picture
 //point to point show picture data, slowly
 void show_picture(void)
 {
 	unsigned char i,j;
 	unsigned int n=0;


    LCD_SetPos(20,20,219,219);

 	for(j=0;j<200;j++)
 	{
 		for(i=0;i<200;i++)
 		{
             Write_Data_U16(pic[n++]);
 	    }
 	}

     return;
 }

 void GC9A01_Draw_Char(uint16_t x, uint16_t y, char c)
 {
     if (c < 0 || c > 127) return;

     const uint8_t *bitmap = font8x16[(uint8_t)c];

     LCD_SetPos(x, y, x + 7, y + 15); // 8 wide × 16 high

     for (int row = 0; row < 16; row++) {
         for (int col = 0; col < 8; col++) {
             uint16_t color = (bitmap[row] & (1 << (7 - col))) ? TEXT_COLOR : BG_COLOR;
             Write_Data2(color >> 8);
             Write_Data2(color & 0xFF);
         }
     }
 }

 // Draw a string at (x, y)void GC9A01_Draw_String(uint16_t x, uint16_t y, const char *str)
 void GC9A01_Draw_String(uint16_t x, uint16_t y, const char *str)
 {
     while (*str && x + 8 < LCD_WIDTH) {
         GC9A01_Draw_Char(x, y, *str++);
         x += 8; // still 8 pixels wide
     }
 }

