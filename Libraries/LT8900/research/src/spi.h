/*
�����Ȩ���У������а��ſɿƼ����޹�˾		2013��11��27�շ���

�й��������˵� ��2.4Gģ�� ����ԭ��

����֧�����ߣ�4008 555 368  0755-61195776  15323435161  �Թ�

�������ۣ�10K����2.6RMB 50K����2.5RMB

���ԣ�128��Ƶ���ɵ�  �տ�����120-150��  �͹���   

������http://www.ai-thinker.com

��ַ������ ���� ���� ���� ����· ��԰��ҵ��������¥ ���ſɿƼ�
*/
#ifndef _SPI_H
#define _SPI_H

#include "stc15l204ea.h"

#define WRITE		0x7F
#define READ		0x80

sbit	TESTLED	= P3^7;
sbit	TXLED	= P1^7;
sbit	RXLED	= P1^6;

sbit	RESET_N	= P1^5;  			//output
sbit	SS 		= P1^4;       		//output
sbit	MOSI 	= P1^3;     		//output
sbit	SCLK	= P1^2;     		//output
sbit	PKT 	= P1^1;  			//input
sbit	MISO 	= P1^0;       		//input

void InitLT8900(void);
void spiWriteReg(unsigned char reg, unsigned char byteH, unsigned char byteL);
void spiReadreg(unsigned char reg);
unsigned char spiReadWrite(unsigned char Byte);

#endif
