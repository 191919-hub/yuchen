/***************************************************************
 *Copyright (C), 2017, Shanghai Eastsoft Microelectronics Co., Ltd.
 *�ļ�����  lib_printf.h
 *��  �ߣ�  Liut
 *��  ����  V1.00
 *��  �ڣ�  2017/07/14
 *��  ����  ���ڴ�ӡ�⺯��ͷ�ļ�
 *��  ע��  ������ ES8P508xоƬ
 ����������ѧϰ����ʾʹ�ã����û�ֱ�����ô����������ķ��ջ������е��κη������Ρ�
 ***************************************************************/
#ifndef __LIBPRINTF_H__
#define __LIBPRINTF_H__

#include "lib_uart.h"
#include <stdio.h>
#include "type.h"
#define  __PRINTF_USE_UART2__
int fputc(int ch, FILE *f);
static char *itoa(int value, char *string, int radix);


#ifdef __clang__ 
ErrorStatus UART_printf(uint8_t *Data, ...);
#define printf  UART_printf
#endif

#endif





/*************************END OF FILE**********************/