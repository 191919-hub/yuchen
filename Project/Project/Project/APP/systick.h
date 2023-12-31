/*********************************************************
*Copyright (C), 2017, Shanghai Eastsoft Microelectronics Co., Ltd.
*文件名:  systick.h
*作  者:  Liut
*版  本:  V1.01
*日  期:  2017/06/29
*描  述:  系统滴答模块程序头文件
*备  注:  适用于HRSDK-GDB-8P506
          本软件仅供学习和演示使用，对用户直接引用代码所带来的风险或后果不承担任何法律责任。
**********************************************************/
#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#include "lib_config.h"
#include "system_es8p508x.h"

void User_SysTickInit(void);
void Delay_100us(__IO uint32_t nTime);
void TimingDelay_Decrement(void);
#endif

