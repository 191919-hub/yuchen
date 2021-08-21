#include "Coupler.h"
#include "esp8266.h"
//#include "uart.h"
#include "logicsub.H"

//#include "lib_config.h"
#include "Disp.H"
#include "math.h"
#include "pdl_header.h"
#include "includes.h"
#include "Program_Cfg.h" //程序功能配置，比如禁止蜂鸣器、控制冷藏温度分辨率


#define CFG_DATA_START_ADDR 0x0000F400       //0x0000F400 为ES8P5086的APP Flash倒数第3页首地址  页尾地址为0x0000F7FF
#define CFG_DATA_CHECK_VALUE_ADDR 0x0000F7FC //页尾最后一个字存储校验值 sizeof(Cfg_Data)
#define CFG_DATA_PAGE_ADDR (CFG_DATA_START_ADDR / 1024)

#define TEMP_BASE 510

extern uint32_t t_halfsec;
extern unsigned char r_lcxswd;
extern unsigned char r_hwsjwd;//环境温度显示值,上偏38
extern unsigned char r_ldzt;//冷冻设定温度,上偏200
extern unsigned char r_lc_high_alarm;//冷藏高温报警设定的差值,(实际值*10)
extern unsigned char r_ld_low_alarm;//冷冻低温报警设定的差值
extern unsigned char r_ldxswd;//冷冻显示温度,上偏510
extern unsigned char r_lc_low_alarm; //冷藏低温报警设定的差值，(实际值*10)
extern unsigned char r_ld_high_alarm;//冷冻高温报警设定的差值
extern unsigned char f_LD_door,f_pow_err;
extern float r_lcxswd_float ;

WIFI_UART_Type Uart_WF;
Para_Base_t para_base;
Para_State_t para_state, para_state_last;
Para_Fault_t para_fault, para_fault_last;
Para_Event_t para_event, para_event_last;
Para_Heart_t para_heart;
int EVT_Queue[EVT_QUEUE_SIZE]; //事件队列
int EVT_Queue_head = 0;
int EVT_Queue_tail = 0;
unsigned long CntDown_n_ms_1 = 0;




static void Delay_ms(unsigned long n);

/*********************************************************
函数名: void UART0_SEND_IRQHandler(void)
描  述: UART中断服务程序
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void UART0_SEND_IRQHandler(void)  //函数名需要根据实际情况修改，函数内容可能需要自行适配
{
    Mfs_Uart_SendData(&UART0, Uart_WF.uart_txbuf[Uart_WF.uart_tx_count]);
    Uart_WF.uart_tx_count++;

    if(Uart_WF.uart_tx_count >= Uart_WF.uart_tx_length)
    {
        Uart_WF.uart_tx_count = 0;
        Mfs_Uart_DisableIrq(&UART0, UartTxIrq); //无效发送中断
    }

}
/*********************************************************
函数名: void UART0_RECV_IRQHandler(void)
描  述: UART中断服务程序
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void UART0_RECV_IRQHandler(void) //函数名需要根据实际情况修改，函数内容可能需要自行适配
{
    if ((&UART0)->SSR_f.PE == 1 || (&UART0)->SSR_f.FRE == 1 || (&UART0)->SSR_f.ORE == 1) //如有错误则清除标志
    {
        (&UART0)->SSR_f.REC = 1;
    }

    Uart_WF.uart_rxbuf[Uart_WF.uart_rx_count] = Mfs_Uart_ReceiveData(&UART0);

    if (Uart_WF.uart_rx_count < UARTX_RX_BUFF_SIZE - 1)
        Uart_WF.uart_rx_count++;

    if (Uart_WF.uart_rx_count >= 2)     //已接收至少2个字节
        Uart_WF.uart_rx_start_flag = 1; //置接收起始标志

    Uart_WF.uart_rx_time = 0; //清零UART通讯计时，表明仍处于数据帧接收过程
}

//处理来自wifi模块的数据
//主循环中不停调用
void WIFI_RX_Data_Deal(void)
{
    if(Uart_WF.uart_rx_finish_flag || (f_Wait_Wifi_Module_Resp && Timeout_Wifi_Module == 0))     //接收数据完成或超时
    {
        Uart_WF.uart_rx_finish_flag = 0;
        Deal_RecData_From_WifiModule();
        Uart_WF.uart_rx_length = 0;
    }
}

//向wifi模块发送数据
//1秒调用1次
void WIFI_TX_Data(void)
{
    Send_To_Wifi_Module();
}
/*********************************************************
函数名: void SendData(unsigned char *str,unsigned int len)
描  述: UART5发送数据
输入值: 无
输出值: 无
返回值: 无 
日  期：
**********************************************************/
void SendData(unsigned char *str,unsigned char len)
{
    memset(Uart_WF.uart_txbuf, 0xff, sizeof(Uart_WF.uart_txbuf)); //填充0xff
    memcpy(Uart_WF.uart_txbuf,str,len);

    Uart_WF.uart_tx_length = len;

    Mfs_Uart_EnableIrq(&UART0, UartTxIrq); //无效发送中断    //使能发送中断，在中断判断并结束发送数据帧   //SetUart5SendStatus();
}


/*********************************************************
函数名: void SendStr(unsigned char *str)
描  述: UART5发送字符串
输入值: 无
输出值: 无
返回值: 无 
日  期：
**********************************************************/
void SendStr(unsigned char *str)
{
	unsigned char len = 0;
	
	while(str[len] != 0)
		len ++;
	
    memset(Uart_WF.uart_txbuf, 0xff, sizeof(Uart_WF.uart_txbuf)); //填充0xff
    memcpy(Uart_WF.uart_txbuf,str,len);

    Uart_WF.uart_tx_length = len;

    Mfs_Uart_EnableIrq(&UART0, UartTxIrq); //  //使能发送中断，在中断判断并结束发送数据帧 
}
/*********************************************************
函数名: void CopyDataToIOTStructure(void)
描  述: 为IOT结构体赋值,根据各自需求进行更改
输入值: 无
输出值: 无
返回值: 无 
日  期：
**********************************************************/
void CopyDataToIOTStructure(void)  //需要自行适配
{
	
    /**************基础数据结构体的赋值*********************/
    para_base.Reserve_1 = 0xff;
	para_base.Reserve_2 = 0xff;
	para_base.Reserve_3 = 0xff;
	para_base.Reserve_4 = 0xff;
	para_base.Reserve_5 = 0xff;
	para_base.Reserve_6 = 0xff;
	para_base.LC_Disp_Temp = (sint16_t)((r_lcxswd_float - 38.0) * 10);  //冷藏显示温度
	para_base.LD_Disp_Temp = (sint16_t)((r_ldxswd - 200) * 10);  //冷冻显示温度 
	para_base.LC_Humi = 0xff;        //冷藏区相对湿度
	para_base.Up_Sens_Temp = 0xffff;  //上传感器温度
	para_base.Down_Sens_Temp = 0xffff;  //下传感器温度
	para_base.Comp_Current = 0xFF;    //压缩机电流
	para_base.EvapFan_Current = 0xFF;  //蒸发风机电流
	para_base.CdsFan_Current = 0xFF;   //冷凝风机电流
	para_base.Defr_Sens_Temp = 0xFFFF;  //化霜传感器温度
	para_base.Cds_Outlet_Temp = 0xFFFF;  //冷凝器出口温度
	para_base.Ambient_Temp = (sint16_t)(((int)r_hwsjwd - 38) * 10);    //环境温度
	para_base.OutGas_Temp = 0xFFFF;    //排气温度
	para_base.BackGas_Temp = 0xFFFF;    //回气温度
	para_base.High_Pressure = 0xFF;  //高压压力值
	para_base.Low_Pressure = 0xFF;  //低压压力值
	

    /**************状态数据结构体的赋值*********************/
    para_state.Reserve_1 = 0xff;
	para_state.Reserve_2 = 0xff;
	para_state.Reserve_3 = 0xff;
	para_state.LC_Temp_Set = (sint16_t)(((int)r_lczt - 38) * 10);  //冷藏设置温度
	para_state.LD_Temp_Set = (sint16_t)(((int)r_ldzt - 200) * 10);  //冷冻设置温度
	para_state.LC_Humi_Set = 0xFF;        //冷藏区相对湿度设定值
	para_state.LC_Temp_Adjust = 0xff;      //冷藏温度校正值
	para_state.LD_Temp_Adjust = 0xFF;      //冷冻温度校正值
	para_state.LC_Humi_Adjust = 0xFF;        //冷藏区相对湿度校正值
	para_state.Up_Sens_Adjust = 0xFF;      //上传感器校正值
	para_state.Down_Sens_Adjust = 0xFF;      //下传感器校正值
	para_state.LC_Alarm_HTemp = (sint16_t)(((int)r_lczt - 38 + r_lc_high_alarm) * 10);        //冷藏高温报警值
	para_state.LC_Alarm_LTemp = (sint16_t)(((int)r_lczt - 38 - r_lc_low_alarm) * 10);        //冷藏低温报警值
	para_state.LD_Alarm_HTemp = (sint16_t)(((int)r_ldzt - 200) * 10 + (int)r_ld_high_alarm * 10);        //冷冻高温报警值
	para_state.LD_Alarm_LTemp = (sint16_t)(((int)r_ldzt - 200) * 10 - (int)r_ld_low_alarm * 10);        //冷冻低温报警值
    para_state.Ambient_Temp_Adjust = 0xff;    //环境温度校准值
	para_state.in_Defrost_State = (uint8_t)(f_defrost);    //化霜状态
    if(f_power_off)
    {
        para_state.LC_Comp_Running = 0;   
    }
    else
    {
        para_state.LC_Comp_Running = (uint8_t)(f_lc_compressor);    //冷藏压机正在运行
    }
    if(f_power_off)
    {
        para_state.LD_Comp_Running = 0;    
    }
    else
    {
        para_state.LD_Comp_Running = (uint8_t)(f_compressor);    //冷冻压机正在运行
    }
    
    if(f_power_off)
    {
        para_state.Light_is_On = 0;
    }
    else
    {
        para_state.Light_is_On = (uint8_t)(f_LightStatusPin);
    }
	para_state.Door_is_Open = (uint8_t)(f_door_state);    //门开着
	para_state.RefrigValve_is_Open = 0xFF;    //制冷电磁阀开着
    

    /**************故障数据结构体的赋值*********************/
    
	para_fault.Reserve_1 = 0xff;
	para_fault.Reserve_2 = 0xff;
	para_fault.Reserve_3 = 0xff;
    if(f_hw_high38)
        para_fault.flag_fault[0] |= BIT0_MASK;
	else
		para_fault.flag_fault[0] &= ~BIT0_MASK;
	
    if(f_ld_low)
        para_fault.flag_fault[0] |= BIT1_MASK;
	else
		para_fault.flag_fault[0] &= ~BIT1_MASK;
	
    if(f_ld_high)
        para_fault.flag_fault[0] |= BIT2_MASK;
	else
		para_fault.flag_fault[0] &= ~BIT2_MASK;
	
    if(f_lc_low)
        para_fault.flag_fault[0] |= BIT3_MASK;
	else
		para_fault.flag_fault[0] &= ~BIT3_MASK;
	
    if(f_lc_high)
        para_fault.flag_fault[0] |= BIT4_MASK;
	else
		para_fault.flag_fault[0] &= ~BIT4_MASK;
		
    // if()
    //     para_fault.flag_fault[0] |= BIT5_MASK;
	
    if(f_ld_sensor_err)
        para_fault.flag_fault[0] |= BIT6_MASK;
	else
		para_fault.flag_fault[0] &= ~BIT6_MASK;
	
    if(f_lc_sensor_err)
        para_fault.flag_fault[0] |= BIT7_MASK;
	else
		para_fault.flag_fault[0] &= ~BIT7_MASK;
	
    if(f_hw_sensor_err)
        para_fault.flag_fault[1] |= BIT0_MASK;
	else
		para_fault.flag_fault[1] &= ~BIT0_MASK;
	
	// if()
    //     para_fault.flag_fault[1] |= BIT1_MASK;
	
    if(f_battery)
        para_fault.flag_fault[1] |= BIT2_MASK;
	else
		para_fault.flag_fault[1] &= ~BIT2_MASK;
	
//     if(f_disp_sensor_err)
//         para_fault.flag_fault[1] |= BIT3_MASK;
// 	else
// 		para_fault.flag_fault[1] &= ~BIT3_MASK;
	
    // if(f_disp_sensor_err)
    //     para_fault.flag_fault[1] |= BIT4_MASK;
	// else
	// 	para_fault.flag_fault[1] &= ~BIT4_MASK;
	
    if(g_Sys_Erflag0_Comm)
        para_fault.flag_fault[1] |= BIT5_MASK;
	else
		para_fault.flag_fault[1] &= ~BIT5_MASK;
	
    if(f_power_off)
        para_fault.flag_fault[1] |= BIT6_MASK;
	else
		para_fault.flag_fault[1] &= ~BIT6_MASK;
	
    // if(f_ambient_low_err)
    //     para_fault.flag_fault[1] |= BIT7_MASK;
	// else
	// 	para_fault.flag_fault[1] &= ~BIT7_MASK;
	
    // if(f_dooropen_buzz)   //或者使用f_door_open，开门后f_door_open立即置位，而f_dooropen_buzz在延时x分钟后置位
    //     para_fault.flag_fault[2] |= BIT7_MASK;
	// else
	// 	para_fault.flag_fault[2] &= ~BIT7_MASK;
	
    // if()
    //     para_fault.flag_fault[2] |= BIT1_MASK;
    // if()
    //     para_fault.flag_fault[2] |= BIT2_MASK;
    // if()
    //     para_fault.flag_fault[2] |= BIT3_MASK;
    // if()
    //     para_fault.flag_fault[2] |= BIT4_MASK;
    // if()
    //     para_fault.flag_fault[2] |= BIT5_MASK;
    // if()
    //     para_fault.flag_fault[2] |= BIT6_MASK;
    // if()
    //     para_fault.flag_fault[2] |= BIT7_MASK;
    
    
    /**************事件数据结构体的赋值*********************/

    /**************心跳数据结构体的赋值*********************/
	para_heart.Sec_From_PwrOn = t_halfsec / 2;  
	//para_heart.Day_CPU_TotalRun = Flash_Data.CPU_TotalRun_Minute / 1440;  
}

//事件记录
void Event_Log(void)   //需要自行适配
{
    static unsigned char f_door_last = 0xff; //上次门状态

    //开关门事件
    if (f_door_state != f_door_last && f_door_last != 0xff)
    {
        //只有在线状态才记录开关门事件
        //if(Wifi_Module_Uart_State == WAIT_MQTT_PUBLISH || Wifi_Module_Uart_State == WAIT_MQTT_PUBLISH_Resp)
        {
            //要先判断队列是否已满
            if ((EVT_Queue_tail + 1) % EVT_QUEUE_SIZE == EVT_Queue_head)
            {
                ;//printf("队列已满，无法从队尾插入元素\n");
            }
            else
            {
                if (f_door_open == 1)
                    EVT_Queue[EVT_Queue_tail] = 1;  //事件码：1代表开门事件，2代表关门事件
                else
                    EVT_Queue[EVT_Queue_tail] = 2;
                EVT_Queue_tail = (EVT_Queue_tail + 1) % EVT_QUEUE_SIZE;
            }

        }
        
    }
    if (f_first_ad)
        f_door_last = f_door_state;
}

//uart接收空闲计时，用于判断一帧是否结束
//每隔Cycle个ms调用一次，Cycle <= (10/2),建议Cycle = 2
void WIFI_UART_IdleTimer(unsigned int Cycle)   //无需修改函数内容
{
    Uart_WF.uart_rx_time++;                        //累加接收计时
    if (Uart_WF.uart_rx_time * Cycle >= 10)  //超过10ms没有接收到数据，则表明一帧接收完
    {
        Uart_WF.uart_rx_time = 0;                  //清零UART通讯计时
        if (Uart_WF.uart_rx_start_flag == 1)       //判断接收起始标志
        {
            Uart_WF.uart_rx_start_flag = 0;        //清接收起始标志
            Uart_WF.uart_rx_finish_flag = 1;       //置接收完成标志

            Uart_WF.uart_rx_length = Uart_WF.uart_rx_count;
            if(WiFi_RxCounter + Uart_WF.uart_rx_count < WiFi_RXBUFF_SIZE)  //WiFi_RXBUFF_SIZE剩余空间足够
            {
                memcpy(&WiFi_RX_BUF[WiFi_RxCounter], Uart_WF.uart_rxbuf, Uart_WF.uart_rx_count);  //把当前接收的一帧数据复制到WiFi_RX_BUF
                WiFi_RxCounter += Uart_WF.uart_rx_count;
            }
            else        //WiFi_RXBUFF_SIZE剩余空间不足，复制到WiFi_RXBUFF_SIZE用尽，其余数据丢弃
            {
                memcpy(&WiFi_RX_BUF[WiFi_RxCounter], Uart_WF.uart_rxbuf, WiFi_RXBUFF_SIZE - WiFi_RxCounter);
                WiFi_RxCounter = WiFi_RXBUFF_SIZE;
            }
                
            Uart_WF.uart_rx_count = 0;
        }
    }
}

//uart接收超时倒计时，用于判断是否串口接收超时
//每隔Cycle个ms调用一次，Cycle <= 10,建议Cycle = 2
void WIFI_UART_TimeOut_CntDown(unsigned int Cycle)  //无需修改函数内容
{
    if(f_Wait_Wifi_Module_Resp)
    {
        if(Timeout_Wifi_Module >= Cycle)
        {
            Timeout_Wifi_Module -= Cycle;
        }
        else
        {
            Timeout_Wifi_Module = 0;
        }
    }
}



//存储Wifi配置数据
void WriteCfgData(void) //需要自行适配
{
    int i;
    char *Write_Ptr = (char *)(&Cfg_Data);
    unsigned int E2_Addr;  //要存储数据的E2的byte地址
    unsigned char E2_Addr_High,E2_Addr_Low;
    unsigned char Chip_Addr = 0xA0;

    Begin();
    WriteByte(Chip_Addr); ///write chip addres 0xa0      
    E2_Addr = 0x0050;
    E2_Addr_Low = E2_Addr & 0xff;
    WriteByte(E2_Addr_Low); ///write data low  addres

    for (i = 0; i < sizeof(Cfg_Data); i++)
    //for (i = 0; i < 120; i++)
    {
        if((i % 16 == 0) && (i != 0))  //写到了页尾，延时等待写入完成，从下一页头继续写
        {
            C_sda = 1;
		    Stop();

            Delay_ms(15);  //等待一页数据写完
            E2_Addr += 16;
            E2_Addr_Low = E2_Addr & 0xff;
            if(E2_Addr >= 0x100)  //chip address 需要变化(E2_Addr向chip address借位)
            {
                Chip_Addr = 0xA2;
            }
            else
            {
                Chip_Addr = 0xA0;
            }
            Begin();
            WriteByte(Chip_Addr); ///write chip addres 
            WriteByte(E2_Addr_Low); ///write data low  addres

        }

        WriteByte(*Write_Ptr);
        Write_Ptr++;
        
    }
    WriteByte(0xAB);
    C_sda = 1;
    Stop();
}

//读取Wifi配置数据
void ReadCfgData(void)  //需要自行适配
{
    volatile int i;
    char *Read_Ptr = (char *)(&Cfg_Data);
	unsigned char temp;
    Delay_ms(15);  
	Begin();
	WriteByte(0xA0); ///write chip addres 0xa0
	WriteByte(0x50); ///write data addres
	Begin();
	WriteByte(0xA1); ///write chip addres 0xa0

    for (i = 0; i < sizeof(Cfg_Data); i++)
    //for (i = 0; i < 20; i++)
    {
        *Read_Ptr = ReadByte();
        Sendscl();
        Read_Ptr++;
    }
    temp = ReadByte();   //校验值
    C_sda = 1;
    Sendscl();
    Stop();

	if (temp != 0xAB)
	{
		strcpy(Cfg_Data.WifiSSID, "AP");
        strcpy(Cfg_Data.WifiPwd, "12345678");
        //strcpy(Cfg_Data.WifiPwd, "12345678");
        strcpy(Cfg_Data.MQTTServerDomain, "msgtest2.haierbiomedical.com");
        strcpy(Cfg_Data.MQTTServerPort, "1777");
        strcpy(Cfg_Data.Username, "haierTest");
        strcpy(Cfg_Data.Password, "haier");
        strcpy(Cfg_Data.BECode, "BE0F130AR00000000000");
        Delay_ms(15);  
        WriteCfgData();
	}

}

/*********************************************************
函数名: void BuzzBp(void)
描  述: 蜂鸣器响一下
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void BuzzBp(void)  //需要自行适配
{
    BuzzBi();
}

static void Delay_ms(unsigned long n)
{
    CntDown_n_ms_1 = n;
    while(CntDown_n_ms_1)
        Clear_Watchdog();
}