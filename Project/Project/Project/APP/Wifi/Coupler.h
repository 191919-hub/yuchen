#ifndef __COUPLER_H__
#define __COUPLER_H__


#define EVT_QUEUE_SIZE 20
#define UART_WIFI UART0
#define WIFI_UART_RX_BUFF_SIZE 1024  //单帧最大接收长度
#define UARTX_RX_BUFF_SIZE 1024

typedef char sint8_t;
typedef unsigned char uint8_t;
typedef short int sint16_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;



typedef struct
{
    uint8_t uart_txbuf[255];
    uint8_t uart_tx_length;
    uint8_t uart_tx_count;
    uint8_t uart_tx_ready_flag;

    uint8_t uart_rxbuf[WIFI_UART_RX_BUFF_SIZE];
    uint16_t uart_rx_length;
    uint16_t uart_rx_count;
    uint8_t uart_rx_finish_flag;

    uint8_t uart_rx_start_flag;
    uint8_t uart_rx_time;
}WIFI_UART_Type;

//基础数据
#pragma  pack (push,1)
typedef struct _Para_Base
{
	uint8_t  Reserve_1;
	uint8_t  Reserve_2;
	uint8_t  Reserve_3;
	uint8_t  Reserve_4;
	uint8_t  Reserve_5;
	uint8_t  Reserve_6;
	sint16_t LC_Disp_Temp;  //冷藏显示温度
	sint16_t LD_Disp_Temp;  //冷冻显示温度
	uint8_t  LC_Humi;        //冷藏区相对湿度
	sint16_t Up_Sens_Temp;  //上传感器温度
	sint16_t Down_Sens_Temp;  //上传感器温度
	uint8_t  Comp_Current;    //压缩机电流
	uint8_t  EvapFan_Current;  //蒸发风机电流
	uint8_t  CdsFan_Current;   //冷凝风机电流
	sint16_t Defr_Sens_Temp;  //化霜传感器温度
	sint16_t Cds_Outlet_Temp;  //冷凝器出口温度
	sint16_t Ambient_Temp;    //环境温度
	sint16_t OutGas_Temp;    //排气温度
	sint16_t BackGas_Temp;    //回气温度
	uint8_t  High_Pressure;  //高压压力值
	uint8_t  Low_Pressure;  //低压压力值

}Para_Base_t;
#pragma pack(pop)

//状态数据
#pragma  pack (push,1)
typedef struct _Para_State
{
	uint8_t  Reserve_1;
	uint8_t  Reserve_2;
	uint8_t  Reserve_3;
	sint16_t LC_Temp_Set;  //冷藏设置温度
	sint16_t LD_Temp_Set;  //冷冻设置温度
	uint8_t  LC_Humi_Set;        //冷藏区相对湿度设定值
	sint8_t    LC_Temp_Adjust;      //冷藏温度校正值
	sint8_t    LD_Temp_Adjust;      //冷冻温度校正值
	sint8_t  LC_Humi_Adjust;        //冷藏区相对湿度校正值
	sint8_t    Up_Sens_Adjust;      //上传感器校正值
	sint8_t    Down_Sens_Adjust;      //上传感器校正值
	sint16_t LC_Alarm_HTemp;        //冷藏高温报警值
	sint16_t LC_Alarm_LTemp;        //冷藏低温报警值
	sint16_t LD_Alarm_HTemp;        //冷冻高温报警值
	sint16_t LD_Alarm_LTemp;        //冷冻低温报警值
	sint8_t  Ambient_Temp_Adjust;    //环境温度校准值
	uint8_t  in_Defrost_State;    //化霜状态
	uint8_t  LC_Comp_Running;    //冷藏压机正在运行
	uint8_t  LD_Comp_Running;    //冷冻压机正在运行
	uint8_t  Light_is_On;    //照明灯亮
	uint8_t  Door_is_Open;    //门开着
	uint8_t  RefrigValve_is_Open;    //制冷电磁阀开着
}Para_State_t;
#pragma pack(pop)

//故障数据
#pragma  pack (push,1)
typedef struct _Para_Fault
{
	uint8_t  Reserve_1;
	uint8_t  Reserve_2;
	uint8_t  Reserve_3;
	uint8_t flag_fault[3];

}Para_Fault_t;
#pragma pack(pop)

//事件数据
#pragma  pack (push,1)
typedef struct _Para_Event
{
	uint8_t flag_event[17];
}Para_Event_t;
#pragma pack(pop)

//心跳数据
#pragma  pack (push,1)
typedef struct _Para_Heart
{
	uint32_t  Sec_From_PwrOn;  //自上电后运行时间  秒
	uint32_t  Day_CPU_TotalRun;  //设备累计工作时间  天
}Para_Heart_t;
#pragma pack(pop)

#define BIT0_MASK 0x01
#define BIT1_MASK 0x02
#define BIT2_MASK 0x04
#define BIT3_MASK 0x08
#define BIT4_MASK 0x10
#define BIT5_MASK 0x20
#define BIT6_MASK 0x40
#define BIT7_MASK 0x80



extern int EVT_Queue[EVT_QUEUE_SIZE]; //事件队列
extern int EVT_Queue_head;
extern int EVT_Queue_tail;
extern WIFI_UART_Type Uart_WF;
extern Para_Base_t para_base;
extern Para_State_t para_state, para_state_last;
extern Para_Fault_t para_fault, para_fault_last;
extern Para_Event_t para_event, para_event_last;
extern Para_Heart_t para_heart;
extern unsigned long CntDown_n_ms_1;

extern void WIFI_UART_Init(void);
extern void UART_Wifi_SEND_IRQHandler(void);
extern void UART_Wifi_RECV_IRQHandler(void);
extern void WIFI_UART_IdleTimer(unsigned int Cycle);
extern void BuzzBp(void);
extern void CopyDataToIOTStructure(void);
extern void WriteCfgData(void);
extern void ReadCfgData(void);
extern void Event_Log(void);
extern void SendData(unsigned char *str,unsigned char len);
extern void SendStr(unsigned char *str);




#endif