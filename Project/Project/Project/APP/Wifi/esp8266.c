
//#include "uart.h"
#include "coupler.h"
#include "esp8266.h"
//#include "HYC-68.h"
//#include "Iot_comm.h"  //使用到了Para_Base_t等定义,CopyDataToIOTStructure函数
#include "stdio.h"
#include "string.h"


extern uint32_t t_halfsec;
unsigned char f_disp_Wifi_Cfg = 0 ;  //显示Wifi正在配置


int Timeout_Wifi_Module = 0;   //wifi模块响应倒计时，等于0代表超时
char f_Wait_Wifi_Module_Resp = 0;  //等待wifi模块的响应
char Wifi_Module_Uart_State = WAIT_EXIT_TRANS;
char f_Enter_WifiCfg = 0;  //按键后置位，用于进入wifi配置模式
char f_Exit_WifiCfg = 0;   //退出配置模式标志
static uint32_t t_Last_Cmd;
static uint32_t Delay_HalfSec_Between_Cmd;  //发给wifi模块两帧数据之间的间隔
static uint32_t t_Last_MQTT_Publish = 0;   //上一次mqtt发布的时刻
static uint32_t t_Last_MQTT_BASE_Publish = 0;  //上一次基础数据发布的时刻
static uint32_t t_Last_MQTT_HEART_Publish = 0;  //上一次心跳数据发布的时刻
char Published_Type = MQTT_PING_DATA;  //已经发送的报文类型  
char f_RePublish = 0;  //重复发送标志
char Cnt_Publish_NoReply = 1;  //publish超时无响应次数
char f_No_WifiModule = 0;  //开机后连续3次发送AT+RST 无响应“OK”,认为没有连接wifi模块，如果串口还复用为其他功能，可以开启其他功能。
uint8_t f_Exist_WifiModule = 0;  //确认存在wifi模块

char Client_ID[51]; //客户端ID
static char AT_CMD_Buf[255];
char WiFi_RX_BUF[WiFi_RXBUFF_SIZE];       
volatile uint32_t WiFi_RxCounter = 0;
uint32_t t_Set_RePublish = 0;
uint8_t First_State_Frame = 1,First_Fault_Frame = 1; //上电首先发送一帧故障帧、状态帧

//__align(4) Wifi_Cfg_t Cfg_Data;
Wifi_Cfg_t Cfg_Data;

unsigned char f_RecStr_Err = 0;  //接收到的字符串不合法
enum MQTT_DATA_TYPE gPubType = MQTT_HEART_DATA;   //将要发布的数据类型


unsigned char Parse_CfgData(void);

void Deal_RecData_From_WifiModule(void)
{
	static uint8_t Cnt_TimeOut = 0;  //响应超时次数
    

    switch(Wifi_Module_Uart_State)
    {
        //正常mqtt传输流程
        case WAIT_EXIT_TRANS_Resp:
            if (strstr(WiFi_RX_BUF, "OK")) //如果接收到
            {
                Wifi_Module_Uart_State = WAIT_RESET;  //发送重启指令
                f_Wait_Wifi_Module_Resp = 0;
            }
            else if(Timeout_Wifi_Module == 0)  //超时,实际上“+++”指令不会有响应，默认成功了
            {
                Wifi_Module_Uart_State = WAIT_RESET;  //发送重启指令
            }
            break;
        case WAIT_RESET_Resp:
            f_disp_Wifi_Cfg = 0;  //退出Cfg显示
            if (strstr(WiFi_RX_BUF, "OK")) //如果接收到OK表示复位成功
            {
                f_Exist_WifiModule = 1;  //确定连接的是wifi模块
                Cnt_TimeOut = 0;
                Wifi_Module_Uart_State = WAIT_PWRON;  //下一步  发送at 等待响应 ok 确认模块正常工作
                f_Wait_Wifi_Module_Resp = 0;
            }
            else if(Timeout_Wifi_Module == 0)  //超时
            {
                Cnt_TimeOut ++;
                if(Cnt_TimeOut < 3)
                    Wifi_Module_Uart_State = WAIT_RESET;
                else
                {
                    if(!f_Exist_WifiModule)
                        f_No_WifiModule = 1;  //认为连接的不是wifi模块
                    Cnt_TimeOut = 0;
                    Wifi_Module_Uart_State = WAIT_EXIT_TRANS;  //怀疑没有成功退出透传模式
                }
                
            }
            break;
        case WAIT_PWRON_Resp:
            if (strstr(WiFi_RX_BUF, "OK")) //接收到正确响应
            {
                Wifi_Module_Uart_State = WAIT_SET_STATION_MODE;  //下一步  发送指令 等待响应 
                f_Wait_Wifi_Module_Resp = 0;
            }
            else if(Timeout_Wifi_Module == 0)  //超时
            {
                Wifi_Module_Uart_State = WAIT_PWRON;
            }
            break;
        case WAIT_SET_STATION_MODE_Resp:
            if (strstr(WiFi_RX_BUF, "OK")) //接收到正确响应
            {
                Wifi_Module_Uart_State = WAIT_CONNECT_AP;  //下一步  发送指令 等待响应 
                f_Wait_Wifi_Module_Resp = 0;
            }
            else  if(Timeout_Wifi_Module == 0)  //超时
            {
                Wifi_Module_Uart_State = WAIT_SET_STATION_MODE;
            }
            break;
        case WAIT_CONNECT_AP_Resp:
            //if (strstr(WiFi_RX_BUF, "WIFI GOT IP")) //接收到第二条响应
            if (strstr(WiFi_RX_BUF, "OK")) //接收到第三条响应
            {
                Cnt_TimeOut = 0;
                Wifi_Module_Uart_State = WAIT_CONNECT_TCP;  //下一步  发送指令 等待响应 
				f_Wait_Wifi_Module_Resp = 0;
                Delay_HalfSec_Between_Cmd = 10;   //延时5秒 
                t_Last_Cmd = t_halfsec;
            }
            else if(Timeout_Wifi_Module == 0)  //超时
            {
                Cnt_TimeOut ++;
                if(Cnt_TimeOut < 5)
                    Wifi_Module_Uart_State = WAIT_CONNECT_AP;
                else
                {
                    Cnt_TimeOut = 0;
                    Wifi_Module_Uart_State = WAIT_EXIT_TRANS;  //怀疑联网有问题，复位从头再来
                }
                
            }
            break;
        case WAIT_CONNECT_TCP_Resp:
			if (strstr(WiFi_RX_BUF, "CONNECT")) //接收到正确响应
            {
                Cnt_TimeOut = 0;
                Wifi_Module_Uart_State = WAIT_SET_TRANSPARENT;  //下一步  发送指令 等待响应 
				f_Wait_Wifi_Module_Resp = 0;
                Delay_HalfSec_Between_Cmd = 1;   //延时两秒 执行进入透传命令
                t_Last_Cmd = t_halfsec;
            }
            else if(Timeout_Wifi_Module == 0)  //超时 
            {
                Cnt_TimeOut ++;
                if(Cnt_TimeOut < 5)
                    Wifi_Module_Uart_State = WAIT_CONNECT_TCP;
                else
                {
                    Cnt_TimeOut = 0;
                    Wifi_Module_Uart_State = WAIT_EXIT_TRANS;  //怀疑联网有问题，复位从头再来
                }
                
            }
            break;
        case WAIT_SET_TRANSPARENT_Resp:
            if (strstr(WiFi_RX_BUF, "OK")) //接收到正确响应
            {
                Cnt_TimeOut = 0;
                Wifi_Module_Uart_State = WAIT_START_TRANSPARENT;  //下一步  发送指令 等待响应 
				f_Wait_Wifi_Module_Resp = 0;
                Delay_HalfSec_Between_Cmd = 1;   //延时1秒 执行进入透传命令
                t_Last_Cmd = t_halfsec;
            }
            else if(Timeout_Wifi_Module == 0)  //超时  
            {
                Cnt_TimeOut ++;
                if(Cnt_TimeOut < 5)
                    Wifi_Module_Uart_State = WAIT_SET_TRANSPARENT;
                else
                {
                    Cnt_TimeOut = 0;
                    Wifi_Module_Uart_State = WAIT_EXIT_TRANS;  //复位从头再来
                }
                
            }
            break;
        case WAIT_START_TRANSPARENT_Resp:
            if (strstr(WiFi_RX_BUF, ">")) //接收到正确响应
            {
                Cnt_TimeOut = 0;
                Wifi_Module_Uart_State = WAIT_MQTT_CONNECT;  //下一步  发送指令 等待响应 
				f_Wait_Wifi_Module_Resp = 0;
                
            }
            else if(Timeout_Wifi_Module == 0)  //超时
            {
                Cnt_TimeOut ++;
                if(Cnt_TimeOut < 5)
                    Wifi_Module_Uart_State = WAIT_START_TRANSPARENT;
                else
                {
                    Cnt_TimeOut = 0;
                    Wifi_Module_Uart_State = WAIT_EXIT_TRANS;  //复位从头再来
                }
                
            }
            break;
		case WAIT_MQTT_CONNECT_Resp:
			if ( WiFi_RX_BUF[0] == 0x20 && WiFi_RX_BUF[1] == 0x02 && WiFi_RX_BUF[2] == 0x00 && WiFi_RX_BUF[3] == 0x00 ) //接收到正确响应
            {
                Cnt_TimeOut = 0;
                Wifi_Module_Uart_State = WAIT_MQTT_PUBLISH;  //下一步  发送指令 等待响应 
                f_Wait_Wifi_Module_Resp = 0;
				
				EVT_Queue_tail = EVT_Queue_head = 0;    //清空事件记录缓存
            }
			else if(WiFi_RX_BUF[0] == 0x20 && WiFi_RX_BUF[1] == 0x02 && WiFi_RX_BUF[2] == 0x00 && WiFi_RX_BUF[3] != 0x00) //有错误，错误码为第四个字节
            {
               Cnt_TimeOut = 0;
               Wifi_Module_Uart_State = WAIT_EXIT_TRANS;  //复位从头再来
               ; 
            }
            else if(Timeout_Wifi_Module == 0)  //超时
            {
                Cnt_TimeOut ++;
                if(Cnt_TimeOut < 5)
                    Wifi_Module_Uart_State = WAIT_MQTT_CONNECT;
                else
                {
                    Cnt_TimeOut = 0;
                    Wifi_Module_Uart_State = WAIT_EXIT_TRANS;  //复位从头再来
                }
                
            }
            break;
        case WAIT_MQTT_PUBLISH_Resp: 
            if(Published_Type == MQTT_PING_DATA)  //PINGREQ
            {
                if ( WiFi_RX_BUF[0] == 0xD0 && WiFi_RX_BUF[1] == 0x00 ) //接收到正确响应 PINGRESP
                {
                    Cnt_Publish_NoReply = 0;
                    f_RePublish = 0;  
                    Wifi_Module_Uart_State = WAIT_MQTT_PUBLISH;  //准备发送下一个报文
                    f_Wait_Wifi_Module_Resp = 0;
                    break;
                }
            }
            else  //心跳 或 基础  或 状态 ………
            {
                if ( WiFi_RX_BUF[0] == 0x40 && WiFi_RX_BUF[1] == 0x02) //接收到正确响应 PUBACK
                {
                    Cnt_Publish_NoReply = 0;
                    f_RePublish = 0;  
                    Wifi_Module_Uart_State = WAIT_MQTT_PUBLISH;  //准备发送下一个报文
                    f_Wait_Wifi_Module_Resp = 0;
                    break;
                }
            }
            if(Timeout_Wifi_Module == 0)  //超时
            {
                Cnt_Publish_NoReply ++;
                if(Cnt_Publish_NoReply < 3) 
                {
                    Wifi_Module_Uart_State = WAIT_MQTT_PUBLISH;  //再次发送报文
                    f_RePublish = 1;  //重复发送标志置位
                    t_Set_RePublish = t_halfsec;
                }
                else   //连续3次收不到响应
                {
                    Cnt_Publish_NoReply = 0;
                    Wifi_Module_Uart_State = WAIT_EXIT_TRANS;  //退出透传模式，复位wifi模块，重连mqtt服务器，
                    //f_RePublish = 0;  
                }
            }
            break;

        //配置流程
        case WAIT_CFGMODE_EXIT_TRANS_Resp:
            if (strstr(WiFi_RX_BUF, "OK")) //如果接收到
            {
                Wifi_Module_Uart_State = WAIT_CFGMODE_ENTER_RST;  //发送重启指令
                f_Wait_Wifi_Module_Resp = 0;
            }
            else  if(Timeout_Wifi_Module == 0)  //超时
            {
                Wifi_Module_Uart_State = WAIT_CFGMODE_ENTER_RST;  //发送重启指令
            }
            break;
        case WAIT_CFGMODE_ENTER_RST_Resp:
            if (strstr(WiFi_RX_BUF, "OK")) //如果接收到
            {
                Wifi_Module_Uart_State = WAIT_CFGMODE_START_AP;  //重启成功，开启ap
                f_Wait_Wifi_Module_Resp = 0;
            }
            else  if(Timeout_Wifi_Module == 0)  //超时
            {
                Wifi_Module_Uart_State = WAIT_CFGMODE_ENTER_RST;  //再次重启
            }
            break;
        case WAIT_CFGMODE_START_AP_Resp:
            if (strstr(WiFi_RX_BUF, "OK")) //如果接收到
            {
                Wifi_Module_Uart_State = WAIT_CFGMODE_SET_IP;  //设置IP
                f_Wait_Wifi_Module_Resp = 0;
            }
            else  if(Timeout_Wifi_Module == 0)  //超时
            {
                Wifi_Module_Uart_State = WAIT_CFGMODE_START_AP;  //再次开启AP
            }
            break;
        case WAIT_CFGMODE_SET_IP_Resp:
            if (strstr(WiFi_RX_BUF, "OK")) //如果接收到
            {
                Wifi_Module_Uart_State = WAIT_CFGMODE_SET_SSID;  //设置wifi名称
                f_Wait_Wifi_Module_Resp = 0;
            }
            else  if(Timeout_Wifi_Module == 0)  //超时
            {
                Wifi_Module_Uart_State = WAIT_CFGMODE_SET_IP;  //再次设置IP
            }
            break;
        case WAIT_CFGMODE_SET_SSID_Resp:
            if (strstr(WiFi_RX_BUF, "OK")) //如果接收到
            {
                Wifi_Module_Uart_State = WAIT_CFGMODE_SET_UDP;  //设置UDP
                f_Wait_Wifi_Module_Resp = 0;
            }
            else  if(Timeout_Wifi_Module == 0)  //超时
            {
                Wifi_Module_Uart_State = WAIT_CFGMODE_SET_SSID;  //再次设置wifi名称
            }
            break;
        case WAIT_CFGMODE_SET_UDP_Resp:
            if (strstr(WiFi_RX_BUF, "OK")) //如果接收到
            {
                Wifi_Module_Uart_State = WAIT_CFGMODE_REC_DATA;  //
                f_Wait_Wifi_Module_Resp = 0;
            }
            else  if(Timeout_Wifi_Module == 0)  //超时
            {
                Wifi_Module_Uart_State = WAIT_CFGMODE_SET_UDP;  //再次设置UDP
            }
            break;
        case WAIT_CFGMODE_REC_DATA_Resp:
            f_disp_Wifi_Cfg = 2; //闪烁显示CFg
            if (strstr(WiFi_RX_BUF, "+IPD")) //如果接收到
            {
                f_RecStr_Err = Parse_CfgData();
                if(f_RecStr_Err == 0)  //解析正确
                {
                    WriteCfgData();
                    Wifi_Module_Uart_State = WAIT_CFGMODE_SEND_ACK_1;  //下一步
                    BuzzBp();  
                }
                else  //数据有误
                {
                    Wifi_Module_Uart_State = WAIT_CFGMODE_SEND_ACK_1;  // 先通知app数据有误，再回来接收
                }
                f_Wait_Wifi_Module_Resp = 0;                        
            }
            else  if(Timeout_Wifi_Module == 0)  //超时
            {
                Wifi_Module_Uart_State = WAIT_RESET;  //退出配置模式，启动mqtt正常传输流程
            }
            break;
        case WAIT_CFGMODE_SEND_ACK_1_Resp:
            if (strstr(WiFi_RX_BUF, ">")) //如果接收到
            {
                Wifi_Module_Uart_State = WAIT_CFGMODE_SEND_ACK_2;  //下一步  
                f_Wait_Wifi_Module_Resp = 0;
            }
            else  if(Timeout_Wifi_Module == 0)  //超时
            {
                Wifi_Module_Uart_State = WAIT_CFGMODE_SEND_ACK_1;  //重新发
            }
            break;
        case WAIT_CFGMODE_SEND_ACK_2_Resp:
            if (strstr(WiFi_RX_BUF, "OK")) //如果接收到
            {
				if(f_RecStr_Err == 0)  //配置数据无误
                {
                    Wifi_Module_Uart_State = WAIT_RESET;  //下一步  退出配置模式，启动mqtt正常传输流程
                }
				else   //
					Wifi_Module_Uart_State = WAIT_CFGMODE_REC_DATA;  //继续接收数据
                f_Wait_Wifi_Module_Resp = 0;
            }
            else  if(Timeout_Wifi_Module == 0)  //超时
            {
				if(f_RecStr_Err == 0)  //配置数据无误
                {
                    Wifi_Module_Uart_State = WAIT_RESET;  //退出配置模式，启动mqtt正常传输流程
                }
					
				else
					Wifi_Module_Uart_State = WAIT_CFGMODE_REC_DATA;  //继续接收数据
            }
            break;

        default:break;
    }
}



void Send_To_Wifi_Module(void)
{
    if(f_Enter_WifiCfg)
    {
        f_Enter_WifiCfg = 0;
        f_disp_Wifi_Cfg = 1;  //闪烁显示"..."
        Wifi_Module_Uart_State = WAIT_CFGMODE_EXIT_TRANS;
    }
    if(f_Exit_WifiCfg)
    {
        f_Exit_WifiCfg = 0;
        f_disp_Wifi_Cfg = 0;
        Wifi_Module_Uart_State = WAIT_EXIT_TRANS;  //退出配置模式，启动mqtt正常传输流程
    }

    switch(Wifi_Module_Uart_State)
    {
        //正常工作流程
        case WAIT_EXIT_TRANS:
            Wifi_Module_Uart_State = WAIT_EXIT_TRANS_Resp;
            WiFi_SendCmd("+++",1000);
            break;
        case WAIT_RESET:
            Wifi_Module_Uart_State = WAIT_RESET_Resp;
            WiFi_SendCmd("AT+RST\r\n",1000);  //1000
            break;
        case WAIT_PWRON:
            Wifi_Module_Uart_State = WAIT_PWRON_Resp;
            WiFi_SendCmd("AT\r\n",1000);
            break;
        case WAIT_SET_STATION_MODE:
            Wifi_Module_Uart_State = WAIT_SET_STATION_MODE_Resp;
            WiFi_SendCmd("AT+CWMODE_CUR=1\r\n",1000);
            break;
        case WAIT_CONNECT_AP:
            Wifi_Module_Uart_State = WAIT_CONNECT_AP_Resp;
            strcpy(AT_CMD_Buf,"AT+CWJAP_CUR=\"");
            strcat(AT_CMD_Buf,Cfg_Data.WifiSSID);
            strcat(AT_CMD_Buf,"\",\"");
            strcat(AT_CMD_Buf,Cfg_Data.WifiPwd);
            strcat(AT_CMD_Buf,"\"\r\n");
            //WiFi_SendCmd("AT+CWJAP_DEF=\"zte\",\"12345678\"\r\n",10000);
            WiFi_SendCmd(AT_CMD_Buf,30000);
            break;
        case WAIT_CONNECT_TCP:
			if(t_halfsec - t_Last_Cmd > Delay_HalfSec_Between_Cmd)
			{
				Wifi_Module_Uart_State = WAIT_CONNECT_TCP_Resp;
                strcpy(AT_CMD_Buf,"AT+CIPSTART=\"TCP\",\"");
                strcat(AT_CMD_Buf,Cfg_Data.MQTTServerDomain);
                strcat(AT_CMD_Buf,"\",");
                strcat(AT_CMD_Buf,Cfg_Data.MQTTServerPort);
				//strcat(AT_CMD_Buf,",12");  //keep alive
                strcat(AT_CMD_Buf,"\r\n");
				//WiFi_SendCmd("AT+CIPSTART=\"TCP\",\"msgtest.haierbiomedical.com\",1777\r\n",10000);
                WiFi_SendCmd(AT_CMD_Buf,10000);
			}
            break;
        case WAIT_SET_TRANSPARENT:
            if(t_halfsec - t_Last_Cmd > Delay_HalfSec_Between_Cmd)
            {
                Wifi_Module_Uart_State = WAIT_SET_TRANSPARENT_Resp;
                WiFi_SendCmd("AT+CIPMODE=1\r\n",1000);
            }
            break;
        case WAIT_START_TRANSPARENT:
            if(t_halfsec - t_Last_Cmd > Delay_HalfSec_Between_Cmd)
            {
                Wifi_Module_Uart_State = WAIT_START_TRANSPARENT_Resp;
                WiFi_SendCmd("AT+CIPSEND\r\n",1000);
            }
            break;
            
        case WAIT_MQTT_CONNECT:
            Wifi_Module_Uart_State = WAIT_MQTT_CONNECT_Resp;
            MQTT_Conect();
            break;

        case WAIT_MQTT_PUBLISH:
            if(f_RePublish)
            {
                if(t_halfsec - t_Set_RePublish > 600)  //重发数据超过5分钟没有发送，就丢弃，不再重发
                {
                    f_RePublish = 0;
                    if(Published_Type == MQTT_ALARM_DATA)
                    {
                        First_Fault_Frame = 1;  //再发一次故障帧
                    }
                    else if(Published_Type == MQTT_STATE_DATA)
                    {
                        First_State_Frame = 1;  //再发一次状态帧
                    }
                }
            }
            if(!f_RePublish )  //不是重发，该发什么就发什么（事件、状态、故障数据有变化立即发送，基础数据、mqtt心跳和应用心跳定时发送）
            {   
                //1.判断事件、状态、故障数据是否有变化，有变化立即发送
                //2.判断是否到了发送两种心跳的时间
                gPubType = Judge_PubType();
                if(gPubType != MQTT_NULL_DATA)
                {
                    MQTT_Send_Data(gPubType,SIGN_FIRST_PUBLISH);

                    Published_Type = gPubType;
                    Wifi_Module_Uart_State = WAIT_MQTT_PUBLISH_Resp;  //等待响应
                }
            }
            else  //重发上次发送的消息
            {
                MQTT_Send_Data(Published_Type,SIGN_RE_PUBLISH);  
                t_Last_MQTT_Publish = t_halfsec;
                Wifi_Module_Uart_State = WAIT_MQTT_PUBLISH_Resp;  //等待响应
                Published_Type = Published_Type;
            }
            break;

        //配置流程
        case WAIT_CFGMODE_EXIT_TRANS:
            Wifi_Module_Uart_State = WAIT_CFGMODE_EXIT_TRANS_Resp;
            WiFi_SendCmd("+++",1000);
            break;

        case WAIT_CFGMODE_ENTER_RST:
            Wifi_Module_Uart_State = WAIT_CFGMODE_ENTER_RST_Resp;
            WiFi_SendCmd("AT+RST\r\n",1000);
            break;

        case WAIT_CFGMODE_START_AP:
            Wifi_Module_Uart_State = WAIT_CFGMODE_START_AP_Resp;
            WiFi_SendCmd("AT+CWMODE_CUR=2\r\n",1000);
            break;

        case WAIT_CFGMODE_SET_IP:
            Wifi_Module_Uart_State = WAIT_CFGMODE_SET_IP_Resp;
            WiFi_SendCmd("AT+CIPAP_CUR=\"192.168.100.1\",\"192.168.100.1\",\"255.255.255.0\"\r\n",1000);
            break;

        case WAIT_CFGMODE_SET_SSID:
            Wifi_Module_Uart_State = WAIT_CFGMODE_SET_SSID_Resp;
            WiFi_SendCmd("AT+CWSAP_CUR=\"Haier_IoT\",\"12345678\",1,3\r\n",1000);
            break;

        case WAIT_CFGMODE_SET_UDP:
            Wifi_Module_Uart_State = WAIT_CFGMODE_SET_UDP_Resp;
            WiFi_SendCmd("AT+CIPSTART=\"UDP\",\"192.168.100.100\",,8000,2\r\n",1000);
            break;

        case WAIT_CFGMODE_REC_DATA:
            Wifi_Module_Uart_State = WAIT_CFGMODE_REC_DATA_Resp;
            WiFi_SendCmd("AT\r\n",120000);    //超过120秒收不到udp数据，自动退出配置模式
            break;

        case WAIT_CFGMODE_SEND_ACK_1:
            Wifi_Module_Uart_State = WAIT_CFGMODE_SEND_ACK_1_Resp;
            if(!f_RecStr_Err)
                WiFi_SendCmd("AT+CIPSEND=10\r\n",1000);
            else
                WiFi_SendCmd("AT+CIPSEND=12\r\n",1000);
            break;

        case WAIT_CFGMODE_SEND_ACK_2:
            Wifi_Module_Uart_State = WAIT_CFGMODE_SEND_ACK_2_Resp;
            if(!f_RecStr_Err)
                WiFi_SendCmd("Config OK!\r\n",2000);
            else
                WiFi_SendCmd("String Error\r\n",2000);
            break;

        default:
            break;

    }

}

/*-------------------------------------------------*/
/*函数名：WiFi发送设置指令                         */
/*参  数：cmd：指令                                */
/*参  数：timeout：超时时间（100ms的倍数）         */
/*返回值：                */
/*-------------------------------------------------*/
void WiFi_SendCmd(char *cmd, int timeout)
{
    char Temp_Buf[255];
    memset(Temp_Buf,0,255);
    sprintf(Temp_Buf, "%s", cmd);
    WiFi_RxCounter = 0;						  //WiFi接收数据量变量清零
	memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE); //清空WiFi接收缓冲区				  
    SendStr((unsigned char *)Temp_Buf);     //发送指令
    f_Wait_Wifi_Module_Resp = 1;
    Timeout_Wifi_Module = timeout;

}




/*----------------------------------------------------------*/
/*函数名：连接服务器报文                                    */
/*参  数：无                                                */
/*返回值：无                                                */
/*----------------------------------------------------------*/
void MQTT_Conect(void)
{
    unsigned char Fixed_len,Variable_len,Payload_len;  //固定报头长度，可变报头长度，载荷长度
    unsigned char ClientID_len,Username_len,Password_len;  //客户端id长度,用户名长度，密码长度
    unsigned char temp_buff_1[255]; //临时缓冲区，构建报文用
	
    //ClientID_len = strlen("651f68d8ea2d46cb8f263391bec936af");
    //memset(Client_ID,0,sizeof(Client_ID));
    //memcpy(Client_ID,"651f68d8ea2d46cb8f263391bec936af",ClientID_len);
    strcpy(Client_ID,Cfg_Data.BECode);
    ClientID_len = strlen(Client_ID);

    Username_len =strlen(Cfg_Data.Username);
    //memset(Cfg_Data.Username,0,sizeof(Cfg_Data.Username));
    //memcpy(Cfg_Data.Username,"haierTest",Username_len);

    Password_len =strlen(Cfg_Data.Password);
    //memset(Password,0,sizeof(Password));
    //memcpy(Password,"haier",Password_len);

    Fixed_len = 2;		//连接报文中，固定报头长度=2
	Variable_len = 10;		//连接报文中，可变报头长度=10
	Payload_len = 2 + ClientID_len + 2 + Username_len + 2 + Password_len; //载荷长度

	temp_buff_1[0] = 0x10;					   //第1个字节 ：固定0x01
	temp_buff_1[1] = Variable_len + Payload_len; //第2个字节 ：可变报头+有效负荷的长度
	temp_buff_1[2] = 0x00;					   //第3个字节 ：固定0x00
	temp_buff_1[3] = 0x04;					   //第4个字节 ：固定0x04
	temp_buff_1[4] = 0x4D;					   //第5个字节 ：固定0x4D
	temp_buff_1[5] = 0x51;					   //第6个字节 ：固定0x51
	temp_buff_1[6] = 0x54;					   //第7个字节 ：固定0x54
	temp_buff_1[7] = 0x54;					   //第8个字节 ：固定0x54
	temp_buff_1[8] = 0x04;					   //第9个字节 ：固定0x04
	temp_buff_1[9] = 0xC2;					   //第10个字节：使能用户名和密码校验，不使用遗嘱，清理会话（Clean Session = 1)(离线期间订阅的主题消息在上线时收不到)
    //temp_buff_1[9] = 0xC0;
	temp_buff_1[10] = 0x00;					   //第11个字节：保活时间高字节 0x00
	temp_buff_1[11] = 0x64;					   //第12个字节：保活时间高字节 0x64   100s

	/*     CLIENT_ID      */
	temp_buff_1[12] = ClientID_len / 256;				//客户端ID长度高字节
	temp_buff_1[13] = ClientID_len % 256;				//客户端ID长度低字节
	memcpy(&temp_buff_1[14], Client_ID, ClientID_len); //复制过来客户端ID字串
	/*     用户名        */
	temp_buff_1[14 + ClientID_len] = Username_len / 256;			   //用户名长度高字节
	temp_buff_1[15 + ClientID_len] = Username_len % 256;			   //用户名长度低字节
	memcpy(&temp_buff_1[16 + ClientID_len], Cfg_Data.Username, Username_len); //复制过来用户名字串
	/*      密码        */
	temp_buff_1[16 + ClientID_len + Username_len] = Password_len / 256;			  //密码长度高字节
	temp_buff_1[17 + ClientID_len + Username_len] = Password_len % 256;			  //密码长度低字节
	memcpy(&temp_buff_1[18 + ClientID_len + Username_len], Cfg_Data.Password, Password_len); //复制过来密码字串

	WiFi_RxCounter = 0;						  //WiFi接收数据量变量清零
	memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE); //清空WiFi接收缓冲区			
	SendData(temp_buff_1, Fixed_len + Variable_len + Payload_len); //加入发送数据缓冲区

    f_Wait_Wifi_Module_Resp = 1;
    Timeout_Wifi_Module = 10000;   //10秒钟
}

/*----------------------------------------------------------*/
/*函数名：给服务器发送基础数据                                    */
/*参  数：Data_Type：数据类型       Sign_RePub:重发标志               */
/*返回值：无                                                */
/*----------------------------------------------------------*/
static void MQTT_Send_Data(enum MQTT_DATA_TYPE Data_Type,unsigned char Sign_RePub)
{

    static unsigned char temp_buff[255]; //临时缓冲区，构建报文用
    unsigned char Sum = 0;
    unsigned char i;
    static unsigned char len_base_data , len_state_data , len_fault_data , len_event_data , len_heart_data;

    if(Sign_RePub)  //重新发布标志
    {
        if(Data_Type !=MQTT_PING_DATA)
            temp_buff[0] = 0x3A; //DUP 置位
        goto MK_REPUBLISH;
    }
        
	
    len_base_data = sizeof(para_base);
    len_state_data = sizeof(para_state);
    len_fault_data = sizeof(para_fault);
    len_event_data = sizeof(para_event);
    len_heart_data = sizeof(para_heart);

    //固定报头 
	temp_buff[0] = 0x32;	//第1个字节 ：qos 0:0x30   qos 1:0x32   RETAIN = 0(服务器不保留最新报文)
    if(Data_Type == MQTT_BASE_DATA)
	    temp_buff[1] = 2 + 43 + 2 + 6 + 14 + len_base_data + 1 + 2;  //第2个字节 ：可变报头+有效负荷的长度 = "主题长度"（2）+主题长度+“帧ID”（2）+ 帧头length type cmd+头数据 + 数据+“累加和”（1）+ “帧尾”  
    else if(Data_Type == MQTT_STATE_DATA)
	    temp_buff[1] = 2 + 43 + 2 + 6 + 14 + len_state_data + 1 + 2;  //第2个字节 ：可变报头+有效负荷的长度 = "主题长度"（2）+主题长度+“帧ID”（2）+ 帧头length type cmd+头数据 + 数据+“累加和”（1）+ “帧尾”  
    else if(Data_Type == MQTT_ALARM_DATA)
	    temp_buff[1] = 2 + 43 + 2 + 6 + 14 + len_fault_data + 1 + 2;  //第2个字节 ：可变报头+有效负荷的长度 = "主题长度"（2）+主题长度+“帧ID”（2）+ 帧头length type cmd+头数据 + 数据+“累加和”（1）+ “帧尾”  
    else if(Data_Type == MQTT_EVENT_DATA)
	    temp_buff[1] = 2 + 43 + 2 + 6 + 14 + len_event_data + 1 + 2;  //第2个字节 ：可变报头+有效负荷的长度 = "主题长度"（2）+主题长度+“帧ID”（2）+ 帧头length type cmd+头数据 + 数据+“累加和”（1）+ “帧尾” 
    else if(Data_Type == MQTT_HEART_DATA)
	    temp_buff[1] = 2 + 43 + 2 + 6 + 14 + len_heart_data + 1 + 2;  //第2个字节 ：可变报头+有效负荷的长度 = "主题长度"（2）+主题长度+“帧ID”（2）+ 帧头length type cmd+头数据 + 数据+“累加和”（1）+ “帧尾” 

    //可变报头  
	temp_buff[2] = 0x00;					   
	temp_buff[3] = 43;	//第3,4个字节 ：主题长度
	memcpy(&temp_buff[4],"/",1)	;//第5个字节 ：主题第一个字符
    memcpy(&temp_buff[5],Cfg_Data.BECode,9);   //复制BE码前9位
    memcpy(&temp_buff[14],"/",1)	;//  斜杠
	memcpy(&temp_buff[15],Cfg_Data.BECode,20);   //复制BE码
    memcpy(&temp_buff[35],"/",1)	;//  斜杠
    memcpy(&temp_buff[36],"user",4)	;//  user
    memcpy(&temp_buff[40],"/",1)	;//  斜杠
    memcpy(&temp_buff[41],"update",6)	;//update
	
    temp_buff[47] = 0x00;					   //帧ID
    temp_buff[48] = 0x00;

    temp_buff[49] = 0x9E;					   //载荷 头
    temp_buff[50] = 0xBF;

    temp_buff[55] = 0x00;  //是否断点续传
    temp_buff[56] = 0xff;  //年
    temp_buff[57] = 0xff;  //月
    temp_buff[58] = 0xff;  //日
    temp_buff[59] = 0xff;  //时
    temp_buff[60] = 0xff;  //分
    temp_buff[61] = 0xff;  //秒
    temp_buff[62] = 0x00;  //4g信号强度
    temp_buff[63] = 0x00;  //4g信号强度
    temp_buff[64] = 0x00;  //wifi信号强度
    temp_buff[65] = 0x00;  //wifi信号强度
    temp_buff[66] = 0x1;   //信号状态：wifi
    temp_buff[67] = 0x0;  //预留
    temp_buff[68] = 0x0;  //预留
    switch (Data_Type)
    {
        case MQTT_BASE_DATA:
        {
            temp_buff[51] = 0;  //帧长高字节
            temp_buff[52] = 1 + 1 + 14 + len_base_data + 1; //帧长：type + cmd + data + checksum
            temp_buff[53] = 0x31;                           //type 相当于设备大类
            temp_buff[54] = 0x02;                           //cmd  基础数据
            memcpy(&temp_buff[69], &para_base, len_base_data);
        }
        break;

        case MQTT_STATE_DATA:
        {
            temp_buff[51] = 0;  //帧长高字节
            temp_buff[52] = 1 + 1 + 14 + len_state_data + 1; //帧长：type + cmd + data + checksum
            temp_buff[53] = 0x31;                           //type 相当于设备大类
            temp_buff[54] = 0x03;                           //cmd  状态数据
            memcpy(&temp_buff[69], &para_state, len_state_data);
        }
        break;

        case MQTT_ALARM_DATA:
        {
            temp_buff[51] = 0;  //帧长高字节
            temp_buff[52] = 1 + 1 + 14 + len_fault_data + 1; //帧长：type + cmd + data + checksum
            temp_buff[53] = 0x31;                           //type 相当于设备大类
            temp_buff[54] = 0x04;                           //cmd  报警数据
            memcpy(&temp_buff[69], &para_fault, len_fault_data);
        }
        break;

        case MQTT_EVENT_DATA:
        {
            temp_buff[51] = 0;  //帧长高字节
            temp_buff[52] = 1 + 1 + 14 + len_event_data + 1; //帧长：type + cmd + data + checksum
            temp_buff[53] = 0x31;                           //type 相当于设备大类
            temp_buff[54] = 0x05;                           //cmd  事件数据
            memcpy(&temp_buff[69], &para_event, len_event_data);
        }
        break;

        case MQTT_HEART_DATA:
        {
            temp_buff[51] = 0;  //帧长高字节
            temp_buff[52] = 1 + 1 + 14 + len_heart_data + 1; //帧长：type + cmd + data + checksum
            temp_buff[53] = 0x31;                           //type 相当于设备大类
            temp_buff[54] = 0x01;                           //cmd  心跳数据
            memcpy(&temp_buff[69], &para_heart, len_heart_data);
        }
        break;

        case MQTT_PING_DATA:
        {
            temp_buff[0] = 0xC0;
            temp_buff[1] = 0x00;
        }
        break;

        default:
            break;
    }
    for(i = 51;i< 52 + temp_buff[52];i++)
    {
        Sum += temp_buff[i];
    }
    temp_buff[52 + temp_buff[52] + 0] = Sum;  //累加和
    temp_buff[52 + temp_buff[52] + 1] = 0xEE;  //帧尾
    temp_buff[52 + temp_buff[52] + 2] = 0xBA;  //帧尾

    MK_REPUBLISH:

    WiFi_RxCounter = 0;						  //WiFi接收数据量变量清零
    memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE); //清空WiFi接收缓冲区			
    SendData(temp_buff, temp_buff[1] + 2); 

    f_Wait_Wifi_Module_Resp = 1;
    Timeout_Wifi_Module = 5000;   //响应超时时间5秒
}

/*解析WIfi模块配置数据*/
unsigned char Parse_CfgData(void)
{
    unsigned char Cnt;
	//char temp;
    //Cnt = sscanf(WiFi_RX_BUF, "%*[^']'%[^']',%*[^']'%[^']',%*[^']'%[^']',%*[^']'%[^']',%*[^']'%[^']',%*[^']'%[^']',%*[^']'%[^']'", WifiSSID,WifiPwd,MQTTServerDomain,MQTTServerPort,Username,Password,BECode);
	//Cnt = sscanf(WiFi_RX_BUF, "%*[^']'%50[^']',%*[^']'%50[^']',%*[^']'%100[^']',%*[^']'%10[^']',%*[^']'%50[^']',%*[^']'%50[^']',%*[^']'%30[^']'%*[^}]%c", Cfg_Data.WifiSSID,Cfg_Data.WifiPwd,Cfg_Data.MQTTServerDomain,Cfg_Data.MQTTServerPort,Cfg_Data.Username,Cfg_Data.Password,Cfg_Data.BECode,temp);
    Cnt = sscanf(WiFi_RX_BUF, "%*[^']'%50[^']','%50[^']','%100[^']','%10[^']','%50[^']','%50[^']','%30[^']'", Cfg_Data.WifiSSID,Cfg_Data.WifiPwd,Cfg_Data.MQTTServerDomain,Cfg_Data.MQTTServerPort,Cfg_Data.Username,Cfg_Data.Password,Cfg_Data.BECode);

    //if(strcmp(Cfg_Data.MQTTServerDomain,"msgtest.haierbiomedical.com") == 0)  //测试服务器，使用用户名“haierTest”，密码“haier",否则连不上
	if(strstr(Cfg_Data.MQTTServerDomain,"msgtest") != NULL)  //测试服务器，使用用户名“haierTest”，密码“haier",否则连不上
    {
        strcpy(Cfg_Data.Username,"haierTest");
        strcpy(Cfg_Data.Password,"haier");
    }

	if(Cnt == 7 )  //7项数据 + 结束符“}”
        return 0;
    else
        return 1;
}

/*判断该发布什么数据*/
enum MQTT_DATA_TYPE Judge_PubType(void)
{
    enum MQTT_DATA_TYPE PubType = MQTT_BASE_DATA;
    static uint8_t flag_iot_state , flag_iot_fault, flag_iot_event ;  //标记->与上次发送出去的数据有变化
    static uint8_t Event_Num;  //事件码

//    CopyDataToIOTStructure();
    flag_iot_state = memcmp(&para_state_last, &para_state, sizeof(Para_State_t));  //比较当前数据与上次发送的数据有没有变化，有变化就发送
	flag_iot_fault = memcmp(&para_fault_last, &para_fault, sizeof(Para_Fault_t));

    if (EVT_Queue_tail == EVT_Queue_head)
    {
        ;//printf("队列为空，元素无法出队列\n");
        flag_iot_event = 0;
    }
    else
    {
        flag_iot_event = 1;
        Event_Num = EVT_Queue[EVT_Queue_head];
        EVT_Queue_head = (EVT_Queue_head + 1) % EVT_QUEUE_SIZE;
    }
        

    //优先上传事件
    if (flag_iot_event ) //发生了事件
    {
        flag_iot_event = 0;
        PubType = MQTT_EVENT_DATA;
        para_event.flag_event[0] = Event_Num;
        para_event.flag_event[1] = 0;

        para_event.flag_event[2] = 0xff;
        para_event.flag_event[3] = 0xff;
        para_event.flag_event[4] = 0xff;
        para_event.flag_event[5] = 0xff;
        para_event.flag_event[6] = 0xff;
        para_event.flag_event[7] = 0xff;
        para_event.flag_event[8] = 0xff;
        para_event.flag_event[9] = 0xff;
        para_event.flag_event[10] = 0xff;
        para_event.flag_event[11] = 0xff;
        para_event.flag_event[12] = 0xff;
        para_event.flag_event[13] = 0xff;
        para_event.flag_event[14] = 0xff;
        para_event.flag_event[15] = 0xff;
        para_event.flag_event[16] = 0xff;
    }
    else if (flag_iot_fault || First_Fault_Frame) //和上一次不相同，发送故障
    {
        First_Fault_Frame = 0;
        PubType = MQTT_ALARM_DATA;
        memcpy(&para_fault_last, &para_fault, sizeof(Para_Fault_t));   //保存上一次数据
    }
    else if (flag_iot_state || First_State_Frame) //和上一次不相同，发送状态
    {
        First_State_Frame = 0;
        PubType = MQTT_STATE_DATA;
        memcpy(&para_state_last, &para_state, sizeof(Para_State_t));   //保存上一次数据
    }
    else if((t_halfsec - t_Last_MQTT_BASE_Publish) >= MQTT_BASE_PUB_INTERVAL * 2) //基础数据  每隔x秒发一次
    {
        PubType = MQTT_BASE_DATA;
        t_Last_MQTT_BASE_Publish = t_halfsec;
    }
    else if((t_halfsec - t_Last_MQTT_HEART_Publish) >= MQTT_HEART_PUB_INTERVAL * 2)  //心跳  每隔x秒发一次
    {
        PubType = MQTT_HEART_DATA;
        t_Last_MQTT_HEART_Publish = t_halfsec;
    }
    else if((t_halfsec - t_Last_MQTT_Publish) >= MQTT_PING_PUB_INTERVAL * 2)  //ping  空闲超过X秒发一次
    {
        PubType = MQTT_PING_DATA;
        t_Last_MQTT_Publish = t_halfsec;
    }
    else
    {
        PubType = MQTT_NULL_DATA;
    }


    return PubType;
}
