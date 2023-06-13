#ifndef __ESP8266_H__
#define __ESP8266_H__

#define MQTT_BASE_PUB_INTERVAL   30   //基础数据发送间隔 单位：秒
#define MQTT_HEART_PUB_INTERVAL  60   //心跳数据发送间隔 单位：秒
#define MQTT_PING_PUB_INTERVAL   5   //PINGREQ发送间隔 单位：秒


//#define WiFi_RxCounter    uart5.uart_rx_count    //
//#define WiFi_RX_BUF       uart5.uart_rxbuf
#define WiFi_RXBUFF_SIZE  1024

#define  WAIT_EXIT_TRANS              0  //确保模块退出透传
#define  WAIT_RESET                   1  //上电之后首先复位wifi模块
#define  WAIT_PWRON                   2  //发送at确认模块开始工作
#define  WAIT_SET_STATION_MODE        3  //设置station模式
#define  WAIT_CONNECT_AP              4  //连接Wifi
#define  WAIT_CONNECT_TCP             5  //连接服务器
#define  WAIT_SET_TRANSPARENT         6  //设置透明传输
#define  WAIT_START_TRANSPARENT       7  //启动透明传输

#define  WAIT_MQTT_CONNECT            21  //mqtt连接
#define  WAIT_MQTT_PUBLISH           22  //mqtt连接完成，定期发送数据
#define  WAIT_MQTT_PING               23

#define  WAIT_CFGMODE_EXIT_TRANS        50  //进入配置模式前先退出透传模式
#define  WAIT_CFGMODE_ENTER_RST         51  //配置之前先复位一下
#define  WAIT_CFGMODE_START_AP          52  //开启AP
#define  WAIT_CFGMODE_SET_IP            53  //设置AP的ip
#define  WAIT_CFGMODE_SET_SSID          54  //设置AP的名字，密码，信道，加密方式
#define  WAIT_CFGMODE_SET_UDP           55  //建立UDP传输
#define  WAIT_CFGMODE_REC_DATA          56  //等待接收app传过来的配置数据
#define  WAIT_CFGMODE_SEND_ACK_1        57  //接收配置数据后发送确认帧
#define  WAIT_CFGMODE_SEND_ACK_2        58  //接收配置数据后发送确认帧
#define  WAIT_CFGMODE_EXIT_RST          59  //配置完成之后再复位一下


#define  WAIT_EXIT_TRANS_Resp              WAIT_EXIT_TRANS+100
#define  WAIT_RESET_Resp                   WAIT_RESET+100
#define  WAIT_PWRON_Resp                   WAIT_PWRON+100
#define  WAIT_SET_STATION_MODE_Resp        WAIT_SET_STATION_MODE+100
#define  WAIT_CONNECT_AP_Resp              WAIT_CONNECT_AP+100
#define  WAIT_CONNECT_TCP_Resp             WAIT_CONNECT_TCP+100
#define  WAIT_SET_TRANSPARENT_Resp         WAIT_SET_TRANSPARENT+100
#define  WAIT_START_TRANSPARENT_Resp       WAIT_START_TRANSPARENT+100

#define  WAIT_MQTT_CONNECT_Resp            WAIT_MQTT_CONNECT+100
#define  WAIT_MQTT_PUBLISH_Resp            WAIT_MQTT_PUBLISH+100
#define  WAIT_MQTT_PING_Resp               WAIT_MQTT_PING+100

#define  WAIT_CFGMODE_EXIT_TRANS_Resp        WAIT_CFGMODE_EXIT_TRANS+100  //进入配置模式前先退出透传模式
#define  WAIT_CFGMODE_ENTER_RST_Resp         WAIT_CFGMODE_ENTER_RST+100  //配置之前先复位一下
#define  WAIT_CFGMODE_START_AP_Resp          WAIT_CFGMODE_START_AP+100  //开启AP
#define  WAIT_CFGMODE_SET_IP_Resp            WAIT_CFGMODE_SET_IP+100  //设置AP的ip
#define  WAIT_CFGMODE_SET_SSID_Resp          WAIT_CFGMODE_SET_SSID+100  //设置AP的名字，密码，信道，加密方式
#define  WAIT_CFGMODE_SET_UDP_Resp           WAIT_CFGMODE_SET_UDP+100  //建立UDP传输
#define  WAIT_CFGMODE_REC_DATA_Resp          WAIT_CFGMODE_REC_DATA+100  //等待接收app传过来的配置数据
#define  WAIT_CFGMODE_SEND_ACK_1_Resp        WAIT_CFGMODE_SEND_ACK_1+100  //接收配置数据后发送确认帧
#define  WAIT_CFGMODE_SEND_ACK_2_Resp        WAIT_CFGMODE_SEND_ACK_2+100  //接收配置数据后发送确认帧
#define  WAIT_CFGMODE_EXIT_RST_Resp          WAIT_CFGMODE_EXIT_RST+100  //配置完成之后再复位一下


#define SIGN_FIRST_PUBLISH 0
#define SIGN_RE_PUBLISH    1

typedef struct 
{
    char WifiSSID[51];  //wifi 名称
    char WifiPwd[51];   //wifi密码
    char MQTTServerDomain[101];
    char MQTTServerPort[11];
    char Username[51];  //mqtt用户名
    char Password[51];  //mqtt密码
    char BECode[31];
}Wifi_Cfg_t;


enum MQTT_DATA_TYPE
{
    MQTT_NULL_DATA = 0,MQTT_HEART_DATA = 1, MQTT_BASE_DATA, MQTT_STATE_DATA, MQTT_ALARM_DATA, MQTT_EVENT_DATA, MQTT_PING_DATA
};

extern char f_Enter_WifiCfg ;
extern char f_Exit_WifiCfg;
extern Wifi_Cfg_t Cfg_Data;
extern char WiFi_RX_BUF[WiFi_RXBUFF_SIZE];       
extern volatile uint32_t WiFi_RxCounter ;
extern unsigned char f_disp_Wifi_Cfg;  //显示Wifi正在配置
extern char f_Wait_Wifi_Module_Resp ;  //等待wifi模块的响应
extern char f_No_WifiModule ;  //确认不存在wifi模块
extern uint8_t f_Exist_WifiModule ;  //确认存在wifi模块


extern int Timeout_Wifi_Module ;   //wifi模块响应倒计时，等于0代表超时
extern char f_Wait_Wifi_Module_Resp;  //等待wifi模块的响应
extern char Wifi_Module_Uart_State;
extern void Send_To_Wifi_Module(void);


void Deal_RecData_From_WifiModule(void);
void WiFi_SendCmd(char *cmd, int timeout);

void MQTT_Conect(void);
static void MQTT_Send_Data(enum MQTT_DATA_TYPE Data_Type,unsigned char Sign_RePub);
enum MQTT_DATA_TYPE Judge_PubType(void);

#endif