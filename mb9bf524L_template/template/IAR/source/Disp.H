//#pragma CODE_SEG DEFAULT @20180920 cfj
#include "includes.h"
#include "Typedefine.h"
#include "Coupler.h"
extern const unsigned char table_lc[];
extern const float table_lc_12b[19][2];

/************************************************************/
extern const unsigned char table_led[];
/************************************************************/
/************************************************************/
extern const unsigned short tab_on[];
/********************************************/
extern const unsigned short tab_off[];
/************************************************************/
extern const unsigned char tab_voltage[];
/************************************************************/
extern  const	unsigned char tab_temperature[];

/************************************************************/
/************************************************************/
//#define nop __NOP //asm nop __NOP  @20181008 CFJ

#define AD_CH10  22 //0b00001010           //环温A/D通道  @20180920 cfj
#define nop asm("nop")
/***********I/O define***************************/
//#define io_ds1302clk  PTCD_PTCD2  
//#define io_ds1302rst  PTCD_PTCD5      
//#define io_ds1302dat  PTCD_PTCD3   

#define LED_DIO   bFM_GPIO_PDOR6_P0   //PTFD_PTFD6   @20180920 cfj
#define LED_CLK   bFM_GPIO_PDOR6_P1  //PTFD_PTFD5    @20180920 cfj
#define LED_STB   bFM_GPIO_PDOR6_P2  //PTFD_PTFD7    @20180920 cfj

#define KEY_SW1      bFM_GPIO_PDIR4_P7    //PTGD_PTGD3     @20180920 cfj
#define KEY_SW2      bFM_GPIO_PDIR4_P6    //PTGD_PTGD2     @20180920 cfj
#define KEY_SW3      bFM_GPIO_PDIR3_PF   //PTDD_PTDD7
#define KEY_SW4      bFM_GPIO_PDIR3_PE    //PTDD_PTDD6
#define KEY_SW5      bFM_GPIO_PDIR3_PD  //PTDD_PTDD5
#define KEY_SW6      bFM_GPIO_PDIR3_PC   //PTDD_PTDD4
#define KEY_SW7      bFM_GPIO_PDIR3_PA   //PTDD_PTDD1

#define C_sda        bFM_GPIO_PDOR3_P2 //PTGD_PTGD1
#define C_scl        bFM_GPIO_PDOR3_P1 //PTGD_PTGD0
#define C_sdaddr     bFM_GPIO_DDR3_P2  //PTGDD_PTGDD1
#define C_sda1       bFM_GPIO_PDIR3_P2 //PTGD_PTGD1

#define LED_ONFLAG     bFM_GPIO_PDIR3_P9 //冷藏灯的照明
#define DOOR_ONFLAG    bFM_GPIO_PDIR3_P3 //冷藏门打开
#define LED_OUT        bFM_GPIO_PDOR3_PB //冷藏照明灯打开

#define RS_485_STATUS  bFM_GPIO_PDOR1_P0

/* 未用到 @20180920 cfj
#define C_sdaA     PTBD_PTBD4
#define C_sclA     PTBD_PTBD3
#define C_sdaddrA  PTBDD_PTBDD4

#define C_sdaB     PTBD_PTBD6
#define C_sclB     PTBD_PTBD5
#define C_sdaddrB  PTBDD_PTBDD6
*/


#define in 0
#define out 1

#define C_send  PTCD_PTCD0
#define C_rec   PTCD_PTCD1
#define C_bit 8      //发送每字节位数
#define C_byte 5     //发送字节总数
#define C_bp_byte 1  //发送蜂鸣器帧时发送的字节数
#define C_rbit 8     //接收每字节位数
#define C_rbyte 9   //接收字节总数
/****************Constant define*****************/
#define KEY_UNLOCK       0x02//0b00000010             // key2 解锁键 
#define KEY_SET          0x01//0b00000001             // key1 设置键  
#define KEY_DISABLE_BUZZ  0x20//0b00100000             // 消音键
#define KEY_ADJUST       0x04//0b00000100             // 调整键
#define KEY_TEST_ALARM   0x10//0b00010000             // 测试报警键
#define KEY_DEFROST      0x40//0b01000000             // 化霜键
#define KEY_SWITCH       0x08//0b00001000             // 切换键
#define KEY_DELAY        0x05//0b00000101             // 压机延时键
#define KEY_WDJZ         0x03//0b00000011             // 温度调整键
#define KEY_BJYCSJ       0x21//0b00100001             // 报警延迟时间
#define KEY_DATE         0x07//0b00000111             // 日期设置键
#define KEY_UsbCheck     0x09//0b00001001             // usb查询状态

#define SET_NC_LD       0//冷冻设置前不闪烁画面
#define SET_LD_L0       1
#define SET_LD          2
#define SET_LD_WDJZ     3
#define SET_LD_HIGH     4
#define SET_LD_LOW      5
#define SET_LD_L1       6
#define SET_LD_L2       7
#define SET_LD_L3       8
#define SET_LD_L4       9
#define SET_LD_L5       10
#define SET_LDYJ        11
#define LD_CHK_VOL      12                   //冷冻区显示电压
#define LD_CHK_HW       13                   //冷冻区显示环温
#define SET_LD_BJYCSJ   14

#define SET_NC_LC       15
#define SET_LC_L0       16
#define SET_LC          17			//冷藏显示画面
#define SET_LC_WDJZ     18
#define SET_LC_HIGH     19
#define SET_LC_LOW      20
#define SET_LC_L1       21
#define SET_LC_L2       22
#define SET_LC_L3       23
#define SET_LC_L4       24
#define SET_LC_L5       25
#define SET_LCYJ        26
#define LC_CHK_VOL      27                    //冷藏区显示电压
#define LC_CHK_HW       28                    //冷藏区显示环温
#define SET_LC_BJYCSJ   29
#define SET_LCYJ_DELAY  30                    //冷藏区显示 冷藏压机延迟时间

#define SET_NC          0
#define SET_F0          1
#define SET_F1          2 
#define USB_Insert_Or_PullOut  31

#define SET_YEAR        32
#define SET_MONTH       33  
#define SET_DAY         34
#define SET_HOUR        35
#define SET_MINUTE      36


#define BIT1  1
#define BIT2  2
#define BIT3  3
#define BIT4  4
#define BIT5  5
#define BIT6  6

#define DISP_NO 10//不显示
#define DISP_FH 11//负号
#define DISP_F  12
#define DISP_L  13
#define DISP_H  14
#define DISP_U  15
#define DISP_E  16
#define DISP_C  17
#define DISP_P  18
#define DISP_D  19
#define DISP_ZH 20//正号

//#define DISP_U  21     //zyj  100621
#define DISP_b  22     
#define DISP_o  23
#define DISP_k  24     
#define DISP_Dot  25 //zyj  100621
#define DISP_N  26 //zyj  100621


#define LOCK_LED_ON    0x80//0b10000000//锁定灯的数据
#define LOCK_LED_OFF   0x7F//0b01111111
#define ALARM_LED_ON   0x04//0b00000100//报警灯的数据
#define ALARM_LED_OFF  0xFB//0b11111011

#define V220 0
#define V110 1

#define TXIE SCC2_TCIE
#define ON 1
#define OFF 0
#define ZHENG_SIGN 0
#define FU_SIGN 1
#define LEFT 1
#define RIGHT 0

/*************************************************/
//#pragma DATA_SEG _DATA_ZEROPAGE @20180920 cfj


extern unsigned char t_key_dly;
extern uint32_t t_key3s;
extern unsigned char r_key;//扫描的时候调整到一个字节的按键值
extern unsigned char r_keyz;//有按键按下的时候的记录的按键值
extern unsigned char r_sfkeyz;//存储按键的值
extern uint32_t t_write_e2;//上次写E2时间
extern unsigned char r_lcad_8b;
extern unsigned char r_set_state;//调节时显示的屏编码
extern unsigned char r_flash_bit;//数码管位数

extern unsigned int  r_lcad_12b;
extern unsigned char Eheatcontrolpin;
extern unsigned char t_rec;   //通讯
extern unsigned char r_rbit;
extern unsigned char r_rbyte;
extern unsigned char t_send;
extern unsigned char r_bit;
extern unsigned char r_byte;
extern unsigned char r_send_byte;  //要发送的字节总数
extern unsigned char send[12];
extern unsigned char rec[18];

extern unsigned char r_buzz;//蜂鸣器连续叫的步骤,共计5个步骤;0,2,4叫100ms;1,3停50ms
extern unsigned char t_buzz;//蜂鸣器叫时计时
extern unsigned char u8_test_alarm_count;
extern unsigned char u8_test_alarm_time;

extern unsigned char r_lczt;//冷藏设定温度
extern unsigned char r_lcztCopy;
extern unsigned char r_lcztx;
extern unsigned char r_ldzt;//冷冻设定温度,上偏200
extern unsigned char r_ldztCopy;
extern unsigned char r_swtj;//设定时的高位数
extern unsigned char r_gwtj;//设定时的低位数
extern unsigned char r_poise_valve_set;
extern unsigned char t_fan2min;

extern unsigned char r_led11;
extern unsigned char r_led12;
extern unsigned char r_led13;
extern unsigned char r_led21;
extern unsigned char r_led22;
extern unsigned char r_led23;
extern unsigned char r_led34;

//#pragma DATA_SEG __SHORT_SEG MY_ZEROPAGE @20180920 cfj
/*************SIGN***********************/


extern BitType myflag1;
#define flag_sign myflag1.Byte

#define f_05s myflag1.Bits.b0 //0.5s的标志
#define f_halfsec myflag1.Bits.b1//0.5s的标志
#define f_1min myflag1.Bits.b2
#define f_onemin myflag1.Bits.b3//1min标志
#define f_10s myflag1.Bits.b4  //
#define f_tens myflag1.Bits.b5   //
#define f_com_ok myflag1.Bits.b6
#define f_first_ad myflag1.Bits.b7//第一次测AD前=0
/****************SIGNKEY***************/

extern BitType myflag2;
#define flag_signkey myflag2.Byte

#define f_key_kp	myflag2.Bits.b0//按键生效标志
#define f_key_km	myflag2.Bits.b1//按键去抖标志 
#define f_bi		myflag2.Bits.b2	//叫一声的标志
#define f_bibibi	myflag2.Bits.b3	//连续叫标志
#define f_e2prom	myflag2.Bits.b4 //真正的写E2标志
#define f_ld_first_disp myflag2.Bits.b5//冷冻第一次显示和传感器故障后,用于直接显示真实温度,没有渐变修正
#define f_key_defrost   myflag2.Bits.b6          //
#define f_test_alarm    myflag2.Bits.b7       //
/****************SIGNKEY1**************/

extern BitType myflag3;
#define flag_signkey1 myflag3.Byte

#define f_zf myflag3.Bits.b0  //调节时正负符号 0正号，1负号
#define f_tx_led myflag3.Bits.b1
#define f_stop_alarm myflag3.Bits.b2 //=1停止鸣叫标志
#define f_need_write_e2 myflag3.Bits.b3//需要写E2标志
#define f_lock myflag3.Bits.b4
#define f_bp_over myflag3.Bits.b5
#define f_bp myflag3.Bits.b6 //
#define f_txbuzz myflag3.Bits.b7
/***************通讯中用到的标志********************/

extern BitType myflag4;
#define f_ddp myflag4.Bits.b0
#define f_startrec myflag4.Bits.b1   
#define f_recdata myflag4.Bits.b2
#define f_newframe myflag4.Bits.b3
#define f_gdp myflag4.Bits.b4
#define f_senddata myflag4.Bits.b5
#define f_sendbit myflag4.Bits.b6
#define f_sbit myflag4.Bits.b7
/****************JQZT******************/

extern BitType myflag5;
#define flag_jqzt myflag5.Byte

#define f_lc_compressor        myflag5.Bits.b0 //冷藏压缩机开机标志
#define f_compressor           myflag5.Bits.b1
#define f_lc_fan               myflag5.Bits.b2
#define f_ln_fan               myflag5.Bits.b3//冷冻交流风机
#define f_defrost              myflag5.Bits.b4
#define f_nd_fan               myflag5.Bits.b5//内胆交流风机
#define f_long_range_alarm     myflag5.Bits.b6//远程报警
#define f_buzz_alarm           myflag5.Bits.b7//蜂鸣器报警标志,向主控板发
/****************LRTBH***************/

extern BitType myflag6;
#define flag_err2 myflag6.Byte

#define f_voltage_err     myflag6.Bits.b0
#define f_power_off       myflag6.Bits.b1//掉电
#define f_battery         myflag6.Bits.b2
#define f_lc_sensor_err   myflag6.Bits.b3
#define f_lc_high         myflag6.Bits.b4 //冷藏高温报警标志
#define f_lc_low          myflag6.Bits.b5  //
#define f_door_open       myflag6.Bits.b6 /////开门标志用于灯wys11.03.19
#define f_dooropen_buzz	  myflag6.Bits.b7 /////开门FMQ报警标志wys11.03.19
/****************BZJCQ***************/
extern BitType myflag7;
#define flag_err1 myflag7.Byte

#define f_ld_sensor_err		myflag7.Bits.b0//冷冻传感器故障  
#define f_hw_sensor_err		myflag7.Bits.b1 
//#define f_pq_sensor_err	myflag7.Bits.b2
//#define f_pq_high       	myflag7.Bits.b3
//#define f_door_switch   	myflag7.Bits.b4
#define f_hw_high38     	myflag7.Bits.b5//环温超38度标志
#define f_ld_high       	myflag7.Bits.b6
#define f_ld_low        	myflag7.Bits.b7
/****************STATUS***************/
extern BitType myflag8;
#define flag_net1 myflag8.Byte

#define f_send_1ff   myflag8.Bits.b0
#define f_send_2ff   myflag8.Bits.b1
#define f_send_data  myflag8.Bits.b2
#define f_send55     myflag8.Bits.b3
#define f_rec_over   myflag8.Bits.b4
#define f_rec_data   myflag8.Bits.b5
#define f_rec_1ff    myflag8.Bits.b6
#define f_rec_2ff    myflag8.Bits.b7
/****************STATUS1***************/
extern BitType myflag9;
#define flag_net2 myflag9.Byte

#define f_rec55               myflag9.Bits.b0
//#define f_compressor_dly    myflag9.Bits.b1
#define f_lc_first_disp       myflag9.Bits.b2//冷藏第一次显示和传感器故障后,用于直接显示真实温度,没有渐变修正
#define f_on_off_dly          myflag9.Bits.b3
#define f_stop_net_alarm      myflag9.Bits.b4
#define f_ack                 myflag9.Bits.b5
#define f_valve_dly           myflag9.Bits.b6
#define f_ldCompressorProtect myflag9.Bits.b7  //
/****************GZZS***************/
extern BitType myflag10;
#define flag_rec1 myflag10.Byte//无效

#define f_voltage_slect   myflag10.Bits.b0 
#define f_Self_first_ad   myflag10.Bits.b1 //@201902101 CFJ
#define f_nd_AcLowfan     myflag10.Bits.b2 //@20190221 CFJ
//#define f__alarm myflag10.Bits.b1
//#define f_ch3 myflag10.Bits.b2
//#define f_ch4 myflag10.Bits.b3
//#define f_ch5 myflag10.Bits.b4
//#define f_ch6 myflag10.Bits.b5
//#define f_ch7 myflag10.Bits.b6
//#define f_ch8 myflag10.Bits.b7
/****************E2***************/

extern BitType myflag11;
#define flag_ajszyjkt    myflag11.Byte          //按键设置压机开停

#define ldyj             myflag11.Bits.b0
#define lcyj             myflag11.Bits.b1  
#define report_enable    myflag11.Bits.b2  
#define report           myflag11.Bits.b3
#define report_time      myflag11.Bits.b4
#define report_ack       myflag11.Bits.b5
#define report_ack_time  myflag11.Bits.b6
/*************************************************/
extern BitType myflag12;
//#define flag_e2 myflag12.flag12

#define f_ld_high_disp        myflag12.Bits.b0 
#define f_ld_low_disp         myflag12.Bits.b1
#define f_voltage_err_disp    myflag12.Bits.b2
#define f_power_off_disp      myflag12.Bits.b3
#define f_hw_high_disp        myflag12.Bits.b4
#define f_ld_sensor_err_disp  myflag12.Bits.b5
#define f_pq_sensor_err_disp  myflag12.Bits.b6
#define f_hw_sensor_err_disp  myflag12.Bits.b7
/*************************************************/

extern BitType myflag13;
#define f_BatteryErrDisp      myflag13.Bits.b0
#define f_key_defrost_disp    myflag13.Bits.b1
#define f_lc_sensor_err_disp  myflag13.Bits.b2
#define f_lc_high_disp        myflag13.Bits.b3
#define f_lc_low_disp         myflag13.Bits.b4

#define f_voltage_buzz        myflag13.Bits.b5
#define f_power_buzz          myflag13.Bits.b6
#define f_lc_high_buzz        myflag13.Bits.b7
/*************************************************/
extern BitType myflag14;
#define f_lc_low_buzz        myflag14.Bits.b0
#define f_ld_high_buzz       myflag14.Bits.b1
#define f_ld_low_buzz        myflag14.Bits.b2
#define f_ldComp1HourProtect myflag14.Bits.b3
#define f_ld_DispErrs        myflag14.Bits.b4
#define f_lc_DispErrs        myflag14.Bits.b5
#define f_Powerupdelayld     myflag14.Bits.b6
#define f_Powerupdelaylc     myflag14.Bits.b7//冷藏上电延迟时间标志
/*************************************************/
/*************************************************/

extern BitType myflag15;
#define f_net_first_set_time        myflag15.Bits.b0
//#define f_Usb_Copy_Done             myflag15.Bits.b1
#define f_Usb_Copy_Start            myflag15.Bits.b2
//#define     myflag15.Bits.b3
//#define     myflag15.Bits.b4
//#define     myflag15.Bits.b5
//#define     myflag15.Bits.b6
//#define     myflag15.Bits.b7
/*************************************************/
extern BitType myflag16;

#define	F_NeedResponse              myflag16.Bits.b0
#define	F_ResponseFirstFrame        myflag16.Bits.b1
#define	F_ReceiveResponseAck        myflag16.Bits.b2
#define	F_FirstPowerRespinseDone    myflag16.Bits.b3
#define	F_StopResponse              myflag16.Bits.b4//停止响应标志
#define	F_FirstPowerResponse        myflag16.Bits.b5
#define f_LightStatusPin               myflag16.Bits.b6
//#define     myflag16.Bits.b7
/*************************************************/
extern BitType mylc_flag;
#define f_lc_on_off_dly         mylc_flag.Bits.b0
#define f_compressor_on_dly     mylc_flag.Bits.b1 //冷藏压缩机延迟开机时间
#define delay_10sec             mylc_flag.Bits.b2
#define f_lcCompressorProtect   mylc_flag.Bits.b3 //累计x小时，停机化霜
#define f_lcComp1HourProtect    mylc_flag.Bits.b4//冷藏压缩机连续运行保护
#define f_door_switch           mylc_flag.Bits.b5
#define f_2Ms_Flag              mylc_flag.Bits.b6
#define f_First_PowerOnFlag     mylc_flag.Bits.b7  //首次上电标志 @20181120 CFJ

extern BitType  g_PannelCommflag1; 

#define bUartSendStartFlag               g_PannelCommflag1.Bits.b0 
#define g_Pannel_Comm_bRcv_Done          g_PannelCommflag1.Bits.b1
#define g_Pannel_Compressor_Frequency    g_PannelCommflag1.Bits.b2
#define g_Compressor_Frequency_Flag1     g_PannelCommflag1.Bits.b3   //压机选择标志
#define g_Compressor_Frequency_Flag2     g_PannelCommflag1.Bits.b4   //压机选择标志
#define g_Rec_Status_Flag                g_PannelCommflag1.Bits.b5   //接受状态
#define bUartReceiveStartFlag            g_PannelCommflag1.Bits.b6   //接受开始
#define g_Sys_ReceiveDataType            g_PannelCommflag1.Bits.b7   //数据类型

extern BitType  g_CommErrflag1;
#define g_Sys_Erflag0_Comm               g_CommErrflag1.Bits.b0 
#define bSelfUartSendStartFlag           g_CommErrflag1.Bits.b1 
//#pragma DATA_SEG  default @20180920 cfj
/*************************************************/

extern unsigned char t_1ms;
extern unsigned int  t_1ms_2 ;
extern unsigned char t_2ms;
extern uint32_t t_halfsec;
extern unsigned char t_1min;
extern unsigned char t_onems;//1ms加1
extern unsigned char t_twoms;
extern unsigned char t_20ms;//20ms加1
extern unsigned char t_j20ms;
extern unsigned char t_10s;
extern unsigned char t_tens;//10s+1
extern uint32_t t_auto_lock;
extern unsigned char t_data_to_led;

extern unsigned char r_hwxswd;
extern unsigned char r_hwsjwd;//环境温度显示值,上偏38
extern unsigned char r_lcwd;
extern unsigned char u8_CompMin;
extern unsigned char u8_CompHour;
extern unsigned char u8_CompStopTime;
extern unsigned char r_lcsdwdx;
extern unsigned char r_ldsdwdx;
extern unsigned char r_lcxswd;
extern unsigned char r_ldxs;
extern unsigned char r_hwad;
extern unsigned char t_ad;
extern unsigned char r_hwadtemp;//环温上次的温度
extern unsigned char t_hwdisp;
extern unsigned char t_on_off_dly;
extern unsigned char t_valve_dly;
extern unsigned char r_hwhc;
extern  uint32_t t_lc_err_disp;
extern unsigned char t_lc_high_buzz;
extern unsigned char t_lc_low_buzz;
extern unsigned char u8_LdRule;
extern unsigned char u8_LcRule;
extern unsigned char u8_CompStop5min;
extern unsigned char u8_CompRunMin;

extern unsigned char u8_ld_bjycsj;//冷冻报警延迟时间,小时
extern unsigned char u8_ld_bjycsjx;
extern unsigned char u8_lc_bjycsj;//冷藏报警延迟时间,小时
extern unsigned char u8_lc_bjycsjx;
extern unsigned char u8_BuzzDlyMinld;
extern unsigned char u8_BuzzDlyHourld;
extern unsigned char u8_BuzzDlyMinlc;
extern unsigned char u8_BuzzDlyHourlc;//冷藏上电延迟报警时间计数单元
extern unsigned char r_ldgzwd;//冷冻修正温度,实际也是现实温度
extern unsigned char r_ldxswd;//冷冻显示温度,上偏200
extern unsigned int  r16_ldad;
extern unsigned char r_ldsjwd;//AD转换的实际温度
extern unsigned char r_voltage_ad;
extern unsigned char r_voltage;
extern unsigned char r_voltageCopy;
extern unsigned char r_lcgzwd;
extern  uint32_t t_lc_rule;
extern unsigned char t_yj_delay;	//压缩机延迟时间
extern unsigned char t_yj_delayx;
extern unsigned char t_yj_dly_time;
extern uint32_t t_ld_rule;//用于冷冻显示规则的一个计时单元
extern unsigned char r_ldwdjz;	//冷冻温度校准,上偏10
extern unsigned char r_ldwdjzx;//冷冻校准差值,上偏10
extern unsigned char r_ld_high_alarm;//冷冻高温报警设定的差值
extern unsigned char r_ld_high_alarmCopy;
extern unsigned char r_ld_low_alarm;//冷冻低温报警设定的差值
extern unsigned char r_ld_low_alarmCopy;
extern unsigned char r_lc_high_alarm;//冷藏高温报警设定的差值
extern unsigned char r_lc_high_alarmCopy;
extern unsigned char r_lc_low_alarm; //冷藏低温报警设定的差值
extern unsigned char r_lc_low_alarmCopy;
extern unsigned char t_compressor;
extern uint32_t t_ld_err_disp;
extern unsigned char t_ld_high_buzz;
extern unsigned char t_ld_low_buzz;
extern uint32_t t_voltage_buzz;
extern uint32_t t_power_buzz;
extern uint32_t t_power_Off; //@20181226 CFJ 掉电的时间
extern unsigned char t_stop_alarm;
extern unsigned char r_battery_ad;
extern unsigned char r_lcwdjz;//冷藏温度校准值,乘以10 再偏移100
extern unsigned char r_lcwdjzx;

extern unsigned char flag_err1_copy;
extern unsigned char flag_err2_copy;
extern unsigned char r_sendr;
extern unsigned char r_sendsum;
extern unsigned char send_net[50];
extern unsigned char t_net_rec;
extern unsigned char rec_net[25];
extern unsigned char r_receiver;
extern unsigned char r_recsum;
extern unsigned char r_rec55sum;
extern unsigned char t_err200ms;
extern uint32_t t_ack5s;
extern unsigned char t_lightms;
extern unsigned char t_lc_on_off_dly;
extern unsigned char t_lc_compressor;
extern unsigned char t_nd_fan;
extern unsigned char r_set_state1;
extern unsigned char t_yj_delay_10sec;

extern unsigned char u8_lcCompMin;
extern unsigned char u8_lcCompHour;
extern unsigned int u8_lcCompRunMin;
extern unsigned char u8_lcCompStopTime;
extern unsigned char u8_lcCompStop5min;

extern uchar u8_Send_Print_Time;

extern uchar    u8_year;
extern uchar    u8_month;
extern uchar    u8_day;
extern uchar    u8_hour;
extern uchar    u8_minute;

extern uchar    u8_year_copy;
extern uchar    u8_month_copy;
extern uchar    u8_day_copy;
extern uchar    u8_hour_copy;
extern uchar    u8_minute_copy;

extern uchar    u8_year_set;
extern uchar    u8_month_set;
extern uchar    u8_day_set;
extern uchar    u8_hour_set;
extern uchar    u8_minute_set;

extern uchar    r1;
extern unsigned int u16_random;
extern unsigned int u16_random_copy;
extern unsigned int u16_random_copyTemp;
extern unsigned int u16_random_errcopy;
extern unsigned int u16_random_time;

extern uchar R_VoidFrameCode;  //zyj 100504
extern uchar R_HomeNetCounter;

extern uchar    r_voltage_report;//无效
extern uchar    r_lczt_report;
extern uchar    r_ldzt_report;
extern uchar    lc_high_alarm_report;
extern uchar    lc_low_alarm_report;
extern uchar    ld_high_alarm_report;
extern uchar    ld_low_alarm_report;
extern uchar    t_voltage;
extern uchar    t_report;
extern uchar    t_report_time;

extern uchar    u8_UsbState;

extern uchar R_NetTimeBase1ms;

extern uchar R_HomeNetResponseTimeSecond;    //100416  zyj
extern uint  R_HomeNetResponseCheckTime100ms;    //100504  zyj
extern uchar R_HomeNetResponseCheckTime_H8;
extern uchar R_HomeNetResponseCheckTime_L8;
extern uint  R_HomeNetResponseCheckTime;
extern uchar t_door;/////wys11.03.19开门计时

extern uchar    ACKok;
extern uchar f8_lcCompAddUp6HourProtect;               // 140819 6小时标志 
extern uint  u16_lcComp_Addup_Minute;                  // 累计时间
extern uchar u8_lcCompAddUp6HourProtectTimer;          // 保护时间 
extern uchar f8_lcCompAddUp6HourTimer;                 // 141017 冷藏6小时时间到 

extern unsigned int  g_Txd_Time;
extern unsigned int  g_Bus_Error_Time;
extern unsigned char  g_UART1_RCV_Cyc;
extern unsigned char  g_UART1_RCV_Counter ;
extern unsigned char  g_IDM_TX_ODM_Data[30];
extern unsigned char  g_IDM_RX_ODM_Data[30];
extern unsigned char  g_Compressor_Frequency;
extern unsigned int   g_Bus_Rec_Time;
extern unsigned char  Uart1RxCnt;
extern unsigned char  Uart1RxSum;
//#define UART_BUF_LEN     24
//#define TX_UART_BUF_LEN   9
extern unsigned char  Pannel_Uart1Buf[27];		// 串口缓存区
extern unsigned char  Pannel_Uart1Data[27];		// 串口缓存区
extern unsigned char  g_UART1_TXD_Counter;
void Delay(uint data);
extern unsigned char  g_UART1_TXD_Step;
//@20181130 CFJ 后续加的 打印机
extern unsigned char  u8_Rec2data[20]; 
extern unsigned char  u8_Send2data[20]; 
extern unsigned char  u8_Send2dataMemory[20];

extern unsigned char  u8_Rec2_data_Num; 
extern unsigned char  u8_Send2_data_Num; 
extern unsigned char  u8_Send2_data_Num_Count; 
extern unsigned char  u8_Rec2_data_State; 
extern unsigned char  u8_Send2_data_State;
extern unsigned char  SelfCheckFlag;
extern unsigned char  t_Self_Times; //@20190215 CFJ
extern unsigned char  SelfCheckNoErrFlag; //@20190221 CFJ
/************************************************************/

