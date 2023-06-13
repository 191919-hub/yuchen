#include "lib_gpio.h"
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

#define AD_CH10  22 //0b00001010           //����A/Dͨ��  @20180920 cfj
#define nop  __nop

#define LED_DIO_SET_HIGH    GPIOA_SetBit(GPIO_Pin_28)
#define LED_DIO_SET_LOW     GPIOA_ResetBit(GPIO_Pin_28)

#define LED_CLK_SET_HIGH    GPIOA_SetBit(GPIO_Pin_27)
#define LED_CLK_SET_LOW     GPIOA_ResetBit(GPIO_Pin_27)

#define LED_STB_SET_HIGH    GPIOA_SetBit(GPIO_Pin_26)
#define LED_STB_SET_LOW     GPIOA_ResetBit(GPIO_Pin_26)

#define KEY_SW1      GPIO_ReadBit(GPIOA, GPIO_Pin_1)
#define KEY_SW2      GPIO_ReadBit(GPIOA, GPIO_Pin_0)
#define KEY_SW3      GPIO_ReadBit(GPIOB, GPIO_Pin_13)
#define KEY_SW4      GPIO_ReadBit(GPIOB, GPIO_Pin_12)
#define KEY_SW5      GPIO_ReadBit(GPIOB, GPIO_Pin_11)
#define KEY_SW6      GPIO_ReadBit(GPIOB, GPIO_Pin_10)
#define KEY_SW7      GPIO_ReadBit(GPIOB, GPIO_Pin_8)

#define C_SDA_SET_LOW   GPIOB_ResetBit(GPIO_Pin_5)
#define C_SDA_SET_HIGH  GPIOB_SetBit(GPIO_Pin_5)

#define C_SCL_SET_LOW   GPIOB_ResetBit(GPIO_Pin_4)
#define C_SCL_SET_HIGH  GPIOB_SetBit(GPIO_Pin_4)

#define C_SDA_OUT       GPIOB_SetDirection(GPIO_Pin_5,GPIO_Dir_Out)
#define C_SDA_IN        GPIOB_SetDirection(GPIO_Pin_5,GPIO_Dir_In)

#define C_SDA_STATUS    GPIO_ReadBit(GPIOB, GPIO_Pin_5)

#define LED_ONFLAG     GPIO_ReadBit(GPIOB, GPIO_Pin_7) //�����ƿ���
#define DOOR_ONFLAG    GPIO_ReadBit(GPIOB, GPIO_Pin_6) //����Ŵ�

#define LED_OUT_SET_HIGH    GPIOB_SetBit(GPIO_Pin_9)
#define LED_OUT_SET_LOW     GPIOB_ResetBit(GPIO_Pin_9)

#define RS_485_STATUS_SET_HIGH  GPIOA_SetBit(GPIO_Pin_12)
#define RS_485_STATUS_SET_LOW   GPIOA_ResetBit(GPIO_Pin_12)


#define in 0
#define out 1


/****************Constant define*****************/
#define KEY_UNLOCK       0x02//0b00000010             // key2 ������ 
#define KEY_SET          0x01//0b00000001             // key1 ���ü�  
#define KEY_DISABLE_BUZZ  0x20//0b00100000             // ������
#define KEY_ADJUST       0x04//0b00000100             // ������
#define KEY_TEST_ALARM   0x10//0b00010000             // ���Ա�����
#define KEY_DEFROST      0x40//0b01000000             // ��˪��
#define KEY_SWITCH       0x08//0b00001000             // �л���
#define KEY_DELAY        0x05//0b00000101             // ѹ����ʱ��
#define KEY_WDJZ         0x03//0b00000011             // �¶ȵ�����
#define KEY_BJYCSJ       0x21//0b00100001             // �����ӳ�ʱ��
#define KEY_DATE         0x07//0b00000111             // �������ü�
#define KEY_UsbCheck     0x09//0b00001001             // usb��ѯ״̬

#define SET_NC_LD       0//�䶳����ǰ����˸����
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
#define LD_CHK_VOL      12                   //�䶳����ʾ��ѹ
#define LD_CHK_HW       13                   //�䶳����ʾ����
#define SET_LD_BJYCSJ   14

#define SET_NC_LC       15
#define SET_LC_L0       16
#define SET_LC          17			//�����ʾ����
#define SET_LC_WDJZ     18
#define SET_LC_HIGH     19
#define SET_LC_LOW      20
#define SET_LC_L1       21
#define SET_LC_L2       22
#define SET_LC_L3       23
#define SET_LC_L4       24
#define SET_LC_L5       25
#define SET_LCYJ        26
#define LC_CHK_VOL      27                    //�������ʾ��ѹ
#define LC_CHK_HW       28                    //�������ʾ����
#define SET_LC_BJYCSJ   29
#define SET_LCYJ_DELAY  30                    //�������ʾ ���ѹ���ӳ�ʱ��

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

#define DISP_NO 10//����ʾ
#define DISP_FH 11//����
#define DISP_F  12
#define DISP_L  13
#define DISP_H  14
#define DISP_U  15
#define DISP_E  16
#define DISP_C  17
#define DISP_P  18
#define DISP_D  19
#define DISP_ZH 20//����

//#define DISP_U  21     //zyj  100621
#define DISP_b  22     
#define DISP_o  23
#define DISP_k  24     
#define DISP_Dot  25 //zyj  100621
#define DISP_N  26 //zyj  100621


#define LOCK_LED_ON    0x80//0b10000000//�����Ƶ�����
#define LOCK_LED_OFF   0x7F//0b01111111
#define ALARM_LED_ON   0x04//0b00000100//�����Ƶ�����
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
extern unsigned char r_key;//ɨ���ʱ�������һ���ֽڵİ���ֵ
extern unsigned char r_keyz;//�а������µ�ʱ��ļ�¼�İ���ֵ
extern unsigned char r_sfkeyz;//�洢������ֵ
extern uint32_t t_write_e2;//�ϴ�дE2ʱ��
extern unsigned char r_lcad_8b;
extern unsigned char r_set_state;//����ʱ��ʾ��������
extern unsigned char r_flash_bit;//�����λ��

extern unsigned int  r_lcad_12b;
extern unsigned char Eheatcontrolpin;
extern unsigned char t_rec;   //ͨѶ
extern unsigned char r_rbit;
extern unsigned char r_rbyte;
extern unsigned char t_send;
extern unsigned char r_bit;
extern unsigned char r_byte;
extern unsigned char r_send_byte;  //Ҫ���͵��ֽ�����
extern unsigned char send[12];
extern unsigned char rec[18];

extern unsigned char r_buzz;//�����������еĲ���,����5������;0,2,4��100ms;1,3ͣ50ms
extern unsigned char t_buzz;//��������ʱ��ʱ
extern unsigned char u8_test_alarm_count;
extern unsigned char u8_test_alarm_time;

extern unsigned char r_lczt;//����趨�¶�
extern unsigned char r_lcztCopy;
extern unsigned char r_lcztx;
extern unsigned char r_ldzt;//�䶳�趨�¶�,��ƫ200
extern unsigned char r_ldztCopy;
extern unsigned char r_swtj;//�趨ʱ�ĸ�λ��
extern unsigned char r_gwtj;//�趨ʱ�ĵ�λ��
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

#define f_05s myflag1.Bits.b0 //0.5s�ı�־
#define f_halfsec myflag1.Bits.b1//0.5s�ı�־
#define f_1min myflag1.Bits.b2
#define f_onemin myflag1.Bits.b3//1min��־
#define f_10s myflag1.Bits.b4  //
#define f_tens myflag1.Bits.b5   //
#define f_com_ok myflag1.Bits.b6
#define f_first_ad myflag1.Bits.b7//��һ�β�ADǰ=0
/****************SIGNKEY***************/

extern BitType myflag2;
#define flag_signkey myflag2.Byte

#define f_key_kp	myflag2.Bits.b0//������Ч��־
#define f_key_km	myflag2.Bits.b1//����ȥ����־ 
#define f_bi		myflag2.Bits.b2	//��һ���ı�־
#define f_bibibi	myflag2.Bits.b3	//�����б�־
#define f_e2prom	myflag2.Bits.b4 //������дE2��־
#define f_ld_first_disp myflag2.Bits.b5//�䶳��һ����ʾ�ʹ��������Ϻ�,����ֱ����ʾ��ʵ�¶�,û�н�������
#define f_key_defrost   myflag2.Bits.b6          //
#define f_test_alarm    myflag2.Bits.b7       //
/****************SIGNKEY1**************/

extern BitType myflag3;
#define flag_signkey1 myflag3.Byte

#define f_zf myflag3.Bits.b0  //����ʱ�������� 0���ţ�1����
#define f_tx_led myflag3.Bits.b1
#define f_stop_alarm myflag3.Bits.b2 //=1ֹͣ���б�־
#define f_need_write_e2 myflag3.Bits.b3//��ҪдE2��־
#define f_lock myflag3.Bits.b4
#define f_bp_over myflag3.Bits.b5
#define f_bp myflag3.Bits.b6 //
#define f_txbuzz myflag3.Bits.b7
/***************ͨѶ���õ��ı�־********************/

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

#define f_lc_compressor        myflag5.Bits.b0 //���ѹ����������־
#define f_compressor           myflag5.Bits.b1
#define f_lc_fan               myflag5.Bits.b2
#define f_ln_fan               myflag5.Bits.b3//�䶳�������
#define f_defrost              myflag5.Bits.b4
#define f_nd_fan               myflag5.Bits.b5//�ڵ��������
#define f_long_range_alarm     myflag5.Bits.b6//Զ�̱���
#define f_buzz_alarm           myflag5.Bits.b7//������������־,�����ذ巢
/****************LRTBH***************/

extern BitType myflag6;
#define flag_err2 myflag6.Byte

#define f_voltage_err     myflag6.Bits.b0
#define f_power_off       myflag6.Bits.b1//����
#define f_battery         myflag6.Bits.b2
#define f_lc_sensor_err   myflag6.Bits.b3
#define f_lc_high         myflag6.Bits.b4 //��ظ��±�����־
#define f_lc_low          myflag6.Bits.b5  //
#define f_door_open       myflag6.Bits.b6 /////���ű�־���ڵ�wys11.03.19
#define f_dooropen_buzz	  myflag6.Bits.b7 /////����FMQ������־wys11.03.19
/****************BZJCQ***************/
extern BitType myflag7;
#define flag_err1 myflag7.Byte

#define f_ld_sensor_err		myflag7.Bits.b0//�䶳����������  
#define f_hw_sensor_err		myflag7.Bits.b1 
//#define f_pq_sensor_err	myflag7.Bits.b2
//#define f_pq_high       	myflag7.Bits.b3
//#define f_door_switch   	myflag7.Bits.b4
#define f_hw_high38     	myflag7.Bits.b5//���³�38�ȱ�־
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
#define f_lc_first_disp       myflag9.Bits.b2//��ص�һ����ʾ�ʹ��������Ϻ�,����ֱ����ʾ��ʵ�¶�,û�н�������
#define f_on_off_dly          myflag9.Bits.b3
#define f_stop_net_alarm      myflag9.Bits.b4
#define f_ack                 myflag9.Bits.b5
#define f_valve_dly           myflag9.Bits.b6
#define f_ldCompressorProtect myflag9.Bits.b7  //
/****************GZZS***************/
extern BitType myflag10;
#define flag_rec1 myflag10.Byte//��Ч

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
#define flag_ajszyjkt    myflag11.Byte          //��������ѹ����ͣ

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
#define f_Powerupdelaylc     myflag14.Bits.b7//����ϵ��ӳ�ʱ���־
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
#define	F_StopResponse              myflag16.Bits.b4//ֹͣ��Ӧ��־
#define	F_FirstPowerResponse        myflag16.Bits.b5
#define f_LightStatusPin               myflag16.Bits.b6
//#define     myflag16.Bits.b7
/*************************************************/
extern BitType mylc_flag;
#define f_lc_on_off_dly         mylc_flag.Bits.b0
#define f_compressor_on_dly     mylc_flag.Bits.b1 //���ѹ�����ӳٿ���ʱ��
#define delay_10sec             mylc_flag.Bits.b2
#define f_lcCompressorProtect   mylc_flag.Bits.b3 //�ۼ�xСʱ��ͣ����˪
#define f_lcComp1HourProtect    mylc_flag.Bits.b4//���ѹ�����������б���
#define f_door_switch           mylc_flag.Bits.b5
#define f_2Ms_Flag              mylc_flag.Bits.b6
#define f_First_PowerOnFlag     mylc_flag.Bits.b7  //�״��ϵ��־ @20181120 CFJ

extern BitType  g_PannelCommflag1; 

#define bUartSendStartFlag               g_PannelCommflag1.Bits.b0 
#define g_Pannel_Comm_bRcv_Done          g_PannelCommflag1.Bits.b1
#define g_Pannel_Compressor_Frequency    g_PannelCommflag1.Bits.b2
#define g_Compressor_Frequency_Flag1     g_PannelCommflag1.Bits.b3   //ѹ��ѡ���־
#define g_Compressor_Frequency_Flag2     g_PannelCommflag1.Bits.b4   //ѹ��ѡ���־
#define g_Rec_Status_Flag                g_PannelCommflag1.Bits.b5   //����״̬
#define bUartReceiveStartFlag            g_PannelCommflag1.Bits.b6   //���ܿ�ʼ
#define g_Sys_ReceiveDataType            g_PannelCommflag1.Bits.b7   //��������

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
extern unsigned char t_onems;//1ms��1
extern unsigned char t_twoms;
extern unsigned char t_20ms;//20ms��1
extern unsigned char t_j20ms;
extern unsigned char t_10s;
extern unsigned char t_tens;//10s+1
extern uint32_t t_auto_lock;
extern unsigned char t_data_to_led;

extern unsigned char r_hwxswd;
extern unsigned char r_hwsjwd;//�����¶���ʾֵ,��ƫ38
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
extern unsigned char r_hwadtemp;//�����ϴε��¶�
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

extern unsigned char u8_ld_bjycsj;//�䶳�����ӳ�ʱ��,Сʱ
extern unsigned char u8_ld_bjycsjx;
extern unsigned char u8_lc_bjycsj;//��ر����ӳ�ʱ��,Сʱ
extern unsigned char u8_lc_bjycsjx;
extern unsigned char u8_BuzzDlyMinld;
extern unsigned char u8_BuzzDlyHourld;
extern unsigned char u8_BuzzDlyMinlc;
extern unsigned char u8_BuzzDlyHourlc;//����ϵ��ӳٱ���ʱ�������Ԫ
extern unsigned char r_ldgzwd;//�䶳�����¶�,ʵ��Ҳ����ʵ�¶�
extern unsigned char r_ldxswd;//�䶳��ʾ�¶�,��ƫ200
extern unsigned int  r16_ldad;
extern unsigned char r_ldsjwd;//ADת����ʵ���¶�
extern unsigned char r_voltage_ad;
extern unsigned char r_voltage;
extern unsigned char r_voltageCopy;
extern unsigned char r_lcgzwd;
extern  uint32_t t_lc_rule;
extern unsigned char t_yj_delay;	//ѹ�����ӳ�ʱ��
extern unsigned char t_yj_delayx;
extern unsigned char t_yj_dly_time;
extern uint32_t t_ld_rule;//�����䶳��ʾ�����һ����ʱ��Ԫ
extern unsigned char r_ldwdjz;	//�䶳�¶�У׼,��ƫ10
extern unsigned char r_ldwdjzx;//�䶳У׼��ֵ,��ƫ10
extern unsigned char r_ld_high_alarm;//�䶳���±����趨�Ĳ�ֵ
extern unsigned char r_ld_high_alarmCopy;
extern unsigned char r_ld_low_alarm;//�䶳���±����趨�Ĳ�ֵ
extern unsigned char r_ld_low_alarmCopy;
extern unsigned char r_lc_high_alarm;//��ظ��±����趨�Ĳ�ֵ
extern unsigned char r_lc_high_alarmCopy;
extern unsigned char r_lc_low_alarm; //��ص��±����趨�Ĳ�ֵ
extern unsigned char r_lc_low_alarmCopy;
extern unsigned char t_compressor;
extern uint32_t t_ld_err_disp;
extern unsigned char t_ld_high_buzz;
extern unsigned char t_ld_low_buzz;
extern uint32_t t_voltage_buzz;
extern uint32_t t_power_buzz;
extern uint32_t t_power_Off; //@20181226 CFJ �����ʱ��
extern unsigned char t_stop_alarm;
extern unsigned char r_battery_ad;
extern unsigned char r_lcwdjz;//����¶�У׼ֵ,����10 ��ƫ��100
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

extern uchar    r_voltage_report;//��Ч
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
extern uchar t_door;/////wys11.03.19���ż�ʱ

extern uchar    ACKok;
extern uchar f8_lcCompAddUp6HourProtect;               // 140819 6Сʱ��־ 
extern uint  u16_lcComp_Addup_Minute;                  // �ۼ�ʱ��
extern uchar u8_lcCompAddUp6HourProtectTimer;          // ����ʱ�� 
extern uchar f8_lcCompAddUp6HourTimer;                 // 141017 ���6Сʱʱ�䵽 

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
extern unsigned char  Pannel_Uart1Buf[27];		// ���ڻ�����
extern unsigned char  Pannel_Uart1Data[27];		// ���ڻ�����
extern unsigned char  g_UART1_TXD_Counter;
extern unsigned char  g_UART1_TXD_Step;
//@20181130 CFJ �����ӵ� ��ӡ��
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
