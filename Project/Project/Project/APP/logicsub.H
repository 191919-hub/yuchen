/*
 控制.h文件
*/
void ClearRam(void);
void Initial(void);
void Time(void);
void PowerUpBuzzDelaylc(void);
void PowerUpBuzzDelayld(void);
void BuzzPrg(void);
void BuzzBi(void);
void BuzzBiBiBi(void);                         
void ad_convert(void);
void DealRecData(void);
void KeyPress(void);
void LdCompressorJudge(void);
void LnFan(void);
void DealDispData(void);
void AlarmControl(void);
void JudgeErrs(void);
void DataToLed(void);
void RecData(void);
void NetRecOver(void);
void RecOkAction(void);
void NetErrAlarm(void);
void ReturnStateFrame(void);
void ReturnVoidFrame(void);
void GetSum(void);
void ReceiveInitial(void);
void ReturnAckFrame(void);
void ReturnIdentifycodeFrame(void);
void ReturnReportFrame(void);
void WriteToE2(void);
void WriteE2(void);
void ReadE2(void); 
void Lc_lightProg(void);
void Lc_CompressorJudge(void);
void Nd_fan_Prog(void); 
void Defrost_Prog(void);
void Compressor_on_delay(void);
void Compressor_delay_10sec(void);
void SetTimeFrame(void); 
void ResponseFirstPowerState(void);
void InitialUserRegister(void);
void NetResponseStateControl(void) ;
void Door_Open(void);////wys11.03.19
void User_GPIO_Init(void);//@20180926 cfj
void WdgHwCallback(void);//@20181016 CFJ
void Rec2Action(void);
void RuleForLcDisp_0D1(void) ;
//-----------------------面板通讯处理------------------------------//
void Pannel_Comm_Deal(void); //面板通讯处理 CFJ  
void l_ODM_Data_Parse(void);
void l_Pannel_DataTx(void);
void l_Send_Data_Build(void);
void l_ClrIDMSendBuf(void);
//----------------------------------------------------------------//
void l_Self_Detect(void); //自检测试程序
void Clear_Watchdog(void);  //看门狗 @20190121 CFJ
//extern uchar u8_Send_Print_Time;
void l_Self_Send_Data_Build(void);
void l_Pannel_Self_DataTx(void);
//---------------------------------------------------------------//
/*********************加热丝控制函数*********************************/
void  EheatConTrolP(void);

extern unsigned char f_door_state ;  //门状态，0：关，1：开
extern void WriteByte(unsigned char tem_dat);
extern unsigned char ReadByte(void);