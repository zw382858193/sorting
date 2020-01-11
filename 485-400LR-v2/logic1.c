/*
********************************************************************************
*                                嵌入式微系统
*                                   msOS
*
*                            硬件平台:msPLC DEMO
*                          主芯片:STM32F103R8T6/RBT6
*                           深圳市雨滴科技有限公司
*
*                                作者:王绍伟
*                                网名:凤舞天
*                                标识:Wangsw
*
*                                QQ:26033613
*                               QQ群:291235815
*                        论坛:http://bbs.huayusoft.com
*                        淘宝店:http://52edk.taobao.com
*                博客:http://forum.eet-cn.com/BLOG_wangsw317_1268.HTM
********************************************************************************
*文件名     : logic.c
*作用       : 业务逻辑处理文件
*原理       : 处理各类消息，如按键、传感器等，标准化一些按键的处理模式
********************************************************************************
*版本     作者            日期            说明
*V0.1    Wangsw        2013/07/21       初始版本
********************************************************************************
*/

#include "drive.h"
#include "system.h"
#include "app.h"

char BD_QUERY_ANTENNA_EXIST[]	=	{"cmd=5;type=5;num=0;"};
char BD_TTS_LOCKED[]			=	{0x63, 0x6d, 0x64, 0x3d, 0x36, 0x3b, 0x6d, 0x73, 0x67, 0x3d, 0x63, 0x62, 0x66, 0x38, 0x62, 0x33, 0x62, 0x35, 0x3b};
char BD_TTS_ANTENNA_MISS[]		=	{0x63, 0x6d, 0x64, 0x3d, 0x36, 0x3b, 0x6d, 0x73, 0x67, 0x3d, 0x62, 0x31, 0x62, 0x31, 0x62, 0x36, 0x62, 0x37, 0x63, 0x63, 0x65, 0x63, 0x63, 0x66, 0x64, 0x66, 0x64, 0x32, 0x65, 0x63, 0x62, 0x33, 0x61, 0x33, 0x3b};
char BD_TTS_START_ENGINE[]		=	{0x63, 0x6d, 0x64, 0x3d, 0x36, 0x3b, 0x6d, 0x73, 0x67, 0x3d, 0x63, 0x37, 0x65, 0x62, 0x63, 0x36, 0x66, 0x34, 0x62, 0x36, 0x61, 0x66, 0x62, 0x37, 0x61, 0x32, 0x62, 0x36, 0x61, 0x66, 0x62, 0x62, 0x66, 0x61, 0x3b};
char BD_TTS_FAIL_VERIFY[]		=	{0x63, 0x6d, 0x64, 0x3d, 0x36, 0x3b, 0x6d, 0x73, 0x67, 0x3d, 0x64, 0x36, 0x62, 0x38, 0x63, 0x65, 0x63, 0x36, 0x64, 0x31, 0x65, 0x39, 0x64, 0x36, 0x61, 0x34, 0x63, 0x61, 0x61, 0x37, 0x62, 0x30, 0x64, 0x63, 0x3b};
char BD_TTS_PLEASE_VERIFY[]		=	{0x63, 0x6d, 0x64, 0x3d, 0x36, 0x3b, 0x6d, 0x73, 0x67, 0x3d, 0x63, 0x37, 0x65, 0x62, 0x62, 0x30, 0x62, 0x34, 0x64, 0x36, 0x62, 0x38, 0x63, 0x65, 0x63, 0x36, 0x3b};

typedef void(* Function)(void);

#define SecondsInDay            86400       // 1天总共86400秒
#define DayInFourYear           1461        // 4年总共1461天，365*3+366
                                                                       
static const byte DaysInNonLeapMonthTable[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};  // 闰年的月份日期表
static const byte DaysInLeapMonthTable[12] =    {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};  // 平年的月份日期表

#define HeartBeatID 0x0CF00203
#define RpmLimitID  0x0C000003

static volatile byte WorkState = 0;
static volatile byte CheckCode = 0;
static volatile ushort DataSum;
#define MessageLenth    11
static byte TimeoutMessage[] = {0xAA, 0x01, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xEE,
                                    0x45, 0x54, 0xAA};  //一帧数据接收超时
static byte CodeErrMessage[] = {0xAA, 0x01, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xEE,
                                    0x45, 0x43, 0XBD};  //接收数据，校验码错误
static byte InstrErrMessage[] = {0xAA, 0x01, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xEE,
                                    0x45, 0x49, 0xB7}; //命令错误(有可能没有此命令)

static volatile byte HeartBeatState = off;
static volatile ushort CarSpeedLimit     = 0;                 //发送机转速限制值
static volatile uint CurrentCarSpeed   = 0;

static volatile bool CarStateLevel0Flag = false;
static volatile bool CarStateLevel1Flag = false;
static volatile bool CarStateLevel2Flag = false;
static volatile bool HeartBeatFlag = false;
void self_check(void);	// 自检流程控制
void RtcSystick100Routine(void);
static void DummyFunction(void)
{}

static void HeartBeatOnOff(byte state);
static void TimerSendStartupMessage(void);
//static void HeartBeatCtrl(void);
void sendtoT2_protocol808(unsigned char * data, int len, unsigned short id);
#define sendtoT2_protocol808_error_msg(data, len)	sendtoT2_protocol808(data, len, 0x9f1f)
volatile char Beidou_GPS_Antenna_Off = 100;	
volatile int Status_Self_Check = 1002; // 初值acc off

extern volatile char FingerprintMatch;
extern void Rev4DataSystick1000Routine(void);

#define ACC_ON 2
#define ACC_OFF 1
#define ACC_UNKNOWN 0

static int gAccState = ACC_UNKNOWN;
	
void trigHeartbeat();
	
static void Mode2HeartbeatCtrl(void)
{
    int i;
    uint currentSecond;
    uint year;
    uint month;
    uint day;
    uint second;
    
    if(App.Data.CarMode == 2)
    {
        //计算当前时间
        year = AppDataPointer->Rtc.Year - 2000;
        day = (year / 4) * DayInFourYear + (year % 4) * 365;
        month = AppDataPointer->Rtc.Month - 1;
        if (AppDataPointer->Rtc.Year % 4 == 0)
        {
            for (i = 0; i < month; i++)
                day = day + DaysInLeapMonthTable[i];
        }
        else
        {
            for (i = 0; i < month; i++)
                day = day + DaysInNonLeapMonthTable[i];
        }
        day = day + AppDataPointer->Rtc.Day - 1;
        currentSecond = day * SecondsInDay + AppDataPointer->Rtc.Hour * 3600 + AppDataPointer->Rtc.Minute * 60 + AppDataPointer->Rtc.Second;  //当前时间

        //计算上次打指纹的时效
        second = App.Data.FingerLogTime + App.Data.FingerprintTime * 60 * 60;       

        if(second > currentSecond)          //上次打指纹的时间还有效
        {
            App.Data.FingerprintStatus = true;
            HeartBeatOnOff(on);
        } 
    }    
}

static void StartToWork(void)
{
    System.Device.Misc.T2ResetCtrl(false);
    System.Device.Misc.AVPowerCtrl(0);
    System.Device.Usart1.Open();
    //System.Device.Usart2.Open();    
}

static void T2PowerOn(void)
{
//    //bool res;
    
    System.Device.Misc.AcinSwitch(true);
    System.Device.Misc.T2ResetCtrl(true);
    
    System.Device.Timer.Start(12, TimerSystick, 2000, StartToWork);
//     while(res == false)
//     {
//         res = System.Device.Timer.Start(12, TimerSystick, 2000, StartToWork);
//     }
}

static void T2Startup(void)
{
//    bool res;

    System.Device.Timer.Stop(9);
    System.Device.Misc.AcinSwitch(false);  
    System.Device.Timer.Start(8, TimerSystick, 2000, T2PowerOn);
//     while(res == false)
//     {
//         res = System.Device.Timer.Start(8, TimerMessage, 2000, T2PowerOn);
//     }
}

static void T2PowerOff(void)
{
    if(App.Data.AdroidStatus == off)
    {
        System.Device.Misc.AcinSwitch(false);       
    }
}

static void Shutdown(void)
{
//    bool res;
    byte shutDownMessage[] = {0xAA, 0x01, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xEE, 0x06, 0xBD};

    if(App.Data.AdroidStatus == off)   return;

    System.Device.Timer.Stop(10);
    System.Device.Timer.Start(10, TimerSystick, App.Data.TimerStartupTime * 60 * 1000, TimerSendStartupMessage);

	sendtoT2_protocol808(shutDownMessage, sizeof(shutDownMessage), 0x9f04);
    //DelayMs(1);                       //等待最后一个字节发送完
    System.Device.Timer.Start(9, TimerSystick, 10000, T2PowerOff);
//     while(res == false)
//     {
//         res = System.Device.Timer.Start(9, TimerSystick, 10000, T2PowerOff);
//     } 
   
    App.Data.AdroidStatus = off;
    HeartBeatOnOff(off);
    App.Data.FingerprintStatus = false;
    System.Device.Misc.AVPowerCtrl(1);
    System.Device.Usart1.Close();
    //System.Device.Usart2.Close();    
}

#if 0
void AccCtrlStrategy(uint data)
{
    //bool carPwrStatus = false;   
    
    if(data == 1)
    {
        System.Device.Timer.Stop(11);
        App.Data.AccStatus = on;        
        HeartBeatCtrl();
        if(App.Data.AdroidStatus == on)    return;
        App.Data.AdroidStatus = on;
        T2Startup();                
		Beidou_GPS_Antenna_Off = 100;
		Status_Self_Check = 0;	// 自检
		
    }
    else
    {   
        App.Data.AccStatus = off;
        Shutdown();		
		
		Beidou_GPS_Antenna_Off = 100;
		Status_Self_Check = 1002;
    }
}
#endif

static void TimerStartupAndroid(void)
{
    if(App.Data.AdroidStatus == on) return;

	
	System.Device.Timer.Start(11, TimerSystick, 300000, Shutdown);
    App.Data.AdroidStatus = on;
    T2Startup();  
    //Mode2HeartbeatCtrl(); 
}

static void TimerSendStartupMessage(void)
{
    PostMessage(MessageTimerStartupAndroid, 0);
    System.Device.Timer.Start(10, TimerSystick, App.Data.TimerStartupTime * 60 * 1000, TimerSendStartupMessage);
}

static void ReleaseResetKey(void)
{
    System.Device.Misc.T2ResetCtrl(false);
    //System.Device.Misc.AcinSwitch(true);
}

static void ResetT2(void)
{
//    bool res;
    
    if(App.Data.AdroidStatus == off)   return;
    System.Device.Misc.T2ResetCtrl(true);
    //System.Device.Misc.AcinSwitch(false);
    System.Device.Timer.Start(6, TimerSystick, 3000, ReleaseResetKey);
//     while(res == false)
//     {
//         res = System.Device.Timer.Start(6, TimerSystick, 3000, ReleaseResetKey);
//     }
}

static void FingerprintConfirm(void)
{
    int i;
    uint year;
    uint month;
    uint day;
    bool res;

    App.Data.FingerprintStatus = true;

    if(App.Data.CarLockStatus == 0 && App.Data.AccStatus == on)
    {
        HeartBeatOnOff(on);
        App.Data.FingerprintStatus = true;
    }
    
    //在模式2下保存此次打指纹的时间
    if(App.Data.CarMode == 2)
    {
        //计算当前时间
        year = AppDataPointer->Rtc.Year - 2000;
        day = (year / 4) * DayInFourYear + (year % 4) * 365;
        month = AppDataPointer->Rtc.Month - 1;
        if (AppDataPointer->Rtc.Year % 4 == 0)
        {
            for (i = 0; i < month; i++)
                day = day + DaysInLeapMonthTable[i];
        }
        else
        {
            for (i = 0; i < month; i++)
                day = day + DaysInNonLeapMonthTable[i];
        }
        day = day + AppDataPointer->Rtc.Day - 1;
        App.Data.FingerLogTime = day * SecondsInDay + AppDataPointer->Rtc.Hour * 3600 + AppDataPointer->Rtc.Minute * 60 + AppDataPointer->Rtc.Second;  //当前时间
        res = System.Device.Storage.Parameter.Write(&App.Data.FingerLogTime);
        if(res == false)
        {
            System.Device.Storage.Parameter.Reorganize();
            App.Data.FingerLogTime = day * SecondsInDay + AppDataPointer->Rtc.Hour * 3600 + AppDataPointer->Rtc.Minute * 60 + AppDataPointer->Rtc.Second;  //当前时间
            System.Device.Storage.Parameter.Write(&App.Data.FingerLogTime);
        }        
    }   
}

__ASM void ModifyStackToMSP(uint addr)
{
    msr msp, r0
    bx  lr
}

static void JumpToBootloader(void)
{
    Function Bootloader;
    
    DisableIrq();
    Bootloader = (Function)*(uint *)(FLASH_BASE + 4);
    ModifyStackToMSP(*(uint *)FLASH_BASE);
    Bootloader();     
}

#if 0
static void ClassifyFrameType(void)
{
    byte tempData[4];
    byte * messagePointer;
    bool res;
    uint timeoutCounter; 
    static uint ResetTimeoutCounter = 0;

    //校验码必须清零
    CheckCode = 0;

    //获取帧头0xAA 0x01 0x00，如果超过10分钟没有数据过来，那就复位T2
    if(System.Device.Usart1.GetByte(&tempData[0]) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(App.Data.AdroidStatus == off && App.Data.AccStatus == off)   ResetTimeoutCounter = 0;
            if(++ResetTimeoutCounter > 75000)      //两分半，10*60*1000/2/2/2
            {
                ResetTimeoutCounter = 0;
#ifdef ENABLE_ANDROID_WATCHDOG								
                PostMessage(MessageResetT2, 0);
#endif							
            }
        }
        return;
    }
    ResetTimeoutCounter = 0;
       
    if(tempData[0] != 0xAA)    return;
    CheckCode ^= tempData[0];

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(&tempData[0], 2) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {           
            if(++timeoutCounter > 1)            //超过2ms没有接收到这俩字节数据就出错
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);
    if((tempData[0] != 0x01) || (tempData[1] != 0x00))  
    {
        messagePointer = InstrErrMessage;
        goto ERR;       
    }
    CheckCode ^= tempData[0];
    CheckCode ^= tempData[1];
    
    //获取数据个数
    timeoutCounter = 0;
    while(System.Device.Usart1.GetByte(&tempData[0]) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {            
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;                
            }
        }
    }
    System.Device.Timer.Stop(0);

    DataSum = tempData[0];
    CheckCode ^= tempData[0];

    //检查4个0xFF
    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(&tempData[0], 4) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {            
            if(++timeoutCounter > 1)                    //超过2ms就重新来过
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0); 
    if(tempData[0] != 0xFF || tempData[1] != 0xFF || 
        tempData[2] != 0xFF || tempData[3] != 0xFF)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }
    CheckCode ^= tempData[0];
    CheckCode ^= tempData[1];
    CheckCode ^= tempData[2];
    CheckCode ^= tempData[3];

    //获取帧类型(状态、命令还是数据)
    timeoutCounter = 0;
    while(System.Device.Usart1.GetByte(&WorkState) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {            
            if(++timeoutCounter > 1)    
            {
                messagePointer = TimeoutMessage;
                goto ERR;                 
            }
        }        
    }  
    CheckCode ^= WorkState;
    System.Device.Timer.Stop(0);
    return;

ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    return;    
}
#endif


static void SetHeartBeatFlag(void)
{
    HeartBeatFlag = true;   
}

static void HeartBeatTimerRoutine(void)
{
    byte buffer[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    if(HeartBeatFlag == false)
    {
        return;
    }
    else if(HeartBeatFlag == true)
    {
		HeartBeatFlag = false;
		
		if (AppDataPointer->EngineSpeed > 0) { // 如果已经打火了就继续发心跳
			System.Device.Can.SendData(HeartBeatID, buffer, sizeof buffer); 
			return;
		}
        
        if(HeartBeatState == on)
        {
			if (Status_Self_Check != 1001) // 自检没完成不发心跳
					return;
				
            //通过CAN发送心跳
            System.Device.Can.SendData(HeartBeatID, buffer, sizeof buffer);        
        }        
    }
}

static void HeartBeatOnOff(byte state)
{
    if(state == on)
    {
        HeartBeatState = on;
    }
    else
    {
        HeartBeatState = off;
    }
}

#if 0
static void HeartBeatCtrlRoutine(void)
{
    byte tempData[2];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;

    if(DataSum != 2)    
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);
    CheckCode ^= tempData[0];
    
    if(CheckCode != tempData[1])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }

    if(tempData[0] == on)
    {
        App.Data.CarLockStatus = 0;        
        res = System.Device.Storage.Parameter.Write(&App.Data.CarLockStatus);
        if(res == false)
        {
            System.Device.Storage.Parameter.Reorganize();
            App.Data.CarLockStatus = 0;
            System.Device.Storage.Parameter.Write(&App.Data.CarLockStatus);
        } 

        if(App.Data.CarMode == 0)
        {
            HeartBeatOnOff(on);
        }
        else
        {
            //if(App.Data.FingerprintStatus == true)
            //{
                HeartBeatOnOff(on);
            //}
        }
    }
    else if(tempData[0] == off)
    {
        App.Data.CarLockStatus = 1;        
        res = System.Device.Storage.Parameter.Write(&App.Data.CarLockStatus);
        if(res == false)
        {
            System.Device.Storage.Parameter.Reorganize();
            App.Data.CarLockStatus = 1;
            System.Device.Storage.Parameter.Write(&App.Data.CarLockStatus);
        }   
        HeartBeatOnOff(off);
    }
    else
    {
        messagePointer = InstrErrMessage;
        goto ERR; 
    }

    //这里应该要返回成功信息
    WorkState = 0;
    return;

ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;    
}
#endif

static void RmpLimitTimerRoutine(void)
{
    byte buffer[] = {0x03, 0xC0, 0x12, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    ushort carSpeedLimit;

    if(CarSpeedLimit < CurrentCarSpeed && CarSpeedLimit > 0)
    {
        App.Data.SpeedLimitStatus = 1;
        //通过CAN发送限速
        carSpeedLimit = 8 * 600;                //只需要让油门失效即可
        buffer[1] = (byte)(carSpeedLimit&0xFF);
        buffer[2] = (byte)(carSpeedLimit>>8);
        System.Device.Can.SendData(RpmLimitID, buffer, sizeof buffer);                       
    }
    else
    {
        App.Data.SpeedLimitStatus = 0;
    }
    System.Device.Timer.Start(2, TimerSystick, 20, RmpLimitTimerRoutine);
}

static void SetRmpLimit(void)
{   
    System.Device.Timer.Start(2, TimerSystick, 20, RmpLimitTimerRoutine);
}

#if 0
static void RpmLimitRountine(void)
{
    byte tempData[3];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;

    if(DataSum != 3) 
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);
    CheckCode ^= tempData[0];
    CheckCode ^= tempData[1];
    if(CheckCode != tempData[2])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }

    //不同车型，计算方法不一样
    CarSpeedLimit = tempData[1] << 8;
    CarSpeedLimit += tempData[0];
    
    SetRmpLimit();
    WorkState = 0;
    return;
    
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;    
}


static void SetCarMode(void)
{
    byte tempData[2];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;

    if(DataSum != 2)    
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);
    CheckCode ^= tempData[0];
    
    if(CheckCode != tempData[1])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }    

    if(tempData[0] < 3 && App.Data.CarMode != tempData[0])
    {
        App.Data.CarMode = tempData[0];
        if(App.Data.CarMode == 0 && App.Data.CarLockStatus == 0)
        {
            HeartBeatOnOff(on);
        }
        
        //把新的模式存入Flash，并且把上一次的打指纹的时间清0保存到Flash中
        res = System.Device.Storage.Parameter.Write(&App.Data.CarMode);
        if(res == false)
        {
            System.Device.Storage.Parameter.Reorganize();
            App.Data.CarMode = tempData[0];
            System.Device.Storage.Parameter.Write(&App.Data.CarMode);
        }

        App.Data.FingerLogTime = 0;
        res = System.Device.Storage.Parameter.Write(&App.Data.FingerLogTime);
        if(res == false)
        {
            System.Device.Storage.Parameter.Reorganize();
            App.Data.FingerLogTime = 0;
            System.Device.Storage.Parameter.Write(&App.Data.FingerLogTime);
        }        
    }

    WorkState = 0;
    return;
    
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;    
}

static void SetFingerprintTime(void)
{
    byte tempData[3];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;

    if(DataSum != 3)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);
    CheckCode ^= tempData[0];
    CheckCode ^= tempData[1];
    if(CheckCode != tempData[2])
    {
        messagePointer = CodeErrMessage;
        goto ERR;
    }

    App.Data.FingerprintTime = tempData[1] << 8;
    App.Data.FingerprintTime += tempData[0];
    if(System.Device.Storage.Parameter.Write(&App.Data.FingerprintTime) == false)
    {
        System.Device.Storage.Parameter.Reorganize();
        App.Data.FingerprintTime = tempData[1] << 8;
        App.Data.FingerprintTime += tempData[0];        
        System.Device.Storage.Parameter.Write(&App.Data.FingerprintTime);
    }

    WorkState = 0;
    return;

ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;  
}

static void LiftBucketCtrl(void)
{
    byte tempData[2];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;

    if(DataSum != 2)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        } 
    }
    System.Device.Timer.Stop(0);
    CheckCode ^= tempData[0];
    if(CheckCode != tempData[1])
    {
        messagePointer = CodeErrMessage;
        goto ERR;
    }

    if(tempData[0] == 0)
    {
        System.Device.Misc.LiftBucketCtrl(0);
        App.Data.LiftLimitStatus = 0;
    }
    else if(tempData[0] == 1)
    {
        System.Device.Misc.LiftBucketCtrl(1);
        App.Data.LiftLimitStatus = 1;
    }
    else
    {
        messagePointer = CodeErrMessage;
        goto ERR;
    }

    WorkState = 0;
    return;
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;    
}

static void SetMcuRtc(void)
{
    byte tempData[7];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;
    
    if(DataSum != 7)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        } 
    }
    System.Device.Timer.Stop(0);
    if(tempData[0] == 0xFF)
    {
        goto EXIT;
    }

    CheckCode ^= tempData[0];
    CheckCode ^= tempData[1];
    CheckCode ^= tempData[2];
    CheckCode ^= tempData[3];
    CheckCode ^= tempData[4];
    CheckCode ^= tempData[5];
    if(CheckCode != tempData[6])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }

    App.Data.Rtc.Year   = tempData[0] + 2000;           //安卓发过来的年范围是0~99，所以需要加2000
    App.Data.Rtc.Month  = tempData[1];
    App.Data.Rtc.Day    = tempData[2];
    App.Data.Rtc.Hour   = tempData[3];
    App.Data.Rtc.Minute = tempData[4];
    App.Data.Rtc.Second = tempData[5];

EXIT:
#if 0    
    printf("year: %d ", App.Data.Rtc.Year);
    printf("month: %d ", App.Data.Rtc.Month);
    printf("day: %d ", App.Data.Rtc.Day);
    printf("hour: %d ", App.Data.Rtc.Hour);                                        
    printf("minute: %d ", App.Data.Rtc.Minute);
    printf("second: %d\n", App.Data.Rtc.Second);
#endif    
    WorkState = 0;
    return;
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;     
}

static void CheckMcuRtc(void)
{
    byte tempData[1];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;
    uint i;
    byte mcuRtcMessage[] = {0xAA, 0x01, 0x00, 0x08, 0xFF, 0xFF, 0xFF, 0xEE, 0x07, 
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB5};

    if(DataSum != 1)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);

    if(CheckCode != tempData[0])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }

    mcuRtcMessage[9]  = App.Data.Rtc.Year & 0xFF;
    mcuRtcMessage[10] = App.Data.Rtc.Year >> 8;
    mcuRtcMessage[11] = App.Data.Rtc.Month;
    mcuRtcMessage[12] = App.Data.Rtc.Day;
    mcuRtcMessage[13] = App.Data.Rtc.Hour;
    mcuRtcMessage[14] = App.Data.Rtc.Minute;
    mcuRtcMessage[15] = App.Data.Rtc.Second;

    CheckCode = 0;
    for(i = 0; i < sizeof mcuRtcMessage - 1; i++)
    {
        CheckCode ^= mcuRtcMessage[i];
    }
    mcuRtcMessage[sizeof mcuRtcMessage - 1] = CheckCode;

    sendtoT2_protocol808_error_msg(mcuRtcMessage, sizeof mcuRtcMessage);  
    WorkState = 0;
    return;
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;       
}



static void UpdateMcuSoftware(void)
{
    byte tempData[1];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;

    if(DataSum != 1)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }    

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);

    if(CheckCode != tempData[0])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }
    
    JumpToBootloader();
    WorkState = 0;
    return;
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;       
}

static void SendHeartBeatEnforce(void)
{
    byte tempData[1];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;

    if(DataSum != 1)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }    

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);

    if(CheckCode != tempData[0])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }
    
    HeartBeatOnOff(on);
    WorkState = 0;
    return;
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;       
}

static void SetStartupTime(void)
{
    byte tempData[3];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;

    if(DataSum != 3)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }    

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);

    EnterCritical();
    App.Data.TimerStartupTime = tempData[1] << 8;
    App.Data.TimerStartupTime += tempData[0];
    ExitCritical();
    CheckCode ^= tempData[0];
    CheckCode ^= tempData[1];

    if(CheckCode != tempData[2])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }

    if(App.Data.TimerStartupTime < 1)
    {
        App.Data.TimerStartupTime = 1;
    }
            
    res = System.Device.Storage.Parameter.Write(&App.Data.TimerStartupTime);
    if(res == false)
    {
        System.Device.Storage.Parameter.Reorganize();
        App.Data.TimerStartupTime = tempData[1] << 8;
        App.Data.TimerStartupTime += tempData[0];
        System.Device.Storage.Parameter.Write(&App.Data.TimerStartupTime);
    } 
       
    WorkState = 0;
    return;
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;       
}

static void LaserPowerCtrl(void)
{
    byte tempData[2];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;

    if(DataSum != 2)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);

    CheckCode ^= tempData[0];

    if(CheckCode != tempData[1])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }

    if(tempData[0] == 0)
    {
        App.Data.LaserPowerStatus = 0;
        System.Device.Misc.LaserPowerCtrl(0);
    }
    else if(tempData[0] == 1)
    {
        App.Data.LaserPowerStatus = 1;
        System.Device.Misc.LaserPowerCtrl(1);
    }
       
    WorkState = 0;
    return;
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;       
}

static void AndroidAutoShutdown(void)
{
    byte tempData[1];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;

    if(DataSum != 1)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }    

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);

    if(CheckCode != tempData[0])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }
    
    Shutdown();
    WorkState = 0;
    return;
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;       
}
#endif

void usart1_send_protocol808(unsigned char * data, unsigned int len)
{
	int i = 0;
	unsigned char buf[2];
	
	for (i=0; i<len; i++) {
		if (data[i] == 0x7e) {
			buf[0] = 0x7d;
			buf[1] = 0x02;
			System.Device.Usart1.SendDirectly(buf, 2);
		} else if (data[i] == 0x7d) {
			buf[0] = 0x7d;
			buf[1] = 0x01;
			System.Device.Usart1.SendDirectly(buf, 2);
		} else {
			buf[0] = data[i];
			System.Device.Usart1.SendDirectly(buf, 1);
		}
	}
}
void sendtoT2_protocol808(unsigned char * data, int len, unsigned short id)
{
	static unsigned short index;
	unsigned char sendBuffer[30], buf808[60];
	unsigned char crc;
	unsigned int i, j;//, len808;
	
	memset(sendBuffer, 0, sizeof(sendBuffer));
	sendBuffer[0] = 0x7e;
	
	sendBuffer[1] = (unsigned char)(id>>8);	// id
	sendBuffer[2] = (unsigned char)id;
	
	sendBuffer[3] = (unsigned char)(len>>8);	// len
	sendBuffer[4] = (unsigned char)len;
	
	sendBuffer[5] = 0x01;	// phone number
	sendBuffer[6] = 0x86;
	sendBuffer[7] = 0x65;
	sendBuffer[8] = 0x82;
	sendBuffer[9] = 0x52;
	sendBuffer[10] = 0x94;

	sendBuffer[11] = (unsigned char)(index >> 8);	// index
	sendBuffer[12] = (unsigned char)index;	
	
	memcpy(&sendBuffer[13], data, len);
	
	for(i=1, crc=0; i<13+len; i++)
		crc ^= sendBuffer[i];
	
	sendBuffer[13+len] = crc;
	sendBuffer[14+len] = 0x7e;
	
	for(i=0,j=0; i<=14+len; i++,j++) {
		
		if (i == 0 || (i == 14+len))	{
			buf808[j] = sendBuffer[i];
			continue;
		}
		
		if (sendBuffer[i] == 0x7e) {
			buf808[j++] = 0x7d;
			buf808[j] = 0x02;
		} else if (sendBuffer[i] == 0x7d) {
			buf808[j++] = 0x7d;
			buf808[j] = 0x01;
		} else {
			buf808[j] = sendBuffer[i];
		}
	}
	
	System.Device.Usart1.SendDirectly(buf808, j);
		
	index++;
}

static void SetCarStateLevel0(void)
{
    CarStateLevel0Flag = true;
    System.Device.Timer.Start(3, TimerSystick, 500, SetCarStateLevel0);
}

static void CarStateLevel0(void)
{
    unsigned char buf[11];
	    
    if(CarStateLevel0Flag == false)
    {
        return;
    }
    else if(CarStateLevel0Flag == true)
    {
        CarStateLevel0Flag = false;
    }
	
	memset(buf, 0, sizeof(buf));
    buf[0] = (byte)(AppDataPointer->Brake);
    if(App.Data.CarType == 0)
    {
        buf[1]  = (byte)(AppDataPointer->CarSpeed1);
        CurrentCarSpeed = AppDataPointer->CarSpeed1;
    }
    else if(App.Data.CarType == 1)
    {
        buf[1]  = (byte)(AppDataPointer->CarSpeed2 & 0xFF);
        buf[2]  = (byte)(AppDataPointer->CarSpeed2 >> 8);
        CurrentCarSpeed = AppDataPointer->CarSpeed2;
    }   
    buf[3] = (byte)(AppDataPointer->EngineSpeed & 0xFF);
    buf[4] = (byte)(AppDataPointer->EngineSpeed >> 8);
    buf[5] = (byte)(AppDataPointer->Torque);
    buf[6] = (byte)(AppDataPointer->Accelerator);
    buf[7] = (byte)(AppDataPointer->FuelConsumption & 0xFF);
    buf[8] = (byte)((AppDataPointer->FuelConsumption>>8) & 0xFF);
    buf[9] = (byte)(AppDataPointer->Level0Status & 0xFF);
    buf[10] = (byte)(AppDataPointer->Level0Status >> 8);
	
    sendtoT2_protocol808(buf, 11, 0x9f01);
}

static void SetCarStateLevel1(void)
{
    CarStateLevel1Flag = true;
    System.Device.Timer.Start(4, TimerSystick, 1000, SetCarStateLevel1);
}

static void CarStateLevel1(void)
{
	unsigned char buf[9];
	
    if(CarStateLevel1Flag == false)
    {
        return;
    }
    else if(CarStateLevel1Flag == true)
    {
        CarStateLevel1Flag = false;
    }	

	memset(buf, 0, sizeof(buf));
#ifdef CHECK_SENSOR_INPUT
	buf[0]  = (byte)((AppDataPointer->LiftIOStatus & 0x0f) << 4);
    buf[0] |= AppDataPointer->LiftSignal & 0x0f;
    buf[1] = (byte)((AppDataPointer->CapeIOStatus & 0x0f) << 4);
    buf[1]|= AppDataPointer->CapeSignal & 0x0f;
#else
	buf[0]  = (byte)((AppDataPointer->LiftIOStatus & 0x01) << 1);
    buf[0] += AppDataPointer->LiftSignal & 0x01;
    buf[1] = (byte)((AppDataPointer->CapeIOStatus & 0x01) << 2);
    buf[1]+= AppDataPointer->CapeSignal & 0x03;
#endif	
	
    buf[2] = (byte)(App.Data.FingerprintStatus);
    buf[3] = (byte)(App.Data.IgnitionStatus);                                                                                                                                                                                                                                   
    buf[4] = (byte)(AppDataPointer->ReverseGear);
    buf[5] = (byte)(AppDataPointer->Direction & 0xFF);
    buf[6] = (byte)((AppDataPointer->Direction>>8) & 0xFF);
    buf[7] = (byte)(AppDataPointer->Level1Status & 0xFF);
    buf[8] = (byte)(AppDataPointer->Level1Status >> 8);
	
    
    sendtoT2_protocol808(buf, 9, 0x9f02);
}

static void SetCarStateLevel2(void)
{
    CarStateLevel2Flag = true;
    System.Device.Timer.Start(5, TimerSystick, 1000, SetCarStateLevel2);
}

static void CarStateLevel2(void)
{
    unsigned char buf[15];

    if(CarStateLevel2Flag == false)
    {
        return;
    }
    else if(CarStateLevel2Flag == true)
    {
        CarStateLevel2Flag = false;
    }    
	
	memset(buf, 0, sizeof(buf));
    buf[0]  = (byte)(App.Data.Version);
    buf[1] = (byte)(App.Data.CarMode & 0xFF);
    buf[2] = (byte)(App.Data.CarLockStatus & 0xFF);
    //buf[3] = (byte)(App.Data.SpeedLimitStatus);
    buf[3] = CarSpeedLimit & 0xFF;
    buf[4] = (byte)(App.Data.LiftLimitStatus);
    buf[5] = (byte)(App.Data.AccStatus);
    buf[6] = App.Data.TimerStartupTime & 0xFF;
    buf[7] = (App.Data.TimerStartupTime >> 8) & 0xFF;
    buf[8] = App.Data.LaserPowerStatus & 0xFF;
		
#ifdef SUPPORT_CHECK_ADC // 过检时候需要使用ADC检测压力传感器 空重载检测传感器。
		{
			unsigned short AdcValue = 0x0;
			AdcValue = System.Device.Adc.get(8);
			buf[10] = AdcValue >> 8;
			buf[9] = AdcValue & 0xFF;
		}
#endif
		
    buf[11] = (byte)(AppDataPointer->Level2Status & 0xFF);
    buf[12] = (byte)(AppDataPointer->Level2Status >> 8);
	buf[13] = (byte)(AppDataPointer->Level2Status >> 16);
	buf[14] = (byte)(AppDataPointer->Level2Status >> 24);

		
    sendtoT2_protocol808(buf, 15, 0x9f03);	

	self_check();	// 自检流程控制
		
	Rev4DataSystick1000Routine(); // 指纹数据接收
		
}
#if 0
static void HeartBeatCtrl(void)
{   
    //开启发送心跳的软定时器
    System.Device.Timer.Start(1, TimerSystick, 20, SetHeartBeatFlag);

    if(App.Data.CarLockStatus == 1)             //假如锁车，那就不发送心跳
    {
        HeartBeatOnOff(off);       
        goto EXIT;
    }

    if(App.Data.CarMode == 0)                   //处于自由模式，直接发心跳
    {
        HeartBeatOnOff(on);
        goto EXIT;
    }
    else if((App.Data.CarMode == 1) || (App.Data.CarMode == 2))
    {
		if (App.Data.CarLockStatus == 0)	// 没有锁车时候直接发心跳
			HeartBeatOnOff(on); 			///////////////////////
        goto EXIT;
    }
//     else if(App.Data.CarMode == 2)
//     {
//         Mode2HeartbeatCtrl();
//         goto EXIT;
//     }

EXIT:
    return;
}


static void EnterDownloadMode(void)
{
    byte tempData[1];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;

    if(DataSum != 1)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }    

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);

    if(CheckCode != tempData[0])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }
    
    App.Data.FingerprintState = 2;
    WorkState = 0;
    return;
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;     
}

static void ExitDownloadMode(void)
{
    byte tempData[1];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;

    if(DataSum != 1)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }    

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);

    if(CheckCode != tempData[0])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }
    
    App.Data.FingerprintState = 0;
    WorkState = 0;
    return;
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;    
}

static void FingerDataPassthrough(void)
{
    static byte tempData[666];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;
    int i;

    if(DataSum != 0xFF)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }    

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, 2) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);

    CheckCode ^= tempData[0];
    CheckCode ^= tempData[1];
    DataSum = tempData[1] << 8;
    DataSum += tempData[0];

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum+1) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 500)//20)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);  

    for(i = 0; i < DataSum; i++)
    {
        CheckCode ^= tempData[i];
    }

    if(CheckCode != tempData[DataSum])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }

    System.Device.Uart4.SendDirectly(tempData, DataSum);
	
	//sendtoT2_protocol808(tempData, DataSum, 0x9e02);
    WorkState = 0;
    return;
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;    
}

static void LedMatrixPassthrough(void)
{
    static byte tempData[512];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;
    int i;

    if(DataSum != 0xFF)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }    

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, 2) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);

    CheckCode ^= tempData[0];
    CheckCode ^= tempData[1];
    DataSum = tempData[1] << 8;
    DataSum += tempData[0];

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum+1) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 20)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);  

    for(i = 0; i < DataSum; i++)
    {
        CheckCode ^= tempData[i];
    }

    if(CheckCode != tempData[DataSum])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }

    System.Device.Uart5.SendDirectly(tempData, DataSum);
    WorkState = 0;
    return;
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;    
}






// 从uart1的T2收到数据转发给uart5的北斗主机
static void BeiDouPassthrough(void)
{
    static byte tempData[512];
    byte * messagePointer;
    bool res;
    uint timeoutCounter;
    int i;

    if(DataSum != 0xFF)
    {
        messagePointer = InstrErrMessage;
        goto ERR;
    }    

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, 2) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 1)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);

    CheckCode ^= tempData[0];
    CheckCode ^= tempData[1];
    DataSum = tempData[1] << 8;
    DataSum += tempData[0];

    timeoutCounter = 0;
    while(System.Device.Usart1.GetData(tempData, DataSum+1) == false)
    {
        res = System.Device.Timer.Start(0, TimerSystick, 2, DummyFunction);
        if(res == true)
        {
            if(++timeoutCounter > 20)
            {
                messagePointer = TimeoutMessage;
                goto ERR;
            }
        }
    }
    System.Device.Timer.Stop(0);  

    for(i = 0; i < DataSum; i++)
    {
        CheckCode ^= tempData[i];
    }

    if(CheckCode != tempData[DataSum])
    {
        messagePointer = CodeErrMessage;
        goto ERR;        
    }

    System.Device.Uart5.SendDirectly(tempData, DataSum);
    WorkState = 0;
    return;
ERR:
    sendtoT2_protocol808_error_msg(messagePointer, MessageLenth);
    WorkState = 0;
    return;    
}
#endif


// 解析从北斗得到的数据，仅仅分析是否gps天线断开信号，其它一律转发给T2
extern unsigned int BeidouGetPackage(unsigned char * data);
extern void BeidouRxReStart(void);
extern unsigned char BeidouRxBuf[];

// 从北斗主机串口得到的数据
void ParseBeidouData(void)
{
	unsigned char * data;
	unsigned int length, len;
	
	length = BeidouGetPackage(data);
	if (length == 0)	return;
	data = BeidouRxBuf;

	if (data[0] == 0x7e && data[1] == 0x99 && data[2] == 0x01) {
		Beidou_GPS_Antenna_Off = data[13];
	}	
	
	
	while(length) {
		
		if (length >= 300)	len = 300;
		else len = length;
		
		System.Device.Usart1.SendDirectly(data, len);
		
		HeartBeatTimerRoutine();
		
		data += len;
		length -= len;
	}
	
	BeidouRxReStart();
	
}

static void InitLogic(void)
{
	System.Device.Can.BaudRateInit(CAN_BAUTRATE);
    System.Device.Can.FilterInit(0, 0x0CF00400, 0xFFFFFFFF, CanFifo0, ExtData);     //扭矩发送机转速
    System.Device.Can.FilterInit(1, 0x18FEF100, 0xFFFFFFFF, CanFifo0, ExtData);     //车速
    System.Device.Can.FilterInit(2, 0x18FF9900, 0xFFFFFFFF, CanFifo0, ExtData);     //举升顶盖信号(三一)
    System.Device.Can.FilterInit(3, 0x0CFE6CEE, 0xFFFFFFFF, CanFifo0, ExtData);     //车速(福田欧曼)
    System.Device.Can.FilterInit(4, 0x0CF00300, 0xFFFFFFFF, CanFifo0, ExtData);     //油门开度
    System.Device.Can.FilterInit(5, 0x18FF0317, 0xFFFFFFFF, CanFifo1, ExtData);     //倒车信号
    System.Device.Can.FilterInit(6, 0x1C1F3E17, 0xFFFFFFFF, CanFifo1, ExtData);     //刹车信号
    System.Device.Can.FilterInit(7, 0x18FF0017, 0xFFFFFFFF, CanFifo1, ExtData);     //左右转向信号，远光灯
    System.Device.Can.FilterInit(8, 0x18FEF200, 0xFFFFFFFF, CanFifo1, ExtData);     //油耗
}


void self_check(void)
{
	if (Status_Self_Check == 0) {
				
		if (Beidou_GPS_Antenna_Off == 1) {
			System.Device.Uart5.SendDirectly(BD_TTS_ANTENNA_MISS, strlen(BD_TTS_ANTENNA_MISS));		
			Status_Self_Check = 1;	// 天线不存在
		} else if (Beidou_GPS_Antenna_Off == 0){				
			Status_Self_Check = 2;	// 有天线存在				
		} else if (Beidou_GPS_Antenna_Off == 100) {
			System.Device.Uart5.SendDirectly(BD_QUERY_ANTENNA_EXIST, strlen(BD_QUERY_ANTENNA_EXIST)); // 查询北斗主机GPS天线是否断开的命令
		}
		
	} else if (Status_Self_Check == 1) {  // 天线不存在的情况
		
		if (Beidou_GPS_Antenna_Off == 0) {  // 检测到天线了
			Status_Self_Check = 2;
		} else {
			System.Device.Uart5.SendDirectly(BD_QUERY_ANTENNA_EXIST, strlen(BD_QUERY_ANTENNA_EXIST)); // 查询北斗主机GPS天线是否断开的命令
		}
		
	} else if (Status_Self_Check == 2) {
		
		if (App.Data.CarLockStatus == 0) {  // 没锁车
			Status_Self_Check = 4;  //1001;	// 结束自检
			
		} else if (App.Data.CarLockStatus == 1) {
			Status_Self_Check = 3;	// 锁车
			System.Device.Uart5.SendDirectly(BD_TTS_LOCKED, strlen(BD_TTS_LOCKED));
		}				
	} else if (Status_Self_Check == 3) {  // 检测到锁车的情况
		
		if (App.Data.CarLockStatus == 0) 
			Status_Self_Check = 4;// 1001;	// 结束自检
		
	} else if (Status_Self_Check == 4) {
		
		if (0) {// 什么模式不需要检测指纹
			System.Device.Uart5.SendDirectly(BD_TTS_START_ENGINE, strlen(BD_TTS_START_ENGINE));
			Status_Self_Check = 1001;
		} else {
			System.Device.Uart5.SendDirectly(BD_TTS_PLEASE_VERIFY, strlen(BD_TTS_PLEASE_VERIFY)); // 请按指纹
			//发送获取指纹的命令
			FingerprintMatch = 0;
			
			App.Data.FingerprintState = 0;
			
			Status_Self_Check = 5;
		}
	} else if (Status_Self_Check == 5) {
		
		//byte cmd[] = {0x02, 0x30, 0x30, 0x30, 0x38, 0x31, 0x37, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3F, 0x3F, 0x30, 0x30, 0x30, 0x30, 0x3E, 0x30, 0x03 };
		//System.Device.Uart4.SendDirectly(cmd, sizeof cmd);
		//SendFingerConfirmCmdRoutine();
		if (FingerprintMatch == 100) {
			Status_Self_Check = 1001;
			System.Device.Uart5.SendDirectly(BD_TTS_START_ENGINE, strlen(BD_TTS_START_ENGINE));
			App.Data.FingerprintState = 2;
		} else if (FingerprintMatch == 99) {
			FingerprintMatch = 0;
			System.Device.Uart5.SendDirectly(BD_TTS_FAIL_VERIFY, strlen(BD_TTS_FAIL_VERIFY));
			App.Data.FingerprintState = 2;
		}
	}	
}


extern unsigned char usart1_buf[];
extern volatile unsigned int usart1_index;
char tmphead[] = {0xaa, 0x01, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff};


	
void usart1_cmd_process(void)
{
	unsigned char *dat = usart1_buf;
	unsigned int  buffer_len, i;
	unsigned char len;
	unsigned char cmd;
	unsigned char crc;
	unsigned char clear = 0;
	
	buffer_len = usart1_index;

	if (buffer_len <= 9)	return;
	
	if (memcmp(dat, tmphead, 3)) {
		usart1_index = 0;
		memset(usart1_buf, 0, 1024);			
		return;
	}
	if (memcmp(&dat[4], &tmphead[4], 4)) {
		usart1_index = 0;
		memset(usart1_buf, 0, 1024);			
		return;
	}
	
	len = dat[3];
	cmd = dat[8];
	
	if ((cmd == 0xF0) || (cmd == 0xF1) || (cmd == 0xF2)) {
		unsigned short len808;
		
		if (buffer_len < 11)	return;
		len808 = (dat[10]<<8) | dat[9];
		if (buffer_len < len808 + 3 + 9) 	return;	
		
	} else {
		if (buffer_len < len + 9)	return;
		if (buffer_len !=  len + 9) {
			usart1_index = 0;
			memset(usart1_buf, 0, 1024);
			return;
		}
		
	}
	
	
	
	if (cmd == 0x08) {	// iap
		
		crc = dat[9];
		JumpToBootloader();
		
	} else if (cmd == 0xF2) {	// beidou
		
		unsigned short len808 = (dat[10]<<8) | dat[9];
		System.Device.Uart5.SendDirectly(&dat[11], len808);
		
		clear = 1;
		
	} else if (cmd == 0x01) { // 锁车命令
		if (dat[9] == 1){	// 解锁
			App.Data.CarLockStatus = 0;        			
			if(System.Device.Storage.Parameter.Write(&App.Data.CarLockStatus) == false)	{
				System.Device.Storage.Parameter.Reorganize();
				App.Data.CarLockStatus = 0;
				System.Device.Storage.Parameter.Write(&App.Data.CarLockStatus);
			} 
			trigHeartbeat();
		} else if (dat[9] == 0) { // 锁车
			App.Data.CarLockStatus = 1;
			if(System.Device.Storage.Parameter.Write(&App.Data.CarLockStatus) == false)	{
				System.Device.Storage.Parameter.Reorganize();
				App.Data.CarLockStatus = 1;
				System.Device.Storage.Parameter.Write(&App.Data.CarLockStatus);
			}   
			trigHeartbeat();
		} else {
			sendtoT2_protocol808(InstrErrMessage, sizeof(InstrErrMessage), 0x9f1f);	// 报告异常，没有此命令
		}	
		
		clear = 1;
	} else if (cmd == 0x02) { // 限速
		CarSpeedLimit = dat[10] << 8;
		CarSpeedLimit += dat[9];
	} else if (cmd == 0x03) { // 设置模式，自由模式，模式1， 模式2
		unsigned char mode = dat[9];
		if (mode < 3 && mode != App.Data.CarMode) {
			App.Data.CarMode = mode;
			if (App.Data.CarMode == 0 && App.Data.CarLockStatus == 0)
				HeartBeatOnOff(on);

			if(System.Device.Storage.Parameter.Write(&App.Data.CarMode) == false) {
				System.Device.Storage.Parameter.Reorganize();
				App.Data.CarMode = mode;
				System.Device.Storage.Parameter.Write(&App.Data.CarMode);
			}

			App.Data.FingerLogTime = 0;
			if(System.Device.Storage.Parameter.Write(&App.Data.FingerLogTime) == false)
			{
				System.Device.Storage.Parameter.Reorganize();
				App.Data.FingerLogTime = 0;
				System.Device.Storage.Parameter.Write(&App.Data.FingerLogTime);
			} 

			trigHeartbeat();			
		}
		clear = 1;
	} else if (cmd == 0x04) { // 设置指纹时间，模式2下，打了一次指纹之后多长时间不需要再打指纹
		sendtoT2_protocol808(InstrErrMessage, sizeof(InstrErrMessage), 0x9f1f);	// 报告异常，没有此命令
		clear = 1;
	} else if (cmd == 0x05) { // 限制举斗
		if (dat[9] == 1){
			System.Device.Misc.LiftBucketCtrl(1);
			App.Data.LiftLimitStatus = 1;
		} else if (dat[9] == 0) {
			System.Device.Misc.LiftBucketCtrl(0);
			App.Data.LiftLimitStatus = 0;
		} else {
			sendtoT2_protocol808(InstrErrMessage, sizeof(InstrErrMessage), 0x9f1f);	// 报告异常，没有此命令
		}	
		
		clear = 1;
		
	} else if (cmd == 0x06) { // android自己关机
		Shutdown();
		clear = 1;
		
	} else if (cmd == 0x07) { // 检查单片机RTC硬件有没有工作起来
		unsigned char crc = 0;
		byte mcuRtcMessage[] = {0xAA, 0x01, 0x00, 0x08, 0xFF, 0xFF, 0xFF, 0xEE, 0x07, 
							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB5};
		
		RtcSystick100Routine();							
							
		mcuRtcMessage[9]  = App.Data.Rtc.Year & 0xFF;
		mcuRtcMessage[10] = App.Data.Rtc.Year >> 8;
		mcuRtcMessage[11] = App.Data.Rtc.Month;
		mcuRtcMessage[12] = App.Data.Rtc.Day;
		mcuRtcMessage[13] = App.Data.Rtc.Hour;
		mcuRtcMessage[14] = App.Data.Rtc.Minute;
		mcuRtcMessage[15] = App.Data.Rtc.Second;	
		for(i=0; i<sizeof(mcuRtcMessage)-1; i++)
			crc ^= mcuRtcMessage[i];
		mcuRtcMessage[sizeof mcuRtcMessage - 1] = crc;	
		sendtoT2_protocol808(mcuRtcMessage, sizeof(mcuRtcMessage), 0x9f05);
		clear = 1;

	} else if (cmd == 0x09) { // 无条件发送心跳。此命令已经作废
		
		sendtoT2_protocol808(InstrErrMessage, sizeof(InstrErrMessage), 0x9f1f);	// 报告异常，没有此命令
		clear = 1;
		
	} else if (cmd == 0x0D) { // 设置单片机唤醒时间
		clear = 1;
		
	} else if (cmd == 0x0E) { // 激光测距模块供电控制
		if (dat[9] == 1){
			App.Data.LaserPowerStatus = 1;
			System.Device.Misc.LaserPowerCtrl(1);
		} else if (dat[9] == 0) {
			App.Data.LaserPowerStatus = 1;
			System.Device.Misc.LaserPowerCtrl(1);
		} else {
			sendtoT2_protocol808(InstrErrMessage, sizeof(InstrErrMessage), 0x9f1f);	// 报告异常，没有此命令
		}
		
		clear = 1;
		
	} else if (cmd == 0xEE) { // 进入指纹模板更新模式
		clear = 1;
	} else if (cmd == 0xEF) { // 退出指纹模板更新模式
		clear = 1;
	} else if (cmd == 0xF0) { // 指纹数据透传模式
		clear = 1;
	} else if (cmd == 0xFF) {	// 设置RTC时间
		App.Data.Rtc.Year   = dat[9] + 2000;
		App.Data.Rtc.Month  = dat[10];
		App.Data.Rtc.Day    = dat[11];
		App.Data.Rtc.Hour   = dat[12];
		App.Data.Rtc.Minute = dat[13];
		App.Data.Rtc.Second = dat[14];		
		RtcSystick100Routine();
		clear = 1;
	} else {
		sendtoT2_protocol808(InstrErrMessage, sizeof(InstrErrMessage), 0x9f1f);	// 报告异常，没有此命令
		clear = 1;			
	}
	
	if (clear) {
		usart1_index = 0;
		memset(usart1_buf, 0, 1024);
	}
			
}

uint getCurrSysTick(){
	return 0;
}

bool isIntervalTimeout(uint lastSystick,uint interval){
	return getCurrSysTick()-lastSystick >= interval;
}


void sendHeartbeat(){
	byte buffer[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    //通过CAN发送心跳
    System.Device.Can.SendData(HeartBeatID, buffer, sizeof buffer);
}
	
void sendLimitrpmCmd(){
	byte buffer[] = {0x03, 0xC0, 0x12, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    ushort carRpmLimit;
	
	//通过CAN发送限速
    carRpmLimit = 8 * 600;                //只需要让油门失效即可
    buffer[1] = (byte)(carRpmLimit&0xFF);
    buffer[2] = (byte)(carRpmLimit>>8);
    System.Device.Can.SendData(RpmLimitID, buffer, sizeof buffer);   
    return;
}

void sendCarLevel0(){
	unsigned char buf[11];
	   
	memset(buf, 0, sizeof(buf));
    buf[0] = (byte)(AppDataPointer->Brake);
    if(App.Data.CarType == 0)
    {
        buf[1]  = (byte)(AppDataPointer->CarSpeed1);
        CurrentCarSpeed = AppDataPointer->CarSpeed1;
    }
    else if(App.Data.CarType == 1)
    {
        buf[1]  = (byte)(AppDataPointer->CarSpeed2 & 0xFF);
        buf[2]  = (byte)(AppDataPointer->CarSpeed2 >> 8);
        CurrentCarSpeed = AppDataPointer->CarSpeed2;
    }   
    buf[3] = (byte)(AppDataPointer->EngineSpeed & 0xFF);
    buf[4] = (byte)(AppDataPointer->EngineSpeed >> 8);
    buf[5] = (byte)(AppDataPointer->Torque);
    buf[6] = (byte)(AppDataPointer->Accelerator);
    buf[7] = (byte)(AppDataPointer->FuelConsumption & 0xFF);
    buf[8] = (byte)((AppDataPointer->FuelConsumption>>8) & 0xFF);
    buf[9] = (byte)(AppDataPointer->Level0Status & 0xFF);
    buf[10] = (byte)(AppDataPointer->Level0Status >> 8);
	
    sendtoT2_protocol808(buf, 11, 0x9f01);
	return;
}
	
void sendCarLevel1(){
	unsigned char buf[9];
	
	memset(buf, 0, sizeof(buf));
#ifdef CHECK_SENSOR_INPUT
	buf[0]  = (byte)((AppDataPointer->LiftIOStatus & 0x0f) << 4);
    buf[0] |= AppDataPointer->LiftSignal & 0x0f;
    buf[1] = (byte)((AppDataPointer->CapeIOStatus & 0x0f) << 4);
    buf[1]|= AppDataPointer->CapeSignal & 0x0f;
#else
	buf[0]  = (byte)((AppDataPointer->LiftIOStatus & 0x01) << 1);
    buf[0] += AppDataPointer->LiftSignal & 0x01;
    buf[1] = (byte)((AppDataPointer->CapeIOStatus & 0x01) << 2);
    buf[1]+= AppDataPointer->CapeSignal & 0x03;
#endif	
	
    buf[2] = (byte)(App.Data.FingerprintStatus);
    buf[3] = (byte)(App.Data.IgnitionStatus);                                                                                                                                                                                                                                   
    buf[4] = (byte)(AppDataPointer->ReverseGear);
    buf[5] = (byte)(AppDataPointer->Direction & 0xFF);
    buf[6] = (byte)((AppDataPointer->Direction>>8) & 0xFF);
    buf[7] = (byte)(AppDataPointer->Level1Status & 0xFF);
    buf[8] = (byte)(AppDataPointer->Level1Status >> 8);
	
    sendtoT2_protocol808(buf, 9, 0x9f02);
	return;
}
	
void sendCarLevel2(){
	unsigned char buf[15];  
	
	memset(buf, 0, sizeof(buf));
    buf[0]  = (byte)(App.Data.Version);
    buf[1] = (byte)(App.Data.CarMode & 0xFF);
    buf[2] = (byte)(App.Data.CarLockStatus & 0xFF);
    //buf[3] = (byte)(App.Data.SpeedLimitStatus);
    buf[3] = CarSpeedLimit & 0xFF;
    buf[4] = (byte)(App.Data.LiftLimitStatus);
    buf[5] = (byte)(App.Data.AccStatus);
    buf[6] = App.Data.TimerStartupTime & 0xFF;
    buf[7] = (App.Data.TimerStartupTime >> 8) & 0xFF;
    buf[8] = App.Data.LaserPowerStatus & 0xFF;
		
#ifdef SUPPORT_CHECK_ADC // 过检时候需要使用ADC检测压力传感器 空重载检测传感器。
		{
			unsigned short AdcValue = 0x0;
			AdcValue = System.Device.Adc.get(8);
			buf[10] = AdcValue >> 8;
			buf[9] = AdcValue & 0xFF;
		}
#endif
		
    buf[11] = (byte)(AppDataPointer->Level2Status & 0xFF);
    buf[12] = (byte)(AppDataPointer->Level2Status >> 8);
	buf[13] = (byte)(AppDataPointer->Level2Status >> 16);
	buf[14] = (byte)(AppDataPointer->Level2Status >> 24);

		
    sendtoT2_protocol808(buf, 15, 0x9f03);	
	return;
}


void processCarLevel(){
	static uint lastSendTimeCarLevel0 = 0;
	static uint lastSendTimeCarLevel1 = 0;
	static uint lastSendTimeCarLevel2 = 0;
	
	//500ms
	if(isIntervalTimeout(lastSendTimeCarLevel0,500)){
		sendCarLevel0();
		lastSendTimeCarLevel0 = getCurrSysTick();
	}
	
	//1s
	if(isIntervalTimeout(lastSendTimeCarLevel1,1000)){
		sendCarLevel1();
		lastSendTimeCarLevel1 = getCurrSysTick();
	}
	
	//1s
	if(isIntervalTimeout(lastSendTimeCarLevel2,1000)){
		sendCarLevel2();
		lastSendTimeCarLevel2 = getCurrSysTick();
	}
}

void processLimitspeed(){
	static uint lastSendLimitrpmCmd = 0;
	
	//20ms
	if(CarSpeedLimit < CurrentCarSpeed && CarSpeedLimit > 0)
    {
        App.Data.SpeedLimitStatus = 1;
        if(isIntervalTimeout(lastSendLimitrpmCmd,20)){
			sendLimitrpmCmd();
			lastSendLimitrpmCmd = getCurrSysTick();
		}
		return;
	}
    else
    {
        App.Data.SpeedLimitStatus = 0;
		return;
    }
}

void doHeartbeat(bool bHeartbeat){
	static bool sbHeartbeatOn = false;
	static uint lastSendHeartbeatCmd = 0;
	
	if(sbHeartbeatOn && !bHeartbeat && (AppDataPointer->EngineSpeed > 0)){
		;
	}else{
		sbHeartbeatOn = bHeartbeat;
	}
	
	if(!sbHeartbeatOn){
		return;
	}
	
	if(isIntervalTimeout(lastSendHeartbeatCmd,20)){
		sendHeartbeat();
		lastSendHeartbeatCmd = getCurrSysTick();
	}
}

bool isHeartbeatOn(){
	//free mode
	if(App.Data.CarMode == 0){
		if(gAccState == ACC_ON){
			return true;
		}else{
			return false;
		}
	}
	
	//lock state
	if(App.Data.CarLockStatus == 1){
		return false;
	}
	
	//self check
	if(Status_Self_Check != 1001){
		return false;
	}
	
	return gAccState == ACC_ON;
}
	
void trigHeartbeat(){
	doHeartbeat(isHeartbeatOn());
	return;
}

#define AccStatusConfirm    300
//#define AccStatusOnConfirm  50
#define AccStatusOnConfirm  300
#define AccStatusOffConfirm 1000


//2: acc on; 1: acc off; 0:undo
static int acc_systick100_routine(void)
{
    static bool Scan   = false;
    static bool Switch = false;
    static ushort AccOnCounter = 0;
	static ushort AccOffCounter = 0;
	
    Switch = ~Switch;    
    if(Switch)
    {
        //if(*pPinAccDet == 0)            //ACC接通，该脚读到0，否则读到1
		if (gpio_get(GPIO_IN_ACC_DET) == 0)
        {
            Scan = true;
        }
        else
        {
            Scan = false;
        }
    }
    else
    {
        if(Scan == true)
        {     
            AccOffCounter = 0;
            if(AccOnCounter < AccStatusOnConfirm)
            {
                AccOnCounter++;
            }
            else if(AccOnCounter == AccStatusOnConfirm)
            {                
                AccOnCounter = 0xFFFF;                
                //PostMessage(MessageAccDet, true);
				//AccCtrlStrategy(true);
				return 2;
            }
        }
        else
        {     
            AccOnCounter = 0;
            if(AccOffCounter < AccStatusOffConfirm)
            {   
                AccOffCounter++;
            }
            else if(AccOffCounter == AccStatusOffConfirm)
            {                
                AccOffCounter = 0xFFFF;                
                //PostMessage(MessageAccDet, false);
				//AccCtrlStrategy(false);
				return 1;
            }
        }
    }
	
	return 0;
}




#define LiftDetConfirm  20
#define CapeDetConfirm  20

static int sensor_lift_detect(void)
{
    static bool Scan   = false;
    static bool Switch = false;
	static byte LiftOnCounter = 0;
	static byte LiftOffCounter = 0;
    
    Switch = ~Switch;    
    if(Switch)
    {
        if(gpio_get(GPIO_IN_LIFT_DET) == 1)  
        {
            Scan = true;
        }
        else
        {
            Scan = false;
        }
    }
    else
    {
        if(Scan == true)
        {
            LiftOffCounter = 0;
            if(LiftOnCounter < LiftDetConfirm)
            {
                LiftOnCounter++;
            }
            else if(LiftOnCounter == LiftDetConfirm)
            {
                LiftOnCounter = invalid;        
                AppDataPointer->LiftIOStatus = 1;
            }
        }
        else
        {
            LiftOnCounter = 0;
            if(LiftOffCounter < LiftDetConfirm)
            {   
                LiftOffCounter++;
            }
            else if(LiftOffCounter == LiftDetConfirm)
            {
                LiftOffCounter = invalid; 
                AppDataPointer->LiftIOStatus = 0;
            }
        }
    }	
}
static int sensor_cape_detect(void)
{
    static bool Scan   = false;
    static bool Switch = false;
	static byte CapeOnCounter = 0;
	static byte CapeOffCounter = 0;
    
    Switch = ~Switch;    
    if(Switch)
    {
        if(gpio_get(GPIO_IN_CAPE_DET) == 1)  
        {
            Scan = true;
        }
        else
        {
            Scan = false;
        }
    }
    else
    {
        if(Scan == true)
        {
            CapeOffCounter = 0;
            if(CapeOnCounter < CapeDetConfirm)
            {
                CapeOnCounter++;
            }
            else if(CapeOnCounter == CapeDetConfirm)
            {
                CapeOnCounter = invalid;        
                AppDataPointer->CapeIOStatus = 1;
            }
        }
        else
        {
            CapeOnCounter = 0;
            if(CapeOffCounter < CapeDetConfirm)
            {   
                CapeOffCounter++;
            }
            else if(CapeOffCounter == CapeDetConfirm)
            {
                CapeOffCounter = invalid; 
                AppDataPointer->CapeIOStatus = 0;
            }
        }
    }    	
}
int acc_onoff_process(void) 
{
	static unsigned int lastACCtime = 0;
	
	if(isIntervalTimeout(lastACCtime,100)){
		lastACCtime = getCurrSysTick();
		sensor_lift_detect();	// 举升io检测
		sensor_cape_detect();	// 篷布io检测
		//RtcSystick100Routine();	// RTC update.
		return acc_systick100_routine();
	}
	
	return 0;
}


void acc_ctrl_strategy(uint data)
{
    //bool carPwrStatus = false;   
    
    if(data == 2)
    {
        System.Device.Timer.Stop(11);
        App.Data.AccStatus = on;        
        //HeartBeatCtrl();
        if(App.Data.AdroidStatus == on)    return;
        App.Data.AdroidStatus = on;
        T2Startup();                
		Beidou_GPS_Antenna_Off = 100;
		Status_Self_Check = 0;	// 自检
		
    }
    else if(data == 1)
    {   
        App.Data.AccStatus = off;
        Shutdown();		
		
		Beidou_GPS_Antenna_Off = 100;
		Status_Self_Check = 1002;
    }
}

void LogicTask(void)
{
    uint message;
    uint data; 
    bool res;
	    
	InitLogic(); 	// can filter init.     

    //GetFingerprintInit();

    //System.Device.Misc.EnableAccDetect(true); 

    //发送获取指纹的命令
    //SendFingerConfirmCmdRoutine();
    
    //启动定时开机功能
    //System.Device.Timer.Start(10, TimerSystick, App.Data.TimerStartupTime * 60 * 1000, TimerSendStartupMessage);

    //启动向android发送数据的例行程序
//     System.Device.Timer.Start(3, TimerSystick, 500, SetCarStateLevel0);
//     System.Device.Timer.Start(4, TimerSystick, 1000, SetCarStateLevel1);
//     System.Device.Timer.Start(5, TimerSystick, 1000, SetCarStateLevel2);
	
    //逻辑业务任务获取消息，分配处理
    while(true)
    {
		//updateLogicTaskTime();
		gAccState = acc_onoff_process();
		if((gAccState == 1) || (gAccState == 2)){
			acc_ctrl_strategy(gAccState);
		}
		
		usart1_cmd_process();	// 从android发过来的命令
		ParseBeidouData();
		
		trigHeartbeat();
		processLimitspeed();
		
		processCarLevel();
#if 0
//         res = System.OS.PendMessageQueue(&message);     //这里没有任务切换，所以要换成这种
//         if(res == true)
//         {
//             data = message & 0x00FFFFFF;
//             switch(Byte3(message))
//             {
//                 case MessageTimer:
//                     Function(data + RomBase);
//                     break;
//                 case MessageAccDet:
//                     AccCtrlStrategy(data);
//                     break;
//                 case MessageTimerStartupAndroid:
//                     TimerStartupAndroid();
//                     break;
//                 case MessageResetT2:
//                     ResetT2();
//                     break;
//                 case MessageFingerprintConfirm:
//                     FingerprintConfirm();
//                     break;
//                 case MessageFingerprintModule:
//                     ParseFingerprintModuleData();
//                     break;
// 				//case MessageBeidouReceived:
// 					//ParseBeidouData();
// 					//break;
//                 default:
//                     break;
//             }
//         }
#endif

		
#if 0		
		
//         //根据标志，发送心跳
//         HeartBeatTimerRoutine();

//         //根据标志，定时做一些事情
//         CarStateLevel0();
//         CarStateLevel1();
//         CarStateLevel2();
		
#endif		
		
#if 0
        //解析串口接收到的数据，10分钟内必须通信一次，否则单片机复位安卓，这里要测试复位键是否能确保安卓重新启动，因为T2的引脚会被漏电过来
//         switch(WorkState)
//         {
//             case 0x00:             //寻找帧头
//                 ClassifyFrameType();
//                 break;
//             case 0x01:             //锁车命令
//                 HeartBeatCtrlRoutine();
//                 break;
//             case 0x02:             //限速命令
//                 RpmLimitRountine();
//                 break;
//             case 0x03:             //设置模式(自由模式、模式1、模式2)
//                 SetCarMode();
//                 break;
//             case 0x04:             //设置指纹时间，在模式2下，打了一次指纹之后多长时间内不需要再打指纹
//                 SetFingerprintTime();
//                 break;
//             case 0x05:             //限制举斗
//                 LiftBucketCtrl();
//                 break;
//             case 0x06:             //Android自主关机
//                 AndroidAutoShutdown();
//                 break;                
//             case 0x07:             //可以用来检查单片机RTC硬件有没有工作起来
//                 CheckMcuRtc();
//                 break;
//             case 0x08:             //更新单片机程序
//                 UpdateMcuSoftware();
//                 break;
//             case 0x09:             //无条件发送心跳
//                 SendHeartBeatEnforce();
//                 break;
//             case 0x0D:             //设置单片机唤醒时间
//                 SetStartupTime();
//                 break;
//             case 0x0E:             //激光测距模块供电控制
//                 LaserPowerCtrl();
//                 break;
//             case 0xEE:             //进入指纹模板更新模式
//                 EnterDownloadMode();
//                 break;
//             case 0xEF:             //退出指纹模板更新模式
//                 ExitDownloadMode();
//                 break;
//             case 0xF0:             //指纹数据，透传用
//                 FingerDataPassthrough();
//                 break;
// 			case 0xF1:				// uart5透传，连接外屏
// 				LedMatrixPassthrough();
// 				break;
// 			case 0xF2:				// 北斗主机透传uart5
// 				BeiDouPassthrough();
// 				break;
//             case 0xFF:             //安卓发送校准RTC时间
//                 SetMcuRtc();
//                 break;
//             default:
//                 sendtoT2_protocol808_error_msg(InstrErrMessage, sizeof InstrErrMessage);
//                 WorkState = 0;
//                 break;
//         }
#endif
    } 
}
