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

#define ACC_ON 1
#define ACC_OFF 2
#define ACC_UNKNOWN 0

static volatile char finger_down_flag = 0;
//static volatile unsigned int finger_number = 0;
unsigned char FingerId[64];



char BD_QUERY_ANTENNA_EXIST[]	=	{"cmd=5;type=5;num=0;"};
char BD_TTS_LOCKED[]			=	{0x63, 0x6d, 0x64, 0x3d, 0x36, 0x3b, 0x6d, 0x73, 0x67, 0x3d, 0x63, 0x62, 0x66, 0x38, 0x62, 0x33, 0x62, 0x35, 0x3b};
char BD_TTS_ANTENNA_MISS[]		=	{0x63, 0x6d, 0x64, 0x3d, 0x36, 0x3b, 0x6d, 0x73, 0x67, 0x3d, 0x62, 0x31, 0x62, 0x31, 0x62, 0x36, 0x62, 0x37, 0x63, 0x63, 0x65, 0x63, 0x63, 0x66, 0x64, 0x66, 0x64, 0x32, 0x65, 0x63, 0x62, 0x33, 0x61, 0x33, 0x3b};
char BD_TTS_START_ENGINE[]		=	{0x63, 0x6d, 0x64, 0x3d, 0x36, 0x3b, 0x6d, 0x73, 0x67, 0x3d, 0x63, 0x37, 0x65, 0x62, 0x63, 0x36, 0x66, 0x34, 0x62, 0x36, 0x61, 0x66, 0x62, 0x37, 0x61, 0x32, 0x62, 0x36, 0x61, 0x66, 0x62, 0x62, 0x66, 0x61, 0x3b};
char BD_TTS_FAIL_VERIFY[]		=	{0x63, 0x6d, 0x64, 0x3d, 0x36, 0x3b, 0x6d, 0x73, 0x67, 0x3d, 0x64, 0x36, 0x62, 0x38, 0x63, 0x65, 0x63, 0x36, 0x64, 0x31, 0x65, 0x39, 0x64, 0x36, 0x61, 0x34, 0x63, 0x61, 0x61, 0x37, 0x62, 0x30, 0x64, 0x63, 0x3b};
char BD_TTS_PLEASE_VERIFY[]		=	{0x63, 0x6d, 0x64, 0x3d, 0x36, 0x3b, 0x6d, 0x73, 0x67, 0x3d, 0x63, 0x37, 0x65, 0x62, 0x62, 0x30, 0x62, 0x34, 0x64, 0x36, 0x62, 0x38, 0x63, 0x65, 0x63, 0x36, 0x3b};
//char BD_TTS_VERIFY_DONE[]       =   {0x63, 0x6d, 0x64, 0x3d, 0x36, 0x3b, 0x6d, 0x73, 0x67, 0x3d, 0xd6, 0xb8, 0xce, 0xc6, 0xd1, 0xe9, 0xd6, 0xa4, 0xb3, 0xc9, 0xb9, 0xa6, 0x3b};
char BD_TTS_VERIFY_DONE[]       =   {0x63, 0x6d, 0x64, 0x3d, 0x36, 0x3b, 0x6d, 0x73, 0x67, 0x3d, 0x64, 0x36, 0x62, 0x38, 0x63, 0x65, 0x63, 0x36, 0x64, 0x31, 0x65, 0x39, 0x64, 0x36, 0x61, 0x34, 0x62, 0x33, 0x63, 0x39, 0x62, 0x39, 0x61, 0x36, 0x3b};
      
typedef void(* Function)(void);

#define SecondsInDay            86400       // 1天总共86400秒
#define DayInFourYear           1461        // 4年总共1461天，365*3+366
                                                                       
static const byte DaysInNonLeapMonthTable[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};  // 闰年的月份日期表
static const byte DaysInLeapMonthTable[12] =    {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};  // 平年的月份日期表

#define HeartBeatID 0x0CF00203
#define RpmLimitID  0x0C000003
#define MessageLenth    11

#define CHECK_OK		201
#define CHECK_INIT		202
#define CHECK_START				0
#define CHECK_GPS_ANT_MISS		1
#define CHECK_GPS_ANT_EXIST		2
#define CHECK_LOCKED			3
#define CHECK_LOCK_NONE			4
#define CHECK_FINGER			5

void self_check(void);	// 自检流程控制
void self_check_process(void);
volatile unsigned char Status_Self_Check = CHECK_INIT; // 初值acc off

extern void RtcSystick100Routine(void);
static int boot_android(int signal);
bool isIntervalTimeout(uint lastSystick,uint interval);
static void TimerSendStartupMessage(void);
void sendtoT2_protocol808(unsigned char * data, int len, unsigned short id);
//#define sendtoT2_protocol808_error_msg(data, len)	sendtoT2_protocol808(data, len, 0x9f1f)
static volatile int FingerprintMatch = 0;
//extern void Rev4DataSystick1000Routine(void);
void android_watchdog(int feed);
void trigHeartbeat(void);
extern unsigned int getCurrSysTick(void);



static int gAccState = ACC_UNKNOWN;
static byte TimeoutMessage[] = {0xAA, 0x01, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xEE,
                                    0x45, 0x54, 0xAA};  //一帧数据接收超时
static byte CodeErrMessage[] = {0xAA, 0x01, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xEE,
                                    0x45, 0x43, 0XBD};  //接收数据，校验码错误
static byte InstrErrMessage[] = {0xAA, 0x01, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xEE,
                                    0x45, 0x49, 0xB7}; //命令错误(有可能没有此命令)

static volatile ushort CarSpeedLimit     = 0;                 //发送机转速限制值
static volatile uint CurrentCarSpeed   = 0;
static volatile bool CarStateLevel0Flag = false;
static volatile bool CarStateLevel1Flag = false;
static volatile bool CarStateLevel2Flag = false;
static volatile bool HeartBeatFlag = false;
volatile char Beidou_GPS_Antenna_Off = 100;	




// static void auto_start_andoird_process(int signal)
// {
// 	static unsigned int next_boot_time = 0;
// 	
// 	if (signal) {
// 		next_boot_time = getCurrSysTick();
// 		return;
// 	}
// 	
// 	if (isIntervalTimeout(next_boot_time, App.Data.TimerStartupTime * 60 * 1000)) { // 自动开机功能
// 		next_boot_time = getCurrSysTick();
// 		boot_android(3);
// 	}
// }


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

static unsigned char sendBuffer[600], buf808[1200];
void sendtoT2_protocol808(unsigned char * data, int len, unsigned short id)
{
	static unsigned short index;  // 索引号
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




void self_check_process(void)
{
	static unsigned int last_time = 0;
	if (isIntervalTimeout(last_time, 1000)) {
		self_check();
		last_time = getCurrSysTick();
	}
}

// 解析从北斗得到的数据，仅仅分析是否gps天线断开信号，其它一律转发给T2
extern unsigned int BeidouGetPackage(unsigned char * data);
extern void BeidouRxClearBuf(unsigned char* data);




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
    //用于避免北斗主机tts太频繁响应不过来
    static unsigned int last_time_tts = 0;
    static char play_once = 0; // 播报请启动发动机的控制。只播报一次，acc on时候恢复初值
    
#if 0	
	Beidou_GPS_Antenna_Off = 0;
	App.Data.CarLockStatus = 0;
#endif
	if (App.Data.AccStatus == on) {
		if (App.Data.CarMode == 0) {
			if (Status_Self_Check != CHECK_OK) {
				Status_Self_Check = CHECK_OK;
                play_once = 0;
				//System.Device.Uart5.SendDirectly(BD_TTS_START_ENGINE, strlen(BD_TTS_START_ENGINE));
			}
		}
	}
	
	if (Status_Self_Check == 0) {
                            
        play_once = 0;
        last_time_tts = getCurrSysTick();
        
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
			Status_Self_Check = 4;
			
		} else if (App.Data.CarLockStatus == 1) {
			Status_Self_Check = 3;	// 锁车
			System.Device.Uart5.SendDirectly(BD_TTS_LOCKED, strlen(BD_TTS_LOCKED));
		}				
	} else if (Status_Self_Check == 3) {  // 检测到锁车的情况
		
		if (App.Data.CarLockStatus == 0) 
			Status_Self_Check = 4;
		
	} else if (Status_Self_Check == 4) {
		
        play_once = 0;
        
		if (App.Data.FingerprintStatus == 0) {// 什么模式不需要检测指纹
			System.Device.Uart5.SendDirectly(BD_TTS_START_ENGINE, strlen(BD_TTS_START_ENGINE));
			Status_Self_Check = CHECK_OK;
		} else {
			System.Device.Uart5.SendDirectly(BD_TTS_PLEASE_VERIFY, strlen(BD_TTS_PLEASE_VERIFY)); // 请按指纹
			//发送获取指纹的命令
			FingerprintMatch = 0;
			//App.Data.FingerprintState = 0;
			Status_Self_Check = 5;            
		}
	} else if (Status_Self_Check == 5) {
		
		if (App.Data.FingerprintStatus == 0){
			Status_Self_Check = CHECK_OK;
            last_time_tts = getCurrSysTick();
			//System.Device.Uart5.SendDirectly(BD_TTS_START_ENGINE, strlen(BD_TTS_START_ENGINE));
		}
		
		if (FingerprintMatch == 1) {	// 指纹验证正确
			Status_Self_Check = CHECK_OK;
			//FingerprintMatch = 0;
			//System.Device.Uart5.SendDirectly(BD_TTS_START_ENGINE, strlen(BD_TTS_START_ENGINE));            
            last_time_tts = getCurrSysTick();
			//App.Data.FingerprintState = 2;           		
            
			//System.Device.Usart1.SendDirectly("checkok\r\n", strlen("checkok\r\n"));
		} else if (FingerprintMatch == 2) {	// 指纹验证不通过
			//FingerprintMatch = 1;
			//System.Device.Uart5.SendDirectly(BD_TTS_FAIL_VERIFY, strlen(BD_TTS_FAIL_VERIFY));
			//App.Data.FingerprintState = 2;
		}
	}  else if (Status_Self_Check == CHECK_OK) {	
        
        if (App.Data.CarMode == 0) { // 自由模式
            
            if (play_once == 0) {            
                System.Device.Uart5.SendDirectly(BD_TTS_START_ENGINE, strlen(BD_TTS_START_ENGINE));
                play_once = 2;
            }
            
        } else {
            
            if (play_once == 0) {            
                
                if (getCurrSysTick() - last_time_tts >= 3000) {
                    if (FingerprintMatch == 1) {	// 指纹验证正确
                        System.Device.Uart5.SendDirectly(BD_TTS_VERIFY_DONE, strlen(BD_TTS_VERIFY_DONE));	                    
                        last_time_tts = getCurrSysTick();
                        play_once = 1;                
                    }                
                }
                
            } else if (play_once == 1) {
                
                if (getCurrSysTick() - last_time_tts >= 3000) { // 3秒后发送请启动发动机
                    System.Device.Uart5.SendDirectly(BD_TTS_START_ENGINE, strlen(BD_TTS_START_ENGINE));
                    play_once = 2;
                }
                
            }
        }
        
    }
}


extern unsigned char usart1_buf[];
extern volatile unsigned int usart1_index;
char tmphead[] = {0xaa, 0x01, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff};

#ifdef NICE_CMD_FORMART
char finger[] = {
	0x36, 0x37, 0x3D, 0x38, 0x3B, 0x30, 0x3E, 0x3D, 0x3A, 0x39, 0x34, 0x35, 0x32, 0x32, 0x38, 0x35, 
	0x3E, 0x36, 0x39, 0x38, 0x32, 0x39, 0x37, 0x35, 0x36, 0x3F, 0x3F, 0x3A, 0x3F, 0x37, 0x3A, 0x33, 
	0x33, 0x34, 0x3D, 0x3E, 0x33, 0x3F, 0x3E, 0x3A, 0x3A, 0x3A, 0x33, 0x32, 0x35, 0x35, 0x3C, 0x38, 
	0x39, 0x3F, 0x35, 0x3B, 0x3E, 0x3E, 0x31, 0x38, 0x30, 0x36, 0x31, 0x39, 0x31, 0x34, 0x30, 0x3A, 
	0x39, 0x3D, 0x33, 0x35, 0x3D, 0x34, 0x34, 0x31, 0x30, 0x31, 0x39, 0x39, 0x3F, 0x3E, 0x3C, 0x33, 
	0x39, 0x34, 0x31, 0x32, 0x38, 0x37, 0x33, 0x31, 0x32, 0x3F, 0x37, 0x30, 0x37, 0x3D, 0x3E, 0x33, 
	0x37, 0x34, 0x35, 0x3C, 0x3B, 0x3D, 0x3A, 0x38, 0x3E, 0x38, 0x3B, 0x30, 0x3D, 0x37, 0x39, 0x3A, 
	0x38, 0x3C, 0x36, 0x35, 0x3C, 0x3B, 0x30, 0x37, 0x3E, 0x3D, 0x3B, 0x3C, 0x3E, 0x31, 0x30, 0x35, 
	0x38, 0x36, 0x30, 0x31, 0x37, 0x36, 0x33, 0x3D, 0x36, 0x33, 0x3B, 0x3B, 0x37, 0x3C, 0x35, 0x3B, 
	0x3C, 0x32, 0x33, 0x36, 0x3A, 0x3A, 0x37, 0x3A, 0x36, 0x30, 0x3F, 0x32, 0x3B, 0x34, 0x34, 0x31, 
	0x33, 0x39, 0x3C, 0x32, 0x3A, 0x36, 0x3D, 0x38, 0x30, 0x3C, 0x36, 0x3C, 0x3D, 0x33, 0x3D, 0x31, 
	0x39, 0x30, 0x39, 0x3E, 0x37, 0x3B, 0x3A, 0x36, 0x31, 0x35, 0x3B, 0x30, 0x3E, 0x35, 0x30, 0x30, 
	0x39, 0x32, 0x3E, 0x3B, 0x31, 0x36, 0x34, 0x3C, 0x36, 0x3B, 0x3D, 0x30, 0x32, 0x31, 0x32, 0x37, 
	0x39, 0x3F, 0x33, 0x35, 0x3D, 0x3C, 0x33, 0x31, 0x3A, 0x39, 0x37, 0x32, 0x39, 0x37, 0x36, 0x36, 
	0x36, 0x35, 0x3D, 0x3D, 0x3D, 0x35, 0x39, 0x3B, 0x3A, 0x35, 0x3D, 0x3F, 0x3B, 0x34, 0x3F, 0x33, 
	0x32, 0x39, 0x34, 0x3F, 0x37, 0x37, 0x3E, 0x3B, 0x3A, 0x3A, 0x36, 0x34, 0x37, 0x34, 0x3F, 0x34, 
	0x38, 0x36, 0x3E, 0x3D, 0x35, 0x38, 0x35, 0x33, 0x31, 0x3E, 0x34, 0x35, 0x3B, 0x37, 0x3C, 0x31, 
	0x39, 0x32, 0x3E, 0x35, 0x3D, 0x34, 0x33, 0x3E, 0x3E, 0x3D, 0x39, 0x3E, 0x35, 0x3C, 0x3A, 0x3A, 
	0x3F, 0x31, 0x39, 0x3E, 0x3A, 0x39, 0x3E, 0x31, 0x34, 0x3E, 0x3F, 0x39, 0x35, 0x30, 0x32, 0x35, 
	0x38, 0x37, 0x39, 0x33, 0x33, 0x34, 0x3A, 0x3B, 0x3C, 0x32, 0x33, 0x3B, 0x36, 0x31, 0x3F, 0x36, 
	0x34, 0x35, 0x36, 0x3D, 0x38, 0x3C, 0x39, 0x39, 0x3D, 0x39, 0x38, 0x31, 0x3E, 0x36, 0x3B, 0x3B, 
	0x3E, 0x3C, 0x36, 0x38, 0x3D, 0x3D, 0x36, 0x3B, 0x37, 0x35, 0x32, 0x3A, 0x32, 0x37, 0x3B, 0x39, 
	0x32, 0x3E, 0x30, 0x36, 0x3E, 0x37, 0x3F, 0x32, 0x3B, 0x32, 0x3E, 0x3A, 0x38, 0x3D, 0x3D, 0x30, 
	0x38, 0x37, 0x30, 0x33, 0x3B, 0x36, 0x30, 0x30, 0x31, 0x3E, 0x34, 0x31, 0x34, 0x3C, 0x3D, 0x32, 
	0x34, 0x35, 0x36, 0x3D, 0x38, 0x3C, 0x39, 0x39, 0x3D, 0x39, 0x38, 0x31, 0x3E, 0x36, 0x3B, 0x3B, 
	0x3E, 0x3C, 0x36, 0x38, 0x3D, 0x3D, 0x36, 0x3B, 0x37, 0x35, 0x32, 0x3A, 0x32, 0x37, 0x3B, 0x39, 
	0x32, 0x3E, 0x30, 0x36, 0x3E, 0x37, 0x3F, 0x32, 0x3B, 0x32, 0x3E, 0x3A, 0x38, 0x3D, 0x3D, 0x30, 
	0x38, 0x37, 0x30, 0x33, 0x3B, 0x36, 0x30, 0x30, 0x31, 0x3E, 0x34, 0x31, 0x34, 0x3C, 0x3D, 0x32, 
	0x34, 0x35, 0x36, 0x3D, 0x38, 0x3C, 0x39, 0x39, 0x3D, 0x39, 0x38, 0x31, 0x3E, 0x36, 0x3B, 0x3B, 
	0x3E, 0x3C, 0x36, 0x38, 0x3D, 0x3D, 0x36, 0x3B, 0x37, 0x35, 0x32, 0x3A, 0x32, 0x37, 0x3B, 0x39, 
	0x32, 0x3E, 0x30, 0x36, 0x3E, 0x37, 0x3F, 0x32, 0x3B, 0x32, 0x3E, 0x3A, 0x38, 0x3D, 0x3D, 0x30, 
	0x38, 0x37, 0x30, 0x33, 0x3B, 0x36, 0x30, 0x30, 0x31, 0x3E, 0x34, 0x31, 0x34, 0x3C, 0x31, 0x3E
};
char _search[] = {0x02, 0x30 ,0x30, 0x30, 0x38, 0x31, 0x37, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3F, 0x3F, 0x30, 0x30, 0x30, 0x30, 0x3E, 0x30, 0x03};
char _write_flash[] = {0x02, 0x30, 0x30, 0x30, 0x34, 0x31, 0x35, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x31, 0x31, 0x03};	

void usart1_cmd_process(void)
{
	unsigned char *cmd = usart1_buf;
	int i; 
	unsigned char crc = 0;
	char tail[2];
	
	if (!strncmp("cmdsc\r\n", cmd, strlen("cmdsc\r\n"))) {
		System.Device.Uart4.SendDirectly(_search, sizeof(_search));
		System.Device.Usart1.SendDirectly("ack\r\n", strlen("ack\r\n"));
		goto _clear;
	} else if (!strncmp("cmdwf\r\n", cmd, strlen("cmdwf\r\n"))) {
		System.Device.Uart4.SendDirectly(_write_flash, sizeof(_write_flash));
		goto _clear;
	} else if (!strncmp("cmddf\r\n", cmd, strlen("cmddf\r\n"))) {
		char head[13] = {0x02, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x02,0x00};
		System.Device.Usart1.SendDirectly("ack\r\n", strlen("ack\r\n"));
		for (i=1; i<13; i++)
			crc ^= head[i];
		for (i=0; i<512; i++)
			crc ^= finger[i];
		
		tail[0] = crc; tail[1] = 0x03;
		System.Device.Uart4.SendDirectly(head, 13);
		System.Device.Uart4.SendDirectly(finger, 512);
		System.Device.Uart4.SendDirectly(tail, 2);
		goto _clear;
	}
	
	goto _exit;	
_clear:
	usart1_index = 0;
	memset(usart1_buf, 0, 1024);	
_exit:
	
}

#else

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
	
	android_watchdog(1); // feed watchdog.
	
	if (cmd == 0x08) {	// iap
		
		crc = dat[9];
		JumpToBootloader();
		
	} else if (cmd == 0xF2) {	// beidou
		
		unsigned short len808 = (dat[10]<<8) | dat[9];
		System.Device.Uart5.SendDirectly(&dat[11], len808);
		
		clear = 1;
	} else if (cmd == 0xF1) {
		// 高4位是指纹数目 低4位是状态0 1 2
		
		if (dat[11]) {  
			App.Data.FingerprintStatus = dat[12]; // App.Data.FingerprintStatus 保存指纹数目
			if(System.Device.Storage.Parameter.Write(&App.Data.FingerprintStatus) == false)
			{
				System.Device.Storage.Parameter.Reorganize();
				App.Data.FingerprintStatus = dat[12];
				System.Device.Storage.Parameter.Write(&App.Data.FingerprintStatus);
			} 
		}
		
		if (dat[13]) {
			finger_down_flag = dat[14];
			//if (finger_down_flag == 0)
				//finger_down_flag = 1;
			//if (Status_Self_Check == CHECK_OK)
				//finger_down_flag = 2;
		}
		
		clear = 1;		
	} else if (cmd == 0xF0) { // 指纹
		unsigned short len808 = (dat[10]<<8) | dat[9];
		System.Device.Uart4.SendDirectly(&dat[11], len808);
		
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
	} else if (cmd == 0x0F) { // 查询指纹ID
		sendtoT2_protocol808(&FingerId[1], FingerId[0], 0x9f07);
		
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

#endif

bool isIntervalTimeout(uint lastSystick,uint interval){
	return (getCurrSysTick()-lastSystick) >= interval;
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
	unsigned char buf[12];
	
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
	
    //buf[2] = (byte)(App.Data.FingerprintStatus);
    buf[2] = (byte)FingerprintMatch;  // 1 表示成功  2表示失败  0表示初值
    buf[3] = (byte)(App.Data.IgnitionStatus);                                                                                                                                                                                                                                   
    buf[4] = (byte)(AppDataPointer->ReverseGear);
    buf[5] = (byte)(AppDataPointer->Direction & 0xFF);
    buf[6] = (byte)((AppDataPointer->Direction>>8) & 0xFF);
    buf[7] = (byte)(AppDataPointer->Level1Status & 0xFF);
    buf[8] = (byte)(AppDataPointer->Level1Status >> 8);
	buf[9] = (byte)Status_Self_Check;
	buf[10] = (byte)finger_down_flag;
	buf[11] = (byte)App.Data.FingerprintStatus;
	
    sendtoT2_protocol808(buf, 12, 0x9f02);
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
			
			if (gpio_get(GPIO_IN_ACC_DET) == 0)   //ACC接通，该脚读到0，否则读到1
				buf[10] &= ~0x80;	// ACC ON
			else
				buf[10] |= 0x80;	// ACC OFF			
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
	if(Status_Self_Check != CHECK_OK){
		return false;
	}
	
	return gAccState == ACC_ON;
}
	
void trigHeartbeat(){
	doHeartbeat(isHeartbeatOn());
	return;
}

#define AccStatusOnConfirm  300
#define AccStatusOffConfirm 1000

static int acc_systick100_routine(void)
{
    static bool Scan   = false;
    static bool Switch = false;
    static ushort AccOnCounter = 0;
	static ushort AccOffCounter = 0;
	
    Switch = ~Switch;    
    if(Switch)
    {
        //if(*pPinAccDet == 0)            
		if (gpio_get(GPIO_IN_ACC_DET) == 0) //ACC接通，该脚读到0，否则读到1
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
				return ACC_ON;
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
				return ACC_OFF;
            }
        }
    }
	
	return ACC_UNKNOWN;
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
	
	if(isIntervalTimeout(lastACCtime,10)){
		lastACCtime = getCurrSysTick();
		sensor_lift_detect();	// 举升io检测
		sensor_cape_detect();	// 篷布io检测
		//RtcSystick100Routine();	// RTC update.
		return acc_systick100_routine();
	}
	
	return ACC_UNKNOWN;
}

#define POWER_ON_POWEROFF 1
#define POWER_ON_POWERON_RESET_LOW 2
#define POWER_ON_POWERON_RESET_HIGH 3
#define POWER_OFF_WAIT 4
#define POWER_OFF_POWEROFF 5
#define POWER_OFF_POWEROFF_OVER 6

void android_watchdog(int feed)
{
	static unsigned int last_feed_time = 0;
	//static unsigned int last_hold_time = 0;
	//static int hold_time = 0;
	
	if (feed) {	// 喂狗
		last_feed_time = getCurrSysTick();
		return;
	}
	
	if (isIntervalTimeout(last_feed_time, ANDROID_WATCHDOG_TIME)) { // 5分钟没喂狗重启
		
		boot_android(2);
		last_feed_time = getCurrSysTick();	// 更新一下时间不然会一直重启
		
		//System.Device.Misc.T2ResetCtrl(true);	// reset io		
		//last_hold_time = getCurrSysTick();	// 开始计算hold时间
		//hold_time = 1000 * 3;				// 开始计算hold时间, 3秒后释放reset io
	}
	
// 	if (hold_time != 0) {
// 		if (isIntervalTimeout(last_hold_time, hold_time)) {
// 			System.Device.Misc.T2ResetCtrl(false);
// 			hold_time = 0;
// 			
// 			last_feed_time = getCurrSysTick();  // 从现在开始计时5分钟喂狗
// 		}
// 	}
}

// signal=1 开机    signal=2 关机
static int boot_android(int signal)
{
	static unsigned int last_time_power = 0;
	static unsigned int last_time_shutdown = 0;
	static int power = 0;
	static int shutdowning = 0;
		
	if (signal == 1) { // 开机
		if (power == 0) {
			if (shutdowning == 0) {
				power = 1;		
				System.Device.Misc.AcinSwitch(false);	// ac power = 0
				last_time_power = getCurrSysTick();
				return 0;
			}
		}		
		
	} else if (signal == 2) { // 关机
		if (shutdowning == 0){
			if (power == 3) {				
				byte shutDownMessage[] = {0xAA, 0x01, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xEE, 0x06, 0xBD};
				sendtoT2_protocol808(shutDownMessage, sizeof(shutDownMessage), 0x9f04);

				System.Device.Misc.AVPowerCtrl(1);
				System.Device.Usart1.Close();
				
				shutdowning = 1;
				last_time_shutdown = getCurrSysTick();
				return 0;
			}			
		}
	}
	
	if (shutdowning == 1) {
		
		if (isIntervalTimeout(last_time_shutdown, 10000)) { // 关机等10秒后断电

			System.Device.Misc.AcinSwitch(false);
			power = 0;
			shutdowning = 0;			
			last_time_shutdown = getCurrSysTick();
		}
		
	}
	
	
	if (power == 1) {
		//电源已经关闭 等待2秒后重新打开电源
		if (isIntervalTimeout(last_time_power, 2000)) { 
			
			System.Device.Misc.AcinSwitch(true);	// ac power = 1
			System.Device.Misc.T2ResetCtrl(true);   // reset 拉低2秒为了可靠启动		
			
			power = 2;
			last_time_power = getCurrSysTick();
		}
		
	} else if (power == 2) { 
		// 开电后并且reset也拉低了2秒之后，恢复reset
		if (isIntervalTimeout(last_time_power, 2000)) {
			
			System.Device.Misc.T2ResetCtrl(false);
			System.Device.Misc.AVPowerCtrl(0);
			System.Device.Usart1.Open();
			
			android_watchdog(1);
			
			power = 3;
			last_time_power = getCurrSysTick();
		}
		
	}
	
	return 0;
}






#define Rxd4BufferSum   666
extern unsigned char Rxd4Buffer[];
extern unsigned char FingerBuffer[];



void finger_process(void)  // 指纹仪处理
{
    static unsigned int last_time_check_ack = 0;    
	static unsigned int last_time_sample = 0;
	static char flag_response = 0;
	static char cnt_idle = 0;
    
	if((Status_Self_Check == 5) && (finger_down_flag != 2)) { // 如果自检没有完成 间隔3秒发一次指纹采集命令
		byte cmd[] = {0x02, 0x30, 0x30, 0x30, 0x38, 0x31, 0x37, 0x30, 0x30, 0x30, \
								0x30, 0x30, 0x30, 0x30, 0x30, 0x3F, 0x3F, 0x30, 0x30, 0x30, 0x30, 0x3E, 0x30, 0x03 };
		
		if (flag_response) {
			if (FingerprintMatch == 2 && finger_down_flag == 0) {
                if (getCurrSysTick() - last_time_sample <= 2500) {
                    System.Device.Uart5.SendDirectly(BD_TTS_FAIL_VERIFY, strlen(BD_TTS_FAIL_VERIFY));
                    cnt_idle = 0;                    
                } else {
                    if (cnt_idle++ >= 10) {
                        System.Device.Uart5.SendDirectly(BD_TTS_PLEASE_VERIFY, strlen(BD_TTS_PLEASE_VERIFY));
                        cnt_idle = 0;                        
                    }
                }
			}

			last_time_sample = getCurrSysTick();
			System.Device.Uart4.SendDirectly(cmd, sizeof cmd);
				
			flag_response = 0;	
								
		} else {
		
			if (isIntervalTimeout(last_time_sample, 5000)) { 
				
				last_time_sample = getCurrSysTick();
				System.Device.Uart4.SendDirectly(cmd, sizeof cmd);
				
				flag_response = 0;
			}		
		}
	}
	
	if (isIntervalTimeout(last_time_check_ack, 50)) {  // 检查一下指纹返回的数据
		int l = 0, len;
		
		last_time_check_ack = getCurrSysTick();
		
		// 收到指纹仪响应
		if (FingerBuffer[0] != 0x02) return;			
		
		for(l=0; l<Rxd4BufferSum; l++) {
			if (FingerBuffer[l] == 0x03)
				break;
		}
		len = l + 1;
		if (l >= Rxd4BufferSum) {
			memset(FingerBuffer, 0, Rxd4BufferSum);
			return;
		}
		
		//System.Device.Usart1.SendDirectly(FingerBuffer, len);
		flag_response = 1;
		
		// if(Status_Self_Check == 5) {
		if ((finger_down_flag == 0) || (finger_down_flag == 1)) {
			if (FingerBuffer[5] == 0x30 && FingerBuffer[6] == 0x30) {	// 指纹验证通过
				FingerprintMatch = 1;
				memset(FingerId, 0, sizeof(FingerId));
				if (len < sizeof(FingerId)-1) {
					FingerId[0] = len;
					memcpy(&FingerId[1], FingerBuffer, len);
				}
			} else {
				FingerprintMatch = 2;				
			}
			
			if (finger_down_flag == 1) {
				finger_down_flag = 2;
			}
			
			if (Status_Self_Check == CHECK_OK) { 
				finger_down_flag = 2;
			}
				
			
		} else { // if (Status_Self_Check == CHECK_OK) {

			if (len < Rxd4BufferSum)
				sendtoT2_protocol808(FingerBuffer, len, 0x9f06);

		}
		
		memset(FingerBuffer, 0, Rxd4BufferSum);
	}

}
void LogicTask(void)
{
    uint message;
    uint data; 
    bool res;
	    
	InitLogic(); 	// can filter init.     

    	
	android_watchdog(1);	// 设置初值
	//auto_start_andoird_process(1);	// 设置初值
	//System.Device.Uart4.Register((uint)Uart4RevData);
    System.Device.Uart4.Open();
	
	
	
	
    while(true)
    {
		//updateLogicTaskTime();
		int accState = acc_onoff_process();
		if(accState != ACC_UNKNOWN){
			if (gAccState != accState) {
				gAccState = accState;
				if (accState == ACC_ON) {
					App.Data.AccStatus = on;
					Beidou_GPS_Antenna_Off = 100;
					finger_down_flag = 0;
					memset(FingerId, 0, sizeof(FingerId));
					//System.Device.Uart5.SendDirectly("cmd=6;msg=3830;;", strlen("cmd=6;msg=3131;;"));
					Status_Self_Check = 0;	// 自检                    
				} else {
					App.Data.AccStatus = off;
					Beidou_GPS_Antenna_Off = 100;
					Status_Self_Check = CHECK_INIT;                    
					//System.Device.Uart5.SendDirectly("cmd=6;msg=3930;;", strlen("cmd=6;msg=3131;;"));
				}
			}
			
		}
		boot_android(gAccState);
		finger_process();
		usart1_cmd_process();	// 从android发过来的命令
		ParseBeidouData();		// 处理uart5的命令响应
		
		trigHeartbeat();
		processLimitspeed();
		
		processCarLevel();
		self_check_process();
		
#ifdef ENABLE_ANDROID_WATCHDOG
		android_watchdog(0);	// 处理安卓的看门狗 默认5分钟
#endif
		//auto_start_andoird_process(0);	// 定时开机功能
		
    } 
}
