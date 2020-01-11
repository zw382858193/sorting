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
*文件名     : data.h
********************************************************************************
*版本     作者            日期            说明
*V0.1    Wangsw        2015/08/29       初始版本
********************************************************************************
*/


#ifndef __DATA_H
#define __DATA_H
#include "system.h"

//一定要控制好需要保存的参数数量，不能超过256个，否则存不下
typedef struct
{    
    uint FingerLogTime;             //用于保存打指纹的时间点，跟指纹时间配合使用，当确认一个指纹的时候，需要记录下此时间，保存到Flash中，计算的时候，减去2000年，要不然几十年后会有溢出
    uint FingerprintTime;           //指纹时间，小时为单位，在模式2下，打了指纹之后多长时间内不需要打指纹，超过时间需要重新打指纹，时间太长会有bug，不过公司能到那时候不倒闭也不错
    uint CarMode;                   //0表示自由模式，1表示模式1，2表示模式2，模式1每次点火都要先确认指纹，模式2在指纹时间内不需要再打指纹       
    uint CarLockStatus;             //锁车状态，这个必须保存，1表示锁车，0表示不锁车

    uint Level0Status;              //车速、转速、扭矩、油门、油耗数据是否有效，以及是否是没接收到数据，这样每一种数据就得用两位来表示，单片机上电默认所有数据都是无效的
    uint Level1Status;              //举升、顶盖、指纹状态、点火状态、倒车、左右转向
    uint Level2Status;              //版本号、模式、锁车状态、限速状态、限举状态、ACC状态
    
    uint EngineSpeed;               //发动机转速
    uint Torque;                    //扭矩
    uint CarSpeed1;                 //车速(三一)
    uint CarSpeed2;                 //车速 福田欧曼车型
    uint LiftSignal;                //举升信号，等于0表示举升，否则为平放
    uint CapeSignal;                //斗篷信号，等于1表示斗篷打开，否则为关闭
    //uint LiftState;               //通过IO读取到的状态，斗举起来应该检测到高电平，斗放下去应该检测到低电平
    //uint CapeState;               //通过IO读取到的状态，篷布盖上应该检测到低电平，篷布打开应该检测到高电平
    uint Accelerator;               //油门
    uint ReverseGear;               //等于0x02为倒车
    uint Brake;                     //刹车
    uint Direction;                 //左右转向灯，远光灯，前后雾灯
    uint FuelConsumption;           //平均油耗

    uint LiftIOStatus;              //通过IO读取到的举升状态
    uint CapeIOStatus;              //通过IO读取到的斗篷状态
    uint AccStatus;                 //ON或者OFF
    uint AdroidStatus;
    uint IgnitionStatus;            //点火状态
    uint SpeedLimitStatus;          //限速状态
    uint LiftLimitStatus;           //限举状态
    uint FingerprintStatus;         //指纹状态

    uint TimerStartupTime;          //定时开机时间，单位是分钟
    uint LaserPowerStatus;          //激光测距供电状态

    uint CarType;                   //0表示三一车型，1表示福田欧曼车型
    uint Version;

    uint FingerprintState;          //0:正常模式，1:指纹下载模式，2:要求进入指纹下载模式

    RtcStruct Rtc;    
}DataStruct;


typedef enum {
	GPIO_IN_ACC_DET,
	GPIO_IN_CAPE_DET,
	GPIO_IN_LIFT_DET,	
	GPIO_IN_CAR_POWER_DET,
	
	GPIO_OUT_ACIN_ONOFF,
	GPIO_OUT_LIFT_CTRL,
	GPIO_OUT_T2_RESET,
	GPIO_OUT_AV_POWER,
	GPIO_OUT_LASER_POWER
}GPIO_NAME;

unsigned int gpio_get(GPIO_NAME io);
void gpio_set(GPIO_NAME io, int value);

#endif /*__Data_H*/

