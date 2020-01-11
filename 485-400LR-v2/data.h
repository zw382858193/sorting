/*
********************************************************************************
*                                Ƕ��ʽ΢ϵͳ
*                                   msOS
*
*                            Ӳ��ƽ̨:msPLC DEMO
*                          ��оƬ:STM32F103R8T6/RBT6
*                           ��������οƼ����޹�˾
*
*                                ����:����ΰ
*                                ����:������
*                                ��ʶ:Wangsw
*
*                                QQ:26033613
*                               QQȺ:291235815
*                        ��̳:http://bbs.huayusoft.com
*                        �Ա���:http://52edk.taobao.com
*                ����:http://forum.eet-cn.com/BLOG_wangsw317_1268.HTM
********************************************************************************
*�ļ���     : data.h
********************************************************************************
*�汾     ����            ����            ˵��
*V0.1    Wangsw        2015/08/29       ��ʼ�汾
********************************************************************************
*/


#ifndef __DATA_H
#define __DATA_H
#include "system.h"

//һ��Ҫ���ƺ���Ҫ����Ĳ������������ܳ���256��������治��
typedef struct
{    
    uint FingerLogTime;             //���ڱ����ָ�Ƶ�ʱ��㣬��ָ��ʱ�����ʹ�ã���ȷ��һ��ָ�Ƶ�ʱ����Ҫ��¼�´�ʱ�䣬���浽Flash�У������ʱ�򣬼�ȥ2000�꣬Ҫ��Ȼ��ʮ���������
    uint FingerprintTime;           //ָ��ʱ�䣬СʱΪ��λ����ģʽ2�£�����ָ��֮��೤ʱ���ڲ���Ҫ��ָ�ƣ�����ʱ����Ҫ���´�ָ�ƣ�ʱ��̫������bug��������˾�ܵ���ʱ�򲻵���Ҳ����
    uint CarMode;                   //0��ʾ����ģʽ��1��ʾģʽ1��2��ʾģʽ2��ģʽ1ÿ�ε��Ҫ��ȷ��ָ�ƣ�ģʽ2��ָ��ʱ���ڲ���Ҫ�ٴ�ָ��       
    uint CarLockStatus;             //����״̬��������뱣�棬1��ʾ������0��ʾ������

    uint Level0Status;              //���١�ת�١�Ť�ء����š��ͺ������Ƿ���Ч���Լ��Ƿ���û���յ����ݣ�����ÿһ�����ݾ͵�����λ����ʾ����Ƭ���ϵ�Ĭ���������ݶ�����Ч��
    uint Level1Status;              //���������ǡ�ָ��״̬�����״̬������������ת��
    uint Level2Status;              //�汾�š�ģʽ������״̬������״̬���޾�״̬��ACC״̬
    
    uint EngineSpeed;               //������ת��
    uint Torque;                    //Ť��
    uint CarSpeed1;                 //����(��һ)
    uint CarSpeed2;                 //���� ����ŷ������
    uint LiftSignal;                //�����źţ�����0��ʾ����������Ϊƽ��
    uint CapeSignal;                //�����źţ�����1��ʾ����򿪣�����Ϊ�ر�
    //uint LiftState;               //ͨ��IO��ȡ����״̬����������Ӧ�ü�⵽�ߵ�ƽ��������ȥӦ�ü�⵽�͵�ƽ
    //uint CapeState;               //ͨ��IO��ȡ����״̬���񲼸���Ӧ�ü�⵽�͵�ƽ���񲼴�Ӧ�ü�⵽�ߵ�ƽ
    uint Accelerator;               //����
    uint ReverseGear;               //����0x02Ϊ����
    uint Brake;                     //ɲ��
    uint Direction;                 //����ת��ƣ�Զ��ƣ�ǰ�����
    uint FuelConsumption;           //ƽ���ͺ�

    uint LiftIOStatus;              //ͨ��IO��ȡ���ľ���״̬
    uint CapeIOStatus;              //ͨ��IO��ȡ���Ķ���״̬
    uint AccStatus;                 //ON����OFF
    uint AdroidStatus;
    uint IgnitionStatus;            //���״̬
    uint SpeedLimitStatus;          //����״̬
    uint LiftLimitStatus;           //�޾�״̬
    uint FingerprintStatus;         //ָ��״̬

    uint TimerStartupTime;          //��ʱ����ʱ�䣬��λ�Ƿ���
    uint LaserPowerStatus;          //�����๩��״̬

    uint CarType;                   //0��ʾ��һ���ͣ�1��ʾ����ŷ������
    uint Version;

    uint FingerprintState;          //0:����ģʽ��1:ָ������ģʽ��2:Ҫ�����ָ������ģʽ

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

