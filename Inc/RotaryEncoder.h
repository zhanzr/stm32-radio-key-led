#ifndef _ROTARYENCODER_H_
#define _ROTARYENCODER_H_

#include "main.h"
#include "stm32f4xx_hal.h"

/////// �������ת������ ///////////////
#define SPIN_KEY_PORT	RK_GPIO_Port
#define SPIN_KEY_PIN		RK_Pin
#define SPIN_A_PORT		RA_GPIO_Port
#define SPIN_A_PIN		RA_Pin
#define SPIN_B_PORT		RB_GPIO_Port
#define SPIN_B_PIN		RB_Pin

#define READ_SPIN_KEY() HAL_GPIO_ReadPin(RK_GPIO_Port,RK_Pin)
#define READ_SPIN_A() HAL_GPIO_ReadPin(RA_GPIO_Port,RA_Pin)
#define READ_SPIN_B() HAL_GPIO_ReadPin(RB_GPIO_Port,RB_Pin)

typedef enum
{
	KEY_NO_PRESSED = 0,
	KEY_JUST_PRESSED ,
	KEY_ALREADY_PRESSED ,
	KEY_JUST_POPUP ,
}KeyStatus;

typedef enum
{
	L = GPIO_PIN_RESET, // �͵�ƽ
	H = GPIO_PIN_SET, // �ߵ�ƽ
}LEVEL;

typedef enum
{
	SPIN_NO_ROTATE = 0, // ����ת
	SPIN_CLOCKWISE , // ˳ʱ����ת
	SPIN_UNTICLOCKWISE , // ��ʱ����ת
}RotateStatus;

typedef struct tagspin
{
	RotateStatus rotatestatus; // ��ת״̬
	KeyStatus keystatus; // ����״̬
}struspin;

typedef enum
{
	SPIN_AB_ST0 = 0, // A=0 B=0;
	SPIN_AB_ST1 , // A=0 B=1;
	SPIN_AB_ST2 , // A=1 B=0;
	SPIN_AB_ST3 , // A=1 B=1; 
}SpinABstatus; // ��ת�������ź��� A B �ĵ�ƽ

typedef struct _SpinProcessTypeDef
{
	void (*pFunSpinkeypress)(void);
	void (*pFunSpinCW)(void);
	void (*pFunSpinCCW)(void);
}SpinProcessTypeDef;

// ��ʼ����ת��������״̬
void Init_Spin_Status(struspin *spin);

// ��ȡ��ת��������״̬
void Read_Spin(struspin *spin);

// ע���û��Լ��Ĵ���
void RegisterForProcess_Spin( SpinProcessTypeDef *funlist);
	
// ��ת���������û��Լ��Ĵ���
void Process_Spin(struspin *spin);

#endif
