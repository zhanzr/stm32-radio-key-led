#ifndef _ROTARYENCODER_H_
#define _ROTARYENCODER_H_

#include "main.h"
#include "stm32f4xx_hal.h"

/////// 左面的旋转编码器 ///////////////
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
	L = GPIO_PIN_RESET, // 低电平
	H = GPIO_PIN_SET, // 高电平
}LEVEL;

typedef enum
{
	SPIN_NO_ROTATE = 0, // 无旋转
	SPIN_CLOCKWISE , // 顺时针旋转
	SPIN_UNTICLOCKWISE , // 反时针旋转
}RotateStatus;

typedef struct tagspin
{
	RotateStatus rotatestatus; // 旋转状态
	KeyStatus keystatus; // 按键状态
}struspin;

typedef enum
{
	SPIN_AB_ST0 = 0, // A=0 B=0;
	SPIN_AB_ST1 , // A=0 B=1;
	SPIN_AB_ST2 , // A=1 B=0;
	SPIN_AB_ST3 , // A=1 B=1; 
}SpinABstatus; // 旋转编码器信号线 A B 的电平

typedef struct _SpinProcessTypeDef
{
	void (*pFunSpinkeypress)(void);
	void (*pFunSpinCW)(void);
	void (*pFunSpinCCW)(void);
}SpinProcessTypeDef;

// 初始化旋转编码器的状态
void Init_Spin_Status(struspin *spin);

// 读取旋转编码器的状态
void Read_Spin(struspin *spin);

// 注册用户自己的代码
void RegisterForProcess_Spin( SpinProcessTypeDef *funlist);
	
// 旋转编码器，用户自己的代码
void Process_Spin(struspin *spin);

#endif
