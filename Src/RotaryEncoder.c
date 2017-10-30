//rotary encoder driver
#include "RotaryEncoder.h"

SpinProcessTypeDef proceeSpinFuncList;

// 初始化旋转编码器的状态
void Init_Spin_Status(struspin *spin)
{
	spin->rotatestatus = SPIN_NO_ROTATE;
	spin->keystatus = KEY_NO_PRESSED;
}

// 读取旋转编码器的状态
static LEVEL last_spinkey_level= H; // 保存的上一次按键的电平情况
static SpinABstatus last_spinABst=SPIN_AB_ST3; // 保存的上一次AB引脚的电平情况
void Read_Spin(struspin *spin)
{
	LEVEL now_spinkey_level;
	LEVEL now_spinA_level;
	LEVEL now_spinB_level;
	SpinABstatus now_spinABst;
	// 先读取旋转编码器的各引脚的电平值
	if( H == READ_SPIN_KEY() ) { now_spinkey_level = H; } // 读取旋转编码器的按键电平值
	else { now_spinkey_level = L; }


	if( H == READ_SPIN_A() ) { now_spinA_level = H; } // 读取旋转编码器的 A 信号电平值
	else { now_spinA_level = L; }

	if( H == READ_SPIN_B() ) { now_spinB_level = H; } // 读取旋转编码器的 B 信号电平值
	else { now_spinB_level = L; }

	// 根据旋转编码器的按键 前后 两次的电平值, 确定按键状态
	if( (last_spinkey_level == H) && (now_spinkey_level == L) ) { spin->keystatus = KEY_JUST_PRESSED; } 
	else if((last_spinkey_level == H) && (now_spinkey_level == H) ) { spin->keystatus = KEY_NO_PRESSED; } 
	else if((last_spinkey_level == L) && (now_spinkey_level == H) ) { spin->keystatus = KEY_JUST_POPUP; } 
	else { spin->keystatus = KEY_ALREADY_PRESSED; }

	// 根据旋转编码器的 A B 的电平值, 确定 AB 状态
	if( (now_spinA_level == H) && (now_spinB_level == H) ) { now_spinABst = SPIN_AB_ST3; } // A=1 B=1; 
	else if((now_spinA_level == H) && (now_spinB_level == L) ) { now_spinABst = SPIN_AB_ST2; } // A=1 B=0;
	else if((now_spinA_level == L) && (now_spinB_level == L) ) { now_spinABst = SPIN_AB_ST0; } // A=0 B=0;
	else { now_spinABst = SPIN_AB_ST1; } // A=0 B=1;

	// 再根据旋转编码器 前后两次 AB信号线的状态, 确定是左旋还是右旋
	if( (last_spinABst == SPIN_AB_ST3) && (now_spinABst == SPIN_AB_ST2)){ spin->rotatestatus = SPIN_UNTICLOCKWISE; }
	else if ((last_spinABst == SPIN_AB_ST3) && (now_spinABst == SPIN_AB_ST1)){ spin->rotatestatus = SPIN_CLOCKWISE; }
	else { spin->rotatestatus = SPIN_NO_ROTATE; }

	last_spinkey_level = now_spinkey_level; // 保存当前状态，作为下次状态的依据
	last_spinABst = now_spinABst;

}

// 注册用户自己的代码
void RegisterForProcess_Spin( SpinProcessTypeDef *funlist)
{
	proceeSpinFuncList.pFunSpinkeypress = funlist->pFunSpinkeypress;
	proceeSpinFuncList.pFunSpinCW = funlist->pFunSpinCW;
	proceeSpinFuncList.pFunSpinCCW = funlist->pFunSpinCCW;	
}

// 旋转编码器处理
void Process_Spin(struspin *spin)
{
	if(spin->keystatus == KEY_JUST_PRESSED) // 旋转编码器的按键刚按下的处理程序，用户自己填入
	{
		proceeSpinFuncList.pFunSpinkeypress();
	}
	else if(spin->rotatestatus == SPIN_CLOCKWISE) // 旋转编码器顺时针旋转的处理程序，用户自己填入
	{
		proceeSpinFuncList.pFunSpinCW();
	}
	else if(spin->rotatestatus == SPIN_UNTICLOCKWISE) // 旋转编码器反时针旋转的处理程序，用户自己填入
	{
		proceeSpinFuncList.pFunSpinCCW();
	} 
}
