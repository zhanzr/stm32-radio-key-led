//rotary encoder driver
#include "RotaryEncoder.h"

SpinProcessTypeDef proceeSpinFuncList;

// ��ʼ����ת��������״̬
void Init_Spin_Status(struspin *spin)
{
	spin->rotatestatus = SPIN_NO_ROTATE;
	spin->keystatus = KEY_NO_PRESSED;
}

// ��ȡ��ת��������״̬
static LEVEL last_spinkey_level= H; // �������һ�ΰ����ĵ�ƽ���
static SpinABstatus last_spinABst=SPIN_AB_ST3; // �������һ��AB���ŵĵ�ƽ���
void Read_Spin(struspin *spin)
{
	LEVEL now_spinkey_level;
	LEVEL now_spinA_level;
	LEVEL now_spinB_level;
	SpinABstatus now_spinABst;
	// �ȶ�ȡ��ת�������ĸ����ŵĵ�ƽֵ
	if( H == READ_SPIN_KEY() ) { now_spinkey_level = H; } // ��ȡ��ת�������İ�����ƽֵ
	else { now_spinkey_level = L; }


	if( H == READ_SPIN_A() ) { now_spinA_level = H; } // ��ȡ��ת�������� A �źŵ�ƽֵ
	else { now_spinA_level = L; }

	if( H == READ_SPIN_B() ) { now_spinB_level = H; } // ��ȡ��ת�������� B �źŵ�ƽֵ
	else { now_spinB_level = L; }

	// ������ת�������İ��� ǰ�� ���εĵ�ƽֵ, ȷ������״̬
	if( (last_spinkey_level == H) && (now_spinkey_level == L) ) { spin->keystatus = KEY_JUST_PRESSED; } 
	else if((last_spinkey_level == H) && (now_spinkey_level == H) ) { spin->keystatus = KEY_NO_PRESSED; } 
	else if((last_spinkey_level == L) && (now_spinkey_level == H) ) { spin->keystatus = KEY_JUST_POPUP; } 
	else { spin->keystatus = KEY_ALREADY_PRESSED; }

	// ������ת�������� A B �ĵ�ƽֵ, ȷ�� AB ״̬
	if( (now_spinA_level == H) && (now_spinB_level == H) ) { now_spinABst = SPIN_AB_ST3; } // A=1 B=1; 
	else if((now_spinA_level == H) && (now_spinB_level == L) ) { now_spinABst = SPIN_AB_ST2; } // A=1 B=0;
	else if((now_spinA_level == L) && (now_spinB_level == L) ) { now_spinABst = SPIN_AB_ST0; } // A=0 B=0;
	else { now_spinABst = SPIN_AB_ST1; } // A=0 B=1;

	// �ٸ�����ת������ ǰ������ AB�ź��ߵ�״̬, ȷ����������������
	if( (last_spinABst == SPIN_AB_ST3) && (now_spinABst == SPIN_AB_ST2)){ spin->rotatestatus = SPIN_UNTICLOCKWISE; }
	else if ((last_spinABst == SPIN_AB_ST3) && (now_spinABst == SPIN_AB_ST1)){ spin->rotatestatus = SPIN_CLOCKWISE; }
	else { spin->rotatestatus = SPIN_NO_ROTATE; }

	last_spinkey_level = now_spinkey_level; // ���浱ǰ״̬����Ϊ�´�״̬������
	last_spinABst = now_spinABst;

}

// ע���û��Լ��Ĵ���
void RegisterForProcess_Spin( SpinProcessTypeDef *funlist)
{
	proceeSpinFuncList.pFunSpinkeypress = funlist->pFunSpinkeypress;
	proceeSpinFuncList.pFunSpinCW = funlist->pFunSpinCW;
	proceeSpinFuncList.pFunSpinCCW = funlist->pFunSpinCCW;	
}

// ��ת����������
void Process_Spin(struspin *spin)
{
	if(spin->keystatus == KEY_JUST_PRESSED) // ��ת�������İ����հ��µĴ�������û��Լ�����
	{
		proceeSpinFuncList.pFunSpinkeypress();
	}
	else if(spin->rotatestatus == SPIN_CLOCKWISE) // ��ת������˳ʱ����ת�Ĵ�������û��Լ�����
	{
		proceeSpinFuncList.pFunSpinCW();
	}
	else if(spin->rotatestatus == SPIN_UNTICLOCKWISE) // ��ת��������ʱ����ת�Ĵ�������û��Լ�����
	{
		proceeSpinFuncList.pFunSpinCCW();
	} 
}
