#ifndef __UI_H__
#define __UI_H__

#include "stdint.h"
#include "main.h"
#include "uart.h"
//#include "flash.h"

void Set_Parameters(void);
void Set_Main_Parameters(void);
void Set_Auto_Mode_Hours(void);
void Set_Auto_Mode_Minutess(void);
void Set_Auto_Mode_Seconds(void);
void Update_Up_Down_Btn_count(void);
void Indicate_Actual_Movement(int data);
void Indicate_Execution_Time();
#endif
