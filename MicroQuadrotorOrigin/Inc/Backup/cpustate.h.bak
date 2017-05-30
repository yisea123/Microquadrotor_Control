#ifndef __CPUSTATE_H__
#define __CPUSTATE_H__
#include "bsp.h"
#define OS_TASK_STAT_EN	1//开启统计任务


#if OS_TASK_STAT_EN>0
#define ABS(x) (x)>=0?(x):(-x)//定义绝对值
/*-----------------------------------------------------------------------------------------*/

#define TaskStat_STACK_SIZE	( ( unsigned short ) 64 ) //统计任务的堆栈大小

#define tskIDLE_STACK_SIZE	configMINIMAL_STACK_SIZE
#define CULCULATE_PERIOD    1000//毫秒
/*-----------------------------------------------------------------------------------------*/
void  OSStatInit (void);
/*-----------------------------------------------------------------------------------------*/
extern  volatile  unsigned int  OSIdleCtr;                                 /* Idle counter                   */
extern  unsigned int      OSIdleCtrMax;             /* Max. value that idle ctr can take in 1 sec.     */
extern  unsigned int      OSIdleCtrRun;             /* Val. reached by idle ctr at run time in 1 sec.  */
extern float      OSCPUUsage;               /* Percentage of CPU used  */
extern  unsigned int      FreeHeapSpace;
extern float FreeHeapSpaceKB;
void prvOSStatInit(void * pvParameters);//统计任务
void OS_TaskStat(void);//统计任务
int StartupInfo(void);
u32 ClkFreq(void);
#endif
#endif /* __LWIPOPTS_H__ */
