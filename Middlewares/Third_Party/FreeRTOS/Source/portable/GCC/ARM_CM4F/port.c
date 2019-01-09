/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM4F port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

#ifndef __VFP_FP__
	#error This port can only be used when the project options are configured to enable hardware floating point support.
#endif

#ifndef configSYSTICK_CLOCK_HZ
	#define configSYSTICK_CLOCK_HZ configCPU_CLOCK_HZ
	/* Ensure the SysTick is clocked at the same frequency as the core. */
	#define portNVIC_SYSTICK_CLK_BIT	( 1UL << 2UL )
#else
	/* The way the SysTick is clocked is not modified in case it is not the same
	as the core. */
	#define portNVIC_SYSTICK_CLK_BIT	( 0 )
#endif

/* Constants required to manipulate the core.  Registers first... */
#define portNVIC_SYSTICK_CTRL_REG			( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG			( * ( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG	( * ( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SYSPRI2_REG				( * ( ( volatile uint32_t * ) 0xe000ed20 ) )
/* ...then bits in the registers. */
#define portNVIC_SYSTICK_INT_BIT			( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT			( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT		( 1UL << 16UL )
#define portNVIC_PENDSVCLEAR_BIT 			( 1UL << 27UL )
#define portNVIC_PEND_SYSTICK_CLEAR_BIT		( 1UL << 25UL )

/* Constants used to detect a Cortex-M7 r0p1 core, which should use the ARM_CM7
r0p1 port. */
#define portCPUID							( * ( ( volatile uint32_t * ) 0xE000ed00 ) )
#define portCORTEX_M7_r0p1_ID				( 0x410FC271UL )
#define portCORTEX_M7_r0p0_ID				( 0x410FC270UL )

#define portNVIC_PENDSV_PRI					( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 16UL )
#define portNVIC_SYSTICK_PRI				( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 24UL )

/* Constants required to check the validity of an interrupt priority. */
#define portFIRST_USER_INTERRUPT_NUMBER		( 16 )
#define portNVIC_IP_REGISTERS_OFFSET_16 	( 0xE000E3F0 )
#define portAIRCR_REG						( * ( ( volatile uint32_t * ) 0xE000ED0C ) )
#define portMAX_8_BIT_VALUE					( ( uint8_t ) 0xff )
#define portTOP_BIT_OF_BYTE					( ( uint8_t ) 0x80 )
#define portMAX_PRIGROUP_BITS				( ( uint8_t ) 7 )
#define portPRIORITY_GROUP_MASK				( 0x07UL << 8UL )
#define portPRIGROUP_SHIFT					( 8UL )

/* Masks off all bits but the VECTACTIVE bits in the ICSR register. */
#define portVECTACTIVE_MASK					( 0xFFUL )

/* Constants required to manipulate the VFP. */
#define portFPCCR							( ( volatile uint32_t * ) 0xe000ef34 ) /* Floating point context control register. */
#define portASPEN_AND_LSPEN_BITS			( 0x3UL << 30UL )

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR					( 0x01000000 )
#define portINITIAL_EXEC_RETURN				( 0xfffffffd )

/* The systick is a 24-bit counter. */
#define portMAX_24_BIT_NUMBER				( 0xffffffUL )

/* For strict compliance with the Cortex-M spec the task start address should
have bit-0 clear, as it is loaded into the PC on exit from an ISR. */
#define portSTART_ADDRESS_MASK		( ( StackType_t ) 0xfffffffeUL )

/* A fiddle factor to estimate the number of SysTick counts that would have
occurred while the SysTick counter is stopped during tickless idle
calculations. */
#define portMISSED_COUNTS_FACTOR			( 45UL )

/* Let the user override the pre-loading of the initial LR with the address of
prvTaskExitError() in case it messes up unwinding of the stack in the
debugger. */
#ifdef configTASK_RETURN_ADDRESS
	#define portTASK_RETURN_ADDRESS	configTASK_RETURN_ADDRESS
#else
	#define portTASK_RETURN_ADDRESS	prvTaskExitError
#endif

/* Each task maintains its own interrupt status in the critical nesting
variable. */
static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;

/*
 * Setup the timer to generate the tick interrupts.  The implementation in this
 * file is weak to allow application writers to change the timer used to
 * generate the tick interrupt.
 */
void vPortSetupTimerInterrupt( void );

/*
 * Exception handlers.
 */
void xPortPendSVHandler( void ) __attribute__ (( naked ));
void xPortSysTickHandler( void );
void vPortSVCHandler( void ) __attribute__ (( naked ));

/*
 * Start first task is a separate function so it can be tested in isolation.
 */
static void prvPortStartFirstTask( void ) __attribute__ (( naked ));

/*
 * Function to enable the VFP.
 */
static void vPortEnableVFP( void ) __attribute__ (( naked ));

/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError( void );

/*-----------------------------------------------------------*/

/*
 * The number of SysTick increments that make up one tick period.
 */
#if configUSE_TICKLESS_IDLE == 1
	static uint32_t ulTimerCountsForOneTick = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * 24 bit resolution of the SysTick timer.
 */
#if configUSE_TICKLESS_IDLE == 1
	static uint32_t xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * Compensate for the CPU cycles that pass while the SysTick is stopped (low
 * power functionality only.
 */
#if configUSE_TICKLESS_IDLE == 1
	static uint32_t ulStoppedTimerCompensation = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * Used by the portASSERT_IF_INTERRUPT_PRIORITY_INVALID() macro to ensure
 * FreeRTOS API functions are not called from interrupts that have been assigned
 * a priority above configMAX_SYSCALL_INTERRUPT_PRIORITY.
 */
#if ( configASSERT_DEFINED == 1 )
	 static uint8_t ucMaxSysCallPriority = 0;
	 static uint32_t ulMaxPRIGROUPValue = 0;
	 static const volatile uint8_t * const pcInterruptPriorityRegisters = ( const volatile uint8_t * const ) portNVIC_IP_REGISTERS_OFFSET_16;
#endif /* configASSERT_DEFINED */

/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
/*
	pxCode -- 创建任务时任务函数地址
	pxTopOfStack -- TCB里面栈顶，即最高地址位置
	将重要寄存器入栈
*/
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
	/* Simulate the stack frame as it would be created by a context switch
	interrupt. */

	/* Offset added to account for the way the MCU uses the stack on entry/exit
	of interrupts, and to ensure alignment. */
	pxTopOfStack--;
	//参看《The Definitive Guide to the ARM Cortex-M3》第24位永远是1 要不然会出错
	*pxTopOfStack = portINITIAL_XPSR;	/* xPSR */		//指令集？thumb
	pxTopOfStack--;
	//这里放PC指针。
	*pxTopOfStack = ( ( StackType_t ) pxCode ) & portSTART_ADDRESS_MASK;	/* PC */
	pxTopOfStack--;
	//LR			link register  R14 store the return program counter	//存你调用一个函数飞出去以后的下一行代码
	*pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS;	/* LR */

	/* Save code space by skipping register initialisation. */
	pxTopOfStack -= 5;	/* R12, R3, R2 and R1. */
	//存变量地址？
	*pxTopOfStack = ( StackType_t ) pvParameters;	/* R0 */

	/* A save method is being used that requires each task to maintain its
	own exec return value. */
	pxTopOfStack--;
	*pxTopOfStack = portINITIAL_EXEC_RETURN;
	//位置空出来了，可东西呢？没放进去呀？可能是实际任务调度的时候放进去。
	pxTopOfStack -= 8;	/* R11, R10, R9, R8, R7, R6, R5 and R4. */

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

static void prvTaskExitError( void )
{
	/* A function that implements a task must not exit or attempt to return to
	its caller as there is nothing to return to.  If a task wants to exit it
	should instead call vTaskDelete( NULL ).

	Artificially force an assert() to be triggered if configASSERT() is
	defined, then stop here so application writers can catch the error. */
	configASSERT( uxCriticalNesting == ~0UL );
	portDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/
/* 
	这里出栈顺序参考前面入栈顺序，pxPortInitialiseStack 函数中分配的地址
	先	 R11, R10, R9, R8, R7, R6, R5 and R4. 
	然后	R0
	然后	 R12, R3, R2 and R1.
	TopOfStack之后就是寄存器，地址递增
*/
void vPortSVCHandler( void )
{
	__asm volatile (
					"	ldr	r3, pxCurrentTCBConst2		\n" /* Restore the context. */	 /* pxCurrentTCBConst2指向处于最高优先级的就绪任务TCB */
					"	ldr r1, [r3]					\n" /* Use pxCurrentTCBConst to get the pxCurrentTCB address. */	/* 获取任务TCB地址 */
					"	ldr r0, [r1]					\n" /* The first item in pxCurrentTCB is the task top of stack. */	/* 获取任务TCB的第一个成员,即当前堆栈栈顶pxTopOfStack */
					"	ldmia r0!, {r4-r11, r14}		\n" /* Pop the registers that are not automatically saved on exception entry and the critical nesting count. */	/* 出栈,将寄存器r4~r11出栈 */
					"	msr psp, r0						\n" /* Restore the task stack pointer. */	/* 最新的栈顶指针赋给线程堆栈指针PSP */
					"	isb								\n"
					"	mov r0, #0 						\n"
					"	msr	basepri, r0					\n"
					"	bx r14							\n"
					"									\n"
					"	.align 4						\n"
					"pxCurrentTCBConst2: .word pxCurrentTCB				\n"
				);
}
/*-----------------------------------------------------------*/

static void prvPortStartFirstTask( void )
{
	__asm volatile(
					" ldr r0, =0xE000ED08 	\n" /* Use the NVIC offset register to locate the stack. */   /* Cortext-M3硬件中,0xE000ED08地址处为VTOR(向量表偏移量)寄存器,存储向量表起始地址*/
					" ldr r0, [r0] 			\n"	/* 取出向量表中的第一项,向量表第一项存储主堆栈指针MSP的初始值  这里执行完，r0里面是msp指针*/	
					" ldr r0, [r0] 			\n" /* 这里执行完，r0里面是主程序地址*/
					" msr msp, r0			\n" /* Set the msp back to the start of the stack. */	 /* 将堆栈地址存入主堆栈指针 msr是写指令*/			
					" cpsie i				\n" /* Globally enable interrupts. 使能全局中断*/			
					" cpsie f				\n" /*	失能全局中断	*/			
					" dsb					\n"
					" isb					\n"
					" svc 0					\n" /* System call to start first task. */	/* 调用SVC启动第一个任务 */
					" nop					\n"
				);
}
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
//真,任务调度函数
BaseType_t xPortStartScheduler( void )
{
	/* configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to 0.
	See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html */
	configASSERT( configMAX_SYSCALL_INTERRUPT_PRIORITY );

	/* This port can be used on all revisions of the Cortex-M7 core other than
	the r0p1 parts.  r0p1 parts should use the port from the
	/source/portable/GCC/ARM_CM7/r0p1 directory. */
	configASSERT( portCPUID != portCORTEX_M7_r0p1_ID );
	configASSERT( portCPUID != portCORTEX_M7_r0p0_ID );

	#if( configASSERT_DEFINED == 1 )
	{
		volatile uint32_t ulOriginalPriority;
		volatile uint8_t * const pucFirstUserPriorityRegister = ( volatile uint8_t * const ) ( portNVIC_IP_REGISTERS_OFFSET_16 + portFIRST_USER_INTERRUPT_NUMBER );
		volatile uint8_t ucMaxPriorityValue;

		/* Determine the maximum priority from which ISR safe FreeRTOS API
		functions can be called.  ISR safe functions are those that end in
		"FromISR".  FreeRTOS maintains separate thread and ISR API functions to
		ensure interrupt entry is as fast and simple as possible.

		Save the interrupt priority value that is about to be clobbered. */
		ulOriginalPriority = *pucFirstUserPriorityRegister;

		/* Determine the number of priority bits available.  First write to all
		possible bits. */
		*pucFirstUserPriorityRegister = portMAX_8_BIT_VALUE;

		/* Read the value back to see how many bits stuck. */
		/* 确定有效的优先级位个数. 首先向所有位写1,然后再读出来,由于无效的优先级位读出为0,然后数一数有多少个1,就能知道有多少位优先级.*/
		ucMaxPriorityValue = *pucFirstUserPriorityRegister;

		/* Use the same mask on the maximum system call priority. */
		/*
			这里ucMaxPriorityValue = 11110000 （8bit）
			表示优先级的位，
			这里与一下就等于自己。
		*/
		ucMaxSysCallPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY & ucMaxPriorityValue;

		/* Calculate the maximum acceptable priority group value for the number
		of bits read back. */
		/* 计算最大优先级组值
			大概可以理解成有多少个位表示优先级？
			每一位就是一个组
			对比肯定从高到低。
		 */
		ulMaxPRIGROUPValue = portMAX_PRIGROUP_BITS;			//portTOP_BIT_OF_BYTE标记出最高位0x80
		while( ( ucMaxPriorityValue & portTOP_BIT_OF_BYTE ) == portTOP_BIT_OF_BYTE )
		{
			ulMaxPRIGROUPValue--;
			ucMaxPriorityValue <<= ( uint8_t ) 0x01;
			/*
			11100000 		6
			11000000		5
			10000000		4
			00000000		3
			*/
		}

		/* Shift the priority group value back to its position within the AIRCR
		register. */
		ulMaxPRIGROUPValue <<= portPRIGROUP_SHIFT;
		ulMaxPRIGROUPValue &= portPRIORITY_GROUP_MASK;
		// ulMaxPRIGROUPValue = 3<<8;

		/* Restore the clobbered interrupt priority register to its original
		value. */
		*pucFirstUserPriorityRegister = ulOriginalPriority;
	}
	#endif /* conifgASSERT_DEFINED */

	/* Make PendSV and SysTick the lowest priority interrupts. */
	//对应PendSV和Systick的中断优先级寄存器。
	portNVIC_SYSPRI2_REG |= portNVIC_PENDSV_PRI;
	portNVIC_SYSPRI2_REG |= portNVIC_SYSTICK_PRI;

	/* Start the timer that generates the tick ISR.  Interrupts are disabled
	here already. */
	//这里开启了systick，我设置系统节拍为1ms
	vPortSetupTimerInterrupt();

	/* Initialise the critical nesting count ready for the first task. */
	uxCriticalNesting = 0;

	/* Ensure the VFP is enabled - it should be anyway. */
	//确保FPU使能
	vPortEnableVFP();

	/* Lazy save always. */
	*( portFPCCR ) |= portASPEN_AND_LSPEN_BITS;

	/* Start the first task. */
	prvPortStartFirstTask();

	/* Should never get here as the tasks will now be executing!  Call the task
	exit error function to prevent compiler warnings about a static function
	not being called in the case that the application writer overrides this
	functionality by defining configTASK_RETURN_ADDRESS. */
	prvTaskExitError();

	/* Should not get here! */
	return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* Not implemented in ports where there is nothing to return to.
	Artificially force an assert. */
	configASSERT( uxCriticalNesting == 1000UL );
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
	portDISABLE_INTERRUPTS();
	uxCriticalNesting++;

	/* This is not the interrupt safe version of the enter critical function so
	assert() if it is being called from an interrupt context.  Only API
	functions that end in "FromISR" can be used in an interrupt.  Only assert if
	the critical nesting count is 1 to protect against recursive calls if the
	assert function also uses a critical section. */
	if( uxCriticalNesting == 1 )
	{
		configASSERT( ( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK ) == 0 );
	}
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
	configASSERT( uxCriticalNesting );
	uxCriticalNesting--;
	if( uxCriticalNesting == 0 )
	{
		portENABLE_INTERRUPTS();
	}
}
/*-----------------------------------------------------------*/
/*
任务调度核心代码

任务调度有两种发法进入：执行系统调用，比如普通任务可以使用taskYIELD()强制任务切换，中断服务程序中使用portYIELD_FROM_ISR()强制任务切换；
系统节拍时钟中断

流程：

1.中断产生，硬件将


*/

void xPortPendSVHandler( void )
{
	/* This is a naked function. */

	__asm volatile
	(
	"	mrs r0, psp							\n" /* PSP内容存入R0 */
	"	isb									\n" /* 指令同步隔离,清流水线 */
	"										\n"
	"	ldr	r3, pxCurrentTCBConst			\n" /* Get the location of the current TCB. */	/* 当前激活的任务TCB指针存入R2 */
	"	ldr	r2, [r3]						\n"
	"										\n"
	"	tst r14, #0x10						\n" /* Is the task using the FPU context?  If so, push high vfp registers. */	/* 看下当前任务是不是在用FPU */
	"	it eq								\n"
	"	vstmdbeq r0!, {s16-s31}				\n"
	"										\n"
	"	stmdb r0!, {r4-r11, r14}			\n" /* Save the core registers. */	 /* 保存剩余的寄存器,异常处理程序执行前,硬件自动将xPSR、PC、LR、R12、R0-R3入栈 */
	"										\n"
	"	str r0, [r2]						\n" /* Save the new top of stack into the first member of the TCB. */	/* 将新的栈顶保存到任务TCB的第一个成员中 */
	"										\n"
	"	stmdb sp!, {r3}						\n"/* 将R3压栈 R3保存的当前激活的任务TCB指针(pxCurrentTCB)地址,函数调用后会用到,因此也要入栈保护*/
	"	mov r0, %0 							\n"//进入临界区，这里用的是gcc版本汇编写法，configMAX_SYSCALL_INTERRUPT_PRIORITY，注意 %0
	"	msr basepri, r0						\n"	//进入临界区
	"	dsb									\n"	
	"	isb									\n"
	"	bl vTaskSwitchContext				\n"/* 调用函数,寻找新的任务运行,通过使变量pxCurrentTCB指向新的任务来实现任务切换 */
	"	mov r0, #0							\n"
	"	msr basepri, r0						\n"	
	"	ldmia sp!, {r3}						\n"/* 恢复R3*/
	"										\n"
	"	ldr r1, [r3]						\n" /* The first item in pxCurrentTCB is the task top of stack. *//* 当前激活的任务TCB第一项保存了任务堆栈的栈顶,现在栈顶值存入R0*/
	"	ldr r0, [r1]						\n"
	"										\n"
	"	ldmia r0!, {r4-r11, r14}			\n" /* Pop the core registers. */	/* 出栈*/
	"										\n"
	"	tst r14, #0x10						\n" /* Is the task using the FPU context?  If so, pop the high vfp registers too. */
	"	it eq								\n"
	"	vldmiaeq r0!, {s16-s31}				\n"
	"										\n"
	"	msr psp, r0							\n"
	"	isb									\n"
	"										\n"
	#ifdef WORKAROUND_PMU_CM001 /* XMC4000 specific errata workaround. */
		#if WORKAROUND_PMU_CM001 == 1
	"			push { r14 }				\n"
	"			pop { pc }					\n"
		#endif
	#endif
	"										\n"
	"	bx r14								\n"
	"										\n"
	"	.align 4							\n"
	"pxCurrentTCBConst: .word pxCurrentTCB	\n"
	::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY)
	);
}
/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
	/* The SysTick runs at the lowest interrupt priority, so when this interrupt
	executes all interrupts must be unmasked.  There is therefore no need to
	save and then restore the interrupt mask value as its value is already
	known. */
	portDISABLE_INTERRUPTS();
	{
		/* Increment the RTOS tick. */
		if( xTaskIncrementTick() != pdFALSE )
		{
			/* A context switch is required.  Context switching is performed in
			the PendSV interrupt.  Pend the PendSV interrupt. */
			portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
		}
	}
	portENABLE_INTERRUPTS();
}
/*-----------------------------------------------------------*/

#if configUSE_TICKLESS_IDLE == 1

	__attribute__((weak)) void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
	{
	uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements, ulSysTickCTRL;
	TickType_t xModifiableIdleTime;

		/* Make sure the SysTick reload value does not overflow the counter. */
		if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
		{
			xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
		}

		/* Stop the SysTick momentarily.  The time the SysTick is stopped for
		is accounted for as best it can be, but using the tickless mode will
		inevitably result in some tiny drift of the time maintained by the
		kernel with respect to calendar time. */
		portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;

		/* Calculate the reload value required to wait xExpectedIdleTime
		tick periods.  -1 is used because this code will execute part way
		through one of the tick periods. */
		ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG + ( ulTimerCountsForOneTick * ( xExpectedIdleTime - 1UL ) );
		if( ulReloadValue > ulStoppedTimerCompensation )
		{
			ulReloadValue -= ulStoppedTimerCompensation;
		}

		/* Enter a critical section but don't use the taskENTER_CRITICAL()
		method as that will mask interrupts that should exit sleep mode. */
		__asm volatile( "cpsid i" );
		__asm volatile( "dsb" );
		__asm volatile( "isb" );

		/* If a context switch is pending or a task is waiting for the scheduler
		to be unsuspended then abandon the low power entry. */
		if( eTaskConfirmSleepModeStatus() == eAbortSleep )
		{
			/* Restart from whatever is left in the count register to complete
			this tick period. */
			portNVIC_SYSTICK_LOAD_REG = portNVIC_SYSTICK_CURRENT_VALUE_REG;

			/* Restart SysTick. */
			portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

			/* Reset the reload register to the value required for normal tick
			periods. */
			portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

			/* Re-enable interrupts - see comments above the cpsid instruction()
			above. */
			__asm volatile( "cpsie i" );
		}
		else
		{
			/* Set the new reload value. */
			portNVIC_SYSTICK_LOAD_REG = ulReloadValue;

			/* Clear the SysTick count flag and set the count value back to
			zero. */
			portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

			/* Restart SysTick. */
			portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

			/* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
			set its parameter to 0 to indicate that its implementation contains
			its own wait for interrupt or wait for event instruction, and so wfi
			should not be executed again.  However, the original expected idle
			time variable must remain unmodified, so a copy is taken. */
			xModifiableIdleTime = xExpectedIdleTime;
			configPRE_SLEEP_PROCESSING( &xModifiableIdleTime );
			if( xModifiableIdleTime > 0 )
			{
				__asm volatile( "dsb" );
				__asm volatile( "wfi" );
				__asm volatile( "isb" );
			}
			configPOST_SLEEP_PROCESSING( &xExpectedIdleTime );

			/* Stop SysTick.  Again, the time the SysTick is stopped for is
			accounted for as best it can be, but using the tickless mode will
			inevitably result in some tiny drift of the time maintained by the
			kernel with respect to calendar time. */
			ulSysTickCTRL = portNVIC_SYSTICK_CTRL_REG;
			portNVIC_SYSTICK_CTRL_REG = ( ulSysTickCTRL & ~portNVIC_SYSTICK_ENABLE_BIT );

			/* Re-enable interrupts - see comments above the cpsid instruction()
			above. */
			__asm volatile( "cpsie i" );

			if( ( ulSysTickCTRL & portNVIC_SYSTICK_COUNT_FLAG_BIT ) != 0 )
			{
				uint32_t ulCalculatedLoadValue;

				/* The tick interrupt has already executed, and the SysTick
				count reloaded with ulReloadValue.  Reset the
				portNVIC_SYSTICK_LOAD_REG with whatever remains of this tick
				period. */
				ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL ) - ( ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG );

				/* Don't allow a tiny value, or values that have somehow
				underflowed because the post sleep hook did something
				that took too long. */
				if( ( ulCalculatedLoadValue < ulStoppedTimerCompensation ) || ( ulCalculatedLoadValue > ulTimerCountsForOneTick ) )
				{
					ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL );
				}

				portNVIC_SYSTICK_LOAD_REG = ulCalculatedLoadValue;

				/* The tick interrupt handler will already have pended the tick
				processing in the kernel.  As the pending tick will be
				processed as soon as this function exits, the tick value
				maintained by the tick is stepped forward by one less than the
				time spent waiting. */
				ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
			}
			else
			{
				/* Something other than the tick interrupt ended the sleep.
				Work out how long the sleep lasted rounded to complete tick
				periods (not the ulReload value which accounted for part
				ticks). */
				ulCompletedSysTickDecrements = ( xExpectedIdleTime * ulTimerCountsForOneTick ) - portNVIC_SYSTICK_CURRENT_VALUE_REG;

				/* How many complete tick periods passed while the processor
				was waiting? */
				ulCompleteTickPeriods = ulCompletedSysTickDecrements / ulTimerCountsForOneTick;

				/* The reload value is set to whatever fraction of a single tick
				period remains. */
				portNVIC_SYSTICK_LOAD_REG = ( ( ulCompleteTickPeriods + 1UL ) * ulTimerCountsForOneTick ) - ulCompletedSysTickDecrements;
			}

			/* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
			again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
			value.  The critical section is used to ensure the tick interrupt
			can only execute once in the case that the reload register is near
			zero. */
			portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
			portENTER_CRITICAL();
			{
				portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
				vTaskStepTick( ulCompleteTickPeriods );
				portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;
			}
			portEXIT_CRITICAL();
		}
	}

#endif /* #if configUSE_TICKLESS_IDLE */
/*-----------------------------------------------------------*/

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
__attribute__(( weak )) void vPortSetupTimerInterrupt( void )
{
	/* Calculate the constants required to configure the tick interrupt. */
	#if configUSE_TICKLESS_IDLE == 1
	{
		ulTimerCountsForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
		xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
		ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
	}
	#endif /* configUSE_TICKLESS_IDLE */

	/* Configure SysTick to interrupt at the requested rate. */
	portNVIC_SYSTICK_LOAD_REG = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
	portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT );
}
/*-----------------------------------------------------------*/

/* This is a naked function. */
static void vPortEnableVFP( void )
{
	__asm volatile
	(
		"	ldr.w r0, =0xE000ED88		\n" /* The FPU enable bits are in the CPACR. */
		"	ldr r1, [r0]				\n"
		"								\n"
		"	orr r1, r1, #( 0xf << 20 )	\n" /* Enable CP10 and CP11 coprocessors, then save back. */
		"	str r1, [r0]				\n"
		"	bx r14						"
	);
}
/*-----------------------------------------------------------*/

#if( configASSERT_DEFINED == 1 )

	void vPortValidateInterruptPriority( void )
	{
	uint32_t ulCurrentInterrupt;
	uint8_t ucCurrentPriority;

		/* Obtain the number of the currently executing interrupt. */
		__asm volatile( "mrs %0, ipsr" : "=r"( ulCurrentInterrupt ) );

		/* Is the interrupt number a user defined interrupt? */
		if( ulCurrentInterrupt >= portFIRST_USER_INTERRUPT_NUMBER )
		{
			/* Look up the interrupt's priority. */
			ucCurrentPriority = pcInterruptPriorityRegisters[ ulCurrentInterrupt ];

			/* The following assertion will fail if a service routine (ISR) for
			an interrupt that has been assigned a priority above
			configMAX_SYSCALL_INTERRUPT_PRIORITY calls an ISR safe FreeRTOS API
			function.  ISR safe FreeRTOS API functions must *only* be called
			from interrupts that have been assigned a priority at or below
			configMAX_SYSCALL_INTERRUPT_PRIORITY.

			Numerically low interrupt priority numbers represent logically high
			interrupt priorities, therefore the priority of the interrupt must
			be set to a value equal to or numerically *higher* than
			configMAX_SYSCALL_INTERRUPT_PRIORITY.

			Interrupts that	use the FreeRTOS API must not be left at their
			default priority of	zero as that is the highest possible priority,
			which is guaranteed to be above configMAX_SYSCALL_INTERRUPT_PRIORITY,
			and	therefore also guaranteed to be invalid.

			FreeRTOS maintains separate thread and ISR API functions to ensure
			interrupt entry is as fast and simple as possible.

			The following links provide detailed information:
			http://www.freertos.org/RTOS-Cortex-M3-M4.html
			http://www.freertos.org/FAQHelp.html */
			configASSERT( ucCurrentPriority >= ucMaxSysCallPriority );
		}

		/* Priority grouping:  The interrupt controller (NVIC) allows the bits
		that define each interrupt's priority to be split between bits that
		define the interrupt's pre-emption priority bits and bits that define
		the interrupt's sub-priority.  For simplicity all bits must be defined
		to be pre-emption priority bits.  The following assertion will fail if
		this is not the case (if some bits represent a sub-priority).

		If the application only uses CMSIS libraries for interrupt
		configuration then the correct setting can be achieved on all Cortex-M
		devices by calling NVIC_SetPriorityGrouping( 0 ); before starting the
		scheduler.  Note however that some vendor specific peripheral libraries
		assume a non-zero priority group setting, in which cases using a value
		of zero will result in unpredicable behaviour. */
		configASSERT( ( portAIRCR_REG & portPRIORITY_GROUP_MASK ) <= ulMaxPRIGROUPValue );
	}

#endif /* configASSERT_DEFINED */


