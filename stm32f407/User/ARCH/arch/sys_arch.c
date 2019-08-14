/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
/*  Porting by Michael Vysotsky <michaelvy@hotmail.com> August 2011   */

/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
/*  Porting by Michael Vysotsky <michaelvy@hotmail.com> August 2011   */

#define SYS_ARCH_GLOBALS

/* lwIP includes. */
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/sys.h"
#include "lwip/mem.h"

#include "arch/sys_arch.h"
#include "os.h"
#include "os_cfg_app.h"
#include "string.h"

/*----------------------------------------------------------------------------*/
/*                      DEFINITIONS                                           */
/*----------------------------------------------------------------------------*/
#define LWIP_ARCH_TICK_PER_MS       (1000/OS_CFG_TICK_RATE_HZ)
/*
   ----------------------------------------
   ---------- Lwip RTOS options -----------
   ----------------------------------------
*/
#define LWIP_TASK_MAX             4
#define LWIP_STK_SIZE             1024

/*----------------------------------------------------------------------------*/
/*                      VARIABLES                                             */
/*----------------------------------------------------------------------------*/

int errno = 0;

const void * const pvNullPointer = (mem_ptr_t*)0xffffffff;

CPU_STK       LwIP_Task_Stk[LWIP_TASK_MAX][LWIP_STK_SIZE];
OS_TCB        LwIP_Task_TCB[LWIP_TASK_MAX];
CPU_INT08U    LwIP_Task_Count = 0;

u32_t lwip_sys_now;


/*----------------------------------------------------------------------------*/
/*                      PROTOTYPES                                            */
/*----------------------------------------------------------------------------*/
static CPU_SR_ALLOC();
sys_prot_t sys_arch_protect(void)
{
	CPU_CRITICAL_ENTER();
	return 0;
}

void sys_arch_unprotect(sys_prot_t pval)
{
	LWIP_UNUSED_ARG(pval);
	CPU_CRITICAL_EXIT();
}

//===========================================================================================
// Function : 创建一个邮箱,这里用UCOS3的消息队列来实现
// *mbox    : 消息邮箱
// size     : 邮箱大小
// Return   : 创建成功返回ERR_OK, 创建失败返回ERR_MEM
//===========================================================================================
err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
  OS_ERR ucErr;
      
  OSQCreate(mbox, "LWIP QUEUE", size, &ucErr); 
  if (ucErr == OS_ERR_NONE)
	{ 
    return ERR_OK; 
  }
	return ERR_MEM;
}

/*-----------------------------------------------------------------------------------*/
/*
  Deallocates a mailbox. If there are messages still present in the
  mailbox when the mailbox is deallocated, it is an indication of a
  programming error in lwIP and the developer should be notified.
*/
void sys_mbox_free(sys_mbox_t *mbox)
{
	OS_ERR ucErr;
	
	OSQFlush(mbox, &ucErr);
	OSQDel(mbox, OS_OPT_DEL_ALWAYS, &ucErr);
}

/*-----------------------------------------------------------------------------------*/
//   Posts the "msg" to the mailbox.
void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
  OS_ERR ucErr; 
	
  if (msg == NULL)
	{
		msg = (void *)&pvNullPointer;
	}

	while(1)
	{
		OSQPost(mbox, msg, 0, OS_OPT_POST_ALL, &ucErr);
		if (ucErr == OS_ERR_NONE)
		{
			break;
		}
		OSTimeDly(1, OS_OPT_TIME_DLY, &ucErr);
	}
}

/*-----------------------------------------------------------------------------------*/
//   Try to post the "msg" to the mailbox.
err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
	OS_ERR ucErr;

  if (msg == NULL)
	{
		msg = (void *)&pvNullPointer;
	}
	
	OSQPost(mbox, msg, 0, OS_OPT_POST_ALL, &ucErr);    
	if (ucErr != OS_ERR_NONE)
	{
		return ERR_MEM;
	}
	return ERR_OK;
}


err_t sys_mbox_trypost_fromisr(sys_mbox_t *q, void *msg)
{
  return sys_mbox_trypost(q, msg);
}

//===========================================================================================
// Function : 等待邮箱中的消息
// *mbox    : 消息邮箱
// *msg     : 消息
// timeout  : 超时时间(ms), 如果timeout为0的话就一直等待
// Return   : 不超时返回等待的时间(ms), 超时则返回SYS_ARCH_TIMEOUT
//===========================================================================================
u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
	void *temp;
  OS_ERR ucErr;
  OS_MSG_SIZE msg_size;
  CPU_TS TickBegin, TickEnd;  
  CPU_TS OSTimeout = timeout / LWIP_ARCH_TICK_PER_MS;
	
  if ((timeout != 0) && OSTimeout == 0)
	{
    OSTimeout = 1;
	}
	TickBegin = OSTimeGet(&ucErr);
  temp = OSQPend(mbox, OSTimeout, OS_OPT_PEND_BLOCKING, &msg_size, NULL, &ucErr);
	//因为lwip发送空消息的时候我们使用了pvNullPointer指针,所以判断pvNullPointer指向的值
	//就可知道请求到的消息是否有效
	if (msg != NULL)
	{
		if (temp == (void*)&pvNullPointer)
			*msg = NULL;
		else
			*msg = temp;
	}
  if (ucErr == OS_ERR_TIMEOUT)
	{
		return SYS_ARCH_TIMEOUT;
	}
	else
	{
		TickEnd = OSTimeGet(&ucErr);
		if (TickEnd > TickBegin)
			return ((TickEnd - TickBegin) * LWIP_ARCH_TICK_PER_MS + 1);
		else
			return ((0xffffffff - TickBegin + TickEnd) * LWIP_ARCH_TICK_PER_MS + 1);
	}
}

//===========================================================================================
// Function : 尝试等待邮箱消息
// *sem     : 等待的信号量
// Return   : 成功获取邮箱消息返回0, 获取失败不阻塞进程直接返回SYS_MBOX_EMPTY
//===========================================================================================
u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
	if (sys_arch_mbox_fetch(mbox, msg, 1) == SYS_ARCH_TIMEOUT)
	{
		return SYS_MBOX_EMPTY;
	}
	else
	{
		return 0;
	}
}

/** 
  * Check if an mbox is valid/allocated: 
  * @param sys_mbox_t *mbox pointer mail box
  * @return 1 for valid, 0 for invalid 
  */ 
int sys_mbox_valid(sys_mbox_t *mbox)
{
  if (mbox->NamePtr)  
    return (strcmp(mbox->NamePtr,"?Q")) ? 1 : 0;
  else
    return 0;
}

/** 
  * Set an mbox invalid so that sys_mbox_valid returns 0 
  */      
void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
  if (sys_mbox_valid(mbox))
		sys_mbox_free(mbox);
}


/*-----------------------------------------------------------------------------------*/
//  Creates a new semaphore. The "count" argument specifies
//  the initial state of the semaphore.
err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
  OS_ERR ucErr;
	
  OSSemCreate(sem, "LWIP Sem", count, &ucErr);
  if (ucErr != OS_ERR_NONE)
	{
    return -1;    
  }
  return 0;
}

//===========================================================================================
// Function : 等待信号量
// *sem     : 等待的信号量
// timeout  : 超时时间(ms), 如果timeout为0的话就一直等待
// Return   : 不超时返回等待的时间(ms), 超时则返回SYS_ARCH_TIMEOUT
//===========================================================================================
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
  OS_ERR ucErr;
  CPU_TS TickBegin, TickEnd; 
  CPU_TS OStimeout = timeout / LWIP_ARCH_TICK_PER_MS;
	
	if ((timeout != 0) && OStimeout == 0)
	{
		OStimeout = 1;
	}
	TickBegin = OSTimeGet(&ucErr);
	OSSemPend(sem, OStimeout, OS_OPT_PEND_BLOCKING, NULL, &ucErr);
	
  if (ucErr == OS_ERR_TIMEOUT)
	{
		return SYS_ARCH_TIMEOUT;
	}
	else
	{
		TickEnd = OSTimeGet(&ucErr);
		if (TickEnd > TickBegin)
			return ((TickEnd - TickBegin) * LWIP_ARCH_TICK_PER_MS + 1);
		else
			return ((0xffffffff - TickBegin + TickEnd) * LWIP_ARCH_TICK_PER_MS + 1);
	}
}

/*-----------------------------------------------------------------------------------*/
// Signals a semaphore
void sys_sem_signal(sys_sem_t *sem)
{
  OS_ERR ucErr;
  
  OSSemPost(sem, OS_OPT_POST_ALL, &ucErr);
}

/*-----------------------------------------------------------------------------------*/
// Deallocates a semaphore
void sys_sem_free(sys_sem_t *sem)
{
	OS_ERR ucErr;
	
	OSSemDel(sem, OS_OPT_DEL_ALWAYS, &ucErr);
}

/*-----------------------------------------------------------------------------------*/
int sys_sem_valid(sys_sem_t *sem)
{
	if (sem->NamePtr)
		return (strcmp(sem->NamePtr, "?SEM")) ? 1 : 0;
	else
		return 0;
}

/*-----------------------------------------------------------------------------------*/
void sys_sem_set_invalid(sys_sem_t *sem)
{
  if (sys_sem_valid(sem))
		sys_sem_free(sem);
}


/*-----------------------------------------------------------------------------------*/
// Initialize sys arch
void sys_init(void)
{
	LwIP_Task_Count = 0;
	memset(LwIP_Task_Stk, 0, sizeof(LwIP_Task_Stk));
	memset(LwIP_Task_TCB, 0, sizeof(LwIP_Task_TCB));
}

u32_t
sys_jiffies(void)
{
  //lwip_sys_now = xTaskGetTickCount();
  return lwip_sys_now;
}

u32_t
sys_now(void)
{
  //lwip_sys_now = xTaskGetTickCount();
	u32_t ucos_time, lwip_time;
	ucos_time=OSTimeGet(NULL);	//获取当前系统时间 得到的是UCOS的节拍数
	lwip_time=(ucos_time*1000/OS_CFG_TICK_RATE_HZ+1);//将节拍数转换为LWIP的时间MS
	return lwip_time; 		//返回lwip_time; 
  //return lwip_sys_now;
}

//互斥体部分不实现了, 用二值信号量取代


/*-----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------*/
// TBD 
/*-----------------------------------------------------------------------------------*/
/*
  Starts a new thread with priority "prio" that will begin its execution in the
  function "thread()". The "arg" argument will be passed as an argument to the
  thread() function. The id of the new thread is returned. Both the id and
  the priority are system dependent.
*/
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio)
{
	OS_ERR err;
	
	CPU_SR_ALLOC();
	
	if (LwIP_Task_Count < LWIP_TASK_MAX)
	{
		CPU_CRITICAL_ENTER();
		
		OSTaskCreate((OS_TCB     *)&LwIP_Task_TCB[LwIP_Task_Count],
                 (CPU_CHAR   *) name,
                 (OS_TASK_PTR ) thread,
                 (void       *) arg,
                 (OS_PRIO     ) prio,
                 (CPU_STK    *) LwIP_Task_Stk[LwIP_Task_Count],
                 (CPU_STK_SIZE) LWIP_STK_SIZE / 10,
                 (CPU_STK_SIZE) LWIP_STK_SIZE,
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
		
		CPU_CRITICAL_EXIT();
		
		if (err == OS_ERR_NONE)
		{
			LwIP_Task_Count ++;
		}
	}

	return err;
}
