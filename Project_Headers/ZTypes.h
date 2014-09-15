/*
 * File       : ZTypes.h
 *
 * Description: Defines the typedef's, macro's and enum's being used by RZK.
 *
 * Copyright 2004 ZiLOG Inc.  ALL RIGHTS RESERVED.
 *
 * This file contains unpublished confidential and proprietary information
 * of ZiLOG, Inc.
 * NO PART OF THIS WORK MAY BE DUPLICATED, STORED, PUBLISHED OR DISCLOSED 
 * IN ANY FORM WITHOUT THE PRIOR WRITTEN CONSENT OF ZiLOG, INC.
 * This is not a license and no use of any kind of this work is authorized
 * in the absence of a written license granted by ZiLOG, Inc. in ZiLOG's 
 * sole discretion 
 */

/*
 * Revision History
 * 27-OCT-2004, Mahadev KC
 * Changed for IAR port
 * TBD: COUNT_t, RZK_SIGNAL_t, RZK_MASK_t, RZK_EVENT_t type definitions
 *
 */

// IAR commented, not required...
// #include "ZMachine.h"
 
#ifndef _ZTYPES

#define _ZTYPES	/* TOKEN FOR CHECKING DECLARATION OF THIS FILE */

#include <ctype.h>

// typedefinitions for standard data types.
typedef unsigned char	UINT8 ;
typedef signed char		INT8 ;
typedef unsigned short	UINT16 ;
typedef signed short	INT16 ;
typedef unsigned int	UINT ;
typedef signed int		INT ;
typedef unsigned int	UINT24 ;
typedef signed int		INT24 ;
typedef unsigned long	UINT32 ;
typedef signed long		INT32 ;	
typedef void			VOID ;	
//typedef signed char     INT8; //CR #5833
//typedef unsigned char	UINT8;  //CR #5833

typedef unsigned char RZK_NAME_t;
typedef unsigned char RZK_THREAD_PRIORITY_t ;
typedef unsigned char RZK_OPERATIONMODE_t; 
typedef void * RZK_MESSAGE_PTR_t;
typedef unsigned char * CADDR_t;
typedef unsigned long TIME_t;
typedef unsigned char RZK_STATE_t;
typedef unsigned int UINTRMASK; // IAR, moved from ZMachine.h
typedef unsigned int RZK_STATUS_t;

#ifdef _IAR_CODE

typedef unsigned int COUNT_t;
typedef UINT32 TICK_t;
//typedef unsigned int  RZK_PREEMPTION_t; // IAR not required

typedef UINT32 RZK_EVENT_t;
typedef unsigned int RZK_SIGNAL_t;
typedef UINT32 RZK_MASK_t;
//typedef unsigned int RZK_KEY_t; // IAR not required
typedef UINT16 RZK_INTERRUPT_NUM_t;
//typedef unsigned long RZK_DEVDESCRIPTOR_t; // IAR not required
typedef unsigned long RZK_LENGTH_t ;	// IAR added

#else

typedef unsigned int COUNT_t;
typedef unsigned int TICK_t;
//typedef unsigned int  RZK_PREEMPTION_t; // IAR not required

typedef unsigned int RZK_EVENT_t;
typedef unsigned int RZK_SIGNAL_t;
typedef unsigned int RZK_MASK_t;
//typedef unsigned int RZK_KEY_t; // IAR not required
typedef unsigned int RZK_INTERRUPT_NUM_t;
//typedef unsigned long RZK_DEVDESCRIPTOR_t; // IAR not required
typedef unsigned int RZK_LENGTH_t ;	// IAR added

//#define RZK_TOOLSMITH

#endif


typedef void * RZK_HANDLE_t;
typedef void * RZK_THREADHANDLE_t;
typedef void * RZK_MAILBOXHANDLE_t; 
typedef void * RZK_MESSAGEQHANDLE_t;
typedef void * RZK_SEMAPHOREHANDLE_t;
typedef void * RZK_EVENTHANDLE_t;
typedef void * RZK_TIMERHANDLE_t;
typedef void * RZK_PARTITIONHANDLE_t;
typedef void * RZK_REGIONHANDLE_t;
typedef void * RZK_PTR_t;

// IAR port commented
//typedef void * RZK_CONDVARHANDLE_t; 

// IAR PORT
typedef void (*FNP_THREAD_ENTRY)( void ) ;
typedef void (*FNP_SIG_HANDLER)( void ) ;
typedef void (*FNP_TIMER_FUNCTION)( void ) ;
typedef void (*RZK_FNP_ISR)( void ) ;

/*static RZK_PTR_t NULL = 0;*/
#ifndef NULL
#define NULL ( (void *) 0) 
#endif

/* To check the given objects by using magic number */
#define MAGIC_NUM_THREAD		0x5A5A5A5A
#define MAGIC_NUM_MESSAGEQ		0xAAAAAAAA
#define MAGIC_NUM_MAILBOX		0x55555555
#define MAGIC_NUM_EVENT		    0xA5A5A5A5
#define MAGIC_NUM_MEMORY		0x3AA3A33A
#define MAGIC_NUM_TIMER			0xA5A55A5A
#define MAGIC_NUM_SEMAPHORE	    0xC5C55C5C
#define MAGIC_NUM_REGION		0xF7F7F7F7


//IAR change
//#define MAX_INFINITE_SUSPEND	((unsigned int)(-1))    /* Maximum time for the infinite suspend */
#define MAX_INFINITE_SUSPEND	((TICK_t)(-1))    /* Maximum time for the infinite suspend */

#define INFINITE_SUSPEND		MAX_INFINITE_SUSPEND

#define MAX_OBJECT_NAME_LEN   12/* Maximum length of an object's name */

/* STATE OF  THE  OBJECTS */
#define OBJECT_CREATED		        (UINT8)0x01    /* The entries in the CB is valid */
#define OBJECT_BUSY				    (UINT8)0x02    /* CB is in used state */
#define OBJECT_PRIORITY_INHERITANCE (UINT8)0x20
#define OBJECT_RECV_PRIORITY	    (UINT8)0x80

/*  STATE OF OBJECTS AND THREAD  */
#define THREAD_CREATED			(UINT8)0x01    /* The entries in the TCB is valid */
#define THREAD_BUSY				(UINT8)0x02    /* TCB is in used state */
#define THREAD_TERMINATED		(UINT8)0x04    /* Thread have been either terminated or finished */
#define THREAD_RUNNING			(UINT8)0x08    /* Thread is ready to run */
#define THREAD_BLOCKEDSUSPEND	(UINT8)0x10   /* Thread has been blocked for want of resource */
#define THREAD_SIGNAL			(UINT8)0x20   /*  The signal handle is running */
#define THREAD_TIMEDBLOCK 		(UINT8)0x40   /* Thread is blocked on a resource for specified time */
#define THREAD_INFINITESUSPEND	(UINT8)0x80  /* Thread is suspended for infinite time */

// 18-JUN-2003
// device driver framework
#define THREAD_PEND_ON_IO		((UINT8)0x01)
#define THREAD_IO_BUSY			((UINT8)0x02)

/* STATE FOR TIMER OBJECT */
#define TIMER_DISABLE				(UINT8)0x04
#define TIMER_RUNNING				(UINT8)0x08		
#define TIMER_ACKNOWLEDGE_DISABLE	(UINT8)0x10

/* REASONS FOR BLOCKING  */
#define BLOCK_TIMEDSUSPEND		(UINT16)0x01   /* Thread is suspended for specific time */
#define BLOCK_ACQUIRESEMAPHORE	(UINT16)0x02  /* Thread is blocked while acquiring a semaphore */
#define BLOCK_POSTMESSAGEQUEUE  (UINT16)0x04  /* Thead is blocked while posting a message  on message Queue*/
#define BLOCK_PENDMESSAGEQUEUE  (UINT16)0x08  /* Thread is blocked while receiving a message  on message Queue */
#define BLOCK_POSTMAILBOX		(UINT16)0x10  /* Thread is blocked while posting a message  on mailbox */
#define BLOCK_PENDMAILBOX		(UINT16)0x20  /* Thread is blocked while receiving  a message  on mailbox */
#define BLOCK_PENDEVENTGROUP	(UINT16)0x40  /* Thread is blocked while waiting on event group */
#define BLOCK_REGIONWAIT		(UINT16)0x400 /* Thread is blocked while waiting on a region*/

/* DEFINES THE MAXIMUM AND MINIMUM VALUES FOR THE PRIORITY */
#define HIGHEST_PRIORITY 0
#define DMAX_PRIORITY	31
#define DMIN_PRIORITY	 1
#define GMAX_PRIORITY	127
#define GMIN_PRIORITY	 4
#define DMAX_PRIORITY_P1	32 /* Must be one greater than MAX Dispatch priority */
#define TMAX_QUEUE		2   /* Specifies the maximum time Queue */
 
/* DEFINES THE STATE OF OBJECTS-MAILBOX */
#define OBJECT_FULL	       (UINT8)0x04

/* DEFINES BLOCKED QUEUE TYPES */
#define TIME_Q1		0 
#define TIME_Q2		1

/* DEFINES THE STATE OF THE FLAGS FOR EVENTS */
#define RZK_TRUE	1
#define RZK_FALSE	0

/* Mailbox Size */
#define MAILBOX_SIZE	4

typedef enum 
{
   	EVENT_AND,			/* Specifies that the event operation is ANDed with the existing value*/
	EVENT_OR			 /* Specifies that the event operation is ORed with the existing value*/
} RZK_EVENT_OPERATION_et;

typedef enum
{
	MESSAGE_NONE=0,
	MESSAGE_SEND,
	MESSAGE_RECEIVE
} RZK_WAITING_THREADS_et ;

typedef enum  
{
	  ROUNDROBIN_MODE,			/* Specifies Round Robin scheduling attributes for thread*/
	  PRIORITY_MODE				/* Specifies that the thread is running in the priority mode*/
} RZK_THREAD_SCHATTRIB_et; 

typedef enum 
{ 
	RECV_ORDER_FIFO,        /* Specifies that the receiving attribute is FIFO */
	RECV_ORDER_PRIORITY,		/* Specifies  that the receiving attribute is PRIORITY */
	RECV_ORDER_FIFO_PI_OVERRIDDEN,  /* Specifies that the receiving attribute is FIFO but priority inheritance is overridden*/
	RECV_ORDER_PI_OVERRIDDEN  /* Specifies that the receiving attirbute is PRIORITY but priority inheritance is overridden*/
} RZK_RECV_ATTRIB_et;

/* VALUES FOR THE ERROR CONDITIONS */
typedef enum  
{
	RZKERR_SUCCESS=0,          /* Specifies that successful */
	RZKERR_INVALID_HANDLE,     /* Specifies invalid handle */
	RZKERR_INVALID_ARGUMENTS,  /* Specifies invalid arguments */
	RZKERR_INVALID_OPERATION,  /* Specifies invalid operation */ 
	RZKERR_CB_UNAVAILABLE,     /* Allocation of control block not possible */
	RZKERR_QUEUE_EMPTY,        /* specifies an empty queue*/
	RZKERR_OBJECT_DELETED,     /* specifies the object was deleted*/
	RZKERR_TIMEOUT,            /* specifies that the time had expired*/
	RZKERR_INVALID_SIZE,       /* specifies that the size is invalid*/
	RZKERR_OBJECT_RESET,       /* specifies the object was reset*/ 
	RZKERR_OBJECT_IN_USE,      /* specifies the object was in use*/
	RZKERR_INVALID_STACK,      /* specifies that the stack is invalid*/
	RZKERR_INVALID_PRIORITY,   /* specifies that the priority is invalid*/
	RZKERR_QUEUE_FULL,         /* specifies a full queue*/
	RZKERR_SCB_UNAVAILABLE,    /* specifies that segment control block is unavailable*/
	RZKERR_OUT_OF_MEMORY,      /* specifies that the user tried to allocate a memory not accessible */
	RZKERR_CB_BUSY,            /* specifies that the given object is busy */
	RZKERR_FATAL,                /* specifies that a Fatal error has occured*/
	RZKERR_PREEMPTION_DISABLED,	/* specifies that the preemption option is disabled
								for the thread */
	RZKERR_INVALID_ERROR_NUMBER,
	RZKERR_DEV_NOT_FOUND,      /* Specifies that the device is not found */
	RZKERR_DEV_IOERR,			
	RZKERR_DEV_BUSY,			/* Specifies that the device is busy */
	RZKERR_DEV_INVALID_NAME,	/* Specifies that the device name is invalid */
	RZKERR_DEV_TIMEOUT,			/* Specifies that the time has expired */
	RZKERR_OPERATION_NOT_SUPPORTED, /* Specifies that the given operation is not supported*/
	RZKERR_MEDIA,
	RZKERR_EOF,
	RZKERR_UNEXPECTED,
	RZKERR_DEV_ALREADY_OPENED,
	RZKERR_MAILBOX_FULL,         /* specifies a full mailbox*/
	RZKERR_SEM_NOTOWNED,
	// device driver framework error values
	// 18-JUN-2003
	RZKERR_DEV_NOT_INITIALIZED,
	RZKERR_DEV_READ_BUSY,
	RZKERR_DEV_WRITE_BUSY,
	RZKERR_DEV_CLOSED,
	RZKERR_DEV_INVALID_MODE,

	RZKERR_MSG_PRESENT,

	RZKERR_USER_ERROR = 251

} RZK_ERROR_et;

#ifdef	_IAR_CODE
#ifndef	TRUE
#define	TRUE	1
#endif
#ifndef	FALSE
#define	FALSE	0
#endif
#endif

#ifndef SYSERR
#define SYSERR (-1)
#endif 
#ifndef OK
#define OK (1)
#endif
#ifndef BOOL
#define BOOL UINT8
#endif

#endif  /* _INC_ZTYPE  */
