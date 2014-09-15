/*
 * File       : ZDevice.h
 *
 * Description: Device driver framework header file.
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


#ifndef  _ZDEVICE_H_
  #define _ZDEVICE_H_

#include "ZTypes.h"									//

#define	 DDFERR_INVALID_ARGUMENTS	 -100
#define	 DDFERR_DCB_UNAVAILABLE		 -101
#define	 DDFERR_INVALID_INITIALIZER	 -102
#define	 DDFERR_DEVICE_CLOSED		 -103

#define MAX_DEV_NAME_LENGTH			 11

struct devCap
{
	UINT8 devType;
	VOID * devHdl;
};


//driver framework data types ;
typedef  INT8 				RZK_DEV_NAME_t ;
typedef  INT8 				RZK_DEV_MODE_t ;
typedef  INT8				RZK_DEV_BUFFER_t ;
typedef  signed long int	RZK_DEV_BYTES_t ;
typedef  INT8				RZK_DEV_STATUS_t ;
typedef  signed long int	DDF_STATUS_t;

// device states
#define RZKDEV_BLOCK			( (UINT8)0x01 )
#define RZKDEV_OPENED			( (UINT8)0x02 )
#define RZKDEV_READ_BUSY		( (UINT8)0x04 )
#define RZKDEV_WRITE_BUSY		( (UINT8)0x08 )

// device mode
#define RZKDEV_RX_INTERRUPT		( (UINT8)0x40 )
#define RZKDEV_RX_POLL			( (UINT8)0x10 )
#define RZKDEV_TX_INTERRUPT		( (UINT8)0x80 )
#define RZKDEV_TX_POLL			( (UINT8)0x20 )

// these macros need to be used to pass as operation type
#define RZKDEV_SET_IOCTL		( (UINT8)0x01 )
#define RZKDEV_GET_IOCTL		( (UINT8)0x00 )

extern struct RZK_DEVICE_CB_t *pdev;
// driver handler prototype that are installed by the driver
typedef DDF_STATUS_t ( * FNPTR_RZKDEV_INIT )(struct RZK_DEVICE_CB_t *pdev);
typedef DDF_STATUS_t ( * FNPTR_RZKDEV_STOP )(struct RZK_DEVICE_CB_t *pdev);
typedef DDF_STATUS_t ( * FNPTR_RZKDEV_OPEN )( struct RZK_DEVICE_CB_t *pdev,RZK_DEV_NAME_t* devName ,RZK_DEV_MODE_t *devMode);
typedef DDF_STATUS_t ( * FNPTR_RZKDEV_CLOSE )(struct RZK_DEVICE_CB_t *pdev);
typedef DDF_STATUS_t ( * FNPTR_RZKDEV_READ )(struct RZK_DEVICE_CB_t *pdev,RZK_DEV_BUFFER_t *buf,RZK_DEV_BYTES_t nBytes);
typedef DDF_STATUS_t ( * FNPTR_RZKDEV_WRITE )(struct RZK_DEVICE_CB_t *pdev,RZK_DEV_BUFFER_t *buf,RZK_DEV_BYTES_t nBytes);
typedef DDF_STATUS_t ( *FNPTR_RZKDEV_SEEK)( struct RZK_DEVICE_CB_t *pdev, RZK_DEV_BYTES_t nBytes);
typedef DDF_STATUS_t ( * FNPTR_RZKDEV_GETC )(struct RZK_DEVICE_CB_t *pdev);
typedef DDF_STATUS_t ( * FNPTR_RZKDEV_PUTC )(struct RZK_DEVICE_CB_t *pdev,RZK_DEV_BUFFER_t buf);
typedef DDF_STATUS_t ( * FNPTR_RZKDEV_IOCTL )(struct RZK_DEVICE_CB_t *pdev,RZK_DEV_BYTES_t opType,INT8 *addr1,INT8 *addr2);

// The RZK Device control block structure
// This structure contains pointers to the interfacing function to the device.

typedef struct RZK_DEVICE_CB_t
{
	UINT8				InUse;
	RZK_DEV_NAME_t		devName[ MAX_DEV_NAME_LENGTH + 1 ];	// device name
	// function handlers installed by the driver
	FNPTR_RZKDEV_INIT   fnInit  ;
	FNPTR_RZKDEV_STOP   fnStop  ;
	FNPTR_RZKDEV_OPEN   fnOpen  ;
	FNPTR_RZKDEV_CLOSE  fnClose ;
 	FNPTR_RZKDEV_READ   fnRead  ;
 	FNPTR_RZKDEV_WRITE  fnWrite ;
 	FNPTR_RZKDEV_SEEK   fnSeek  ;	
 	FNPTR_RZKDEV_GETC   fnGetc  ;
 	FNPTR_RZKDEV_PUTC   fnPutc  ;
 	FNPTR_RZKDEV_IOCTL  fnIoctl ;
	RZK_PTR_t			dvintvector;
	UINT16				dvivec;
	UINT8				*dvinputoutput;
	RZK_DEV_MODE_t		devMode;	
	UINT16 				dvminor;
} RZK_DEVICE_CB_t;


DDF_STATUS_t DevInit(RZK_DEVICE_CB_t *pdev);			// device id 
DDF_STATUS_t RZKDevAttach(RZK_DEVICE_CB_t *pdev);		// device id 
DDF_STATUS_t RZKDevDetach(RZK_DEVICE_CB_t *pdev) ; 		// 
DDF_STATUS_t RZKDevStop(RZK_DEVICE_CB_t *pdev);			// device id 
RZK_DEVICE_CB_t *RZKDevOpen(RZK_DEV_NAME_t *devName, RZK_DEV_MODE_t *devMode);

DDF_STATUS_t RZKDevRead( 
					 RZK_DEVICE_CB_t *pdev,				// device id
					 RZK_DEV_BUFFER_t *buf,				// buffer pointer where the data need to be stored
					 RZK_DEV_BYTES_t nBytes				// input-bytes to read, output-bytes read
					);

DDF_STATUS_t RZKDevWrite( 
					 RZK_DEVICE_CB_t *pdev,				// device id
					 RZK_DEV_BUFFER_t *buf,				// buffer pointer where the data need to be stored
					 RZK_DEV_BYTES_t nBytes				// input-bytes to read, output-bytes read
					);

DDF_STATUS_t RZKDevClose(RZK_DEVICE_CB_t *pdev);		// device id
					

DDF_STATUS_t RZKDevIOCTL( 
					 RZK_DEVICE_CB_t *pdev,				// device id
					 RZK_DEV_BYTES_t uOperation,
					 void *addr1,
					 void *addr2
						 );

DDF_STATUS_t RZKDevGetc(RZK_DEVICE_CB_t *pdev);			// device id
						

DDF_STATUS_t RZKDevPutc( 
					 RZK_DEVICE_CB_t *pdev,				// device id
					 RZK_DEV_BUFFER_t buf				// buffer pointer where the data need to be stored
					);

#endif /* _ZDEVICE_H_ */
