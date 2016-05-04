/**************************************************************************************************
  Filename:       HVACTest.h
  Revised:        $Date: 2012-02-12 16:04:42 -0800 (Sun, 12 Feb 2012) $
  Revision:       $Revision: 29217 $

  Description:    This file contains the Generic Application definitions.


  Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#ifndef HVACQUEEN_H
#define HVACQUEEN_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define HVACQUEEN_ENDPOINT           10

#define HVACQUEEN_PROFID             0x0F04
#define HVACQUEEN_DEVICEID           0x0001
#define HVACQUEEN_DEVICE_VERSION     5
#define HVACQUEEN_FLAGS              0

#define HVACQUEEN_MAX_CLUSTERS       6
#define HVACQUEEN_TRS_SEND_CLUSTERID    1
#define HVACQUEEN_ALIVE_CLUSTERID       2
#define HVACQUEEN_FW_CLUSTERID          3
#define HVACQUEEN_CONTROL               4
#define HVACQUEEN_FIRMWAREACK           5  
#define HVACQUEEN_REMOTEACK             6

// Event handler timeout
#define HVACQUEEN_EVTHANDLE_TIMEOUT  PTL0_EVENT_TIMEINVER     // Every PTL0_EVENT_TIMEINVER ms  
#define HVAC_WDT_CLEAR_TIMEOUT       125        // every 125ms                
#define HVAC_PTL0_FAIL_TIMEOUT       1000       // 1s timeout
   
// Application Events (OSAL) - These are bit weighted definitions.
#define HVACPTL0_EVENT_TIMEOUT_EVT   0x0004   
#define HVAC_WDT_CLEAR_EVT           0x0008
#define HVAC_PTL0_GUT_EVT            0x0400
  
// Flash related
#define HVAC_VALIDFLASH_ITEM    0x501
#define HVAC_MACANUM_ITEM       0x502
#define HVAC_MACADDRESS_ITEM    0x503
#define HVAC_NETWORKADDR_ITEM   0x504

#define HVAC_MAC_FLASH_VALID     0x5A
#define HVAC_MAC_FLASH_INVALID   0x00       

#define HVAC_MAC_FLAHS_MAXLEN   240
#define HVAC_NET_FLAHS_MAXLEN   80

#define HVAC_MAX_DRONE_NUM      30  
  
#if defined( IAR_ARMCM3_LM )
#define HVACQUEEN_RTOS_MSG_EVT       0x0002
#endif  

/*********************************************************************
 * MACROS
 */

   
#ifdef HVAC_WHITELIST
extern uint8 hvac_whiteList[HVAC_MAX_DRONE_NUM][Z_EXTADDR_LEN];
#endif
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void HVACTest_Init( byte task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 HVACTest_ProcessEvent( byte task_id, UINT16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HVACQUEEN_H */
