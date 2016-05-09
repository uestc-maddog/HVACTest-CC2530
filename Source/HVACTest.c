/******************************************************************************
  Filename:       HVACTest.c
  Revised:        $Date: 2014-09-07 13:36:30 -0700 (Sun, 07 Sep 2014) $
  Revision:       $Revision: 40046 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2014 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
******************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful - it is
  intended to be a simple example of an application's structure.

  This application periodically sends a "Hello World" message to
  another "Generic" application (see 'txMsgDelay'). The application
  will also receive "Hello World" packets.

  This application doesn't have a profile, so it handles everything
  directly - by itself.

  Key control:
    SW1:  changes the delay between TX packets
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request

LED:    LED3	
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "HVACTest.h"
#include "DebugTrace.h"

#if !defined( WIN32 ) || defined( ZBIT )
  #include "OnBoard.h"
#endif

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif

/* MT */
#include "MT_UART.h"
#include "MT.h"

/* HAL */
#include "hal_uart.h"
#include "hal_led.h"

/* ZDO */
#include "ZDObject.h"

/* NWK */
#include "NLMEDE.h"
#include "AddrMgr.h"

/* NV */
#include "OSAL_Nv.h"

/* ZDO */
#include "ZDApp.h"
#include "ZDSecMgr.h"

/* Application */
#include "hvac_protocol0.h"

#include "string.h"
/*********************************************************************
 * MACROS
 */
#define HVAC_STM32_RESET_DELAY  2       // reset trigger 2s
#define HVAC_STM32_RESET        false   
#define HVAC_STM32_RECOVER      true

#define HVACTEST_SELFCHECK      5      // 10s to setup network and allow 32 to perform a self-check
#define HVACTEST_INFODISPLAY    10     // display information in 5s, then system reset
   
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t HVACTest_ClusterList[HVACQUEEN_MAX_CLUSTERS] =
{
  HVACQUEEN_TRS_SEND_CLUSTERID,
  HVACQUEEN_ALIVE_CLUSTERID,
  HVACQUEEN_FW_CLUSTERID,
  HVACQUEEN_CONTROL,
  HVACQUEEN_FIRMWAREACK,
  HVACQUEEN_REMOTEACK
};

const SimpleDescriptionFormat_t HVACTest_SimpleDesc =
{
  HVACQUEEN_ENDPOINT,              //  int Endpoint;
  HVACQUEEN_PROFID,                //  uint16 AppProfId[2];
  HVACQUEEN_DEVICEID,              //  uint16 AppDeviceId[2];
  HVACQUEEN_DEVICE_VERSION,        //  int   AppDevVer:4;
  HVACQUEEN_FLAGS,                 //  int   AppFlags:4;
  HVACQUEEN_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)HVACTest_ClusterList,  //  byte *pAppInClusterList;
  HVACQUEEN_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)HVACTest_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in HVACTest_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t HVACTest_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
// This white list is also used as mac - network address mapping.
// So this array should not claim only with HVAC_WHITELIST
uint8 hvac_whiteList[HVAC_MAX_DRONE_NUM][Z_EXTADDR_LEN] = {0};

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte HVACTest_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // HVACTest_Init() is called.

devStates_t HVACTest_NwkState;

byte HVACTest_TransID;  // This is the unique message ID (counter)

afAddrType_t HVACTest_DstAddr;

typedef enum
{
  CC2530_INIT,
  STM32_NORMAL,
  HAS_ZIGBEE_NETWORK,
  NO_ZIGBEE_NETWORK,
  NO_FLASH,
  NO_ADC,
  NO_HUAWEI,
  NO_STM32
}hvac_test_error_t;

// special for test code
hvac_test_error_t HVACTest_errorcode = CC2530_INIT;
uint8 test_counter = HVACTEST_SELFCHECK;
uint8 test_infodisplay = false;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
#if defined( IAR_ARMCM3_LM )
static void HVACTest_ProcessRtosMessage( void );
#endif

// UART handler functions
static void hvacUART_PTL0_PING( hvac_test_error_t );

// Reset Control
static void hvac_STM32ResetInit( void );

// SBL 
#ifdef HVAC_SBL
static void appForceBoot(void);
#endif

#ifdef HAL_UART
static void HVACTest_HandleUart (mtOSALSerialData_t *pMsg);
#endif


/*********************************************************************
 * TEST Related
 */
#pragma location=0x4000
const __code uint8 firm_version = HVACQUEEN_DEVICE_VERSION;

#pragma location=0x4001
const __code uint8 hardware_version = 1;

#pragma location=0x4002
const __code uint8 chip_version = 1;

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HVACTest_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void HVACTest_Init( uint8 task_id )
{
  HVACTest_TaskID = task_id;
  HVACTest_NwkState = DEV_INIT;
  HVACTest_TransID = 0;
  uint8 initVersion = 0;
  uint8 startupMsg[4] = {0xFF,0xFF,0,0};

  // use this code to trick compiler
  initVersion = firm_version + hardware_version + chip_version;
  (void)initVersion;  
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().
  hvac_STM32ResetInit();
  // Register UART, init UART
  MT_UartInit ();
  MT_UartRegisterTaskID (HVACTest_TaskID);
  
  HVACTest_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  HVACTest_DstAddr.endPoint = HVACQUEEN_ENDPOINT;
  HVACTest_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  HVACTest_epDesc.endPoint = HVACQUEEN_ENDPOINT;
  HVACTest_epDesc.task_id = &HVACTest_TaskID;
  HVACTest_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&HVACTest_SimpleDesc;
  HVACTest_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &HVACTest_epDesc );
  
  // Register ZDO Message
  ZDO_RegisterForZDOMsg( HVACTest_TaskID, Device_annce );
  ZDO_RegisterForZDOMsg( HVACTest_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( HVACTest_TaskID, Match_Desc_rsp );

  // Init critical resource
  ptl0_initPTL0Status();
  
  // check valid MAC address?
  //
  // If a mac address copy is valid in flash, send "FF FF FF" Msg to 
  // STM32 and start the zigbee network.
  //
  // If not, send "FF FF 00" Msg to STM32, and prepare to handle MAC 
  // address message.
  //
  // The forth byte is the firmware version
  
// Test program, no need init white list
// However, to disable any device to join, enable the empty white list function
  
  // Add firmware version
  startupMsg[3] = firm_version;
    
  // send valid flash startup mag
  while(HalUARTWrite(0, startupMsg, 4) != 4)
  {
#ifdef WDT_IN_PM1
    // clear WDT
    WDCTL |= WDCLP1; 
    WDCTL |= WDCLP2;
#endif
    HalUARTPoll();
  }
   
  // start network directly, form a network and check
  ZDOInitDevice(0);
 
  // Init timer
  osal_start_timerEx( HVACTest_TaskID,
               HVAC_PTL0_GUT_EVT,
               HVAC_PTL0_FAIL_TIMEOUT );
  
  // WDT
#ifdef WDT_IN_PM1
  // Start WDT reset timer
  osal_start_timerEx( HVACTest_TaskID,
                      HVAC_WDT_CLEAR_EVT,
                      HVAC_WDT_CLEAR_TIMEOUT ); 
#endif
  
#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, HVACQUEEN_RTOS_MSG_EVT );
#endif
}

/*********************************************************************
 * @fn      HVACTest_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 HVACTest_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( HVACTest_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          break;

        case CMD_SERIAL_MSG:
          // UART data, uart handler
          HVACTest_HandleUart ((mtOSALSerialData_t *)MSGpkt);
          break;
          
        case ZDO_STATE_CHANGE:
          HVACTest_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (HVACTest_NwkState == DEV_ZB_COORD) ||
               (HVACTest_NwkState == DEV_ROUTER) ||
               (HVACTest_NwkState == DEV_END_DEVICE) )
          {
            // Change to a known device type
            if (HVACTest_errorcode == STM32_NORMAL )
              HVACTest_errorcode = HAS_ZIGBEE_NETWORK;
          }
          
          // State change. Report STM32. Report everything for now.
          // Need to consider!!!!!!!!!!!!!!
          PTL0_InitTypeDef outGoing_ptl0locUpdate;
           
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( HVACTest_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Event handler timer, 1s timer
  if (events & HVAC_PTL0_GUT_EVT )
  {
    // Timer event. Not GUT function
    test_counter --;
    
    if(!test_counter)
    {
      if (!test_infodisplay)
      {
        // check process finish, flash LED according to the check result
        test_counter = HVACTEST_INFODISPLAY; // display information for HVACTEST_INFODISPLAYs, then reset
        
        test_infodisplay = true;        // ready to display information
        
        // display LED according to HVACTest_errorcode
        switch(HVACTest_errorcode)
        {
          case STM32_NORMAL:
            // display NORMAL, STM32 was good, but no ZIGBEE netowk
            HalLedBlink (HAL_LED_3, 200, 15, 1000);
            break;
           
          case HAS_ZIGBEE_NETWORK:
            // all good
            HalLedSet( HAL_LED_3, HAL_LED_MODE_ON );
            break;
            
          case NO_FLASH:
            // STM32 fail to detect a flash
            HalLedBlink (HAL_LED_3, 200, 15, 2000);
            break;
            
          case NO_ADC:
            // STM32 ADC fail
            HalLedBlink (HAL_LED_3, 200, 85, 2000);
            break;
            
          case NO_HUAWEI:
            // STM32 no HUAWEI
            HalLedBlink (HAL_LED_3, 200, 50, 4000);
            break;
            
          case CC2530_INIT:
            // no STM32 detected
            HalLedBlink (HAL_LED_3, 200, 85, 1000);
            break;
        }
      }
      else
        ptl0_sendACK(); // display finish, send ACK to STM32 and 32 will reset
    }

      // Init timer
    osal_start_timerEx( HVACTest_TaskID,
               HVAC_PTL0_GUT_EVT,
               HVAC_PTL0_FAIL_TIMEOUT );
    
    return ( events ^ HVAC_PTL0_GUT_EVT ); 
  }  

#ifdef WDT_IN_PM1
  // WDT Reload Timer Event
  if(events & HVAC_WDT_CLEAR_EVT)
  {
    // clear WDT
    WDCTL |= WDCLP1; 
    WDCTL |= WDCLP2;
       
    // reload timer 
    osal_start_timerEx( HVACTest_TaskID,
                        HVAC_WDT_CLEAR_EVT,
                        HVAC_WDT_CLEAR_TIMEOUT ); 
    
    return (events ^ HVAC_WDT_CLEAR_EVT);
  }
#endif  
 
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      HVACTest_HandleUart
 *
 * @brief   Handles all UART events for this device.
 *
 * @param   pMsg - incoming uart msg
 *          pMsg->hdr : msg header. 
 *          pMsg->msg : msg buffer
 *          pMsg->msg[0], data length high 8 bits    
 *          pMsg->msg[1], data length low 8 bits
 *          pMsg->msg[2], CMD1
 *          pMsg->msg[3], CMD2
 *          pMsg->msg[4...], data payload (if there is one)
 *
 * @return  none
 */
static void HVACTest_HandleUart (mtOSALSerialData_t *pMsg) 
{
  PTL0_InitTypeDef inComing_ptl0;
  
  /* load the frame */
  inComing_ptl0.length = (pMsg->msg[0] << 8) + pMsg->msg[1]; //load length
  inComing_ptl0.CMD1 = pMsg->msg[2];    // load CMD1
  inComing_ptl0.CMD2 = pMsg->msg[3];    // load CMD2

  if(inComing_ptl0.length)              // if there is a data payload       
    inComing_ptl0.datapointer = &pMsg->msg[4]; // load data payload
  
  /* msg handler, react according to different tasks 
   *
   * refer to hvac_protocol0.h for more cmd detail
   */
  // Must have switch case, for ACK and PING command.
  // Also the mac address compare command
  switch(inComing_ptl0.CMD1)
  { 
    case PTL0_PING:
      // Ping command, send ack
      hvacUART_PTL0_PING((hvac_test_error_t)inComing_ptl0.CMD2);
      break;
  }
}

/*********************************************************************
 * @fn      hvacUART_PTL0_PING
 *
 * @brief   Receive PING CMD. Responce with ACK frame.
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_PING( hvac_test_error_t CMD2 )
{ 
  // Do not check available, handle directly
  
  // update error code according to test result
  HVACTest_errorcode = CMD2;
  
  // do not send ACK here, send ACK while finish display information

  // back to idle
  ptl0_updateStat(PTL0_STA_IDLE);
}

/*********************************************************************
 * @fn      hvac_STM32ResetInit
 *
 * @brief   Initialize relative I/O to control STM32 reset pin. 
 *          Use P2.0 as reset control.
 *
 * @param   none
 *
 * @return  none
 */
static void hvac_STM32ResetInit( void )
{
  // Defualt P2 value after reset is 1, set as pull up resistor,
  // so here only need to set the direction
  P2DIR |= 0x01;
}

/*********************************************************************
 */