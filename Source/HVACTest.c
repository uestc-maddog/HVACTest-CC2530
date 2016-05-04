/******************************************************************************
  Filename:       HVACQueen.c
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
const cId_t HVACQueen_ClusterList[HVACQUEEN_MAX_CLUSTERS] =
{
  HVACQUEEN_TRS_SEND_CLUSTERID,
  HVACQUEEN_ALIVE_CLUSTERID,
  HVACQUEEN_FW_CLUSTERID,
  HVACQUEEN_CONTROL,
  HVACQUEEN_FIRMWAREACK,
  HVACQUEEN_REMOTEACK
};

const SimpleDescriptionFormat_t HVACQueen_SimpleDesc =
{
  HVACQUEEN_ENDPOINT,              //  int Endpoint;
  HVACQUEEN_PROFID,                //  uint16 AppProfId[2];
  HVACQUEEN_DEVICEID,              //  uint16 AppDeviceId[2];
  HVACQUEEN_DEVICE_VERSION,        //  int   AppDevVer:4;
  HVACQUEEN_FLAGS,                 //  int   AppFlags:4;
  HVACQUEEN_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)HVACQueen_ClusterList,  //  byte *pAppInClusterList;
  HVACQUEEN_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)HVACQueen_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in HVACQueen_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t HVACQueen_epDesc;

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
byte HVACQueen_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // HVACQueen_Init() is called.

devStates_t HVACQueen_NwkState;

byte HVACQueen_TransID;  // This is the unique message ID (counter)

afAddrType_t HVACQueen_DstAddr;

static uint8 hvacSPIControl = false; // flag use to control SPI flash

// mac address valid flag in flash
static uint8 hvacMACADDRValid = false;

// mac address valid control flag
static uint8 hvacMACAvailable = false;

static uint8 hvacMACADDRNum = 0;

// store the previous message address information
// if the device is not belong to the network, system can remove the 
// device afterwards
static zAddrType_t lastMsgAddr;

// use this variable to enable reset pin trigger for 1 (or more) seconds
static uint8 hvacSTM32ResetSta = false;

// network address buffer 
static uint16 hvac_nwkAddr[HVAC_MAX_DRONE_NUM] = {0};

// Firmware upgrade
static uint16 hvacFirmwareShortAddr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void HVACQueen_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void HVACQueen_MessageMSGCB( afIncomingMSGPacket_t *pckt );

#if defined( IAR_ARMCM3_LM )
static void HVACQueen_ProcessRtosMessage( void );
#endif

// Zigbee announce handler
static uint8 hvacHandleZDODeviceAnnounce(zdoIncomingMsg_t * );

// UART handler functions
static void hvacUART_PTL0_PING( void );
static void hvacUART_PTL0_ACK( PTL0_InitTypeDef  );
static void hvacUART_PTL0_ERROR( PTL0_InitTypeDef );
static uint8 hvacUART_PTL0_TRS_TRANS( PTL0_InitTypeDef * );
static void hvacUART_PTL0_NWK_STATUS_RP( PTL0_InitTypeDef * );
static void hvacUART_PTL0_NWK_CMD( PTL0_InitTypeDef * );
static void hvacUART_PTL0_LOC_STATUS_RP( PTL0_InitTypeDef * );
static void hvacUART_PTL0_LOC_CMD( PTL0_InitTypeDef * );
static uint8 hvacUART_PTL0_FIRMWARE_INIT( PTL0_InitTypeDef * );
static uint8 hvacUART_PTL0_FIRMWARE( PTL0_InitTypeDef * );
static uint8 hvacUART_PTL0_FIRMWARE_LAST( PTL0_InitTypeDef * );
// MAC related
static void hvacUART_PTL0_MAC_REQ( PTL0_InitTypeDef );
static void hvacUART_PTL0_MAC_DATA( PTL0_InitTypeDef );
static void hvacUART_PTL0_MAC_END( void );

// AF Msg handler
static uint8 hvacMSGHandleTRS_SendMsg( afIncomingMSGPacket_t * );
static uint8 hvacHandleALIVE_Msg( afIncomingMSGPacket_t * );
static uint8 hvacMSGHandleCMD(afIncomingMSGPacket_t * );

// Reset Control
static void hvac_STM32ResetInit( void );
static void hvac_STM32ResetCtrl( uint8 );

// SBL 
#ifdef HVAC_SBL
static void appForceBoot(void);
#endif

// address management 
static uint8 hvac_LookupMac( uint16 * , uint8 * );
static uint8 hvac_LookupNwk( uint16 * , uint8 * );
static uint8 hvac_addMacNwkLookupEntry( uint16 * , uint8 *  );

#ifdef HAL_UART
static void HVACQueen_HandleUart (mtOSALSerialData_t *pMsg);
#endif


/*********************************************************************
 * TEST Related
 */
#ifdef HVAC_TEST
static uint8 testa = 0;
static uint8 testb = 0;
static uint8 testc = 0;
static uint8 testd = 0;
static uint8 teste = 0;
#endif

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
 * @fn      HVACQueen_Init
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
void HVACQueen_Init( uint8 task_id )
{
  HVACQueen_TaskID = task_id;
  HVACQueen_NwkState = DEV_INIT;
  HVACQueen_TransID = 0;
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
  MT_UartRegisterTaskID (HVACQueen_TaskID);
  
  HVACQueen_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  HVACQueen_DstAddr.endPoint = HVACQUEEN_ENDPOINT;
  HVACQueen_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  HVACQueen_epDesc.endPoint = HVACQUEEN_ENDPOINT;
  HVACQueen_epDesc.task_id = &HVACQueen_TaskID;
  HVACQueen_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&HVACQueen_SimpleDesc;
  HVACQueen_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &HVACQueen_epDesc );
  
  // Register ZDO Message
  ZDO_RegisterForZDOMsg( HVACQueen_TaskID, Device_annce );
  ZDO_RegisterForZDOMsg( HVACQueen_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( HVACQueen_TaskID, Match_Desc_rsp );

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
#ifdef HVAC_WHITELIST  
  // init flash items
  osal_nv_item_init(HVAC_VALIDFLASH_ITEM, 1, NULL);
  osal_nv_item_init(HVAC_MACANUM_ITEM, 1, NULL);
  osal_nv_item_init(HVAC_MACADDRESS_ITEM, HVAC_MAC_FLAHS_MAXLEN, NULL);
  osal_nv_item_init(HVAC_NETWORKADDR_ITEM, HVAC_NET_FLAHS_MAXLEN, NULL);
  
  osal_nv_read(HVAC_VALIDFLASH_ITEM, 0, 1, &hvacMACADDRValid);
  
  if(hvacMACADDRValid != HVAC_MAC_FLASH_VALID)
  {
    hvacMACADDRValid = HVAC_MAC_FLASH_INVALID; 
  }
  // value valid, read mac number to RAM.
  else
  {
    osal_nv_read(HVAC_MACANUM_ITEM, 0, 1, &hvacMACADDRNum);
    // send valid flash startup mag
    startupMsg[2] = 0xFF;
    
    // configure white table
    osal_nv_read(HVAC_MACADDRESS_ITEM, 0
                 , (hvacMACADDRNum * Z_EXTADDR_LEN), hvac_whiteList);
    // configure network table 
    osal_nv_read(HVAC_NETWORKADDR_ITEM, 0
                 , (hvacMACADDRNum * 2), hvac_nwkAddr);
    
    // start cc2530 network
    ZDOInitDevice(0);
    // set mac address available
    hvacMACAvailable = true;
  }
#else
  // by pass the MAC validation
  hvacMACADDRValid = HVAC_MAC_FLASH_VALID;
  
  // start cc2530 network
  ZDOInitDevice(0);
  // set mac address available
  hvacMACAvailable = true;
#endif
  
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
   
#ifdef HVAC_TEST
  //testa = 0;
  // start cc2530 network
  ZDOInitDevice(0);
  // set mac address available
  hvacMACAvailable = true;  
#endif
  
  // Init timer
  osal_start_timerEx( HVACQueen_TaskID,
               HVAC_PTL0_GUT_EVT,
               HVAC_PTL0_FAIL_TIMEOUT );
  
  // WDT
#ifdef WDT_IN_PM1
  // Start WDT reset timer
  osal_start_timerEx( HVACQueen_TaskID,
                      HVAC_WDT_CLEAR_EVT,
                      HVAC_WDT_CLEAR_TIMEOUT ); 
#endif
  
#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, HVACQUEEN_RTOS_MSG_EVT );
#endif
}

/*********************************************************************
 * @fn      HVACQueen_ProcessEvent
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
uint16 HVACQueen_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( HVACQueen_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          HVACQueen_ProcessZDOMsgs((zdoIncomingMsg_t *)MSGpkt);
          break;

        case CMD_SERIAL_MSG:
          // UART data, uart handler
          HVACQueen_HandleUart ((mtOSALSerialData_t *)MSGpkt);
          break;
          
        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;

          sentEP = afDataConfirm->endpoint;
          (void)sentEP;  // This info not used now
          sentTransID = afDataConfirm->transID;
          (void)sentTransID;  // This info not used now

          sentStatus = afDataConfirm->hdr.status;
          // Action taken when confirmation is received.
          
          // Only reply ACK/ERROR when perform TRS_SEND command
          if (ptl0_queryStat() == PTL0_STA_TRS_TRANS_REC)
          {
            if ( sentStatus != ZSuccess )
            {
              // The data wasn't delivered -- Do something
              // data not able to delivery, error code
              ptl0_sendError(PTL0_ERROR_NotDeliver);
              
              // update status
              ptl0_updateStat(PTL0_STA_IDLE);
            }
            else
            {
              // data delivered, ACK to STM32
              
              // now the firmware ACK is done in HVACQUEEN_FIRMWARE AF data handler
              
              // send ACK
              ptl0_sendACK();
              
              // send ACK, communication complete, back to idle
              ptl0_updateStat(PTL0_STA_IDLE);
            }
          }
          break;

        case AF_INCOMING_MSG_CMD:
          HVACQueen_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          HVACQueen_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (HVACQueen_NwkState == DEV_ZB_COORD) ||
               (HVACQueen_NwkState == DEV_ROUTER) ||
               (HVACQueen_NwkState == DEV_END_DEVICE) )
          {
            // Change to a known device type
            // Do nothing now, CC2530 will upload everything.
          }
          
          // State change. Report STM32. Report everything for now.
          // Need to consider!!!!!!!!!!!!!!
          PTL0_InitTypeDef outGoing_ptl0locUpdate;
          
          // configure error code
          outGoing_ptl0locUpdate.CMD1 = PTL0_LOC_STATUS_RP;
          outGoing_ptl0locUpdate.CMD2 = PTL0_LOC_STATUS_RP_NWK_CHG;
          outGoing_ptl0locUpdate.length = 1;
          outGoing_ptl0locUpdate.SOF = PTL0_SOF;
          outGoing_ptl0locUpdate.version = PTL0_FRAMEVER;
          outGoing_ptl0locUpdate.datapointer = &(MSGpkt->hdr.status);
          
          // upload Msg
          ptl0_uploadMsg(outGoing_ptl0locUpdate
                         ,outGoing_ptl0locUpdate.datapointer
                         ,outGoing_ptl0locUpdate.length
                         ,HVACQueen_TaskID);   
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( HVACQueen_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Event handler timer, 1s timer
  if (events & HVACPTL0_EVENT_TIMEOUT_EVT )
  {
    // call event timer service function
    ptl0_eventTimerService(HVACQueen_TaskID
                           ,HVACPTL0_EVENT_TIMEOUT_EVT);
                                 
    return ( events ^ HVACPTL0_EVENT_TIMEOUT_EVT ); 
  }  

  // Resend timer
  if (events & HVAC_PTL0_GUT_EVT)
  {
    if (ptl0_resend_Enable)
    {
      ptl0_send_timer --;
      
      if (!ptl0_send_timer)
      {
        ptl0_send_rep --;
        
        // reload timer
        ptl0_send_timer = HVAC_PTL0_FAIL_TIMER;
        
        // still trying
        if (ptl0_send_rep)
        {
          // send the message again
          ptl0_sendMsg(*ptl0_send_EventStruc);    
          
          if(ptl0_send_EventStruc->CMD1 == PTL0_ACK)
            ptl0_updateStat(PTL0_STA_IDLE);            
        }
        
        // it already tried HVAC_PTL0_FAIL_REPEAT times
        else
        {
          // reset STM32
          hvac_STM32ResetCtrl(HVAC_STM32_RESET);
        }
      }
    }
    
    // STM32 reset control
    if (hvacSTM32ResetSta)
    {
      hvacSTM32ResetSta --;
    }
    else
    {
      hvac_STM32ResetCtrl(HVAC_STM32_RECOVER);
    }
    
    // reload the timer
    osal_start_timerEx( HVACQueen_TaskID,
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
    osal_start_timerEx( HVACQueen_TaskID,
                        HVAC_WDT_CLEAR_EVT,
                        HVAC_WDT_CLEAR_TIMEOUT ); 
    
    return (events ^ HVAC_WDT_CLEAR_EVT);
  }
#endif  
  
#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & HVACQUEEN_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    HVACQueen_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ HVACQUEEN_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      HVACQueen_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void HVACQueen_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case Device_annce:
      if(hvacHandleZDODeviceAnnounce(inMsg))
        HalLedBlink (HAL_LED_3, 8, 30, 125);
      break;
      
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            HVACQueen_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            HVACQueen_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            HVACQueen_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      HVACQueen_HandleUart
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
static void HVACQueen_HandleUart (mtOSALSerialData_t *pMsg) 
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
      hvacUART_PTL0_PING();
      break;
      
    case PTL0_LOC_CMD:
      // Local command
      hvacUART_PTL0_LOC_CMD(&inComing_ptl0);
      break;  
        
    case PTL0_ACK:
      // ACK received, clear flag
      hvacUART_PTL0_ACK(inComing_ptl0);
      break;   
      
    case PTL0_MAC_REQ:
      hvacUART_PTL0_MAC_REQ(inComing_ptl0);
      break;
      
    case PTL0_MAC_DATA:
      hvacUART_PTL0_MAC_DATA(inComing_ptl0);
      break; 
      
    case PTL0_MAC_END:
      hvacUART_PTL0_MAC_END();
      break;
      
    default:
      break;
  }
  
   
  // if mac address valid, network established. Accept external command  
  if(hvacMACAvailable)
  {
    switch(inComing_ptl0.CMD1)
      {
      case PTL0_ERROR:
        // ERROR received, responce
        hvacUART_PTL0_ERROR(inComing_ptl0);
        break;
        
      case PTL0_TRS_TRANS:
        // Transmission command, send to destination
        hvacUART_PTL0_TRS_TRANS(&inComing_ptl0);
        break;
          
      case PTL0_NWK_STATUS_RP:
        // Network status report 
        hvacUART_PTL0_NWK_STATUS_RP(&inComing_ptl0);
        break;
            
      case PTL0_NWK_CMD:
        // Network command  
        hvacUART_PTL0_NWK_CMD(&inComing_ptl0);
        break;
            
      case PTL0_LOC_STATUS_RP:
        // Local status report
        hvacUART_PTL0_LOC_STATUS_RP(&inComing_ptl0);
        break;
             
      case PTL0_FIRMWARE_INIT:
        // firmware init command
        hvacUART_PTL0_FIRMWARE_INIT(&inComing_ptl0);
        break;

      case PTL0_FIRMWARE:
        // firmware init command
        hvacUART_PTL0_FIRMWARE(&inComing_ptl0);
        break;

      case PTL0_FIRMWARE_LAST:
        // firmware init command
        hvacUART_PTL0_FIRMWARE_LAST(&inComing_ptl0);
        break;
        
      default:
        break;
    }
  }
}


/*********************************************************************
 * @fn      hvacUART_PTL0_MAC_REQ
 *
 * @brief   Receive MAC transmission request. Reply ACK.
 *
 * @param   inComingPTL0 - in coming ptl0 message
 *
 * @return  none
 */
static void hvacUART_PTL0_MAC_REQ( PTL0_InitTypeDef inComingPTL0 )
{ 
  // if mac address not valid, write number to flash
  osal_nv_write(HVAC_MACANUM_ITEM, 0, 1, &(inComingPTL0.CMD2));

  // set address not available
  hvacMACAvailable = false;
  
  // if mac valid flag set in flash, set to invalid before any operation
  if(hvacMACADDRValid)
  {
    hvacMACADDRValid = 0;
    osal_nv_write(HVAC_VALIDFLASH_ITEM, 0, 1, &hvacMACADDRValid);
  }
  
  // decide whether update mac number
  if (hvacMACADDRNum != inComingPTL0.CMD2)
  {
    hvacMACADDRNum = inComingPTL0.CMD2;
    osal_nv_write(HVAC_MACANUM_ITEM, 0, 1, &hvacMACADDRNum);
  }
    
  // send ACK
  ptl0_sendACK();
  
  // back to idle
  ptl0_updateStat(PTL0_STA_IDLE);
}


/*********************************************************************
 * @fn      hvacUART_PTL0_MAC_DATA
 *
 * @brief   Receive MAC data frame. Read relative flash page see 
 *          wheter there is a same value. If not or invalid data, 
 *          write data into flash.
 *
 * @param   inComingPTL0 - in coming ptl0 message
 *
 * @return  none
 */
static void hvacUART_PTL0_MAC_DATA( PTL0_InitTypeDef inComingPTL0 )
{
  uint8 cmpResult = false;
  uint8 cmpTemp[Z_EXTADDR_LEN];
  
  // read relative MAC address in flash
  osal_nv_read(HVAC_MACADDRESS_ITEM, Z_EXTADDR_LEN * (inComingPTL0.CMD2 - 1)
               , Z_EXTADDR_LEN, cmpTemp);
  // compare two mac address
  cmpResult = AddrMgrExtAddrEqual(inComingPTL0.datapointer, cmpTemp);

  
  // if compare not equal, or data not valid, write flash
  if ((cmpResult == false) || !hvacMACADDRValid)
    osal_nv_write(HVAC_MACADDRESS_ITEM, Z_EXTADDR_LEN * (inComingPTL0.CMD2 - 1)
                 , Z_EXTADDR_LEN, inComingPTL0.datapointer);
    
  // finish, send ACK
  ptl0_sendACK();
  
  // back to idle
  ptl0_updateStat(PTL0_STA_IDLE);
}


/*********************************************************************
 * @fn      hvacUART_PTL0_MAC_END
 *
 * @brief   Receive Mac End CMD frame. No more mac address. Start 
 *          network.
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_MAC_END( void )
{
  // clear all table component after hvacMACADDRNum
  osal_memset(&hvac_whiteList[hvacMACADDRNum][0],0, 
              (sizeof(hvac_whiteList) - hvacMACADDRNum * Z_EXTADDR_LEN));
  
  // write valid flash flag  
  hvacMACADDRValid = HVAC_MAC_FLASH_VALID;
  osal_nv_write(HVAC_VALIDFLASH_ITEM, 0, 1, &hvacMACADDRValid);
  
  // finish, send ACK
  ptl0_sendACK();
  
  // back to idle
  ptl0_updateStat(PTL0_STA_IDLE);

#ifdef HVAC_WHITELIST  
  // update white table
  osal_nv_read(HVAC_MACADDRESS_ITEM, 0
               , (hvacMACADDRNum * Z_EXTADDR_LEN), hvac_whiteList);
#endif
  
  // start cc2530 network
  if ( devState == DEV_HOLD )
    ZDOInitDevice(0);
  
  // set mac address available
  hvacMACAvailable = true;
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
static void hvacUART_PTL0_PING( void )
{ 
  // If PTL0 idle, send ACK
  if(ptl0_queryStat() == PTL0_STA_IDLE)
  {
    // update PTL0 status
    ptl0_updateStat(PTL0_STA_PING_REC);
  
    // send ACK
    ptl0_sendACK();
    
    // back to idle
    ptl0_updateStat(PTL0_STA_IDLE);
  }
}

/*********************************************************************
 * @fn      hvacUART_PTL0_ACK
 *
 * @brief   Receive ACK. Set PTL0 status.
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_ACK( PTL0_InitTypeDef inComing_ptl0 )
{
  // if not in idle, then the transmission should comes to an end
  if (ptl0_queryStat() != PTL0_STA_IDLE)
      // stop timer
  {
    ptl0_resend_Enable = false;
  }
  
  // check current state, see whether anything need to do
  switch(ptl0_queryStat())
  { 
    case PTL0_STA_LOCCMD_SENT:
    // send CMD to STM32?   
      // If request SPI?
      if(inComing_ptl0.CMD2 == PTL0_LOC_CMD_REQSPI)
        hvacSPIControl = true;
      
      // back to idle
      ptl0_updateStat(PTL0_STA_IDLE);
      break;
      
    default:
      // By default, receive ACK, communication complete, back to idle
      ptl0_updateStat(PTL0_STA_IDLE);
      break;
  } 
}

/*********************************************************************
 * @fn      hvacUART_PTL0_ERROR
 *
 * @brief   Receive ACK. Set PTL0 status.
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_ERROR( PTL0_InitTypeDef inComing_ptl0 )
{
  uint8 afMsgBuf[10] = {0};
  
  // if not in idle, then the transmission should comes to an end
  if (ptl0_queryStat() != PTL0_STA_IDLE)
      // stop timer
    ptl0_resend_Enable = false;
  
  // check current state, see whether anything need to do
  switch(ptl0_queryStat())
  { 
    case PTL0_STA_LOCCMD_SENT:
    // send CMD to STM32?   
      // If request SPI?
      if(inComing_ptl0.CMD2 == PTL0_LOC_CMD_REQSPI)
        hvacSPIControl = false;
      
      // back to idle
      ptl0_updateStat(PTL0_STA_IDLE);
      break;
      
    case PTL0_STA_TRS_SENT:
    case PTL0_NWK_STATUS_RP:
    // Send Transparent/HB
      // If receiving message not belong to this network
      if (inComing_ptl0.CMD2 == PTL0_ERROR_NoAvailabeDevice)
      {
        // Remove corresponding device
        // not remove children, ensable rejoin, enable secure
        if(!hvac_LookupNwk(&lastMsgAddr.addr.shortAddr, lastMsgAddr.addr.extAddr))
        {       
          // update status
          ptl0_updateStat(PTL0_STA_IDLE);
          
          // query fail, break loop
          break;
        }
          
        ZDP_MgmtLeaveReq(&lastMsgAddr, lastMsgAddr.addr.extAddr,0,1,1); // need test!!!!!!!!!!!!!!!!!!!!!!!!!
        
        // Send a message to drone, reset CC2530, force to leave network
        afMsgBuf[0] = 0;        // DLEN = 0
        afMsgBuf[1] = PTL0_NWK_CMD;        // PTL0_NWK_CMD command
        afMsgBuf[2] = PTL0_NWK_CMD_RESET_DEV_2530;      // Reset 2530 command
          
        // configure send structure  
        HVACQueen_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
        HVACQueen_DstAddr.addr.shortAddr = (uint16)lastMsgAddr.addr.shortAddr;
        // Take the first endpoint, Can be changed to search through endpoints
        HVACQueen_DstAddr.endPoint = HVACQUEEN_ENDPOINT;
        
        if ( AF_DataRequest( &HVACQueen_DstAddr, &HVACQueen_epDesc,
                             HVACQUEEN_CONTROL,
                             (byte)3,
                             (byte *)afMsgBuf,
                             &HVACQueen_TransID,
                             AF_EN_SECURITY, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
        {
          // Successfully requested to be sent.
 
          // Flash LED to indicate network active
          HalLedBlink (HAL_LED_3, 8, 30, 125);
        }
        else
        {
          // Fail, do something
        }
      }
      
      // back to idle
      ptl0_updateStat(PTL0_STA_IDLE);      
      break;

    default:
      // By default, receive ACK, communication complete, back to idle
      ptl0_updateStat(PTL0_STA_IDLE);
      break;
  } 
}

/*********************************************************************
 * @fn      hvacUART_PTL0_TRS_TRANS
 *
 * @brief   Receiving transparent sending frame, send AF Msg.
 *          CC2530 should reply with either success ACK or error code.
 *
 * @param   none
 *
 * @return  none
 */
static uint8 hvacUART_PTL0_TRS_TRANS( PTL0_InitTypeDef *ptl0_buf )
{
  PTL0_TransTypeDef inComing_ptl0trs;
  uint8 coorMacAddrTemp[Z_EXTADDR_LEN] = {0};
  uint16 nwkAddr_temp;
  uint8 *sendBufTemp;
  
  // Only proceed when the PTL0 is idle
  if(ptl0_queryStat() == PTL0_STA_IDLE)
  {  
    // update stat, receive TRS_Send
    ptl0_updateStat(PTL0_STA_TRS_TRANS_REC);
    
    // set default value
    nwkAddr_temp = 0x0000;
    
    // extract ptl0 message to PTL0_TransTypeDef
    inComing_ptl0trs.datapointer = ptl0_buf->datapointer;
    inComing_ptl0trs.length = ptl0_buf->length;
    inComing_ptl0trs.macAddrptr = ptl0_buf->datapointer;
    inComing_ptl0trs.subdataptr = ptl0_buf->datapointer + Z_EXTADDR_LEN; // first 8 bytes for mac address

    // lookup network address
    if(AddrMgrExtAddrEqual(coorMacAddrTemp, inComing_ptl0trs.macAddrptr) == true)
    {
      // send msg to coordinator
      // method may not correct!!!!!!!!!!!!!!!!!!!!!!!!!!
      
      // get network address to network coordinator
      nwkAddr_temp = 0x0000;
    }
    // its a drone device?
    else if(hvac_LookupNwk(&nwkAddr_temp, inComing_ptl0trs.macAddrptr) == false)
    {
      // fail, send destination cc2530 not exist in network CMD
      // Send error
      ptl0_sendError(PTL0_ERROR_NoAvailabeDevice);
      
      // update status
      ptl0_updateStat(PTL0_STA_IDLE);
      
      return false;
    }
    
    // Get buffer to send info
    sendBufTemp = osal_mem_alloc((inComing_ptl0trs.length - Z_EXTADDR_LEN) + 1);
    *sendBufTemp = (inComing_ptl0trs.length - Z_EXTADDR_LEN);
    memcpy((sendBufTemp + 1), inComing_ptl0trs.subdataptr, *sendBufTemp);
    
    
    // configure send structure  
    HVACQueen_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    HVACQueen_DstAddr.addr.shortAddr = nwkAddr_temp;
    // Take the first endpoint, Can be changed to search through endpoints
    HVACQueen_DstAddr.endPoint = HVACQUEEN_ENDPOINT;
    
    if ( AF_DataRequest( &HVACQueen_DstAddr, &HVACQueen_epDesc,
                         HVACQUEEN_TRS_SEND_CLUSTERID,
                         (byte)(inComing_ptl0trs.length - Z_EXTADDR_LEN + 1),
                         (byte *)inComing_ptl0trs.subdataptr,
                         &HVACQueen_TransID,
                         (AF_ACK_REQUEST & AF_EN_SECURITY), AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      // Successfully requested to be sent.
      // Wait the AF ACK, then send ACK to STM32
      
      // release memory
      if (sendBufTemp != NULL)
        osal_mem_free(sendBufTemp);
      
      // Flash LED to indicate network active
      HalLedBlink (HAL_LED_3, 8, 30, 125);
      
      return true;
    }
    else
    {
      // Error occurred in request to send.
      // Send error
      ptl0_sendError(PTL0_ERROR_NotOTA);
      
      // update status
      ptl0_updateStat(PTL0_STA_IDLE);
      
      // release memory
      if (sendBufTemp != NULL)
        osal_mem_free(sendBufTemp);
      
      return false;
    }
  }
  return false;
}

/*********************************************************************
 * @fn      hvacUART_PTL0_NWK_STATUS_RP
 *
 * @brief   Receiving network status report frame. Not available for 
 *          CC2530.
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_NWK_STATUS_RP( PTL0_InitTypeDef *ptl0_buf )
{
  // Not an option for CC2530, network report only initialize by 
  // CC2530 and received by STM32. 
  asm("NOP");
}

/*********************************************************************
 * @fn      hvacUART_PTL0_NWK_CMD
 *
 * @brief   Receiving network cmd report frame. 
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_NWK_CMD( PTL0_InitTypeDef *ptl0_buf )
{
  // prepare CMD frame
  uint8 tempALLDEV_START[3] = {0,PTL0_NWK_CMD,PTL0_NWK_CMD_ALLDEV_START};
  
  if(ptl0_queryStat() == PTL0_STA_IDLE)
  {
    // update status, receive network cmd frame
    ptl0_updateStat(PTL0_STA_NWKCMD_REC);
    
    switch(ptl0_buf->CMD2)
    {
      // response according to different command
      
      case PTL0_NWK_CMD_ALLDEV_START:
      // Receiving ALL Device start CMD, send a broad cast msg to all devices
      // in network.       
        // configure send structure, broadcast this msg to everyone in the network
        HVACQueen_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
        HVACQueen_DstAddr.addr.shortAddr = 0xFFFF;
        // Take the first endpoint, Can be changed to search through endpoints
        HVACQueen_DstAddr.endPoint = HVACQUEEN_ENDPOINT;
        
        if ( AF_DataRequest( &HVACQueen_DstAddr, &HVACQueen_epDesc,
                             HVACQUEEN_CONTROL,
                             (byte)3,
                             (byte *)tempALLDEV_START,
                             &HVACQueen_TransID,
                             AF_EN_SECURITY, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
        {
          // Successfully requested to be sent.
          // send ACK back
          ptl0_sendACK();
          // send ACK, communication complete, back to idle
          ptl0_updateStat(PTL0_STA_IDLE);
          
          // Flash LED to indicate network active
          HalLedBlink (HAL_LED_3, 8, 30, 125);
        }
        else
        {
          // Error occurred in request to send.
          // Send error
          ptl0_sendError(PTL0_ERROR_NotOTA);
          
          // update status
          ptl0_updateStat(PTL0_STA_IDLE);
        }
        break;
        
      default:
      // default procedure, send ACK back
        // send ACK
        ptl0_sendACK();
        // send ACK, communication complete, back to idle
        ptl0_updateStat(PTL0_STA_IDLE);
        break;
    }
  }
}

/*********************************************************************
 * @fn      hvacUART_PTL0_LOC_STATUS_RP
 *
 * @brief   Receiving local status report.
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_LOC_STATUS_RP( PTL0_InitTypeDef *ptl0_buf )
{
  if(ptl0_queryStat() == PTL0_STA_IDLE)
  {
    // update status, receive local status report frame
    ptl0_updateStat(PTL0_STA_LOCSTARP_REC);
    
    switch(ptl0_buf->CMD2)
    {
      // response according to different command
      default:
      // default procedure, send ACK back
        // send ACK
        ptl0_sendACK();
        // send ACK, communication complete, back to idle
        ptl0_updateStat(PTL0_STA_IDLE);
        break;
    }
  }
}

/*********************************************************************
 * @fn      hvacUART_PTL0_LOC_CMD
 *
 * @brief   receiving local cmd report.
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_LOC_CMD( PTL0_InitTypeDef *ptl0_buf )
{
  uint8 upgrade_Indicator;
  
  if(ptl0_queryStat() == PTL0_STA_IDLE)
  {
    // variables used in PTL0_LOC_CMD_MACREQ
    uint8 * macTemp;
    uint8 resetdelay;
    uint8 macDataTemp[Z_EXTADDR_LEN];
    PTL0_InitTypeDef uartPtl0Temp;
        
    // update status, receive local cmd frame
    ptl0_updateStat(PTL0_STA_LOCCMD_REC);
    
    switch(ptl0_buf->CMD2)
    {
      // response according to different command
      case PTL0_LOC_CMD_MACREQ:
        // STM32 asks for its CC2530 MAC address      
        // get mac address
        macTemp = NLME_GetExtAddr();
        // prepare mac data
        memcpy(macDataTemp, macTemp, Z_EXTADDR_LEN);       
  
        // configurate the ACK structure
        uartPtl0Temp.SOF = PTL0_SOF;
        uartPtl0Temp.version = PTL0_FRAMEVER;
        uartPtl0Temp.length = Z_EXTADDR_LEN;   
        uartPtl0Temp.CMD1 = PTL0_ACK;
        uartPtl0Temp.CMD2 = PTL0_ACK_MAC;
        uartPtl0Temp.datapointer = macDataTemp;
        
        // send through PTL0 UART
        ptl0_sendMsg(uartPtl0Temp);
        // process finish, back to idle
        ptl0_updateStat(PTL0_STA_IDLE);
        break;
        
      case PTL0_LOC_CMD_RESETSBL:  
        // STM32 force CC2530 reset to SBL
#ifdef  HVAC_SBL
        // configurate the ACK structure
        uartPtl0Temp.SOF = PTL0_SOF;
        uartPtl0Temp.version = PTL0_FRAMEVER;
        uartPtl0Temp.length = 0;      // ACK frame, no data payload
        uartPtl0Temp.CMD1 = PTL0_ACK;
        uartPtl0Temp.CMD2 = PTL0_ACK_SBL;
        
        // send out the message
        ptl0_sendMsg(uartPtl0Temp);
        
        // delay, wait for UART send
        for(resetdelay = 0;resetdelay <= 100;resetdelay++)
          HalUARTPoll();
        
        // since the chip will reset, no need to handle the PTL0 status.
        
        // goes to SBL
        appForceBoot();
#endif        
        break;
        
      default:
      // default procedure, send ACK back
        // send ACK
        ptl0_sendACK();
        // send ACK, communication complete, back to idle
        ptl0_updateStat(PTL0_STA_IDLE);
        break;
    }
  }
}

/*********************************************************************
 * @fn      hvacUART_PTL0_FIRMWARE_INIT
 *
 * @brief   receiving firmware transmit init cmd. Prepare for firmware
 *          transmit.
 *
 *          This command will only used by Queen to send Msg on the 
 *          air. 
 *
 * @param   none
 *
 * @return  none
 */
uint8 hvacUART_PTL0_FIRMWARE_INIT( PTL0_InitTypeDef *ptl0_buf )
{
  // If the status is idle and not performing upgrade
#if defined HVAC_TEST
  ptl0_firmwareUpgrade = false;
#endif
  uint8 macTemp[Z_EXTADDR_LEN] = {0};
  uint8 processUpload = false;
  uint8 sendBufTemp[15] = {0}; 
  
  if(ptl0_queryStat() == PTL0_STA_IDLE)
  {
    if (ptl0_firmwareUpgrade)
    {
      // already start a firmware upgrade
      // check whether the firmware upgrade target is same as previous
      
      hvac_LookupMac( &hvacFirmwareShortAddr, macTemp );
      
      if (AddrMgrExtAddrEqual(macTemp, ptl0_buf->datapointer))
        processUpload = true;
      else
      {
        // fail, already transmitting an firmware
        // Send error
        ptl0_sendError(PTL0_ERROR_DEVBUSY);
        
        // update status
        ptl0_updateStat(PTL0_STA_IDLE);
        
        return false; 
      }
    }
    
    else if ((!ptl0_firmwareUpgrade) || processUpload)
    {
      // update status, receive local cmd frame
      ptl0_updateStat(PTL0_STA_FIRM_INIT_REC);
      
      // prepare buffer to be sent
      sendBufTemp[0] = 12;  // Data Len
      sendBufTemp[1] = PTL0_FIRMWARE_INIT;  // CMD1
      sendBufTemp[2] = PTL0_EMPTYCMD;  // CMD2
      
      // leave mac address empty/unchanged
      
      // load sequence number to the global variables
      // 9th and 10th byte in data payload is the sequence number
      ptl0_lastfirmwareSeqNum = ((uint16)*(ptl0_buf->datapointer + 8) << 8)
        + *(ptl0_buf->datapointer + 9);
      sendBufTemp[11] = *(ptl0_buf->datapointer + 8);
      sendBufTemp[12] = *(ptl0_buf->datapointer + 9);
      
      // The 11th byte indicates its a CC2530 firmware or STM32 firmware by 0x01
      // (STM32) or 0x02(CC2530). 
      sendBufTemp[13] = *(ptl0_buf->datapointer + 10);
      
      // The 12th byte leave as the transmitting firmware version
      sendBufTemp[14] = *(ptl0_buf->datapointer + 11);
      
      // get sequence counter ready
      ptl0_firmwareSeqNum = 1;
      
      // set firmware upgrade flag
      ptl0_firmwareUpgrade = true;
      
      // Okay then, send a message to drone, let drone prepare for 
      // firmware update.
      // lookup the short address
      if (hvac_LookupNwk( &hvacFirmwareShortAddr, ptl0_buf->datapointer ) == false)
      {
        // no such mac address valid
        ptl0_sendError(PTL0_ERROR_NoAvailabeDevice);
        
        // update status
        ptl0_updateStat(PTL0_STA_IDLE);
        
        // de-set upgrade flag
        ptl0_firmwareUpgrade = false;
        
        return false;
      }
      
      HVACQueen_DstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
      HVACQueen_DstAddr.addr.shortAddr = hvacFirmwareShortAddr;
      // Take the first endpoint, Can be changed to search through endpoints
      HVACQueen_DstAddr.endPoint = HVACQUEEN_ENDPOINT;
      
      if ( AF_DataRequest( &HVACQueen_DstAddr, &HVACQueen_epDesc,
                           HVACQUEEN_CONTROL,
                           (byte)15,
                           (byte *)sendBufTemp,
                           &HVACQueen_TransID,
                           AF_EN_SECURITY, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
      {
        // Successfully requested to be sent.
        // No ACK send here. While STM32 receiving this CMD and reply 
        // other side CC2530, the message should be replied

        // Flash LED to indicate network active
        HalLedBlink (HAL_LED_3, 8, 30, 125);
            
        return true;
      }
      return false;
    }
    else
      return false;
  }

  return false;
}


/*********************************************************************
 * @fn      hvacUART_PTL0_FIRMWARE
 *
 * @brief   Receiving firmware data, send through the air.
 *
 *          This command will only used by Queen to send Msg on the 
 *          air. 
 *
 * @param   none
 *
 * @return  none
 */
uint8 hvacUART_PTL0_FIRMWARE( PTL0_InitTypeDef *ptl0_buf )
{
  uint16 firmwareseqTemp;
  // send back error code, sequence number not match          
  PTL0_InitTypeDef outGoing_ptl0error;
  uint8 expectingSeq[2] = {0};   
  
  if(ptl0_queryStat() == PTL0_STA_IDLE)
  {
    // already set up upgrade?
    if (ptl0_firmwareUpgrade)
    {
      // sequence match?
      firmwareseqTemp = ((uint16)*(ptl0_buf->datapointer) << 8)
        + *(ptl0_buf->datapointer + 1);
      
      if (firmwareseqTemp == ptl0_firmwareSeqNum)
      {    
        // update status, receive local cmd frame
        ptl0_updateStat(PTL0_STA_FIRM_REC);
        
        // prepare information send on the air
        HVACQueen_DstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
        HVACQueen_DstAddr.addr.shortAddr = hvacFirmwareShortAddr;
        // Take the first endpoint, Can be changed to search through endpoints
        HVACQueen_DstAddr.endPoint = HVACQUEEN_ENDPOINT;
        
        // send data on the air
        if ( AF_DataRequest( &HVACQueen_DstAddr, &HVACQueen_epDesc,
                             HVACQUEEN_FW_CLUSTERID,
                             (byte)(PTL0_FIRMWARE_DLEN + 2),
                             (byte *)ptl0_buf->datapointer,
                             &HVACQueen_TransID,
                             ( AF_EN_SECURITY), AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
        {
          // success send over the air
          // do nothing now..
          // wait Zigbee ACK, then send PTL0 ACK
          
          // Flash LED to indicate network active
          HalLedBlink (HAL_LED_3, 8, 30, 125);
        }
        else
        {
          // Error occurred in request to send.
          // Send error 
          ptl0_sendError(PTL0_ERROR_NotOTA);
          
          // update status
          ptl0_updateStat(PTL0_STA_IDLE);
        }   
        return true;
      }
      else
      {
        expectingSeq[0] = (uint8)((ptl0_firmwareSeqNum & 0xFF00) >> 8);
        expectingSeq[1] = (uint8)(ptl0_firmwareSeqNum & 0x00FF);
          
        // configure error code
        outGoing_ptl0error.CMD1 = PTL0_ERROR;
        outGoing_ptl0error.CMD2 = PTL0_ERROR_SEQMISMATCH;
        outGoing_ptl0error.length = 2;
        outGoing_ptl0error.SOF = PTL0_SOF;
        outGoing_ptl0error.version = PTL0_FRAMEVER;
        outGoing_ptl0error.datapointer = expectingSeq;
        
        // send error information to STM32
        ptl0_sendMsg(outGoing_ptl0error);
        
        // update status
        ptl0_updateStat(PTL0_STA_IDLE);
      }
    }
    else
    {
      // send back error code, need init first
      // Not likely to happen. 
      ptl0_sendError(PTL0_ERROR_DEVBUSY);
      
      // update status
      ptl0_updateStat(PTL0_STA_IDLE);
    }
  }

  return false;
}


/*********************************************************************
 * @fn      hvacUART_PTL0_FIRMWARE_LAST
 *
 * @brief   receiving firmware transmit last packet. Prepare for 
 *          firmware transmit.
 *
 *          This command will only used by Queen to send Msg on the 
 *          air. 
 *
 * @param   none
 *
 * @return  none
 */
uint8 hvacUART_PTL0_FIRMWARE_LAST( PTL0_InitTypeDef *ptl0_buf )
{
  uint16 firmwareseqTemp;
  uint8 * airBuf;
  // send back error code, sequence number not match
  PTL0_InitTypeDef outGoing_ptl0error;
  uint8 expectingSeq[2] = {0};
  
  if(ptl0_queryStat() == PTL0_STA_IDLE)
  {
    if(ptl0_firmwareUpgrade)
    {
      // sequence match?
      firmwareseqTemp = ((uint16)*(ptl0_buf->datapointer) << 8) 
        + *(ptl0_buf->datapointer + 1);
      
      if( firmwareseqTemp == ptl0_firmwareSeqNum )
      {
        if (ptl0_firmwareSeqNum == ptl0_lastfirmwareSeqNum)
        {
          // update status, receive local cmd frame
          ptl0_updateStat(PTL0_STA_FIRM_LAST_REC);
          
          // Assemble message send over the air
          // get buffer
          airBuf = osal_mem_alloc(ptl0_buf->length + 1);
          if(airBuf == NULL)
          {
            // not able to allocate memory
            ptl0_sendError(PTL0_ERROR_NotOTA);
            return false;
          }
          // get the last sequence indicator
          *(airBuf) = 0xFF;
          *(airBuf + 1) = 0xFF;
          // get the correct firmware length
          *(airBuf + 2) = (ptl0_buf->length - 2); 
          if ((ptl0_buf->length - 2))
            // if this package is not empty
            osal_memcpy(airBuf + 3, (ptl0_buf->datapointer + 2), (ptl0_buf->length - 2));
          
          // prepare information send on the air
          HVACQueen_DstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
          HVACQueen_DstAddr.addr.shortAddr = hvacFirmwareShortAddr;
          // Take the first endpoint, Can be changed to search through endpoints
          HVACQueen_DstAddr.endPoint = HVACQUEEN_ENDPOINT;
          
          // send data on the air
          if ( AF_DataRequest( &HVACQueen_DstAddr, &HVACQueen_epDesc,
                               HVACQUEEN_FW_CLUSTERID,
                               (byte)(ptl0_buf->length + 1),
                               (byte *)airBuf,
                               &HVACQueen_TransID,
                               (AF_EN_SECURITY), AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
          {
            // success send over the air
            // do nothing now..
            // wait Zigbee ACK, then send PTL0 ACK
            
            // Flash LED to indicate network active
            HalLedBlink (HAL_LED_3, 8, 30, 125);
          }
          else
          {
            // Error occurred in request to send.
            // Send error
            ptl0_sendError(PTL0_ERROR_NotOTA);
            
            // update status
            ptl0_updateStat(PTL0_STA_IDLE);
          }
          
          // release memory if its not empty
          if (airBuf != NULL)
            osal_mem_free(airBuf);
          
          return true;
        }
        else
        {
          // send back error code, total sequence number not match  
          // leave for other end, impossible to happen
        }
      }
      else
      {       
        expectingSeq[0] = (uint8)((ptl0_firmwareSeqNum & 0xFF00) >> 8);
        expectingSeq[1] = (uint8)(ptl0_firmwareSeqNum & 0x00FF);
          
        // configure error code
        outGoing_ptl0error.CMD1 = PTL0_ERROR;
        outGoing_ptl0error.CMD2 = PTL0_ERROR_SEQMISMATCH;
        outGoing_ptl0error.length = 2;
        outGoing_ptl0error.SOF = PTL0_SOF;
        outGoing_ptl0error.version = PTL0_FRAMEVER;
        outGoing_ptl0error.datapointer = expectingSeq;
        
        // send error information to STM32
        ptl0_sendMsg(outGoing_ptl0error);
        
        // update status
        ptl0_updateStat(PTL0_STA_IDLE);
      }
    }
    else
    {
      // send back error code, need init first
      // Not likely to happen. 
      ptl0_sendError(PTL0_ERROR_DEVBUSY);
      
      // update status
      ptl0_updateStat(PTL0_STA_IDLE);
    }
  }
 
  return false;
}


/*********************************************************************
 * @fn      hvacHandleZDODeviceAnnounce
 *
 * @brief   Handles all network announce. Once a new device join 
 *          network, a announcement will send to network. 
 *
 * @param   MSGpkt - incoming message 
 *
 * @return  true or false
 */
uint8 hvacHandleZDODeviceAnnounce(zdoIncomingMsg_t * MSGpkt)
{
  ZDO_DeviceAnnce_t Annce;
  PTL0_InitTypeDef outGoingPTL0Msg;
  uint8 datapayload[PTL0_NWK_STATUS_RP_NEWDEV_DATALENGTH];
  
  // Parse message
  ZDO_ParseDeviceAnnce( MSGpkt, &Annce );
 
  // assemble message.
  // datapointer will handle by ptl0_pushEvent function. 
  outGoingPTL0Msg.CMD1 = PTL0_NWK_STATUS_RP;
  outGoingPTL0Msg.CMD2 = PTL0_NWK_STATUS_RP_NEWDEV;
  outGoingPTL0Msg.length = PTL0_NWK_STATUS_RP_NEWDEV_DATALENGTH;
  outGoingPTL0Msg.SOF = PTL0_SOF;
  outGoingPTL0Msg.version = PTL0_FRAMEVER;
  
  // Get mac address
  osal_memcpy(datapayload, Annce.extAddr, Z_EXTADDR_LEN);
  // Get device type
  *(datapayload + Z_EXTADDR_LEN) = Annce.capabilities;
  // set data pointer
  outGoingPTL0Msg.datapointer = datapayload;

  // update network address buffer according to mac address white table

  // The mac address allowed is already saved in the white list, find the 
  // correct place and save the network address
  hvac_addMacNwkLookupEntry(&Annce.nwkAddr,Annce.extAddr);
  // write the table once finish 
  osal_nv_write(HVAC_NETWORKADDR_ITEM, 0
                 , (hvacMACADDRNum * 2), hvac_nwkAddr);
  
  // upload message
  return (ptl0_uploadMsg(outGoingPTL0Msg,outGoingPTL0Msg.datapointer
                 ,PTL0_NWK_STATUS_RP_NEWDEV_DATALENGTH
                   ,HVACQueen_TaskID));
}

/*********************************************************************
 * @fn      HVACQueen_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void HVACQueen_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case HVACQUEEN_TRS_SEND_CLUSTERID:    
    // process message, upload to STM32
      // flash LED to indicate active network
      HalLedBlink (HAL_LED_3, 8, 30, 125);
      hvacMSGHandleTRS_SendMsg(pkt);     
      break;
      
    case HVACQUEEN_ALIVE_CLUSTERID:
    // process heart beat message. Upload to STM32
      hvacHandleALIVE_Msg(pkt);
      break;
    
    case HVACQUEEN_FW_CLUSTERID:
    // receiving firmware on the air message
      // flash LED to indicate active network
      HalLedBlink (HAL_LED_3, 8, 30, 125);    
      // Queen will never receive this message
      break;
      
    case HVACQUEEN_CONTROL:
    // receiving control message, process
      // flash LED to indicate active network
      HalLedBlink (HAL_LED_3, 8, 30, 125);
      // call a function to process control function
      hvacMSGHandleCMD(pkt);
      break;
      
    case HVACQUEEN_REMOTEACK:
    // Receiving remote ACK
    // ptl0_queryStat()>PTL0_STA_FIRMWARE including receiving normal package
    // and also firmware receiving
      // flash LED to indicate active network
      HalLedBlink (HAL_LED_3, 8, 30, 125);
      if (ptl0_queryStat() > PTL0_STA_FIRMWARE_REC)
      {
        // ptl0 just receiving something before and need responce to STM32
        // No receiving REMOTE ACK, 2530 should send ACK and release line
        
        // count sequence number
        ptl0_firmwareSeqNum = (uint16)((pkt->cmd.Data[0] << 8) + pkt->cmd.Data[1]); 
        
        if (ptl0_queryStat() == PTL0_STA_FIRM_LAST_REC)
        {
          // firmware transmit finish, reset flag
          ptl0_firmwareUpgrade = false;
          ptl0_firmwareSeqNum = 0;    
        }
               
        // send ACK
        ptl0_sendACK();
        
        // send ACK, communication complete, back to idle
        ptl0_updateStat(PTL0_STA_IDLE);        
      }
      break;
    
    case HVACQUEEN_FIRMWAREACK:
    // Receiving firmware data ack, send ACK to STM32
      // flash LED to indicate active network
      HalLedBlink (HAL_LED_3, 8, 30, 125);
      // if sending firmware
      // prepare the sequence expecting
      if ((ptl0_queryStat() == PTL0_STA_FIRM_REC) || (ptl0_queryStat() == PTL0_STA_FIRM_INIT_REC))
      {
        // count sequence number
        ptl0_firmwareSeqNum = (uint16)((pkt->cmd.Data[0] << 8) + pkt->cmd.Data[1]); 
        
        // send ACK
        ptl0_sendACK();
        
        // send ACK, communication complete, back to idle
        // ptl0_updateStat(PTL0_STA_IDLE);
      }
      
      // send ACK, communication complete, back to idle
      ptl0_updateStat(PTL0_STA_IDLE);
      break;
      
#if defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
  }
}


/*********************************************************************
 * @fn      hvacMSGHandleTRS_SendMsg
 *
 * @brief   Handle all incoming af message. Transfer into PTL0 message
 *
 * @param   none
 *
 * @return  none
 */
static uint8 hvacMSGHandleTRS_SendMsg( afIncomingMSGPacket_t *pkt )
{
  PTL0_InitTypeDef incomingPTL0Msg;
  uint8 *tempbuf;
  
  // get data from the in coming message
  incomingPTL0Msg.length = pkt->cmd.Data[0] + Z_EXTADDR_LEN;
  
  // get data payload
  tempbuf = osal_mem_alloc(pkt->cmd.Data[0] + Z_EXTADDR_LEN);
  if(tempbuf == NULL)
    return false;
  
  // write mac address
  if(hvac_LookupMac(&pkt->srcAddr.addr.shortAddr, tempbuf) == false)
    // no such network address available
    return false;
  
  // write data payload
  osal_memcpy(tempbuf + Z_EXTADDR_LEN, (pkt->cmd.Data + 1),
              pkt->cmd.Data[0]);
  // load data pointer
  incomingPTL0Msg.datapointer = tempbuf;
  
  // reload data to a ptl0 structure
  incomingPTL0Msg.CMD1 = PTL0_TRS_TRANS;
  incomingPTL0Msg.CMD2 = PTL0_EMPTYCMD;
  incomingPTL0Msg.SOF = PTL0_SOF;
  incomingPTL0Msg.version = PTL0_FRAMEVER;
  
  // save the address information
  lastMsgAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
  lastMsgAddr.addrMode = pkt->srcAddr.addrMode;
    
  // Must use event to upload. Use osal_mem_free function at end
  // of this function. Those data must copy to a static arrary.
  
  // push to event buffer
  ptl0_uploadMsg(incomingPTL0Msg,incomingPTL0Msg.datapointer
                 ,incomingPTL0Msg.length
                   ,HVACQueen_TaskID);
  
  // release memory
  osal_mem_free(tempbuf);
  
  return true;
}


/*********************************************************************
 * @fn      hvacHandleALIVE_Msg
 *
 * @brief   Handle incoming ALIVE Msg. Upload to STM32 with PTL0 frame
 *
 * @param   none
 *
 * @return  none
 */
static uint8 hvacHandleALIVE_Msg( afIncomingMSGPacket_t *pkt )
{
  PTL0_InitTypeDef incomingPTL0Msg;
  uint8 tempbuf[Z_EXTADDR_LEN];
  
  // get data from the in coming message
  incomingPTL0Msg.length = Z_EXTADDR_LEN;
  
  // write mac address
  if(hvac_LookupMac(&pkt->srcAddr.addr.shortAddr, tempbuf) == false)
    // no such network address 
    return false;
  
  // write data payload

  // load data pointer
  incomingPTL0Msg.datapointer = tempbuf;
  
  // reload data to a ptl0 structure
  incomingPTL0Msg.CMD1 = PTL0_NWK_STATUS_RP;
  incomingPTL0Msg.CMD2 = PTL0_NWK_STATUS_RP_HB;
  incomingPTL0Msg.SOF = PTL0_SOF;
  incomingPTL0Msg.version = PTL0_FRAMEVER;
  
  // save the address information
  lastMsgAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
  lastMsgAddr.addrMode = pkt->srcAddr.addrMode; 
  
  // upload message 
  return(ptl0_uploadMsg(incomingPTL0Msg, incomingPTL0Msg.datapointer
                 , incomingPTL0Msg.length ,HVACQueen_TaskID));
}


/*********************************************************************
 * @fn      hvacMSGHandleCMD
 *
 * @brief   Handle COMMAND message. CC2530 doesn't need to know the 
 *          detail of the message. Just unpack the message and send 
 *          to STM32.
 *
 * @param   none
 *
 * @return  none
 */
uint8 hvacMSGHandleCMD(afIncomingMSGPacket_t *pkt)
{
  PTL0_InitTypeDef incomingPTL0Msg;
  uint8 DLen;
  uint8 *ptr = NULL;
  
  // unpack MSG
  DLen = pkt->cmd.Data[0];
  incomingPTL0Msg.length = DLen;
  incomingPTL0Msg.CMD1 = pkt->cmd.Data[1];
  incomingPTL0Msg.CMD2 = pkt->cmd.Data[2];
  
  // if its an 0x14-device information report command
  if ((incomingPTL0Msg.CMD1 == PTL0_LOC_STATUS_RP) 
      && (incomingPTL0Msg.CMD2 == PTL0_LOC_STATUS_DEV_INFO))
  {
    // save the information
    
    // update network table
    osal_nv_write(HVAC_NETWORKADDR_ITEM, 2 * (hvac_addMacNwkLookupEntry(&(pkt->srcAddr.addr.shortAddr),&(pkt->cmd.Data[3])) - 1)
               , 2, &(pkt->srcAddr.addr.shortAddr));
  }
  
  // if there is any data?
  if(DLen)
  {
    // copy data to temperary buffer
    ptr = osal_mem_alloc(DLen);
    // valid?
    if(ptr == NULL)
      return false;
    
    // copy data
    memcpy(ptr, &(pkt->cmd.Data[3]), DLen);    
  }
  
  // get the PTL0 frame
  incomingPTL0Msg.SOF = PTL0_SOF;
  incomingPTL0Msg.version = PTL0_FRAMEVER;
  if(ptr != NULL)
    incomingPTL0Msg.datapointer = ptr;
  
  // get into event no data?
  ptl0_uploadMsg(incomingPTL0Msg, ptr, DLen, HVACQueen_TaskID);
  
  // release memory if there's any
  if(ptr != NULL)
    osal_mem_free(ptr);
  
  return true;
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
 * @fn      hvac_STM32ResetCtrl
 *
 * @brief   trigger a STM32 reset through the STM32 reset pin. The 
 *          relative I/O on CC2530 will be set as active, and 1s 
 *          later reset in event.
 *          Use 2.0 as reset control.
 *
 * @param   CMD - command, HVAC_STM32_RESET/HVAC_STM32_RECOVER
 *
 * @return  none
 */
static void hvac_STM32ResetCtrl( uint8 CMD )
{
  // Reset STM32, pull down to reset
  if(CMD == HVAC_STM32_RESET)
  {
    P2 &= 0xFE;
    // Pull down for 2 seconds to reset 
    hvacSTM32ResetSta = HVAC_STM32_RESET_DELAY;
  }
  else
    P2 |= 0X01; 
}


/*********************************************************************
 * @fn      hvac_addMacNwkLookupEntry
 *
 * @brief   add an mac - network address entry according to the 
 *          announcement message received.
 *
 * @param   * nwkAddr - the network address
 *          * macAddr - corresponding mac address
 *
 * @return  success|fail true|false
 */
static uint8 hvac_addMacNwkLookupEntry( uint16 * nwkAddr, uint8 * macAddr )
{
  uint8 i;
  uint8 resultTemp = false;
#if !defined HVAC_WHITELIST   
  uint8 compTemp[Z_EXTADDR_LEN] = {0};
#endif 
  
  for(i = 0; i < HVAC_MAX_DRONE_NUM; i++)
  { 
    if(AddrMgrExtAddrEqual(&hvac_whiteList[i][0], macAddr))
    {
      if(resultTemp)
      {
        // if already match
        // clear relative entry
        memset(&hvac_whiteList[i][0], 0x00, Z_EXTADDR_LEN);
        hvac_nwkAddr[i] = 0;
        
        // continue loop
        continue;
      }
      
      // mac address match a table entry, 
      // save the info in corresponding
      // network address list
      hvac_nwkAddr[i] = *nwkAddr;

      // set compare result
      resultTemp = true;
    }
    else if(hvac_nwkAddr[i] == *nwkAddr)
    {
      // if the mac address not match, but the network address match.
      // clear the table 
      hvac_nwkAddr[i] = 0;
    }
  }
  
#if !defined HVAC_WHITELIST      
  if (!resultTemp)
  {
    // if the entry is already created, then its been edit before
    // if not, then the entry should be create here
      
    for(i = 0; i < HVAC_MAX_DRONE_NUM; i++)
    {
      if(AddrMgrExtAddrEqual(&hvac_whiteList[i][0], compTemp))
      {
        // get the first zero space
        // network address list
        hvac_nwkAddr[i] = *nwkAddr;
        
        // also update the mac entry
        memcpy(&hvac_whiteList[i][0], macAddr, Z_EXTADDR_LEN);
        
        // break the loop
        break;
      }
    }   
  }
#endif  
  
  return i;
}

/*********************************************************************
 * @fn      hvac_LookupNwk
 *
 * @brief   Lookup Zigbee network address according to the mac address
 *          provided.
 *
 * @param   uint16 * nwkAddr - network address 
 *          uint8 * macAddr - mac address use to query
 *
 * @return  success|fail true|false
 */
static uint8 hvac_LookupNwk( uint16 * nwkAddr, uint8 * macAddr )
{
  uint8 i;
  
  for(i = 0; i < HVAC_MAX_DRONE_NUM; i++)
  {
    if(AddrMgrExtAddrEqual(&hvac_whiteList[i][0], macAddr))
    {
      // mac address matched, get the nwkAddr back
      *nwkAddr = hvac_nwkAddr[i];
      
      return true;
    }
  }
  
  return false;
}


/*********************************************************************
 * @fn      hvac_LookupMac
 *
 * @brief   Lookup Zigbee mac address according to the network address
 *          provided.
 *
 * @param   uint16 * nwkAddr - network address use to query
 *          uint8 * macAddr - mac address 
 *
 * @return  success|fail true|false
 */
static uint8 hvac_LookupMac( uint16 * nwkAddr, uint8 * macAddr )
{
  uint8 i;
  
  for(i = 0; i < HVAC_MAX_DRONE_NUM; i++)
  {
    if(hvac_nwkAddr[i] == *nwkAddr)
    {
      // mac address matched, get the nwkAddr back   
      memcpy(macAddr, &hvac_whiteList[i][0], Z_EXTADDR_LEN);
        
      return true;
    }
  }
  
  return false; 
}

#ifdef HVAC_SBL
/*********************************************************************
 * @fn      appForceBoot
 *
 * @brief   Force device goes to SBL code
 *
 * @param   none
 *
 * @return  none
 */
static void appForceBoot(void)
{
  asm("LJMP 0x0000\n");  // Immediate jump to SBL-code.
  HAL_SYSTEM_RESET();
}
#endif

#if defined( IAR_ARMCM3_LM )
/*********************************************************************
 * @fn      HVACQueen_ProcessRtosMessage
 *
 * @brief   Receive message from RTOS queue, send response back.
 *
 * @param   none
 *
 * @return  none
 */
static void HVACQueen_ProcessRtosMessage( void )
{
  osalQueue_t inMsg;

  if ( osal_queue_receive( OsalQueue, &inMsg, 0 ) == pdPASS )
  {
    uint8 cmndId = inMsg.cmnd;
    uint32 counter = osal_build_uint32( inMsg.cbuf, 4 );

    switch ( cmndId )
    {
      case CMD_INCR:
        counter += 1;  /* Increment the incoming counter */
                       /* Intentionally fall through next case */

      case CMD_ECHO:
      {
        userQueue_t outMsg;

        outMsg.resp = RSP_CODE | cmndId;  /* Response ID */
        osal_buffer_uint32( outMsg.rbuf, counter );    /* Increment counter */
        osal_queue_send( UserQueue1, &outMsg, 0 );  /* Send back to UserTask */
        break;
      }

      default:
        break;  /* Ignore unknown command */
    }
  }
}
#endif

/*********************************************************************
 */