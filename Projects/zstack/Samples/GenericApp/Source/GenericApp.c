/******************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2010-12-21 10:27:34 -0800 (Tue, 21 Dec 2010) $
  Revision:       $Revision: 24670 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
  This application is for GW-withoutSTM32
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "GenericApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

/* USER */
#include "Serial.h"
#include "EndDeviceManage.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
uint8 buff1[100] = {
0x2E,0x1F,0xAC,0x0,0x20,0x0,0x0,0xA0,0x34,0x12,0x0,0x0,0x4D,0x4D,0x47,0x57,0x30,0x30,0x31,0x0,0x32,0x2E,0x30,0x2E,0x35,0x0,0x82,0x0,0x0,0x4,0x0,0x0,0x0,0x0,0x0,0x0,0x14,0xAE,0x77,0x40,0xA,0xD7,0x63,0x3F,0x40,0x0,0x0,0x0,0x0,0x4,0x0,0x0,0x38,0x39,0x2D,0x31,0x46,0x2D,0x33,0x39,0x2D,0x44,0x45,0x2D,0x33,0x44,0x2D,0x34,0x38,0x0,0x37,0x39,0x32,0x2E,0x31,0x0,0x0,0x0,0x0,0xFF,0x34,0x12,0x0,0x0,0x1,0x0,0x6,0x74,0xF7,0x12,0x4,0xE,0x1,0x0,0x0,0x0,0x1,0x0,0x0,0x0
};
uint8 buff2[78] = {
0xA2,0x0,0x0,0x0,0xC2,0xCC,0x17,0x7,0x0,0x4B,0x12,0x0,0x33,0x2E,0x30,0x38,0x2E,0x30,0x39,0x0,0x52,0xB8,0x86,0x40,0x66,0x66,0x66,0x3F,0x40,0x0,0x0,0x0,0x0,0x1,0x0,0x0,0x37,0x39,0x2D,0x32,0x46,0x2D,0x35,0x37,0x2D,0x46,0x46,0x2D,0x33,0x38,0x2D,0x34,0x45,0x0,0xC5,0x13,0x31,0x2E,0x30,0x0,0x0,0x4,0x0,0x0,0x14,0xAE,0x77,0x40,0xA,0xD7,0x63,0x3F,0x40,0x0,0x0,0x0,0x3D,0x4B
};
uint8 buff3[144] = {
0x2E,0x1F,
0x8A,0x0,
0x17,0x0,0x0,0xA0,
0x3,0x0,0x0,0x0,
0x1,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,
0xA2,0x0,0x0,0x0,
0xC2,0xCC,0x17,0x7,0x0,0x4B,0x12,0x0,
0xB,0x0,0x0,0x0,
0x1,0x0,0x0,0x0,
0x1,0x0,0x0,0x0,
0x54,0x0,0x0,0x0,
0x75,0x8,
0x19,0x0,
0x2D,0x0,
0x20,0xF2,0x97,0x48,
0x0,0x40,0x0,0x0,
0x6F,0x12,0x83,0x3B,
0x1,0x0,0x0,0x0,
0x22,0x0,0x0,0x0,

0x4B,0x0,0x4C,0x0,0x4D,0x0,0x4E,0x0,
0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,
0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,
0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,0x3,0x0,

0x3D,0x4B 
};

uint8 buff4[80] = {
0x2E,0x1F,0x4A,0x00,0x17,0x00,0x00,0xA0,0x03,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA1,0x00,0x00,0x00,0x27,0xCB,0x17,0x07,0x00,0x4B,0x12,0x00,0x0B,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x14,0x00,0x00,0x00,0xD0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3D,0x4B
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_InClusterList[GENERICAPP_IN_CLUSTERS] =
{
  GENERICAPP_CLUSTERID,
  GENERICAPP_CLUSTERID_ECG_SYNC_OVER,
  GENERICAPP_CLUSTERID_ECG_RESULT,
  GENERICAPP_CLUSTERID_TEMPR_SYNC_OVER,
  GENERICAPP_CLUSTERID_TEMPR_RESULT,
  GENERICAPP_CLUSTERID_SPO2_SYNC_OVER,
  GENERICAPP_CLUSTERID_SPO2_RESULT
};

const cId_t GenericApp_OutClusterList[GENERICAPP_OUT_CLUSTERS] =
{
  GENERICAPP_CLUSTERID,
  GENERICAPP_CLUSTERID_START,
  GENERICAPP_CLUSTERID_STOP,
  GENERICAPP_CLUSTERID_SYNC
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,                  //  int Endpoint;
  GENERICAPP_PROFID,                    //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,                  //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,            //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                     //  int   AppFlags:4;
  GENERICAPP_IN_CLUSTERS,               //  byte  AppNumInClusters;
  (cId_t *)GenericApp_InClusterList,    //  byte *pAppInClusterList;
  GENERICAPP_OUT_CLUSTERS,              //  byte  AppNumOutClusters;
  (cId_t *)GenericApp_OutClusterList    //  byte *pAppOutClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.
devStates_t GenericApp_NwkState;


byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;
static zAddrType_t simpleDescReqAddr;                // destination addresses for simple desc request

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void GenericApp_HandleKeys( byte shift, byte keys );
void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );

void GenericApp_ProcessUartData( OSALSerialData_t *inMsg );
void GenericApp_GetDeviceNWKAddress( uint8 *dataMsg );
void halMcuWaitUs(uint16 microSecs);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_Init
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
void GenericApp_Init( byte task_id )
{
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().
  
  // 单播shortAddress方式发送，短地址在Device annce处理部分添加
  GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  
  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );

  // Init the UART
  Serial_Init();
  
  // Register for serial events - This app will handle all serial events
  Serial_UartRegisterTaskID( GenericApp_TaskID );
  
  // Init Enddevice info list
  endDevice_info_listInit();
  
  // Update the display
#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Device_annce );
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Simple_Desc_rsp);
}

/*********************************************************************
 * @fn      GenericApp_ProcessEvent
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
UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events )
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
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          GenericApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          GenericApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (GenericApp_NwkState == DEV_ZB_COORD)
              || (GenericApp_NwkState == DEV_ROUTER)
              || (GenericApp_NwkState == DEV_END_DEVICE) )
          {
          }
          break;
         
         case CMD_SERIAL_UART_MSG:
           GenericApp_ProcessUartData((OSALSerialData_t *)MSGpkt);
           
          break;
          
          
        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  // event to get simple descriptor of the newly joined device
  if ( events & SIMPLE_DESC_QUERY_EVT)
  {
    ZDP_SimpleDescReq( &simpleDescReqAddr, simpleDescReqAddr.addr.shortAddr,
                      GENERICAPP_ENDPOINT, 0);

    return ( events ^ SIMPLE_DESC_QUERY_EVT );    
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
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
            GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            GenericApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
      
     case Device_annce:
       {
         //inMsg->asdu的前两个字节是连接上节点的短地址
         //从第三个字节开始的8个字节是连接上节点的IEEE地址
         //两个都是低位在前，高位在后，比如inMsg->asdu[0]=0x11,inMsg->asdu[1]=0x22
         //则连接节点的Short Address是0x2211
         //GenericApp_DstAddr.addr.shortAddr = (((uint16)inMsg->asdu[1]) << 8) | (uint16)(inMsg->asdu[0]);
         //可以直接调用现成的消息处理函数获取
         ZDO_DeviceAnnce_t devAnnce;
         ZDO_ParseDeviceAnnce( inMsg, &devAnnce );
         
         simpleDescReqAddr.addrMode = (afAddrMode_t)Addr16Bit;
         simpleDescReqAddr.addr.shortAddr=devAnnce.nwkAddr;
         
         // set simple descriptor query event
         osal_set_event(GenericApp_TaskID,SIMPLE_DESC_QUERY_EVT);
       }
       break;

     case Simple_Desc_rsp:
       {
         ZDO_SimpleDescRsp_t *pSimpleDescRsp;   // pointer to received simple desc response
         pSimpleDescRsp = (ZDO_SimpleDescRsp_t *)osal_mem_alloc( sizeof( ZDO_SimpleDescRsp_t ) );
         
         if(pSimpleDescRsp)
         {
           pSimpleDescRsp->simpleDesc.pAppInClusterList = NULL;
           pSimpleDescRsp->simpleDesc.pAppOutClusterList = NULL;

           ZDO_ParseSimpleDescRsp( inMsg, pSimpleDescRsp );
           
           switch(pSimpleDescRsp->simpleDesc.AppDeviceId)
           {
            case M_DEVICEID_ECG:// 这次连接到的设备是ECG
             // 将ECG的device_id和shortAddress存储在链表中
             if(endDevice_info_find(M_DEVICEID_ECG) == FALSE)// 不存在，添加信息
               endDevice_info_add( M_DEVICEID_ECG , pSimpleDescRsp->nwkAddr );
             else // 存在，更新地址
               endDevice_info_update( M_DEVICEID_ECG , pSimpleDescRsp->nwkAddr );
             break;
             
            case M_DEVICEID_TEMPR:// 这次连接到的设备是Tempr
             // 将TemprSN的device_id和shortAddress存储在链表中
             if(endDevice_info_find(M_DEVICEID_TEMPR) == FALSE)// 不存在，添加信息
               endDevice_info_add( M_DEVICEID_TEMPR , pSimpleDescRsp->nwkAddr );
             else // 存在，更新地址
               endDevice_info_update( M_DEVICEID_TEMPR , pSimpleDescRsp->nwkAddr );
             break;

            case M_DEVICEID_SPO2 :// 这次连接到的设备是ECG
             // 将ECG的device_id和shortAddress存储在链表中
             if(endDevice_info_find(M_DEVICEID_SPO2) == FALSE)// 不存在，添加信息
               endDevice_info_add( M_DEVICEID_SPO2 , pSimpleDescRsp->nwkAddr );
             else // 存在，更新地址
               endDevice_info_update( M_DEVICEID_SPO2 , pSimpleDescRsp->nwkAddr );
             break;
             
            default:break;
           }
           
           // free memory for InClusterList
           if (pSimpleDescRsp->simpleDesc.pAppInClusterList)
           {
             osal_mem_free(pSimpleDescRsp->simpleDesc.pAppInClusterList);
           }

           // free memory for OutClusterList
           if (pSimpleDescRsp->simpleDesc.pAppOutClusterList)
           {
             osal_mem_free(pSimpleDescRsp->simpleDesc.pAppOutClusterList);
           }
           
           osal_mem_free( pSimpleDescRsp );
         }
       }
       break;
  }
}

/*********************************************************************
 * @fn      GenericApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void GenericApp_HandleKeys( byte shift, byte keys )
{
  if(keys & HAL_KEY_SW_6)
  {
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint8 i,j;
  uint8 endNum;
  uint8 data = 0xFF;
  switch ( pkt->clusterId )
  {
    case GENERICAPP_CLUSTERID:
      // "the" message
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
    case GENERICAPP_CLUSTERID_ECG_RESULT:
      for( j = 0 ; j < 60 ; ++j)
          buff3[82+j] = pkt->cmd.Data[j];
      while( Serial_UartSendMsg( buff3 , 144 ) == 0);
      
      HalLedSet(HAL_LED_2,HAL_LED_MODE_TOGGLE);
      break;
      
    case GENERICAPP_CLUSTERID_TEMPR_RESULT:
      for ( i = 0 ; i < 4 ; ++i )
        buff4[74+i] = pkt->cmd.Data[i+8];      
      while( Serial_UartSendMsg( buff4 , 80 ) == 0);
   
      HalLedSet(HAL_LED_2,HAL_LED_MODE_TOGGLE);      
      break;
    
    case GENERICAPP_CLUSTERID_SPO2_RESULT:
      for( j = 0 ; j < 60 ; ++j)
          buff3[82+j] = pkt->cmd.Data[j];
      while( Serial_UartSendMsg( buff3 , 144 ) == 0);
      
      HalLedSet(HAL_LED_2,HAL_LED_MODE_TOGGLE);
      break;
      
    case GENERICAPP_CLUSTERID_ECG_SYNC_OVER:
    case GENERICAPP_CLUSTERID_SPO2_SYNC_OVER:
      for(i = 0 ; i < 104; ++i)
        while( Serial_UartSendMsg( &data , 1 ) == 0);      
      break;
      
    case GENERICAPP_CLUSTERID_TEMPR_SYNC_OVER:
      for(i = 0 ; i < 80; ++i)
        while( Serial_UartSendMsg( &data , 1 ) == 0);      
      break;

      
  }
}


/*********************************************************************
 * @fn      GenericApp_ProcessUartData()
 *
 * @brief   Process UART receive messages
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_ProcessUartData( OSALSerialData_t *inMsg )
{
  uint8 *pMsg;
  uint8 dataLen;
  
  pMsg = inMsg->msg;
  dataLen = inMsg->hdr.status;
    
  // 得到对应设备网络地址
  GenericApp_GetDeviceNWKAddress(pMsg);
  
  // 把消息发回去
  // Serial_UartSendMsg( pMsg , dataLen );
  if( pMsg[4] == 0x20 )
    return;
  else if( pMsg[4] == 0x13 ) // 对节点发送开始测量命令
  {
    // 向APP发送开始发送指令
    //Serial_UartSendMsg( buff1 , 100 );
    //while( Serial_UartSendMsg( buff2 , 78 ) == 0);
    AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                    GENERICAPP_CLUSTERID_START,
                    0,
                    NULL,
                    &GenericApp_TransID,
                    AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
  }
  else if( pMsg[4] == 0x14 ) // 对节点发送结束测量命令
  {
    AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                    GENERICAPP_CLUSTERID_STOP,
                    0,
                    NULL,
                    &GenericApp_TransID,
                    AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );    
  }
  else if( pMsg[4] == 0x15 ) // 对节点发送同步命令
  {
    AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                    GENERICAPP_CLUSTERID_SYNC,
                    0,
                    NULL,
                    &GenericApp_TransID,
                    AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );      
  }
  
    
}


/*********************************************************************
 * @fn      GenericApp_GetDeviceNWKAddress()
 *
 * @brief   Get Target device NWK address
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_GetDeviceNWKAddress( uint8 *dataMsg )
{
  switch(dataMsg[20])
  {
    case SENSORTYPE_ELECTROCARDIOGRAMMETER: // 心电相关控制命令
      GenericApp_DstAddr.addr.shortAddr = endDevice_info_find(M_DEVICEID_ECG);
      break;
      
    case SENSORTYPE_THERMOMETE: // 体温控制命令
      GenericApp_DstAddr.addr.shortAddr = endDevice_info_find(M_DEVICEID_TEMPR);
      break;
      
    case SENSORTYPE_BLOODOXYGENMETER: // 血氧控制命令
      GenericApp_DstAddr.addr.shortAddr = endDevice_info_find(M_DEVICEID_SPO2);
      break;
      
    default:break;
  }
}


/**************************************************************************************************
 * @fn      halMcuWaitUs
 *
 * @brief   wait for x us. @ 32MHz MCU clock it takes 32 "nop"s for 1 us delay.
 *
 * @param   x us. range[0-65536]
 *
 * @return  None
 **************************************************************************************************/
void halMcuWaitUs(uint16 microSecs)
{
  while(microSecs--)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}
/*********************************************************************
*********************************************************************/
