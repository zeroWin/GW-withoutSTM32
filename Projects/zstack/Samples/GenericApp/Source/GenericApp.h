/**************************************************************************************************
  Filename:       GenericApp.h
  Revised:        $Date: 2007-10-27 17:22:23 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15795 $

  Description:    This file contains the Generic Application definitions.


  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
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
**************************************************************************************************/

#ifndef GENERICAPP_H
#define GENERICAPP_H

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
#define GENERICAPP_ENDPOINT           10

#define GENERICAPP_PROFID             0x0F04
#define GENERICAPP_DEVICEID           0x00FF //GW device ID
#define GENERICAPP_DEVICE_VERSION     0
#define GENERICAPP_FLAGS              0

#define GENERICAPP_IN_CLUSTERS        9
#define GENERICAPP_OUT_CLUSTERS       4  

#define GENERICAPP_CLUSTERID                  0x0001   // I/O
#define GENERICAPP_CLUSTERID_START            0x0010   // O
#define GENERICAPP_CLUSTERID_STOP             0x0011   // O
  
#define GENERICAPP_CLUSTERID_SYNC             0x0020   // O 0x002x 是同步相关指令
#define GENERICAPP_CLUSTERID_ECG_SYNC_OVER    0x0021   // I
#define GENERICAPP_CLUSTERID_TEMPR_SYNC_OVER  0x0022   // I
#define GENERICAPP_CLUSTERID_SPO2_SYNC_OVER   0x0023   // I
#define GENERICAPP_CLUSTERID_BP_SYNC_OVER     0x0024   // I 
  
#define GENERICAPP_CLUSTERID_ECG_RESULT       0x0030   // I 0x003x 是测量结果相关指令
#define GENERICAPP_CLUSTERID_TEMPR_RESULT     0x0031   // I
#define GENERICAPP_CLUSTERID_SPO2_RESULT      0x0032   // I
#define GENERICAPP_CLUSTERID_BP_RESULT        0x0033   // I 
  
// Send Message Timeout
#define GENERICAPP_SEND_MSG_TIMEOUT   5000     // Every 5 seconds

// Application Events (OSAL) - These are bit weighted definitions.
// 除了Ox8000-SYS_EVENT_MSG均可用，一共可以注册15个用户事件
//#define GENERICAPP_SEND_MSG_EVT       0x0001
#define SIMPLE_DESC_QUERY_EVT         0x0002
  
// Measure Device IDs
#define M_DEVICEID_ECG                0x0001  // ECG device ID
#define M_DEVICEID_TEMPR              0x0002  // TEMPR device ID
#define M_DEVICEID_SPO2               0x0003  // SpO2 deivce ID
#define M_DEVICEID_BP                 0x0004  // BP device ID
  
// SENSORTYPE ID
#define SENSORTYPE_THERMOMETE               0xA1    // 体温
#define SENSORTYPE_ELECTROCARDIOGRAMMETER   0xA2    // 心电
#define SENSORTYPE_BLOODPRESSUREMETER       0xA3    // 血压
#define SENSORTYPE_BLOODOXYGENMETER         0xA6    // 血氧
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void GenericApp_Init( byte task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GENERICAPP_H */
