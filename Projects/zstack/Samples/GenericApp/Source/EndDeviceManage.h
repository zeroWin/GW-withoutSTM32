/**************************************************************************************************
  Filename:       EndDeviceManage.h
  Revised:        $Date: 2016-03-30 15:34:16 +0800 (Wen, 30 Mar 2016) $
  Revision:       $Revision: 1 $

  Description:    This file contains the interface of Network management.


  Copyright 2016 Bupt. All rights reserved.

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
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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
  contact kylinnevercry@gmail.com. 
**************************************************************************************************/

#ifndef END_DEVICE_MANAGE_H
#define END_DEVICE_MANAGE_H

#ifdef __cplusplus
extern "C"
{
#endif


/**************************************************************************************************
 * INCLUDES
 */
#include "Onboard.h"
#include "OSAL.h"


/**************************************************************************************************
 * MACROS
 */
  
/**************************************************************************************************
 * CONSTANTS
 */

/**************************************************************************************************
 * TYPEDEFS
 */
typedef struct
{
  void    *next;
  uint16  device_id;
  uint16  shortAddress;
} endDevice_info_t;

/**************************************************************************************************
 * FUNCTIONS - API
 */

/*** Network Management  ***/

  /*
   * Init the list head points
   */
  extern void endevice_info_listInit( void );

  /*
   * Add the connect device info on the List when the device connect to the GW
   */
  extern uint8 endDevice_info_add( uint16 device_id , uint16 shortAddress);
  
  /*
   * Delete the connect device info on the List when the device disconnect to the GW
   */
  extern void endDevice_info_delete( uint16 device_id );
  
  /*
   * Find shortAddress
   */
  extern uint16 endDevice_info_find( uint16 device_id );


#ifdef __cplusplus
}
#endif  
#endif





