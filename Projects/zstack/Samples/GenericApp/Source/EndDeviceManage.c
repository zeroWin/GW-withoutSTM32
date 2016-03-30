/**************************************************************************************************
  Filename:       EndDeviceManage.c
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

/*********************************************************************
 * INCLUDES
 */

#include "EndDeviceManage.h"
#include "OSAL.h"
#include "ZComDef.h"

#include "hal_mcu.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// endDevice Info list head point Definitions
endDevice_info_t *endDeviceHead;

/**************************************************************************************************
 * FUNCTIONS - Local
 */

/**************************************************************************************************
 * FUNCTIONS - API
 */

/*********************************************************************
 * @fn     endevice_info_listInit
 *
 * @brief  Init the list head point.
 *
 * @param   
 *
 * @return  none
 */
void endDevice_info_listInit( void )
{
  endDeviceHead = NULL;
}

/*********************************************************************
 * @fn      endDevice_info_add
 *
 * @brief   Add a device info list
 *
 * @param   
 *
 * @return  status
 */
uint8 endDevice_info_add( uint16 device_id , uint16 shortAddress)
{
  endDevice_info_t *newDevice;
  endDevice_info_t *srchDevice;
  
  
  // Allocate the space
  newDevice = osal_mem_alloc(sizeof(endDevice_info_t));
  
  if(newDevice)
  {
    newDevice->next = (void *)NULL;
    newDevice->device_id = device_id;
    newDevice->shortAddress = shortAddress;
    
    // Dose the device info list already exist
    if ( endDeviceHead == NULL )
    {
      endDeviceHead = newDevice;
    }
    else
    {
      // Add it to the end of the device info list
      srchDevice = endDeviceHead;
      
      // Stop at the last record
      while( srchDevice->next )
        srchDevice = srchDevice->next;
      
      // Add to the list
      srchDevice->next = newDevice;
    }
    return ( ZSuccess );
  }
  else
    return ( ZFailure );
}

/*********************************************************************
 * @fn      endDevice_info_delete
 *
 * @brief   delete a device info from the device info list
 *
 * @param   device_id
 *
 * @return  none
 */
void endDevice_info_delete( uint16 device_id )
{
  endDevice_info_t *srchDevice;
  endDevice_info_t *prevDevice;

  if( endDeviceHead != NULL )
  {
    srchDevice = endDeviceHead;
    prevDevice = (void *)NULL;
    
    // Look for the device id
    while( srchDevice )
    {
      endDevice_info_t *freeDevice = NULL;
      
      if( srchDevice->device_id == device_id )//find it
      {
        // Take out of list
        if( prevDevice == NULL )
          endDeviceHead = srchDevice->next;
        else
          prevDevice->next = srchDevice->next;
      
        // Setup to free memory
        freeDevice = srchDevice;
        
        // Next
        srchDevice = srchDevice->next;
      }
      else
      {
        // Get next
        prevDevice = srchDevice;
        srchDevice = srchDevice->next;
      }
      
      if( freeDevice )
      {
        osal_mem_free( freeDevice );
      }
    }
  }
}


/*********************************************************************
 * @fn      endDevice_info_find
 *
 * @brief   Find a device address on device info list
 *
 * @param   device_id
 *
 * @return  shortAddress(find) or false(not find)
 */
uint16 endDevice_info_find( uint16 device_id )
{
  endDevice_info_t *srchDevice;
  
  // Head of the device list
  srchDevice = endDeviceHead;
  
  // Stop when found or at the end
  while( srchDevice )
  {
    if( srchDevice->device_id == device_id )
      break;
    
    // Not this one ,check another
    srchDevice = srchDevice->next;
  }
  if( srchDevice ) //find it
    return srchDevice->shortAddress;
  else
    return FALSE;
}







