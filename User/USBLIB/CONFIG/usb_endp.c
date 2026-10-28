/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_endp.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/08/08
 * Description        : Endpoint routines
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/ 
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "usb_prop.h"
#include "FreeRTOS.h"
#include "queue.h"

volatile uint8_t USBD_Endp3_Busy = 0;

extern QueueHandle_t cdc_rx_queue;

void EP1_IN_Callback (void)
{ 
}

void EP2_OUT_Callback (void)
{ 
    uint32_t len;
    uint8_t buffer[64];
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    len = GetEPRxCount( EP2_OUT & 0x7F );
    PMAToUserBufferCopy( buffer, GetEPRxAddr( EP2_OUT & 0x7F ), len );
    
    if (cdc_rx_queue != NULL)
    {
        for(uint32_t i=0; i<len; i++)
        {
            xQueueSendFromISR(cdc_rx_queue, &buffer[i], &xHigherPriorityTaskWoken);
        }
    }
    
    SetEPRxValid( ENDP2 );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void EP3_IN_Callback (void)
{ 
	USBD_Endp3_Busy = 0;
}

uint8_t USBD_ENDPx_DataUp( uint8_t endp, uint8_t *pbuf, uint16_t len )
{
	if( endp == ENDP3 )
	{
		if (USBD_Endp3_Busy)
		{
			return USB_ERROR;
		}
		USB_SIL_Write( EP3_IN, pbuf, len );
		USBD_Endp3_Busy = 1;
		SetEPTxStatus( ENDP3, EP_TX_VALID );
	}
	else
	{
		return USB_ERROR;
	}
	return USB_SUCCESS;
}
