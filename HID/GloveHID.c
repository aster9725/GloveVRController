/*
             LUFA Library
     Copyright (C) Dean Camera, 2019.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2019  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/


#include "GloveHID.h"
#include "UART.h"
#include "base85.h"

volatile USB_GloveReport_Data_t gGloveReportData[2] = {0, };
volatile uint8_t* pRxProbe = rxUART, readCnt = 0;
uint8_t* volatile pReportProbe = (uint8_t*)&gGloveReportData;
uint8_t* volatile pReportData = (uint8_t*)&gGloveReportData;

int main(void)
{
	SetupHardware();

	GlobalInterruptEnable();
	
	for (;;)
	{
		HID_Task();
		USB_USBTask();
	}
}

void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	USB_Init();
	
	/* Serial Initialization */
	UART_INIT(115200);
}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Device_Connect(void)
{
	/* Indicate USB enumerating */
	UART_printString("Connected\r\n");
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs and stops the USB management task.
 */
void EVENT_USB_Device_Disconnect(void)
{
	/* Indicate USB not ready */
	UART_printString("Disconnected\n\r");
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host sets the current configuration
 *  of the USB device after enumeration, and configures the generic HID device endpoints.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	/* Setup HID Report Endpoints */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(GLOVE_IN_EPADDR, EP_TYPE_INTERRUPT, GLOVE_EPSIZE, 1);
	ConfigSuccess &= Endpoint_ConfigureEndpoint(GLOVE_OUT_EPADDR, EP_TYPE_INTERRUPT, GLOVE_EPSIZE, 1);

	/* Indicate endpoint configuration success or failure */
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
	/* Handle HID Class specific requests */
	switch (USB_ControlRequest.bRequest)
	{
		case HID_REQ_GetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				USB_GloveReport_Data_t GloveReportData;
				if(GetNextReport(&GloveReportData))
				{
					Endpoint_ClearSETUP();

					/* Write the report data to the control endpoint */
					Endpoint_Write_Control_Stream_LE(&GloveReportData, sizeof(USB_GloveReport_Data_t));
					Endpoint_ClearOUT();
				}
			}
			break;
		case HID_REQ_SetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				USB_GloveReport_Data_t GloveReportData;

				Endpoint_ClearSETUP();

				/* Read the report data from the control endpoint */
				Endpoint_Read_Control_Stream_LE(&GloveReportData, sizeof(USB_GloveReport_Data_t));
				Endpoint_ClearIN();

//				ProcessGenericHIDReport(&GloveReportData);	// Do something for input data
			}

			break;
	}
}

void ProcessGenericHIDReport(uint8_t* DataArray)
{
	/*
		This is where you need to process reports sent from the host to the device. This
		function is called each time the host has sent a new report. DataArray is an array
		holding the report sent from the host.
	*/

}

bool GetNextReport(USB_GloveReport_Data_t* const ReportData)
{
	bool ret = false;

	if(flagReportData & FRD_READY)
	{
		memcpy(ReportData, pReportData, 46);
		
		flagReportData &= ~FRD_READY;
		flagReportData |= FRD_SEND;
		ret = true;
	}

	return ret;
}

void HID_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;
	  
	Endpoint_SelectEndpoint(GLOVE_OUT_EPADDR);

	/* Check to see if a packet has been sent from the host */
	if (Endpoint_IsOUTReceived())
	{
		/* Check to see if the packet contains data */
		if (Endpoint_IsReadWriteAllowed())
		{
			/* Create a temporary buffer to hold the read in report from the host */
			USB_GloveReport_Data_t GloveReportData_temp;

			/* Read Generic Report Data */
			Endpoint_Read_Stream_LE(&GloveReportData_temp, sizeof(USB_GloveReport_Data_t), NULL);
			UART_printString("[ReportData Read] \n\r");

			/* Process Generic Report Data */
//			ProcessGenericHIDReport(GloveReportData);
		}

		/* Finalize the stream transfer to send the last packet */
		Endpoint_ClearOUT();
	}

	Endpoint_SelectEndpoint(GLOVE_IN_EPADDR);

	/* Check to see if the host is ready to accept another packet */
	if (Endpoint_IsINReady())
	{
		/* Create a temporary buffer to hold the report to send to the host */
		USB_GloveReport_Data_t GloveReportData;

		/* Create Generic Report Data */
		if(GetNextReport(&GloveReportData))
		{
			/* Write Generic Report Data */
			Endpoint_Write_Stream_LE(&GloveReportData, sizeof(USB_GloveReport_Data_t), NULL);
			/* Finalize the stream transfer to send the last packet */
			Endpoint_ClearIN();
		}
	}
}

ISR(USART1_RX_vect)
{
	uint8_t data = 0;
	
	data = UDR1;
	
	if (data == '[' /*&& (flagReportData & FRD_SEND)*/)
	{
		flagReportData  = 0x00;
		flagReportData |= FRD_READ;
		pRxProbe = rxUART;
		
		if(pReportProbe <= (uint8_t*)&gGloveReportData[1])
			pReportProbe = (uint8_t*)&gGloveReportData[1];
		else
			pReportProbe = (uint8_t*)&gGloveReportData[0];
			
		readCnt = 0;
		goto END_UART_ISR;
	}

	if((pRxProbe < rxUART + RXUART_BUFF_SIZE) && (flagReportData & FRD_READ))
	{
		if(data == ']')
		{
			flagReportData &= ~FRD_READ;
			flagReportData |= FRD_READY;
			
			if(pReportProbe > (uint8_t*)&gGloveReportData[1])
				pReportData = (uint8_t*)&gGloveReportData[1];
			else
				pReportData = (uint8_t*)&gGloveReportData[0];
			goto END_UART_ISR;
		}
		*(pRxProbe++) = data;

		if(++readCnt >= 5)
		{
			*pRxProbe = 255;	// to stop b85tob
			b85tob(pReportProbe, pRxProbe - 5);
			readCnt = 0;
			pReportProbe += 4;
		}
		
	}
	
END_UART_ISR:
	return;
}
