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

#define FRD_READY		(1<<0)
#define FRD_REPORTED	(1<<1)
#define FRD_DIRTY		(1<<2)

volatile uint8_t flagReportData = 0;	// FRD
volatile USB_GloveReport_Data_t gGloveReportData = {0, };

int main(void)
{
	SetupHardware();
	
	UART_printString("GloveHID Setup Start\n\r");
	GlobalInterruptEnable();

	for (;;)
	{
		HID_Task();
		USB_USBTask();
	}
}


ISR(USART1_RX_vect)	// while(!(UCSR0A & (1<<RXC0)));
{
	volatile uint8_t data;
	volatile static uint8_t read_byte_cnt = 0;
	volatile static uint16_t* pData = (uint16_t*)&(gGloveReportData.accX);
	data = UDR1;
	if (data == START_CHAR)
	{
		flagReportData &= ~(FRD_DIRTY);
		flagReportData &= ~(FRD_READY);
		pData = (uint16_t*)&(gGloveReportData.accX);
		read_byte_cnt = 0;
		return;
	}
	else if(data == END_CHAR)
	{
		if(read_byte_cnt == 18){
			flagReportData &= ~(FRD_DIRTY);
			flagReportData |= FRD_READY;	
		}
		else
			flagReportData |= FRD_DIRTY;
		return;
	}
	
	if(flagReportData & FRD_DIRTY)
		return;
	
	if(read_byte_cnt%2)
	{
		*pData |= data;
		++pData;
	}
	else
		*pData = data<<8;

	++read_byte_cnt;
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
	UART_INIT(9600);
}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Device_Connect(void)
{
	/* Indicate USB enumerating */
	UART_printString("Connected\n\r");
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
				static USB_GloveReport_Data_t GloveReportData;
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
				USB_GloveReport_Data_t GloveReportData_temp;

				Endpoint_ClearSETUP();

				/* Read the report data from the control endpoint */
				Endpoint_Read_Control_Stream_LE(&GloveReportData_temp, sizeof(USB_GloveReport_Data_t));
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
	if(flagReportData & FRD_READY)
	{
		ReportData->accX = gGloveReportData.accX;
		ReportData->accY = gGloveReportData.accX;
		ReportData->accZ = gGloveReportData.accX;
		
		ReportData->gyroX = gGloveReportData.gyroX;
		ReportData->gyroY = gGloveReportData.gyroY;
		ReportData->gyroZ = gGloveReportData.gyroZ;
		
		ReportData->geoX = gGloveReportData.geoX;
		ReportData->geoY = gGloveReportData.geoY;
		ReportData->geoZ = gGloveReportData.geoZ;
		
		ReportData->btn = 0;
		flagReportData &= ~(FRD_READY);
		return true;
	}
	return false;
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
		static USB_GloveReport_Data_t GloveReportData;

		/* Create Generic Report Data */
		if(GetNextReport(&GloveReportData))
		{
		
			//UART_printString("[ReportData Write] \n\r");
			//UART_printString("\tAcc  [ X:Y:Z ] | [ ");
			//UART_printUINT(GloveReportData.accX);
			//UART_printString(" : ");
			//UART_printUINT(GloveReportData.accY);
			//UART_printString(" : ");
			//UART_printUINT(GloveReportData.accZ);
			//UART_printString(" ]\n\r");
			//UART_printString("\tGyro [ X:Y:Z ] | [ ");
			//UART_printUINT(GloveReportData.gyroX);
			//UART_printString(" : ");
			//UART_printUINT(GloveReportData.gyroY);
			//UART_printString(" : ");
			//UART_printUINT(GloveReportData.gyroZ);
			//UART_printString(" ]\n\r");
			//UART_printString("\tGeo  [ X:Y:Z ] | [ ");
			//UART_printUINT(GloveReportData.geoX);
			//UART_printString(" : ");
			//UART_printUINT(GloveReportData.geoY);
			//UART_printString(" : ");
			//UART_printUINT(GloveReportData.geoZ);
			//UART_printString(" ]\n\r");

			/* Write Generic Report Data */
			Endpoint_Write_Stream_LE(&GloveReportData, sizeof(USB_GloveReport_Data_t), NULL);

			/* Finalize the stream transfer to send the last packet */
			Endpoint_ClearIN();
		}
	}
}

