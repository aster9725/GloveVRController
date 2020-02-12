#include <wtypes.h>
#include <setupapi.h>
#include "hidsdi.h"
#include "hid.h"
#include <strsafe.h>
#include <intsafe.h>
#include <stdlib.h>

#pragma warning(disable:28146) // Warning is meant for kernel mode drivers 

BOOLEAN
OpenHidDevice(
    _In_     STRSAFE_LPSTR DevicePath,
    _In_     BOOL           HasReadAccess,
    _In_     BOOL           HasWriteAccess,
    _In_     BOOL           IsOverlapped,
    _In_     BOOL           IsExclusive,
    _Out_    PHID_DEVICE    HidDevice
)
/*++
RoutineDescription:
    Given the HardwareDeviceInfo, representing a handle to the plug and
    play information, and deviceInfoData, representing a specific hid device,
    open that device and fill in all the relivant information in the given
    HID_DEVICE structure.

    return if the open and initialization was successfull or not.

--*/
{
    DWORD   accessFlags = 0;
    DWORD   sharingFlags = 0;
    BOOLEAN bRet = FALSE;
    INT     iDevicePathSize;

    RtlZeroMemory(HidDevice, sizeof(HID_DEVICE));
    HidDevice->HidDevice = INVALID_HANDLE_VALUE;

    if (NULL == DevicePath)
    {
        goto Done;
    }

    iDevicePathSize = (INT)strlen(DevicePath) + 1;

    HidDevice->DevicePath = (STRSAFE_LPSTR)malloc(iDevicePathSize);

    if (NULL == HidDevice->DevicePath)
    {
        goto Done;
    }

    StringCbCopy(HidDevice->DevicePath, iDevicePathSize, DevicePath);

    if (HasReadAccess)
    {
        accessFlags |= GENERIC_READ;
    }

    if (HasWriteAccess)
    {
        accessFlags |= GENERIC_WRITE;
    }

    if (!IsExclusive)
    {
        sharingFlags = FILE_SHARE_READ | FILE_SHARE_WRITE;
    }

    //
    //  The hid.dll api's do not pass the overlapped structure into deviceiocontrol
    //  so to use them we must have a non overlapped device.  If the request is for
    //  an overlapped device we will close the device below and get a handle to an
    //  overlapped device
    //

    HidDevice->HidDevice = CreateFile(DevicePath,
        accessFlags,
        sharingFlags,
        NULL,        // no SECURITY_ATTRIBUTES structure
        OPEN_EXISTING, // No special create flags
        0,   // Open device as non-overlapped so we can get data
        NULL);       // No template file

    if (INVALID_HANDLE_VALUE == HidDevice->HidDevice)
    {
        goto Done;
    }

    HidDevice->OpenedForRead = HasReadAccess;
    HidDevice->OpenedForWrite = HasWriteAccess;
    HidDevice->OpenedOverlapped = IsOverlapped;
    HidDevice->OpenedExclusive = IsExclusive;

    //
    // If the device was not opened as overlapped, then fill in the rest of the
    //  HidDevice structure.  However, if opened as overlapped, this handle cannot
    //  be used in the calls to the HidD_ exported functions since each of these
    //  functions does synchronous I/O.
    //

    if (!HidD_GetPreparsedData(HidDevice->HidDevice, &HidDevice->Ppd))
    {
        goto Done;
    }

    if (!HidD_GetAttributes(HidDevice->HidDevice, &HidDevice->Attributes))
    {
        goto Done;
    }

    if (!HidP_GetCaps(HidDevice->Ppd, &HidDevice->Caps))
    {
        goto Done;
    }

    //
    // At this point the client has a choice.  It may chose to look at the
    // Usage and Page of the top level collection found in the HIDP_CAPS
    // structure.  In this way it could just use the usages it knows about.
    // If either HidP_GetUsages or HidP_GetUsageValue return an error then
    // that particular usage does not exist in the report.
    // This is most likely the preferred method as the application can only
    // use usages of which it already knows.
    // In this case the app need not even call GetButtonCaps or GetValueCaps.
    //
    // In this example, however, we will call FillDeviceInfo to look for all
    //    of the usages in the device.
    //

    if (FALSE == FillDeviceInfo(HidDevice))
    {
        goto Done;
    }

    if (IsOverlapped)
    {
        CloseHandle(HidDevice->HidDevice);
        HidDevice->HidDevice = INVALID_HANDLE_VALUE;

        HidDevice->HidDevice = CreateFile(DevicePath,
            accessFlags,
            sharingFlags,
            NULL,        // no SECURITY_ATTRIBUTES structure
            OPEN_EXISTING, // No special create flags
            FILE_FLAG_OVERLAPPED, // Now we open the device as overlapped
            NULL);       // No template file

        if (INVALID_HANDLE_VALUE == HidDevice->HidDevice)
        {
            goto Done;
        }
    }

    bRet = TRUE;

Done:
    if (!bRet)
    {
        CloseHidDevice(HidDevice);
    }

    return (bRet);
}

BOOLEAN
FindKnownHidDevice(
    OUT PHID_DEVICE HidDevice // A array of struct _HID_DEVICE
)
/*++
Routine Description:
   Do the required PnP things in order to find all the HID devices in
   the system at this time.
--*/
{
    HDEVINFO                            hardwareDeviceInfo = INVALID_HANDLE_VALUE;
    SP_DEVICE_INTERFACE_DATA            deviceInfoData;
    ULONG                               i;
    BOOLEAN                             done = FALSE;
    PHID_DEVICE                         hidDeviceInst;
    GUID                                hidGuid;
    PSP_DEVICE_INTERFACE_DETAIL_DATA    functionClassDeviceData = NULL;
    ULONG                               predictedLength = 0;
    ULONG                               requiredLength = 0;
    PHID_DEVICE                         newHidDevices;


    HidD_GetHidGuid(&hidGuid);

    

    HidDevice = NULL;

    //
    // Open a handle to the plug and play dev node.
    //
    hardwareDeviceInfo = SetupDiGetClassDevs(&hidGuid,
        NULL, // Define no enumerator (global)
        NULL, // Define no
        (DIGCF_PRESENT | // Only Devices present
            DIGCF_DEVICEINTERFACE)); // Function class devices.
    
    if (INVALID_HANDLE_VALUE == hardwareDeviceInfo)
    {
        goto Done;
    }

    //
    // Take a wild guess to start
    //

    done = FALSE;
    deviceInfoData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

    while (!done)
    {
        if (!HidDevice) {
            HidDevice = (PHID_DEVICE)calloc(1, sizeof(HID_DEVICE));

            if (NULL == HidDevice)
            {
                goto Done;
            }
        }

        hidDeviceInst = HidDevice;
        i = 0;
        while (1)
        {
            //
            // Initialize an empty HID_DEVICE
            //
            RtlZeroMemory(hidDeviceInst, sizeof(HID_DEVICE));
            
            hidDeviceInst->HidDevice = INVALID_HANDLE_VALUE;

            if (SetupDiEnumDeviceInterfaces(hardwareDeviceInfo,
                0, // No care about specific PDOs
                &hidGuid,
                i++,
                &deviceInfoData))
            {
                //
                // allocate a function class device data structure to receive the
                // goods about this particular device.
                //

                SetupDiGetDeviceInterfaceDetail(
                    hardwareDeviceInfo,
                    &deviceInfoData,
                    NULL, // probing so no output buffer yet
                    0, // probing so output buffer length of zero
                    &requiredLength,
                    NULL); // not interested in the specific dev-node

                
                predictedLength = requiredLength;

                functionClassDeviceData = (PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(predictedLength);
                if (functionClassDeviceData)
                {
                    functionClassDeviceData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
                    ZeroMemory(functionClassDeviceData->DevicePath, sizeof(functionClassDeviceData->DevicePath));
                }
                else
                {
                    goto Done;
                }

                //
                // Retrieve the information from Plug and Play.
                //

                if (SetupDiGetDeviceInterfaceDetail(
                    hardwareDeviceInfo,
                    &deviceInfoData,
                    functionClassDeviceData,
                    predictedLength,
                    &requiredLength,
                    NULL))
                {
                    //
                    // Open device with just generic query abilities to begin with
                    //

                    if (!OpenHidDevice(functionClassDeviceData->DevicePath,
                        FALSE,      // ReadAccess - none
                        FALSE,      // WriteAccess - none
                        FALSE,       // Overlapped - no
                        FALSE,       // Exclusive - no
                        hidDeviceInst))
                    {
                        //
                        // Save the device path so it can be still listed.
                        //
                        INT     iDevicePathSize;

                        iDevicePathSize = (INT)strlen(functionClassDeviceData->DevicePath) + 1;

                        hidDeviceInst->DevicePath = (STRSAFE_LPSTR)malloc(iDevicePathSize);

                        if (NULL != hidDeviceInst->DevicePath)
                        {
                            StringCbCopy(hidDeviceInst->DevicePath, iDevicePathSize, functionClassDeviceData->DevicePath);
                        }
                    }
                }
                
                free(functionClassDeviceData);
                functionClassDeviceData = NULL;

                if (hidDeviceInst->Attributes.VendorID == 0x1991 && hidDeviceInst->Attributes.ProductID == 0x1129)
                {
                    done = TRUE;
                    break;
                }
            }
            else
            {
                if (ERROR_NO_MORE_ITEMS == GetLastError())
                {
                    break;
                }
            }
        }
    }

Done:
    if (FALSE == done)
    {
        if (NULL != HidDevice)
        {
            free(HidDevice);
            HidDevice = NULL;
        }
    }

    if (INVALID_HANDLE_VALUE != hardwareDeviceInfo)
    {
        SetupDiDestroyDeviceInfoList(hardwareDeviceInfo);
        hardwareDeviceInfo = INVALID_HANDLE_VALUE;
    }

    return done;
}

BOOLEAN
FillDeviceInfo(
    IN  PHID_DEVICE HidDevice
)
{
    ULONG               numValues;
    USHORT              numCaps;
    PHIDP_BUTTON_CAPS   buttonCaps;
    PHIDP_VALUE_CAPS    valueCaps;
    PHID_DATA           data;
    ULONG               i;
    USAGE               usage;
    UINT                dataIdx;
    ULONG               newFeatureDataLength;
    ULONG               tmpSum;
    BOOLEAN             bRet = FALSE;

    //
    // setup Input Data buffers.
    //

    //
    // Allocate memory to hold on input report
    //

    HidDevice->InputReportBuffer = (PCHAR)
        calloc(HidDevice->Caps.InputReportByteLength, sizeof(CHAR));


    //
    // Allocate memory to hold the button and value capabilities.
    // NumberXXCaps is in terms of array elements.
    //

    HidDevice->InputButtonCaps = buttonCaps = (PHIDP_BUTTON_CAPS)
        calloc(HidDevice->Caps.NumberInputButtonCaps, sizeof(HIDP_BUTTON_CAPS));

    if (NULL == buttonCaps)
    {
        goto Done;
    }

    HidDevice->InputValueCaps = valueCaps = (PHIDP_VALUE_CAPS)
        calloc(HidDevice->Caps.NumberInputValueCaps, sizeof(HIDP_VALUE_CAPS));

    if (NULL == valueCaps)
    {
        goto Done;
    }

    //
    // Have the HidP_X functions fill in the capability structure arrays.
    //

    numCaps = HidDevice->Caps.NumberInputButtonCaps;

    if (numCaps > 0)
    {
        if (HIDP_STATUS_SUCCESS != (HidP_GetButtonCaps(HidP_Input,
            buttonCaps,
            &numCaps,
            HidDevice->Ppd)))
        {
            goto Done;
        }
    }

    numCaps = HidDevice->Caps.NumberInputValueCaps;

    if (numCaps > 0)
    {
        if (HIDP_STATUS_SUCCESS != (HidP_GetValueCaps(HidP_Input,
            valueCaps,
            &numCaps,
            HidDevice->Ppd)))
        {
            goto Done;
        }
    }


    //
    // Depending on the device, some value caps structures may represent more
    // than one value.  (A range).  In the interest of being verbose, over
    // efficient, we will expand these so that we have one and only one
    // struct _HID_DATA for each value.
    //
    // To do this we need to count up the total number of values are listed
    // in the value caps structure.  For each element in the array we test
    // for range if it is a range then UsageMax and UsageMin describe the
    // usages for this range INCLUSIVE.
    //

    numValues = 0;
    for (i = 0; i < HidDevice->Caps.NumberInputValueCaps; i++, valueCaps++)
    {
        if (valueCaps->IsRange)
        {
            numValues += valueCaps->Range.UsageMax - valueCaps->Range.UsageMin + 1;
            if (valueCaps->Range.UsageMin > valueCaps->Range.UsageMax)
            {
                goto Done;  // overrun check
            }
        }
        else
        {
            numValues++;
        }
    }

    valueCaps = HidDevice->InputValueCaps;

    //
    // Allocate a buffer to hold the struct _HID_DATA structures.
    // One element for each set of buttons, and one element for each value
    // found.
    //

    HidDevice->InputDataLength = HidDevice->Caps.NumberInputButtonCaps
        + numValues;

    HidDevice->InputData = data = (PHID_DATA)
        calloc(HidDevice->InputDataLength, sizeof(HID_DATA));

    if (NULL == data)
    {
        goto Done;
    }

    //
    // Fill in the button data
    //
    dataIdx = 0;
    for (i = 0;
        i < HidDevice->Caps.NumberInputButtonCaps;
        i++, data++, buttonCaps++, dataIdx++)
    {
        data->IsButtonData = TRUE;
        data->Status = HIDP_STATUS_SUCCESS;
        data->UsagePage = buttonCaps->UsagePage;
        if (buttonCaps->IsRange)
        {
            data->ButtonData.UsageMin = buttonCaps->Range.UsageMin;
            data->ButtonData.UsageMax = buttonCaps->Range.UsageMax;
        }
        else
        {
            data->ButtonData.UsageMin = data->ButtonData.UsageMax = buttonCaps->NotRange.Usage;
        }

        data->ButtonData.MaxUsageLength = HidP_MaxUsageListLength(
            HidP_Input,
            buttonCaps->UsagePage,
            HidDevice->Ppd);
        data->ButtonData.Usages = (PUSAGE)
            calloc(data->ButtonData.MaxUsageLength, sizeof(USAGE));

        data->ReportID = buttonCaps->ReportID;
    }

    //
    // Fill in the value data
    //

    for (i = 0; i < HidDevice->Caps.NumberInputValueCaps; i++, valueCaps++)
    {
        if (valueCaps->IsRange)
        {
            for (usage = valueCaps->Range.UsageMin;
                usage <= valueCaps->Range.UsageMax;
                usage++)
            {
                if (dataIdx >= (HidDevice->InputDataLength))
                {
                    goto Done; // error case
                }
                data->IsButtonData = FALSE;
                data->Status = HIDP_STATUS_SUCCESS;
                data->UsagePage = valueCaps->UsagePage;
                data->ValueData.Usage = usage;
                data->ReportID = valueCaps->ReportID;
                data++;
                dataIdx++;
            }
        }
        else
        {
            if (dataIdx >= (HidDevice->InputDataLength))
            {
                goto Done; // error case
            }
            data->IsButtonData = FALSE;
            data->Status = HIDP_STATUS_SUCCESS;
            data->UsagePage = valueCaps->UsagePage;
            data->ValueData.Usage = valueCaps->NotRange.Usage;
            data->ReportID = valueCaps->ReportID;
            data++;
            dataIdx++;
        }
    }

    //
    // setup Output Data buffers.
    //

    HidDevice->OutputReportBuffer = (PCHAR)
        calloc(HidDevice->Caps.OutputReportByteLength, sizeof(CHAR));

    HidDevice->OutputButtonCaps = buttonCaps = (PHIDP_BUTTON_CAPS)
        calloc(HidDevice->Caps.NumberOutputButtonCaps, sizeof(HIDP_BUTTON_CAPS));

    if (NULL == buttonCaps)
    {
        goto Done;
    }

    HidDevice->OutputValueCaps = valueCaps = (PHIDP_VALUE_CAPS)
        calloc(HidDevice->Caps.NumberOutputValueCaps, sizeof(HIDP_VALUE_CAPS));

    if (NULL == valueCaps)
    {
        goto Done;
    }

    numCaps = HidDevice->Caps.NumberOutputButtonCaps;
    if (numCaps > 0)
    {
        if (HIDP_STATUS_SUCCESS != (HidP_GetButtonCaps(HidP_Output,
            buttonCaps,
            &numCaps,
            HidDevice->Ppd)))
        {
            goto Done;
        }
    }

    numCaps = HidDevice->Caps.NumberOutputValueCaps;
    if (numCaps > 0)
    {
        if (HIDP_STATUS_SUCCESS != (HidP_GetValueCaps(HidP_Output,
            valueCaps,
            &numCaps,
            HidDevice->Ppd)))
        {
            goto Done;
        }
    }

    numValues = 0;
    for (i = 0; i < HidDevice->Caps.NumberOutputValueCaps; i++, valueCaps++)
    {
        if (valueCaps->IsRange)
        {
            numValues += valueCaps->Range.UsageMax
                - valueCaps->Range.UsageMin + 1;
        }
        else
        {
            numValues++;
        }
    }
    valueCaps = HidDevice->OutputValueCaps;

    HidDevice->OutputDataLength = HidDevice->Caps.NumberOutputButtonCaps
        + numValues;

    HidDevice->OutputData = data = (PHID_DATA)
        calloc(HidDevice->OutputDataLength, sizeof(HID_DATA));

    if (NULL == data)
    {
        goto Done;
    }

    for (i = 0;
        i < HidDevice->Caps.NumberOutputButtonCaps;
        i++, data++, buttonCaps++)
    {
        if (i >= HidDevice->OutputDataLength)
        {
            goto Done;
        }

        if (FAILED(ULongAdd((HidDevice->Caps).NumberOutputButtonCaps,
            (valueCaps->Range).UsageMax, &tmpSum)))
        {
            goto Done;
        }

        if ((valueCaps->Range).UsageMin == tmpSum)
        {
            goto Done;
        }

        data->IsButtonData = TRUE;
        data->Status = HIDP_STATUS_SUCCESS;
        data->UsagePage = buttonCaps->UsagePage;

        if (buttonCaps->IsRange)
        {
            data->ButtonData.UsageMin = buttonCaps->Range.UsageMin;
            data->ButtonData.UsageMax = buttonCaps->Range.UsageMax;
        }
        else
        {
            data->ButtonData.UsageMin = data->ButtonData.UsageMax = buttonCaps->NotRange.Usage;
        }

        data->ButtonData.MaxUsageLength = HidP_MaxUsageListLength(
            HidP_Output,
            buttonCaps->UsagePage,
            HidDevice->Ppd);

        data->ButtonData.Usages = (PUSAGE)
            calloc(data->ButtonData.MaxUsageLength, sizeof(USAGE));

        data->ReportID = buttonCaps->ReportID;
    }

    for (i = 0; i < HidDevice->Caps.NumberOutputValueCaps; i++, valueCaps++)
    {
        if (valueCaps->IsRange)
        {
            for (usage = valueCaps->Range.UsageMin;
                usage <= valueCaps->Range.UsageMax;
                usage++)
            {
                data->IsButtonData = FALSE;
                data->Status = HIDP_STATUS_SUCCESS;
                data->UsagePage = valueCaps->UsagePage;
                data->ValueData.Usage = usage;
                data->ReportID = valueCaps->ReportID;
                data++;
            }
        }
        else
        {
            data->IsButtonData = FALSE;
            data->Status = HIDP_STATUS_SUCCESS;
            data->UsagePage = valueCaps->UsagePage;
            data->ValueData.Usage = valueCaps->NotRange.Usage;
            data->ReportID = valueCaps->ReportID;
            data++;
        }
    }

    //
    // setup Feature Data buffers.
    //

    HidDevice->FeatureReportBuffer = (PCHAR)
        calloc(HidDevice->Caps.FeatureReportByteLength, sizeof(CHAR));

    HidDevice->FeatureButtonCaps = buttonCaps = (PHIDP_BUTTON_CAPS)
        calloc(HidDevice->Caps.NumberFeatureButtonCaps, sizeof(HIDP_BUTTON_CAPS));

    if (NULL == buttonCaps)
    {
        goto Done;
    }

    HidDevice->FeatureValueCaps = valueCaps = (PHIDP_VALUE_CAPS)
        calloc(HidDevice->Caps.NumberFeatureValueCaps, sizeof(HIDP_VALUE_CAPS));

    if (NULL == valueCaps)
    {
        goto Done;
    }

    numCaps = HidDevice->Caps.NumberFeatureButtonCaps;
    if (numCaps > 0)
    {
        if (HIDP_STATUS_SUCCESS != (HidP_GetButtonCaps(HidP_Feature,
            buttonCaps,
            &numCaps,
            HidDevice->Ppd)))
        {
            goto Done;
        }
    }

    numCaps = HidDevice->Caps.NumberFeatureValueCaps;
    if (numCaps > 0)
    {
        if (HIDP_STATUS_SUCCESS != (HidP_GetValueCaps(HidP_Feature,
            valueCaps,
            &numCaps,
            HidDevice->Ppd)))
        {
            goto Done;
        }
    }

    numValues = 0;
    for (i = 0; i < HidDevice->Caps.NumberFeatureValueCaps; i++, valueCaps++)
    {
        if (valueCaps->IsRange)
        {
            numValues += valueCaps->Range.UsageMax
                - valueCaps->Range.UsageMin + 1;
        }
        else
        {
            numValues++;
        }
    }
    valueCaps = HidDevice->FeatureValueCaps;

    if (FAILED(ULongAdd(HidDevice->Caps.NumberFeatureButtonCaps,
        numValues, &newFeatureDataLength)))
    {
        goto Done;
    }

    HidDevice->FeatureDataLength = newFeatureDataLength;

    HidDevice->FeatureData = data = (PHID_DATA)
        calloc(HidDevice->FeatureDataLength, sizeof(HID_DATA));

    if (NULL == data)
    {
        goto Done;
    }

    dataIdx = 0;
    for (i = 0;
        i < HidDevice->Caps.NumberFeatureButtonCaps;
        i++, data++, buttonCaps++, dataIdx++)
    {
        data->IsButtonData = TRUE;
        data->Status = HIDP_STATUS_SUCCESS;
        data->UsagePage = buttonCaps->UsagePage;

        if (buttonCaps->IsRange)
        {
            data->ButtonData.UsageMin = buttonCaps->Range.UsageMin;
            data->ButtonData.UsageMax = buttonCaps->Range.UsageMax;
        }
        else
        {
            data->ButtonData.UsageMin = data->ButtonData.UsageMax = buttonCaps->NotRange.Usage;
        }

        data->ButtonData.MaxUsageLength = HidP_MaxUsageListLength(
            HidP_Feature,
            buttonCaps->UsagePage,
            HidDevice->Ppd);
        data->ButtonData.Usages = (PUSAGE)
            calloc(data->ButtonData.MaxUsageLength, sizeof(USAGE));

        data->ReportID = buttonCaps->ReportID;
    }

    for (i = 0; i < HidDevice->Caps.NumberFeatureValueCaps; i++, valueCaps++)
    {
        if (valueCaps->IsRange)
        {
            for (usage = valueCaps->Range.UsageMin;
                usage <= valueCaps->Range.UsageMax;
                usage++)
            {
                if (dataIdx >= (HidDevice->FeatureDataLength))
                {
                    goto Done; // error case
                }
                data->IsButtonData = FALSE;
                data->Status = HIDP_STATUS_SUCCESS;
                data->UsagePage = valueCaps->UsagePage;
                data->ValueData.Usage = usage;
                data->ReportID = valueCaps->ReportID;
                data++;
                dataIdx++;
            }
        }
        else
        {
            if (dataIdx >= (HidDevice->FeatureDataLength))
            {
                goto Done; // error case
            }
            data->IsButtonData = FALSE;
            data->Status = HIDP_STATUS_SUCCESS;
            data->UsagePage = valueCaps->UsagePage;
            data->ValueData.Usage = valueCaps->NotRange.Usage;
            data->ReportID = valueCaps->ReportID;
            data++;
            dataIdx++;
        }
    }

    bRet = TRUE;

Done:
    //
    // We leave the resource clean-up to the caller. 
    //
    return (bRet);
}

VOID
CloseHidDevice(
    IN PHID_DEVICE HidDevice
)
{
    if (NULL != HidDevice->DevicePath)
    {
        free(HidDevice->DevicePath);
        HidDevice->DevicePath = NULL;
    }

    if (INVALID_HANDLE_VALUE != HidDevice->HidDevice)
    {
        CloseHandle(HidDevice->HidDevice);
        HidDevice->HidDevice = INVALID_HANDLE_VALUE;
    }

    if (NULL != HidDevice->Ppd)
    {
        HidD_FreePreparsedData(HidDevice->Ppd);
        HidDevice->Ppd = NULL;
    }

    if (NULL != HidDevice->InputReportBuffer)
    {
        free(HidDevice->InputReportBuffer);
        HidDevice->InputReportBuffer = NULL;
    }

    if (NULL != HidDevice->InputData)
    {
        free(HidDevice->InputData);
        HidDevice->InputData = NULL;
    }

    if (NULL != HidDevice->InputButtonCaps)
    {
        free(HidDevice->InputButtonCaps);
        HidDevice->InputButtonCaps = NULL;
    }

    if (NULL != HidDevice->InputValueCaps)
    {
        free(HidDevice->InputValueCaps);
        HidDevice->InputValueCaps = NULL;
    }

    if (NULL != HidDevice->OutputReportBuffer)
    {
        free(HidDevice->OutputReportBuffer);
        HidDevice->OutputReportBuffer = NULL;
    }

    if (NULL != HidDevice->OutputData)
    {
        free(HidDevice->OutputData);
        HidDevice->OutputData = NULL;
    }

    if (NULL != HidDevice->OutputButtonCaps)
    {
        free(HidDevice->OutputButtonCaps);
        HidDevice->OutputButtonCaps = NULL;
    }

    if (NULL != HidDevice->OutputValueCaps)
    {
        free(HidDevice->OutputValueCaps);
        HidDevice->OutputValueCaps = NULL;
    }

    if (NULL != HidDevice->FeatureReportBuffer)
    {
        free(HidDevice->FeatureReportBuffer);
        HidDevice->FeatureReportBuffer = NULL;
    }

    if (NULL != HidDevice->FeatureData)
    {
        free(HidDevice->FeatureData);
        HidDevice->FeatureData = NULL;
    }

    if (NULL != HidDevice->FeatureButtonCaps)
    {
        free(HidDevice->FeatureButtonCaps);
        HidDevice->FeatureButtonCaps = NULL;
    }

    if (NULL != HidDevice->FeatureValueCaps)
    {
        free(HidDevice->FeatureValueCaps);
        HidDevice->FeatureValueCaps = NULL;
    }

    return;
}

BOOLEAN
Read(
    PHID_DEVICE    HidDevice
)
/*++
RoutineDescription:
   Given a struct _HID_DEVICE, obtain a read report and unpack the values
   into the InputData array.
--*/
{
    DWORD       bytesRead;
    BOOLEAN     result = FALSE;

    if (!ReadFile(HidDevice->HidDevice,
        HidDevice->InputReportBuffer,
        HidDevice->Caps.InputReportByteLength,
        &bytesRead,
        NULL))
    {
        goto Done;
    }

    ASSERT(bytesRead == HidDevice->Caps.InputReportByteLength);
    if (bytesRead != HidDevice->Caps.InputReportByteLength)
    {
        goto Done;
    }

    result = UnpackReport(HidDevice->InputReportBuffer,
        HidDevice->Caps.InputReportByteLength,
        HidP_Input,
        HidDevice->InputData,
        HidDevice->InputDataLength,
        HidDevice->Ppd);
Done:
    return result;
}

BOOLEAN
ReadOverlapped(
    PHID_DEVICE     HidDevice,
    HANDLE          CompletionEvent,
    LPOVERLAPPED    Overlap
)
/*++
RoutineDescription:
   Given a struct _HID_DEVICE, obtain a read report and unpack the values
   into the InputData array.
--*/
{
    DWORD       bytesRead;
    BOOL        readStatus;

    /*
    // Setup the overlap structure using the completion event passed in to
    //  to use for signalling the completion of the Read
    */

    memset(Overlap, 0, sizeof(OVERLAPPED));

    Overlap->hEvent = CompletionEvent;

    /*
    // Execute the read call saving the return code to determine how to
    //  proceed (ie. the read completed synchronously or not).
    */

    readStatus = ReadFile(HidDevice->HidDevice,
        HidDevice->InputReportBuffer,
        HidDevice->Caps.InputReportByteLength,
        &bytesRead,
        Overlap);

    /*
    // If the readStatus is FALSE, then one of two cases occurred.
    //  1) ReadFile call succeeded but the Read is an overlapped one.  Here,
    //      we should return TRUE to indicate that the Read succeeded.  However,
    //      the calling thread should be blocked on the completion event
    //      which means it won't continue until the read actually completes
    //
    //  2) The ReadFile call failed for some unknown reason...In this case,
    //      the return code will be FALSE
    */

    if (!readStatus)
    {
        return (ERROR_IO_PENDING == GetLastError());
    }

    /*
    // If readStatus is TRUE, then the ReadFile call completed synchronously,
    //   since the calling thread is probably going to wait on the completion
    //   event, signal the event so it knows it can continue.
    */

    else
    {
        SetEvent(CompletionEvent);
        return (TRUE);
    }
}

BOOLEAN
Write(
    PHID_DEVICE    HidDevice
)
/*++
RoutineDescription:
   Given a struct _HID_DEVICE, take the information in the HID_DATA array
   pack it into multiple write reports and send each report to the HID device
--*/
{
    DWORD     bytesWritten;
    PHID_DATA pData;
    ULONG     Index;
    BOOLEAN   Status;
    BOOLEAN   WriteStatus;

    /*
    // Begin by looping through the HID_DEVICE's HID_DATA structure and setting
    //   the IsDataSet field to FALSE to indicate that each structure has
    //   not yet been set for this Write call.
    */

    pData = HidDevice->OutputData;

    for (Index = 0; Index < HidDevice->OutputDataLength; Index++, pData++)
    {
        pData->IsDataSet = FALSE;
    }

    /*
    // In setting all the data in the reports, we need to pack a report buffer
    //   and call WriteFile for each report ID that is represented by the
    //   device structure.  To do so, the IsDataSet field will be used to
    //   determine if a given report field has already been set.
    */

    Status = TRUE;

    pData = HidDevice->OutputData;
    for (Index = 0; Index < HidDevice->OutputDataLength; Index++, pData++)
    {

        if (!pData->IsDataSet)
        {
            /*
            // Package the report for this data structure.  PackReport will
            //    set the IsDataSet fields of this structure and any other
            //    structures that it includes in the report with this structure
            */

            PackReport(HidDevice->OutputReportBuffer,
                HidDevice->Caps.OutputReportByteLength,
                HidP_Output,
                pData,
                HidDevice->OutputDataLength - Index,
                HidDevice->Ppd);

            /*
            // Now a report has been packaged up...Send it down to the device
            */

            WriteStatus = WriteFile(HidDevice->HidDevice,
                HidDevice->OutputReportBuffer,
                HidDevice->Caps.OutputReportByteLength,
                &bytesWritten,
                NULL) && (bytesWritten == HidDevice->Caps.OutputReportByteLength);

            Status = Status && WriteStatus;
        }
    }
    return (Status);
}

BOOLEAN
SetFeature(
    PHID_DEVICE    HidDevice
)
/*++
RoutineDescription:
Given a struct _HID_DEVICE, take the information in the HID_DATA array
pack it into multiple reports and send it to the hid device via HidD_SetFeature()
--*/
{
    PHID_DATA pData;
    ULONG     Index;
    BOOLEAN   Status;
    BOOLEAN   FeatureStatus;
    /*
    // Begin by looping through the HID_DEVICE's HID_DATA structure and setting
    //   the IsDataSet field to FALSE to indicate that each structure has
    //   not yet been set for this SetFeature() call.
    */

    pData = HidDevice->FeatureData;

    for (Index = 0; Index < HidDevice->FeatureDataLength; Index++, pData++)
    {
        pData->IsDataSet = FALSE;
    }

    /*
    // In setting all the data in the reports, we need to pack a report buffer
    //   and call WriteFile for each report ID that is represented by the
    //   device structure.  To do so, the IsDataSet field will be used to
    //   determine if a given report field has already been set.
    */

    Status = TRUE;

    pData = HidDevice->FeatureData;
    for (Index = 0; Index < HidDevice->FeatureDataLength; Index++, pData++)
    {
        if (!pData->IsDataSet)
        {
            /*
            // Package the report for this data structure.  PackReport will
            //    set the IsDataSet fields of this structure and any other
            //    structures that it includes in the report with this structure
            */

            PackReport(HidDevice->FeatureReportBuffer,
                HidDevice->Caps.FeatureReportByteLength,
                HidP_Feature,
                pData,
                HidDevice->FeatureDataLength - Index,
                HidDevice->Ppd);

            /*
            // Now a report has been packaged up...Send it down to the device
            */

            FeatureStatus = (HidD_SetFeature(HidDevice->HidDevice,
                HidDevice->FeatureReportBuffer,
                HidDevice->Caps.FeatureReportByteLength));

            Status = FeatureStatus && Status;
        }
    }
    return (Status);
}

BOOLEAN
GetFeature(
    PHID_DEVICE    HidDevice
)
/*++
RoutineDescription:
   Given a struct _HID_DEVICE, fill in the feature data structures with
   all features on the device.  May issue multiple HidD_GetFeature() calls to
   deal with multiple report IDs.
--*/
{
    ULONG     Index;
    PHID_DATA pData;
    BOOLEAN   FeatureStatus;
    BOOLEAN   Status;

    /*
    // As with writing data, the IsDataSet value in all the structures should be
    //    set to FALSE to indicate that the value has yet to have been set
    */

    pData = HidDevice->FeatureData;

    for (Index = 0; Index < HidDevice->FeatureDataLength; Index++, pData++)
    {
        pData->IsDataSet = FALSE;
    }

    /*
    // Next, each structure in the HID_DATA buffer is filled in with a value
    //   that is retrieved from one or more calls to HidD_GetFeature.  The
    //   number of calls is equal to the number of reportIDs on the device
    */

    Status = TRUE;
    pData = HidDevice->FeatureData;

    for (Index = 0; Index < HidDevice->FeatureDataLength; Index++, pData++)
    {
        /*
        // If a value has yet to have been set for this structure, build a report
        //    buffer with its report ID as the first byte of the buffer and pass
        //    it in the HidD_GetFeature call.  Specifying the report ID in the
        //    first specifies which report is actually retrieved from the device.
        //    The rest of the buffer should be zeroed before the call
        */

        if (!pData->IsDataSet)
        {
            memset(HidDevice->FeatureReportBuffer, 0x00, HidDevice->Caps.FeatureReportByteLength);

            HidDevice->FeatureReportBuffer[0] = (UCHAR)pData->ReportID;

            FeatureStatus = HidD_GetFeature(HidDevice->HidDevice,
                HidDevice->FeatureReportBuffer,
                HidDevice->Caps.FeatureReportByteLength);

            /*
            // If the return value is TRUE, scan through the rest of the HID_DATA
            //    structures and fill whatever values we can from this report
            */


            if (FeatureStatus)
            {
                FeatureStatus = UnpackReport(HidDevice->FeatureReportBuffer,
                    HidDevice->Caps.FeatureReportByteLength,
                    HidP_Feature,
                    HidDevice->FeatureData,
                    HidDevice->FeatureDataLength,
                    HidDevice->Ppd);
            }

            Status = Status && FeatureStatus;
        }
    }

    return (Status);
}


BOOLEAN
UnpackReport(
    _In_reads_bytes_(ReportBufferLength)PCHAR ReportBuffer,
    IN       USHORT               ReportBufferLength,
    IN       HIDP_REPORT_TYPE     ReportType,
    IN OUT   PHID_DATA            Data,
    IN       ULONG                DataLength,
    IN       PHIDP_PREPARSED_DATA Ppd
)
/*++
Routine Description:
   Given ReportBuffer representing a report from a HID device where the first
   byte of the buffer is the report ID for the report, extract all the HID_DATA
   in the Data list from the given report.
--*/
{
    ULONG       numUsages; // Number of usages returned from GetUsages.
    ULONG       i;
    UCHAR       reportID;
    ULONG       Index;
    ULONG       nextUsage;
    BOOLEAN     result = FALSE;

    reportID = ReportBuffer[0];

    for (i = 0; i < DataLength; i++, Data++)
    {
        if (reportID == Data->ReportID)
        {
            if (Data->IsButtonData)
            {
                numUsages = Data->ButtonData.MaxUsageLength;

                Data->Status = HidP_GetUsages(ReportType,
                    Data->UsagePage,
                    0, // All collections
                    Data->ButtonData.Usages,
                    &numUsages,
                    Ppd,
                    ReportBuffer,
                    ReportBufferLength);

                if (HIDP_STATUS_SUCCESS != Data->Status)
                {
                    goto Done;
                }

                //
                // Get usages writes the list of usages into the buffer
                // Data->ButtonData.Usages newUsage is set to the number of usages
                // written into this array.
                // A usage cannot not be defined as zero, so we'll mark a zero
                // following the list of usages to indicate the end of the list of
                // usages
                //
                // NOTE: One anomaly of the GetUsages function is the lack of ability
                //        to distinguish the data for one ButtonCaps from another
                //        if two different caps structures have the same UsagePage
                //        For instance:
                //          Caps1 has UsagePage 07 and UsageRange of 0x00 - 0x167
                //          Caps2 has UsagePage 07 and UsageRange of 0xe0 - 0xe7
                //
                //        However, calling GetUsages for each of the data structs
                //          will return the same list of usages.  It is the 
                //          responsibility of the caller to set in the HID_DEVICE
                //          structure which usages actually are valid for the
                //          that structure. 
                //      

                /*
                // Search through the usage list and remove those that
                //    correspond to usages outside the define ranged for this
                //    data structure.
                */

                for (Index = 0, nextUsage = 0; Index < numUsages; Index++)
                {
                    if (Data->ButtonData.UsageMin <= Data->ButtonData.Usages[Index] &&
                        Data->ButtonData.Usages[Index] <= Data->ButtonData.UsageMax)
                    {
                        Data->ButtonData.Usages[nextUsage++] = Data->ButtonData.Usages[Index];

                    }
                }

                if (nextUsage < Data->ButtonData.MaxUsageLength)
                {
                    Data->ButtonData.Usages[nextUsage] = 0;
                }
            }
            else
            {
                Data->Status = HidP_GetUsageValue(
                    ReportType,
                    Data->UsagePage,
                    0,               // All Collections.
                    Data->ValueData.Usage,
                    &Data->ValueData.Value,
                    Ppd,
                    ReportBuffer,
                    ReportBufferLength);

                if (HIDP_STATUS_SUCCESS != Data->Status)
                {
                    goto Done;
                }

                Data->Status = HidP_GetScaledUsageValue(
                    ReportType,
                    Data->UsagePage,
                    0, // All Collections.
                    Data->ValueData.Usage,
                    &Data->ValueData.ScaledValue,
                    Ppd,
                    ReportBuffer,
                    ReportBufferLength);

                if (HIDP_STATUS_SUCCESS != Data->Status &&
                    HIDP_STATUS_NULL != Data->Status)
                {
                    goto Done;
                }

            }
            Data->IsDataSet = TRUE;
        }
    }

    result = TRUE;

Done:
    return (result);
}


BOOLEAN
PackReport(
    _Out_writes_bytes_(ReportBufferLength)PCHAR ReportBuffer,
    IN  USHORT               ReportBufferLength,
    IN  HIDP_REPORT_TYPE     ReportType,
    IN  PHID_DATA            Data,
    IN  ULONG                DataLength,
    IN  PHIDP_PREPARSED_DATA Ppd
)
/*++
Routine Description:
   This routine takes in a list of HID_DATA structures (DATA) and builds
      in ReportBuffer the given report for all data values in the list that
      correspond to the report ID of the first item in the list.

   For every data structure in the list that has the same report ID as the first
      item in the list will be set in the report.  Every data item that is
      set will also have it's IsDataSet field marked with TRUE.

   A return value of FALSE indicates an unexpected error occurred when setting
      a given data value.  The caller should expect that assume that no values
      within the given data structure were set.

   A return value of TRUE indicates that all data values for the given report
      ID were set without error.
--*/
{
    ULONG       numUsages; // Number of usages to set for a given report.
    ULONG       i;
    ULONG       CurrReportID;
    BOOLEAN     result = FALSE;

    /*
    // All report buffers that are initially sent need to be zero'd out
    */

    memset(ReportBuffer, (UCHAR)0, ReportBufferLength);

    /*
    // Go through the data structures and set all the values that correspond to
    //   the CurrReportID which is obtained from the first data structure
    //   in the list
    */

    CurrReportID = Data->ReportID;

    for (i = 0; i < DataLength; i++, Data++)
    {
        /*
        // There are two different ways to determine if we set the current data
        //    structure:
        //    1) Store the report ID were using and only attempt to set those
        //        data structures that correspond to the given report ID.  This
        //        example shows this implementation.
        //
        //    2) Attempt to set all of the data structures and look for the
        //        returned status value of HIDP_STATUS_INVALID_REPORT_ID.  This
        //        error code indicates that the given usage exists but has a
        //        different report ID than the report ID in the current report
        //        buffer
        */

        if (Data->ReportID == CurrReportID)
        {
            if (Data->IsButtonData)
            {
                numUsages = Data->ButtonData.MaxUsageLength;
                Data->Status = HidP_SetUsages(ReportType,
                    Data->UsagePage,
                    0, // All collections
                    Data->ButtonData.Usages,
                    &numUsages,
                    Ppd,
                    ReportBuffer,
                    ReportBufferLength);
            }
            else
            {
                Data->Status = HidP_SetUsageValue(ReportType,
                    Data->UsagePage,
                    0, // All Collections.
                    Data->ValueData.Usage,
                    Data->ValueData.Value,
                    Ppd,
                    ReportBuffer,
                    ReportBufferLength);
            }

            if (HIDP_STATUS_SUCCESS != Data->Status)
            {
                goto Done;
            }
        }
    }

    /*
    // At this point, all data structures that have the same ReportID as the
    //    first one will have been set in the given report.  Time to loop
    //    through the structure again and mark all of those data structures as
    //    having been set.
    */

    for (i = 0; i < DataLength; i++, Data++)
    {
        if (CurrReportID == Data->ReportID)
        {
            Data->IsDataSet = TRUE;
        }
    }

    result = TRUE;

Done:
    return result;
}
