
#pragma warning(disable:4067)
#include <Windows.h>
#include "hidsdi.h"
#include "setupapi.h"
#include <strsafe.h>

#define ASSERT(x)

typedef LIST_ENTRY      LIST_NODE_HDR, * PLIST_NODE_HDR;
typedef LIST_NODE_HDR   LIST, * PLIST;

typedef VOID PLIST_CALLBACK(PLIST_NODE_HDR);

#define InitializeList(nl) \
        { ((PLIST)(nl)) -> Flink = ((PLIST)(nl)) -> Blink = nl; }

typedef struct _SP_FNCLASS_DEVICE_DATA {
    DWORD cbSize;
    GUID  FunctionClassGuid;
    STRSAFE_LPWSTR DevicePath[ANYSIZE_ARRAY];
} SP_FNCLASS_DEVICE_DATA, * PSP_FNCLASS_DEVICE_DATA;

//
// A structure to hold the steady state data received from the hid device.
// Each time a read packet is received we fill in this structure.
// Each time we wish to write to a hid device we fill in this structure.
// This structure is here only for convenience.  Most real applications will
// have a more efficient way of moving the hid data to the read, write, and
// feature routines.
//
typedef struct _HID_DATA {
    BOOLEAN     IsButtonData;
    UCHAR       Reserved;
    USAGE       UsagePage;   // The usage page for which we are looking.
    ULONG       Status;      // The last status returned from the accessor function
                             // when updating this field.
    ULONG       ReportID;    // ReportID for this given data structure
    BOOLEAN     IsDataSet;   // Variable to track whether a given data structure
                             //  has already been added to a report structure

    union {
        struct {
            ULONG       UsageMin;       // Variables to track the usage minimum and max
            ULONG       UsageMax;       // If equal, then only a single usage
            ULONG       MaxUsageLength; // Usages buffer length.
            PUSAGE      Usages;         // list of usages (buttons ``down'' on the device.

        } ButtonData;
        struct {
            USAGE       Usage; // The usage describing this value;
            USHORT      Reserved;

            ULONG       Value;
            LONG        ScaledValue;
        } ValueData;
    };
} HID_DATA, * PHID_DATA;

typedef struct _HID_DEVICE {
    STRSAFE_LPSTR       DevicePath;
    HANDLE               HidDevice; // A file handle to the hid device.
    BOOL                 OpenedForRead;
    BOOL                 OpenedForWrite;
    BOOL                 OpenedOverlapped;
    BOOL                 OpenedExclusive;

    PHIDP_PREPARSED_DATA Ppd; // The opaque parser info describing this device
    HIDP_CAPS            Caps; // The Capabilities of this hid device.
    HIDD_ATTRIBUTES      Attributes;

    PCHAR                InputReportBuffer;
    _Field_size_(InputDataLength)
        PHID_DATA            InputData; // array of hid data structures
    ULONG                InputDataLength; // Num elements in this array.
    PHIDP_BUTTON_CAPS    InputButtonCaps;
    PHIDP_VALUE_CAPS     InputValueCaps;

    PCHAR                OutputReportBuffer;
    _Field_size_(OutputDataLength)
        PHID_DATA            OutputData;
    ULONG                OutputDataLength;
    PHIDP_BUTTON_CAPS    OutputButtonCaps;
    PHIDP_VALUE_CAPS     OutputValueCaps;

    PCHAR                FeatureReportBuffer;
    _Field_size_(FeatureDataLength) PHID_DATA            FeatureData;
    ULONG                FeatureDataLength;
    PHIDP_BUTTON_CAPS    FeatureButtonCaps;
    PHIDP_VALUE_CAPS     FeatureValueCaps;
} HID_DEVICE, * PHID_DEVICE;


BOOLEAN
OpenHidDevice(
    _In_     STRSAFE_LPSTR DevicePath,
    _In_     BOOL           HasReadAccess,
    _In_     BOOL           HasWriteAccess,
    _In_     BOOL           IsOverlapped,
    _In_     BOOL           IsExclusive,
    _Out_    PHID_DEVICE    HidDevice
);

BOOLEAN
FindKnownHidDevice(
    OUT PHID_DEVICE HidDevices // A array of struct _HID_DEVICE
);

BOOLEAN
FindKnownHidDevices(
    OUT PHID_DEVICE* HidDevices, // A array of struct _HID_DEVICE
    OUT PULONG        NumberDevices // the length of this array.
);


BOOLEAN
FillDeviceInfo(
    IN  PHID_DEVICE HidDevice
);

VOID
CloseHidDevice(
    IN PHID_DEVICE   HidDevice
);


BOOLEAN
Read(
    PHID_DEVICE    HidDevice
);

BOOLEAN
ReadOverlapped(
    PHID_DEVICE     HidDevice,
    HANDLE          CompletionEvent,
    LPOVERLAPPED    Overlap
);

BOOLEAN
Write(
    PHID_DEVICE    HidDevice
);

BOOLEAN
UnpackReport(
    _In_reads_bytes_(ReportBufferLength)PCHAR ReportBuffer,
    IN       USHORT               ReportBufferLength,
    IN       HIDP_REPORT_TYPE     ReportType,
    IN OUT   PHID_DATA            Data,
    IN       ULONG                DataLength,
    IN       PHIDP_PREPARSED_DATA Ppd
);

BOOLEAN
PackReport(
    _Out_writes_bytes_(ReportBufferLength)PCHAR ReportBuffer,
    IN       USHORT               ReportBufferLength,
    IN       HIDP_REPORT_TYPE     ReportType,
    IN       PHID_DATA            Data,
    IN       ULONG                DataLength,
    IN       PHIDP_PREPARSED_DATA Ppd
);

BOOLEAN
SetFeature(
    PHID_DEVICE    HidDevice
);

BOOLEAN
GetFeature(
    PHID_DEVICE    HidDevice
);

VOID
ReportToString(
    PHID_DATA    pData,
    _Inout_updates_bytes_(iBuffSize) LPSTR szBuff,
    UINT          iBuffSize
);
