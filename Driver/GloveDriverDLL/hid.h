
#pragma warning(disable:4067)
#include <Windows.h>
#include "hidsdi.h"
#include "setupapi.h"
#include <strsafe.h>

#define ASSERT(x)

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

//double qData[] = {
//0.988984	,	-0.098494	,	-0.077762	,	-0.052829,
//0.99	,	-0.09841	,	-0.0584	,	-0.058582		 ,
//0.990536	,	-0.093252	,	-0.057814	,	-0.058545,
//0.989684	,	-0.094458	,	-0.063658	,	-0.064641,
//0.989471	,	-0.100935	,	-0.052436	,	-0.068093,
//0.989519	,	-0.101379	,	-0.032695	,	-0.078304,
//0.989698	,	-0.098344	,	-0.032019	,	-0.080185,
//0.990136	,	-0.093597	,	-0.031983	,	-0.080472,
//0.991476	,	-0.085452	,	-0.019226	,	-0.077017,
//0.991671	,	-0.079127	,	0.001467	,	-0.083374,
//0.991155	,	-0.074061	,	0.014007	,	-0.092505,
//0.990467	,	-0.076147	,	0.01596	,	-0.09772	 ,
//0.990551	,	-0.077974	,	0.013394	,	-0.095797,
//0.99093	,	-0.079116	,	0.004431	,	-0.091688	 ,
//0.991078	,	-0.080744	,	-0.005877	,	-0.088524,
//0.989363	,	-0.08743	,	-0.033472	,	-0.094987,
//0.989207	,	-0.087371	,	-0.045801	,	-0.091463,
//0.989089	,	-0.08653	,	-0.052254	,	-0.090072,
//0.988585	,	-0.089105	,	-0.059961	,	-0.088277,
//0.987941	,	-0.092017	,	-0.067094	,	-0.087356,
//0.988046	,	-0.091832	,	-0.061432	,	-0.090473,
//0.989462	,	-0.09038	,	-0.044108	,	-0.086479,
//0.991417	,	-0.081342	,	-0.015784	,	-0.082778,
//0.991885	,	-0.074996	,	0.001025	,	-0.084643,
//0.991964	,	-0.068971	,	0.011306	,	-0.088033,
//0.991462	,	-0.068314	,	0.020625	,	-0.092397,
//0.991272	,	-0.07079	,	0.017567	,	-0.093208,
//0.991251	,	-0.073165	,	0.008987	,	-0.092817,
//0.990977	,	-0.079533	,	0.000559	,	-0.090922,
//0.99022	,	-0.082339	,	-0.017593	,	-0.094878	 ,
//0.988963	,	-0.086764	,	-0.0374	,	-0.098245	 ,
//0.988326	,	-0.088015	,	-0.048913	,	-0.09849 ,
//0.988207	,	-0.088208	,	-0.053444	,	-0.097144,
//0.98807	,	-0.089066	,	-0.053185	,	-0.097894	 ,
//0.988615	,	-0.089184	,	-0.04151	,	-0.097932,
//0.989802	,	-0.08598	,	-0.021461	,	-0.095214,
//0.990193	,	-0.08263	,	-0.005121	,	-0.096391,
//0.990619	,	-0.078847	,	0.01948	,	-0.09329	 ,
//0.990327	,	-0.074329	,	0.026215	,	-0.098326,
//0.990009	,	-0.073969	,	0.029321	,	-0.10089 ,
//0.990188	,	-0.075023	,	0.023014	,	-0.099989,
//0.990438	,	-0.080406	,	0.004795	,	-0.095764,
//0.989797	,	-0.085101	,	-0.01403	,	-0.097409,
//0.989162	,	-0.085982	,	-0.0276	,	-0.100157	 ,
//0.988742	,	-0.086455	,	-0.039546	,	-0.099887,
//0.988162	,	-0.089021	,	-0.050234	,	-0.09856 ,
//0.987799	,	-0.089198	,	-0.051302	,	-0.101456,
//0.98772	,	-0.090178	,	-0.048303	,	-0.10282	 ,
//0.988813	,	-0.090573	,	-0.027735	,	-0.09951 ,
//0.989189	,	-0.089761	,	-0.01587	,	-0.099118,
//0.989903	,	-0.086917	,	0.001885	,	-0.095708,
//0.990053	,	-0.085338	,	0.013954	,	-0.09458 ,
//0.990006	,	-0.081093	,	0.02283	,	-0.097045	 ,
//0.989914	,	-0.081952	,	0.021435	,	-0.097584,
//0.989785	,	-0.08551	,	0.011131	,	-0.097558,
//0.989048	,	-0.09335	,	-0.008358	,	-0.098118,
//0.988612	,	-0.094558	,	-0.025054	,	-0.098506,
//0.987543	,	-0.097913	,	-0.039647	,	-0.101125,
//0.987297	,	-0.099045	,	-0.049543	,	-0.098022,
//0.987107	,	-0.099549	,	-0.053111	,	-0.097552,
//0.987027	,	-0.100697	,	-0.052988	,	-0.097246,
//0.986988	,	-0.101486	,	-0.042925	,	-0.10168 ,
//0.987965	,	-0.100483	,	-0.02692	,	-0.098648,
//0.98815	,	-0.101112	,	-0.007714	,	-0.099519	 ,
//0.988879	,	-0.09745	,	0.007792	,	-0.095855,
//0.989033	,	-0.093116	,	0.019589	,	-0.096884,
//0.988921	,	-0.093281	,	0.022801	,	-0.097172,
//0.988769	,	-0.093731	,	0.023853	,	-0.098023,
//0.988651	,	-0.096582	,	0.011817	,	-0.098637,
//0.988139	,	-0.102636	,	-0.006938	,	-0.098108,
//0.987358	,	-0.105449	,	-0.024889	,	-0.100064,
//0.987156	,	-0.105607	,	-0.038462	,	-0.097564,
//0.986925	,	-0.107007	,	-0.049191	,	-0.093467,
//0.986828	,	-0.108205	,	-0.051859	,	-0.091657,
//0.986884	,	-0.109321	,	-0.048241	,	-0.091705,
//0.987035	,	-0.109886	,	-0.039365	,	-0.093616,
//0.986989	,	-0.109937	,	-0.037244	,	-0.094903,
//0.988261	,	-0.106463	,	-0.01767	,	-0.091219,
//0.988945	,	-0.104548	,	-0.003617	,	-0.08759 ,
//0.989147	,	-0.103027	,	0.006038	,	-0.08697 ,
//0.989072	,	-0.103876	,	0.005457	,	-0.086854,
//0.989192	,	-0.101897	,	0.006787	,	-0.087736,
//0.988678	,	-0.102794	,	-0.002555	,	-0.092579,
//0.988226	,	-0.104437	,	-0.01549	,	-0.094284,
//0.98749	,	-0.108569	,	-0.029758	,	-0.093901	 ,
//0.986616	,	-0.112413	,	-0.044883	,	-0.09254 ,
//0.986365	,	-0.113281	,	-0.048877	,	-0.092137,
//0.98694	,	-0.112186	,	-0.040891	,	-0.091213	 ,
//0.987217	,	-0.111771	,	-0.032284	,	-0.092169,
//0.987596	,	-0.110868	,	-0.012782	,	-0.093946,
//0.988442	,	-0.106718	,	0.007008	,	-0.090392,
//0.988644	,	-0.100327	,	0.03339	,	-0.089598	 ,
//0.988494	,	-0.09851	,	0.038976	,	-0.09102 ,
//0.988328	,	-0.099218	,	0.036036	,	-0.093231,
//0.988252	,	-0.102058	,	0.027846	,	-0.093779,
//0.988121	,	-0.105363	,	0.017601	,	-0.093984,
//0.986653	,	-0.111664	,	-0.012107	,	-0.10259 ,
//0.986384	,	-0.10991	,	-0.031008	,	-0.103112,
//0.986511	,	-0.109475	,	-0.041889	,	-0.098409,
//0.98713	,	-0.107744	,	-0.042792	,	-0.093609	 ,
//0.988582	,	-0.105268	,	-0.023076	,	-0.087849,
//0.988871	,	-0.104127	,	-0.000153	,	-0.088987,
//0.988874	,	-0.103831	,	0.013719	,	-0.088241,
//0.988546	,	-0.104563	,	0.02482	,	-0.088622	 ,
//0.98851	,	-0.104341	,	0.027736	,	-0.088422	 ,
//0.988527	,	-0.104794	,	0.026272	,	-0.088145,
//0.988704	,	-0.103997	,	0.008948	,	-0.090526,
//0.988757	,	-0.104912	,	-0.003912	,	-0.089246,
//0.988624	,	-0.106411	,	-0.013953	,	-0.087928,
//0.98841	,	-0.107481	,	-0.020552	,	-0.087743	 ,
//0.988222	,	-0.11065	,	-0.026631	,	-0.084212,
//0.987939	,	-0.112676	,	-0.028729	,	-0.084158,
//0.988406	,	-0.110276	,	-0.021731	,	-0.083955,
//0.989728	,	-0.101229	,	-0.001689	,	-0.082553,
//0.98995	,	-0.096878	,	0.021611	,	-0.082294	 ,
//0.99005	,	-0.092134	,	0.028076	,	-0.084562	 ,
//0.990042	,	-0.090862	,	0.028105	,	-0.086012,
//0.989933	,	-0.092255	,	0.025385	,	-0.08663 ,
//0.989218	,	-0.102901	,	0.007758	,	-0.08617 ,
//0.988799	,	-0.104906	,	-0.003543	,	-0.088805,
//0.987887	,	-0.106451	,	-0.023224	,	-0.09399 ,
//0.988162	,	-0.103743	,	-0.031533	,	-0.091683,
//0.988755	,	-0.101159	,	-0.030455	,	-0.088488,
//0.98876	,	-0.100942	,	-0.029139	,	-0.089122	 ,
//0.989468	,	-0.100085	,	-0.015942	,	-0.085493,
//0.989554	,	-0.100239	,	0.000005	,	-0.0858	 ,
//0.989986	,	-0.095292	,	0.02531	,	-0.08266	 ,
//0.9901	,	-0.089136	,	0.037784	,	-0.083409	 ,
//0.989797	,	-0.0873	,	0.040934	,	-0.087359	 ,
//0.989715	,	-0.089145	,	0.037248	,	-0.088081,
//0.989905	,	-0.094307	,	0.019677	,	-0.08622 ,
//0.989306	,	-0.099035	,	-0.001666	,	-0.089944,
//0.98896	,	-0.097129	,	-0.019189	,	-0.093716	 ,
//0.989256	,	-0.094745	,	-0.028681	,	-0.090559,
//0.989415	,	-0.095349	,	-0.031703	,	-0.087112,
//0.989508	,	-0.095839	,	-0.031674	,	-0.085516,
//0.989503	,	-0.098751	,	-0.015492	,	-0.086713,
//0.98977	,	-0.098887	,	0.005547	,	-0.084689	 ,
//0.990127	,	-0.095777	,	0.025635	,	-0.080281,
//0.990051	,	-0.091607	,	0.040919	,	-0.079742,
//0.990041	,	-0.091279	,	0.041637	,	-0.079886,
//0.98995	,	-0.093217	,	0.037438	,	-0.080841	 ,
//0.989901	,	-0.09866	,	0.024299	,	-0.079986,
//0.989268	,	-0.103903	,	0.004336	,	-0.084624,
//0.98873	,	-0.103526	,	-0.01467	,	-0.090036	 ,
//0.988681	,	-0.10153	,	-0.028453	,	-0.089547,
//0.989062	,	-0.100076	,	-0.034023	,	-0.084919,
//0.989125	,	-0.100083	,	-0.032443	,	-0.084797,
//0.988983	,	-0.102147	,	-0.02457	,	-0.086612,
//0.98936	,	-0.101825	,	-0.003903	,	-0.086083	 ,
//0.990025	,	-0.096451	,	0.015685	,	-0.083234,
//0.990402	,	-0.091014	,	0.029094	,	-0.081244,
//0.990516	,	-0.088976	,	0.02953	,	-0.081955	 ,
//0.990335	,	-0.089493	,	0.028945	,	-0.083772,
//0.990026	,	-0.09336	,	0.021576	,	-0.085413,
//0.989641	,	-0.09934	,	0.007407	,	-0.085527,
//0.988418	,	-0.104023	,	-0.020557	,	-0.091715,
//0.987931	,	-0.102468	,	-0.037546	,	-0.093325,
//0.988004	,	-0.10112	,	-0.046323	,	-0.090026,
//0.987984	,	-0.100962	,	-0.049357	,	-0.088804,
//0.988005	,	-0.101691	,	-0.047032	,	-0.088994,
//0.988154	,	-0.104233	,	-0.0333	,	-0.090589	 ,
//0.988666	,	-0.105726	,	-0.01236	,	-0.088515,
//0.9898	,	-0.09958	,	0.010534	,	-0.083031	 ,
//0.990013	,	-0.097041	,	0.019248	,	-0.081944,
//0.989968	,	-0.097805	,	0.017761	,	-0.081915,
//0.989731	,	-0.100827	,	0.011966	,	-0.082167,
//0.989015	,	-0.10492	,	-0.005382	,	-0.086252,
//0.987958	,	-0.106681	,	-0.031091	,	-0.090649,
//0.987447	,	-0.105817	,	-0.049644	,	-0.088954,
//0.987153	,	-0.107643	,	-0.056156	,	-0.086122,
//0.987093	,	-0.108429	,	-0.056184	,	-0.085796,
//0.98699	,	-0.110225	,	-0.051794	,	-0.087443	 ,
//0.987169	,	-0.111509	,	-0.037833	,	-0.090884,
//0.988294	,	-0.109196	,	-0.02147	,	-0.086698,
//0.989471	,	-0.103471	,	-0.000797	,	-0.082862,
//0.989595	,	-0.101801	,	0.007236	,	-0.083146,
//0.989627	,	-0.100302	,	0.010637	,	-0.08421 ,
//0.989538	,	-0.10079	,	0.015527	,	-0.083921,
//0.989308	,	-0.102722	,	0.012964	,	-0.084725,
//0.989023	,	-0.108167	,	0.002805	,	-0.082172,
//0.987941	,	-0.115667	,	-0.013599	,	-0.083877,
//0.986776	,	-0.117618	,	-0.03516	,	-0.088485,
//0.985959	,	-0.116382	,	-0.052636	,	-0.090536,
//0.985291	,	-0.11587	,	-0.064689	,	-0.09066 ,
//0.98526	,	-0.115759	,	-0.06711	,	-0.08937	 ,
//0.985811	,	-0.117349	,	-0.055865	,	-0.088955,
//0.987181	,	-0.116814	,	-0.0322	,	-0.086122	 ,
//0.987618	,	-0.116757	,	-0.009968	,	-0.086634,
//0.987766	,	-0.116644	,	0.001772	,	-0.085654,
//0.987797	,	-0.115933	,	0.009475	,	-0.085755,
//0.987782	,	-0.115352	,	0.011994	,	-0.086394,
//0.987628	,	-0.114465	,	0.002694	,	-0.090048,
//0.987467	,	-0.115288	,	-0.019266	,	-0.088726,
//0.987347	,	-0.115866	,	-0.031379	,	-0.085815,
//0.987289	,	-0.11595	,	-0.036429	,	-0.084363,
//0.987218	,	-0.116622	,	-0.039536	,	-0.082852,
//0.987002	,	-0.117854	,	-0.045886	,	-0.080368,
//0.98673	,	-0.119347	,	-0.049363	,	-0.079445	 ,
//0.987094	,	-0.119001	,	-0.044399	,	-0.078358,
//0.988185	,	-0.115405	,	-0.025541	,	-0.078406,
//0.989324	,	-0.107158	,	-0.001606	,	-0.079869,
//0.989554	,	-0.101052	,	0.006841	,	-0.084573,
//0.989417	,	-0.099495	,	0.011343	,	-0.087482,
//0.989141	,	-0.104049	,	0.004726	,	-0.085898,
//};