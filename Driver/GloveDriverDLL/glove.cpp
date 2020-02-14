#include "openvr_driver.h"
#include "bones.h"
#include "driverlog.h"
#include <thread>
#include <atomic>
#include <chrono>
#include <math.h>

using namespace vr;
using namespace std;

extern "C" {
#include "hid.h"
}

#define DEVICE_NAME "glove"
#define DEVICE_VID  0x1991
#define DEVICE_PID  0x1129

static LIST               PhysicalDeviceList;

static const char* device_manufacturer = "sean";
static const char *device_controller_type = DEVICE_NAME;
static const char* device_model_number = DEVICE_NAME "1";
static const char* device_serial_number = DEVICE_NAME "SN0";
static const char* device_render_model_name = "{" DEVICE_NAME "}/rendermodels/" DEVICE_NAME;
static const char* device_input_profile_path = "{" DEVICE_NAME "}/input/" DEVICE_NAME "_profile.json";

////////////////////////////////////////////////////////////////////////////////////
// RightHandTest : Represents a single device to vrserver
//  As a test, all this device does it sets itself up, and published a fixed 
//  controller pose and alternates between two hand skeleton poses.
////////////////////////////////////////////////////////////////////////////////////
class RightHandTest : public ITrackedDeviceServerDriver
{
public:
    uint32_t m_id;
    DriverPose_t m_pose;
    VRInputComponentHandle_t m_skeleton;
    int m_frame_count;
    bool m_found_hmd;
    atomic<bool> m_active;
    thread m_pose_thread, m_hid_thread;
    PHID_DEVICE gloveHID;
    HID_DEVICE asyncDevice;

    RightHandTest()
        :   m_frame_count(0),
            m_found_hmd(false),
            m_active(false)
    {
        m_id = 0;
        m_pose = { 0 };
        m_skeleton = 0;
        gloveHID = NULL;
    }


    EVRInitError Activate(uint32_t unObjectId) override
    {
        m_id = unObjectId;
        m_pose = { 0 };
        m_pose.poseIsValid = true;
        m_pose.result = TrackingResult_Running_OK;
        m_pose.deviceIsConnected = true;

        m_pose.qRotation.w = 1;
        m_pose.qRotation.x = 0;
        m_pose.qRotation.y = 0;
        m_pose.qRotation.z = 0;

        m_pose.qWorldFromDriverRotation.w = 1;
        m_pose.qWorldFromDriverRotation.x = 0;
        m_pose.qWorldFromDriverRotation.y = 0;
        m_pose.qWorldFromDriverRotation.z = 0;

        m_pose.qDriverFromHeadRotation.w = 1;
        m_pose.qDriverFromHeadRotation.x = 0;
        m_pose.qDriverFromHeadRotation.y = 0;
        m_pose.qDriverFromHeadRotation.z = 0;

        m_pose.vecPosition[0] = 0.5;
        m_pose.vecPosition[1] = -0.5;
        m_pose.vecPosition[2] = -1.0;

        PropertyContainerHandle_t props = VRProperties()->TrackedDeviceToPropertyContainer(m_id);
        VRProperties()->SetStringProperty(props, Prop_SerialNumber_String, device_serial_number);
        VRProperties()->SetStringProperty(props, Prop_ModelNumber_String, device_model_number);
        //VRProperties()->SetStringProperty(props, Prop_RenderModelName_String, device_render_model_name);
        VRProperties()->SetStringProperty(props, Prop_RenderModelName_String, "vr_controller_vive_1_5");
        VRProperties()->SetStringProperty(props, Prop_ManufacturerName_String, device_manufacturer);
        VRProperties()->SetStringProperty(props, Prop_InputProfilePath_String, device_input_profile_path);
        VRProperties()->SetStringProperty(props, Prop_ControllerType_String, device_controller_type);
        VRProperties()->SetInt32Property(props, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);
        VRProperties()->SetInt32Property(props, Prop_DeviceClass_Int32, (int32_t)TrackedDeviceClass_Controller);
        //VRProperties()->SetStringProperty(props, Prop_LegacyInputProfile_String, device_controller_type);

        EVRInputError error = VRDriverInput()->CreateSkeletonComponent(props,
            "/input/skeleton/right", // component_name
            "/skeleton/hand/right", // skeleton_path
            "/pose_raw", // base_pose_path
            VRSkeletalTracking_Partial,
			right_grip_limit_transforms,
			NUM_BONES,
            &m_skeleton);
        m_active = true;
        m_pose_thread = std::thread(&RightHandTest::UpdatePoseThread, this);
        
        DriverLog("Dev] Glove Contorller Activate Start");
        
        return VRInitError_None;
    }

    void Deactivate() override 
    {
        if (m_active)
        {
            m_active = false;
            m_pose_thread.join();
            DriverLog("Dev] Glove Contorller Deactivated");
        }
        CloseHidDevice(&asyncDevice);
        free(gloveHID);
    }

    void EnterStandby() override {}
    void *GetComponent(const char *pchComponentNameAndVersion) override
    {
        DriverLog("GetComponent called for %s", pchComponentNameAndVersion);
        return nullptr;
    }

    void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) override {}
    DriverPose_t GetPose() override { return m_pose; }

    // update the controller position based on tracking system.
    // in this case we just offset from the hmd
    void UpdateControllerPose()
    {
        static int index = 0;

        VRServerDriverHost()->TrackedDevicePoseUpdated(m_id, m_pose, sizeof(DriverPose_t));
    }

    // update the hand skeleton based on hand state.
    // in this case we just alternate between an open hand pose and a fist pose
    void UpdateHandSkeletonPoses()
    {
        VRBoneTransform_t *hand_pose = (m_frame_count % 200 < 100) ? right_open_hand_pose : right_fist_pose;
        VRDriverInput()->UpdateSkeletonComponent(m_skeleton, VRSkeletalMotionRange_WithoutController, hand_pose, NUM_BONES);
        VRDriverInput()->UpdateSkeletonComponent(m_skeleton, VRSkeletalMotionRange_WithController, hand_pose, NUM_BONES);
    }

    void UpdatePoseThread()
    {
        PHID_DEVICE hidList = NULL;
        ULONG numDevices;
        BOOLEAN readAsync;

        if (FindKnownHidDevices(&hidList, &numDevices) == TRUE)
        {
            for (ULONG i = 0; i <= numDevices; i++)
            {
                if ((hidList + i)->Attributes.VendorID == DEVICE_VID && (hidList + i)->Attributes.ProductID == DEVICE_PID) 
                {
                    gloveHID = (PHID_DEVICE)calloc(1, sizeof(HID_DEVICE));
                    if (gloveHID)
                    {
                        memcpy(gloveHID, (hidList + i), sizeof(HID_DEVICE));
                        DriverLog("Dev] HID Device find VID.PID : %04x.%04x", gloveHID->Attributes.VendorID, gloveHID->Attributes.ProductID);
                    }
                    else
                        DriverLog("Dev] HID Device List memory allocation failed");
                }
            }
        }
        else
            DriverLog("Dev] HID Devices Not Found");

        if (hidList)
            free(hidList);

        if (gloveHID)
        {
            readAsync = OpenHidDevice(gloveHID->DevicePath, TRUE, FALSE, TRUE, FALSE, &asyncDevice);
            if (readAsync)
                DriverLog("Dev] Open Device Async Success");
            else {
                DriverLog("Dev] Open Device Async Failed");
                return;
            }

            HANDLE  completionEvent;
            BOOL    readResult;
            DWORD   waitStatus;
            ULONG   numReadsDone;
            OVERLAPPED overlap;
            DWORD      bytesTransferred;

            completionEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

            if (completionEvent)
            {
                DriverLog("Dev] Create CompletionEvent Success");

                numReadsDone = 0;
                
                while (m_active)
                {
                    readResult = ReadOverlapped(&asyncDevice, completionEvent, &overlap);
                    if (readResult)
                    {
                        DriverLog("Dev] Read Device File Success");
                        waitStatus = WaitForSingleObject(completionEvent, 1000);
                        if (WAIT_OBJECT_0 == waitStatus)
                        {
                            DriverLog("Dev] Driver successfully got single USB data object");
                            readResult = GetOverlappedResult(asyncDevice.HidDevice, &overlap, &bytesTransferred, TRUE);

                            if (!m_active)
                                break;

                            numReadsDone++;

                            UnpackReport(asyncDevice.InputReportBuffer,
                                asyncDevice.Caps.InputReportByteLength,
                                HidP_Input,
                                asyncDevice.InputData,
                                asyncDevice.InputDataLength,
                                asyncDevice.Ppd);

                            CHAR        szTempBuff[1024] = { 0 };
                            PHID_DATA   pData = asyncDevice.InputData;
                            UINT        uLoop;

                            for (uLoop = 0; uLoop < asyncDevice.InputDataLength; uLoop++)
                            {
                                ReportToString(pData, szTempBuff, sizeof(szTempBuff));

                                DriverLog("Dev] %s", szTempBuff);

                                pData++;
                            }

                            m_frame_count++;
                            UpdateControllerPose();
                            UpdateHandSkeletonPoses();
                        }
                        else
                            DriverLog("Dev] Driver fail to get single USB data object");
                    }
                    else
                    {
                        DriverLog("Dev] Read Device File Failed");
                        //goto ASYNCREAD_END;
                    }
                    //this_thread::sleep_for(chrono::milliseconds(500));
                }
            }
            else
            {
                DriverLog("Dev] Create CompletionEvent Failed");
            }
        }
    }
};

////////////////////////////////////////////////////////////////////////////////////
// DeviceProvider : Notifies vrserver of the presence of devices
//                  For this test, a single device is assumed to be present and so 
//                  vrserver is notified of it.
////////////////////////////////////////////////////////////////////////////////////
class DeviceProvider : public IServerTrackedDeviceProvider
{
public:
    RightHandTest m_device;
    EVRInitError Init(IVRDriverContext *pDriverContext) override
    {
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
        InitDriverLog(vr::VRDriverLog());
        bool rc = VRServerDriverHost()->TrackedDeviceAdded(device_serial_number, TrackedDeviceClass_Controller, &m_device);
        return VRInitError_None;
    }
    void Cleanup() override {}
    const char * const *GetInterfaceVersions() override
    {
        return k_InterfaceVersions;
    }
    void RunFrame() override {}
    bool ShouldBlockStandbyMode() override { return false; }
    void EnterStandby() override {}
    void LeaveStandby() override {}
} g_device_provider;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

////////////////////////////////////////////////////////////////////////////////////
// HmdDriverFactory : DLL entrypoint called by vrserver
////////////////////////////////////////////////////////////////////////////////////
HMD_DLL_EXPORT void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
{
    if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
    {
        return &g_device_provider;
    }
    DriverLog("HmdDriverFactory called for %s", pInterfaceName);
    return nullptr;
}
