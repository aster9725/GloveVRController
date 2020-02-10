#include "driverFactory.h"

//HMD_DLL_EXPORT void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
//{
//	if (0 == _stricmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
//	{
//		return &server_driver;
//	}
//	if (0 == _stricmp(IVRWatchdogProvider_Version, pInterfaceName))
//	{
//		return &watchdog_driver;
//	}
//
//	if (pReturnCode)
//		*pReturnCode = VRInitError_Init_InterfaceNotFound;
//
//	return NULL;
//}

void* HmdDriverFactory(const char* interface_name, int* return_code) {

	if (std::string(interface_name) == std::string(vr::IServerTrackedDeviceProvider_Version)) {
		return ServerDriver::get();
	}

	if (return_code)
		*return_code = vr::VRInitError_Init_InterfaceNotFound;

	return nullptr;
}