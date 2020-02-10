#pragma once

/*
We wont be using any cryptography, DDE, RPC, etc.. so exclude these from the build process, not necessary but speeds up the build a bit
*/
//#define WIN32_LEAN_AND_MEAN
#include <string>
#include <openvr_driver.h>
//#include <windows.h>

#include "ServerDriver.h"

/*
Obviously we need to include the openvr_driver.h file so we can do the stuff
*/
//#include <openvr_driver.h>
#define HMD_DLL_EXPORT extern "C" __declspec(dllexport) 

/*
Other helpful includes
*/


/// <summary>
/// Driver factory function
/// </summary>
/// <param name="interface_name">Requested interface name</param>
/// <param name="return_code">Return code, 0 if no error</param>
/// <returns>Non-owning pointer to the requested interface</returns>
HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode);