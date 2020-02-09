#include "ServerDriver.h"

ServerDriver* ServerDriver::_instance = nullptr;

ServerDriver::ServerDriver()
{
}

ServerDriver* ServerDriver::get()
{
	if (_instance == nullptr)
		_instance = new ServerDriver();
	return _instance;
}

vr::EVRInitError ServerDriver::Init(vr::IVRDriverContext* driver_context)
{
	if (vr::EVRInitError init_error = vr::InitServerDriverContext(driver_context); init_error != vr::EVRInitError::VRInitError_None) {
		return init_error;
	}
	for (int i = 0; i < 2; i++) {
		_controllers.push_back(DummyController::make_new());
		vr::VRServerDriverHost()->TrackedDeviceAdded(_controllers.back()->get_serial().c_str(), vr::TrackedDeviceClass_Controller, _controllers.back().get());
	}

	return vr::EVRInitError::VRInitError_None;
}

void ServerDriver::Cleanup()
{
}

const char* const* ServerDriver::GetInterfaceVersions()
{
	return vr::k_InterfaceVersions;;
}

void ServerDriver::RunFrame()
{
	for (auto& controller : _controllers) {
		controller->update();
	}

	vr::VREvent_t event;
	while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event))) {
		for (auto& controller : _controllers) {
			if (controller->get_index() == event.trackedDeviceIndex)
				controller->process_event(event);
		}
	}

}

bool ServerDriver::ShouldBlockStandbyMode()
{
	return false;
}

void ServerDriver::EnterStandby()
{
}

void ServerDriver::LeaveStandby()
{
}