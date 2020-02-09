#include "DummyController.h"
#include <cmath>

DummyController::DummyController() :
	_pose({ 0 })
{
	// Create some random but unique serial
	_serial = "glove_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count());

	// Set up some defalt rotation pointing down -z
	_pose.qRotation.w = 1.0;
	_pose.qRotation.x = 0.0;
	_pose.qRotation.y = 0.0;
	_pose.qRotation.z = 0.0;

	_pose.qWorldFromDriverRotation.w = 1.0;
	_pose.qWorldFromDriverRotation.x = 0.0;
	_pose.qWorldFromDriverRotation.y = 0.0;
	_pose.qWorldFromDriverRotation.z = 0.0;

	_pose.qDriverFromHeadRotation.w = 1.0;
	_pose.qDriverFromHeadRotation.x = 0.0;
	_pose.qDriverFromHeadRotation.y = 0.0;
	_pose.qDriverFromHeadRotation.z = 0.0;

	_pose.vecPosition[0] = -1.5;
	_pose.vecPosition[1] = -0.5;
	_pose.vecPosition[2] = -1.0;

	_pose.vecAngularVelocity[0] = 0;
	_pose.vecAngularVelocity[1] = 0;
	_pose.vecAngularVelocity[2] = 0;

	// To ensure no complaints about tracking
	_pose_timestamp = static_cast<std::chrono::milliseconds>(0);
	_pose.poseIsValid = true;
	_pose.result = vr::ETrackingResult::TrackingResult_Running_OK;
	_pose.deviceIsConnected = true;
}

std::shared_ptr<DummyController> DummyController::make_new()
{
	return std::shared_ptr<DummyController>(new DummyController());
}

std::string DummyController::get_serial() const
{
	return _serial;
}

void DummyController::update()
{
	// Update time delta (for working out velocity)
	std::chrono::milliseconds time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	double time_since_epoch_seconds = time_since_epoch.count() / 1000.0;
	double pose_time_delta_seconds = (time_since_epoch - _pose_timestamp).count() / 1000.0;
	double previous_position[3] = { 0 };
	double previous_velocity[3] = { 0 };

	// Update pose timestamp
	if (_pose_timestamp == static_cast<std::chrono::milliseconds>(0))
	{
		_pose.vecPosition[0] = -1.5;
		_pose.vecPosition[1] = -0.5;
		_pose.vecPosition[2] = -1.0;

		goto END_UPDATE;
	}

	// Copy the previous position data
	std::copy(std::begin(_pose.vecPosition), std::end(_pose.vecPosition), std::begin(previous_position));
	//std::copy(std::begin(_pose.vecVelocity), std::end(_pose.vecVelocity), std::begin(previous_velocity));

	// Update the position with our new data
	_pose.vecPosition[0] = -1.5 + static_cast<double>(_index) + 0.3 * std::sin(time_since_epoch_seconds);
	_pose.vecPosition[1] = -0.5;
	_pose.vecPosition[2] = -1.0 + 0.3 * std::cos(time_since_epoch_seconds);

	_pose.qRotation.x = std::sin(time_since_epoch_seconds);
	_pose.qRotation.y = 0;
	_pose.qRotation.z = std::cos(time_since_epoch_seconds);
END_UPDATE:
	_pose_timestamp = time_since_epoch;
	// If we are still tracking, update openvr with our new pose data
	if (_index != vr::k_unTrackedDeviceIndexInvalid)
	{
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(_index, _pose, sizeof(vr::DriverPose_t));
		vr::VRDriverInput()->UpdateSkeletonComponent(_components._skeletal, vr::VRSkeletalMotionRange_WithoutController, gripLimitTransforms, 31);
	}
}

vr::TrackedDeviceIndex_t DummyController::get_index() const
{
	return _index;
}

void DummyController::process_event(const vr::VREvent_t& event)
{
}

vr::EVRInitError DummyController::Activate(vr::TrackedDeviceIndex_t index)
{
	//vr::ETrackedPropertyError e;

	// Save the device index
	_index = index;

	// Get the properties handle for our controller
	_props = vr::VRProperties()->TrackedDeviceToPropertyContainer(_index);

	// Set our universe ID
	vr::VRProperties()->SetUint64Property(_props, vr::Prop_CurrentUniverseId_Uint64, 2);

	// Set our controller to use the vive controller render model
	if (_index % 2)
	{
		vr::VRProperties()->SetStringProperty(_props, vr::Prop_RenderModelName_String, "{glove}/rendermodels/vr_glove_left_model_slim");
		vr::VRDriverInput()->CreateSkeletonComponent(_props, "/input/skeleton/left", "/skeleton/hand/left", "/pose/raw", 
			vr::VRSkeletalTracking_Estimated, gripLimitTransforms, nBoneCount, &_components._skeletal);
	}
	else
	{
		vr::VRProperties()->SetStringProperty(_props, vr::Prop_RenderModelName_String, "{glove}/rendermodels/vr_glove_right_model_slim");
		vr::VRDriverInput()->CreateSkeletonComponent(_props, "/input/skeleton/right", "/skeleton/hand/right", "/pose/raw",
			vr::VRSkeletalTracking_Estimated, gripLimitTransforms, nBoneCount, &_components._skeletal);
	}

	//vr::VRProperties()->SetStringProperty(_props, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");
	//if (e);

	return vr::VRInitError_None;
}

void DummyController::Deactivate()
{
	// Clear device id
	_index = vr::k_unTrackedDeviceIndexInvalid;
}

void DummyController::EnterStandby()
{
}

void* DummyController::GetComponent(const char* component)
{
	// No extra components on this device so always return nullptr
	return nullptr;
}

void DummyController::DebugRequest(const char* request, char* response_buffer, uint32_t response_buffer_size)
{
	// No custom debug requests defined
	if (response_buffer_size >= 1)
		response_buffer[0] = 0;
}

vr::DriverPose_t DummyController::GetPose()
{
	return _pose;
}

void DummyController::set_pose(vr::DriverPose_t new_pose)
{
	_pose = new_pose;
}