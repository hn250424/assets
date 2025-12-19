#pragma once

namespace interfaces {

	namespace nodes {
		constexpr const char* const SENSOR = "SensorNode";
		constexpr const char* const DISPLAY = "DisplayNode";
		constexpr const char* const WORKSTATION = "WorkstationNode";
	}

	namespace topics {
		constexpr const char* const UPDATE_SENSOR = "/update/sensor";
	}

	namespace services {
		// constexpr const char* const UPDATE_SENSOR = "/update/sensor";
	}
}
