#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VEHICLE_MODE_NONE = 0,
    VEHICLE_MODE_STOPPED,      
    VEHICLE_MODE_FORWARD,
    VEHICLE_MODE_REVERSE
} VehicleMode_t;

typedef enum{
	CONTROLLER_STATE_NONE = 0,
    CONTROLLER_STATE_WAIT,
	CONTROLLER_STATE_IDLE,
	CONTROLLER_STATE_KEEP,
	CONTROLLER_STATE_ENTRY,
	CONTROLLER_STATE_CURVE
}ControllerState_t;

#ifdef __cplusplus
}
#endif