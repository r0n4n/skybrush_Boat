#include "Rover.h"

#include <skybrush/colors.h>

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * Implementation of drone show flight mode
 */

bool AC_DroneShowManager_Copter::get_current_location(Location& loc) const
{
    loc = rover.current_loc; 
    return rover.have_position;
}

bool AC_DroneShowManager_Copter::get_current_relative_position_NED_origin(Vector3f& vec) const
{
    //return rover.ahrs.get_relative_position_NED_origin(vec);
    rover.current_loc.get_vector_from_origin_NEU(vec);
    return rover.have_position;
}

void AC_DroneShowManager_Copter::_request_switch_to_show_mode()
{
    // Drone show manager requested the copter to switch to show mode. We do this
    // only if the motors are not armed.
    if (!rover.arming.is_armed()) {
        rover.set_mode(Mode::Number::DRONE_SHOW, ModeReason::SCRIPTING);
    }
};

// Constructor.
ModeDroneShow::ModeDroneShow(void) : Mode(),
    _stage(DroneShow_Off),
    _last_home_position_reset_attempt_at(0),
    _last_stage_change_at(0)
{
}

bool ModeDroneShow::_enter()
{
    initialization_start();
    return true;
}

void ModeDroneShow::_exit()
{
    // Clear the timestamp when we last attempted to arm the drone
    _prevent_arming_until_msec = 0;

    // Clear all the status information that depends on the start time
    notify_start_time_changed();

    // Set the stage to "off"
    _set_stage(DroneShow_Off);

    // Notify the drone show manager that the drone show mode exited
    rover.g2.drone_show_manager.notify_drone_show_mode_exited();
}


bool ModeDroneShow::cancel_requested() const
{
    return rover.g2.drone_show_manager.cancel_requested();
}



int32_t ModeDroneShow::get_default_yaw_cd() const
{
    // return rover.initial_armed_bearing;
    return 0; // return 0 for the moment
}

int32_t ModeDroneShow::get_elapsed_time_since_last_home_position_reset_attempt_msec() const
{
    return AP_HAL::millis() - _last_home_position_reset_attempt_at;
}

int32_t ModeDroneShow::get_elapsed_time_since_last_stage_change_msec() const
{
    return AP_HAL::millis() - _last_stage_change_at;
}

// ModeDroneShow::run - runs the main drone show controller
// should be called at 25hz or more. This function is actually running at
// 400 Hz
void ModeDroneShow::update()
{
    check_changes_in_parameters();
    // call the correct auto controller
    switch (_stage) {

    case DroneShow_Init:
        // mode has just been initialized
        initialization_run();
        break;

    case DroneShow_WaitForStartTime:
        // waiting for start time
        wait_for_start_time_run();
        break;

    case DroneShow_Performing:
        // performing show
        performing_run();
        break;

    case DroneShow_RTL:
        // returning to home position
        // - either due to abnormal termination
        // - or because the show is over and the post-show action is RTL or "RTL or land"
        rtl_run();
        break;

    case DroneShow_Loiter:
        // holding position
        // - either because joined the show while airborne
        // - or because the show is over and the post-show action is "loiter"
        loiter_run();
        break;

    case DroneShow_Error:
        // failed to start a show
        error_run();
        break;
    /*    
    case DroneShow_TestingLights:
        // testing light program without starting motors
        light_testing_run();
        break;
    */
    default:
        break;
    }
}

// Checks changes in relevant parameter values and reports them to the console
void ModeDroneShow::check_changes_in_parameters()
{
    static DroneShowAuthorization last_seen_authorization;
    static uint64_t last_seen_start_time;
    DroneShowAuthorization current_authorization;
    uint64_t current_start_time;

    current_start_time = rover.g2.drone_show_manager.get_start_time_epoch_undefined();
    current_authorization = rover.g2.drone_show_manager.get_authorization_scope();

    if (current_start_time != last_seen_start_time) {
        last_seen_start_time = current_start_time;
        notify_start_time_changed();
    }

    if (last_seen_authorization != current_authorization) {
        last_seen_authorization = current_authorization;
        notify_authorization_changed();
    }
}


float ModeDroneShow::crosstrack_error() const
{
    switch (_stage) {
    case DroneShow_Performing:
        return rover.mode_guided.crosstrack_error();
    case DroneShow_Loiter:
        return rover.mode_loiter.crosstrack_error();
    case DroneShow_RTL:
        return rover.mode_rtl.crosstrack_error();
    default:
        return false;
    }
}

// starts the initialization phase of the drone
void ModeDroneShow::initialization_start()
{
    // Set the appropriate stage
    _set_stage(DroneShow_Init);

    // Assume normal operation: we will start performing after the takeoff
    _next_stage_after_takeoff = DroneShow_Performing;

    // Clear the timestamp when we last attempted to arm the drone
    _prevent_arming_until_msec = 0;

    // This is copied from ModeAuto::init()

    // initialise waypoint and spline controller
    //// wp_nav->wp_and_spline_init();

    // Clear the limits of the guided mode; we will use guided mode internally
    // to control the show
    rover.mode_guided.limit_clear();

    // Set auto-yaw mode to HOLD -- we don't want the drone to start turning
    // towards waypoints, but we don't have a fixed heading at this point where
    // we could force the drone to.
    //// auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // Part from ModeAuto::init() ends here

    // Clear all the status information that depends on the start time
    notify_start_time_changed();

    // Notify the drone show manager that the drone show mode was initialized
    rover.g2.drone_show_manager.notify_drone_show_mode_initialized();
}

// initializes the drone show mode after it has been activated the first time
void ModeDroneShow::initialization_run()
{
    // for the moment no conditions to wait for the start time
    wait_for_start_time_start();
    // loiter until we start
    // loiter_start();
}

// starts the phase where we are waiting for the start time of the show
void ModeDroneShow::wait_for_start_time_start()
{
    _set_stage(DroneShow_WaitForStartTime);

    // Reset home position to current location
    try_to_update_home_position();
}

// waits for the start time of the show
void ModeDroneShow::wait_for_start_time_run()
{
    AC_DroneShowManager_Copter& show_manager = rover.g2.drone_show_manager;
    float time_until_takeoff_sec = show_manager.get_time_until_takeoff_sec();
    float time_since_takeoff_sec = -time_until_takeoff_sec;
    const float latest_takeoff_attempt_after_scheduled_takeoff_time_in_seconds = 5.0f;

    // Drone is in standby so keep all I terms in controllers at zero
    ////attitude_control->reset_yaw_target_and_rate();
    ////attitude_control->reset_rate_controller_I_terms();
    ////pos_control->standby_xyz_reset();

    // Force attitude controller to zero target angles and yaw rate in case it
    // received something else from somewhere before we switched to this mode
    ////attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

    // This is copied from ModeStabilize::run() -- it is needed to allow the 
    // user to turn on the motors and spin them up while idling on the ground.
    // The part that allows unlimited throttle is removed; we allow unlimited
    // throttle only if we somehow ended up in the air for some strange reason
    /*if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (!copter.ap.land_complete) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }*/

    if (time_since_takeoff_sec > latest_takeoff_attempt_after_scheduled_takeoff_time_in_seconds + 1) {
        loiter_start();
    } else {
        if (time_until_takeoff_sec <= 10) {
            /*
            if (!_preflight_calibration_done) {
                // We calibrate the barometer 10 seconds before our takeoff time.
                //
                // Preflight calibration does not hurt anyone so we don't need the
                // takeoff authorization for this

                // This is copied from GCS_MAVLINK::_handle_command_preflight_calibration_baro()
                AP::baro().update_calibration();

                _preflight_calibration_done = true;
            }*/

            if (!_home_position_set) {
                // Update our home to the current location so we have zero AGL
                if (!try_to_update_home_position()) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Could not set home position, giving up");
                    AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_INITIALISE);
                    error_start();
                } else {
                    _home_position_set = true;
                }
            }
        } else {
            // We still have plenty of time until takeoff so note that we haven't
            // done the preflight calibration and haven't set the home position.
            _preflight_calibration_done = false;
            _home_position_set = false;
        }

        // once it's time to start 
        if (time_until_takeoff_sec <= 0 && _home_position_set) {
                // Time to start!
                performing_start();
        }
    }
}


// starts the phase where we are actually performing the show
void ModeDroneShow::performing_start()
{
    _set_stage(DroneShow_Performing);

    // call regular guided flight mode initialisation
    rover.mode_guided._enter();

    // initialise guided start time and position as reference for limit checking
    rover.mode_guided.limit_init_time_and_location();
}

// executes the show performance
void ModeDroneShow::performing_run()
{
    static uint32_t last_guided_command = 0;
    bool exited_mode = 0;
    uint32_t now = AP_HAL::millis();
    uint32_t target_dt = rover.g2.drone_show_manager.get_controller_update_delta_msec();

    if (now - last_guided_command >= target_dt) {
        if (!send_guided_mode_command_during_performance()) {
            // Failed to send guided mode command; try to switch to position
            // hold instead. This should not happen anyway.
            gcs().send_text(MAV_SEVERITY_ERROR, "Failed to send guided mode command");
            loiter_start();
            exited_mode = 1;
        }
        last_guided_command = now;
    }

    // call regular guided flight mode run function
    if (!exited_mode) {
        rover.mode_guided.update();
    }

    if (cancel_requested()) {
        // if a cancellation was requested, loiter until another command is sent
        loiter_start();
    } else if (!rover.arming.is_armed()) {
        
        // if the motors are not armed any more, something is wrong so move to the
        // error stage. This typically happens if we crash during a show.
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Motors disarmed during show");
        error_start();
    } else if (performing_completed()) {
        // if we have finished the show, check the configured post-show action
        // and switch to RTL, position hold or land
        switch (rover.g2.drone_show_manager.get_action_at_end_of_show()) {
            case PostAction_RTL:
                rtl_start();
                break;
            case PostAction_Loiter:
                loiter_start();
                break;
            default:
                // This should not happen but let's be defensive. Safest is to
                // land in place, and it is consistent with legacy behaviour
                gcs().send_text(MAV_SEVERITY_WARNING, "Invalid post-show action, landing in place");
                loiter_start();
                break;
        }
    }
}

bool ModeDroneShow::performing_completed() const
{
    // TODO(ntamas): what if we are late and we are not at the designated landing
    // position yet?
    return rover.g2.drone_show_manager.get_time_until_landing_sec() <= 0;
}


// starts the phase where we are returning to our home position, used during
// aborted shows or when the show trajectory has ended and the post-show action
// is set up to "RTL" or "RTL or land"
void ModeDroneShow::rtl_start()
{
    _set_stage(DroneShow_RTL);

    // call regular RTL flight mode initialisation and ask it to ignore checks
    rover.mode_rtl._enter();
}

// performs the return to landing position stage
void ModeDroneShow::rtl_run()
{
    // call regular rtl flight mode run function
    rover.mode_rtl.update();

    // if we have finished landing, move to the "landed" state
    if (rtl_completed()) {
        loiter_start();
    }
}

// returns whether the RTL operation has finished successfully. Must be called
// from the RTL stage only.
bool ModeDroneShow::rtl_completed() const
{
    if (_stage == DroneShow_RTL) {
        return (
            rover.mode_rtl.reached_destination() 
        );
    } else {
        return false;
    }
}

// starts the phase where we are holding our position indefinitely; this happens
// when we exited show mode and then entered it again while in the air
void ModeDroneShow::loiter_start()
{
    _set_stage(DroneShow_Loiter);

    // call regular position hold nav mode initialisation
    rover.mode_loiter.enter();
}

// performs the phase where we are holding our position indefinitely; this happens
// when we exited show mode and then entered it again while in the air
void ModeDroneShow::loiter_run()
{
    // call regular position hold flight mode run function
    rover.mode_loiter.update();
}

// starts the error phase where we have failed to start a show and we do nothing any more
void ModeDroneShow::error_start()
{
    _set_stage(DroneShow_Error);
}

// performs the error stage where we do nothing any more
void ModeDroneShow::error_run()
{
    // Ensure that we stay disarmed even if someone tries to arm us remotely
    if (AP::arming().is_armed()) {
        AP::arming().disarm(AP_Arming::Method::SCRIPTING);
    }
}

// starts the light testing phase on the ground
/*void ModeDroneShow::light_testing_start()
{
    _set_stage(DroneShow_TestingLights);
}*/

// performs the light testing stage where we do nothing any more except waiting
// for the light program to finish
/*void ModeDroneShow::light_testing_run()
{
    // Ensure that we stay disarmed even if someone tries to arm us remotely
    if (AP::arming().is_armed()) {
        AP::arming().disarm(AP_Arming::Method::SCRIPTING);
    }

    if (!rover.g2.drone_show_manager.has_authorization()) {
        initialization_start();
    } else if (light_testing_completed()) {
        landed_start();
    }
}*/

// returns whether we should exit the light testing mode
/*bool ModeDroneShow::light_testing_completed() const
{
    return copter.g2.drone_show_manager.get_time_until_landing_sec() <= 0;
}*/

// Handler function that is called when the authorization state of the show has
// changed in the drone show manager
void ModeDroneShow::notify_authorization_changed()
{
    if (
        _stage == DroneShow_WaitForStartTime &&
        rover.g2.drone_show_manager.has_authorization()
    ) {
        // Update home position and reset AGL to zero when the show is
        // authorized and we are in the "waiting for start time" phase
        try_to_update_home_position();
    }
}

// Handler function that is called when the start time of the show has changed in the
// drone show manager
void ModeDroneShow::notify_start_time_changed()
{
    // Clear whether the preflight calibration was performed
    _preflight_calibration_done = false;

    // Clear whether the home position was set before takeoff
    _home_position_set = false;
}

// Sends a guided mode command during the show performance, calculated from the
// trajectory that the drone should follow
bool ModeDroneShow::send_guided_mode_command_during_performance()
{
    AC_DroneShowManager::GuidedModeCommand command;
    Location desired_destination;

    if (rover.g2.drone_show_manager.get_current_guided_mode_command_to_send(
        command, get_default_yaw_cd(),
        false // no altitude limitation
    )) {   
        desired_destination.lat = command.pos[0] ; // TBD
        desired_destination.lng = command.pos[1] ; // TBD

        // set desired speed in m/s
        rover.mode_guided.set_desired_speed(command.vel[0]) ;
        rover.mode_guided.set_desired_location(desired_destination);


        rover.g2.drone_show_manager.notify_guided_mode_command_sent(command);

        return true;
    } else {
        return false;
    }
}

// Starts the motors before the show if they are not running already, irrespectively
// of whether the drone is ready to perform the show or not.
bool ModeDroneShow::start_motors_if_not_running()
{
    bool success = false;

    if (AP::arming().is_armed()) {
        // Already armed
        success = true;
    } else if (_prevent_arming_until_msec > AP_HAL::millis()) {
        // Arming prevented because we have tried it recently
    } else if (AP::arming().arm(AP_Arming::Method::SCRIPTING, /* do_arming_checks = */ true)) {
        // Started motors successfully
        success = true;
    } else {
        // Prearm checks failed; prevent another attempt for the next second
        _prevent_arming_until_msec = AP_HAL::millis() + 1000;
    }

    return success;
}

// Starts the motors before the show if they are not running already, after
// checking whether the drone is prepared to take off (according to the
// show manager)
bool ModeDroneShow::try_to_start_motors_if_prepared_to_take_off()
{
    return rover.g2.drone_show_manager.is_prepared_to_take_off() && start_motors_if_not_running();
}

// Tries to update the home position of the drone to its current location
bool ModeDroneShow::try_to_update_home_position()
{
    _last_home_position_reset_attempt_at = AP_HAL::millis();

    return rover.set_home_to_current_location(/* lock = */ false);
}

// Sets the stage of the drone show module and synchronizes it with the DroneShowManager
void ModeDroneShow::_set_stage(DroneShowModeStage value)
{
    _stage = value;
    _last_stage_change_at = AP_HAL::millis();

    _altitude_locked_above_takeoff_altitude = (_stage == DroneShowModeStage::DroneShow_Performing);

    rover.g2.drone_show_manager.notify_drone_show_mode_entered_stage(_stage);
}

#else

#error "You need to enable guided mode support to use the drone show mode."

#endif
