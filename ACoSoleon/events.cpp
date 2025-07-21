#include "Soleon.h"

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */

bool Soleon::failsafe_option(FailsafeOption opt) const
{
    return (g2.fs_options & (uint32_t)opt);
}

void Soleon::failsafe_radio_on_event()
{
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_OCCURRED);

    // set desired action based on FS_THR_ENABLE parameter
    FailsafeAction desired_action;
    switch (g.failsafe_throttle) {
        case FS_THR_DISABLED:
            desired_action = FailsafeAction::NONE;
            break;
        case FS_THR_ENABLED_ALWAYS_RTL:
        case FS_THR_ENABLED_CONTINUE_MISSION:
            desired_action = FailsafeAction::RTL;
            break;
        case FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL:
            desired_action = FailsafeAction::SMARTRTL;
            break;
        case FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND:
            desired_action = FailsafeAction::SMARTRTL_LAND;
            break;
        case FS_THR_ENABLED_ALWAYS_LAND:
            desired_action = FailsafeAction::LAND;
            break;
        case FS_THR_ENABLED_AUTO_RTL_OR_RTL:
            desired_action = FailsafeAction::AUTO_DO_LAND_START;
            break;
        case FS_THR_ENABLED_BRAKE_OR_LAND:
            desired_action = FailsafeAction::BRAKE_LAND;
            break;
        default:
            desired_action = FailsafeAction::LAND;
    }

    // Conditions to deviate from FS_THR_ENABLE selection and send specific GCS warning
    if (((battery.has_failsafed() && battery.get_highest_failsafe_priority() <= FAILSAFE_LAND_PRIORITY))) {
        // Allow landing to continue when battery failsafe requires it (not a user option)
        announce_failsafe("Radio + Battery", "Continuing Landing");
        desired_action = FailsafeAction::LAND;

    } else {
        announce_failsafe("Radio");
    }

    // Call the failsafe action handler
    do_failsafe_action(desired_action, ModeReason::RADIO_FAILSAFE);
}

// failsafe_off_event - respond to radio contact being regained
void Soleon::failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_RESOLVED);
    gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe Cleared");
}

void Soleon::announce_failsafe(const char *type, const char *action_undertaken)
{
    if (action_undertaken != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "%s Failsafe - %s", type, action_undertaken);
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "%s Failsafe", type);
    }
}

void Soleon::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_BATT, LogErrorCode::FAILSAFE_OCCURRED);

    FailsafeAction desired_action = (FailsafeAction)action;

 
    // Battery FS options already use the Failsafe_Options enum. So use them directly.
    do_failsafe_action(desired_action, ModeReason::BATTERY_FAILSAFE);

}

// failsafe_gcs_check - check for ground station failsafe
void Soleon::failsafe_gcs_check()
{
    // Bypass GCS failsafe checks if disabled or GCS never connected
    if (g.failsafe_gcs == FS_GCS_DISABLED) {
        return;
    }

    const uint32_t gcs_last_seen_ms = gcs().sysid_myggcs_last_seen_time_ms();
    if (gcs_last_seen_ms == 0) {
        return;
    }

}

// failsafe_gcs_on_event - actions to take when GCS contact is lost
void Soleon::failsafe_gcs_on_event(void)
{
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_GCS, LogErrorCode::FAILSAFE_OCCURRED);
    RC_Channels::clear_overrides();

    // convert the desired failsafe response to the FailsafeAction enum
    FailsafeAction desired_action;
    switch (g.failsafe_gcs) {
        case FS_GCS_DISABLED:
            desired_action = FailsafeAction::NONE;
            break;
        case FS_GCS_ENABLED_ALWAYS_RTL:
        case FS_GCS_ENABLED_CONTINUE_MISSION:
            desired_action = FailsafeAction::RTL;
            break;
        case FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_RTL:
            desired_action = FailsafeAction::SMARTRTL;
            break;
        case FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_LAND:
            desired_action = FailsafeAction::SMARTRTL_LAND;
            break;
        case FS_GCS_ENABLED_ALWAYS_LAND:
            desired_action = FailsafeAction::LAND;
            break;
        case FS_GCS_ENABLED_AUTO_RTL_OR_RTL:
            desired_action = FailsafeAction::AUTO_DO_LAND_START;
            break;
        case FS_GCS_ENABLED_BRAKE_OR_LAND:
            desired_action = FailsafeAction::BRAKE_LAND;
            break;
        default: // if an invalid parameter value is set, the fallback is RTL
            desired_action = FailsafeAction::RTL;
    }

    // Conditions to deviate from FS_GCS_ENABLE parameter setting
    
        announce_failsafe("GCS");

    // Call the failsafe action handler
    do_failsafe_action(desired_action, ModeReason::GCS_FAILSAFE);
}

// failsafe_gcs_off_event - actions to take when GCS contact is restored
void Soleon::failsafe_gcs_off_event(void)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "GCS Failsafe Cleared");
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_GCS, LogErrorCode::FAILSAFE_RESOLVED);
}

// executes terrain failsafe if data is missing for longer than a few seconds
void Soleon::failsafe_terrain_check()
{

}

// set terrain data status (found or not found)
void Soleon::failsafe_terrain_set_status(bool data_ok)
{
    uint32_t now = millis();

    // record time of first and latest failures (i.e. duration of failures)
    if (!data_ok) {
        failsafe.terrain_last_failure_ms = now;
        if (failsafe.terrain_first_failure_ms == 0) {
            failsafe.terrain_first_failure_ms = now;
        }
    } else {
        // failures cleared after 0.1 seconds of persistent successes
        if (now - failsafe.terrain_last_failure_ms > 100) {
            failsafe.terrain_last_failure_ms = 0;
            failsafe.terrain_first_failure_ms = 0;
        }
    }
}






void Soleon::do_failsafe_action(FailsafeAction action, ModeReason reason){

    // Execute the specified desired_action
    switch (action) {
        case FailsafeAction::NONE:
            return;
        case FailsafeAction::LAND:
            break;
        case FailsafeAction::RTL:
           // set_mode_RTL_or_land_with_pause(reason);
            break;
        case FailsafeAction::SMARTRTL:
           // set_mode_SmartRTL_or_RTL(reason);
            break;
        case FailsafeAction::SMARTRTL_LAND:
          //  set_mode_SmartRTL_or_land_with_pause(reason);
            break;
        case FailsafeAction::TERMINATE: {

            break;
        }
        case FailsafeAction::AUTO_DO_LAND_START:
            //set_mode_auto_do_land_start_or_RTL(reason);
            break;
        case FailsafeAction::BRAKE_LAND:
          //  set_mode_brake_or_land_with_pause(reason);
            break;
    }

#if AP_GRIPPER_ENABLED
    if (failsafe_option(FailsafeOption::RELEASE_GRIPPER)) {
        soleon.g2.gripper.release();
    }
#endif
}

