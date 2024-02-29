/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  Soleon 
 *  Creator:        Hans Reier
 *  Based on code from the Arducopter 
 *
 */

#include "Soleon.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, _interval_ticks, _max_time_micros, _prio) SCHED_TASK_CLASS(Soleon, &soleon, func, _interval_ticks, _max_time_micros, _prio)
#define FAST_TASK(func) FAST_TASK_CLASS(Soleon, &soleon, func)

/*
  scheduler table - all tasks should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table in AP_Vehicle to determine
  the order in which tasks are run.  Convenience methods SCHED_TASK
  and SCHED_TASK_CLASS are provided to build entries in this structure:

SCHED_TASK arguments:
 - name of static function to call
 - rate (in Hertz) at which the function should be called
 - expected time (in MicroSeconds) that the function should take to run
 - priority (0 through 255, lower number meaning higher priority)

SCHED_TASK_CLASS arguments:
 - class name of method to be called
 - instance on which to call the method
 - method to call on that instance
 - rate (in Hertz) at which the method should be called
 - expected time (in MicroSeconds) that the method should take to run
 - priority (0 through 255, lower number meaning higher priority)

 */
const AP_Scheduler::Task Soleon::scheduler_tasks[] = {
    // update INS immediately to get current gyro data populated
    FAST_TASK_CLASS(AP_InertialSensor, &soleon.ins, update),

    // run EKF state estimator (expensive)
    FAST_TASK(read_AHRS),

    // Runs the Soleon Aircontroller thread 
    // It's similar to update_flight_mode in the pilot some kind of 'entry point' for the Soleon Controller
    FAST_TASK(update_soleon_ctrl_mode),
    
    // update home from EKF if necessary
    FAST_TASK(update_home_from_EKF),

#if HAL_MOUNT_ENABLED
    // camera mount's fast update
    FAST_TASK_CLASS(AP_Mount, &soleon.camera_mount, update_fast),
#endif
    FAST_TASK(Log_Video_Stabilisation),

    SCHED_TASK(rc_loop,              250,    130,  3),
    SCHED_TASK(throttle_loop,         50,     75,  6),
    SCHED_TASK_CLASS(AP_GPS,               &soleon.gps,                 update,          50, 200,   9),
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(AP_OpticalFlow,          &soleon.optflow,             update,         200, 160,  12),
#endif
    SCHED_TASK(update_batt_compass,   10,    120, 15),
    SCHED_TASK_CLASS(RC_Channels, (RC_Channels*)&soleon.g2.rc_channels, read_aux_all,    10,  50,  18),
    SCHED_TASK(auto_trim,             10,     75,  30),

#if HAL_PROXIMITY_ENABLED
    SCHED_TASK_CLASS(AP_Proximity,         &soleon.g2.proximity,        update,         200,  50,  36),
#endif
#if AP_BEACON_ENABLED
    SCHED_TASK_CLASS(AP_Beacon,            &soleon.g2.beacon,           update,         400,  50,  39),
#endif
    SCHED_TASK(update_altitude,       10,    100,  42),

    SCHED_TASK_CLASS(SO_TankSupervision,     &soleon.TankSupervision,       update,        10,  90,  54),
    SCHED_TASK(three_hz_loop,          3,     75, 57),
#if AP_SERVORELAYEVENTS_ENABLED
    SCHED_TASK_CLASS(AP_ServoRelayEvents,  &soleon.ServoRelayEvents,      update_events, 50,  75,  60),
#endif
    SCHED_TASK_CLASS(AP_Baro,              &soleon.barometer,             accumulate,    50,  90,  63),

#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(loop_rate_logging, LOOP_RATE,    50,  75),
#endif
    SCHED_TASK(one_hz_loop,            1,    100,  81),
    SCHED_TASK(gpsglitch_check,       10,     50,  90),

    SCHED_TASK_CLASS(GCS,                  (GCS*)&soleon._gcs,          update_receive, 400, 180, 102),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&soleon._gcs,          update_send,    400, 550, 105),
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount,             &soleon.camera_mount,        update,          50,  75, 108),
#endif
#if AP_CAMERA_ENABLED
    SCHED_TASK_CLASS(AP_Camera,            &soleon.camera,              update,          50,  75, 111),
#endif
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350, 114),
    SCHED_TASK(twentyfive_hz_logging, 25,    110, 117),
    SCHED_TASK_CLASS(AP_Logger,            &soleon.logger,              periodic_tasks, 400, 300, 120),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,    &soleon.ins,                 periodic,       400,  50, 123),

    SCHED_TASK_CLASS(AP_Scheduler,         &soleon.scheduler,           update_logging, 0.1,  75, 126),
#if AP_RPM_ENABLED
    SCHED_TASK_CLASS(AP_RPM,               &soleon.rpm_sensor,          update,          40, 200, 129),
#endif
    SCHED_TASK_CLASS(AP_TempCalibration,   &soleon.g2.temp_calibration, update,          10, 100, 135),

#if AP_GRIPPER_ENABLED
    SCHED_TASK_CLASS(AP_Gripper,           &soleon.g2.gripper,          update,          10,  75, 147),
#endif

#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75, 153),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75, 156),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75, 159),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,      3.3,   75, 162),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,     75, 165),
#endif
#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button,            &soleon.button,              update,           5, 100, 168),
#endif
#if STATS_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Stats,             &soleon.g2.stats,            update,           1, 100, 171),
#endif
};

void Soleon::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Soleon::_failsafe_priorities[7];

#if AP_SCRIPTING_ENABLED
// set desired speed (m/s). Used for scripting.
bool Soleon::set_desired_speed(float speed)
{

    wp_nav->set_speed_xy(speed * 100.0f);
    return true;
}

// returns true if the EKF failsafe has triggered.  Only used by Lua scripts
bool Soleon::has_ekf_failsafed() const
{
    return failsafe.ekf;
}

#endif // AP_SCRIPTING_ENABLED

// returns true if vehicle is landing. Only used by Lua scripts
bool Soleon::is_landing() const
{
    return false; //HaRe
}

// returns true if vehicle is taking off. Only used by Lua scripts
bool Soleon::is_taking_off() const
{
    return false; //HaRe
}

bool Soleon::current_mode_requires_mission() const
{
#if MODE_AUTO_ENABLED == ENABLED
        return flightmode == &mode_auto;
#else
        return false;
#endif
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Soleon::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    rc().read_mode_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Soleon::throttle_loop()
{
    // check auto_armed status
    update_auto_armed();

}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Soleon::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if(AP::compass().available()) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors->get_throttle());
        compass.set_voltage(battery.voltage());
        compass.read();
    }
}

// Full rate logging of attitude, rate and pid loops
// should be run at loop rate
void Soleon::loop_rate_logging()
{

}

// ten_hz_logging_loop
// should be run at 10hz
void Soleon::ten_hz_logging_loop()
{

    // log EKF attitude data always at 10Hz unless ATTITUDE_FAST, then do it in the 25Hz loop
    if (!should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        motors->Log_Write();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
        if (rssi.enabled()) {
            logger.Write_RSSI();
        }
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }

    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }

#if FRAME_CONFIG == HELI_FRAME
    Log_Write_Heli();
#endif
#if AP_WINCH_ENABLED
    if (should_log(MASK_LOG_ANY)) {
        g2.winch.write_log();
    }
#endif
#if HAL_MOUNT_ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif
}

// twentyfive_hz_logging - should be run at 25hz
void Soleon::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }

    if (should_log(MASK_LOG_IMU) && !(should_log(MASK_LOG_IMU_FAST))) {
        AP::ins().Write_IMU();
    }

#if MODE_AUTOROTATE_ENABLED == ENABLED
    if (should_log(MASK_LOG_ATTITUDE_MED) || should_log(MASK_LOG_ATTITUDE_FAST)) {
        //update autorotation log
        g2.arot.Log_Write_Autorotation();
    }
#endif
#if HAL_GYROFFT_ENABLED
    if (should_log(MASK_LOG_FTN_FAST)) {
        gyro_fft.write_log_messages();
    }
#endif
}

// three_hz_loop - 3hz loop
void Soleon::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

    // check for deadreckoning failsafe
    failsafe_deadreckon_check();
}

// one_hz_loop - runs at 1Hz
void Soleon::one_hz_loop()
{
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap.value);
    }

    if (!motors->armed()) {
        // check the user hasn't updated the frame class or type
        motors->set_frame_class_and_type((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());

#if FRAME_CONFIG != HELI_FRAME
        // set all throttle channel settings
        motors->update_throttle_range();
#endif
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    AP_Notify::flags.flying = !ap.land_complete;
}

void Soleon::init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Soleon::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (simple_mode == SimpleMode::NONE || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    if (simple_mode == SimpleMode::SIMPLE) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = channel_roll->get_control_in()*simple_cos_yaw - channel_pitch->get_control_in()*simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*simple_sin_yaw + channel_pitch->get_control_in()*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = channel_roll->get_control_in()*super_simple_cos_yaw - channel_pitch->get_control_in()*super_simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*super_simple_sin_yaw + channel_pitch->get_control_in()*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    channel_roll->set_control_in(rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw());
}


void Soleon::read_AHRS(void)
{
    // we tell AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
}

// read baro and log control tuning
void Soleon::update_altitude()
{
    // read in baro altitude
    read_barometer();

    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
        if (!should_log(MASK_LOG_FTN_FAST)) {
            AP::ins().write_notch_log_messages();
#if HAL_GYROFFT_ENABLED
            gyro_fft.write_log_messages();
#endif
        }
    }
}

// vehicle specific waypoint info helpers
bool Soleon::get_wp_distance_m(float &distance) const
{
    return true;
}

// vehicle specific waypoint info helpers
bool Soleon::get_wp_bearing_deg(float &bearing) const
{
    return true;
}

// vehicle specific waypoint info helpers
bool Soleon::get_wp_crosstrack_error_m(float &xtrack_error) const
{
     return true;
}

// get the target earth-frame angular velocities in rad/s (Z-axis component used by some gimbals)
bool Soleon::get_rate_ef_targets(Vector3f& rate_ef_targets) const
{

    return true;
}

/*
  constructor for main Soleon class
 */
Soleon::Soleon(void)
    : logger(g.log_bitmask),
    flight_modes(&g.flight_mode1),
    simple_cos_yaw(1.0f),
    super_simple_cos_yaw(1.0),
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    rc_throttle_control_in_filter(1.0f),
    inertial_nav(ahrs),
    param_loader(var_info),
    soleon_ctrl_mode(&ctrl_disabled)
{
}


Soleon soleon;
AP_Vehicle& vehicle = soleon;

AP_HAL_MAIN_CALLBACKS(&soleon);
