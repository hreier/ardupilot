#include "Soleon.h"
#include <AP_ESC_Telem/AP_ESC_Telem.h>

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

static void failsafe_check_static()
{
    soleon.failsafe_check();
}

void Soleon::init_ardupilot()
{

#if STATS_ENABLED == ENABLED
    // initialise stats module
    g2.stats.init();
#endif

    BoardConfig.init();
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    can_mgr.init();
#endif

    // init cargo gripper
#if AP_GRIPPER_ENABLED
    g2.gripper.init();
#endif

    // init winch
#if AP_WINCH_ENABLED
    g2.winch.init();
#endif

    // initialise notify system
    notify.init();
    notify_flight_mode();

    // initialise battery monitor
    battery.init();

    // Init RSSI
    rssi.init();
    
    barometer.init();

    // setup telem slots with serial ports
    gcs().setup_uarts();



#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    init_rc_in();               // sets up rc channels from radio

    // initialise surface to be tracked in SurfaceTracking
    // must be before rc init to not override initial switch position
    // allocate the motors class
    allocate_motors();

    // initialise rc channels including setting mode
   // rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM_AIRMODE);
    rc().init();

    // sets up motors and output to escs
    init_rc_out();

    // motors initialised so parameters can be sent
    ap.initialised_params = true;

#if AP_RELAY_ENABLED
    relay.init();
#endif

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Do GPS init
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if AP_AIRSPEED_ENABLED
    airspeed.set_log_bit(MASK_LOG_IMU);
#endif

#if AP_OPTICALFLOW_ENABLED
    // initialise optical flow sensor
    optflow.init(MASK_LOG_OPTFLOW);
#endif      // AP_OPTICALFLOW_ENABLED

#if HAL_MOUNT_ENABLED
    // initialise camera mount
    camera_mount.init();
#endif

#if AP_CAMERA_ENABLED
    // initialise camera
    camera.init();
#endif

#if AC_PRECLAND_ENABLED
    // initialise precision landing
    init_precland();
#endif

#if AP_LANDINGGEAR_ENABLED
    // initialise landing gear position
    landinggear.init();
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

    // read Baro pressure at ground
    //-----------------------------
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

    // initialise the scale module
    g2.so_scale.init();
    
    // initialize the pressure module
    g2.so_press.init();
    
#if HAL_PROXIMITY_ENABLED
    // init proximity sensor
    g2.proximity.init();
#endif

#if AP_BEACON_ENABLED
    // init beacons used for non-gps position estimation
    g2.beacon.init();
#endif

#if AP_RPM_ENABLED
    // initialise AP_RPM library
    rpm_sensor.init();
#endif

#if MODE_AUTO_ENABLED == ENABLED
    // initialise mission library
    mode_auto.mission.init();
#endif

#if MODE_SMARTRTL_ENABLED == ENABLED
    // initialize SmartRTL
    g2.smart_rtl.init();
#endif

    // initialise AP_Logger library
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&soleon, &Soleon::Log_Write_Vehicle_Startup_Messages, void));

    startup_INS_ground();

#if AP_SCRIPTING_ENABLED
    g2.scripting.init();
#endif // AP_SCRIPTING_ENABLED

#if AC_CUSTOMCONTROL_MULTI_ENABLED == ENABLED
    custom_control.init();
#endif

    // enable CPU failsafe
    failsafe_enable();

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

 //   motors->output_min();  // output lowest possible value to motors

    // attempt to set the intial_mode, else set to STABILIZE
    if (!set_mode((enum Mode::Number)g.so_controlmode.get(), ModeReason::INITIALISED)) {
        // set mode to STABILIZE will trigger mode change notification to pilot
        set_mode(Mode::Number::CTRL_DISABLED, ModeReason::UNAVAILABLE);
    }
 //   TankSupervision.init(true);
    
    // flag that initialisation has completed
    ap.initialised = true;
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void Soleon::startup_INS_ground()
{
    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::COPTER);

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());

    // reset ahrs including gyro bias
    ahrs.reset();
}

// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Soleon::position_ok() const
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return (ekf_has_absolute_position() || ekf_has_relative_position());
}

// ekf_has_absolute_position - returns true if the EKF can provide an absolute WGS-84 position estimate
bool Soleon::ekf_has_absolute_position() const
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

   return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode); 
}

// ekf_has_relative_position - returns true if the EKF can provide a position estimate relative to it's starting position
bool Soleon::ekf_has_relative_position() const
{
    // return immediately if EKF not used
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // return immediately if neither optflow nor visual odometry is enabled and dead reckoning is inactive
    bool enabled = false;
#if AP_OPTICALFLOW_ENABLED
    if (optflow.enabled()) {
        enabled = true;
    }
#endif
#if HAL_VISUALODOM_ENABLED
    if (visual_odom.enabled()) {
        enabled = true;
    }
#endif
    if (dead_reckoning.active && !dead_reckoning.timeout) {
        enabled = true;
    }
    if (!enabled) {
        return false;
    }

    // get filter status from EKF
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal relative position
    // if (!motors->armed()) {
    //     return (filt_status.flags.pred_horiz_pos_rel);
    // } else {
    //     return (filt_status.flags.horiz_pos_rel && !filt_status.flags.const_pos_mode);
    // }

    return (filt_status.flags.horiz_pos_rel && !filt_status.flags.const_pos_mode);
}

// returns true if the ekf has a good altitude estimate (required for modes which do AltHold)
bool Soleon::ekf_alt_ok() const
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow alt control with only dcm
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // require both vertical velocity and position
    return (filt_status.flags.vert_vel && filt_status.flags.vert_pos);
}

// update_auto_armed - update status of auto_armed flag
void Soleon::update_auto_armed()
{

}

/*
  should we log a message type now?
 */
bool Soleon::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    ap.logging_started = logger.logging_started();
    return logger.should_log(mask);
#else
    return false;
#endif
}

/*
  allocate the motors class
 */
void Soleon::allocate_motors(void)
{
/*     switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
#if FRAME_CONFIG != HELI_FRAME
        case AP_Motors::MOTOR_FRAME_QUAD:
        case AP_Motors::MOTOR_FRAME_HEXA:
        case AP_Motors::MOTOR_FRAME_Y6:
        case AP_Motors::MOTOR_FRAME_OCTA:
        case AP_Motors::MOTOR_FRAME_OCTAQUAD:
        case AP_Motors::MOTOR_FRAME_DODECAHEXA:
        case AP_Motors::MOTOR_FRAME_DECA:
        case AP_Motors::MOTOR_FRAME_SCRIPTING_MATRIX:
        default:
            motors = new AP_MotorsMatrix(soleon.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_TRI:
            motors = new AP_MotorsTri(soleon.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsTri::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_TRICOPTER);
            break;
        case AP_Motors::MOTOR_FRAME_SINGLE:
            motors = new AP_MotorsSingle(soleon.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsSingle::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_COAX:
            motors = new AP_MotorsCoax(soleon.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsCoax::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_TAILSITTER:
            motors = new AP_MotorsTailsitter(soleon.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsTailsitter::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_6DOF_SCRIPTING:
#if AP_SCRIPTING_ENABLED
            motors = new AP_MotorsMatrix_6DoF_Scripting(soleon.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix_6DoF_Scripting::var_info;
#endif // AP_SCRIPTING_ENABLED
            break;
        case AP_Motors::MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX:
#if AP_SCRIPTING_ENABLED
            motors = new AP_MotorsMatrix_Scripting_Dynamic(soleon.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix_Scripting_Dynamic::var_info;
#endif // AP_SCRIPTING_ENABLED
            break;
#else // FRAME_CONFIG == HELI_FRAME
        case AP_Motors::MOTOR_FRAME_HELI_DUAL:
            motors = new AP_MotorsHeli_Dual(soleon.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Dual::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;

        case AP_Motors::MOTOR_FRAME_HELI_QUAD:
            motors = new AP_MotorsHeli_Quad(soleon.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Quad::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;
            
        case AP_Motors::MOTOR_FRAME_HELI:
        default:
            motors = new AP_MotorsHeli_Single(soleon.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Single::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;
#endif
    }
    if (motors == nullptr) {
        AP_BoardConfig::allocation_error("FRAME_CLASS=%u", (unsigned)g2.frame_class.get());
    }
    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    ahrs_view = ahrs.create_view(ROTATION_NONE);
    if (ahrs_view == nullptr) {
        AP_BoardConfig::allocation_error("AP_AHRS_View");
    }

    // brushed 16kHz defaults to 16kHz pulses
    if (motors->is_brushed_pwm_type()) {
        g.rc_speed.set_default(16000);
    }
    
    // upgrade parameters. This must be done after allocating the objects
    convert_pid_parameters();
#if FRAME_CONFIG == HELI_FRAME
    convert_tradheli_parameters();
#endif

#if HAL_PROXIMITY_ENABLED
    // convert PRX to PRX1_ parameters
    convert_prx_parameters();
#endif

    // param count could have changed
    AP_Param::invalidate_count(); */
}

bool Soleon::is_tradheli() const
{
#if FRAME_CONFIG == HELI_FRAME
    return true;
#else
    return false;
#endif
}
