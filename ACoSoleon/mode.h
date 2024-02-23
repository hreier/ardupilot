#pragma once

#include "Soleon.h"
#include <AP_Math/chirp.h>
class Parameters;
class ParametersG2;

class GCS_Soleon;

class Mode {

public:

    // Auto Pilot Modes enumeration
    enum class Number : uint8_t {
        STABILIZE =     0,  // manual airframe angle with manual throttle
        ACRO =          1,  // manual body-frame angular rate with manual throttle
        ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        AUTO =          3,  // fully automatic waypoint control using mission commands
        GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        RTL =           6,  // automatic return to launching point
        CIRCLE =        7,  // automatic circular flight with automatic throttle
        LAND =          9,  // automatic landing with horizontal position control
        DRIFT =        11,  // semi-autonomous position, yaw and throttle control
        SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        FLIP =         14,  // automatically flip the vehicle on the roll axis
        AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
        FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
        FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
        ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
        SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
        AUTOROTATE =   26,  // Autonomous autorotation
        AUTO_RTL =     27,  // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
        TURTLE =       28,  // Flip over after crash

        // Mode number 127 reserved for the "drone show mode" in the Skybrush
        // fork at https://github.com/skybrush-io/ardupilot
    };

    // constructor
    Mode(void);

    // do not allow copying
    CLASS_NO_COPY(Mode);

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // child classes should override these methods
    virtual bool init(bool ignore_checks) {
        return true;
    }
    virtual void exit() {};
    virtual void run() = 0;
    virtual bool requires_GPS() const = 0;
    virtual bool has_manual_throttle() const = 0;
    virtual bool allows_arming(AP_Arming::Method method) const = 0;
    virtual bool is_autopilot() const { return false; }
    virtual bool has_user_takeoff(bool must_navigate) const { return false; }
    virtual bool in_guided_mode() const { return false; }
    virtual bool logs_attitude() const { return false; }
    virtual bool allows_save_trim() const { return false; }
    virtual bool allows_autotune() const { return false; }
    virtual bool allows_flip() const { return false; }

    // return a string for this flightmode
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;

    virtual bool is_taking_off() const;
    static void takeoff_stop() { takeoff.stop(); }

    virtual bool is_landing() const { return false; }

    // mode requires terrain to be present to be functional
    virtual bool requires_terrain_failsafe() const { return false; }

    // functions for reporting to GCS
    virtual bool get_wp(Location &loc) const { return false; };
    virtual int32_t wp_bearing() const { return 0; }
    virtual uint32_t wp_distance() const { return 0; }
    virtual float crosstrack_error() const { return 0.0f;}

    // functions to support MAV_CMD_DO_CHANGE_SPEED
    virtual bool set_speed_xy(float speed_xy_cms) {return false;}
    virtual bool set_speed_up(float speed_xy_cms) {return false;}
    virtual bool set_speed_down(float speed_xy_cms) {return false;}

    int32_t get_alt_above_ground_cm(void);

    // pilot input processing
    void get_pilot_desired_lean_angles(float &roll_out_cd, float &pitch_out_cd, float angle_max_cd, float angle_limit_cd) const;
    Vector2f get_pilot_desired_velocity(float vel_max) const;
    float get_pilot_desired_yaw_rate(float yaw_in);
    float get_pilot_desired_throttle() const;

    // returns climb target_rate reduced to avoid obstacles and
    // altitude fence
    float get_avoidance_adjusted_climbrate(float target_rate);

    const Vector3f& get_vel_desired_cms() {
        // note that position control isn't used in every mode, so
        // this may return bogus data:
        return pos_control->get_vel_desired_cms();
    }

    // send output to the motors, can be overridden by subclasses
    virtual void output_to_motors();

    // returns true if pilot's yaw input should be used to adjust vehicle's heading
    virtual bool use_pilot_yaw() const {return true; }

    // pause and resume a mode
    virtual bool pause() { return false; };
    virtual bool resume() { return false; };

    // true if weathervaning is allowed in the current mode
#if WEATHERVANE_ENABLED == ENABLED
    virtual bool allows_weathervaning() const { return false; }
#endif

protected:

    // helper functions
    bool is_disarmed_or_landed() const;
    void zero_throttle_and_relax_ac(bool spool_up = false);
    void zero_throttle_and_hold_attitude();
    void make_safe_ground_handling(bool force_throttle_unlimited = false);

    // functions to control normal landing.  pause_descent is true if vehicle should not descend
    void land_run_horizontal_control();
    void land_run_vertical_control(bool pause_descent = false);
    void land_run_horiz_and_vert_control(bool pause_descent = false) {
        land_run_horizontal_control();
        land_run_vertical_control(pause_descent);
    }

    // run normal or precision landing (if enabled)
    // pause_descent is true if vehicle should not descend
    void land_run_normal_or_precland(bool pause_descent = false);

#if AC_PRECLAND_ENABLED
    // Go towards a position commanded by prec land state machine in order to retry landing
    // The passed in location is expected to be NED and in meters
    void precland_retry_position(const Vector3f &retry_pos);

    // Run precland statemachine. This function should be called from any mode that wants to do precision landing.
    // This handles everything from prec landing, to prec landing failures, to retries and failsafe measures
    void precland_run();
#endif  

    // return expected input throttle setting to hover:
    virtual float throttle_hover() const;

    // Alt_Hold based flight mode states used in Alt_Hold, Loiter, and Sport
    enum AltHoldModeState {
        AltHold_MotorStopped,
        AltHold_Takeoff,
        AltHold_Landed_Ground_Idle,
        AltHold_Landed_Pre_Takeoff,
        AltHold_Flying
    };
    AltHoldModeState get_alt_hold_state(float target_climb_rate_cms);

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AC_WPNav *&wp_nav;
    AC_Loiter *&loiter_nav;
    AC_PosControl *&pos_control;
    AP_InertialNav &inertial_nav;
    AP_AHRS &ahrs;
    MOTOR_CLASS *&motors;
    RC_Channel *&channel_roll;
    RC_Channel *&channel_pitch;
    RC_Channel *&channel_throttle;
    RC_Channel *&channel_yaw;
    float &G_Dt;

    // note that we support two entirely different automatic takeoffs:

    // "user-takeoff", which is available in modes such as ALT_HOLD
    // (see has_user_takeoff method).  "user-takeoff" is a simple
    // reach-altitude-based-on-pilot-input-or-parameter routine.

    // "auto-takeoff" is used by both Guided and Auto, and is
    // basically waypoint navigation with pilot yaw permitted.

    // user-takeoff support; takeoff state is shared across all mode instances
    class _TakeOff {
    public:
        void start(float alt_cm);
        void stop();
        void do_pilot_takeoff(float& pilot_climb_rate);
        bool triggered(float target_climb_rate) const;

        bool running() const { return _running; }
    private:
        bool _running;
        float take_off_start_alt;
        float take_off_complete_alt;
    };

    static _TakeOff takeoff;

    virtual bool do_user_takeoff_start(float takeoff_alt_cm);


public:
    // Navigation Yaw control
    class AutoYaw {

    public:

        // Autopilot Yaw Mode enumeration
        enum class Mode {
            HOLD =             0,  // hold zero yaw rate
            LOOK_AT_NEXT_WP =  1,  // point towards next waypoint (no pilot input accepted)
            ROI =              2,  // point towards a location held in roi (no pilot input accepted)
            FIXED =            3,  // point towards a particular angle (no pilot input accepted)
            LOOK_AHEAD =       4,  // point in the direction the copter is moving
            RESETTOARMEDYAW =  5,  // point towards heading at time motors were armed
            ANGLE_RATE =       6,  // turn at a specified rate from a starting angle
            RATE =             7,  // turn at a specified rate (held in auto_yaw_rate)
            CIRCLE =           8,  // use AC_Circle's provided yaw (used during Loiter-Turns commands)
            PILOT_RATE =       9,  // target rate from pilot stick
            WEATHERVANE =     10,  // yaw into wind
        };

        // mode(): current method of determining desired yaw:
        Mode mode() const { return _mode; }
        void set_mode_to_default(bool rtl);
        void set_mode(Mode new_mode);
        Mode default_mode(bool rtl) const;

        void set_rate(float new_rate_cds);

        // set_roi(...): set a "look at" location:
        void set_roi(const Location &roi_location);

        void set_fixed_yaw(float angle_deg,
                           float turn_rate_dps,
                           int8_t direction,
                           bool relative_angle);

        void set_yaw_angle_rate(float yaw_angle_d, float yaw_rate_ds);

        bool reached_fixed_yaw_target();

#if WEATHERVANE_ENABLED == ENABLED
        void update_weathervane(const int16_t pilot_yaw_cds);
#endif

        AC_AttitudeControl::HeadingCommand get_heading();

    private:

        // yaw_cd(): main product of AutoYaw; the heading:
        float yaw_cd();

        // rate_cds(): desired yaw rate in centidegrees/second:
        float rate_cds();

        float look_ahead_yaw();
        float roi_yaw() const;

        // auto flight mode's yaw mode
        Mode _mode = Mode::LOOK_AT_NEXT_WP;
        Mode _last_mode;

        // Yaw will point at this location if mode is set to Mode::ROI
        Vector3f roi;

        // yaw used for YAW_FIXED yaw_mode
        float _fixed_yaw_offset_cd;

        // Deg/s we should turn
        float _fixed_yaw_slewrate_cds;

        // time of the last yaw update
        uint32_t _last_update_ms;

        // heading when in yaw_look_ahead_yaw
        float _look_ahead_yaw;

        // turn rate (in cds) when auto_yaw_mode is set to AUTO_YAW_RATE
        float _yaw_angle_cd;
        float _yaw_rate_cds;
        float _pilot_yaw_rate_cds;
    };
    static AutoYaw auto_yaw;

    // pass-through functions to reduce code churn on conversion;
    // these are candidates for moving into the Mode base
    // class.
    float get_pilot_desired_climb_rate(float throttle_control);
    float get_non_takeoff_throttle(void);
    void update_simple_mode(void);
    bool set_mode(Mode::Number mode, ModeReason reason);
    void set_land_complete(bool b);
    GCS_Soleon &gcs();
    uint16_t get_pilot_speed_dn(void);
    // end pass-through functions
};

