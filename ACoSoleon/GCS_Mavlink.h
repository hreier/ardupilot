#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Soleon : public GCS_MAVLINK
{

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    uint32_t telem_delay() const override;

    MAV_RESULT handle_flight_termination(const mavlink_command_int_t &packet) override;

    uint8_t sysid_my_gcs() const override;
    bool sysid_enforce() const override;

    bool params_ready() const override;
    void send_banner() override;

    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    void send_attitude_target() override;
    void send_position_target_global_int() override;
    void send_position_target_local_ned() override;

    MAV_RESULT handle_command_do_set_roi(const Location &roi_loc) override;
    MAV_RESULT handle_preflight_reboot(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
#if HAL_MOUNT_ENABLED
    MAV_RESULT handle_command_mount(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
#endif
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);
    MAV_RESULT handle_command_pause_continue(const mavlink_command_int_t &packet);

#if HAL_MOUNT_ENABLED
    void handle_mount_message(const mavlink_message_t &msg) override;
#endif

    void handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) override;
    void handle_copter_hud_msg(const mavlink_message_t &msg, bool log_hud);


    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;
    void send_nav_controller_output() const override;
    uint64_t capabilities() const override;

    virtual MAV_VTOL_STATE vtol_state() const override { return MAV_VTOL_STATE_MC; };
    virtual MAV_LANDED_STATE landed_state() const override;

    void handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow) override;

private:

    // sanity check velocity or acceleration vector components are numbers
    // (e.g. not NaN) and below 1000. vec argument units are in meters/second or
    // metres/second/second
    bool sane_vel_or_acc_vector(const Vector3f &vec) const;

    MISSION_STATE mission_state(const class AP_Mission &mission) const override;

    void    handle_message(const mavlink_message_t &msg) override;
    void send_global_position_int() override {};

    void handle_command_ack(const mavlink_message_t &msg) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    bool try_send_message(enum ap_message id) override;

    void packetReceived(const mavlink_status_t &status,
                        const mavlink_message_t &msg) override;

    MAV_MODE base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    float vfr_hud_airspeed() const override;
    int16_t vfr_hud_throttle() const override;
    float vfr_hud_alt() const override;

    void send_pid_tuning() override;

    void send_winch_status() const override;

    void send_wind() const;
    void send_so_status(void);
    void send_so_tunnel(void);

    typedef union {
      struct{
        //--- 
        uint64_t timestamp;                   ///< Timestamp, in microseconds since UNIX epoch GMT
    
        //--- DEBUG
        float    dbg1Float;
        float    dbg2Float;
        uint32_t dbg3Uint32;
        uint16_t dbg4Uint32;
    
        //--- COUNTERS
        uint32_t cntrCtrLoops;
        uint16_t cntrRxMp;
        uint16_t cntrTxStatus;
    
        //--- MEASURES
        float pressureLeft;
        float pressureRight;
        uint8_t owerRideSw;       //-- forcedOff; MissionPlan; forcedOn
        uint8_t owerRideOnSw;     //-- front; all; back
        float offsetTrim;       //--5% to 5% (l/ha)
    
        //--- CONTROLLER
        uint16_t ppmPumpLeft;
        uint16_t ppmPumpRight;
        uint16_t ppmPumpLeftMax;
        uint16_t ppmPumpRightMax;
        float    eValLeftMax;
        float    eValRightMax;
    
        //--- ERRORS
        uint32_t errorFlags;
        uint16_t cntPressLeftWindups;
        uint16_t cntPressRightWindups;
        uint16_t cntMavLinkErrors;
    
        //--- mavlink_status channel_0
        uint8_t msg_received_0;               ///< Number of received messages
        uint8_t buffer_overrun_0;             ///< Number of buffer overruns
        uint8_t parse_error_0;                ///< Number of parse errors
        uint16_t packet_rx_success_count_0;   ///< Received packets
        uint16_t packet_rx_drop_count_0;      ///< Number of packet drops
    
        //--- mavlink_status channel_1
        uint8_t msg_received_1;               ///< Number of received messages
        uint8_t buffer_overrun_1;             ///< Number of buffer overruns
        uint8_t parse_error_1;                ///< Number of parse errors
        uint16_t packet_rx_success_count_1;   ///< Received packets
        uint16_t packet_rx_drop_count_1;      ///< Number of packet drops
      };
      uint8_t buf[1];
    } so_tunnel_f010_t;

    so_tunnel_f010_t so_tunnel_f010;
    uint8_t tunnel_buf[128];

    bool update_so_tunnel_f010(void); 
    void handle_msg_tunnel(const mavlink_message_t &msg);



#if HAL_HIGH_LATENCY2_ENABLED
    int16_t high_latency_target_altitude() const override;
    uint8_t high_latency_tgt_heading() const override;
    uint16_t high_latency_tgt_dist() const override;
    uint8_t high_latency_tgt_airspeed() const override;
    uint8_t high_latency_wind_speed() const override;
    uint8_t high_latency_wind_direction() const override;
#endif // HAL_HIGH_LATENCY2_ENABLED
};
