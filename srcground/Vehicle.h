#ifndef LIBS_MAVIO_INCLUDE_VEHICLE_H_
#define LIBS_MAVIO_INCLUDE_VEHICLE_H_

#include <string>

#include "timelib.h"

#include "MAVLinkLib.h"
#include "MAVLinkSMSChannel.h"
#include "MAVLinkISBDChannel.h"
#include "MAVLinkTCPChannel.h"

namespace mavio {

class Vehicle {

    public:

    Vehicle();
    ~Vehicle();

    bool init(int instance);

    bool initialized();

    bool pull_queue_mavmsg(mavlink_message_t& msg);
    
    bool pull_queue_smsmsg(mavio::SMSmessage& msg);
    
    bool pull_queue_sbdmsg(mavio::SBDmessage& msg);

    bool update();

    enum enum_vehicle_act_chan {
        WIFI_ACTIVE = 0,
        RFD_ACTIVE,
        GSM_ACTIVE,
        SBD_ACTIVE
    };

    enum_vehicle_act_chan  get_active_chan() const;
    bool                   set_active_chan(enum_vehicle_act_chan active_chan);
    int                    get_mavid();
    void                   process_gcs_message(mavlink_message_t& msg);
    void                   process_message(mavlink_message_t& msg);
    void                   process_message(mavio::SMSmessage& sms);
    void                   process_message(mavio::SBDmessage& msg);

    bool                   pull_queue_tcp_cli(mavlink_message_t& msg);

    private:

    void send_heartbeat_sms(std::string number);

    void send_high_lat_heartbeats();

    void send_heartbeat_tcp();

    void update_active_channel();

    bool set_wifi_active();

    bool set_rfd_active();

    bool set_gsm_active();

    bool set_isbd_active();

    void handle_wifi_out();

    void handle_rfd_out();

    void handle_gsm_out();

    // Buffers 

    CircularBuffer<mavlink_message_t> _outgoing_queue_gcsmavmsg;
    CircularBuffer<mavio::SMSmessage> _outgoing_queue_smsmsg;
    CircularBuffer<mavio::SBDmessage> _outgoing_queue_sbdmsg;


    // init channel is wifi. If not available it will change
    // to the next priority ones
    enum_vehicle_act_chan _active_channel = WIFI_ACTIVE;

    bool rfd_active_message = false;
    bool gsm_active_message = false;
    bool isbd_active_message = false;
    bool wifi_active_message = false;

    // state machine parameters

    std::string last_aircraft_number = "";
    bool gsm_rehook_sent = false;

    // Groundpi hardware parameters
    
    bool _isbd_initialized = false;
    bool _gsm_initialized = false;
    bool _radio_initialized = false;
    bool _wifi_initialized = false;

    // Config parameters

    int _gcs_id = 255;

    bool _enabled = false;
    std::string _tlf_1 = "0";
    std::string _tlf_2 = "0";
    std::string _tlf_3 = "0";
    int _rock_address = 0 ;
    int _mav_id = 0;

    std::chrono::milliseconds period_heartbeat_gcs;
    std::chrono::milliseconds period_gsm_gcs_hearbeat;

    // initialization parameters

    bool _initialized = false;
    int _vehicle_index = 0;
    int _gsm_number = 0;

    // timers for state machine

    timelib::Stopwatch timer_rfd_last_hearbeat;
    timelib::Stopwatch timer_gsm_update_report;
    timelib::Stopwatch timer_sbd_update_report;
    timelib::Stopwatch timer_autopilot_heartbeat;
    timelib::Stopwatch timer_update_report;
    timelib::Stopwatch timer_sms_alive;
    timelib::Stopwatch timer_update_active_channel;
    timelib::Stopwatch timer_gcs_heartbeat;

    // timestamps for state machine

    // initialize??
    std::chrono::milliseconds time_current; // initialize!!
    std::chrono::milliseconds time_last_wifi; // initialize!!
    std::chrono::milliseconds time_last_rfd; // initialize!!
    std::chrono::milliseconds time_last_sms; // initialize!!
    std::chrono::milliseconds timeout_wifi;  // initialize those!!
    std::chrono::milliseconds timeout_rfd;  // initialize those!!
    std::chrono::milliseconds timeout_sms;  // initialize those!!

    // timers for GCS commands

    timelib::Stopwatch timer_last_cmd_long;
    timelib::Stopwatch timer_last_cmd_set_current;
    timelib::Stopwatch timer_last_cmd_mission_item;
    timelib::Stopwatch timer_last_cmd_set_mode;

    // last vehicle state
    
    uint8_t last_base_mode = 0;
    uint32_t last_custom_mode = 0;
    uint16_t last_sys_errors_com = 0;
    uint16_t last_sys_errors_count1 = 0;
    uint16_t last_sys_errors_count2 = 0;
    uint16_t last_sys_errors_count3 = 0;
    uint16_t last_sys_errors_count4 = 0;
    uint16_t last_sys_drop_rate_com = 0;
    uint16_t last_sys_load = 0;
    uint32_t last_sys_sensors_present = 0;
    uint32_t last_sys_sensors_enabled = 0;
    uint32_t last_sys_sensors_health = 0;

    mavlink_message_t last_high_latency;
    bool last_high_latency_valid = false;

    // TCP channel for wifi - independent stream for each vehicle that is
    // why we have it here instead of in MAVLinkHandlerGround
    mavio::MAVLinkTCPChannel tcp_cli_channel;
};

} // namespace mavio

#endif // LIBS_MAVIO_INCLUDE_VEHICLE_H_