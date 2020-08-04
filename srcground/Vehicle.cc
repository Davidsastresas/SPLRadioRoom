#include "Vehicle.h"
#include "MAVLinkLogger.h"
#include "Config.h"

namespace mavio {

constexpr size_t max_vehicle_queue_size = 1024;
constexpr size_t max_vehicle_queue_size_high_lat = 256;

const std::chrono::milliseconds cmd_timer_default(timelib::sec2ms(2));
const std::chrono::milliseconds interval_update_active_channel(timelib::sec2ms(0.1));

Vehicle::Vehicle() 
    : _outgoing_queue_gcsmavmsg(max_vehicle_queue_size),
      _outgoing_queue_smsmsg(max_vehicle_queue_size_high_lat),
      _outgoing_queue_sbdmsg(max_vehicle_queue_size_high_lat),
      timer_rfd_last_hearbeat(),
      timer_gsm_update_report(),
      timer_sbd_update_report(),
      timer_autopilot_heartbeat(),
      timer_update_report(),
      timer_sms_alive(),
      timer_update_active_channel(),
      timer_last_cmd_long(),
      timer_last_cmd_set_current(),
      timer_last_cmd_mission_item(),
      timer_last_cmd_set_mode() {
      
      _enabled = false; // paranoid check it start as not initialized
      _initialized = false;
      _gsm_number = 0;
}

Vehicle::~Vehicle() {

}

bool Vehicle::initialized() {
    return _initialized;
}

// going to GCS
bool Vehicle::pull_queue_mavmsg(mavlink_message_t& msg) {
    return _outgoing_queue_gcsmavmsg.pop(msg);
}

// going to smschannel
bool Vehicle::pull_queue_smsmsg(mavio::SMSmessage& msg) {
    return _outgoing_queue_smsmsg.pop(msg);
}

// going to sbdchannel
bool Vehicle::pull_queue_sbdmsg(mavio::SBDmessage& msg) {
    return _outgoing_queue_sbdmsg.pop(msg);
}

bool Vehicle::update() {
  if ( !_initialized ) {
    return false;
  }

  // log(LOG_INFO, "vehicle %d update", _vehicle_index);

  update_active_channel();
  send_heartbeat_tcp();
  send_high_lat_heartbeats();

  // send signal quality();


  return true;
}

void Vehicle::send_heartbeat_sms(std::string number) {

  if ( number.size() < 10 ) {
    return;
  }

  mavlink_message_t sms_heartbeat_msg;
  mavio::SMSmessage sms_msg;

  mavlink_msg_heartbeat_pack(_gcs_id, 0,
                           &sms_heartbeat_msg, MAV_TYPE_GCS,
                           MAV_AUTOPILOT_INVALID, 0, 0, 0);

  sms_msg.set_mavlink_msg(sms_heartbeat_msg);
  sms_msg.set_number(number);
  _outgoing_queue_smsmsg.push(sms_msg);
}

void Vehicle::send_high_lat_heartbeats() {
  if ( _isbd_initialized && _active_channel == SBD_ACTIVE ) { 
    if ( !gsm_rehook_sent ) {
      send_heartbeat_sms(_tlf_1);
      send_heartbeat_sms(_tlf_2);
      send_heartbeat_sms(_tlf_3);

      gsm_rehook_sent = true;
    
    }
  }

  // heartbeat from SMS system, for air not to change to isbd
  if ( _gsm_initialized && _active_channel == GSM_ACTIVE ) {
    if ( timer_sms_alive.elapsed_time() >= period_gsm_gcs_hearbeat ) {
      timer_sms_alive.reset();
      send_heartbeat_sms(last_aircraft_number);
    }
  }
}

void Vehicle::send_heartbeat_tcp() {
  if (timer_gcs_heartbeat.elapsed_time() >= period_heartbeat_gcs) {
    if ( _active_channel != RFD_ACTIVE ) {
      timer_gcs_heartbeat.reset();
      
      mavlink_message_t heartbeat_msg;
  
      mavlink_msg_heartbeat_pack(_mav_id, 1,
                                 &heartbeat_msg, MAV_TYPE_FIXED_WING,
                                 MAV_AUTOPILOT_ARDUPILOTMEGA, last_base_mode, 
                                 last_custom_mode, 0);
  
      _outgoing_queue_gcsmavmsg.push(heartbeat_msg);

      if (last_high_latency_valid) { // CHECK HOW THE HELL WE KNOW THIS
        _outgoing_queue_gcsmavmsg.push(last_high_latency);
      }
    }
  }
}

void Vehicle::update_active_channel() {
  
  if ( timer_update_active_channel.elapsed_time() < interval_update_active_channel ) {
    return;
  }

  timer_update_active_channel.reset();
  time_current = timelib::time_since_epoch();

  switch (_active_channel) {
    case RFD_ACTIVE: {
      if ( time_current - time_last_rfd > timeout_rfd ) {
        handle_rfd_out();
      }
    }
    break;

    case GSM_ACTIVE: {
      if ( time_current - time_last_rfd < timeout_rfd ) {
        set_rfd_active();
      } else if ( time_current - time_last_sms > timeout_sms ) {
        handle_gsm_out();
      }
    }
    break;
    
    case SBD_ACTIVE: {
      if ( time_current - time_last_rfd < timeout_rfd ) {
        set_rfd_active();
      } else if ( time_current - time_last_sms < timeout_sms ) {
        set_gsm_active();
      }
    }
    break;
    
    default:
      set_rfd_active();
    break;
  }
}

bool Vehicle::set_isbd_active() {
  if (_isbd_initialized) {
    _active_channel = SBD_ACTIVE;

    gsm_rehook_sent = false;

    log(LOG_INFO, "ISBD active");

    mavlink_message_t status_msg;
    mavlink_msg_statustext_pack(1,1, &status_msg, MAV_SEVERITY_NOTICE, "ISBD active", 0, 0);
    _outgoing_queue_gcsmavmsg.push(status_msg);
    
    return true;
  } 
  
  return false;
}

bool Vehicle::set_gsm_active() {
  if (_gsm_initialized) {
    _active_channel = GSM_ACTIVE;

    // gsm_channel.reset_timer();
    time_last_sms = timelib::time_since_epoch(); // needed for not jumping directly to sbd
    timer_sms_alive.reset();
    log(LOG_INFO, "GSM active");

    mavlink_message_t status_msg;
    mavlink_msg_statustext_pack(1,1, &status_msg, MAV_SEVERITY_NOTICE, "GSM active", 0, 0);
    _outgoing_queue_gcsmavmsg.push(status_msg);

    return true;
  } 

  return false;
}

bool Vehicle::set_rfd_active() {
  if (_radio_initialized) {
    _active_channel = RFD_ACTIVE;

    log(LOG_INFO, "RFD active");

    last_high_latency_valid = false;

    mavlink_message_t status_msg;
    mavlink_msg_statustext_pack(1,1, &status_msg, MAV_SEVERITY_NOTICE, "RFD active", 0, 0);
    _outgoing_queue_gcsmavmsg.push(status_msg);
    
    return true;
  } 

  return false;
}

void Vehicle::handle_rfd_out() {
  if (set_gsm_active()) {
    return;
  } else if (set_isbd_active()) { 
    return;
  } else {
    // we may eliminate this so it doesn't send repeated message to gcs
    set_rfd_active();
  }
}

void Vehicle::handle_gsm_out() {
  if (set_isbd_active()) {
    return; 
  } else {
    // we may eliminate this so it doesn't send repeated message to gcs
    set_rfd_active();
  }
}

bool Vehicle::init(int instance) {

  _vehicle_index = (instance - 1);                   
      
  _enabled = radioroom::config.get_aircraftx_enabled(instance);
  
  if (_enabled) {
  
      if ( radioroom::config.get_aircraftx_tlfx(instance, 1).size() > 10 ) {

          _gsm_initialized = true;
          _gsm_number++;
          _tlf_1 = radioroom::config.get_aircraftx_tlfx(instance, 1);
      }
  
      if ( radioroom::config.get_aircraftx_tlfx(instance, 2).size() > 10 ) {
          
          _gsm_initialized = true;
          _gsm_number++;
          _tlf_2 = radioroom::config.get_aircraftx_tlfx(instance, 2);
      }
  
      if ( radioroom::config.get_aircraftx_tlfx(instance, 3).size() > 10 ) {
          
          _gsm_initialized = true;
          _gsm_number++;
          _tlf_3 = radioroom::config.get_aircraftx_tlfx(instance, 3);
      }
  
      if ( radioroom::config.get_aircraftx_rock_address(instance) ) {

          // _sbd_enabled = true; // is this needed??
          _isbd_initialized = true;
          _rock_address = radioroom::config.get_aircraftx_rock_address(instance);
      }             
  
      if ( radioroom::config.get_aircraftx_mav_id(instance) ) {

          _mav_id = radioroom::config.get_aircraftx_mav_id(instance);

      } else {

          _initialized = false;

          mavio::log(LOG_ERR, "Vehicle %d MAV_ID (%d) not valid",     
                     _vehicle_index, _mav_id);
          return false;
      }
  
      _initialized = true;

      _radio_initialized = true;
      
      // timer_gcs_heartbeat.reset();
      // timer_update_active_channel.reset();

      timer_rfd_last_hearbeat.reset();
      timer_gsm_update_report.reset();
      timer_sbd_update_report.reset();
      timer_autopilot_heartbeat.reset();
      timer_update_report.reset();
      timer_sms_alive.reset();
      timer_update_active_channel.reset();
      timer_gcs_heartbeat.reset();

      timer_last_cmd_long.reset();
      timer_last_cmd_set_current.reset();
      timer_last_cmd_mission_item.reset();
      timer_last_cmd_set_mode.reset();


      time_current = timelib::time_since_epoch();
      time_last_rfd = timelib::time_since_epoch();
      time_last_sms = timelib::time_since_epoch();

      last_high_latency_valid = false;
      last_aircraft_number = _tlf_1;

      period_heartbeat_gcs = timelib::sec2ms(2);
      timeout_rfd = timelib::sec2ms(4);
      timeout_sms = timelib::sec2ms(30);
      period_gsm_gcs_hearbeat = timelib::sec2ms(30);

      if (radioroom::config.get_heartbeat_period() > 1) {
        period_heartbeat_gcs = timelib::sec2ms((double)radioroom::config.get_heartbeat_period());
      }

      if ( radioroom::config.get_rfd_timeout() > 2 ) {
        timeout_rfd = timelib::sec2ms((double)radioroom::config.get_rfd_timeout());
      }

      if ( radioroom::config.get_sms_timeout() > 25 ) {
        timeout_sms = timelib::sec2ms((double)radioroom::config.get_sms_timeout());
      }

      if ( radioroom::config.get_sms_heartbeat_period() > 20 ) {
        period_gsm_gcs_hearbeat = timelib::sec2ms((double)radioroom::config.get_sms_heartbeat_period());
      }

      if ( !radioroom::config.get_rfd_enabled() ) {
        _radio_initialized = false;
      }

      log(LOG_INFO, "Vehicle %d tlf 1: %s", instance, _tlf_1.c_str());
      log(LOG_INFO, "Vehicle %d tlf 2: %s", instance, _tlf_2.c_str());
      log(LOG_INFO, "Vehicle %d tlf 3: %s", instance, _tlf_3.c_str());

      log(LOG_INFO, "Vehicle %d rock address: %d", instance, _rock_address);
      log(LOG_INFO, "Vehicle %d mav_id: %d", instance, _mav_id);

      log(LOG_INFO, "Vehicle initialized: %d", _initialized);

      log(LOG_INFO,"rfd_timeout %d", timeout_rfd);
      log(LOG_INFO,"sms_timeout %d", timeout_sms);
      log(LOG_INFO,"sms_alive_period %d", period_gsm_gcs_hearbeat);
      log(LOG_INFO,"heartbeat_period %d", period_heartbeat_gcs);

      return true;

    } else {
  
        _initialized = false;

        // mavio::log(LOG_INFO, "Vehicle %d not enabled", instance);
        return false;
    }
}

Vehicle::enum_vehicle_act_chan Vehicle::get_active_chan() const {
    return _active_channel;
}

bool Vehicle::set_active_chan(enum_vehicle_act_chan active_chan) {
    _active_channel = active_chan;

    return true;
}

int Vehicle::get_mavid() {
    return _mav_id;
}

void Vehicle::process_gcs_message(mavlink_message_t& msg) {
  if ( !_initialized ) {
    return;
  }

  if ( _active_channel != RFD_ACTIVE ) {

      switch (msg.msgid) {

        case MAVLINK_MSG_ID_COMMAND_LONG:{  
          if ( mavlink_msg_command_long_get_target_system(&msg) != _mav_id ) {
            return;
          }

          if ( timer_last_cmd_long.elapsed_time() < cmd_timer_default ) {
            return;
          }

          mavlink_message_t ack_cmd_long;
          mavlink_command_ack_t ack_prep;
          ack_prep.command = mavlink_msg_command_long_get_command(&msg);
          ack_prep.result = 0;
          mavlink_msg_command_ack_encode(_mav_id, 1, &ack_cmd_long, &ack_prep);
          _outgoing_queue_gcsmavmsg.push(ack_cmd_long);

          timer_last_cmd_long.reset();

          break;
        } 

        case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
          if ( mavlink_msg_mission_set_current_get_target_system(&msg) != _mav_id ) {
            return;
          }

          if ( timer_last_cmd_set_current.elapsed_time() < cmd_timer_default ) {
            return;
          }

          mavlink_message_t ack_mission_set_current;
          mavlink_mission_current_t ack_prep;
          ack_prep.seq = mavlink_msg_mission_current_get_seq(&msg);
          mavlink_msg_mission_current_encode(_mav_id, 1, &ack_mission_set_current, &ack_prep);
          _outgoing_queue_gcsmavmsg.push(ack_mission_set_current);

          timer_last_cmd_set_current.reset();

          break;
        }

        case MAVLINK_MSG_ID_MISSION_ITEM: {
          if ( mavlink_msg_mission_item_int_get_target_system(&msg) != _mav_id ) {
            return;
          }

          if ( timer_last_cmd_mission_item.elapsed_time() < cmd_timer_default ) {
            return;
          }

          mavlink_message_t ack_mission_item;
          mavlink_mission_ack_t ack_prep;
          ack_prep.mission_type = 0;
          ack_prep.target_component = msg.compid;
          ack_prep.target_system = msg.sysid;
          ack_prep.type = 0;
          mavlink_msg_mission_ack_encode(_mav_id, 1, &ack_mission_item, &ack_prep);
          _outgoing_queue_gcsmavmsg.push(ack_mission_item);

          timer_last_cmd_mission_item.reset();

          break;
        }

        case MAVLINK_MSG_ID_SET_MODE: {
          if ( mavlink_msg_set_mode_get_target_system(&msg) != _mav_id ) {
            return;
          }

          if ( timer_last_cmd_set_mode.elapsed_time() < cmd_timer_default ) {
            return;
          }

          mavlink_message_t ack_cmd_set_current;
          mavlink_command_ack_t ack_prep;
          ack_prep.command = mavlink_msg_command_long_get_command(&msg);
          ack_prep.result = 0;
          mavlink_msg_command_ack_encode(_mav_id, 1, &ack_cmd_set_current, &ack_prep);
          _outgoing_queue_gcsmavmsg.push(ack_cmd_set_current);

          timer_last_cmd_set_mode.reset();

          break;
        }

        default: return;
          break;
      }
    // refactor here the stuff
    if ( _active_channel == GSM_ACTIVE ) {
            mavio::SMSmessage sms_msg(msg, last_aircraft_number);
            _outgoing_queue_smsmsg.push(sms_msg);
          }

    if ( _active_channel == SBD_ACTIVE ) {
            mavio::SBDmessage sbd_msg(msg, _rock_address );
            _outgoing_queue_sbdmsg.push(sbd_msg);
    }

    timer_sms_alive.reset();
  }
}

void Vehicle::process_message(mavlink_message_t& msg) {
  if ( !_initialized ) {
    return;
  }

  // update state of the heartbeat sent to GCS to represent the last state of vehicle
  time_last_rfd = timelib::time_since_epoch();

  if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {

    timer_rfd_last_hearbeat.reset(); // dont send the groundpi originate heartbeat unless necesary
    last_base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
    last_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
  }

  // prepare for the extended battery message from High_latency
  if (msg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {

    last_sys_drop_rate_com = mavlink_msg_sys_status_get_drop_rate_comm(&msg);
    last_sys_errors_com = mavlink_msg_sys_status_get_errors_comm(&msg);
    last_sys_errors_count1 = mavlink_msg_sys_status_get_errors_count1(&msg);
    last_sys_errors_count2 = mavlink_msg_sys_status_get_errors_count2(&msg);
    last_sys_errors_count3 = mavlink_msg_sys_status_get_errors_count3(&msg);
    last_sys_errors_count4 = mavlink_msg_sys_status_get_errors_count4(&msg);
    last_sys_load = mavlink_msg_sys_status_get_load(&msg);
    last_sys_sensors_enabled = mavlink_msg_sys_status_get_onboard_control_sensors_enabled(&msg);
    last_sys_sensors_health = mavlink_msg_sys_status_get_onboard_control_sensors_health(&msg);
    last_sys_sensors_present = mavlink_msg_sys_status_get_onboard_control_sensors_present(&msg);
  }
}

void Vehicle::process_message(mavio::SMSmessage& sms) {
  if ( !_initialized ) {
    return;
  }

  mavlink_message_t msg = sms.get_mavlink_msg();
  last_aircraft_number = sms.get_number();

  if (sms.get_mavlink_msg().msgid == MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT) {

    last_base_mode = mavlink_msg_am_telemetry_high_lat_get_base_mode(&msg);
    last_base_mode = mavlink_msg_am_telemetry_high_lat_get_base_mode(&msg);
    last_custom_mode = mavlink_msg_am_telemetry_high_lat_get_custom_mode(&msg);
    last_high_latency = msg;

    // check this
    last_high_latency_valid = true;

    mavlink_message_t sys_status;
    mavlink_sys_status_t status_mavlink_msg;

    status_mavlink_msg.battery_remaining = mavlink_msg_am_telemetry_high_lat_get_battery_remaining(&msg);
    status_mavlink_msg.current_battery = mavlink_msg_am_telemetry_high_lat_get_landed_state(&msg) * 100;
    status_mavlink_msg.voltage_battery = mavlink_msg_am_telemetry_high_lat_get_temperature(&msg);
    status_mavlink_msg.voltage_battery = status_mavlink_msg.voltage_battery << 8;
    status_mavlink_msg.voltage_battery += mavlink_msg_am_telemetry_high_lat_get_temperature_air(&msg);
    status_mavlink_msg.drop_rate_comm = last_sys_drop_rate_com;
    status_mavlink_msg.errors_comm = last_sys_errors_com;
    status_mavlink_msg.errors_count1 = last_sys_errors_count1;
    status_mavlink_msg.errors_count2 = last_sys_errors_count2;
    status_mavlink_msg.errors_count3 = last_sys_errors_count3;
    status_mavlink_msg.errors_count4 = last_sys_errors_count4;
    status_mavlink_msg.load = last_sys_load;
    status_mavlink_msg.onboard_control_sensors_enabled = last_sys_sensors_enabled;
    status_mavlink_msg.onboard_control_sensors_health = last_sys_sensors_health;
    status_mavlink_msg.onboard_control_sensors_present = last_sys_sensors_present;

    mavlink_msg_sys_status_encode(_mav_id, 1, &sys_status, &status_mavlink_msg);

    _outgoing_queue_gcsmavmsg.push(sys_status);

    // encode high latency
    mavlink_message_t high_lat;
    mavlink_high_latency_t high_latency_msg;

    high_latency_msg.custom_mode = mavlink_msg_am_telemetry_high_lat_get_custom_mode(&msg);
    high_latency_msg.latitude = mavlink_msg_am_telemetry_high_lat_get_latitude(&msg);
    high_latency_msg.longitude = mavlink_msg_am_telemetry_high_lat_get_longitude(&msg);
    high_latency_msg.roll = mavlink_msg_am_telemetry_high_lat_get_roll(&msg);
    high_latency_msg.pitch = mavlink_msg_am_telemetry_high_lat_get_pitch(&msg);
    high_latency_msg.heading = mavlink_msg_am_telemetry_high_lat_get_heading(&msg);
    high_latency_msg.heading_sp = mavlink_msg_am_telemetry_high_lat_get_heading_sp(&msg);
    high_latency_msg.altitude_amsl = mavlink_msg_am_telemetry_high_lat_get_altitude_amsl(&msg);
    high_latency_msg.altitude_sp = mavlink_msg_am_telemetry_high_lat_get_altitude_sp(&msg);
    high_latency_msg.wp_distance = mavlink_msg_am_telemetry_high_lat_get_wp_distance(&msg);
    high_latency_msg.base_mode = mavlink_msg_am_telemetry_high_lat_get_base_mode(&msg);
    high_latency_msg.landed_state = mavlink_msg_am_telemetry_high_lat_get_landed_state(&msg);
    high_latency_msg.throttle = mavlink_msg_am_telemetry_high_lat_get_throttle(&msg);
    high_latency_msg.airspeed = mavlink_msg_am_telemetry_high_lat_get_airspeed(&msg);
    high_latency_msg.airspeed_sp = mavlink_msg_am_telemetry_high_lat_get_airspeed_sp(&msg);
    high_latency_msg.groundspeed = mavlink_msg_am_telemetry_high_lat_get_groundspeed(&msg);
    high_latency_msg.climb_rate = mavlink_msg_am_telemetry_high_lat_get_climb_rate(&msg);
    high_latency_msg.gps_nsat = mavlink_msg_am_telemetry_high_lat_get_gps_nsat(&msg);
    high_latency_msg.gps_fix_type = mavlink_msg_am_telemetry_high_lat_get_gps_fix_type(&msg);
    high_latency_msg.battery_remaining = mavlink_msg_am_telemetry_high_lat_get_battery_remaining(&msg);
    high_latency_msg.temperature = mavlink_msg_am_telemetry_high_lat_get_temperature(&msg);
    high_latency_msg.temperature_air = mavlink_msg_am_telemetry_high_lat_get_temperature_air(&msg);
    high_latency_msg.failsafe = mavlink_msg_am_telemetry_high_lat_get_failsafe(&msg);
    high_latency_msg.wp_num = mavlink_msg_am_telemetry_high_lat_get_wp_num(&msg);


    mavlink_msg_high_latency_encode(_mav_id, 1, &high_lat, &high_latency_msg);

    _outgoing_queue_gcsmavmsg.push(high_lat);

    // encode link_status on radio_status as follows
    //
    // rxerros - sbd rssi
    // fixed - not used
    // rssi - gsm 1 rssi
    // remrsssi - gsm 2 rssi
    // txbuf - gsm 3 rssi
    // noise - comp bitmask
    // remnoise - telemetry bitmask
    // 
    // compid is mav_id plus 10

    mavlink_message_t radio_status;
    mavlink_radio_status_t radio_status_msg;

    radio_status_msg.rxerrors = mavlink_msg_am_telemetry_high_lat_get_rssi_sbd(&msg);
    radio_status_msg.rssi = mavlink_msg_am_telemetry_high_lat_get_rssi_gsm1(&msg);
    radio_status_msg.remrssi = mavlink_msg_am_telemetry_high_lat_get_rssi_gsm2(&msg);
    radio_status_msg.txbuf = mavlink_msg_am_telemetry_high_lat_get_rssi_gsm3(&msg);
    radio_status_msg.noise = mavlink_msg_am_telemetry_high_lat_get_comp_status_bitmask(&msg);
    radio_status_msg.remnoise = mavlink_msg_am_telemetry_high_lat_get_telem_status_bitmask(&msg);

    mavlink_msg_radio_status_encode(_mav_id, _mav_id + 10, &radio_status, &radio_status_msg);

    _outgoing_queue_gcsmavmsg.push(radio_status);
  }

  time_last_sms = timelib::time_since_epoch();
}

void Vehicle::process_message(mavio::SBDmessage& sbdmsg) {
  if ( !_initialized ) {
    return;
  }

  if ( _active_channel == SBD_ACTIVE ) {
    mavlink_message_t msg = sbdmsg.get_mavlink_msg();

    if (msg.msgid == MAVLINK_MSG_ID_HIGH_LATENCY) {
      last_base_mode = mavlink_msg_am_telemetry_high_lat_get_base_mode(&msg);
      last_custom_mode = mavlink_msg_am_telemetry_high_lat_get_custom_mode(&msg);
      last_high_latency = msg;
      last_high_latency_valid = true;
    }
  }
}



} // namespace mavio