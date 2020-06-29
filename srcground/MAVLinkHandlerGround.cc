/*
 MAVLinkHandlerGround.cc

 Telemetry for MAVLink autopilots.

 (C) Copyright 2019 Envirover.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 Created on: Oct 17, 2017
     Author: Pavel Bobov
 */

#include "MAVLinkHandlerGround.h"

#include "Config.h"
#include "MAVLinkLogger.h"

using std::string;
using std::vector;

using mavio::log;
using mavio::MAVLinkChannel;
using mavio::Serial;
using radioroom::Config;

namespace radioroom {

MAVLinkHandlerGround::MAVLinkHandlerGround()
    : rfd(),
      isbd_channel(),
      tcp_channel(),
      sms_channel(),
      current_time(0),
      last_rfd_heartbeat_timer(),
      update_report_timer(),
      heartbeat_timer(),
      primary_report_timer(),
      secondary_report_timer(),
      missions_received(0),
      retry_timer(),
      retry_msg(),
      retry_timeout(0),
      retry_count(0),
      _sleep(false),
      active_update_timer(),
      sms_alive_timer(),
      isbd_alive_timer() {}

/**
 * One iteration of the message handler.
 * No blocking calls here
 */
bool MAVLinkHandlerGround::loop() {
  mavlink_message_t mo_msg;
  mavlink_message_t mt_msg;
  mavio::SMSmessage mo_sms;
  bool sleep = true;

  if (rfd.receive_message(mo_msg)) {
    handle_mo_message(mo_msg);
    sleep = false; 
  }

  if (sms_channel.receive_message(mo_sms)) {
    handle_mo_sms(mo_sms);
    sleep = false;
  }

  if (isbd_initialized) {
    if (isbd_channel.receive_message(mo_msg)) {
      handle_mo_message(mo_msg);
      sleep = false;
    }
  }

  if (tcp_channel.receive_message(mt_msg)) {
    handle_mt_message(mt_msg);
    sleep = false;
  }

  update_active_channel();

  send_heartbeats();

  return sleep;
}

void MAVLinkHandlerGround::update_active_channel() {

  if (active_update_timer.elapsed_time() < active_update_interval) {
    return;
  }
  
  active_update_timer.reset();
  current_time = timelib::time_since_epoch();

  if (rfd_active) {
    if ( current_time - rfd.last_receive_time() > rfd_timeout ) {
      handle_rfd_out();
    }

  } else if (gsm_active) {
    if ( current_time - rfd.last_receive_time() < rfd_timeout ) {
      set_rfd_active();
    } else if ( current_time - last_sms_time > sms_timeout ) {
      handle_gsm_out();
    }

  } else if (isbd_active) {
    if ( current_time - rfd.last_receive_time() < rfd_timeout ) {
      set_rfd_active();
    } else if ( current_time - last_sms_time < sms_timeout ) {
      set_gsm_active();
    }
     
  } else {
    set_rfd_active();
  }
}

void MAVLinkHandlerGround::handle_mo_message(const mavlink_message_t& msg) {
  
  tcp_channel.send_message(msg);

  // update state of the heartbeat sent to GCS to represent the last state of vehicle
  if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
    last_rfd_heartbeat_timer.reset(); // dont send the groundpi originate heartbeat unless necesary
    last_base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
    last_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
  }

  if (!rfd_active) {
    if (msg.msgid == MAVLINK_MSG_ID_HIGH_LATENCY) {
      last_base_mode = mavlink_msg_high_latency_get_base_mode(&msg);
      last_custom_mode = mavlink_msg_high_latency_get_custom_mode(&msg);
      last_high_latency = msg;
    }
  }
}

void MAVLinkHandlerGround::handle_mo_sms(mavio::SMSmessage& sms) {
  
  mavlink_message_t msg = sms.get_mavlink_msg();
  last_aircraft_number = sms.get_number();

  tcp_channel.send_message(msg);

  if (sms.get_mavlink_msg().msgid == MAVLINK_MSG_ID_HIGH_LATENCY) {
    last_base_mode = mavlink_msg_high_latency_get_base_mode(&msg);
    last_custom_mode = mavlink_msg_high_latency_get_custom_mode(&msg);
    last_high_latency = msg;
  }

  last_sms_time = timelib::time_since_epoch();
}

void MAVLinkHandlerGround::handle_mt_message(const mavlink_message_t& msg) {

  rfd.send_message(msg);
}

void MAVLinkHandlerGround::send_heartbeats() {

  // if ( isbd_alive_timer.elapsed_time() >= isbd_alive_period ) {
  //    isbd_alive_timer.reset();
  //    log(LOG_INFO, "send_hearbeat_isbd");
  //    send_hearbeat_isbd();
  //  }
  
  // if ( rfd_active ) {
  //   return;
  // }

  // heartbeat from SMS system, for air not to change to isbd
  if ( gsm_initialized && gsm_active ) {
    if ( sms_alive_timer.elapsed_time() >= sms_alive_period ) {
      sms_alive_timer.reset();
      send_hearbeat_sms();
    }
  }
  // hearbeat from isbd, for the air unit to know ground is ok
  // if ( isbd_active ) {
  //   if ( isbd_alive_timer.elapsed_time() >= isbd_alive_period ) {
  //     isbd_alive_timer.reset();
  //     send_hearbeat_isbd();
  //   }
  // }
  // hearbeat to GCS, for not showing fail safe
  if (heartbeat_timer.elapsed_time() >= heartbeat_period) {
    heartbeat_timer.reset();
    send_hearbeat_tcp();
    tcp_channel.send_message(last_high_latency);
  }
}

bool MAVLinkHandlerGround::send_hearbeat_isbd() {

  mavlink_message_t sms_heartbeat_msg;

  mavlink_msg_heartbeat_pack(255, 0,
                           &sms_heartbeat_msg, MAV_TYPE_GCS,
                           MAV_AUTOPILOT_INVALID, 0, 0, 0);
  
  isbd_channel.send_message(sms_heartbeat_msg);

  return true;
}

bool MAVLinkHandlerGround::send_hearbeat_sms() {

  mavlink_message_t sms_heartbeat_msg;
  mavio::SMSmessage sms_msg;

  mavlink_msg_heartbeat_pack(255, 0,
                           &sms_heartbeat_msg, MAV_TYPE_GCS,
                           MAV_AUTOPILOT_INVALID, 0, 0, 0);

  sms_msg.set_mavlink_msg(sms_heartbeat_msg);
  sms_msg.set_number(last_aircraft_number);
  sms_channel.send_message(sms_msg);

  return true;
}

bool MAVLinkHandlerGround::send_hearbeat_tcp() {

  mavlink_message_t heartbeat_msg;
  
  mavlink_msg_heartbeat_pack(1, 1,
                             &heartbeat_msg, MAV_TYPE_FIXED_WING,
                             MAV_AUTOPILOT_ARDUPILOTMEGA, last_base_mode, last_custom_mode, 0);
  
  tcp_channel.send_message(heartbeat_msg);

  return true;
}

/**
 * Initializes autopilot and comm channels.
 *
 * Automatically detects the correct serial devices if autopilot and ISBD
 * transceiver do not respond on the devices specified by the configuration
 * properties.
 */
bool MAVLinkHandlerGround::init() {

  vector<string> devices;

  // Radio ---------------------------------------------------------------------------------------------
  if (config.get_rfd_enabled()) {  
    if (!rfd.init(config.get_rfd_serial(), config.get_rfd_serial_speed(), devices, config.get_rfd_id())) {
      log(LOG_ERR, "UV Radio Room initialization failed: cannot connect to rfd900x.");
      return false;
    }
    radio_initialized = true;
    // Exclude the serial device used by rfd
    for (vector<string>::iterator iter = devices.begin(); iter != devices.end();
         ++iter) {
      if (*iter == rfd.get_path()) {
        devices.erase(iter);
        break;
      }
    }
  } else {
    log(LOG_INFO, "Radio disabled.");
    radio_initialized = false;
  }

  // GSM -------------------------------------------------------------------------------------------------
  if (config.get_gsm_enabled()) {  
    if (sms_channel.init(config.get_gsm_serial1(), config.get_gsm_serial2(), config.get_gsm_serial3(),
                         config.get_gsm_pin1(), config.get_gsm_pin2(), config.get_gsm_pin3(),
                         config.get_gsm_serial_speed(), config.get_gsm_pdu_enabled())) {
                           
      log(LOG_INFO, "SMS channel initialized.");
      gsm_initialized = true;
    } else {
      log(LOG_WARNING, "SMS channel initialization failed.");
      return false;
    }
  } else {
    log(LOG_INFO, "GSM disabled.");
    gsm_initialized = false;
  }

  // SBD -------------------------------------------------------------------------------------------------
  if (config.get_isbd_enabled()) {  
    if (isbd_channel.init(config.get_isbd_serial(), config.get_isbd_serial_speed(), devices, config.get_aircraft1_rock_address())) {
      log(LOG_INFO, "ISBD channel initialized.");
      isbd_initialized = true;
      isbd_first_contact = !config.get_isbd_get_noticed(); 
    } else {
      log(LOG_WARNING, "ISBD channel initialization failed.");
      return false;
    }
  } else {
    log(LOG_INFO, "SBD disabled.");
    isbd_initialized = false;
  }

  // TCP --------------------------------------------------------------------------------------------------
  if (tcp_channel.init(config.get_tcp_port())) {
    log(LOG_INFO, "TCP channel initialized.");
  } else {
    log(LOG_WARNING, "TCP channel initialization failed.");
    return false;
  }

  heartbeat_timer.reset();
  // sms_channel.reset_timer();
  last_sms_time = timelib::time_since_epoch();
  rfd.reset_timer();

  active_update_interval = timelib::sec2ms(0.1);
  isbd_alive_period = timelib::sec2ms(600);
  
  // configurable options

  last_aircraft_number = config.get_aircraft1_tlf_number1();

  heartbeat_period = timelib::sec2ms(1);
  rfd_timeout = timelib::sec2ms(2);
  sms_timeout = timelib::sec2ms(25);
  sms_alive_period = timelib::sec2ms(20);

  if (config.get_heartbeat_period() > 1) {
    heartbeat_period = timelib::sec2ms((double)config.get_heartbeat_period());
  }

  if ( config.get_rfd_timeout() > 2 ) {
    rfd_timeout = timelib::sec2ms((double)config.get_rfd_timeout());
  }

  if ( config.get_sms_timeout() > 25 ) {
    sms_timeout = timelib::sec2ms((double)config.get_sms_timeout());
  }

  if ( config.get_sms_heartbeat_period() > 20 ) {
    sms_alive_period = timelib::sec2ms((double)config.get_sms_heartbeat_period());
  }

  log(LOG_INFO,"aircraft1 number1 %s", last_aircraft_number.c_str());
  log(LOG_INFO,"rfd_timeout %d", rfd_timeout);
  log(LOG_INFO,"sms_timeout %d", sms_timeout);
  log(LOG_INFO,"sms_alive_period %d", sms_alive_period);
  log(LOG_INFO,"heartbeat_period %d", heartbeat_period);


  log(LOG_INFO, "UV Radio Room initialization succeeded.");
  return true;
}

/**
 * Closes all opened connections.
 */
void MAVLinkHandlerGround::close() {
  tcp_channel.close();
  isbd_channel.close();
  sms_channel.close();
  rfd.close();
}

void MAVLinkHandlerGround::set_retry_send_timer(const mavlink_message_t& msg,
                                       const std::chrono::milliseconds& timeout,
                                       int retries) {
  retry_timer.reset();
  retry_msg = msg;
  retry_timeout = timeout;
  retry_count = retries;
}

void MAVLinkHandlerGround::cancel_retry_send_timer(int msgid) {
  if (retry_msg.msgid == msgid) {
    retry_count = 0;
  }
}

void MAVLinkHandlerGround::check_retry_send_timer() {
  if (retry_count > 0 && retry_timer.elapsed_time() >= retry_timeout) {
    rfd.send_message(retry_msg);
    retry_count--;
    retry_timer.reset();
  }
}

bool MAVLinkHandlerGround::set_isbd_active() {
  if (isbd_initialized) {
    rfd_active = false;
    gsm_active = false;
    isbd_active = true;
    log(LOG_INFO, "ISBD active");

    mavlink_message_t status_msg;
    mavlink_msg_statustext_pack(1,1, &status_msg, MAV_SEVERITY_NOTICE, "ISBD active", 0, 0);
    tcp_channel.send_message(status_msg);
    
  } else {
    isbd_active = false;
  }
  return isbd_active;
}

bool MAVLinkHandlerGround::set_gsm_active() {
  if (gsm_initialized) {
    rfd_active = false;
    isbd_active = false;
    gsm_active = true;
    // sms_channel.reset_timer();
    last_sms_time = timelib::time_since_epoch();
    sms_alive_timer.reset();
    log(LOG_INFO, "GSM active");

    mavlink_message_t status_msg;
    mavlink_msg_statustext_pack(1,1, &status_msg, MAV_SEVERITY_NOTICE, "GSM active", 0, 0);
    tcp_channel.send_message(status_msg);

  } else {
    gsm_active = false;
  }
  return gsm_active;
}

bool MAVLinkHandlerGround::set_rfd_active() {
  if (radio_initialized) {
    isbd_active = false;
    gsm_active = false;
    rfd_active = true;
    log(LOG_INFO, "RFD active");

    mavlink_message_t status_msg;
    mavlink_msg_statustext_pack(1,1, &status_msg, MAV_SEVERITY_NOTICE, "RFD active", 0, 0);
    tcp_channel.send_message(status_msg);
    
  } else {
    rfd_active = false;
  }
  return rfd_active;
}

void MAVLinkHandlerGround::handle_rfd_out() {
  if (set_gsm_active()) {
    return;
  } else if (set_isbd_active()) { 
    return;
  } else {
    set_rfd_active();
  }
}

void MAVLinkHandlerGround::handle_gsm_out() {
  if (set_isbd_active()) {
    return; 
  } else {
    set_rfd_active();
  }
}

}  // namespace radioroom
