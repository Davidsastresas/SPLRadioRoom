/*
 MAVLinkHandlerAir.cc

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

#include "MAVLinkHandlerAir.h"

#include "Config.h"
#include "MAVLinkLogger.h"

using std::string;
using std::vector;

using mavio::log;
using mavio::MAVLinkChannel;
using mavio::Serial;
using radioroom::Config;

namespace radioroom {

constexpr int autopilot_send_retries = 5;
constexpr uint16_t data_stream_rate = 2;  // Hz

const std::chrono::milliseconds heartbeat_period(1000);
// const std::chrono::milliseconds autopilot_send_retry_timeout(250);

const std::chrono::milliseconds active_update_interval(100);
const std::chrono::milliseconds rfd_timeout(2000);
const std::chrono::milliseconds sms_timeout(120000);

const std::chrono::milliseconds report_period(18000);

constexpr char hl_report_period_param[] = "HL_REPORT_PERIOD";

MAVLinkHandlerAir::MAVLinkHandlerAir()
    : rfd(),
      autopilot(),
      isbd_channel(),
      sms_channel(),
      // tcp_channel(),
      current_time(0),
      update_report_timer(),
      heartbeat_timer(),
      primary_report_timer(),
      secondary_report_timer(),
      missions_received(0),
      rfd_active(true),
      sms_active(false),
      isbd_active(false),
      retry_timer(),
      retry_msg(),
      retry_timeout(0),
      retry_count(0),
      _sleep(false) {}

/**
 * Initializes autopilot and comm channels.
 */
bool MAVLinkHandlerAir::init() {

  vector<string> devices;
  Serial::get_serial_devices(devices);

  // ----------------- RFD ------------------
  if (!rfd.init(config.get_rfd_serial(), config.get_rfd_serial_speed(), devices)) {
    log(LOG_ERR, "UV Radio Room initialization failed: cannot connect to rfd900x.");
    return false;
  }
  // Exclude the serial device used by rfd 
  for (vector<string>::iterator iter = devices.begin(); iter != devices.end();
       ++iter) {
    if (*iter == rfd.get_path()) {
      devices.erase(iter);
      break;
    }
  }

  // ----------------- Autopilot ------------------
  if (!autopilot.init(config.get_autopilot_serial(), config.get_autopilot_serial_speed(), devices)) {
    log(LOG_ERR,"UV Radio Room initialization failed: cannot connect to autopilot.");
    return false;
  }
  // Exclude the serial device used by autopilot 
  for (vector<string>::iterator iter = devices.begin(); iter != devices.end();
       ++iter) {
    if (*iter == autopilot.get_path()) {
      devices.erase(iter);
      break;
    }
  }

  // ----------------- ISBD ------------------
  if (isbd_channel.init(config.get_isbd_serial(), config.get_isbd_serial_speed(), devices)) {
    log(LOG_INFO, "ISBD channel initialized.");
  } else {
    log(LOG_WARNING, "ISBD channel initialization failed.");
  }

  // ------------------- GSM -------------------
  if (sms_channel.init(config.get_gsm_serial(), config.get_gsm_serial_speed(), devices)) {
    log(LOG_INFO, "GSM channel initialized.");
  } else {
    log(LOG_WARNING, "GSM channel initialization failed.");
  }

  log(LOG_INFO, "UV Radio Room initialization succeeded.");
  
  return true;
}

/**
 * Closes all opened connections.
 */
void MAVLinkHandlerAir::close() {
  isbd_channel.close();
  autopilot.close();
  rfd.close();
  sms_channel.close();
}

/**
 * One iteration of the message handler.
 *
 * NOTE: Must not use blocking calls.
 */
bool MAVLinkHandlerAir::loop() {
  mavlink_message_t mo_msg;
  mavlink_message_t mt_msg;
  _sleep = true;

  if (rfd.receive_message(mt_msg)) {
    handle_mt_message(mt_msg);
    _sleep = false; 
  }

  if (sms_channel.receive_message(mt_msg)) {
    handle_mt_message(mt_msg);
    _sleep = false;
  }

  if (isbd_channel.receive_message(mt_msg)) {
    handle_mt_message(mt_msg);
    _sleep = false;
  }


  if (autopilot.receive_message(mo_msg)) {
    handle_mo_message(mo_msg);
    _sleep = false;
  }

  update_active_channel();
  
  send_report();

  send_heartbeat();

  return _sleep;
}

void MAVLinkHandlerAir::update_active_channel() {

  if (update_active_timer.elapsed_time() < active_update_interval) {
    return;
  }
  
  update_active_timer.reset();
  current_time = timelib::time_since_epoch();

  if ( rfd_active ) {
    if ( current_time - rfd.last_receive_time() > rfd_timeout ) {
      rfd_active = false;
      sms_active = true;
      isbd_active = false;
      log(LOG_INFO, "time since last rfd %d", current_time - rfd.last_receive_time());
      log(LOG_INFO, "change to sms");
    }
    return;

  } else if ( sms_active ) {
    if ( current_time - rfd.last_receive_time() <= rfd_timeout ) {
      rfd_active = true;
      sms_active = false;
      isbd_active = false;
      log(LOG_INFO, "time since last rfd %d", current_time - rfd.last_receive_time());
      log(LOG_INFO, "change to rfd");

    } else if ( current_time - sms_channel.last_receive_time() >= sms_timeout ) {
      rfd_active = false;
      sms_active = false;
      isbd_active = true;
      log(LOG_INFO, "time since last rfd %d", current_time - rfd.last_receive_time());
      log(LOG_INFO, "change to isbd");
    }
    return;

  } else { // isbd active 
    if ( current_time - rfd.last_receive_time() <= rfd_timeout ) {
      rfd_active = true;
      sms_active = false;
      isbd_active = false;
      log(LOG_INFO, "time since last rfd %d", current_time - rfd.last_receive_time());
      log(LOG_INFO, "change to rfd");

    } else if ( current_time - sms_channel.last_receive_time() <= sms_timeout ) {
      rfd_active = false;
      sms_active = true;
      isbd_active = false;
      log(LOG_INFO, "time since last rfd %d", current_time - rfd.last_receive_time());
      log(LOG_INFO, "change to sms");
    
    }
    return;
  }
}

// Forward ACK messages to the specified channel.
// Use missions array to get mission items on MISSION_REQUEST.
// Pass all other messages to update_report_msg().
void MAVLinkHandlerAir::handle_mo_message(const mavlink_message_t& msg) {

  rfd.send_message(msg);
  
  if ( rfd_active ) {
    return;

  } else {
    report.update(msg);
  }
}

/**
 * Handles writing waypoints list as described  in
 * http://qgroundcontrol.org/mavlink/waypoint_protocol
 *
 * If message specified by msg parameter is of type MISSION_COUNT,
 * the method retrieves all the mission items from the channel, sends them
 * to the autopilot, and sends MISSION_ACK to the channel. Otherwise the
 * method does nothing and just returns false.
 */
void MAVLinkHandlerAir::handle_mt_message(const mavlink_message_t& msg) {

  autopilot.send_message(msg);
}

bool MAVLinkHandlerAir::send_report() {

  if ( rfd_active ) {
    return false;
  }

  if (primary_report_timer.elapsed_time() >= report_period) {
    primary_report_timer.reset();

    mavlink_message_t report_msg;
    report.get_message(report_msg);

    if ( sms_active ) {
      return sms_channel.send_message(report_msg);
    
    } else {
      return sms_channel.send_message(report_msg);
    }
  }

  return false;
}

bool MAVLinkHandlerAir::send_heartbeat() {

  if ( rfd_active ) {
    return false;
  }

  if (heartbeat_timer.elapsed_time() >= heartbeat_period) {
    heartbeat_timer.reset();

    mavlink_message_t heartbeat_msg;
    mavlink_msg_heartbeat_pack(mavio::gcs_system_id, mavio::gcs_component_id,
                               &heartbeat_msg, MAV_TYPE_GCS,
                               MAV_AUTOPILOT_INVALID, 0, 0, 0);

    return autopilot.send_message(heartbeat_msg);
  }

  return false;
}



void MAVLinkHandlerAir::set_retry_send_timer(const mavlink_message_t& msg,
                                       const std::chrono::milliseconds& timeout,
                                       int retries) {
  retry_timer.reset();
  retry_msg = msg;
  retry_timeout = timeout;
  retry_count = retries;
}

void MAVLinkHandlerAir::cancel_retry_send_timer(int msgid) {
  if (retry_msg.msgid == msgid) {
    retry_count = 0;
  }
}

void MAVLinkHandlerAir::check_retry_send_timer() {
  if (retry_count > 0 && retry_timer.elapsed_time() >= retry_timeout) {
    rfd.send_message(retry_msg);
    retry_count--;
    retry_timer.reset();
  }
}

}  // namespace radioroom
