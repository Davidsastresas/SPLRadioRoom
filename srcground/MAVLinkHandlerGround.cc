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

constexpr int autopilot_send_retries = 5;
constexpr uint16_t data_stream_rate = 2;  // Hz

const std::chrono::milliseconds heartbeat_period(1000);
// const std::chrono::milliseconds autopilot_send_retry_timeout(250);

const std::chrono::milliseconds active_update_interval(100);
const std::chrono::milliseconds rfd_timeout(2000);
const std::chrono::milliseconds sms_timeout(30000);

const std::chrono::milliseconds sms_alive_period(30000);

constexpr char hl_report_period_param[] = "HL_REPORT_PERIOD";

MAVLinkHandlerGround::MAVLinkHandlerGround()
    : rfd(),
      isbd_channel(),
      tcp_channel(),
      sms_channel(),
      current_time(0),
      sms_alive_timer(),
      update_active_timer(),
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
 *
 * Automatically detects the correct serial devices if autopilot and ISBD
 * transceiver do not respond on the devices specified by the configuration
 * properties.
 */
bool MAVLinkHandlerGround::init() {
  // if (!config.get_tcp_enabled() && !config.get_isbd_enabled()) {
  //   log(LOG_ERR, "Invalid configuration: no enabled comm channels.");
  //   return false;
  // }

  vector<string> devices;

  // maybe out??
  if (config.get_auto_detect_serials()) {
    Serial::get_serial_devices(devices);
  }

  if (config.get_rfd_enabled()) {
    if (!rfd.init(config.get_rfd_serial(),
                        config.get_rfd_serial_speed(), devices)) {
      log(LOG_ERR,
          "UV Radio Room initialization failed: cannot connect to rfd900x.");
      return false;
    }

    // Exclude the serial device used by autopilot from the device list used
    // for ISBD transceiver serial device auto-detection.
    for (vector<string>::iterator iter = devices.begin(); iter != devices.end();
         ++iter) {
      if (*iter == rfd.get_path()) {
        devices.erase(iter);
        break;
      }
    }
  }

  sms_channel.init("whatever", config.get_isbd_serial_speed(), devices);

  if (config.get_tcp_enabled()) {
    if (tcp_channel.init(config.get_tcp_host(), config.get_tcp_port())) {
      log(LOG_INFO, "TCP channel initialized.");
    } else {
      log(LOG_WARNING, "TCP channel initialization failed.");
    }
  }

  if (config.get_isbd_enabled()) {
    string isbd_serial = config.get_isbd_serial();

    if (isbd_serial == rfd.get_path() && devices.size() > 0) {
      log(LOG_WARNING,
          "rfd detected at serial device '%s' that was assigned "
          "to ISBD transceiver by the configuration settings.",
          rfd.get_path().data());

      isbd_serial = devices[0];
    }

    if (isbd_channel.init(isbd_serial, config.get_isbd_serial_speed(),
                          devices)) {
      log(LOG_INFO, "ISBD channel initialized.");
    } else {
      log(LOG_WARNING, "ISBD channel initialization failed.");
    }
  }

  log(LOG_INFO, "UV Radio Room initialization succeeded.");
  return true;
}

/**
 * Closes all opened connections.
 */
void MAVLinkHandlerGround::close() {
  tcp_channel.close();
  isbd_channel.close();
  rfd.close();
}

/**
 * One iteration of the message handler.
 *
 * NOTE: Must not use blocking calls.
 */
bool MAVLinkHandlerGround::loop() {
  mavlink_message_t mo_msg;
  mavlink_message_t mt_msg;
  bool sleep = true;

  if (rfd.receive_message(mo_msg)) {
    handle_mo_message(mo_msg);
    sleep = false; 
  }

  if (sms_channel.receive_message(mo_msg)) {
    handle_mo_message(mo_msg);
    sleep = false;
  }

  if (tcp_channel.receive_message(mt_msg)) {
    handle_mt_message(mt_msg);
    sleep = false;
  }

  update_active_channel();

  send_heartbeat();

  return sleep;
}

void MAVLinkHandlerGround::update_active_channel() {

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
void MAVLinkHandlerGround::handle_mo_message(const mavlink_message_t& msg) {
  
  tcp_channel.send_message(msg);
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
void MAVLinkHandlerGround::handle_mt_message(const mavlink_message_t& msg) {

  rfd.send_message(msg);
  if ( rfd_active ) {
    return;
  }
}

bool MAVLinkHandlerGround::send_report() {

  return false;
}

bool MAVLinkHandlerGround::send_heartbeat() {
  
  if ( rfd_active ) {
    return false;
  }

  bool ret = false;

  // heartbeat from SMS system, for air not to change to
  if ( sms_active ) {
    if ( sms_alive_timer.elapsed_time() >= sms_alive_period ) {
      sms_alive_timer.reset();

      mavlink_message_t sms_heartbeat_msg;

      mavlink_msg_heartbeat_pack(255, 0,
                               &sms_heartbeat_msg, MAV_TYPE_GCS,
                               MAV_AUTOPILOT_INVALID, 0, 0, 0);

      sms_channel.send_message(sms_heartbeat_msg);

      ret = true;
    }
  }

  
  if (heartbeat_timer.elapsed_time() >= heartbeat_period) {
    heartbeat_timer.reset();


    mavlink_message_t heartbeat_msg;
    mavlink_msg_heartbeat_pack(1, 1,
                               &heartbeat_msg, MAV_TYPE_FIXED_WING,
                               MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0);
    
    tcp_channel.send_message(heartbeat_msg);
  
    ret = true;
  }
  
  return ret;
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

}  // namespace radioroom
