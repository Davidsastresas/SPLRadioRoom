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

const std::chrono::milliseconds autopilot_send_interval(10);
const std::chrono::milliseconds heartbeat_period(1000);
const std::chrono::milliseconds autopilot_send_retry_timeout(250);

constexpr char hl_report_period_param[] = "HL_REPORT_PERIOD";

MAVLinkHandlerAir::MAVLinkHandlerAir()
    : rfd(),
      autopilot(),
      isbd_channel(),
      // tcp_channel(),
      heartbeat_timer(),
      primary_report_timer(),
      secondary_report_timer(),
      missions_received(0),
      retry_timer(),
      retry_msg(),
      retry_timeout(0),
      retry_count(0) {}

/**
 * Initializes autopilot and comm channels.
 *
 * Automatically detects the correct serial devices if autopilot and ISBD
 * transceiver do not respond on the devices specified by the configuration
 * properties.
 */
bool MAVLinkHandlerAir::init() {
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

  if (config.get_autopilot_enabled()) {
    if (!autopilot.init(config.get_autopilot_serial(),
                        config.get_autopilot_serial_speed(), devices)) {
      log(LOG_ERR,
          "UV Radio Room initialization failed: cannot connect to autopilot.");
      return false;
    }

    // Exclude the serial device used by autopilot from the device list used
    // for ISBD transceiver serial device auto-detection.
    for (vector<string>::iterator iter = devices.begin(); iter != devices.end();
         ++iter) {
      if (*iter == autopilot.get_path()) {
        devices.erase(iter);
        break;
      }
    }
  }

  // if (config.get_tcp_enabled()) {
  //   if (tcp_channel.init(config.get_tcp_host(), config.get_tcp_port())) {
  //     log(LOG_INFO, "TCP channel initialized.");
  //   } else {
  //     log(LOG_WARNING, "TCP channel initialization failed.");
  //   }
  // }

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
void MAVLinkHandlerAir::close() {
  isbd_channel.close();
  autopilot.close();
  rfd.close();
}

/**
 * One iteration of the message handler.
 *
 * NOTE: Must not use blocking calls.
 */
bool MAVLinkHandlerAir::loop() {
  mavlink_message_t msg;
  bool sleep = true;

  if (rfd.receive_message(msg)) {
    autopilot.send_message(msg);
    sleep = false; 
  }

  if (autopilot.receive_message(msg)) {
    rfd.send_message(msg);
    sleep = false;
  }
  return sleep;
}

// Forward ACK messages to the specified channel.
// Use missions array to get mission items on MISSION_REQUEST.
// Pass all other messages to update_report_msg().
void MAVLinkHandlerAir::handle_mo_message(const mavlink_message_t& msg,
                                       MAVLinkChannel& channel) {
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
void MAVLinkHandlerAir::handle_mt_message(const mavlink_message_t& msg,
                                       MAVLinkChannel& channel) {
}

bool MAVLinkHandlerAir::send_report() {
  std::chrono::milliseconds report_period =
      timelib::sec2ms(config.get_tcp_report_period());

  if (config.get_tcp_enabled() && !config.get_isbd_enabled()) {
    report_period = timelib::sec2ms(config.get_tcp_report_period());
  } else if (!config.get_tcp_enabled() && config.get_isbd_enabled()) {
    report_period = timelib::sec2ms(config.get_isbd_report_period());
  } else if (config.get_tcp_report_period() > config.get_isbd_report_period()) {
    report_period = timelib::sec2ms(config.get_isbd_report_period());
  }

  if (primary_report_timer.elapsed_time() >= report_period) {
    primary_report_timer.reset();

    // if ((report_mask & mavlink_msg_mask_high_latency) !=
    //     mavlink_msg_mask_high_latency) {
    //   log(LOG_WARNING, "Report message is incomplete. Mask = %x.",
    //   report_mask);
    // }

    mavlink_message_t report_msg;
    report.get_message(report_msg);

    // // Select the channel to send report
    // if (config.get_tcp_enabled() && !config.get_isbd_enabled()) {
    //   return tcp_channel.send_message(report_msg);
    // }

    if (!config.get_tcp_enabled() && config.get_isbd_enabled()) {
      return isbd_channel.send_message(report_msg);
    }

    // Both channels are enabled.
    // Select primary and secondary channels and secondary report period.
    // MAVLinkChannel& primary_channel = tcp_channel;
    MAVLinkChannel& secondary_channel = isbd_channel;
    std::chrono::milliseconds secondary_report_period =
        timelib::sec2ms(config.get_isbd_report_period());

    if (config.get_tcp_report_period() > config.get_isbd_report_period()) {
      // primary_channel = isbd_channel;
      // secondary_channel = tcp_channel;
      secondary_report_period = timelib::sec2ms(config.get_tcp_report_period());
    }

    // Send report to secondary channel if secondary report period elapsed
    // and messages were not successfully sent over the primary channel
    // over that period.
    // if (secondary_report_timer.elapsed_time() >= secondary_report_period) {
    //   if (timelib::time_since_epoch() - primary_channel.last_send_time() >=
    //       secondary_report_period) {
    //     secondary_report_timer.reset();
    //     return secondary_channel.send_message(report_msg);
    //   }
    // }

    // return primary_channel.send_message(report_msg);
  }

  return false;
}

bool MAVLinkHandlerAir::send_heartbeat() {
  if (heartbeat_timer.elapsed_time() >= heartbeat_period) {
    heartbeat_timer.reset();

    // Channel is healthy if it is enabled and succesfully sent report or
    // another MO message within its report period times 2.
    std::chrono::milliseconds time = timelib::time_since_epoch();

    // bool tcp_healthy = config.get_tcp_enabled() &&
    //                    (time - tcp_channel.last_send_time()) <=
    //                        2 * timelib::sec2ms(config.get_tcp_report_period());

    bool isbd_healthy =
        config.get_isbd_enabled() &&
        (time - isbd_channel.last_send_time()) <=
            2 * timelib::sec2ms(config.get_isbd_report_period());

    if (/* tcp_healthy || */ isbd_healthy) {
      mavlink_message_t heartbeat_msg;
      mavlink_msg_heartbeat_pack(mavio::gcs_system_id, mavio::gcs_component_id,
                                 &heartbeat_msg, MAV_TYPE_GCS,
                                 MAV_AUTOPILOT_INVALID, 0, 0, 0);
      return rfd.send_message(heartbeat_msg);
    }
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
