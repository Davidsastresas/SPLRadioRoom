/*
 MAVLinkHandlerGround.h

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
*/

#ifndef SRC_MAVLINKHANDLERGROUND_H_
#define SRC_MAVLINKHANDLERGROUND_H_

// #include "MAVLinkAutopilot.h"
#include "MAVLinkRFD900x.h"
#include "MAVLinkISBDChannel.h"
#include "MAVLinkSMSChannel.h"
#include "MAVLinkTCPChannelServer.h"
#include "MAVReport.h"
#include "timelib.h"

namespace radioroom {

// constexpr size_t max_mission_count = 1024;

/**
 * Telemetry for MAVLink autopilots.
 */
class MAVLinkHandlerGround {
 public:
  /**
   * Default constructor.
   */
  MAVLinkHandlerGround();

  /**
   * Initializes enabled comm channels and autopilot connections.
   *
   * Returns true if autopilot and enabled comm link connections were configured
   * successfully.
   */
  bool init();

  /*
   * Closes all opened connections.
   */
  void close();

  /**
   * Single turn of the main message pump that timers, routes and processes
   * messages received from autopilot and comm channels.
   *
   * The pump must run in a tight loop started after init().
   */
  bool loop();

 private:
  /*
   * Hanlde mobile-originated message received from autopilot.
   */
  void handle_mo_message(const mavlink_message_t& msg);

  void handle_mo_sms(mavio::SMSmessage& sms);

  /*
   * Handle mobile-terminated message received from a comm channel.
   */
  void handle_mt_message(const mavlink_message_t& msg);

  /*
   * Sends heartbeat message to autopilot if hearbeat period has elapsed and
   * the comm channels are not at faulted state (one of the channels successfuly
   * sent a message during it's report period).
   *
   * This allows autopilots to handle lost link gracefully if heartbeats are not
   * received.
   */
  void send_heartbeats();

  bool send_hearbeat_isbd();

  bool send_hearbeat_sms();

  bool send_hearbeat_tcp();

  bool set_rfd_active();

  bool set_gsm_active();

  bool set_isbd_active();

  void handle_rfd_out();

  void handle_gsm_out();

  /*
   * Requests autopilot data streams required to compose report message.
   */
  // void request_data_streams();

  /*
   * Sets send retry timer in milliseconds for the specified message.
   */
  void set_retry_send_timer(const mavlink_message_t& msg, 
                         const std::chrono::milliseconds& timeout,
                         int retries);

  /*
   * Cancels send retry timer for the specified message id.
   */
  void cancel_retry_send_timer(int msgid);

  /*
   * Retries sending message specifif in set_retry_send_timer() call if
   * the retry timeout elapses and the retry counter iz nonzero.
   *
   * Decrements the retries counter.
   */
  void check_retry_send_timer();

  void update_active_channel();

  mavio::MAVLinkRFD900x rfd;
  mavio::MAVLinkISBDChannel isbd_channel;
  mavio::MAVLinkTCPChannelServer tcp_channel;
  mavio::MAVLinkSMSChannel sms_channel;

  //
  std::chrono::milliseconds current_time;
  std::chrono::milliseconds last_sms_time;

  timelib::Stopwatch last_rfd_heartbeat_timer;
  timelib::Stopwatch update_report_timer;
  timelib::Stopwatch heartbeat_timer;
  timelib::Stopwatch primary_report_timer;
  timelib::Stopwatch secondary_report_timer;
  MAVReport report;
  mavlink_message_t mission_count_msg;
  // mavlink_message_t missions[max_mission_count];
  size_t missions_received;

  //
  bool rfd_active = true;
  bool gsm_active = false;
  bool isbd_active = false;

  timelib::Stopwatch retry_timer;
  mavlink_message_t retry_msg;
  std::chrono::milliseconds retry_timeout;
  int retry_count;
  bool _sleep;

  // save state of vehicle for heartbeat
  uint8_t last_base_mode = 0;
  uint32_t last_custom_mode = 0;

  bool isbd_first_contact = false;

  bool isbd_initialized = false;
  bool radio_initialized = false;
  bool gsm_initialized = false;

  // Period of heartbeat sent to GCS if no hearbeat received from radio
  std::chrono::milliseconds heartbeat_period;

// Period of checking active channel
  timelib::Stopwatch active_update_timer;
  std::chrono::milliseconds active_update_interval;

// After this time with no rfd messages it switches to GSM
  std::chrono::milliseconds rfd_timeout;

// After this time with no sms messages it switches back to SBD
  std::chrono::milliseconds sms_timeout;

  timelib::Stopwatch sms_alive_timer;
  std::chrono::milliseconds sms_alive_period; // 1 minute

  timelib::Stopwatch isbd_alive_timer;
  std::chrono::milliseconds isbd_alive_period; // 5 minutes

  std::string last_aircraft_number = "";

  mavlink_message_t last_high_latency;
};

}  // namespace radioroom

#endif  // SRC_MAVLINKHANDLERGround_H_
