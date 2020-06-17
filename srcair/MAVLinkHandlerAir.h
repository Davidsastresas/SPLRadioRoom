/*
 MAVLinkHandlerAir.h

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

#ifndef SRC_MAVLINKHANDLERAIR_H_
#define SRC_MAVLINKHANDLERAIR_H_

#include "MAVLinkAutopilotAir.h"
#include "MAVLinkRFD900x.h"
#include "MAVLinkISBDChannel.h"
#include "MAVLinkSMSChannel.h"
// #include "MAVLinkTCPChannel.h"
#include "MAVReport.h"
#include "timelib.h"

namespace radioroom {

// constexpr size_t max_mission_count = 1024;

/**
 * Telemetry for MAVLink autopilots.
 */
class MAVLinkHandlerAir {
 public:
  /**
   * Default constructor.
   */
  MAVLinkHandlerAir();

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

  /*
   * Handle mobile-terminated message received from a comm channel.
   */
  void handle_mt_message(const mavlink_message_t& msg);

  /**
   * Sends report message to one of the comm channels if the channel report
   * period has elapsed.
   *
   * returns true if report was sent.
   */
  bool send_report();

  /*
   * Sends heartbeat message to autopilot if hearbeat period has elapsed and
   * the comm channels are not at faulted state (one of the channels successfuly
   * sent a message during it's report period).
   *
   * This allows autopilots to handle lost link gracefully if heartbeats are not
   * received.
   */
  bool send_heartbeat();

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
  mavio::MAVLinkAutopilotAir autopilot;
  mavio::MAVLinkISBDChannel isbd_channel;
  mavio::MAVLinkSMSChannel sms_channel;

  //
  std::chrono::milliseconds current_time;

  // mavio::MAVLinkTCPChannel tcp_channel;
  timelib::Stopwatch update_active_timer;
  timelib::Stopwatch update_report_timer;
  timelib::Stopwatch heartbeat_timer;
  timelib::Stopwatch primary_report_timer;
  timelib::Stopwatch secondary_report_timer;
  MAVReport report;
  mavlink_message_t mission_count_msg;
  // mavlink_message_t missions[max_mission_count];
  size_t missions_received;

  //
  bool rfd_active;
  bool sms_active;
  bool isbd_active;

  timelib::Stopwatch retry_timer;
  mavlink_message_t retry_msg;
  std::chrono::milliseconds retry_timeout;
  int retry_count;
  bool _sleep;

  bool isbd_initialized = false;

  std::chrono::milliseconds heartbeat_period;

  std::chrono::milliseconds active_update_interval;
  std::chrono::milliseconds rfd_timeout;
  std::chrono::milliseconds sms_timeout;

  std::chrono::milliseconds sms_report_period;
  std::chrono::milliseconds isbd_report_period;
};

}  // namespace radioroom

#endif  // SRC_MAVLINKHANDLERAir_H_
