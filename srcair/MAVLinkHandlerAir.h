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
#include "SystemManager.h"
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

  MAVLinkHandlerAir();

  bool init();

  void close();

  bool loop();

 private:

  void handle_mo_message(const mavlink_message_t& msg);

  void handle_mt_message(const mavlink_message_t& msg);

  void handle_mt_sms(mavio::SMSmessage& msg);

  bool send_report();

  bool send_heartbeat();

  void update_active_channel();

  bool set_rfd_active();

  bool set_gsm_active();

  bool set_isbd_active();

  void handle_rfd_out();

  void handle_gsm_out();

  void send_status_rfd();

  void update_telem_status();

  MAVReport report;
  
  // channels
  mavio::MAVLinkRFD900x rfd_channel;
  mavio::MAVLinkAutopilotAir autopilot_channel;
  mavio::MAVLinkISBDChannel sbd_channel;
  mavio::MAVLinkSMSChannel gsm_channel;
  mavio::SystemManager system_manager;

  // timers
  timelib::Stopwatch timer_update_active;
  timelib::Stopwatch heartbeat_timer;
  timelib::Stopwatch timer_report_sms;
  timelib::Stopwatch timer_report_sbd;
  timelib::Stopwatch timer_rfd_status;


  // active links
  bool rfd_active;
  bool gsm_active;
  bool isbd_active;

  // links initialization
  bool radio_initialized = false;
  bool gsm_initialized = false;
  bool isbd_initialized = false;

  // if false, first sms is sent to all 3 ground numbers
  bool ground_sms_all = false;

  // mavlink ids
  uint8_t gcs_id = 255;
  int _mav_id = 0;

  // time measurements
  std::chrono::milliseconds time_current;
  std::chrono::milliseconds time_last_sms;

  std::chrono::milliseconds timeout_rfd;
  std::chrono::milliseconds timeout_gsm;

  std::chrono::milliseconds period_autopilot_heartbeat;
  std::chrono::milliseconds period_update_act_channel;

  std::chrono::milliseconds period_sms_report;
  std::chrono::milliseconds period_sbd_report;
  std::chrono::milliseconds period_rfd_status;

  // number from last GCS SMS
  std::string last_gcs_number = "";

  // radio_status
  int sbd_quality = 0;
  int sms_quality1 = 0;
  int sms_quality2 = 0;
  int sms_quality3 = 0;
  uint8_t status_bitmask = 255; // uninitialized value
  uint8_t link_bitmask = 0;

  // gcs rb address
  int _gcs_rb_address;

};

}  // namespace radioroom

#endif  // SRC_MAVLINKHANDLERAir_H_
