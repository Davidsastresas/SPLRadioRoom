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

MAVLinkHandlerAir::MAVLinkHandlerAir()
    : rfd(),
      autopilot(),
      isbd_channel(),
      sms_channel(),
      current_time(0),
      update_active_timer(),
      update_report_timer(),
      heartbeat_timer(),
      primary_report_timer(),
      secondary_report_timer(),
      missions_received(0),
      rfd_active(true),
      gsm_active(false),
      isbd_active(false),
      retry_timer(),
      retry_msg(),
      retry_timeout(0),
      retry_count(0),
      _sleep(false) {}

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

  if (isbd_initialized) {
    if (isbd_channel.receive_message(mt_msg)) {
      handle_mt_message(mt_msg);
      _sleep = false;
    }
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
      handle_rfd_out();
    }
  
  } else if ( gsm_active ) {
    if ( current_time - rfd.last_receive_time() <= rfd_timeout ) {
      set_rfd_active();

    } else if ( current_time - sms_channel.last_receive_time() >= sms_timeout ) {
      handle_gsm_out();
    }

  } else if ( isbd_active ) { 
    if ( current_time - rfd.last_receive_time() <= rfd_timeout ) {
      set_rfd_active();
    } else if ( current_time - sms_channel.last_receive_time() <= sms_timeout ) {
      set_gsm_active();
    }

  } else {
      set_rfd_active();
  }
}

void MAVLinkHandlerAir::handle_mo_message(const mavlink_message_t& msg) {

  rfd.send_message(msg);
  
  if ( rfd_active ) {
    return;

  } else {
    report.update(msg);
  }
}

void MAVLinkHandlerAir::handle_mt_message(const mavlink_message_t& msg) {

  autopilot.send_message(msg);
}

bool MAVLinkHandlerAir::send_report() {

  if (radio_initialized && rfd_active) {
    return false;
  }

  if (gsm_initialized && gsm_active) {
    if (primary_report_timer.elapsed_time() >= sms_report_period) {
      primary_report_timer.reset();

      mavlink_message_t sms_report_msg;
      report.get_message(sms_report_msg);

      return sms_channel.send_message(sms_report_msg);
    }
  }
  if (isbd_initialized && isbd_active) {
    if (secondary_report_timer.elapsed_time() >= isbd_report_period) {
      secondary_report_timer.reset();

      mavlink_message_t isbd_report_msg;
      report.get_message(isbd_report_msg);

      return isbd_channel.send_message(isbd_report_msg);
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

bool MAVLinkHandlerAir::set_rfd_active() {
  if (radio_initialized) {
    gsm_active = false;
    isbd_active = false;
    rfd_active = true;
    log(LOG_INFO, "RFD active");
  } else {
    rfd_active = false;
  }
  return rfd_active;
}

bool MAVLinkHandlerAir::set_gsm_active() {
  if (gsm_initialized) {
    rfd_active = false;
    isbd_active = false;
    gsm_active = true;
    sms_channel.reset_timer();
    primary_report_timer.reset();
    log(LOG_INFO, "GSM active");
  } else {
    gsm_active = false;
  }
  return gsm_active;
}

bool MAVLinkHandlerAir::set_isbd_active() {
  if (isbd_initialized) {
    rfd_active = false;
    gsm_active = false;
    isbd_active = true;
    secondary_report_timer.reset();
    log(LOG_INFO, "ISBD active");
  } else {
    isbd_active = false;
  }
  return isbd_active;
}

void MAVLinkHandlerAir::handle_rfd_out() {
  if (set_gsm_active()) {
    return;
  } else if (set_isbd_active()) {
    return;
  } else {
    set_rfd_active();
  }
}

void MAVLinkHandlerAir::handle_gsm_out() {
  if (set_isbd_active()) {
    return;
  } else {
    set_rfd_active();
  }
}

/**
 * Initializes autopilot and comm channels.
 */
bool MAVLinkHandlerAir::init() {

  vector<string> devices;
  Serial::get_serial_devices(devices);

    // ----------------- Autopilot ------------------
  if (!autopilot.init(config.get_autopilot_serial(), config.get_autopilot_serial_speed(), devices, config.get_autopilot_id())) {
    log(LOG_ERR,"UV Radio Room initialization failed: cannot connect to autopilot.");
    return false;
  }

  log(LOG_INFO,"Autopilot initialization succesful.");
  // Exclude the serial device used by autopilot 
  for (vector<string>::iterator iter = devices.begin(); iter != devices.end();
       ++iter) {
    if (*iter == autopilot.get_path()) {
      devices.erase(iter);
      break;
    }
  }

  // ----------------- RFD ------------------
  // if autopilot serial was switched, switch back now
  if (config.get_rfd_enabled()) {
    string rfd_serial_string;
    if ( autopilot.get_path() == config.get_rfd_serial() ) {
      rfd_serial_string = config.get_autopilot_serial();
    } else {
      rfd_serial_string = config.get_rfd_serial();
    }
    if (!rfd.init(rfd_serial_string, config.get_rfd_serial_speed(), devices, config.get_rfd_id())) {
      log(LOG_ERR, "UV Radio Room initialization failed: cannot connect to Radio");
      return false;
    }

    log(LOG_INFO,"Radio initialization succesful.");
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
    log(LOG_INFO,"Radio disabled.");
    radio_initialized = false;
  }

  // ------------------- GSM -------------------
  if (config.get_gsm_enabled()) {  
    if (sms_channel.init(config.get_gsm_serial(), config.get_gsm_serial_speed(), devices, config.get_gsm_pin1(), config.get_groundstation_tlf_number1())) {
      log(LOG_INFO, "GSM channel initialized.");
      gsm_initialized = true;
    } else {
      log(LOG_WARNING, "GSM channel initialization failed.");
      return false;
    }
  } else {
    log(LOG_INFO,"GSM disabled.");
    gsm_initialized = false;
  }

  // ----------------- ISBD ------------------
  if (config.get_isbd_enabled()) {  
    if (isbd_channel.init(config.get_isbd_serial(), config.get_isbd_serial_speed(), devices, config.get_groundstation_rock_address())) {
      log(LOG_INFO, "ISBD channel initialized.");
      isbd_initialized = true;
    } else {
      log(LOG_WARNING, "ISBD channel initialization failed.");
      return false;
    }
  } else {
    log(LOG_INFO,"SBD disabled");
    isbd_initialized = false;
  }

  heartbeat_period = timelib::sec2ms(1);
  active_update_interval = timelib::sec2ms(0.1);

  // configurable options, set limits

  if ( config.get_rfd_timeout() < 2 ) {
    rfd_timeout = timelib::sec2ms(2);
  } else {
    rfd_timeout = timelib::sec2ms((double)config.get_rfd_timeout());
  }

  if ( config.get_sms_timeout() < 20 ) {
    sms_timeout = timelib::sec2ms(20);
  } else {
    sms_timeout = timelib::sec2ms((double)config.get_sms_timeout());
  }

  if ( config.get_sms_period() < 14 ) {
    sms_report_period = timelib::sec2ms(14);
  } else {
    sms_report_period = timelib::sec2ms((double)config.get_sms_period());
  }

  if ( config.get_sbd_period() < 40 ) {
    isbd_report_period = timelib::sec2ms(40);
  } else {
    isbd_report_period = timelib::sec2ms((double)config.get_sbd_period());
  }

  log(LOG_INFO,"rfd_timeout %d", rfd_timeout);
  log(LOG_INFO,"sms_timeout %d", sms_timeout);
  log(LOG_INFO,"sms_report_period %d", sms_report_period);
  log(LOG_INFO,"isbd_report_period %d", isbd_report_period);

  sms_channel.reset_timer();
  rfd.reset_timer();
  
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

}  // namespace radioroom
