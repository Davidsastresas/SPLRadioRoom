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
    : rfd_channel(),
      autopilot_channel(),
      sbd_channel(),
      gsm_channel(),
      system_manager(),
      timer_update_active(),
      heartbeat_timer(),
      timer_report_sms(),
      timer_report_sbd(),
      timer_rfd_status() {}

bool MAVLinkHandlerAir::init() {

  vector<string> devices;
  Serial::get_serial_devices(devices);

    // ----------------- Autopilot ------------------
  if (!autopilot_channel.init(config.get_autopilot_serial(), config.get_autopilot_serial_speed(), devices, config.get_autopilot_id())) {
    log(LOG_ERR,"UV Radio Room initialization failed: cannot connect to autopilot_channel.");
    return false;
  }

  log(LOG_INFO,"Autopilot initialization succesful.");
  // Exclude the serial device used by autopilot 
  for (vector<string>::iterator iter = devices.begin(); iter != devices.end();
       ++iter) {
    if (*iter == autopilot_channel.get_path()) {
      devices.erase(iter);
      break;
    }
  }

  // ----------------- RFD ------------------
  // if autopilot serial was switched, switch back now
  if (config.get_rfd_enabled()) {
    string rfd_serial_string;
    radio_initialized = true;
    if ( autopilot_channel.get_path() == config.get_rfd_serial() ) {
      rfd_serial_string = config.get_autopilot_serial();
    } else {
      rfd_serial_string = config.get_rfd_serial();
    }
    if (!rfd_channel.init(rfd_serial_string, config.get_rfd_serial_speed(), devices, config.get_rfd_id())) {
      log(LOG_ERR, "Radio initialization failed: cannot connect to Radio");
      radio_initialized = false;
      // return false;
    } else {
      log(LOG_INFO,"Radio initialization succesful.");
    }
    
    // Exclude the serial device used by rfd 
    for (vector<string>::iterator iter = devices.begin(); iter != devices.end();
         ++iter) {
      if (*iter == rfd_channel.get_path()) {
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
    if (gsm_channel.init(config.get_gsm_serial1(), config.get_gsm_serial2(), config.get_gsm_serial3(),
                         config.get_gsm_pin1(), config.get_gsm_pin2(), config.get_gsm_pin3(),
                         config.get_gsm_serial_speed(), false)) {
      log(LOG_INFO, "GSM channel initialized.");
      gsm_initialized = true;
    } else {
      log(LOG_WARNING, "GSM channel initialization failed.");
      gsm_initialized = false;
      // return false;
    }
  } else {
    log(LOG_INFO,"GSM disabled.");
    gsm_initialized = false;
  }

  // ----------------- ISBD ------------------
  if (config.get_isbd_enabled()) {  
    if (sbd_channel.init(config.get_isbd_serial(), config.get_isbd_serial_speed(), devices, config.get_groundstation_rock_address())) {
      log(LOG_INFO, "ISBD channel initialized.");
      isbd_initialized = true;
    } else {
      log(LOG_WARNING, "ISBD channel initialization failed.");
      isbd_initialized = false;
      // return false;
    }
  } else {
    log(LOG_INFO,"SBD disabled");
    isbd_initialized = false;
  }

  // Syste manager --------------------------------------------------------------------------------------

  if (!system_manager.init()) {
    log(LOG_ERR, "System manager initialization failed");
  } else {
    log(LOG_INFO, "System manager initialization succesfull");
  }

  period_rfd_status = timelib::sec2ms(2);
  timer_rfd_status.reset();

  _mav_id = config.get_autopilot_id();

  period_autopilot_heartbeat = timelib::sec2ms(1);
  period_update_act_channel = timelib::sec2ms(0.1);

  // set gcs id for the hearbeat back to autopilot
  gcs_id = config.get_groundstation_mav_id();
  
  // send at first contact sms to all 3 numbers of ground
  ground_sms_all = false;

  // configurable options, set limits

  timeout_rfd = timelib::sec2ms(2);
  timeout_gsm = timelib::sec2ms(20);
  period_sms_report = timelib::sec2ms(14);
  period_sbd_report = timelib::sec2ms(40);
  
  if ( config.get_rfd_timeout() > 2 ) {
    timeout_rfd = timelib::sec2ms((double)config.get_rfd_timeout());
  }

  if ( config.get_sms_timeout() > 20 ) {
    timeout_gsm = timelib::sec2ms((double)config.get_sms_timeout());
  }

  if ( config.get_sms_period() > 14 ) {
    period_sms_report = timelib::sec2ms((double)config.get_sms_period());
  }

  if ( config.get_sbd_period() > 40 ) {
    period_sbd_report = timelib::sec2ms((double)config.get_sbd_period());
  }

  last_gcs_number = config.get_groundstation_tlf_number1();

  log(LOG_INFO,"GCS number %s", last_gcs_number.c_str());
  log(LOG_INFO,"timeout_rfd %d", timeout_rfd);
  log(LOG_INFO,"timeout_gsm %d", timeout_gsm);
  log(LOG_INFO,"period_sms_report %d", period_sms_report);
  log(LOG_INFO,"period_sbd_report %d", period_sbd_report);

  // gsm_channel.reset_timer();
  time_last_sms = timelib::time_since_epoch();
  rfd_channel.reset_timer();
  
  log(LOG_INFO, "UV Radio Room initialization succeeded.");

  return true;
}

void MAVLinkHandlerAir::close() {
  sbd_channel.close();
  autopilot_channel.close();
  rfd_channel.close();
  gsm_channel.close();
  system_manager.close();
}

bool MAVLinkHandlerAir::loop() {
  mavlink_message_t mo_msg;
  mavlink_message_t mt_msg;
  mavio::SMSmessage mt_sms;
  bool sleep = true;

  if (rfd_channel.receive_message(mt_msg)) {
    handle_mt_message(mt_msg);
    sleep = false; 
  }

  if (gsm_channel.receive_message(mt_sms)) {
    handle_mt_sms(mt_sms);
    sleep = false;
  }

  if (isbd_initialized) {
    if (sbd_channel.receive_message(mt_msg)) {
      handle_mt_message(mt_msg);
      sleep = false;
    }
  }

  if (autopilot_channel.receive_message(mo_msg)) {
    handle_mo_message(mo_msg);
    sleep = false;
  }

  update_active_channel();
  
  send_report();

  send_heartbeat();

  send_status_rfd();

  return sleep;
}

void MAVLinkHandlerAir::update_active_channel() {

  if (timer_update_active.elapsed_time() < period_update_act_channel) {
    return;
  }
  
  timer_update_active.reset();
  time_current = timelib::time_since_epoch();

  if ( rfd_active ) {
    if ( time_current - rfd_channel.last_receive_time() > timeout_rfd ) {
      handle_rfd_out();
    }
  
  } else if ( gsm_active ) {
    if ( time_current - rfd_channel.last_receive_time() <= timeout_rfd ) {
      set_rfd_active();

    } else if ( time_current - time_last_sms >= timeout_gsm ) {
      handle_gsm_out();
    }

  } else if ( isbd_active ) { 
    if ( time_current - rfd_channel.last_receive_time() <= timeout_rfd ) {
      set_rfd_active();
    } else if ( time_current - time_last_sms <= timeout_gsm ) {
      set_gsm_active();
    }

  } else {
      set_rfd_active();
  }
}

void MAVLinkHandlerAir::handle_mo_message(const mavlink_message_t& msg) {

  rfd_channel.send_message(msg);
  
  if ( rfd_active ) {
    return;

  } else {
    report.update(msg);
  }
}

void MAVLinkHandlerAir::handle_mt_message(const mavlink_message_t& msg) {

  autopilot_channel.send_message(msg);
}

void MAVLinkHandlerAir::send_status_rfd() {

  if ( timer_rfd_status.elapsed_time() > period_rfd_status ) {
    timer_rfd_status.reset();

    // update variables
    update_telem_status();

    // prepare message
    mavlink_message_t radio_status;
    mavlink_radio_status_t radio_status_msg;

    radio_status_msg.rxerrors = sbd_quality;
    radio_status_msg.rssi = sms_quality1;
    radio_status_msg.remrssi = sms_quality2;
    radio_status_msg.txbuf = sms_quality3;
    radio_status_msg.noise = status_bitmask;
    radio_status_msg.remnoise = link_bitmask;

    mavlink_msg_radio_status_encode(_mav_id, _mav_id + 10, &radio_status, &radio_status_msg);

    // send message
    rfd_channel.send_message(radio_status);
  }
}

bool MAVLinkHandlerAir::send_report() {

  if (radio_initialized && rfd_active) {
    return false;
  }

  if (gsm_initialized && gsm_active) {
    if (timer_report_sms.elapsed_time() >= period_sms_report) {
      timer_report_sms.reset();

      // prepare sms
      mavio::SMSmessage sms_report;
      mavlink_message_t sms_report_msg;

      // update system variables and get report
      update_telem_status();
      report.get_message_sms(sms_report_msg, sbd_quality, sms_quality1,
                             sms_quality2, sms_quality3, status_bitmask,
                             link_bitmask);
      
      // attach message to SMS
      sms_report.set_mavlink_msg(sms_report_msg);

      // send sms to all ground numbers configured for ground to pick the best signal
      if (!ground_sms_all) {
        if (config.get_groundstation_tlf_number2().size() > 5) {
          sms_report.set_number(config.get_groundstation_tlf_number2());
          gsm_channel.send_message(sms_report);
        }
        if (config.get_groundstation_tlf_number3().size() > 5) {
          sms_report.set_number(config.get_groundstation_tlf_number3());
          gsm_channel.send_message(sms_report);
        }
        if (config.get_groundstation_tlf_number1().size() > 5) {
          sms_report.set_number(config.get_groundstation_tlf_number1());
          gsm_channel.send_message(sms_report);
        }
        ground_sms_all = true;
      
      // just regular loop, send to last known number of gcs, which gcs will pick best rssi
      } else {
        sms_report.set_number(last_gcs_number);
        return gsm_channel.send_message(sms_report);
      }

    }
  }

  if (isbd_initialized && isbd_active) {
    if (timer_report_sbd.elapsed_time() >= period_sbd_report) {
      timer_report_sbd.reset();

      mavlink_message_t isbd_report_msg;
      int sbd_quality;
      sbd_channel.get_signal_quality(sbd_quality);
      report.get_message_sbd(isbd_report_msg, sbd_quality);

      return sbd_channel.send_message(isbd_report_msg);
    }
  }

  return false;
}

void MAVLinkHandlerAir::handle_mt_sms(mavio::SMSmessage& sms) {

  if (!ground_sms_all) {
    ground_sms_all = true;
  }
  mavlink_message_t msg = sms.get_mavlink_msg();
  last_gcs_number = sms.get_number();
  time_last_sms = sms.get_time();
  autopilot_channel.send_message(msg);
}

bool MAVLinkHandlerAir::send_heartbeat() {

  if ( rfd_active ) {
    return false;
  }

  if (heartbeat_timer.elapsed_time() >= period_autopilot_heartbeat) {
    heartbeat_timer.reset();

    mavlink_message_t heartbeat_msg;
    mavlink_msg_heartbeat_pack(gcs_id, mavio::gcs_component_id,
                               &heartbeat_msg, MAV_TYPE_GCS,
                               MAV_AUTOPILOT_INVALID, 0, 0, 0);

    return autopilot_channel.send_message(heartbeat_msg);
  }

  return false;
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
    // gsm_channel.reset_timer();
    time_last_sms = timelib::time_since_epoch();
    timer_report_sms.reset();
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
    // timer_report_sbd.reset();
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

void MAVLinkHandlerAir::update_telem_status() {
    // get system status
    if ( !system_manager.get_status_bitmask(status_bitmask) ) {
      status_bitmask = 255;
    }

    // get sms channel link quality and active channel
    gsm_channel.get_signal_quality(sms_quality1, 0);
    gsm_channel.get_signal_quality(sms_quality2, 1);
    gsm_channel.get_signal_quality(sms_quality3, 2);

    int active_sms = 0;
    gsm_channel.get_active_link(active_sms);

    // get sbd link quality
    sbd_quality = 0;
    sbd_channel.get_signal_quality(sbd_quality);

    // get telemetry_bitmask, 
    //
    // bit 1 rfd active
    // bit 2 gsm active
    // bit 3 sbd active
    // bit 4 gsm1 active
    // bit 5 gsm2 active
    // bit 6 gsm3 active
    // bit 7,8 unused

    link_bitmask = 0;
  
    if ( rfd_active ) {
      link_bitmask += 1;
    }
    if ( gsm_active ) {
      link_bitmask += 2;
    } 
    if ( isbd_active ) {
      link_bitmask += 4;
    }

    switch(active_sms) {
      case 0:
        link_bitmask += 8;
        break;
      case 1:
        link_bitmask += 16;
        break;
      case 2:
        link_bitmask += 32;
        break;
      default:
        break;
    }
}

}  // namespace radioroom
