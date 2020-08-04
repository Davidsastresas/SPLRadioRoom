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
    : vehicle_manager(),
      rfd_channel(),
      isbd_channel(),
      tcp_channel(),
      gsm_channel(),
      system_manager(),
      timer_send_link_status() {
}

/**
 * One iteration of the message handler.
 * No blocking calls here
 */
bool MAVLinkHandlerGround::loop() {

  mavlink_message_t msg;
  mavio::SMSmessage sms;
  mavio::SBDmessage sbdmessage;

  bool sleep = true;
  
  // incomming messages
  if (rfd_channel.receive_message(msg)) {
    handle_mo_message(msg);
    sleep = false;
  }

  if (gsm_channel.receive_message(sms)) {
    handle_mo_sms(sms);
    sleep = false;
  }

  if (isbd_channel.receive_message(sbdmessage)) {
    handle_mo_sbd(sbdmessage);
    sleep = false;
  }

  if (tcp_channel.receive_message(msg)) {
    handle_mt_message(msg);
    sleep = false;
  }

  // outgoing messages
  if (vehicle_manager.pull_queue_gsm(sms)) {
    gsm_channel.send_message(sms);
    sleep = false;
  }

  if (vehicle_manager.pull_queue_sbd(sbdmessage)) {
    isbd_channel.send_message(sbdmessage);
    sleep = false;
  }

  if (vehicle_manager.pull_queue_tcp(msg)) {
    tcp_channel.send_message(msg);
    sleep = false;
  }

  return sleep;
}

void MAVLinkHandlerGround::handle_mo_message(const mavlink_message_t& msg) {

  tcp_channel.send_message(msg);
  vehicle_manager.push_queue_rfd(msg);
}

void MAVLinkHandlerGround::handle_mo_sms(mavio::SMSmessage& sms) {

  vehicle_manager.push_queue_gsm(sms);
}

void MAVLinkHandlerGround::handle_mo_sbd(mavio::SBDmessage& sbdmessage) {

  mavlink_message_t msg = sbdmessage.get_mavlink_msg();
  tcp_channel.send_message(msg);

  vehicle_manager.push_queue_sbd(sbdmessage);
}

void MAVLinkHandlerGround::handle_mt_message(const mavlink_message_t& msg) {

  rfd_channel.send_message(msg);
  vehicle_manager.push_queue_tcp(msg);
}

void MAVLinkHandlerGround::send_gcs_signal_quality() {
    // if ( timer_send_link_status.elapsed_time() >= timer_send_link_status_period ) {
    //   mavlink_message_t sms_status_msg;
    //   if (!gsm_active) {
    //     rem_gsm_quality = 0;
    //   }
      
    //   gsm_channel.get_signal_quality(gsm_quality);
    //   mavlink_msg_radio_status_pack(249, 1, &sms_status_msg, 
    //                                 gsm_quality, rem_gsm_quality, gsm_active, 0, 0, 0, 0);

    // tcp_channel.send_message(sms_status_msg);    
    // timer_send_link_status.reset();
    // }
}

bool MAVLinkHandlerGround::init() {

  // Radio ---------------------------------------------------------------------------------------------
  
  vector<string> devices;

  if (config.get_rfd_enabled()) {  
    if (!rfd_channel.init(config.get_rfd_serial(), config.get_rfd_serial_speed(), devices, config.get_rfd_id())) {
      log(LOG_ERR, "UV Radio Room initialization failed: cannot connect to rfd900x.");
      return false;
    }
    _radio_initialized = true;
    // Exclude the serial device used by rfd
    for (vector<string>::iterator iter = devices.begin(); iter != devices.end();
         ++iter) {
      if (*iter == rfd_channel.get_path()) {
        devices.erase(iter);
        break;
      }
    }
  } else {
    log(LOG_INFO, "Radio disabled.");
    _radio_initialized = false;
  }

  // GSM -------------------------------------------------------------------------------------------------
  if (config.get_gsm_enabled()) {  
    if (gsm_channel.init(config.get_gsm_serial1(), config.get_gsm_serial2(), config.get_gsm_serial3(),
                         config.get_gsm_pin1(), config.get_gsm_pin2(), config.get_gsm_pin3(),
                         config.get_gsm_serial_speed(), false)) {
                           
      log(LOG_INFO, "SMS channel initialized.");
      _gsm_initialized = true;
    } else {
      log(LOG_WARNING, "SMS channel initialization failed.");
      return false;
    }
  } else {
    log(LOG_INFO, "GSM disabled.");
    _gsm_initialized = false;
  }

  // SBD -------------------------------------------------------------------------------------------------
  if (config.get_isbd_enabled()) {  
    if (isbd_channel.init(config.get_isbd_serial(), config.get_isbd_serial_speed(), devices, config.get_aircraft1_rock_address())) {
      log(LOG_INFO, "ISBD channel initialized.");
      _isbd_initialized = true;
      _isbd_first_contact = !config.get_isbd_get_noticed(); 
    } else {
      log(LOG_WARNING, "ISBD channel initialization failed.");
      return false;
    }
  } else {
    log(LOG_INFO, "SBD disabled.");
    _isbd_initialized = false;
  }

  // Vehicle manager -----------------------------------------------------------------------------------

  if (!vehicle_manager.init()) {
    log(LOG_ERR, "Vehicle manager initialization failed");
  } else {
    log(LOG_INFO, "Vehicle manager initialization succesfull");
  }

  // Syste manager --------------------------------------------------------------------------------------

  if (!system_manager.init()) {
    log(LOG_ERR, "System manager initialization failed");
  } else {
    log(LOG_INFO, "System manager initialization succesfull");
  }

  // TCP --------------------------------------------------------------------------------------------------
  if (tcp_channel.init(config.get_tcp_port())) {
    log(LOG_INFO, "TCP channel initialized.");
  } else {
    log(LOG_WARNING, "TCP channel initialization failed.");
    return false;
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
  gsm_channel.close();
  rfd_channel.close();
  vehicle_manager.close();
}

}  // namespace radioroom
