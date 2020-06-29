/*
 MAVLinkSBD.cc

MAVIO MAVLink I/O library.

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

#include "MAVLinkSMS.h"

#include <stdio.h>

#include "MAVLinkLogger.h"

namespace mavio {

using std::string;
using std::vector;

MAVLinkSMS::MAVLinkSMS() : stream(), sms(stream) {}

MAVLinkSMS::~MAVLinkSMS() {}

bool MAVLinkSMS::detect_transceiver(string device, string pin) {
  int ret = sms.begin(pin);

  int retdeletelist = sms.deleteSMSlist();

  if (retdeletelist != GSM_SUCCESS) {
    mavio::log(LOG_WARNING,"SMS delete mesages failed, error %d", retdeletelist);
  }

  if (ret == GSM_SUCCESS || ret == GSM_ALREADY_AWAKE) {
    char model[256], imea[256];
    imea[0] = model[0] = 0;
    ret = sms.getTransceiverModel(model, sizeof(model));
    if (ret == GSM_SUCCESS) {
      ret = sms.getTransceiverSerialNumber(imea, sizeof(imea));
      if (ret == GSM_SUCCESS) {
        mavio::log(LOG_NOTICE, "%s (IMEA %s) detected at serial device '%s'.",
                   model, imea, device.data());

        initialized = true;

        return true;
      }
    }
  }

  mavio::log(
      LOG_WARNING,
      "sms transceiver not detected at serial device '%s'. Error code = %d.",
      device.data(), ret);

  initialized = false;

  return false;
}

bool MAVLinkSMS::get_signal_quality(int& quality) {
  return sms.getSignalQuality(quality) == GSM_SUCCESS;
}

bool MAVLinkSMS::init(string path, int speed, string pin, int instance) {
  if ( path.size() < 2 || pin.size() < 2 ) {
    initialized = false;
    mavio::log(LOG_NOTICE, "GSM modem %d deactivated or invalid path/pin.", instance);
    return false;    
  }
  mavio::log(LOG_NOTICE, "Connecting GSM modem %d (%s %d)...", 
                        instance, path.data(), speed);

  if (stream.open(path, speed) == 0) {
    if (detect_transceiver(path, pin)) {
      mavio::log(LOG_NOTICE, "GSM modem %d initialized", instance);
      return true;
    }

    stream.close();
  } else {
    mavio::log(LOG_INFO, "Failed to open serial device '%s'.", path.data());
  }

  mavio::log(LOG_ERR,
             "GSM modem was not detected on any of the serial devices.");

  initialized = false;

  return false;
}

void MAVLinkSMS::close() {
  stream.close();
  mavio::log(LOG_DEBUG, "GSM connection closed.");
}

bool MAVLinkSMS::get_initialized() {
  return initialized;
}

bool MAVLinkSMS::send_message_text(const mavlink_message_t& mo_msg, std::string tlf) {
  
  if (!initialized) {
    return false;
  }
  
  uint8_t buf[GSM_MAX_MT_MGS_SIZE];
  uint16_t len = 0;

  if (mo_msg.len != 0) {
    len = mavlink_msg_to_send_buffer(buf, &mo_msg);
  }

  mavio::log(LOG_INFO, "SMS: destination number: %s" , tlf.c_str());

  int ret = sms.sendSMSText(buf, len, tlf);

  if (ret != GSM_SUCCESS) {
    char prefix[32];
    snprintf(prefix, sizeof(prefix), "SMS << FAILED(%d)", ret);
    MAVLinkLogger::log(LOG_WARNING, prefix, mo_msg);
    
    return false;
  }
  MAVLinkLogger::log(LOG_INFO, "SMS <<", mo_msg);

  return true;
}

bool MAVLinkSMS::receive_message_text(mavlink_message_t& mt_msg, bool& inbox_empty, std::string& sender_number) {
  
  if (!initialized) {
    return false;
  }
  
  uint8_t buf[160];
  size_t size = sizeof(buf);
  mavlink_status_t mavlink_status;
  bool received = false;
  int ret;

  ret = sms.receiveSMSText(buf, size, inbox_empty, sender_number);
  // mavio::log(LOG_INFO, "inbox ret %d" , ret);
  // mavio::log(LOG_INFO, "buf size %d" , size);

  if ( ret == 0 ) {
    for (size_t i = 0; i < size; i++) {
      if (mavlink_parse_char(MAVLINK_COMM_2, buf[i], &mt_msg, 
                            &mavlink_status)) {
        MAVLinkLogger::log(LOG_INFO, "SMS >>", mt_msg);
        received = true;
        break;
      }
      // mavio::log(LOG_INFO, "sms parse(%d): %x", i, buf[i]);
      // mavio::log(LOG_INFO, "sms parse status: %d", mavlink_status.parse_state);
    }
  }
  if ( received ) {
    mavio::log(LOG_INFO, "SMS ^ sender number: %s" , sender_number.c_str());
    // mavio::log(LOG_INFO, "parsing ok!");
  } else {
    // mavio::log(LOG_INFO, "parsing not!");
  }
  // mavio::log(LOG_INFO, "inbox empty %d" , inbox_empty);
  return received;
}

bool MAVLinkSMS::send_message_bin(const mavlink_message_t& mo_msg, std::string tlf) {
 
  if (!initialized) {
    return false;
  }
  
  uint8_t buf[GSM_MAX_MT_MGS_SIZE];
  uint16_t len = 0;

  if (mo_msg.len != 0) {
    len = mavlink_msg_to_send_buffer(buf, &mo_msg);
  }

  int ret = sms.sendSMSBinary(buf, len, tlf);

  if (ret != GSM_SUCCESS) {
    char prefix[32];
    snprintf(prefix, sizeof(prefix), "SMS << FAILED(%d)", ret);
    MAVLinkLogger::log(LOG_WARNING, prefix, mo_msg);
    
    return false;
  }
  MAVLinkLogger::log(LOG_INFO, "SMS <<", mo_msg);

  return true;
}

bool MAVLinkSMS::receive_message_bin(mavlink_message_t& mt_msg, bool& inbox_empty, std::string& sender_number) {

  if (!initialized) {
    return false;
  }

  uint8_t buf[160];
  size_t size = sizeof(buf);
  mavlink_status_t mavlink_status;
  bool received = false;
  int ret;

  ret = sms.receiveSMSBinary(buf, size, inbox_empty, sender_number);

  if ( ret == 0 ) {
    for (size_t i = 0; i < size; i++) {
      if (mavlink_parse_char(MAVLINK_COMM_2, buf[i], &mt_msg, 
                            &mavlink_status)) {
        MAVLinkLogger::log(LOG_INFO, "SMS >>", mt_msg);
        received = true;
        break;
      }
      // mavio::log(LOG_INFO, "sms parse(%d): %x", i, buf[i]);
      // mavio::log(LOG_INFO, "sms parse status: %d", mavlink_status.parse_state);
    }
  }
  if ( received ) {
    // mavio::log(LOG_INFO, "parsing ok!");
  } else {
    // mavio::log(LOG_INFO, "parsing not!");
  }
  // mavio::log(LOG_INFO, "inbox empty %d" , inbox_empty);
  return received;
}

}  // namespace mavio
