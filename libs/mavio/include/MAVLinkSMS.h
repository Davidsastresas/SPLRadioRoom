/*
 MAVLinkSBD.h

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

#ifndef LIBS_MAVIO_INCLUDE_MAVLINKSMS_H_
#define LIBS_MAVIO_INCLUDE_MAVLINKSMS_H_

#include <vector>
#include <string>

#include "SMS.h"
#include "MAVLinkLib.h"

namespace mavio {

/**
 * MAVLinkSMS is used to send/receive MAVLink messages to/from an gsm
 * transceiver.
 */
class MAVLinkSMS {
 public:
  MAVLinkSMS();
  ~MAVLinkSMS();

  /**
   * Initializes connection to gsm transceiver on the specified serial
   * device. The method automatically detects
   * the serial device if the transceiver is available on any of the serial
   * devices.
   *
   * Returns true if connection was successful.
   */
  bool init(std::string path, int speed, std::string pin);

  /*
   * Closes the serial device used to connect to gsm.
   */
  void close();

  bool send_message_bin(const mavlink_message_t& mo_msg, std::string tlf);

  bool send_message_text(const mavlink_message_t& mo_msg, std::string tlf);

  bool receive_message_bin(mavlink_message_t& mt_msg, bool& inbox_empty);

  bool receive_message_text(mavlink_message_t& mt_msg, bool& inbox_empty);

  /**
   * Returns true if gsm transceiver detected at the specified serial device.
   */
  bool detect_transceiver(std::string device, std::string pin);

  /**
   * Queries gsm signal quality.
   * 
   * Returns true if the operation succeeded. 
   */
  bool get_signal_quality(int& quality);

 private:
  Serial stream;
  SMS sms;
};

}  // namespace mavio

#endif  // LIBS_MAVIO_INCLUDE_MAVLINKSMS_H_
