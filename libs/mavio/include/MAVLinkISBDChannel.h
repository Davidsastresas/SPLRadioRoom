/*
 MAVLinkISBDChannel.h

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

#ifndef LIBS_MAVIO_INCLUDE_MAVLINKISBDCHANNEL_H_
#define LIBS_MAVIO_INCLUDE_MAVLINKISBDCHANNEL_H_

#include "CircularBuffer.h"
#include "MAVLinkChannel.h"
#include "MAVLinkISBD.h"
#include "MAVLinkLib.h"

#include <atomic>
#include <string>
#include <thread>
#include <chrono>

namespace mavio {

class SBDmessage {
  public:

  SBDmessage();
  SBDmessage(mavlink_message_t msg, int address, std::chrono::milliseconds time);
  SBDmessage(mavlink_message_t msg, int address);
  ~SBDmessage();

  mavlink_message_t get_mavlink_msg();
  void set_mavlink_msg(mavlink_message_t msg);

  int get_address();
  void set_address(int address);

  std::chrono::milliseconds get_time();
  void set_time(std::chrono::milliseconds time);

  private:

  mavlink_message_t _message;
  int _address;
  std::chrono::milliseconds _receive_time;

};

class MAVLinkISBDChannel : public MAVLinkChannel {
 public:
  MAVLinkISBDChannel();
  ~MAVLinkISBDChannel();

  /**
   * Initializes connection to ISBD transceiver on the specified serial
   * device. The method automatically detects
   * the serial device if the transceiver is available on any of the serial
   * devices.
   *
   * Returns true if connection was successful.
   */
  bool init(std::string path, int speed,
            const std::vector<std::string>& devices, uint32_t remoteid);

  /*
   * Closes the serial device used to connect to ISBD.
   */
  void close();

  // dummy for abstract class 
  bool send_message(const mavlink_message_t& msg) { std::ignore = msg; return false; }

  bool send_message(SBDmessage& msg);

  // dummy for abstract class
  bool receive_message(mavlink_message_t& msg) { std::ignore = msg; return false; }

  bool receive_message(SBDmessage& msg);

  /**
   * Checks if data is available in ISBD transceiver.
   *
   * Returns true if data is available.
   */
  bool message_available();

  /**
   * Returns time of the last successfully sent message.
   */
  std::chrono::milliseconds last_send_time();

  /**
   * Returns time of the last successfully received message.
   */
  std::chrono::milliseconds last_receive_time();

    /**
   * Queries ISBD signal quality.
   * 
   * Returns true if the operation succeeded. 
   */
  bool get_signal_quality(int& quality);

 private:
  /**
   * While running is true, executes send-receive ISBD sessions.
   */
  void send_receive_task();

  MAVLinkISBD isbd;
  std::atomic<bool> running;
  // Thread of send_receive_task
  std::thread send_receive_thread;
  // Queue that buffers messages to be sent to the socket
  CircularBuffer<SBDmessage> send_queue;
  // Queue that buffers messages received from the socket
  CircularBuffer<SBDmessage> receive_queue;
  std::chrono::milliseconds send_time;  // Last send epoch time
  std::chrono::milliseconds receive_time;  // Last receive epoch time
  std::atomic<int> signal_quality;
};

}  // namespace mavio

#endif  // LIBS_MAVIO_INCLUDE_MAVLINKISBDCHANNEL_H_
