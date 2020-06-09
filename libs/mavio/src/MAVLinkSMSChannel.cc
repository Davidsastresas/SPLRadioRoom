/*
 MAVLinkSMSChannel.cc

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

#include "MAVLinkSMSChannel.h"

#include "timelib.h"

#include "MAVLinkLogger.h"

namespace mavio {

using std::string;
using std::vector;
using timelib::sleep;

constexpr size_t max_sms_channel_queue_size = 1024;

const std::chrono::milliseconds sms_channel_poll_interval(1000);

MAVLinkSMSChannel::MAVLinkSMSChannel()
    : MAVLinkChannel("sms"),
      sms(),
      running(false),
      send_receive_thread(),
      send_queue(max_sms_channel_queue_size),
      receive_queue(max_sms_channel_queue_size),
      send_time(0),
      receive_time(0),
      signal_quality(0) {}

MAVLinkSMSChannel::~MAVLinkSMSChannel() {}

bool MAVLinkSMSChannel::init(std::string path, int speed,
                              const vector<string>& devices) {
  bool ret = sms.init("/dev/ttyACM1", 19200, devices);

  if (ret) {
    if (!running) {
      running = true;

      std::thread send_receive_th(&MAVLinkSMSChannel::send_receive_task, this);
      send_receive_thread.swap(send_receive_th);
    }
  }
    return ret;
}

void MAVLinkSMSChannel::close() {
  if (running) {
    running = false;

    send_receive_thread.join();
  }

  sms.close();
}

bool MAVLinkSMSChannel::send_message(const mavlink_message_t& msg) {
  if (msg.len == 0 && msg.msgid == 0) {
    return true;
  }

  send_queue.push(msg);

  return true;
}

bool MAVLinkSMSChannel::receive_message(mavlink_message_t& msg) {
  return receive_queue.pop(msg);
}

bool MAVLinkSMSChannel::message_available() { return !receive_queue.empty(); }

std::chrono::milliseconds MAVLinkSMSChannel::last_send_time() {
  return send_time;
}

std::chrono::milliseconds MAVLinkSMSChannel::last_receive_time() {
  return receive_time;
}

bool MAVLinkSMSChannel::get_signal_quality(int& quality) {
  quality = signal_quality;
  return true;
}

/**
 * If there are messages in send_queue or ring alert flag of  sms transceiver
 * is up, pop send_queue, run send-receive session, and push received messages
 * to receive_queue.
 */
void MAVLinkSMSChannel::send_receive_task() {
  while (running) {
    // int quality = 0;
    // if (sms.get_signal_quality(quality)) {
    //   signal_quality = quality;
    // } else {
    //   signal_quality = 0;
    // }

    // sms.list_sms();

    if ( !send_queue.empty() ) {
      mavlink_message_t mo_msg;
      if ( !send_queue.pop(mo_msg) ) {
        mavio::log(LOG_INFO, "SMS: send_queue error!");
      } else {
        if ( sms.send_message(mo_msg) ) {
          send_time = timelib::time_since_epoch();
        } else {
          mavio::log(LOG_INFO, "SMS: error sending sms");
        }
      }
    }

    bool received = false;
    mavlink_message_t mt_msg;
    if ( sms.receive_message(mt_msg, received) ) {
      if ( received ) {
        receive_time = timelib::time_since_epoch();
        receive_queue.push(mt_msg);
      }
    }

    sleep(sms_channel_poll_interval);
  }
}

}  // namespace mavio
