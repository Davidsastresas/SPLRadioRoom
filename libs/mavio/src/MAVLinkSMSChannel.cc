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
      signal_quality(0) {

      receive_time = timelib::time_since_epoch();
}

void MAVLinkSMSChannel::reset_timer() {
  receive_time = timelib::time_since_epoch();
}

MAVLinkSMSChannel::~MAVLinkSMSChannel() {}

bool MAVLinkSMSChannel::init(std::string path, int speed, bool pdu_enabled, std::string pin, std::string numb1) {
  bool ret = sms.init(path, speed, pin);

  if (ret) {
    tlf1 = prepare_number_gsm(numb1, pdu_enabled);

    if (!running) {
      running = true;
      pdu_mode_enabled = pdu_enabled;

      if (pdu_mode_enabled) {
        std::thread send_receive_th(&MAVLinkSMSChannel::send_receive_task_pdu, this);
        send_receive_thread.swap(send_receive_th);
      } else {
        std::thread send_receive_th(&MAVLinkSMSChannel::send_receive_task_text, this);
        send_receive_thread.swap(send_receive_th);
      }
    }
  }
    return ret;
}

// we need this function to present the tlf number to gsm backend the way it is used by PDU mode
// it supports standard 11 digit international number
std::string MAVLinkSMSChannel::prepare_number_gsm(std::string number, bool pdu_enabled) {

  std::string str = "000000000000";

  // if pdu we need to do this weird switch by pairs, and pad the last with F if odd number of digits
  if (pdu_enabled) {
    for ( uint i = 0; i < number.size(); i++ ) {
      if ( !( i % 2 ) ) { // even
        str[i + 1] = number[i];
      } else { // odd
        str[i - 1] = number[i];
      }
    }

    if ( number.size() < 12 ) {
      str[10] = 'F';
    }

  // if not pdu, the number is copied as it is, and the size of the string adjusted if less than 12 digits
  } else {
    for ( uint i = 0; i < number.size(); i++) {
      str[i] = number[i];
    }
    if ( number.size() < 12) {
      str.erase(11,12);
    }
  }

  return str;
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
void MAVLinkSMSChannel::send_receive_task_pdu() {
  while (running) {
    bool inbox_empty = true;
    int quality = 0;
    if (sms.get_signal_quality(quality)) {
      signal_quality = quality;
    } else {
      signal_quality = 0;
    }

    if ( !send_queue.empty() ) {
      mavlink_message_t mo_msg;
      if ( !send_queue.pop(mo_msg) ) {
        mavio::log(LOG_INFO, "SMS: send_queue error!");
      } else {
        if ( sms.send_message_bin(mo_msg, tlf1) ) {
          send_time = timelib::time_since_epoch();
        } else {
          mavio::log(LOG_INFO, "SMS: error sending sms");
        }
      }
    }

    mavlink_message_t mt_msg;
    if ( sms.receive_message_bin(mt_msg, inbox_empty) ) {
      receive_time = timelib::time_since_epoch();
      receive_queue.push(mt_msg);
    }

    if ( inbox_empty ) {
      sleep(sms_channel_poll_interval);
    }
  }
}

void MAVLinkSMSChannel::send_receive_task_text() {
  while (running) {
    bool inbox_empty = true;
    int quality = 0;
    if (sms.get_signal_quality(quality)) {
      signal_quality = quality;
    } else {
      signal_quality = 0;
    }

    if ( !send_queue.empty() ) {
      mavlink_message_t mo_msg;
      if ( !send_queue.pop(mo_msg) ) {
        mavio::log(LOG_INFO, "SMS: send_queue error!");
      } else {
        if ( sms.send_message_text(mo_msg, tlf1) ) {
          send_time = timelib::time_since_epoch();
        } else {
          mavio::log(LOG_INFO, "SMS: error sending sms");
        }
      }
    }

    mavlink_message_t mt_msg;
    if ( sms.receive_message_text(mt_msg, inbox_empty) ) {
      receive_time = timelib::time_since_epoch();
      receive_queue.push(mt_msg);
    }

    if ( inbox_empty ) {
      sleep(sms_channel_poll_interval);
    }
  }
}

}  // namespace mavio
