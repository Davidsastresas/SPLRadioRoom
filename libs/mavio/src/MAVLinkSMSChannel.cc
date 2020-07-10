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

const std::chrono::milliseconds sms_channel_poll_interval(500);

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

bool MAVLinkSMSChannel::init(std::string path1, std::string path2, std::string path3, 
                             std::string pin1, std::string pin2, std::string pin3,
                             int speed, bool pdu_enabled) {

  bool ret = sms[0].init(path1, speed, pin1, 1);
  ret += sms[1].init(path2, speed, pin2, 2);
  ret += sms[2].init(path3, speed, pin3, 3);

  initialized_instances = 0;

  for (int i = 0; i < 3; i++) {
    if (sms[i].get_initialized()) {
      initialized_instances ++;
    }
  }

  if (ret) {
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

  for (int i = 0; i < 3; i++) {
    if (sms[i].get_initialized()) {  
      sms[i].close();
    }
  }
}

bool MAVLinkSMSChannel::send_message(SMSmessage& msg) {
  if (msg.get_mavlink_msg().len == 0 && msg.get_mavlink_msg().msgid == 0) {
    return true;
  }

  send_queue.push(msg);

  return true;
}

bool MAVLinkSMSChannel::receive_message(SMSmessage& sms) {
  return receive_queue.pop(sms);
}

bool MAVLinkSMSChannel::message_available() { return !receive_queue.empty(); }

std::chrono::milliseconds MAVLinkSMSChannel::last_send_time() {
  return send_time;
}

std::chrono::milliseconds MAVLinkSMSChannel::last_receive_time() {
  return receive_time;
}

// not used
bool MAVLinkSMSChannel::get_signal_quality(int& quality) {
  quality = signal_quality;
  return true;
}

void MAVLinkSMSChannel::send_receive_task_text() {
  while (running) {
    bool sleeping = true;
    int quality = 0;
    int active_instance = 0;
    int active_instance_quality = 0;
    int i = 0;

    for (i = 0; i < 3; i ++) {
      if (!sms[i].get_initialized()) {
        continue;
      }

      if (sms[i].get_signal_quality(quality)) {
        signal_quality = quality;
      } else {
        signal_quality = 0;
      }

      if (signal_quality > active_instance_quality) {
        active_instance = i;
        active_instance_quality = signal_quality;
      }

      bool inbox_empty_instance;
      mavlink_message_t mt_msg;
      std::string sender_number;

      if ( sms[i].receive_message_text(mt_msg, inbox_empty_instance, sender_number) ) {
        
        receive_time = timelib::time_since_epoch();
        SMSmessage mt_sms(mt_msg, sender_number, timelib::time_since_epoch());
        receive_queue.push(mt_sms);

        if (!inbox_empty_instance) {
          sleeping = false;
        }
      }

    }

    signal_quality = active_instance_quality;

    if ( !send_queue.empty() ) {
      SMSmessage mo_sms;
      if ( !send_queue.pop(mo_sms) ) {
        mavio::log(LOG_INFO, "SMS: send_queue error!");
      } else {
        mavio::log(LOG_INFO, "SMS: sending message from modem %d ...", active_instance);
        if ( sms[active_instance].send_message_text(mo_sms.get_mavlink_msg(), mo_sms.get_number()) ) {
          send_time = timelib::time_since_epoch();
        } else {
          mavio::log(LOG_INFO, "SMS: error sending sms");
        }
      }
      sleeping = false;
    }

    if ( sleeping ) {
      sleep(sms_channel_poll_interval);
    }
  }
}

// we need to completely redo this. not working with multi gsm.

void MAVLinkSMSChannel::send_receive_task_pdu() {
  while (running) {
    bool inbox_empty = true;
    int quality = 0;

    for (int i = 0; i < 3; i ++) {
      if (!sms[i].get_initialized()) {
        continue;
      }

      if (sms[i].get_signal_quality(quality)) {
        signal_quality = quality;
      } else {
        signal_quality = 0;
      }
  
      if ( !send_queue.empty() ) {
        SMSmessage mo_sms;
        if ( !send_queue.pop(mo_sms) ) {
          mavio::log(LOG_INFO, "SMS: send_queue error!");
        } else {
          if ( sms[i].send_message_bin(mo_sms.get_mavlink_msg(), mo_sms.get_number()) ) {
            send_time = timelib::time_since_epoch();
          } else {
            mavio::log(LOG_INFO, "SMS: error sending sms");
          }
        }
      }
  
      mavlink_message_t mt_msg;
      std::string sender_number;
      if ( sms[i].receive_message_bin(mt_msg, inbox_empty, sender_number) ) {
        receive_time = timelib::time_since_epoch();
        SMSmessage mt_sms(mt_msg, sender_number, timelib::time_since_epoch());
        receive_queue.push(mt_sms);
      }
    
    }
    if ( inbox_empty ) {
      sleep(sms_channel_poll_interval);
    }
  }
}

// SMSmessage definitions

SMSmessage::SMSmessage() {
  
}

SMSmessage::SMSmessage(mavlink_message_t msg, std::string numb, std::chrono::milliseconds time) {
  message = msg;
  number = numb;
  receive_time = time;
}

SMSmessage::SMSmessage(mavlink_message_t msg, std::string numb) {
  message = msg;
  number = numb;
}

SMSmessage::~SMSmessage() {
  
}

mavlink_message_t SMSmessage::get_mavlink_msg() {
  return message;
}

void SMSmessage::set_mavlink_msg(mavlink_message_t msg) {
  message = msg;
}

std::string SMSmessage::get_number() {
  return number;
}

void SMSmessage::set_number(std::string numb) {
  number = numb;
}

std::chrono::milliseconds SMSmessage::get_time() {
  return receive_time;
}

void SMSmessage::set_time(std::chrono::milliseconds time) {
  receive_time = time;
}

}  // namespace mavio
