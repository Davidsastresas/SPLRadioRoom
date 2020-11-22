/*
 MAVLinkISBDChannel.cc

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

#include "MAVLinkISBDChannel.h"

#include "timelib.h"

namespace mavio {

using std::string;
using std::vector;
using timelib::sleep;

constexpr size_t max_isbd_channel_queue_size = 1024;

const std::chrono::milliseconds isbd_channel_poll_interval(10);

MAVLinkISBDChannel::MAVLinkISBDChannel()
    : MAVLinkChannel("isbd"),
      isbd(),
      running(false),
      send_receive_thread(),
      send_queue(max_isbd_channel_queue_size),
      receive_queue(max_isbd_channel_queue_size),
      send_time(0),
      receive_time(0),
      signal_quality(0) {}

MAVLinkISBDChannel::~MAVLinkISBDChannel() {}

bool MAVLinkISBDChannel::init(std::string path, int speed,
                              const vector<string>& devices, uint32_t remoteid) {
  bool ret = isbd.init(path, speed, devices, remoteid);

  if (ret) {
    if (!running) {
      running = true;

      std::thread send_receive_th(&MAVLinkISBDChannel::send_receive_task, this);
      send_receive_thread.swap(send_receive_th);
    }
  }
  return ret;
}

void MAVLinkISBDChannel::close() {
  isbd.close();

  if (running) {
    running = false;

    send_receive_thread.join();
  }
}

bool MAVLinkISBDChannel::send_message(SBDmessage& msg) {
  if (msg.get_mavlink_msg().len == 0 && msg.get_mavlink_msg().msgid == 0) {
    return true;
  }

  send_queue.push(msg);

  return true;
}

bool MAVLinkISBDChannel::receive_message(SBDmessage& msg) {
  return receive_queue.pop(msg);
}

bool MAVLinkISBDChannel::message_available() { return !receive_queue.empty(); }

std::chrono::milliseconds MAVLinkISBDChannel::last_send_time() {
  return send_time;
}

std::chrono::milliseconds MAVLinkISBDChannel::last_receive_time() {
  return receive_time;
}

bool MAVLinkISBDChannel::get_signal_quality(int& quality) {
  quality = signal_quality;
  return true;
}

/**
 * If there are messages in send_queue or ring alert flag of  ISBD transceiver
 * is up, pop send_queue, run send-receive session, and push received messages
 * to receive_queue.
 */
void MAVLinkISBDChannel::send_receive_task() {
  while (running) {
    int quality = 0;
    if (isbd.get_signal_quality(quality)) {
      signal_quality = quality;
    } else {
      signal_quality = 0;
    }

    if (!send_queue.empty() || isbd.message_available()) {
      mavlink_message_t mt_msg;
      SBDmessage mo_sbd_msg;

      if (!send_queue.pop(mo_sbd_msg)) {
        mavlink_message_t mo_msg;
        mo_msg.len = 0;
        mo_msg.msgid = 0;
        mo_sbd_msg.set_mavlink_msg(mo_msg);
      }

      bool received = false;
      if (isbd.send_receive_message(mo_sbd_msg.get_mavlink_msg(), mt_msg, received, mo_sbd_msg.get_address())) {
        send_time = timelib::time_since_epoch();
        if (received) {
          receive_time = send_time;

          SBDmessage mt_sbd_msg(mt_msg, 0, receive_time);
          receive_queue.push(mt_sbd_msg);
        }
      }
    }

    sleep(isbd_channel_poll_interval);
  }
}

// SBDmessage definitions

SBDmessage::SBDmessage() {

}

SBDmessage::SBDmessage(mavlink_message_t msg, int address, std::chrono::milliseconds time) {
  _message = msg;
  _address = address;
  _receive_time = time;
}

SBDmessage::SBDmessage(mavlink_message_t msg, int address) {
  _message = msg;
  _address = address;
}

SBDmessage::~SBDmessage() {
  
}

mavlink_message_t SBDmessage::get_mavlink_msg() {
  return _message;
}

void SBDmessage::set_mavlink_msg(mavlink_message_t msg) {
  _message = msg;
}

int SBDmessage::get_address() {
  return _address;
}

void SBDmessage::set_address(int address) {
  _address = address;
}

std::chrono::milliseconds SBDmessage::get_time() {
  return _receive_time;
}

void SBDmessage::set_time(std::chrono::milliseconds time) {
  _receive_time = time;
}

}  // namespace mavio
