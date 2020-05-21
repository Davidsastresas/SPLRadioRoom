/*
 MAVLinkTCPChannelServer.cc

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

#include "MAVLinkTCPChannelServer.h"

#include "timelib.h"

namespace mavio {

using timelib::sleep;

constexpr size_t max_tcp_channnel_queue_size = 1024;

const std::chrono::milliseconds tcp_channel_send_interval(1);
const std::chrono::milliseconds tcp_channel_receive_interval(1);

MAVLinkTCPChannelServer::MAVLinkTCPChannelServer()
    : MAVLinkChannel("tcp"),
      running(false),
      send_thread(),
      receive_thread(),
      socket(),
      send_queue(max_tcp_channnel_queue_size),
      receive_queue(max_tcp_channnel_queue_size),
      send_time(0),
      receive_time(0) {}

MAVLinkTCPChannelServer::~MAVLinkTCPChannelServer() { close(); }

bool MAVLinkTCPChannelServer::init(const std::string address, uint16_t port) {
  bool ret = socket.init(address, port);

  if (!running) {
    running = true;

    std::thread send_th(&MAVLinkTCPChannelServer::send_task, this);
    send_thread.swap(send_th);

    std::thread receive_th(&MAVLinkTCPChannelServer::receive_task, this);
    receive_thread.swap(receive_th);
  }

  return ret;
}

void MAVLinkTCPChannelServer::close() {
  if (running) {
    running = false;

    receive_thread.join();
    send_thread.join();
  }

  socket.close();
}

bool MAVLinkTCPChannelServer::send_message(const mavlink_message_t& msg) {
  send_queue.push(msg);

  return true;
}

bool MAVLinkTCPChannelServer::receive_message(mavlink_message_t& msg) {
  return receive_queue.pop(msg);
}

bool MAVLinkTCPChannelServer::message_available() { return !receive_queue.empty(); }

std::chrono::milliseconds MAVLinkTCPChannelServer::last_send_time() {
  return send_time;
}

std::chrono::milliseconds MAVLinkTCPChannelServer::last_receive_time() {
  return receive_time;
}

void MAVLinkTCPChannelServer::send_task() {
  while (running) {
    mavlink_message_t msg;

    if (send_queue.pop(msg)) {
      if (socket.send_message(msg)) {
        send_time = timelib::time_since_epoch();
      }

      continue;
    }

    sleep(tcp_channel_send_interval);
  }
}

void MAVLinkTCPChannelServer::receive_task() {
  while (running) {
    mavlink_message_t msg;

    if (socket.receive_message(msg)) {
      receive_time = timelib::time_since_epoch();
      receive_queue.push(msg);

      continue;
    }

    sleep(tcp_channel_receive_interval);
  }
}

}  // namespace mavio
