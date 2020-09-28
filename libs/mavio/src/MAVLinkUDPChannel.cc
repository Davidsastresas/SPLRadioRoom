#include "MAVLinkUDPChannel.h"

#include "timelib.h"

namespace mavio {

using timelib::sleep;

constexpr size_t max_udp_channnel_queue_size = 1024;

const std::chrono::milliseconds udp_channel_sleep_interval(1000);
const std::chrono::milliseconds udp_channel_send_interval(1);
const std::chrono::milliseconds udp_channel_receive_interval(1);

MAVLinkUDPChannel::MAVLinkUDPChannel()
    : MAVLinkChannel("udpserver"),
      running(false),
      send_thread(),
      receive_thread(),
      socket(),
      send_queue(max_udp_channnel_queue_size),
      receive_queue(max_udp_channnel_queue_size),
      send_time(0),
      receive_time(0) {}

MAVLinkUDPChannel::~MAVLinkUDPChannel() { close(); }

bool MAVLinkUDPChannel::init(uint16_t port) {
  bool ret = socket.init(port);

  if (!running) {
    running = true;

    std::thread send_th(&MAVLinkUDPChannel::send_task, this);
    send_thread.swap(send_th);

    std::thread receive_th(&MAVLinkUDPChannel::receive_task, this);
    receive_thread.swap(receive_th);
  }

  return ret;
}

void MAVLinkUDPChannel::close() {
  if (running) {
    running = false;

    receive_thread.join();
    send_thread.join();
  }

  // this is what is delaying the closing of the app?
  socket.close();
}

bool MAVLinkUDPChannel::send_message(const mavlink_message_t& msg) {
  send_queue.push(msg);

  return true;
}

bool MAVLinkUDPChannel::receive_message(mavlink_message_t& msg) {
  return receive_queue.pop(msg);
}

bool MAVLinkUDPChannel::message_available() { return !receive_queue.empty(); }

std::chrono::milliseconds MAVLinkUDPChannel::last_send_time() {
  return send_time;
}

std::chrono::milliseconds MAVLinkUDPChannel::last_receive_time() {
  return receive_time;
}

void MAVLinkUDPChannel::send_task() {
  while (running) {
    mavlink_message_t msg;

    if (send_queue.pop(msg)) {
      if (socket.send_message(msg)) {
        send_time = timelib::time_since_epoch();
      }

      continue;
    }

    sleep(udp_channel_send_interval);
  }
}

void MAVLinkUDPChannel::receive_task() {
  while (running) {
    mavlink_message_t msg;

    if (socket.receive_message(msg)) {
      receive_time = timelib::time_since_epoch();
      receive_queue.push(msg);

      continue;
    }

    sleep(udp_channel_receive_interval);
  }
}

}  // namespace mavio
