#pragma once

#include "CircularBuffer.h"
#include "MAVLinkChannel.h"
#include "MAVLinkLib.h"
#include "MAVLinkUDP.h"

#include <atomic>
#include <string>
#include <thread>
#include <chrono>

namespace mavio {

/**
 * Asyncronous sends/receives MAVLink messages to/from a UDP/IP socket.
 */
class MAVLinkUDP : public MAVLinkChannel {
 public:
  /**
   * Constructs an instance of MAVLinkUDPClient.
   */
  MAVLinkUDP();

  /**
   * Closes connection and frees the resources.
   */
  virtual ~MAVLinkUDP();

  /**
   * Connects to the UDP/IP socket at the specified address and port.
   *
   * Returns true if the connection was successful.
   */
  bool init(uint16_t port);

  /**
   * Closes the connection if it was open.
   */
  void close();

  /**
   * Sends the specified MAVLink message to the socket.
   *
   * Returns true if the message was sent successfully.
   */
  bool send_message(const mavlink_message_t& msg);

  /**
   * Receives MAVLink message from the socket.
   *
   * Returns true if a message was received.
   */
  bool receive_message(mavlink_message_t& msg);

  /**
   * Checks if data is available in the socket input buffer.
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

  bool get_connected() { return socket.get_connected(); }

 private:
  /**
   * While running is true, retrieves messages from send_queue and sends them to
   * serial.
   */
  void send_task();

  /**
   *  While running is true, receives messages from serial and pushes them to
   * receive_queue.
   */
  void receive_task();

  // send and receive threads are running while this flag is true
  std::atomic<bool> running;
  // Thread of send_task
  std::thread send_thread;
  // Thread of receive_task
  std::thread receive_thread;
  // MAVLink UDP socket connection
  // MAVLinkUDP socket;
  MAVLinkUDP socket;
  // Queue that buffers messages to be sent to the socket
  CircularBuffer<mavlink_message_t> send_queue;
  // Queue that buffers messages received from the socket
  CircularBuffer<mavlink_message_t> receive_queue;
  std::chrono::milliseconds send_time;  // Last send epoch time
  std::chrono::milliseconds receive_time;  // Last receive epoch time
};

}  // namespace mavio
