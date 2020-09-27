#pragma once

#include "MAVLinkLib.h"
#include <netinet/in.h>
#include <string>
#include <atomic>

namespace mavio {

/**
 * Sends/receives MAVLink messages to/from a UPD/IP socket.
 */
class MAVLinkUDPServer {
 public:
  /**
   * Constructs an instance of MAVLinkUPDServer.
   */
  MAVLinkUDPServer();

  /**
   * Closes connection and frees the resources.
   */
  virtual ~MAVLinkUDPServer();

  /**
   * Connects to the UPD/IP socket at the specified address and port.
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
   * Accepts incomming connection 
   * 
   * Return true if connection succesful
   */
  // bool accept_connection();

  bool get_connected() { return _socketconnected; }
  
 private:
  /**
   * Connects to the UPD/IP socket specified by the init() call.
   */
  bool connect();

  std::atomic<bool> _socketconnected;

  struct sockaddr_in serv_addr, cli_addr;  // Socket address
  int socket_fd, newsocket_fd;             // Socket file descriptor
  unsigned int cli_len;                    // cli lenght
};

}  // namespace mavio