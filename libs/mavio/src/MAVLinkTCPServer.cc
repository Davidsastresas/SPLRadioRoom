/*
 MAVLinkTCP.cc

 MAVIO MAVLink I/O library.

 (C) Copyright 2020 Envirover.

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

#include "MAVLinkTCPServer.h"

#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "MAVLinkLogger.h"

namespace mavio {

// TCP keepalive options values
constexpr int so_keepalive_value  = 1;   // enabled

MAVLinkTCPServer::MAVLinkTCPServer() : socket_fd(0) {}

MAVLinkTCPServer::~MAVLinkTCPServer() { close(); }

bool MAVLinkTCPServer::init(const std::string address, uint16_t port) {
  memset(&serv_addr, 0, sizeof(serv_addr));
  
  serv_addr.sin_family = AF_INET;
  
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  
  serv_addr.sin_port = htons(port);

  return connect();
}

bool MAVLinkTCPServer::connect() {
  if (socket_fd > 0) {
    ::close(socket_fd);
  }

  socket_fd = ::socket(AF_INET, SOCK_STREAM, 0);

  if (socket_fd < 0) {
    mavio::log(LOG_ERR, "Socket creation failed. %s", strerror(errno));
    socket_fd = 0;
    return false;
  }

  if (::setsockopt(socket_fd, SOL_SOCKET, SO_KEEPALIVE, &so_keepalive_value,
                   sizeof(so_keepalive_value))) {
    mavio::log(LOG_ERR, "Failed to enable TCP socket keepalive. %s",
               strerror(errno));
    return false;
  }

  //Bind
  if( bind(socket_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr) ) < 0 ) {
    mavio::log(LOG_ERR, "Failed to bind socket. %s", strerror(errno));
    return false;
  }

  mavio::log(LOG_INFO, "Socket set up succesfully, going to listen ..");

  listen(socket_fd , 3);

  return accept_connection();
}

bool MAVLinkTCPServer::accept_connection() {
  cli_len = sizeof(cli_addr);
  newsocket_fd = accept( socket_fd, (struct sockaddr *) &cli_addr, &cli_len );
  if (newsocket_fd < 0) {
    mavio::log(LOG_ERR, "Fail to accept connection. %s", strerror(errno));
    return false;
  }
  mavio::log(LOG_INFO, "connection accepted");
  return true;
}

void MAVLinkTCPServer::close() {
  if (socket_fd != 0) {
    ::close(socket_fd);
    socket_fd = 0;
  }
}

bool MAVLinkTCPServer::send_message(const mavlink_message_t& msg) {
  if (newsocket_fd == 0) {
    return false;
  }

  if (msg.len == 0 && msg.msgid == 0) {
    return true;
  }

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Copy the message to send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  uint16_t n = ::send(newsocket_fd, buf, len, 0);

  if (n == len) {
    MAVLinkLogger::log(LOG_INFO, "TCP <<", msg);
    return true;
  }

  mavio::log(LOG_ERR, "TCP socket send failed. %s", strerror(errno));

  MAVLinkLogger::log(LOG_WARNING, "TCP << FAILED", msg);

  // Re-connect to the socket.
  connect();

  return false;
}

bool MAVLinkTCPServer::receive_message(mavlink_message_t& msg) {
  if (newsocket_fd == 0) {
    return false;
  }

  uint8_t stx;
  int rc = ::recv(newsocket_fd, &stx, 1, MSG_WAITALL);

  if (rc > 0) {
    if (stx != MAVLINK_STX) {
      return false;
    }

    uint8_t payload_length;
    rc = ::recv(newsocket_fd, &payload_length, 1, MSG_WAITALL);

    if (rc > 0) {
      uint8_t buffer[263];
      rc = ::recv(newsocket_fd, buffer, payload_length + 6, MSG_WAITALL);

      if (rc > 0) {
        mavlink_status_t mavlink_status;

        mavlink_parse_char(MAVLINK_COMM_0, stx, &msg, &mavlink_status);
        mavlink_parse_char(MAVLINK_COMM_0, payload_length, &msg,
                           &mavlink_status);

        for (int i = 0; i < rc; i++) {
          if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg,
                                 &mavlink_status)) {
            MAVLinkLogger::log(LOG_INFO, "TCP >>", msg);
            return true;
          }
        }
      }
    }
  }

  if (rc > 0) {
    mavio::log(LOG_DEBUG,
               "Failed to receive MAVLink message from socket. %s",
               strerror(errno));
  } else if (rc == 0) {
    mavio::log(LOG_DEBUG,
               "TCP >> FAILED (The stream socket peer has performed an "
               "orderly shutdown)");
  } else {
    mavio::log(LOG_WARNING, "Failed to parse MAVLink message. %s",
               strerror(errno));
  }

  return false;
}

}  // namespace mavio
