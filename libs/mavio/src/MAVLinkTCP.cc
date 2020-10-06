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

#include "MAVLinkTCP.h"

#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

#include "MAVLinkLogger.h"

namespace mavio {

// TCP keepalive options values
constexpr int so_keepalive_value  = 1;   // enabled

MAVLinkTCP::MAVLinkTCP() : socket_fd(0) {

  _socketconnected = false;
}

MAVLinkTCP::~MAVLinkTCP() { close(); }

bool MAVLinkTCP::init(const std::string address, uint16_t port) {
  if (address.empty()) {
    return false;
  }

  server_addr_str = address;
  server_port = port;

  return true;
}

bool MAVLinkTCP::init(const std::string address, uint16_t port, int instance) {
  if ( init(address, port) ) {
    
    uav_instance = instance;
    return true;
  }

  return false;
}

bool MAVLinkTCP::setup_server_info() {

  struct hostent* server = ::gethostbyname(server_addr_str.c_str());

  if (server == NULL) {
    mavio::log(LOG_ERR, "TCP Vehicle %d: No such host '%s'.", uav_instance, server_addr_str.c_str());
    return false;
  }

  memset(&serv_addr, 0, sizeof(serv_addr));

  serv_addr.sin_family = AF_INET;

  memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);

  serv_addr.sin_port = htons(server_port);

  return true;
}

bool MAVLinkTCP::connect() {

  if (!setup_server_info()) {

    _socketconnected = false; // paranoid check
    return false;
  }
  // gracefully close socket just in case
  close();

  socket_fd = ::socket(AF_INET, SOCK_STREAM, 0);

  if (socket_fd < 0) {
    mavio::log(LOG_ERR, "TCP Vehicle %d: Socket creation failed. %s", uav_instance, strerror(errno));
    socket_fd = 0;
    return false;
  }

  if (::setsockopt(socket_fd, SOL_SOCKET, SO_KEEPALIVE, &so_keepalive_value,
                   sizeof(so_keepalive_value))) {
    mavio::log(LOG_ERR, "TCP Vehicle %d: Failed to enable socket keepalive. %s",
                   uav_instance, strerror(errno));
    return false;
  }

  char sin_addr_str[INET_ADDRSTRLEN];
  ::inet_ntop(AF_INET, &(serv_addr.sin_addr), sin_addr_str, INET_ADDRSTRLEN);

  if (::connect(socket_fd, reinterpret_cast<sockaddr*>(&serv_addr),
                sizeof(serv_addr)) < 0) {
    
    // This should be false, but just in case
    _socketconnected = false;

    if ( errno == ECONNREFUSED ) {
      
      // ignore this error as it is just that airpi is not nearby 
      return false;
    }

    mavio::log(LOG_ERR, "TCP Vehicle %d: Connection to 'tcp://%s:%d' failed. %s",
              uav_instance, sin_addr_str,  ntohs(serv_addr.sin_port), strerror(errno));
      
    
    return false;
  }

  mavio::log(LOG_NOTICE, "TCP Vehicle %d: Connected to 'tcp://%s:%d'.", uav_instance, 
             sin_addr_str, ntohs(serv_addr.sin_port));

  // update status of socket
  _socketconnected = true;

  // set to non blocking
  set_blocking(socket_fd, true);

  return true;
}

void MAVLinkTCP::close() {
  if (socket_fd != 0) {
    ::shutdown(socket_fd, SHUT_RD);
    ::close(socket_fd);
    socket_fd = 0;
  }
}

bool MAVLinkTCP::send_message(const mavlink_message_t& msg) {
  if (socket_fd == 0) {
    return false;
  }

  if (!_socketconnected) {
    return false;
  }

  if (msg.len == 0 && msg.msgid == 0) {
    return true;
  }

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Copy the message to send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  uint16_t n = ::send(socket_fd, buf, len, 0);

  if (n == len) {
    MAVLinkLogger::log(LOG_DEBUG, "TCP <<", msg);
    return true;
  }

  mavio::log(LOG_ERR, "TCP socket send failed. %s", strerror(errno));

  MAVLinkLogger::log(LOG_WARNING, "TCP << FAILED", msg);

  _socketconnected = false;

  return false;
}

bool MAVLinkTCP::receive_message(mavlink_message_t& msg) {
  if (socket_fd == 0) {
    return false;
  }

  if (!_socketconnected) {
    return false;
  }

  uint8_t stx;
  uint8_t payload_length;
  mavlink_status_t mavlink_status;
  int rc = ::recv(socket_fd, &stx, 1, MSG_WAITALL);

  if (rc > 0) {
    switch (stx) { 
        
      case 0xFE: {  

        rc = ::recv(socket_fd, &payload_length, 1, MSG_WAITALL);
        
        if (rc > 0) {
          uint8_t buffer[263];
          rc = ::recv(socket_fd, buffer, payload_length + 6, MSG_WAITALL);
          
          if (rc > 0) {

            mavlink_parse_char(MAVLINK_COMM_3, stx, &msg, &mavlink_status);
            mavlink_parse_char(MAVLINK_COMM_3, payload_length, &msg, &mavlink_status);
          
            for (int i = 0; i < rc; i++) {
              if (mavlink_parse_char(MAVLINK_COMM_3, buffer[i], &msg,
                                   &mavlink_status)) {
                MAVLinkLogger::log(LOG_DEBUG, "TCP >>", msg);
                return true;
              }
            }
          }
        }
        break;
      }

      case 0xFD: {

        rc = ::recv(socket_fd, &payload_length, 1, MSG_WAITALL);

        if (rc > 0) {
          uint8_t buffer[263];
          rc = ::recv(socket_fd, buffer, payload_length + 10, MSG_WAITALL);
          
          if (rc > 0) {

            mavlink_parse_char(MAVLINK_COMM_3, stx, &msg, &mavlink_status);
            mavlink_parse_char(MAVLINK_COMM_3, payload_length, &msg, &mavlink_status);
          
            for (int i = 0; i < rc; i++) {
              if (mavlink_parse_char(MAVLINK_COMM_3, buffer[i], &msg,
                                   &mavlink_status)) {
                MAVLinkLogger::log(LOG_DEBUG, "TCP >>", msg);
                return true;
              }
            }
          }
        }
        break;
      }

      default: return false;
    }
  }

  if (rc > 0) {

    if ( errno == EAGAIN ) {
      // mavio::log(LOG_WARNING, "TCP Vehicle %d >> Failed. %s",
      //          uav_instance, strerror(errno));

      // just the socket in non blocking mode reporting resource temporarily unavailable
      return false;
    }
    
    mavio::log(LOG_WARNING,
               "TCP Vehicle %d >> Failed to receive MAVLink message from socket. %s",
               uav_instance, strerror(errno));
               
    _socketconnected = false; 

  } else if (rc == 0) {
    
    mavio::log(LOG_INFO,"TCP Vehicle %d: connection closed by the client", uav_instance);

    _socketconnected = false;

  } else {
    
    if ( errno == EAGAIN ) {
      // mavio::log(LOG_WARNING, "TCP Vehicle %d >> Failed. %s",
      //          uav_instance, strerror(errno));

      // just the socket in non blocking mode reporting resource temporarily unavailable
      return false;
    }

    mavio::log(LOG_WARNING, "TCP Vehicle %d >> Failed to parse MAVLink message. %s",
               uav_instance, strerror(errno));

    // close the socket and start again, just in case. It shouldn't fail to parse mavlink
    // unless something is wrong
    _socketconnected = false;           
  }

  return false;
}

bool MAVLinkTCP::set_blocking(int fd, bool blocking)
{
  if (fd < 0) {
    mavio::log(LOG_INFO,"TCP Server fail to set_blocking");
    return false;
  }

    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
      mavio::log(LOG_INFO,"TCP Server fail to get fd flags");
      return false;
    }
    
    if (blocking) {
      flags = (flags | O_NONBLOCK);
    } else {
      flags = (flags & ~O_NONBLOCK);
    }
    return (fcntl(fd, F_SETFL, flags) == 0) ? true : false;
}

}  // namespace mavio
