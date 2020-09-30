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
#include <fcntl.h>
#include <unistd.h>

#include "MAVLinkLogger.h"

namespace mavio {

// TCP keepalive options values
constexpr int so_keepalive_value  = 1;   // enabled

MAVLinkTCPServer::MAVLinkTCPServer() : 
  _socketconnected(false), 
  socket_fd(0)

{}

MAVLinkTCPServer::~MAVLinkTCPServer() { close(); }

bool MAVLinkTCPServer::init(uint16_t port) {
  memset(&serv_addr, 0, sizeof(serv_addr));
  
  serv_addr.sin_family = AF_INET;
  
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  
  serv_addr.sin_port = htons(port);

  // return connect();

  return true;
}

bool MAVLinkTCPServer::connect() {

  // gracefully close socket just in case
  close();

  socket_fd = ::socket(AF_INET, SOCK_STREAM, 0);

  if (socket_fd < 0) {
    mavio::log(LOG_ERR,"TCP Server Socket creation failed. %s", strerror(errno));
    socket_fd = 0;
    return false;
  }

  if (::setsockopt(socket_fd, SOL_SOCKET, SO_KEEPALIVE, &so_keepalive_value,
                   sizeof(so_keepalive_value))) {
    mavio::log(LOG_ERR,"TCP Server Failed to enable TCP socket keepalive. %s",
               strerror(errno));
    return false;
  }

  if (::setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &so_keepalive_value,
                   sizeof(so_keepalive_value))) {
    mavio::log(LOG_ERR,"TCP Server Failed to enable TCP socket reuseadrr. %s",
               strerror(errno));
    return false;
  }

  //Bind
  if( bind(socket_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr) ) < 0 ) {
    mavio::log(LOG_ERR,"TCP Server Failed to bind socket. %s", strerror(errno));
    return false;
  }

  mavio::log(LOG_INFO,"TCP Server socket listening ..");

  listen(socket_fd , 3);

  return accept_connection();
}

bool MAVLinkTCPServer::accept_connection() {
  cli_len = sizeof(cli_addr);
  newsocket_fd = accept( socket_fd, (struct sockaddr *) &cli_addr, &cli_len );
  if (newsocket_fd < 0) {
    mavio::log(LOG_ERR,"TCP Server Fail to accept connection. %s", strerror(errno));
    return false;
  }
  mavio::log(LOG_INFO,"TCP Server connection accepted");
  _socketconnected = true;

  // set to non blocking
  set_blocking(newsocket_fd, true);

  return true;
}

void MAVLinkTCPServer::close() {
  if (socket_fd != 0) {
    ::shutdown(socket_fd, SHUT_RD);
    ::close(socket_fd);
    socket_fd = 0;
  }
}

bool MAVLinkTCPServer::send_message(const mavlink_message_t& msg) {
  if (newsocket_fd == 0) {
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

  uint16_t n = ::send(newsocket_fd, buf, len, 0);

  if (n == len) {
    MAVLinkLogger::log(LOG_DEBUG, "TCP Server <<", msg);
    return true;
  }

  mavio::log(LOG_ERR, "TCP Server socket send failed. %s", strerror(errno));
  mavio::log(LOG_ERR, "TCP Server socket send failed. %d", errno);

  MAVLinkLogger::log(LOG_WARNING, "TCP Server << FAILED", msg);

  // Re-connect to the socket.
  _socketconnected = false;

  return false;
}

bool MAVLinkTCPServer::receive_message(mavlink_message_t& msg) {
  if (newsocket_fd == 0) {
    return false;
  }

  if (!_socketconnected) {
    return false;
  }

  uint8_t stx;
  uint8_t payload_length;
  mavlink_status_t mavlink_status;
  int rc = ::recv(newsocket_fd, &stx, 1, MSG_WAITALL);

  if (rc > 0) {
    switch (stx) { 
        
      case 0xFE: {  

        rc = ::recv(newsocket_fd, &payload_length, 1, MSG_WAITALL);
        
        if (rc > 0) {
          uint8_t buffer[263];
          rc = ::recv(newsocket_fd, buffer, payload_length + 6, MSG_WAITALL);
          
          if (rc > 0) {

            mavlink_parse_char(MAVLINK_COMM_0, stx, &msg, &mavlink_status);
            mavlink_parse_char(MAVLINK_COMM_0, payload_length, &msg, &mavlink_status);
          
            for (int i = 0; i < rc; i++) {
              if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg,
                                   &mavlink_status)) {
                MAVLinkLogger::log(LOG_DEBUG, "TCP Server >>", msg);
                return true;
              }
            }
          }
        }
        break;
      }

      case 0xFD: {

        rc = ::recv(newsocket_fd, &payload_length, 1, MSG_WAITALL);

        if (rc > 0) {
          uint8_t buffer[263];
          rc = ::recv(newsocket_fd, buffer, payload_length + 10, MSG_WAITALL);
          
          if (rc > 0) {

            mavlink_parse_char(MAVLINK_COMM_0, stx, &msg, &mavlink_status);
            mavlink_parse_char(MAVLINK_COMM_0, payload_length, &msg, &mavlink_status);
          
            for (int i = 0; i < rc; i++) {
              if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg,
                                   &mavlink_status)) {
                MAVLinkLogger::log(LOG_DEBUG, "TCP Server >>", msg);
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
      // just the socket in non blocking mode reporting resource temporarily unavailable
      return false;
    }
    
    mavio::log(LOG_WARNING,
               "TCP Server >> Failed to receive MAVLink message from socket. %s",
               strerror(errno));
    _socketconnected = false; 

  } else if (rc == 0) {
    mavio::log(LOG_INFO,
               "TCP Server: connection closed by the client");
    _socketconnected = false;
  
  } else {
    if ( errno == EAGAIN ) {
      // just the socket in non blocking mode reporting resource temporarily unavailable
      return false;
    }
    mavio::log(LOG_WARNING, "TCP Server >> Failed to parse MAVLink message. %s",
               strerror(errno));

    // close the socket and start again, just in case. It shouldn't fail to parse mavlink
    // unless something is wrong
    _socketconnected = false;
  }

  return false;
}

bool MAVLinkTCPServer::set_blocking(int fd, bool blocking)
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
