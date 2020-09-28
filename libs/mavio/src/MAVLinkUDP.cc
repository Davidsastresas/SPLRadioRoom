#include "MAVLinkUDP.h"

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

// UDP keepalive options values
constexpr int so_option_en_value  = 1;   // enabled

MAVLinkUDP::MAVLinkUDP() : 
  _socketconnected(true), 
  socket_fd(0)

{}

MAVLinkUDP::~MAVLinkUDP() { close(); }

bool MAVLinkUDP::init(uint16_t port) {
  memset(&serv_addr, 0, sizeof(serv_addr));
  
  serv_addr.sin_family = AF_INET;
  
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  
  serv_addr.sin_port = htons(port);

  return connect();
}

bool MAVLinkUDP::connect() {
  if (socket_fd > 0) {
    ::close(socket_fd);
  }

  socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);

  if (socket_fd < 0) {
    mavio::log(LOG_ERR, "Socket creation failed. %s", strerror(errno));
    socket_fd = 0;
    return false;
  }

  // if (::setsockopt(socket_fd, SOL_SOCKET, SO_KEEPALIVE || SO_REUSEADDR || SO_REUSEPORT, &so_option_en_value,
  //                  sizeof(so_option_en_value))) {
  //   mavio::log(LOG_ERR, "Failed to set udp socket options. %s",
  //              strerror(errno));
  //   return false;
  // }

  // //Bind
  // if( bind(socket_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr) ) < 0 ) {
  //   mavio::log(LOG_ERR, "Failed to bind socket. %s", strerror(errno));
  //   return false;
  // }

  // mavio::log(LOG_INFO, "UDP Socket set up succesfully, going to listen ..");

  // listen(socket_fd , 3);

  // return accept_connection();

  mavio::log(LOG_INFO, "UDP Socket succesfully created");

  return true;
}

// bool MAVLinkUDP::accept_connection() {
//   cli_len = sizeof(cli_addr);
//   newsocket_fd = accept( socket_fd, (struct sockaddr *) &cli_addr, &cli_len );
//   if (newsocket_fd < 0) {
//     mavio::log(LOG_ERR, "Fail to accept connection. %s", strerror(errno));
//     return false;
//   }
//   mavio::log(LOG_INFO, "connection accepted");
//   _socketconnected = true;
//   return true;
// }

void MAVLinkUDP::close() {
  if (socket_fd != 0) {
    ::close(socket_fd);
    socket_fd = 0;
  }
}

bool MAVLinkUDP::send_message(const mavlink_message_t& msg) {
  if (socket_fd == 0) {
    return false;
  }

  if (msg.len == 0 && msg.msgid == 0) {
    return true;
  }

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Copy the message to send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // todo, check this one
  uint16_t n = ::sendto(socket_fd, buf, len, 0, (const struct sockaddr *) &servaddr,  
            sizeof(servaddr));

  if (n == len) {
    MAVLinkLogger::log(LOG_DEBUG, "UDP <<", msg);
    return true;
  }

  mavio::log(LOG_ERR, "UDP socket send failed. %s", strerror(errno));

  MAVLinkLogger::log(LOG_WARNING, "UDP << FAILED", msg);

  // Re-connect to the socket.
  _socketconnected = false;
  connect();

  return false;
}

bool MAVLinkUDP::receive_message(mavlink_message_t& msg) {
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

            mavlink_parse_char(MAVLINK_COMM_0, stx, &msg, &mavlink_status);
            mavlink_parse_char(MAVLINK_COMM_0, payload_length, &msg, &mavlink_status);
          
            for (int i = 0; i < rc; i++) {
              if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg,
                                   &mavlink_status)) {
                MAVLinkLogger::log(LOG_DEBUG, "UDP >>", msg);
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

            mavlink_parse_char(MAVLINK_COMM_0, stx, &msg, &mavlink_status);
            mavlink_parse_char(MAVLINK_COMM_0, payload_length, &msg, &mavlink_status);
          
            for (int i = 0; i < rc; i++) {
              if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg,
                                   &mavlink_status)) {
                MAVLinkLogger::log(LOG_DEBUG, "UDP >>", msg);
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
    mavio::log(LOG_DEBUG,
               "Failed to receive MAVLink message from socket. %s",
               strerror(errno));
  } else if (rc == 0) {
    mavio::log(LOG_DEBUG,
               "UDP >> FAILED (The stream socket peer has performed an "
               "orderly shutdown)");
  } else {
    mavio::log(LOG_WARNING, "Failed to parse MAVLink message. %s",
               strerror(errno));
  }

  return false;
}

}  // namespace mavio
