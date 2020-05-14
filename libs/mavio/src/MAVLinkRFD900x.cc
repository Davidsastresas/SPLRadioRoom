/*
 MAVLinkRFD900x.cc
 
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

#include "MAVLinkRFD900x.h"

#include "timelib.h"

#include "MAVLinkLogger.h"

namespace mavio {

using std::string;
using std::vector;
using timelib::sleep;
using timelib::Stopwatch;

constexpr int send_retries = 5;
constexpr int receive_retries = 10;

const std::chrono::milliseconds max_heartbeat_interval(2000);
const std::chrono::milliseconds autopilot_send_interval(10);
const std::chrono::milliseconds receive_retry_delay(10);

constexpr size_t max_autopilot_queue_size = 1024;

MAVLinkRFD900x::MAVLinkRFD900x()
    : MAVLinkChannel("RFD900x"),
      running(false),
      send_thread(),
      receive_thread(),
      serial(),
      send_queue(max_autopilot_queue_size),
      receive_queue(max_autopilot_queue_size),
      system_id(0),
      send_time(0),
      receive_time(0) {}

bool MAVLinkRFD900x::init(const string& path, int speed,
                            const vector<string>& devices) {
  bool ret = connect(path, speed, devices);

  if (!running) {
    running = true;

    std::thread send_th(&MAVLinkRFD900x::send_task, this);
    send_thread.swap(send_th);

    std::thread receive_th(&MAVLinkRFD900x::receive_task, this);
    receive_thread.swap(receive_th);
  }

  return ret;
}

void MAVLinkRFD900x::close() {
  if (running) {
    running = false;

    receive_thread.join();
    send_thread.join();
  }

  serial.close();
}

// bool MAVLinkRFD900x::request_autopilot_version(
//     uint8_t& autopilot, uint8_t& mav_type, uint8_t& sys_id,
//     mavlink_autopilot_version_t& autopilot_version) {
//   mavlink_message_t msg, msg_command_long;
//   autopilot = mav_type = sys_id = 0;
//   memset(&autopilot_version, 0, sizeof(autopilot_version));

//   Stopwatch timer;

//   while (timer.elapsed_time() < max_heartbeat_interval) {
//     if (serial.receive_message(msg)) {
//       if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
//         autopilot = mavlink_msg_heartbeat_get_autopilot(&msg);
//         mav_type = mavlink_msg_heartbeat_get_type(&msg);
//         sys_id = msg.sysid;

//         if (autopilot != MAV_AUTOPILOT_INVALID)  // Filter out heartbeat
//                                                  // messages forwarded from GCS
//           break;
//       }
//     }

//     sleep(receive_retry_delay);
//   }

//   // Return false if heartbeat message was not received
//   if (sys_id == 0) {
//     mavio::log(LOG_DEBUG, "Heartbeat not received.\n");
//     return false;
//   }

//   for (int i = 0; i < send_retries; i++) {
//     mavlink_msg_command_long_pack(
//         gcs_system_id, gcs_component_id, &msg_command_long, sys_id,
//         ardupilot_component_id, MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, i, 1.0,
//         0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//     if (serial.send_message(msg_command_long)) {
//       for (int j = 0; j < receive_retries; j++) {
//         if (serial.receive_message(msg)) {
//           // printf("**** msg.msgid = %d\n", msg.msgid);
//           if (msg.msgid == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
//             mavlink_msg_autopilot_version_decode(&msg, &autopilot_version);
//             sys_id = msg.sysid;
//             return true;
//           }
//         }
//       }
//     } else {
//       mavio::log(LOG_DEBUG, "Failed to send message to autopilot.\n");
//     }

//     sleep(receive_retry_delay);
//   }

//   return true;
// // }

// char* MAVLinkRFD900x::get_firmware_version(
//     const mavlink_autopilot_version_t& autopilot_version, char* buff,
//     size_t buff_size) {
//   strncpy(buff, "unknown", buff_size);

//   if (autopilot_version.flight_sw_version != 0) {
//     int majorVersion, minorVersion, patchVersion;
//     FIRMWARE_VERSION_TYPE versionType;

//     majorVersion = (autopilot_version.flight_sw_version >> (8 * 3)) & 0xFF;
//     minorVersion = (autopilot_version.flight_sw_version >> (8 * 2)) & 0xFF;
//     patchVersion = (autopilot_version.flight_sw_version >> (8 * 1)) & 0xFF;
//     versionType = (FIRMWARE_VERSION_TYPE)(
//         (autopilot_version.flight_sw_version >> (8 * 0)) & 0xFF);

//     snprintf(buff, buff_size, "%d.%d.%d/%d ", majorVersion, minorVersion,
//              patchVersion, versionType);
//   }

//   return buff;
// }

bool MAVLinkRFD900x::send_message(const mavlink_message_t& msg) {
  send_queue.push(msg);
  return true;
}

bool MAVLinkRFD900x::receive_message(mavlink_message_t& msg) {
  return receive_queue.pop(msg);
}

bool MAVLinkRFD900x::message_available() { return !receive_queue.empty(); }

std::chrono::milliseconds MAVLinkRFD900x::last_send_time() {
  return send_time;
}

std::chrono::milliseconds MAVLinkRFD900x::last_receive_time() {
  return receive_time;
}

bool MAVLinkRFD900x::connect(const string& path, int speed,
                               const vector<string>& devices) {
  mavio::log(LOG_NOTICE, "Connecting to RFD900x (%s %d)...", path.data(),
             speed);

  if (serial.init(path, speed)) {
    mavio::log(LOG_NOTICE, "RFD900x serial port opened succesfully");
    return true;
  } else {
    mavio::log(LOG_WARNING, "Failed to open serial rfd900x device '%s'.", path.data());
    serial.close();
    return false;
  }
}

// uint8_t MAVLinkRFD900x::detect_autopilot(const string device) {
//   mavlink_autopilot_version_t autopilot_version;
//   uint8_t autopilot, mav_type, sys_id;

//   mavio::log(LOG_NOTICE, "Detecting autopilot at serial device '%s'...",
//              device.data());

//   if (!request_autopilot_version(autopilot, mav_type, sys_id,
//                                  autopilot_version)) {
//     mavio::log(LOG_DEBUG, "Autopilot not detected at serial device '%s'.",
//                device.data());
//     return 0;
//   }

//   char buff[64];
//   get_firmware_version(autopilot_version, buff, sizeof(buff));

//   mavio::log(LOG_NOTICE, "Autopilot detected at serial device '%s'.",
//              device.data());
//   mavio::log(
//       LOG_NOTICE,
//       "MAV type: %d, system id: %d, autopilot class: %d, firmware version: %s",
//       mav_type, sys_id, autopilot, buff);

//   return sys_id;
// }

void MAVLinkRFD900x::send_task() {
  while (running) {
    mavlink_message_t msg;

    if (send_queue.pop(msg)) {
      if (serial.send_message(msg, true)) {
        send_time = timelib::time_since_epoch();
      }
    }

    timelib::sleep(autopilot_send_interval);
  }
}

void MAVLinkRFD900x::receive_task() {
  while (running) {
    mavlink_message_t msg;

    if (serial.receive_message(msg, true)) {
      receive_time = timelib::time_since_epoch();
      receive_queue.push(msg);
    }
  }
}

}  // namespace mavio
