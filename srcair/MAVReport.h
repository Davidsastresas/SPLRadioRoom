/*
 MAVReport.h

 Telemetry for MAVLink autopilots.

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
#ifndef SRC_MAVREPORT_H_
#define SRC_MAVREPORT_H_

#include "MAVLinkLib.h"
#include "timelib.h"
#include "MAVLinkLogger.h"

namespace radioroom {

// Class used to update and retrieve vehicle state report.
class MAVReport {
 public:
  MAVReport();

  // Integrates the specified message into report message of HIGH_LATENCY type.
  //
  // Returns true if the message was integrated.
  bool update(const mavlink_message_t& msg);

  // Retrieves HIGH_LATENCY report message
  void get_message_sms(mavlink_message_t& msg, uint8_t sbd_quality, 
                       uint8_t sms_quality1, uint8_t sms_quality2, 
                       uint8_t sms_quality3, uint8_t status_bitmask,
                       uint8_t sys_bitmask);
  
  void get_message_sbd(mavlink_message_t& msg, uint8_t link_quality);

  void zero_averages();

 private:

  mavlink_high_latency_t report_sbd;
  mavlink_am_telemetry_high_lat_t report_sms;
  uint8_t sysid;
  uint8_t compid;
  uint16_t mask;

  // average variables
  uint16_t heading_av = 0;
  uint32_t heading_acum = 0;
  uint16_t heading_samples = 0;

  int16_t  altitude_amsl_av = 0;
  int32_t  altitude_amsl_acum = 0;
  int16_t  altitude_amsl_samples = 0;

  uint8_t  current_av = 0;
  uint16_t current_acum = 0;
  uint16_t current_samples = 0;

  uint16_t voltage_av = 0;
  uint32_t voltage_acum = 0;
  uint16_t voltage_samples = 0;

  uint8_t  airspeed_av = 0;
  uint16_t airspeed_acum = 0;
  uint16_t airspeed_samples = 0;
  
  uint8_t  groundspeed_av = 0;
  uint16_t groundspeed_acum = 0;
  uint16_t groundspeed_samples = 0;

  timelib::Stopwatch timer_averages;
  std::chrono::milliseconds time_averages;


};

}  // namespace radioroom

#endif  // SRC_MAVREPORT_H_
