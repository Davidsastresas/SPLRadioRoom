/*
 MAVReport.cc

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

#include "MAVReport.h"

namespace radioroom {

// Masks of MAVLink messages used to compose single HIGH_LATENCY message
constexpr uint16_t mavlink_msg_mask_heartbeat = 0x0001;
constexpr uint16_t mavlink_msg_mask_sys_status = 0x0002;
constexpr uint16_t mavlink_msg_mask_gps_raw_int = 0x0004;
constexpr uint16_t mavlink_msg_mask_attitude = 0x0008;
constexpr uint16_t mavlink_msg_mask_global_position_int = 0x0010;
constexpr uint16_t mavlink_msg_mask_mission_current = 0x0020;
constexpr uint16_t mavlink_msg_mask_nav_controller_output = 0x0040;
constexpr uint16_t mavlink_msg_mask_vfr_hud = 0x0080;
constexpr uint16_t mavlink_msg_mask_battery2 = 0x0100;  // optional

constexpr uint16_t mavlink_msg_mask_high_latency = 0x00FF;

inline int16_t rad_to_centidegrees(float rad) {
  return rad * 18000.0 / 3.14159265358979323846;
}

MAVReport::MAVReport() : sysid(1), compid(1), mask(0) {
  report_sms.failsafe = 0;
  report_sbd.failsafe = 0;
}

/**
 * Integrates data from the specified MAVLink message into the HIGH_LATENCY
 * message.
 */
bool MAVReport::update(const mavlink_message_t& msg) {
  if (msg.sysid > 250) {
    return false;
  }
  sysid = msg.sysid;
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:  // 0
      report_sms.base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
      report_sbd.base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
      report_sms.custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
      report_sbd.custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
      // compid = msg.compid;
      mask |= mavlink_msg_mask_heartbeat;
      return true;
    case MAVLINK_MSG_ID_SYS_STATUS:  // 1
      report_sms.battery_remaining =
          mavlink_msg_sys_status_get_battery_remaining(&msg);
      report_sbd.battery_remaining =
          mavlink_msg_sys_status_get_battery_remaining(&msg);
      report_sms.temperature =
          mavlink_msg_sys_status_get_voltage_battery(&msg) >> 8;
      report_sbd.temperature =
          mavlink_msg_sys_status_get_voltage_battery(&msg) >> 8;
      report_sms.temperature_air =
          mavlink_msg_sys_status_get_voltage_battery(&msg);
      report_sbd.temperature_air =
          mavlink_msg_sys_status_get_voltage_battery(&msg);
      report_sms.landed_state = 
          mavlink_msg_sys_status_get_current_battery(&msg) / 100;
      report_sbd.landed_state = 
          mavlink_msg_sys_status_get_current_battery(&msg) / 100;
      mask |= mavlink_msg_mask_sys_status;
      return true;
    case MAVLINK_MSG_ID_GPS_RAW_INT:  // 24
      // report_sms_sbdg.latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
      // report_sms_msg.longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
      // report_sbd.altitude_amsl = mavlink_msg_gps_raw_int_get_alt(&msg) / 1000;
      report_sms.groundspeed = mavlink_msg_gps_raw_int_get_vel(&msg) / 100;
      report_sbd.groundspeed = mavlink_msg_gps_raw_int_get_vel(&msg) / 100;
      report_sms.gps_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
      report_sbd.gps_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
      report_sms.gps_nsat = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
      report_sbd.gps_nsat = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
      mask |= mavlink_msg_mask_gps_raw_int;
      return true;
    case MAVLINK_MSG_ID_ATTITUDE:  // 30
      report_sms.heading =
          (rad_to_centidegrees(mavlink_msg_attitude_get_yaw(&msg)) + 36000) %
          36000;
      report_sbd.heading =
          (rad_to_centidegrees(mavlink_msg_attitude_get_yaw(&msg)) + 36000) %
          36000;
      report_sms.roll = rad_to_centidegrees(mavlink_msg_attitude_get_roll(&msg));
      report_sbd.roll = rad_to_centidegrees(mavlink_msg_attitude_get_roll(&msg));
      report_sms.pitch = rad_to_centidegrees(mavlink_msg_attitude_get_pitch(&msg));
      report_sbd.pitch = rad_to_centidegrees(mavlink_msg_attitude_get_pitch(&msg));
      mask |= mavlink_msg_mask_attitude;
      return true;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  // 33
      report_sms.latitude = mavlink_msg_global_position_int_get_lat(&msg);
      report_sbd.latitude = mavlink_msg_global_position_int_get_lat(&msg);
      report_sms.longitude = mavlink_msg_global_position_int_get_lon(&msg);
      report_sbd.longitude = mavlink_msg_global_position_int_get_lon(&msg);
      report_sms.altitude_amsl =
          mavlink_msg_global_position_int_get_alt(&msg) / 1000;
      report_sbd.altitude_amsl =
          mavlink_msg_global_position_int_get_alt(&msg) / 1000;
      report_sms.altitude_sp =
          mavlink_msg_global_position_int_get_relative_alt(&msg) / 1000;
      report_sbd.altitude_sp =
          mavlink_msg_global_position_int_get_relative_alt(&msg) / 1000;
      mask |= mavlink_msg_mask_global_position_int;
      return true;
    case MAVLINK_MSG_ID_MISSION_CURRENT:  // 42
      report_sms.wp_num = mavlink_msg_mission_current_get_seq(&msg);
      report_sbd.wp_num = mavlink_msg_mission_current_get_seq(&msg);
      mask |= mavlink_msg_mask_mission_current;
      return true;
    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:  // 62
      report_sms.wp_distance = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
      report_sbd.wp_distance = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
      report_sms.heading_sp =
          mavlink_msg_nav_controller_output_get_nav_bearing(&msg) * 100;
      report_sbd.heading_sp =
          mavlink_msg_nav_controller_output_get_nav_bearing(&msg) * 100;
      mask |= mavlink_msg_mask_nav_controller_output;
      return true;
    case MAVLINK_MSG_ID_VFR_HUD:  // 74
      report_sms.airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
      report_sbd.airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
      report_sms.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
      report_sbd.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
      // high_latency.heading = mavlink_msg_vfr_hud_get_heading(&msg) * 100;
      report_sms.climb_rate = mavlink_msg_vfr_hud_get_climb(&msg);
      report_sbd.climb_rate = mavlink_msg_vfr_hud_get_climb(&msg);
      report_sms.throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
      report_sbd.throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
      mask |= mavlink_msg_mask_vfr_hud;
      return true;
  }

  return false;
}

void MAVReport::get_message_sms(mavlink_message_t& msg, uint8_t sbd_quality,
                                uint8_t sms_quality1, uint8_t sms_quality2,
                                uint8_t sms_quality3, uint8_t status_bitmask,
                                uint8_t link_bitmask) {

  report_sms.comp_status_bitmask = status_bitmask;
  report_sms.rssi_gsm1 = sms_quality1;
  report_sms.rssi_gsm2 = sms_quality2;
  report_sms.rssi_gsm3 = sms_quality3;
  report_sms.rssi_sbd = sbd_quality;
  report_sms.telem_status_bitmask = link_bitmask; 

  mavlink_msg_am_telemetry_high_lat_encode(sysid, compid, &msg, &report_sms);
}

void MAVReport::get_message_sbd(mavlink_message_t& msg, uint8_t link_quality) {
  report_sbd.throttle = link_quality;
  mavlink_msg_high_latency_encode(sysid, compid, &msg, &report_sbd);
}

}  // namespace radioroom
