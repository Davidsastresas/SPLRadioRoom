/*
 Config.cc

 This file is a part of RadioRoom project.

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

#include "Config.h"
#include "INIReader.h"

namespace radioroom {

Config config;

constexpr bool default_enabled = false;
constexpr int default_int = 0;
constexpr char default_string[] = "0";

// radioroom.conf properties
constexpr char autopilot_config_section[] = "autopilot";
constexpr char autopilot_serial_property[] = "serial";
constexpr char autopilot_serial_speed_property[] = "serial_speed";
constexpr char autopilot_id_property[] = "vehicle_id";

constexpr char rfd_config_section[] = "rfd";
constexpr char rfd_serial_property[] = "serial";
constexpr char rfd_serial_speed_property[] = "serial_speed";
constexpr char rfd_id_property[] = "radio_id";

constexpr char radioroom_config_section[] = "radioroom";

constexpr char isbd_config_section[] = "isbd";
constexpr char isbd_serial_property[] = "serial";
constexpr char isbd_serial_speed_property[] = "serial_speed";
constexpr char isbd_get_noticed_property[] = "get_noticed";

constexpr char gsm_config_section[] = "gsm";
constexpr char gsm_serial_property[] = "serial";
constexpr char gsm_serial_speed_property[] = "serial_speed";
constexpr char gsm_pin1_property[] = "pin1";
constexpr char gsm_pin2_property[] = "pin2";
constexpr char gsm_pin3_property[] = "pin3";

constexpr char tcp_config_section[] = "tcp";
constexpr char tcp_port_property[] = "port";

constexpr char misc_config_section[] = "misc";
constexpr char debug_mode_property[] = "debug";
constexpr char rfd_timeout_property[] = "rfd_timeout";
constexpr char sms_timeout_property[] = "sms_timeout";
constexpr char sms_period_property[] = "sms_period";
constexpr char sbd_period_property[] = "sbd_period";
constexpr char heartbeat_period_property[] = "heartbeat_period";
constexpr char sms_heartbeat_period_property[] = "sms_heartbeat_period";

constexpr char groundstation_config_section[] = "groundstation";
constexpr char aircraft1_config_section[] = "aircraft1";
constexpr char aircraft2_config_section[] = "aircraft2";
constexpr char aircraft3_config_section[] = "aircraft3";
constexpr char aircraft4_config_section[] = "aircraft4";
constexpr char aircraft5_config_section[] = "aircraft5";

constexpr char enabled[] = "enabled";
constexpr char tlf_number1[] = "tlf_number1";
constexpr char tlf_number2[] = "tlf_number2";
constexpr char tlf_number3[] = "tlf_number3";
constexpr char rock_address[] = "rock_address";
constexpr char mav_id[] = "mav_id";

// TODO we may need to check this out
Config::Config() {}

int Config::init(const std::string& config_file) {
  INIReader conf(config_file);

  int ret = conf.ParseError();

  if (ret < 0) {
    return ret;
  }

  /* [autopilot] config section */

  set_autopilot_serial(conf.Get(autopilot_config_section,
                       autopilot_serial_property, default_string));

  set_autopilot_serial_speed(conf.GetInteger(autopilot_config_section,
                             autopilot_serial_speed_property, default_int));

  set_autopilot_id(conf.GetInteger(autopilot_config_section, autopilot_id_property,
                                    default_int));

  /* [rfd] config section */

  set_rfd_serial(conf.Get(rfd_config_section,
                  rfd_serial_property, default_string));

  set_rfd_serial_speed(conf.GetInteger(rfd_config_section,
                       rfd_serial_speed_property, default_int));

  set_rfd_id(conf.GetInteger(rfd_config_section, rfd_id_property,
                                    default_int));

  set_rfd_enabled(conf.GetBoolean(rfd_config_section, enabled,
                                    default_enabled));

  /* [isbd] config section */

  set_isbd_serial(conf.Get(isbd_config_section, isbd_serial_property,
                  default_string));

  set_isbd_serial_speed(conf.GetInteger(isbd_config_section, 
                        isbd_serial_speed_property, default_int));

  set_isbd_enabled(conf.GetBoolean(isbd_config_section, enabled,
                                    default_enabled));

  set_isbd_get_noticed(conf.GetBoolean(isbd_config_section, isbd_get_noticed_property,
                                          true));

  /* [gsm] config section */

  set_gsm_serial(conf.Get(gsm_config_section, gsm_serial_property, default_string));

  set_gsm_serial_speed(conf.GetInteger(
      gsm_config_section, gsm_serial_speed_property, default_int));

  set_gsm_pin1(conf.Get(gsm_config_section, gsm_pin1_property, default_string));

  set_gsm_pin2(conf.Get(gsm_config_section, gsm_pin2_property, default_string));

  set_gsm_pin3(conf.Get(gsm_config_section, gsm_pin3_property, default_string));

  set_gsm_enabled(conf.GetBoolean(gsm_config_section, enabled, default_enabled));

  /* [tcp] config section */

  set_tcp_port(
      conf.GetInteger(tcp_config_section, tcp_port_property, default_int));

  /* [misc] config section */

  //common
  set_debug_mode(conf.GetBoolean(misc_config_section, debug_mode_property, default_enabled));

  set_rfd_timeout(conf.GetInteger(misc_config_section, rfd_timeout_property, default_int));

  set_sms_timeout(conf.GetInteger(misc_config_section, sms_timeout_property, default_int));

  // air
  set_sms_period(conf.GetInteger(misc_config_section, sms_period_property, default_int));

  set_sbd_period(conf.GetInteger(misc_config_section, sbd_period_property, default_int));
  
  // ground
  set_heartbeat_period(conf.GetInteger(misc_config_section, heartbeat_period_property, default_int));

  set_sms_heartbeat_period(conf.GetInteger(misc_config_section, sms_heartbeat_period_property, default_int));

  /* [groundstation] config section */ 

  set_groundstation_tlf_number1(conf.Get(groundstation_config_section,
                                tlf_number1,
                                default_string));
  
  set_groundstation_tlf_number2(conf.Get(groundstation_config_section,
                                tlf_number2,
                                default_string));
  
  set_groundstation_tlf_number3(conf.Get(groundstation_config_section,
                                tlf_number3,
                                default_string));

  set_groundstation_rock_address(conf.GetInteger(groundstation_config_section,
                                rock_address,
                                default_int));

  set_groundstation_mav_id(conf.GetInteger(
        groundstation_config_section, mav_id, default_int));

  /* [aircraft1] config section */ 

  set_aircraft1_enabled(conf.GetBoolean(aircraft1_config_section, enabled,
                                  default_enabled));

  set_aircraft1_tlf_number1(conf.Get(aircraft1_config_section,
                                tlf_number1,
                                default_string));
  
  set_aircraft1_tlf_number2(conf.Get(aircraft1_config_section,
                                tlf_number2,
                                default_string));
  
  set_aircraft1_tlf_number3(conf.Get(aircraft1_config_section,
                                tlf_number3,
                                default_string));

  set_aircraft1_rock_address(conf.GetInteger(aircraft1_config_section,
                                rock_address,
                                default_int));

  set_aircraft1_mav_id(conf.GetInteger(
        aircraft1_config_section, mav_id, default_int));

  /* [aircraft2] config section */ 

  set_aircraft2_enabled(conf.GetBoolean(aircraft2_config_section, enabled,
                                  default_enabled));

  set_aircraft2_tlf_number1(conf.Get(aircraft2_config_section,
                                tlf_number1,
                                default_string));
  
  set_aircraft2_tlf_number2(conf.Get(aircraft2_config_section,
                                tlf_number2,
                                default_string));
  
  set_aircraft2_tlf_number3(conf.Get(aircraft2_config_section,
                                tlf_number3,
                                default_string));

  set_aircraft2_rock_address(conf.GetInteger(aircraft2_config_section,
                                rock_address,
                                default_int));

  set_aircraft2_mav_id(conf.GetInteger(
        aircraft2_config_section, mav_id, default_int));

  /* [aircraft3] config section */ 

  set_aircraft3_enabled(conf.GetBoolean(aircraft3_config_section, enabled,
                                  default_enabled));

  set_aircraft3_tlf_number1(conf.Get(aircraft3_config_section,
                                tlf_number1,
                                default_string));
  
  set_aircraft3_tlf_number2(conf.Get(aircraft3_config_section,
                                tlf_number2,
                                default_string));
  
  set_aircraft3_tlf_number3(conf.Get(aircraft3_config_section,
                                tlf_number3,
                                default_string));

  set_aircraft3_rock_address(conf.GetInteger(aircraft3_config_section,
                                rock_address,
                                default_int));

  set_aircraft3_mav_id(conf.GetInteger(
        aircraft3_config_section, mav_id, default_int));

  /* [aircraft4] config section */ 

  set_aircraft4_enabled(conf.GetBoolean(aircraft4_config_section, enabled,
                                  default_enabled));

  set_aircraft4_tlf_number1(conf.Get(aircraft4_config_section,
                                tlf_number1,
                                default_string));
  
  set_aircraft4_tlf_number2(conf.Get(aircraft4_config_section,
                                tlf_number2,
                                default_string));
  
  set_aircraft4_tlf_number3(conf.Get(aircraft4_config_section,
                                tlf_number3,
                                default_string));

  set_aircraft4_rock_address(conf.GetInteger(aircraft4_config_section,
                                rock_address,
                                default_int));

  set_aircraft4_mav_id(conf.GetInteger(
        aircraft4_config_section, mav_id, default_int));
  
  /* [aircraft5] config section */ 
  set_aircraft5_enabled(conf.GetBoolean(aircraft5_config_section, enabled,
                                  default_enabled));
  
  set_aircraft5_tlf_number1(conf.Get(aircraft5_config_section,
                                tlf_number1,
                                default_string));
  
  set_aircraft5_tlf_number2(conf.Get(aircraft5_config_section,
                                tlf_number2,
                                default_string));
  
  set_aircraft5_tlf_number3(conf.Get(aircraft5_config_section,
                                tlf_number3,
                                default_string));
  set_aircraft5_rock_address(conf.GetInteger(aircraft5_config_section,
                                rock_address,
                                default_int));
  set_aircraft5_mav_id(conf.GetInteger(
        aircraft5_config_section, mav_id, default_int));

  return 0;
}

/* Autopilot */

std::string Config::get_autopilot_serial() const { return autopilot_serial; }
void Config::set_autopilot_serial(const std::string& path) {autopilot_serial = path;}

int Config::get_autopilot_serial_speed() const {return autopilot_serial_speed;}
void Config::set_autopilot_serial_speed(int speed) {autopilot_serial_speed = speed;}

int Config::get_autopilot_id() const {return autopilot_id;}
void Config::set_autopilot_id(int id) {autopilot_id = id;}

/* RFD */

std::string Config::get_rfd_serial() const { return rfd_serial; }
void Config::set_rfd_serial(const std::string& path) {rfd_serial = path;}

int Config::get_rfd_serial_speed() const {return rfd_serial_speed;}
void Config::set_rfd_serial_speed(int speed) {rfd_serial_speed = speed;}

int Config::get_rfd_id() const {return rfd_id;}
void Config::set_rfd_id(int id) {rfd_id = id;}

bool Config::get_rfd_enabled() const {return rfd_enabled;}
void Config::set_rfd_enabled(bool enabled) {rfd_enabled = enabled;}

/* ISBD */

int Config::get_isbd_serial_speed() const { return isbd_serial_speed; }
void Config::set_isbd_serial_speed(int speed) { isbd_serial_speed = speed; }

std::string Config::get_isbd_serial() const { return isbd_serial; }
void Config::set_isbd_serial(const std::string& path) { isbd_serial = path; }

bool Config::get_isbd_enabled() const {return isbd_enabled;}
void Config::set_isbd_enabled(bool enabled) {isbd_enabled = enabled;}

bool Config::get_isbd_get_noticed() const {return isbd_get_noticed;}
void Config::set_isbd_get_noticed(bool get_noticed) {isbd_get_noticed = get_noticed;}

/* GSM */

int Config::get_gsm_serial_speed() const { return gsm_serial_speed; }
void Config::set_gsm_serial_speed(int speed) { gsm_serial_speed = speed; }

std::string Config::get_gsm_serial() const { return gsm_serial; }
void Config::set_gsm_serial(const std::string& path) { gsm_serial = path; }

std::string Config::get_gsm_pin1() const { return gsm_pin1; }
void Config::set_gsm_pin1(const std::string& path) { gsm_pin1 = path; }

std::string Config::get_gsm_pin2() const { return gsm_pin2; }
void Config::set_gsm_pin2(const std::string& path) { gsm_pin2 = path; }

std::string Config::get_gsm_pin3() const { return gsm_pin3; }
void Config::set_gsm_pin3(const std::string& path) { gsm_pin3 = path; }

bool Config::get_gsm_enabled() const {return gsm_enabled;}
void Config::set_gsm_enabled(bool enabled) {gsm_enabled = enabled;}

/* TCP */

int Config::get_tcp_port() const { return tcp_port; }
void Config::set_tcp_port(int port) { tcp_port = port; }

/* misc configuration properties */

// common
bool Config::get_debug_mode() const { return debug;}
void Config::set_debug_mode(bool deb) { debug = deb; }

int Config::get_rfd_timeout() const { return rfd_timeout;}
void Config::set_rfd_timeout(int timeout) {rfd_timeout = timeout;}

int Config::get_sms_timeout() const { return sms_timeout;}
void Config::set_sms_timeout(int timeout) {sms_timeout = timeout;}

// air
int Config::get_sms_period() const { return sms_period;}
void Config::set_sms_period(int period) {sms_period = period;}

int Config::get_sbd_period() const { return sbd_period;}
void Config::set_sbd_period(int period) {sbd_period = period;}

// ground
int Config::get_heartbeat_period() const { return heartbeat_period;}
void Config::set_heartbeat_period(int period) {heartbeat_period = period;}

int Config::get_sms_heartbeat_period() const { return sms_heartbeat_period;}
void Config::set_sms_heartbeat_period(int period) {sms_heartbeat_period = period;}

/* groundstation configuration properties */

std::string Config::get_groundstation_tlf_number1() const { return groundstation_tlf_number1; }
void Config::set_groundstation_tlf_number1(const std::string& number) { groundstation_tlf_number1 = number; }

std::string Config::get_groundstation_tlf_number2() const { return groundstation_tlf_number2; }
void Config::set_groundstation_tlf_number2(const std::string& number) { groundstation_tlf_number2 = number; }

std::string Config::get_groundstation_tlf_number3() const { return groundstation_tlf_number3; }
void Config::set_groundstation_tlf_number3(const std::string& number) { groundstation_tlf_number3 = number; }

int Config::get_groundstation_rock_address() const { return groundstation_rock_address; }
void Config::set_groundstation_rock_address(int address) { groundstation_rock_address = address; }

int Config::get_groundstation_mav_id() const { return groundstation_mav_id; }
void Config::set_groundstation_mav_id(int mavid) { groundstation_mav_id = mavid; }

 /* aircraft1 configuration properties */

bool Config::get_aircraft1_enabled() const { return aircraft1_enabled; }
void Config::set_aircraft1_enabled(bool enabled) { aircraft1_enabled = enabled; }

std::string Config::get_aircraft1_tlf_number1() const { return aircraft1_tlf_number1; }
void Config::set_aircraft1_tlf_number1(const std::string& number) { aircraft1_tlf_number1 = number; }

std::string Config::get_aircraft1_tlf_number2() const { return aircraft1_tlf_number2; }
void Config::set_aircraft1_tlf_number2(const std::string& number) { aircraft1_tlf_number2 = number; }

std::string Config::get_aircraft1_tlf_number3() const { return aircraft1_tlf_number3; }
void Config::set_aircraft1_tlf_number3(const std::string& number) { aircraft1_tlf_number3 = number; }

int Config::get_aircraft1_rock_address() const { return aircraft1_rock_address; }
void Config::set_aircraft1_rock_address(int address) { aircraft1_rock_address = address; }

int Config::get_aircraft1_mav_id() const { return aircraft1_mav_id; }
void Config::set_aircraft1_mav_id(int mavid) { aircraft1_mav_id = mavid; }

 /* aircraft2 configuration properties */

bool Config::get_aircraft2_enabled() const { return aircraft2_enabled; }
void Config::set_aircraft2_enabled(bool enabled) { aircraft2_enabled = enabled; }

std::string Config::get_aircraft2_tlf_number1() const { return aircraft2_tlf_number1; }
void Config::set_aircraft2_tlf_number1(const std::string& number) { aircraft2_tlf_number1 = number; }

std::string Config::get_aircraft2_tlf_number2() const { return aircraft2_tlf_number2; }
void Config::set_aircraft2_tlf_number2(const std::string& number) { aircraft2_tlf_number2 = number; }

std::string Config::get_aircraft2_tlf_number3() const { return aircraft2_tlf_number3; }
void Config::set_aircraft2_tlf_number3(const std::string& number) { aircraft2_tlf_number3 = number; }

int Config::get_aircraft2_rock_address() const { return aircraft2_rock_address; }
void Config::set_aircraft2_rock_address(int address) { aircraft2_rock_address = address; }

int Config::get_aircraft2_mav_id() const { return aircraft2_mav_id; }
void Config::set_aircraft2_mav_id(int mavid) { aircraft2_mav_id = mavid; }

 /* aircraft3 configuration properties */

bool Config::get_aircraft3_enabled() const { return aircraft3_enabled; }
void Config::set_aircraft3_enabled(bool enabled) { aircraft3_enabled = enabled; }

std::string Config::get_aircraft3_tlf_number1() const { return aircraft3_tlf_number1; }
void Config::set_aircraft3_tlf_number1(const std::string& number) { aircraft3_tlf_number1 = number; }

std::string Config::get_aircraft3_tlf_number2() const { return aircraft3_tlf_number2; }
void Config::set_aircraft3_tlf_number2(const std::string& number) { aircraft3_tlf_number2 = number; }

std::string Config::get_aircraft3_tlf_number3() const { return aircraft3_tlf_number3; }
void Config::set_aircraft3_tlf_number3(const std::string& number) { aircraft3_tlf_number3 = number; }

int Config::get_aircraft3_rock_address() const { return aircraft3_rock_address; }
void Config::set_aircraft3_rock_address(int address) { aircraft3_rock_address = address; }

int Config::get_aircraft3_mav_id() const { return aircraft3_mav_id; }
void Config::set_aircraft3_mav_id(int mavid) { aircraft3_mav_id = mavid; }

 /* aircraft4 configuration properties */

bool Config::get_aircraft4_enabled() const { return aircraft4_enabled; }
void Config::set_aircraft4_enabled(bool enabled) { aircraft4_enabled = enabled; }

std::string Config::get_aircraft4_tlf_number1() const { return aircraft4_tlf_number1; }
void Config::set_aircraft4_tlf_number1(const std::string& number) { aircraft4_tlf_number1 = number; }

std::string Config::get_aircraft4_tlf_number2() const { return aircraft4_tlf_number2; }
void Config::set_aircraft4_tlf_number2(const std::string& number) { aircraft4_tlf_number2 = number; }

std::string Config::get_aircraft4_tlf_number3() const { return aircraft4_tlf_number3; }
void Config::set_aircraft4_tlf_number3(const std::string& number) { aircraft4_tlf_number3 = number; }

int Config::get_aircraft4_rock_address() const { return aircraft4_rock_address; }
void Config::set_aircraft4_rock_address(int address) { aircraft4_rock_address = address; }

int Config::get_aircraft4_mav_id() const { return aircraft4_mav_id; }
void Config::set_aircraft4_mav_id(int mavid) { aircraft4_mav_id = mavid; }

 /* aircraft5 configuration properties */

bool Config::get_aircraft5_enabled() const { return aircraft5_enabled; }
void Config::set_aircraft5_enabled(bool enabled) { aircraft5_enabled = enabled; }

std::string Config::get_aircraft5_tlf_number1() const { return aircraft5_tlf_number1; }
void Config::set_aircraft5_tlf_number1(const std::string& number) { aircraft5_tlf_number1 = number; }

std::string Config::get_aircraft5_tlf_number2() const { return aircraft5_tlf_number2; }
void Config::set_aircraft5_tlf_number2(const std::string& number) { aircraft5_tlf_number2 = number; }

std::string Config::get_aircraft5_tlf_number3() const { return aircraft5_tlf_number3; }
void Config::set_aircraft5_tlf_number3(const std::string& number) { aircraft5_tlf_number3 = number; }

int Config::get_aircraft5_rock_address() const { return aircraft5_rock_address; }
void Config::set_aircraft5_rock_address(int address) { aircraft5_rock_address = address; }

int Config::get_aircraft5_mav_id() const { return aircraft5_mav_id; }
void Config::set_aircraft5_mav_id(int mavid) { aircraft5_mav_id = mavid; }

}  // namespace radioroom
