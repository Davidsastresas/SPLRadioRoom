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

constexpr bool default_autopilot_enabled = false;
constexpr char default_autopilot_serial[] = "/dev/ttyusb0";
constexpr int autopilot_serial_baud_rate = 115200;

constexpr bool default_rfd_enabled = false;
constexpr char default_rfd_serial[] = "/dev/ttyusb1";
constexpr int rfd_serial_baud_rate = 115200;

constexpr bool default_isbd_enabled = false;
constexpr char default_isbd_serial[] = "/dev/ttyusb1";
constexpr int isbd_serial_baud_rate = 19200;

constexpr bool default_tcp_enabled = false;
constexpr char default_tcp_host[] = "";
constexpr int default_tcp_port = 5060;

constexpr double default_isbd_report_period = 300.0;  // 5 minutes
constexpr double default_tcp_report_period = 60.0;    // 1 minute

constexpr bool default_enabled = false;
constexpr int default_int = 0;
constexpr char default_string[] = "0";

// radioroom.conf properties
constexpr char autopilot_enabled_property[] = "enabled";
constexpr char autopilot_config_section[] = "autopilot";
constexpr char autopilot_serial_property[] = "serial";
constexpr char autopilot_serial_speed_property[] = "serial_speed";

constexpr char rfd_enabled_property[] = "enabled";
constexpr char rfd_config_section[] = "rfd";
constexpr char rfd_serial_property[] = "serial";
constexpr char rfd_serial_speed_property[] = "serial_speed";

constexpr char radioroom_config_section[] = "radioroom";
constexpr char auto_detect_serials_property[] = "auto_detect_serials";
constexpr char air_unit_property[] = "air_unit";
constexpr char report_period_property[] = "report_period";

constexpr char isbd_config_section[] = "isbd";
constexpr char isbd_enabled_property[] = "enabled";
constexpr char isbd_serial_property[] = "serial";
constexpr char isbd_serial_speed_property[] = "serial_speed";

constexpr char tcp_config_section[] = "tcp";
constexpr char tcp_enabled_property[] = "enabled";
constexpr char tcp_host_property[] = "host";
constexpr char tcp_port_property[] = "port";

constexpr char aircraft1_config_section[] = "aircraft1";
constexpr char aircraft2_config_section[] = "aircraft2";
constexpr char aircraft3_config_section[] = "aircraft3";
constexpr char aircraft4_config_section[] = "aircraft4";
constexpr char aircraft5_config_section[] = "aircraft5";

constexpr char groundstation_config_section[] = "groundstation";

constexpr char enabled[] = "enabled";
constexpr char tlf_number1[] = "tlf_number1";
constexpr char tlf_number2[] = "tlf_number2";
constexpr char tlf_number3[] = "tlf_number3";
constexpr char rock_address[] = "rock_address";
constexpr char mav_id[] = "mav_id";

// TODO we may need to check this out
Config::Config()
    : autopilot_serial(default_autopilot_serial),
      autopilot_serial_speed(autopilot_serial_baud_rate),
      auto_detect_serials(true),
      debug_mode(false),
      isbd_enabled(default_isbd_enabled),
      isbd_serial(default_isbd_serial),
      isbd_serial_speed(isbd_serial_baud_rate),
      isbd_report_period(default_isbd_report_period),
      tcp_enabled(default_tcp_enabled),
      tcp_host(default_tcp_host),
      tcp_port(default_tcp_port),
      tcp_report_period(default_tcp_report_period) {}

int Config::init(const std::string& config_file) {
  INIReader conf(config_file);

  int ret = conf.ParseError();

  if (ret < 0) {
    return ret;
  }

  /* [autopilot] config section */

  set_autopilot_enabled(conf.GetBoolean(autopilot_config_section, autopilot_enabled_property,
                                   default_autopilot_enabled));


  set_autopilot_serial(conf.Get(autopilot_config_section,
                                autopilot_serial_property,
                                default_autopilot_serial));

  set_autopilot_serial_speed(conf.GetInteger(autopilot_config_section,
                                             autopilot_serial_speed_property,
                                             autopilot_serial_baud_rate));

  /* [rfd] config section */

  set_rfd_enabled(conf.GetBoolean(rfd_config_section, rfd_enabled_property,
                                   default_rfd_enabled));

  set_rfd_serial(conf.Get(rfd_config_section,
                                rfd_serial_property,
                                default_rfd_serial));

  set_rfd_serial_speed(conf.GetInteger(rfd_config_section,
                                             rfd_serial_speed_property,
                                             rfd_serial_baud_rate));

  /* [radioroom] config section */

  set_auto_detect_serials(conf.GetBoolean(radioroom_config_section,
                                          auto_detect_serials_property, true));

  set_air_unit(conf.GetBoolean(radioroom_config_section,
                                          air_unit_property, true));

  /* [isbd] config section */

  set_isbd_enabled(conf.GetBoolean(isbd_config_section, isbd_enabled_property,
                                   default_isbd_enabled));

  set_isbd_serial(
      conf.Get(isbd_config_section, isbd_serial_property, default_isbd_serial));

  set_isbd_serial_speed(conf.GetInteger(
      isbd_config_section, isbd_serial_speed_property, isbd_serial_baud_rate));

  set_isbd_report_period(conf.GetReal(
      isbd_config_section, report_period_property, default_isbd_report_period));

  /* [tcp] config section */

  set_tcp_enabled(conf.GetBoolean(tcp_config_section, tcp_enabled_property,
                                  default_tcp_enabled));

  set_tcp_host(
      conf.Get(tcp_config_section, tcp_host_property, default_tcp_host));

  set_tcp_port(
      conf.GetInteger(tcp_config_section, tcp_port_property, default_tcp_port));

  set_tcp_report_period(conf.GetReal(tcp_config_section, report_period_property,
                                     default_tcp_report_period));

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

  set_groundstation_rock_address(conf.Get(groundstation_config_section,
                                rock_address,
                                default_string));

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

  set_aircraft1_rock_address(conf.Get(aircraft1_config_section,
                                rock_address,
                                default_string));

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

  set_aircraft2_rock_address(conf.Get(aircraft2_config_section,
                                rock_address,
                                default_string));

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

  set_aircraft3_rock_address(conf.Get(aircraft3_config_section,
                                rock_address,
                                default_string));

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

  set_aircraft4_rock_address(conf.Get(aircraft4_config_section,
                                rock_address,
                                default_string));

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

    set_aircraft5_rock_address(conf.Get(aircraft5_config_section,
                                  rock_address,
                                  default_string));

    set_aircraft5_mav_id(conf.GetInteger(
          aircraft5_config_section, mav_id, default_int));

  return 0;
}

bool Config::get_debug_mode() const { return debug_mode; }

void Config::set_debug_mode(bool debug) { debug_mode = debug; }

std::string Config::get_autopilot_serial() const { return autopilot_serial; }

std::string Config::get_rfd_serial() const { return rfd_serial; }

void Config::set_autopilot_serial(const std::string& path) {
  autopilot_serial = path;
}

int Config::get_autopilot_serial_speed() const {
  return autopilot_serial_speed;
}

void Config::set_rfd_serial(const std::string& path) {
  rfd_serial = path;
}

int Config::get_rfd_serial_speed() const {
  return rfd_serial_speed;
}

void Config::set_autopilot_serial_speed(int speed) {
  autopilot_serial_speed = speed;
}

void Config::set_rfd_serial_speed(int speed) {
  rfd_serial_speed = speed;
}

bool Config::get_isbd_enabled() const { return isbd_enabled; }

bool Config::get_autopilot_enabled() const { return autopilot_enabled; }

bool Config::get_rfd_enabled() const { return rfd_enabled; }

void Config::set_rfd_enabled(bool enabled) { rfd_enabled = enabled; }

void Config::set_autopilot_enabled(bool enabled) { autopilot_enabled = enabled; }

void Config::set_isbd_enabled(bool enabled) { isbd_enabled = enabled; }

std::string Config::get_isbd_serial() const { return isbd_serial; }

void Config::set_isbd_serial(const std::string& path) { isbd_serial = path; }

int Config::get_isbd_serial_speed() const { return isbd_serial_speed; }

void Config::set_isbd_serial_speed(int speed) { isbd_serial_speed = speed; }

bool Config::get_auto_detect_serials() const { return auto_detect_serials; }

void Config::set_auto_detect_serials(bool a) { auto_detect_serials = a; }

bool Config::get_air_unit() const { return air_unit; }

void Config::set_air_unit(bool a) { air_unit = a; }

double Config::get_isbd_report_period() const { return isbd_report_period; }

void Config::set_isbd_report_period(double period) { isbd_report_period = period; }

bool Config::get_tcp_enabled() const { return tcp_enabled; }

void Config::set_tcp_enabled(bool enabled) { tcp_enabled = enabled; }

std::string Config::get_tcp_host() const { return tcp_host; }

void Config::set_tcp_host(const std::string& host) { tcp_host = host; }

int Config::get_tcp_port() const { return tcp_port; }

void Config::set_tcp_port(int port) { tcp_port = port; }

double Config::get_tcp_report_period() const { return tcp_report_period; }

void Config::set_tcp_report_period(double period) { tcp_report_period = period; };

 /* groundstation configuration properties */

std::string Config::get_groundstation_tlf_number1() const { return groundstation_tlf_number1; }
void Config::set_groundstation_tlf_number1(const std::string& number) { groundstation_tlf_number1 = number; }

std::string Config::get_groundstation_tlf_number2() const { return groundstation_tlf_number2; }
void Config::set_groundstation_tlf_number2(const std::string& number) { groundstation_tlf_number2 = number; }

std::string Config::get_groundstation_tlf_number3() const { return groundstation_tlf_number3; }
void Config::set_groundstation_tlf_number3(const std::string& number) { groundstation_tlf_number3 = number; }

std::string Config::get_groundstation_rock_address() const { return groundstation_rock_address; }
void Config::set_groundstation_rock_address(const std::string& address) { groundstation_rock_address = address; }

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

std::string Config::get_aircraft1_rock_address() const { return aircraft1_rock_address; }
void Config::set_aircraft1_rock_address(const std::string& address) { aircraft1_rock_address = address; }

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

std::string Config::get_aircraft2_rock_address() const { return aircraft2_rock_address; }
void Config::set_aircraft2_rock_address(const std::string& address) { aircraft2_rock_address = address; }

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

std::string Config::get_aircraft3_rock_address() const { return aircraft3_rock_address; }
void Config::set_aircraft3_rock_address(const std::string& address) { aircraft3_rock_address = address; }

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

std::string Config::get_aircraft4_rock_address() const { return aircraft4_rock_address; }
void Config::set_aircraft4_rock_address(const std::string& address) { aircraft4_rock_address = address; }

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

std::string Config::get_aircraft5_rock_address() const { return aircraft5_rock_address; }
void Config::set_aircraft5_rock_address(const std::string& address) { aircraft5_rock_address = address; }

int Config::get_aircraft5_mav_id() const { return aircraft5_mav_id; }
void Config::set_aircraft5_mav_id(int mavid) { aircraft5_mav_id = mavid; }

}  // namespace radioroom