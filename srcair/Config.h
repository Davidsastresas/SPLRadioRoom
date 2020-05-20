/*
 Config.h

 This file is a part of RadioRoom project.

 (C) Copyright 2017-2019 Envirover.

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
#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_

#include <string>

namespace radioroom {

constexpr char default_config_file[] = "/etc/radioroomair.conf";

/**
 * RadioRoom configuration properties.
 */
class Config {
 public:
  Config();

  /*
   * Loads configuration from the specified config file.
   *
   * Returns 0 in case of success and exit code in case of invalid
   * configuration.
   */
  int init(const std::string& config_file);

  /* Autopilot configuration properties */

  std::string get_autopilot_serial() const;
  void set_autopilot_serial(const std::string& path);

  int get_autopilot_serial_speed() const;
  void set_autopilot_serial_speed(int speed);

  bool get_autopilot_enabled() const;
  void set_autopilot_enabled(bool enabled);

  /* rfd configuration properties */

  std::string get_rfd_serial() const;
  void set_rfd_serial(const std::string& path);

  int get_rfd_serial_speed() const;
  void set_rfd_serial_speed(int speed);

  bool get_rfd_enabled() const;
  void set_rfd_enabled(bool enabled);

  /* RadioRoom configuration properties */

  bool get_auto_detect_serials() const;
  void set_auto_detect_serials(bool a);
  
  bool get_air_unit() const;
  void set_air_unit(bool a);

  bool get_debug_mode() const;
  void set_debug_mode(bool debug);

  /* ISBD comm link configuration properties */

  bool get_isbd_enabled() const;
  void set_isbd_enabled(bool enabled);

  std::string get_isbd_serial() const;
  void set_isbd_serial(const std::string& path);

  int get_isbd_serial_speed() const;
  void set_isbd_serial_speed(int speed);

  double get_isbd_report_period() const;
  void set_isbd_report_period(double period);

  /* TCP/IP comm link configuration properties */

  bool get_tcp_enabled() const;
  void set_tcp_enabled(bool enabled);

  std::string get_tcp_host() const;
  void set_tcp_host(const std::string& host);

  int get_tcp_port() const;
  void set_tcp_port(int port);

  double get_tcp_report_period() const;
  void set_tcp_report_period(double period);

  /* groundstation configuration properties */

  std::string get_groundstation_tlf_number1() const;
  void set_groundstation_tlf_number1(const std::string& number);

  std::string get_groundstation_tlf_number2() const;
  void set_groundstation_tlf_number2(const std::string& number);
  
  std::string get_groundstation_tlf_number3() const;
  void set_groundstation_tlf_number3(const std::string& number);

  std::string get_groundstation_rock_address() const;
  void set_groundstation_rock_address(const std::string& address);

  int get_groundstation_mav_id() const;
  void set_groundstation_mav_id(int mavid);

  /* aircraft1 configuration properties */

  bool get_aircraft1_enabled() const;
  void set_aircraft1_enabled(bool enabled);

  std::string get_aircraft1_tlf_number1() const;
  void set_aircraft1_tlf_number1(const std::string& number);
  
  std::string get_aircraft1_tlf_number2() const;
  void set_aircraft1_tlf_number2(const std::string& number);
  
  std::string get_aircraft1_tlf_number3() const;
  void set_aircraft1_tlf_number3(const std::string& number);

  std::string get_aircraft1_rock_address() const;
  void set_aircraft1_rock_address(const std::string& address);

  int get_aircraft1_mav_id() const;
  void set_aircraft1_mav_id(int mavid);

  /* aircraft2 configuration properties */

  bool get_aircraft2_enabled() const;
  void set_aircraft2_enabled(bool enabled);

  std::string get_aircraft2_tlf_number1() const;
  void set_aircraft2_tlf_number1(const std::string& number);

  std::string get_aircraft2_tlf_number2() const;
  void set_aircraft2_tlf_number2(const std::string& number);
  
  std::string get_aircraft2_tlf_number3() const;
  void set_aircraft2_tlf_number3(const std::string& number);

  std::string get_aircraft2_rock_address() const;
  void set_aircraft2_rock_address(const std::string& address);

  int get_aircraft2_mav_id() const;
  void set_aircraft2_mav_id(int mavid);

  /* aircraft3 configuration properties */

  bool get_aircraft3_enabled() const;
  void set_aircraft3_enabled(bool enabled);

  std::string get_aircraft3_tlf_number1() const;
  void set_aircraft3_tlf_number1(const std::string& number);
  
  std::string get_aircraft3_tlf_number2() const;
  void set_aircraft3_tlf_number2(const std::string& number);

  std::string get_aircraft3_tlf_number3() const;
  void set_aircraft3_tlf_number3(const std::string& number);

  std::string get_aircraft3_rock_address() const;
  void set_aircraft3_rock_address(const std::string& address);

  int get_aircraft3_mav_id() const;
  void set_aircraft3_mav_id(int mavid);

  /* aircraft4 configuration properties */

  bool get_aircraft4_enabled() const;
  void set_aircraft4_enabled(bool enabled);

  std::string get_aircraft4_tlf_number1() const;
  void set_aircraft4_tlf_number1(const std::string& number);
  
  std::string get_aircraft4_tlf_number2() const;
  void set_aircraft4_tlf_number2(const std::string& number);

  std::string get_aircraft4_tlf_number3() const;
  void set_aircraft4_tlf_number3(const std::string& number);

  std::string get_aircraft4_rock_address() const;
  void set_aircraft4_rock_address(const std::string& address);

  int get_aircraft4_mav_id() const;
  void set_aircraft4_mav_id(int mavid);

  /* aircraft5 configuration properties */

  bool get_aircraft5_enabled() const;
  void set_aircraft5_enabled(bool enabled);

  std::string get_aircraft5_tlf_number1() const;
  void set_aircraft5_tlf_number1(const std::string& number);

  std::string get_aircraft5_tlf_number2() const;
  void set_aircraft5_tlf_number2(const std::string& number);
  
  std::string get_aircraft5_tlf_number3() const;
  void set_aircraft5_tlf_number3(const std::string& number);

  std::string get_aircraft5_rock_address() const;
  void set_aircraft5_rock_address(const std::string& address);

  int get_aircraft5_mav_id() const;
  void set_aircraft5_mav_id(int mavid);



 private:

  bool autopilot_enabled;
  std::string autopilot_serial;
  int autopilot_serial_speed;
  
  bool rfd_enabled;
  std::string rfd_serial;
  int rfd_serial_speed;

  bool auto_detect_serials;
  bool air_unit;
  bool debug_mode;

  bool isbd_enabled;
  std::string isbd_serial;
  int isbd_serial_speed;
  double isbd_report_period;

  bool tcp_enabled;
  std::string tcp_host;
  int tcp_port;
  double tcp_report_period;

  bool groundstation_enabled;
  std::string groundstation_tlf_number1;
  std::string groundstation_tlf_number2;
  std::string groundstation_tlf_number3;
  std::string groundstation_rock_address;
  int groundstation_mav_id;
  
  bool aircraft1_enabled;
  std::string aircraft1_tlf_number1;
  std::string aircraft1_tlf_number2;
  std::string aircraft1_tlf_number3;
  std::string aircraft1_rock_address;
  int aircraft1_mav_id;

  bool aircraft2_enabled;
  std::string aircraft2_tlf_number1;
  std::string aircraft2_tlf_number2;
  std::string aircraft2_tlf_number3;
  std::string aircraft2_rock_address;
  int aircraft2_mav_id;

  bool aircraft3_enabled;
  std::string aircraft3_tlf_number1;
  std::string aircraft3_tlf_number2;
  std::string aircraft3_tlf_number3;
  std::string aircraft3_rock_address;
  int aircraft3_mav_id;

  bool aircraft4_enabled;
  std::string aircraft4_tlf_number1;
  std::string aircraft4_tlf_number2;
  std::string aircraft4_tlf_number3;
  std::string aircraft4_rock_address;
  int aircraft4_mav_id;

  bool aircraft5_enabled;
  std::string aircraft5_tlf_number1;
  std::string aircraft5_tlf_number2;
  std::string aircraft5_tlf_number3;
  std::string aircraft5_rock_address;
  int aircraft5_mav_id;
};

extern Config config;

}  // namespace radioroom

#endif  // SRC_CONFIG_H_
