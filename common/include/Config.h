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

// constexpr char default_config_file[] = "/boot/radioroomair.conf";

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

  int get_autopilot_id() const;
  void set_autopilot_id(int id);

  /* rfd configuration properties */

  std::string get_rfd_serial() const;
  void set_rfd_serial(const std::string& path);

  int get_rfd_serial_speed() const;
  void set_rfd_serial_speed(int speed);

  int get_rfd_id() const;
  void set_rfd_id(int id);

  bool get_rfd_enabled() const;
  void set_rfd_enabled(bool enabled);

  /* ISBD comm link configuration properties */

  std::string get_isbd_serial() const;
  void set_isbd_serial(const std::string& path);

  int get_isbd_serial_speed() const;
  void set_isbd_serial_speed(int speed);

  bool get_isbd_enabled() const;
  void set_isbd_enabled(bool enabled);

  bool get_isbd_get_noticed() const;
  void set_isbd_get_noticed(bool get_noticed);

  /* gsm comm link configuration properties */

  std::string get_gsm_serial1() const;
  void set_gsm_serial1(const std::string& path);
  
  std::string get_gsm_serial2() const;
  void set_gsm_serial2(const std::string& path);
  
  std::string get_gsm_serial3() const;
  void set_gsm_serial3(const std::string& path);

  int get_gsm_serial_speed() const;
  void set_gsm_serial_speed(int speed);

  std::string get_gsm_pin1() const;
  void set_gsm_pin1(const std::string& pin);

  std::string get_gsm_pin2() const;
  void set_gsm_pin2(const std::string& pin);

  std::string get_gsm_pin3() const;
  void set_gsm_pin3(const std::string& pin);

  bool get_gsm_enabled() const;
  void set_gsm_enabled(bool enabled);
  
  bool get_gsm_pdu_enabled() const;
  void set_gsm_pdu_enabled(bool pdu);

  /* TCP/IP comm link configuration properties */

  int get_tcp_port() const;
  void set_tcp_port(int port);

  /* misc configuration properties */

  // common
  bool get_debug_mode() const;
  void set_debug_mode(bool deb);

  int get_rfd_timeout() const;
  void set_rfd_timeout(int timeout);

  int get_sms_timeout() const;
  void set_sms_timeout(int timeout);
  
  // air
  int get_sms_period() const;
  void set_sms_period(int period);

  int get_sbd_period() const;
  void set_sbd_period(int period);

  // ground
  int get_heartbeat_period() const;
  void set_heartbeat_period(int period);

  int get_sms_heartbeat_period() const;
  void set_sms_heartbeat_period(int period);

  /* groundstation configuration properties */

  std::string get_groundstation_tlf_number1() const;
  void set_groundstation_tlf_number1(const std::string& number);

  std::string get_groundstation_tlf_number2() const;
  void set_groundstation_tlf_number2(const std::string& number);
  
  std::string get_groundstation_tlf_number3() const;
  void set_groundstation_tlf_number3(const std::string& number);

  int get_groundstation_rock_address() const;
  void set_groundstation_rock_address(int address);

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

  int get_aircraft1_rock_address() const;
  void set_aircraft1_rock_address(int address);

  int get_aircraft1_mav_id() const;
  void set_aircraft1_mav_id(int mavid);

  void set_aircraft1_ip_address(const std::string& addr);

  void set_aircraft1_ip_port(int port);

  /* aircraft2 configuration properties */

  bool get_aircraft2_enabled() const;
  void set_aircraft2_enabled(bool enabled);

  std::string get_aircraft2_tlf_number1() const;
  void set_aircraft2_tlf_number1(const std::string& number);

  std::string get_aircraft2_tlf_number2() const;
  void set_aircraft2_tlf_number2(const std::string& number);
  
  std::string get_aircraft2_tlf_number3() const;
  void set_aircraft2_tlf_number3(const std::string& number);

  int get_aircraft2_rock_address() const;
  void set_aircraft2_rock_address(int address);

  int get_aircraft2_mav_id() const;
  void set_aircraft2_mav_id(int mavid);

  void set_aircraft2_ip_address(const std::string& addr);

  void set_aircraft2_ip_port(int port);

  /* aircraft3 configuration properties */

  bool get_aircraft3_enabled() const;
  void set_aircraft3_enabled(bool enabled);

  std::string get_aircraft3_tlf_number1() const;
  void set_aircraft3_tlf_number1(const std::string& number);
  
  std::string get_aircraft3_tlf_number2() const;
  void set_aircraft3_tlf_number2(const std::string& number);

  std::string get_aircraft3_tlf_number3() const;
  void set_aircraft3_tlf_number3(const std::string& number);

  int get_aircraft3_rock_address() const;
  void set_aircraft3_rock_address(int address);

  int get_aircraft3_mav_id() const;
  void set_aircraft3_mav_id(int mavid);

  void set_aircraft3_ip_address(const std::string& addr);

  void set_aircraft3_ip_port(int port);

  /* aircraft4 configuration properties */

  bool get_aircraft4_enabled() const;
  void set_aircraft4_enabled(bool enabled);

  std::string get_aircraft4_tlf_number1() const;
  void set_aircraft4_tlf_number1(const std::string& number);
  
  std::string get_aircraft4_tlf_number2() const;
  void set_aircraft4_tlf_number2(const std::string& number);

  std::string get_aircraft4_tlf_number3() const;
  void set_aircraft4_tlf_number3(const std::string& number);

  int get_aircraft4_rock_address() const;
  void set_aircraft4_rock_address(int address);

  int get_aircraft4_mav_id() const;
  void set_aircraft4_mav_id(int mavid);

  void set_aircraft4_ip_address(const std::string& addr);

  void set_aircraft4_ip_port(int port);

  /* aircraft5 configuration properties */

  bool get_aircraft5_enabled() const;
  void set_aircraft5_enabled(bool enabled);

  std::string get_aircraft5_tlf_number1() const;
  void set_aircraft5_tlf_number1(const std::string& number);

  std::string get_aircraft5_tlf_number2() const;
  void set_aircraft5_tlf_number2(const std::string& number);
  
  std::string get_aircraft5_tlf_number3() const;
  void set_aircraft5_tlf_number3(const std::string& number);

  int get_aircraft5_rock_address() const;
  void set_aircraft5_rock_address(int address);

  int get_aircraft5_mav_id() const;
  void set_aircraft5_mav_id(int mavid);

  void set_aircraft5_ip_address(const std::string& addr);

  void set_aircraft5_ip_port(int port);

  /* global aircraft getter functions */

  std::string get_groundstation_tlfx(int tlfindex) const;

  bool get_aircraftx_enabled(int index) const;

  std::string get_aircraftx_tlfx(int index, int tlfindex) const;

  int get_aircraftx_rock_address(int index) const;

  int get_aircraftx_mav_id(int index) const;

  std::string get_aircraftx_ip_address(int index) const;

  int get_aircraftx_ip_port(int index) const;

 private:

  // autopilot
  std::string autopilot_serial = "0";
  int autopilot_serial_speed = 0;
  int autopilot_id = 0;
  
  // rfd
  std::string rfd_serial = "0";
  int rfd_serial_speed = 0;
  int rfd_id = 0;
  bool rfd_enabled = false;

  // isbd
  std::string isbd_serial = "0";
  int isbd_serial_speed = 0;
  bool isbd_enabled = false;
  bool isbd_get_noticed = false;

  // gsm
  std::string gsm_serial1 = "0";
  std::string gsm_serial2 = "0";
  std::string gsm_serial3 = "0";
  int gsm_serial_speed = 0;
  std::string gsm_pin1 = "0";
  std::string gsm_pin2 = "0";
  std::string gsm_pin3 = "0";
  bool gsm_enabled = false;
  bool gsm_pdu_enabled = false;

  // tcp
  int tcp_port = 0;

  // misc
  bool debug = false;
  int rfd_timeout = 0;
  int sms_timeout = 0;
  int sms_period = 0;
  int sbd_period = 0;
  int heartbeat_period = 0;
  int sms_heartbeat_period = 0;

  // groundstation
  bool groundstation_enabled = false;
  std::string groundstation_tlf_number1 = "0";
  std::string groundstation_tlf_number2 = "0";
  std::string groundstation_tlf_number3 = "0";
  int groundstation_rock_address = 0;
  int groundstation_mav_id = 0;
  
  // aircraft 1
  bool aircraft1_enabled = false;
  std::string aircraft1_tlf_number1 = "0";
  std::string aircraft1_tlf_number2 = "0";
  std::string aircraft1_tlf_number3 = "0";
  int aircraft1_rock_address = 0;
  int aircraft1_mav_id = 0;
  std::string aircraft1_ip_addr = "0";
  int aircraft1_ip_port = 0;

  // aircraft 2
  bool aircraft2_enabled = false;
  std::string aircraft2_tlf_number1 = "0";
  std::string aircraft2_tlf_number2 = "0";
  std::string aircraft2_tlf_number3 = "0";
  int aircraft2_rock_address = 0;
  int aircraft2_mav_id = 0;
  std::string aircraft2_ip_addr = "0";
  int aircraft2_ip_port = 0;

  // aircraft 3
  bool aircraft3_enabled = false;
  std::string aircraft3_tlf_number1 = "0";
  std::string aircraft3_tlf_number2 = "0";
  std::string aircraft3_tlf_number3 = "0";
  int aircraft3_rock_address = 0;
  int aircraft3_mav_id = 0;
  std::string aircraft3_ip_addr = "0";
  int aircraft3_ip_port = 0;

  // aircraft 4
  bool aircraft4_enabled = false;
  std::string aircraft4_tlf_number1 = "0";
  std::string aircraft4_tlf_number2 = "0";
  std::string aircraft4_tlf_number3 = "0";
  int aircraft4_rock_address = 0;
  int aircraft4_mav_id = 0;
  std::string aircraft4_ip_addr = "0";
  int aircraft4_ip_port = 0;

  // aircraft 5
  bool aircraft5_enabled = false;
  std::string aircraft5_tlf_number1 = "0";
  std::string aircraft5_tlf_number2 = "0";
  std::string aircraft5_tlf_number3 = "0";
  int aircraft5_rock_address = 0;
  int aircraft5_mav_id = 0;
  std::string aircraft5_ip_addr = "0";
  int aircraft5_ip_port = 0;
};

extern Config config;

}  // namespace radioroom

#endif  // SRC_CONFIG_H_
