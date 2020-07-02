/*
 radioroom.cpp

 Telemetry for MAVLink autopilots.

 (C) Copyright 2018-2019 Envirover.

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

#include <signal.h>
#include <stdio.h>
#include <unistd.h>

#include <atomic>
#include <iostream>

#include "Config.h"
#include "MAVLinkHandlerAir.h"
#include "MAVLinkLogger.h"
#include "build.h"
#include "timelib.h"

using mavio::log;
using radioroom::MAVLinkHandlerAir;
using std::cout;
using std::endl;

constexpr char log_identity[] = "radioroom";

const std::chrono::milliseconds msg_handler_loop_period(1);

std::atomic<bool> running(false);

void print_help() {
  cout << "Usage: radioroom [options]" << endl;
  cout << "options:" << endl;
  cout << "    -c <config file>" << endl;
  cout << "          Alternative configuration file instead of "
          "/etc/radioroom.conf."
       << endl;
  cout << endl;
  cout << "    -h    Print this help and exit." << endl;
  cout << endl;
  cout << "    -v    Verbose logging." << endl;
  cout << endl;
  cout << "    -V    Print version and exit." << endl;
}

void print_version() {
  cout << RADIO_ROOM_VERSION << "." << BUILD_NUM << endl;
  cout << "MAVLink wire protocol version " << MAVLINK_WIRE_PROTOCOL_VERSION
       << endl;
}

void handle_signal(int sig) {
  if (sig == SIGTERM) {
    running = false;

    /* Reset signal handling to default behavior */
    signal(SIGTERM, SIG_DFL);
  }
}

int main(int argc, char** argv) {
  MAVLinkHandlerAir msg_handler;
  std::string config_file = "/boot/radioroomair.conf";

  int c;
  while ((c = getopt(argc, argv, "c:hvV")) != -1) {
    switch (c) {
      case 'c':
        config_file = optarg;
        break;
      case 'h':
        print_help();
        return EXIT_SUCCESS;
      case 'v':
        radioroom::config.set_debug_mode(true);
        break;
      case 'V':
        print_version();
        return EXIT_SUCCESS;
      case '?':
        if (optopt == 'c') {
          cout << "Option -c requires an argument." << endl;
        } else if (isprint(optopt)) {
          cout << "Unknown option '-" << std::string(1, optopt) << "'." << endl;
        } else {
          cout << "Unknown option character '" << std::string(1, optopt) << "'."
               << endl;
        }
        return EXIT_FAILURE;
    }
  }

  mavio::openlog(log_identity,  radioroom::config.get_debug_mode()
                                   ? LOG_UPTO(LOG_DEBUG)
                                   : LOG_UPTO(LOG_INFO));

  log(LOG_INFO, "Starting %s.%s...", RADIO_ROOM_VERSION, BUILD_NUM);

  if (radioroom::config.init(config_file) < 0) {
    log(LOG_ERR, "Can't load configuration file '%s'", config_file.data());
  }

  if (msg_handler.init()) {
    log(LOG_NOTICE, "%s.%s started.", RADIO_ROOM_VERSION, BUILD_NUM);
  } else {
    log(LOG_CRIT, "%s.%s initialization failed.", RADIO_ROOM_VERSION,
        BUILD_NUM);
    return EXIT_FAILURE;
  }

  signal(SIGTERM, handle_signal);

  running = true;

  while (running) {
    if (msg_handler.loop()) {
      timelib::sleep(msg_handler_loop_period);
    }
  }

  log(LOG_INFO, "Stopping %s.%s...", RADIO_ROOM_VERSION, BUILD_NUM);

  msg_handler.close();

  log(LOG_NOTICE, "%s.%s stopped.", RADIO_ROOM_VERSION, BUILD_NUM);

  mavio::closelog();

  return EXIT_SUCCESS;
}
