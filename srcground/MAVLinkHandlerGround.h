#ifndef SRC_MAVLINKHANDLERGROUND_H_
#define SRC_MAVLINKHANDLERGROUND_H_

// #include "MAVLinkAutopilot.h"
#include "MAVLinkRFD900x.h"
#include "MAVLinkISBDChannel.h"
#include "MAVLinkSMSChannel.h"
#include "MAVLinkTCPChannelServer.h"
#include "VehicleManager.h"
#include "timelib.h"

namespace radioroom {

class MAVLinkHandlerGround {
 public:
  MAVLinkHandlerGround();

  bool init();

  void close();

  bool loop();

 private:

  void handle_mo_message(const mavlink_message_t& msg);

  void handle_mo_sms(mavio::SMSmessage& sms);

  void handle_mo_sbd(mavio::SBDmessage& sbdmessage);

  void handle_mt_message(const mavlink_message_t& msg);

  void send_gcs_signal_quality();

  mavio::VehicleManager vehicle_manager;
  mavio::MAVLinkRFD900x rfd_channel;
  mavio::MAVLinkISBDChannel isbd_channel;
  mavio::MAVLinkTCPChannelServer tcp_channel;
  mavio::MAVLinkSMSChannel gsm_channel;

  // link status
  int gsm_quality = 0;
  int sbd_quality = 0;
  timelib::Stopwatch timer_send_link_status;
  std::chrono::milliseconds timer_send_link_status_period = timelib::sec2ms(5);

  // hardware parameters
  bool _radio_initialized = false;
  bool _gsm_initialized = false;
  bool _isbd_initialized = false;

  bool _isbd_first_contact = false;

};

}  // namespace radioroom

#endif  // SRC_MAVLINKHANDLERGround_H_
