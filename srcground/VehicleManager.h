#ifndef LIBS_MAVIO_INCLUDE_VEHICLEMANAGER_H_
#define LIBS_MAVIO_INCLUDE_VEHICLEMANAGER_H_

#include "CircularBuffer.h"
#include "MAVLinkLib.h"
#include "MAVLinkSMSChannel.h"
#include "MAVLinkISBDChannel.h"
#include "Vehicle.h"


#include <atomic>
#include <string>
#include <thread>
#include <chrono>

namespace mavio {

constexpr int max_vehicles_allowed = 5;

class VehicleManager {
    public:

    VehicleManager();
    ~VehicleManager();

    static VehicleManager *get_singleton() {
        return _singleton;
    }

    bool init();

    bool push_queue_tcp(const mavlink_message_t& msg);
    bool pull_queue_tcp(mavlink_message_t& msg);

    bool push_queue_rfd(const mavlink_message_t& msg);
    bool pull_queue_rfd(mavlink_message_t& msg);

    bool push_queue_gsm(const mavio::SMSmessage sms);
    bool pull_queue_gsm(mavio::SMSmessage& sms);

    bool push_queue_sbd(const mavio::SBDmessage& sbdmessage);
    bool pull_queue_sbd(mavio::SBDmessage& sbdmessage);
    
    void close();
    
    private:

    bool get_vehicle_by_mav_id(int id, int& vehicleinstance);

    void send_task();

    void receive_task();

    static VehicleManager *_singleton;

    std::atomic<bool> _running;

    std::thread send_thread;
    std::thread receive_thread;

    CircularBuffer<mavlink_message_t> _send_queue_tcp;
    CircularBuffer<mavlink_message_t> _receive_queue_tcp;
    CircularBuffer<mavlink_message_t> _send_queue_rfd;
    CircularBuffer<mavlink_message_t> _receive_queue_rfd;
    CircularBuffer<mavio::SMSmessage> _send_queue_gsm;
    CircularBuffer<mavio::SMSmessage> _receive_queue_gsm;
    CircularBuffer<mavio::SBDmessage> _send_queue_sbd;
    CircularBuffer<mavio::SBDmessage> _receive_queue_sbd;

    Vehicle _vehicles[max_vehicles_allowed];
    int _vehicles_initialized = 0;
};

} // namespace mavio

#endif // LIBS_MAVIO_INCLUDE_VEHICLE_H_