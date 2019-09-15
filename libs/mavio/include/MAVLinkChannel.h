/*
 MAVLinkChannel.h

 MAVIO MAVLink I/O library.

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

 Created on: Mar 26, 2018
     Author: Pavel Bobov
 */

#ifndef MAVLINKCHANNEL_H_
#define MAVLINKCHANNEL_H_

#include "MAVLinkLib.h"

#include <chrono>
#include <string>

namespace mavio {

/*
 * Interface for send/receive channels of MAVLink messages.
 */
class MAVLinkChannel {
public:
    MAVLinkChannel(std::string channel_id) : channel_id(channel_id) {}

    virtual ~MAVLinkChannel(){};

    /**
     * Returns the channel ID.
     */
    virtual std::string get_channel_id() const { return channel_id; }

    /**
     * Closes the connection if it was open.
     */
    virtual void close() = 0;

    /**
     * Sends the specified MAVLink message to the socket.
     *
     * Returns true if the message was sent successfully.
     */
    virtual bool send_message(const mavlink_message_t& msg) = 0;

    /**
     * Receives MAVLink message from the socket.
     *
     * Returns true if a message was received.
     */
    virtual bool receive_message(mavlink_message_t& msg) = 0;

    /**
     * Checks if data is available in the socket input buffer.
     *
     * Returns true if data is available.
     */
    virtual bool message_available() = 0;

    /**
     * Returns time of the last successfully sent message.  
     */
    virtual std::chrono::high_resolution_clock::time_point last_send_time() = 0;

    /**
     * Returns time of the last successfully received message.  
     */
    virtual std::chrono::high_resolution_clock::time_point last_receive_time() = 0;

private:
    std::string channel_id;
};

} // namespace mavio

#endif /* MAVLINKCHANNEL_H_ */
