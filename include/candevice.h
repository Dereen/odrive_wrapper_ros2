/**
 * @file candevice.h
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Class for basic socketCAN communication framework 
 * @version 0.1
 * @date 2023-03-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <memory>
#include <iostream>
#include <boost/optional.hpp>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

//#define DEBUG
/**
 * @brief Defines basic functions supported in CAN communication.
 * 
 */
class CanDevice{

	private:
		int                   s;			/*!< Socket id, filled automatically */
		struct ifreq          ifr;       	/*!< Structure with CAN socket info */
		struct sockaddr_can * addr;         /*!< Reference to CAN socket address */
		std::string           dev_name;		/*!< CAN device name, dafaults to can0 */
		bool                  active; 		/*!< True if socket was opened and not closed */

        struct can_frame *    frame;        /*!< Reference to can_frame */

	
	public:
		/**
		 * @brief Inits connection with can device. 
		 * The adapter should be already configured. Creates a socket connection with the can device.
		 * 
		 * @return int Returns 0 at sucess, 1 at failure
		 */
		int init_connection();

		/**
		 * @brief Closes open socket connection.
		 * 
		 * @return int Returns 0 at sucess, 1 at failure
		 */
		int close_connection();

		/**
		 * @brief Sends a CAN message
		 * 
		 * @param[in] id Message's CAN id
		 * @param[in] dlc Message's length in bytes
		 * @param[in] data Data to be sent
		 * @param[in] rtr Bool, defines if the message should have a response
		 * @return int Returns 0 at sucess, 1 at failure
		 */
		int send(uint16_t id, uint16_t dlc, char * data, bool rtr = false); // send can message

				/**
		 * @brief Sends a CAN message
		 * 
		 * @param[in] id Message's CAN id
		 * @param[in] dlc Message's length in bytes
		 * @param[in] rtr Bool, defines if the message should have a response
		 * @return int Returns 0 at sucess, 1 at failure
		 */
		int send(uint16_t id, uint16_t dlc, bool rtr = false); // send can message
		
		/**
		 * @brief Set the message filter.
		 * The reciever will only get the messages that are compatible with the filter.
		 * 
		 * @param[in] id Message's CAN id
		 * @param[in] mask Mask for message's id
		 * @return int Returns 0 at sucess, 1 at failure
		 */
		int set_filter(uint16_t id, uint16_t mask=0xFFF);

		/**
		 * @brief Recieves Can messages.
		 * 
		 * @param[out] frame Pointer to frame to store Can message
		 * @param[out] recieved timestamp
		 * @param[in] msg_num Defines how meany messages should be read
		 * @return int Returns 0 at sucess, 1 at failure
		 */
		int recieve(struct can_frame * frame,struct timeval * timestamp, int msg_num);

		CanDevice(): dev_name("can0"), active(false){};

		CanDevice(std::string name): dev_name(name), active(false){};

		~CanDevice(){
			if (this->active) close_connection();  // if connection is active, close socket
		};

        void send_message( uint16_t messageID, uint16_t msgLen, char * data=NULL, bool respond=false){
            
            send(messageID, msgLen, data, respond); 
            
        };
};

static inline std::ostream& operator<< (std::ostream &out, struct timeval const* time) {
    out << time->tv_sec << " s " << time->tv_usec << " us";
    return out;
}

static inline std::ostream& operator<< (std::ostream &out, struct timeval const& time) {
    out << time.tv_sec << " s " << time.tv_usec << " us";
    return out;
}
