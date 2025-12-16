/**
 * @file candevice.cpp
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Class for basic socketCAN communication framework 
 * @version 0.1
 * @date 2023-03-02
 * 
 # @copyright (c) JettyVision s.r.o in Prague 2023 - All Rights Reserved
 * 
 */
#include "odrive_wrapper/candevice.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <cstring>
#include <sys/time.h>

using namespace odrive_wrapper;

int CanDevice::send(uint16_t id, uint16_t dlc, char *data, bool rtr) {

    struct can_frame f;
    // define can frame header
    f.can_id = rtr ? id + CAN_RTR_FLAG : id;
    f.can_dlc = dlc;

    if (data) { // fill data
        //*output_stream << "fill input data" << std::endl;
        for (int i = 0; i < dlc; i++) {
            f.data[i] = data[i];
        }
    }

    // write created can message
    int ret = write(s, &f, sizeof(struct can_frame));
    if (ret != sizeof(struct can_frame)) {
        *error_stream << "[CanDev] Write error: " << std::strerror(errno) << std::endl;
        return -1;
    }

#ifdef DEBUG
   *output_stream << "-> 0x%03X [%d] ", f.can_id, f.can_dlc;

    for (int i = 0; i < f.can_dlc; i++)
       *output_stream << "%02X ", f.data[i];

   *output_stream << std::endl;

#endif

    return 0;
}

int CanDevice::send(uint16_t id, uint16_t dlc, bool rtr) {

    struct can_frame f;

    //memset(&f, 0, sizeof(struct can_frame));
    // define can frame header
    f.can_id = rtr ? id + CAN_RTR_FLAG : id;
    f.can_dlc = dlc;
    memset(f.data, 0, dlc);

    // write created can message
    int ret = write(s, &f, sizeof(struct can_frame));
    if (ret != sizeof(struct can_frame)) {
        *error_stream << "[CanDev] Write error: " << std::strerror(errno) << std::endl;
        return -1;
    }

#ifdef DEBUG
   *output_stream << "-> 0x%03X [%d] ", f.can_id, f.can_dlc;

    for (int i = 0; i < f.can_dlc; i++)
       *output_stream << "%02X ", f.data[i];

   *output_stream << std::endl;

#endif

    return 0;
}

int CanDevice::set_filter(uint16_t id, uint16_t mask) {
    int nbytes;
    struct can_frame frame;
    struct can_filter rfilter[1];

    rfilter[0].can_id = id;
    rfilter[0].can_mask = mask;

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    nbytes = read(s, &frame, sizeof(struct can_frame));

    if (nbytes < 0) {
        *error_stream << "[CanDev] Read error: " << std::strerror(errno) << std::endl;
        return 1;
    }

#ifdef DEBUG
   *output_stream << "0x%03X [%d] ", frame.can_id, frame.can_dlc;

    for (int i = 0; i < frame.can_dlc; i++)
       *output_stream << "%02X ", frame.data[i];

   *output_stream << std::endl;
#endif

    return 0;
}

int CanDevice::recieve(struct can_frame *frame, struct timeval *timestamp, int msg_num) {
    int nbytes = read(s, frame, sizeof(struct can_frame) * msg_num);
    if (nbytes < 0) {
        *output_stream << "Read error: " << std::strerror(errno) << std::endl;
        return 1;
    } else {
#ifdef SIOCGSTAMP
        int error = ioctl(s, SIOCGSTAMP, timestamp);
#else
        int error = ioctl(s, SIOCGSTAMP_OLD, timestamp);
#endif
        if (error < 0) {
            *error_stream << "[CanDev] Read error: " << std::strerror(errno) << std::endl;
            return 1;
        }
    }

    return 0;
}

int CanDevice::init_connection() {
    if ((this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        *error_stream << "[CanDev] Socket error: " << std::strerror(errno) << std::endl;
        return 1;
    }
    strcpy(this->ifr.ifr_name, this->dev_name.c_str());
    ioctl(this->s, SIOCGIFINDEX, &this->ifr);
    struct sockaddr_can tmp;
    addr = &tmp;
    memset(&tmp, 0, sizeof(tmp));
    tmp.can_family = AF_CAN;
    tmp.can_ifindex = this->ifr.ifr_ifindex;
    if (bind(this->s, (struct sockaddr *) &tmp, sizeof(tmp)) < 0) {
        *error_stream << "[CanDev] Bind: " << std::strerror(errno) << std::endl;
        return 1;
    }

    this->active = true;
    *output_stream << "[CanDev] Opened CAN socket with device " << this->dev_name.c_str() << std::endl;
    return 0;
}

int CanDevice::close_connection() {
    // close socket only if valid (not -1)
    if (this->s >= 0) {
        if (close(this->s) < 0) {
            *error_stream << "[CanDev] Close: " << std::strerror(errno) << std::endl;
            return 1;
        }
    }

    this->active = false;
    return 0;
}

bool CanDevice::is_connected(void) {
    return fcntl(s, F_GETFD) != -1 || errno != EBADF;
}