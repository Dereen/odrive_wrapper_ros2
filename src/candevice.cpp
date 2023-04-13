/**
 * @file candevice.cpp
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Class for basic socketCAN communication framework 
 * @version 0.1
 * @date 2023-03-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "candevice.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

int CanDevice::send(uint16_t id, uint16_t dlc, char *data=NULL, bool rtr=false)
{

	struct can_frame f;
	// define can frame header
	f.can_id = rtr ? id + CAN_RTR_FLAG : id;
	f.can_dlc = dlc;

	if (data)	{ // fill recieved data
		for (int i =0; i< dlc; i++){
			f.data[i] = *data;
			data++;
			}
		}

	// write created can message
	if (write(s, &f, sizeof(struct can_frame)) != sizeof(struct can_frame))	{
		perror("Write");
		return 1;
	}

	return 0;
}

int CanDevice::set_filter(uint16_t id, uint16_t mask)
{
	std::cout << "[CanDev] Set message filter for msg id " << id << " with mask " << mask << std::endl;
	
	int nbytes;
	struct can_frame frame;
	struct can_filter rfilter[1];

	rfilter[0].can_id = id;
	rfilter[0].can_mask = mask;

	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

	nbytes = read(s, &frame, sizeof(struct can_frame));

	if (nbytes < 0)	{
		perror("Read");
		return 1;
	}

#ifdef DEBUG
	printf("0x%03X [%d] ", frame.can_id, frame.can_dlc);

	for (i = 0; i < frame.can_dlc; i++)
		printf("%02X ", frame.data[i]);

	printf("\r\n");
#endif

	return 0;
}

int CanDevice::recieve(struct can_frame *frame, int msg_num)
{
	// read a single can frame

	int nbytes = read(s, frame, sizeof(struct can_frame) * msg_num);

	if (nbytes < 0)	{ // nothing to read -> error, should at least get heartbeat
		perror("Read");
		return 1;
	}

#ifdef DEBUG
	for (int j = 0; j < msg_num; ++j)
	{
		printf("0x%03X [%d] ", frame->can_id, frame->can_dlc);

		for (int i = 0; i < frame->can_dlc; i++)
			printf("%02X ", frame->data[i]);

		printf("\r\n");
	}
#endif

	return 0;
}

int CanDevice::init_connection()
{
	std::cout << "[CanDev] Init connection" << std::endl;

	if ((this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0){
		perror("Socket");
		return 1;
	}

	strcpy(this->ifr.ifr_name, this->dev_name.c_str()); // fill the ifreq name parameter by value can0
	ioctl(this->s, SIOCGIFINDEX, &this->ifr);			// Get the index number of a Linux network interface

	// set can address
	struct sockaddr_can tmp;
	addr = &tmp;
	memset(&tmp, 0, sizeof(tmp));

	tmp.can_family = AF_CAN;
	tmp.can_ifindex = this->ifr.ifr_ifindex;

	// open socket
	if (bind(this->s, (struct sockaddr *)&tmp, sizeof(tmp)) < 0)	{
		perror("Bind");
		return 1;
	}
	this->active = true;

	std::cout << "[CanDev] Opened CAN socket" << std::endl;

	return 0;
}

int CanDevice::close_connection()
{
	// close socket
	if (close(this->s) < 0)	{
		perror("Close");
		return 1;
	}

	this->active = false;
	return 0;
}