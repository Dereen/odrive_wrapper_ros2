#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "candevice.h"


int send(int *s, struct can_frame * frame){
	struct can_frame f;
	f.can_id = 0x000 + CAN_RTR_FLAG;
	f.can_dlc = 8;
	//sprintf(frame->data, "Hello");

	if (write(*s, &f, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("Write");
		return 1;
	}

	return 0;
}


int set_filter(int *s, struct can_frame *frame){

	int i; 
	int nbytes;

	struct can_filter rfilter[1];

	rfilter[0].can_id   = 0x000;
	rfilter[0].can_mask = 0x00F;

	setsockopt(*s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

	nbytes = read(*s, frame, sizeof(struct can_frame));

	if (nbytes < 0) {
		perror("Read");
		return 1;
	}

	printf("0x%03X [%d] ",frame->can_id, frame->can_dlc);

	for (i = 0; i < frame->can_dlc; i++)
		printf("%02X ",frame->data[i]);

	printf("\r\n");
	printf("Return from can filter\n");

	return 0;

}


int recieve(int *s, struct can_frame * frame) {
	// read a single can frame
	int nbytes = read(*s, frame, sizeof(struct can_frame));

 	if (nbytes < 0) { // nothing to read -> error, should at least get heartbeat
		perror("Read");
		return 1;
	}

	printf("0x%03X [%d] ",frame->can_id, frame->can_dlc);

	for (int i = 0; i < frame->can_dlc; i++)
		printf("%02X ",frame->data[i]);

	printf("\r\n");
	return 0;
}

int dev_init(int * s,  struct ifreq * ifr, struct sockaddr_can * addr){

	if ((*s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return 1;
	}

	strcpy(ifr->ifr_name, "can0" ); // fill the ifreq name parameter by value can0
	ioctl(*s, SIOCGIFINDEX, ifr); // Get the index number of a Linux network interface

	// set can address
	memset(addr, 0, sizeof(*addr));

	addr->can_family = AF_CAN;
	addr->can_ifindex = ifr->ifr_ifindex;

	// open socket
	if (bind(*s, (struct sockaddr *) addr, sizeof(*addr)) < 0) {
		perror("Bind");
		return 1;
	}
	printf("Opened CAN socket\n");
	return 0;
}


int main(int argc, char **argv)
{
	printf("create can device\n");
	CanDevice cd = CanDevice();
	// int s;
	// struct ifreq ifr;
	//  struct sockaddr_can addr;
	//  dev_init(&s, &ifr, &addr);
	cd.init_connection();
	struct can_frame frame[2];
	cd.recieve(frame, 2);

	return 0;
}