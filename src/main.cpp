#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <iostream>

#include "odrivecan.h"



int main(int argc, char **argv)
{
	printf("create can device\n");
	OdriveCan * odrive = new OdriveCan();
	// int s;
	// struct ifreq ifr;
	// struct sockaddr_can addr;
	// dev_init(&s, &ifr, &addr);
	// struct can_frame frame[2];
	printf("Ask to get version\n");
	odrive->get_version(0);
	// std::cout << odrive << std::endl;

	sleep(10);

	return 0;
}