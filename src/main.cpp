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
	OdriveCan * odrive = new OdriveCan(2);

	printf("Ask to get version\n");
	odrive->call_get_version(1);
	odrive->call_get_bus_ui(1);
	odrive->call_get_adc_voltage(1);

	// print axis stats
	for (auto i =0 ; i< 2; i++){
		std::cout << odrive << std::endl;

		sleep(1);
	}

	return 0;
}