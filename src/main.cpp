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
	OdriveCan * odrive = new OdriveCan(1);

	printf("Ask to get version\n");
	odrive->call_get_version(0);
	odrive->call_get_bus_ui(0);
	odrive->call_get_adc_voltage(0);
	//odrive->call_start_anticogging(1);
	odrive->call_set_controller_mode(0, 1, 1);
	sleep(1);
	odrive->call_reboot(0);

	// print axis stats
	for (auto i =0 ; i< 2; i++){
		std::cout << odrive << std::endl;

		sleep(1);
		odrive->call_get_tempterature(0);
	//	odrive->call_set_absolute_position(1, 123);
	}

	std::cout << odrive << std::endl;

	return 0;
}