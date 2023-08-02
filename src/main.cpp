#include <stdio.h>
#include <unistd.h>
#include <iostream>

#include "odrive_wrapper/odrivecan.h"

using namespace odrive_wrapper;

int main(int argc, char **argv) {
    printf("create can device\n");
    OdriveCan odrive(2);

    sleep(1);
    std::cout << odrive.call_set_axis_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL);
    std::cout << odrive.call_set_axis_state(1, AXIS_STATE_CLOSED_LOOP_CONTROL);
    //odrive->call_start_anticogging(1);
    //odrive->call_set_controller_mode(0, CONTROL_MODE_VELOCITY_CONTROL, 1);
    sleep(1);
    //odrive->call_set_input_vel(0, 0xADDDDD41, 0);
    odrive.call_set_input_vel(0, 10, 0);
    odrive.call_set_input_vel(1, 1, 0);
    // odrive->call_reboot(0);
    //odrive->call_get_controller_error(0);

    const OdriveAxis &data = odrive[0];


    // print axis stats
    for (auto i = 0; i < 5; i++) {
        std::cout << odrive << std::endl;

        //odrive->call_get_error(0);

        sleep(1);
        //odrive->call_get_tempterature(0);
        odrive.call_set_absolute_position(0, 123);
    }
    //odrive->call_set_input_vel(0, 0, 0);

    std::cout << odrive << std::endl;

    odrive.call_set_axis_state(0, AXIS_STATE_IDLE);
    odrive.call_set_axis_state(1, AXIS_STATE_IDLE);

    return 0;
}