/**
 * @file odriveaxis.h
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Structure for storing odrive settings info
 * @version 0.1
 * @date 2023-03-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include <stdint.h>
#include <memory>
#include "candevice.h"


/**
 * @brief Struct for containing information recieved in CAN AxisX_Get_Version
 * 
 */
struct {
    uint8_t hw_version_major;
    uint8_t hw_version_minor;

    uint8_t hw_version_variant;

    uint8_t fw_version_major;
    uint8_t fw_version_minor;

    uint8_t fw_version_revision;

}axisVersion;

struct{
    uint32_t axis_error;        // heartbeat
    uint32_t disarm_reason;     // get_error
    uint32_t active_errors;     // get_error
    uint32_t controller_error;  // Get_Controller_Error
}axisErrors;


struct{
    // set_controller_mode
    uint32_t control_mode;
    uint32_t input_mode;
    //set_limits
    uint32_t velocity_limit;
    uint32_t current_limit;
    bool start_anticogging;
    // set_input_pos
    uint32_t input_pos;
    float vel_ff;
    float input_torque_ff;
    // set_input_vel
    uint32_t input_vel;
    uint32_t input_torque_ff;
    //set_input_torque
    uint32_t input_torque;
    // set_traj_vel_limit
    uint32_t traj_vel_limit;
    //set_traj_accel_limits
    uint32_t traj_accel_limit;
    uint32_t traj_decel_limit;
    //set_traj_inertia
    uint32_t traj_inertia;

}axisRegSettings;

struct{
    // set_pos_gain
    uint32_t pos_gain;
    // set_vel_gains
    uint32_t vel_gain;
    uint32_t vel_integrator_gain;
}gains;

struct{

        //get_temperature
        uint32_t fet_temperature;
        uint32_t motor_temperature;
}temp;

struct{

        // Get_Encoder_Estimates
        uint32_t pos_estimate;
        uint32_t vel_estiamte;
} encoder;

struct{
        // get_iq
        uint32_t iq_setpoint;
        uint32_t iq_measured;
}iq;

struct{

        //Get_Bus_Voltage_Current
        uint32_t bus_voltage;
        uint32_t bus_current;
}busUI;

class OdriveAxis
{
    private:
        int id;  /*!< Axis ID */

        std::shared_ptr<struct axisVersion>  ver;
        std::shared_ptr<struct axisErrors>   err;
        std::shared_ptr<struct gains>        gains;
        std::shared_ptr<struct temp>         temp;
        std::shared_ptr<struct encoder>      encoder;
        std::shared_ptr<struct iq>           iq;
        std::shared_ptr<struct busUI>        busUI;


        // heartbeat
        uint8_t  state;
        uint32_t axis_requested_state; // set_axis_state
        bool     trajectory_done_flag;
        uint8_t  procedure_result;

        // set_absolute_position
        uint32_t position;

        // set_axis_node_id
        uint32_t node_id;

        // ADC voltage
        uint32_t adc_voltage;

        // reboot
        bool reboot;

        // estop
        bool estop;


    public:
        
        struct axisVersion parse_get_version(char * data);
        virtual bool set_regulator();
        virtual struct axisErrors get_errors();

        OdriveAxis():id(0) {
            ver   = std::make_shared<class axisVersion>();
            err   = std::make_shared<class axisErrors>();
            gains = std::make_shared<class gains>();
        };

        OdriveAxis(int id): id(id){};

};


