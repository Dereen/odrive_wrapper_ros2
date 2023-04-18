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
#include <iostream>
#include "candevice.h"


/**
 * @brief Struct for containing information recieved in CAN AxisX_Get_Version
 * 
 */
typedef struct {
    uint8_t hw_version_major;
    uint8_t hw_version_minor;

    uint8_t hw_version_variant;

    uint8_t fw_version_major;
    uint8_t fw_version_minor;

    uint8_t fw_version_revision;

    struct timeval timestamp;

}axisVersion;

typedef struct{
    uint32_t axis_error;        // heartbeat
    uint32_t disarm_reason;     // get_error
    uint32_t active_errors;     // get_error
    uint32_t controller_error;  // Get_Controller_Error
    struct timeval timestamp;
}axisErrors;


typedef struct{
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
    float torque_ff;
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

typedef struct{
    // set_pos_gain
    uint32_t pos_gain;
    // set_vel_gains
    uint32_t vel_gain;
    uint32_t vel_integrator_gain;
}gains;

typedef struct{

        //get_temperature
        uint32_t fet_temperature;
        uint32_t motor_temperature;
        struct timeval timestamp;
}tempStruct;

typedef struct{

        // Get_Encoder_Estimates
        uint32_t pos_estimate;
        uint32_t vel_estimate;
        struct timeval timestamp;
} encoderStruct;

typedef struct{
        // get_iq
        uint32_t iq_setpoint;
        uint32_t iq_measured;
        struct timeval timestamp;
}iqStruct;

typedef struct{

        //Get_Bus_Voltage_Current
        uint32_t bus_voltage;
        uint32_t bus_current;
        struct timeval timestamp;
}busUI;

typedef struct{

        uint8_t  ax_state;
        bool     trajectory_done_flag;
        uint8_t  procedure_result;
        struct timeval timestamp;
}axisState;

typedef struct {
    // ADC voltage
    uint32_t adc_voltage;
    struct timeval timestamp;
}adcVoltage;



class OdriveAxis
{
   // friend std::ostream& operator<< (std::ostream& out, const OdriveAxis& obj); 
   // friend std::ostream& operator<< (std::ostream &out, struct timeval const& time) ;
    private:
        int id;  /*!< Axis ID */

        std::shared_ptr<axisVersion>  ver;
        std::shared_ptr<axisErrors>   err;
        std::shared_ptr<gains>        gain;
        std::shared_ptr<tempStruct>   temp;
        std::shared_ptr<encoderStruct>      encoder;
        std::shared_ptr<iqStruct>     iq;
        std::shared_ptr<busUI>        ui;
        std::shared_ptr<axisState>    state;
        std::shared_ptr<adcVoltage>   adc;


        // heartbeat
        uint32_t axis_requested_state; // set_axis_state

        // set_absolute_position
        uint32_t position;

        // set_axis_node_id
        uint32_t node_id;

        // reboot
        bool reboot;

        // estop
        bool estop;

        void init(){

            gain  = std::make_shared<gains>();

            ver   = std::make_shared<axisVersion>();
            ver->timestamp.tv_sec = -1;

            err   = std::make_shared<axisErrors>();
            err->timestamp.tv_sec = -1;
            
            state = std::make_shared<axisState>();
            state->timestamp.tv_sec = -1;

            iq    = std::make_shared<iqStruct>();
            iq->timestamp.tv_sec = -1;

            temp  = std::make_shared<tempStruct>();
            temp->timestamp.tv_sec = -1;

            ui    = std::make_shared<busUI>();
            ui->timestamp.tv_sec = -1;

            encoder = std::make_shared<encoderStruct>();
            encoder->timestamp.tv_sec = -1;

            adc = std::make_shared<adcVoltage>();
            adc->timestamp.tv_sec = -1;
        }

    public:
        
      //  axisVersion parse_get_version(char * data);
      //  virtual bool set_regulator();
      //  virtual axisErrors get_errors();

        OdriveAxis():id(0) {
            this->init();
        };

        OdriveAxis(int id): id(id){
            this->init();
        };

        void set_axis_version(uint8_t hw_version_major,  uint8_t hw_version_minor, 
                                uint8_t hw_version_variant,   uint8_t fw_version_major,  
                                uint8_t fw_version_minor,uint8_t fw_version_revision,
                                struct timeval timestamp){
            
            this->ver->hw_version_major    = hw_version_major;
            this->ver->hw_version_minor    = hw_version_minor;
            this->ver->hw_version_variant  = hw_version_variant;
            this->ver->fw_version_major    = fw_version_major;
            this->ver->fw_version_minor    = fw_version_minor;
            this->ver->fw_version_revision = fw_version_revision;
            this->ver->timestamp           = timestamp;
        }

        void set_axis_state(uint8_t error, uint8_t  state, uint8_t  procedure_result, bool trajectory_done_flag, struct timeval timestamp){
            this->err->axis_error = error;
            this->err->timestamp  = timestamp;


            this->state->ax_state = state;
            this->state->trajectory_done_flag = trajectory_done_flag;
            this->state->procedure_result = procedure_result;
            this->state->timestamp = timestamp;
        }

        void update_error(uint32_t errors, uint32_t disarm_reason, struct timeval timestamp){
            this->err->active_errors = errors;
            this->err->disarm_reason = disarm_reason;
            this->err->timestamp = timestamp;
        }

        void update_estimates(uint32_t pos, uint32_t vel, struct timeval timestamp){
            this->encoder->pos_estimate = pos;
            this->encoder->vel_estimate = vel;
            this->encoder->timestamp = timestamp;
        }

        void update_iq(uint32_t setpoint, uint32_t measured, struct timeval timestamp){
            this->iq->iq_measured = measured;
            this->iq->iq_setpoint = setpoint;
            this->iq->timestamp = timestamp;
        }

        void update_temp(uint32_t fet, uint32_t motor, struct timeval timestamp){
            this->temp->fet_temperature = fet;
            this->temp->motor_temperature = motor;
            this->temp->timestamp = timestamp;
        }

        void update_ui(uint32_t voltage, uint32_t current, struct timeval timestamp){
            this->ui->bus_voltage = voltage;
            this->ui->bus_current = current;
            this->ui->timestamp = timestamp;
        }

        void update_adc(uint32_t voltage, struct timeval timestamp){
            this->adc->adc_voltage = voltage;
            this->adc->timestamp = timestamp;
        }
        
        void update_controller_err(uint32_t error, struct timeval timestamp){
            this->err->controller_error = error;
            this->err->timestamp = timestamp;
        }
};
/*
std::ostream& operator<< (std::ostream &out, OdriveAxis const& ax) {
    // print Odrive version
    if (ax.ver->timestamp.tv_sec > -1){
        out << "Odrive version: [" << ax.ver->timestamp << "]:" << std::endl;
        out << "\tHardware version " << (ax.ver->hw_version_major) << "." << (ax.ver->hw_version_minor)<< "." << ax.ver->hw_version_variant << std::endl;
        out << "\tFirmware version" << (ax.ver->fw_version_major) << "." << (ax.ver->fw_version_minor)<< "." << ax.ver->fw_version_revision << std::endl;
    }
    // print Axis state
    out << "Axis status [" << ax.err->timestamp << "]:" << std::endl;
        if(ax.state->timestamp.tv_sec > -1){
        out << "\tAxis state:" << ax.state->ax_state << std::endl;
        out << "\tProcedure result: " << ax.state->procedure_result << std::endl;
        out << "\tTrajectory done: " << (ax.state->trajectory_done_flag? "yes" : "no") << std::endl;
        if (ax.err->axis_error){
            out << "\tERROR" << std::endl;
            out << "\tActive errors: " << ax.err->active_errors << std::endl;
            out << "\tDisarm reason: " << ax.err->disarm_reason << std::endl;
            out << "\tController error: " << ax.err->controller_error << std::endl;

        }
    }

    // print Encoder estimates
    if (ax.encoder->timestamp.tv_sec > -1){
        out << "Encoder estimates [" << ax.encoder->timestamp << "]:" << std::endl;
        out << "\tPosition estimate: " << ax.encoder->pos_estimate << std::endl;
        out << "\tVelocity estimate: " << ax.encoder->vel_estimate << std::endl;
    }

    // current temperature
    if (ax.temp->timestamp.tv_sec > -1){
        out << "Temperature [" << ax.temp->timestamp << "]:" << std::endl;
        out << "\tFET temperature: "   << ax.temp->fet_temperature   << std::endl;
        out << "\tMotor temperature: " << ax.temp->motor_temperature << std::endl;
    }

    // current, voltage
    if (ax.ui->timestamp.tv_sec > -1){
        out << "Bus voltage and current [" << ax.ui->timestamp << "]:" << std::endl;
        out << "\tBus current: " << ax.ui->bus_current << std::endl;
        out << "\tBus voltage: " << ax.ui->bus_voltage << std::endl;
    }

    // ADC voltage
    if (ax.adc->timestamp.tv_sec > -1)
        out << "ADC voltage[" << ax.adc->timestamp << "]: " << ax.adc->adc_voltage << std::endl;

    return out;
}

std::ostream& operator<< (std::ostream &out, struct timeval const& time) {
    out << time.tv_sec << "s " << time.tv_usec << "us";
    return out;
}
*/