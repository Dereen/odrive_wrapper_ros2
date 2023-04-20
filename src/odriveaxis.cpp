/**
 * @file odriveaxis.cpp
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Structure for fetching and storing odrive settings info
 * @version 0.1
 * @date 2023-03-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "odriveaxis.h"

void OdriveAxis::init()
{

    gain = std::make_shared<gains>();

    ver = std::make_shared<axisVersion>();
    ver->timestamp.tv_sec = -1;

    err = std::make_shared<axisErrors>();
    err->timestamp.tv_sec = -1;

    state = std::make_shared<axisState>();
    state->timestamp.tv_sec = -1;

    iq = std::make_shared<iqStruct>();
    iq->timestamp.tv_sec = -1;

    temp = std::make_shared<tempStruct>();
    temp->timestamp.tv_sec = -1;

    ui = std::make_shared<busUI>();
    ui->timestamp.tv_sec = -1;

    encoder = std::make_shared<encoderStruct>();
    encoder->timestamp.tv_sec = -1;

    adc = std::make_shared<adcVoltage>();
    adc->timestamp.tv_sec = -1;
}

void OdriveAxis::set_axis_version(uint8_t hw_version_major, uint8_t hw_version_minor,
                                  uint8_t hw_version_variant, uint8_t fw_version_major,
                                  uint8_t fw_version_minor, uint8_t fw_version_revision,
                                  struct timeval timestamp)
{

    this->ver->hw_version_major = hw_version_major;
    this->ver->hw_version_minor = hw_version_minor;
    this->ver->hw_version_variant = hw_version_variant;
    this->ver->fw_version_major = fw_version_major;
    this->ver->fw_version_minor = fw_version_minor;
    this->ver->fw_version_revision = fw_version_revision;
    this->ver->timestamp = timestamp;
    std::cout << "Set axis version [" << timestamp << "] " << hw_version_major << std::endl;
}

void OdriveAxis::set_axis_state(uint8_t error, uint8_t state, uint8_t procedure_result, bool trajectory_done_flag, struct timeval timestamp)
{
    this->err->axis_error = error;
    this->err->timestamp = timestamp;

    this->state->ax_state = state;
    this->state->trajectory_done_flag = trajectory_done_flag;
    this->state->procedure_result = procedure_result;
    this->state->timestamp = timestamp;
}

void OdriveAxis::update_error(uint32_t errors, uint32_t disarm_reason, struct timeval timestamp)
{
    this->err->active_errors = errors;
    this->err->disarm_reason = disarm_reason;
    this->err->timestamp = timestamp;
}

void OdriveAxis::update_estimates(uint32_t pos, uint32_t vel, struct timeval timestamp)
{
    this->encoder->pos_estimate = pos;
    this->encoder->vel_estimate = vel;
    this->encoder->timestamp = timestamp;
}

void OdriveAxis::update_iq(uint32_t setpoint, uint32_t measured, struct timeval timestamp)
{
    this->iq->iq_measured = measured;
    this->iq->iq_setpoint = setpoint;
    this->iq->timestamp = timestamp;
}

void OdriveAxis::update_temp(uint32_t fet, uint32_t motor, struct timeval timestamp)
{
    this->temp->fet_temperature = fet;
    this->temp->motor_temperature = motor;
    this->temp->timestamp = timestamp;
}

void OdriveAxis::update_ui(uint32_t voltage, uint32_t current, struct timeval timestamp)
{
    this->ui->bus_voltage = voltage;
    this->ui->bus_current = current;
    this->ui->timestamp = timestamp;
}

void OdriveAxis::update_adc(uint32_t voltage, struct timeval timestamp)
{
    this->adc->adc_voltage = voltage;
    this->adc->timestamp = timestamp;
}

void OdriveAxis::update_controller_err(uint32_t error, struct timeval timestamp)
{
    this->err->controller_error = error;
    this->err->timestamp = timestamp;
}

std::ostream &operator<<(std::ostream &out, OdriveAxis const *ax)
{
    // print Odrive version
    if (ax->ver->timestamp.tv_sec > -1)
    {
        out << "Odrive version [" << ax->ver->timestamp << "]:" << std::endl;
        out << "\tHardware version " << unsigned(ax->ver->hw_version_major) << "." 
            << unsigned(ax->ver->hw_version_minor) << "." << (unsigned) ax->ver->hw_version_variant << std::endl;
        out << "\tFirmware version " << unsigned(ax->ver->fw_version_major) << "." 
            << unsigned(ax->ver->fw_version_minor) << "." << (unsigned) ax->ver->fw_version_revision << std::endl;
    }
    // print Axis state
    out << "Axis status [" << ax->state->timestamp << "]:" << std::endl;
    if (ax->state->timestamp.tv_sec > -1)
    {
        out << "\tAxis state:" << (unsigned) ax->state->ax_state << std::endl;
        out << "\tProcedure result: " <<(unsigned)  ax->state->procedure_result << std::endl;
        out << "\tTrajectory done: " << (ax->state->trajectory_done_flag ? "yes" : "no") << std::endl;
        if (ax->err->axis_error)
        {
            out << "\tERROR" << std::endl;
            out << "\tActive errors: " << ax->err->active_errors << std::endl;
            out << "\tDisarm reason: " << ax->err->disarm_reason << std::endl;
            out << "\tController error: " << ax->err->controller_error << std::endl;
        }
    }

    // print Encoder estimates
    if (ax->encoder->timestamp.tv_sec > -1)
    {
        out << "Encoder estimates [" << ax->encoder->timestamp << "]:" << std::endl;
        out << "\tPosition estimate: " << ax->encoder->pos_estimate << std::endl;
        out << "\tVelocity estimate: " << ax->encoder->vel_estimate << std::endl;
    }

    // current temperature
    if (ax->temp->timestamp.tv_sec > -1)
    {
        out << "Temperature [" << ax->temp->timestamp << "]:" << std::endl;
        out << "\tFET temperature: " << ax->temp->fet_temperature << std::endl;
        out << "\tMotor temperature: " << ax->temp->motor_temperature << std::endl;
    }

    // current, voltage
    if (ax->ui->timestamp.tv_sec > -1)
    {
        out << "Bus voltage and current [" << ax->ui->timestamp << "]:" << std::endl;
        out << "\tBus current: " << ax->ui->bus_current << std::endl;
        out << "\tBus voltage: " << ax->ui->bus_voltage << std::endl;
    }

    // ADC voltage
    if (ax->adc->timestamp.tv_sec > -1)
        out << "ADC voltage[" << ax->adc->timestamp << "]: " << ax->adc->adc_voltage << std::endl;

    return out;
}

    void OdriveAxis::update_axis_requested_state(uint32_t state){
        this->axis_requested_state = state;

    };

    uint32_t OdriveAxis::get_axis_requested_state(){
        return this->axis_requested_state;
    };