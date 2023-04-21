/**
 * @file datastruct.h
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Structures for storing odrive settings info
 * @version 0.1
 * @date 2023-04-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once
#include <stdint.h>
#include <memory>
#include <iostream>

/**
 * @brief Struct for containing information recieved in CAN AxisX_Get_Version
 */
typedef struct
{
    uint8_t hw_version_major;
    uint8_t hw_version_minor;

    uint8_t hw_version_variant;

    uint8_t fw_version_major;
    uint8_t fw_version_minor;

    uint8_t fw_version_revision;

    struct timeval timestamp;

} axisVersion;

/**
 * @brief Struct for containing information recieved by CAN about active errors 
 * Concatenates responces from heartbeat, get_error and get_controller_error
 */
typedef struct
{
    uint32_t axis_error;       // heartbeat
    uint32_t disarm_reason;    // get_error
    uint32_t active_errors;    // get_error
    uint32_t controller_error; // Get_Controller_Error
    struct timeval timestamp;
} axisErrors;

/**
 * @brief Struct for containing information about regulator settings
 */
typedef struct
{
    // set_controller_mode
    uint32_t control_mode;
    uint32_t input_mode;
    // set_limits
    uint32_t velocity_limit;
    uint32_t current_limit;
    struct timeval  anticogging_timestamp;
    // set_input_pos
    uint32_t input_pos;
    float vel_ff;
    float torque_ff;
    // set_input_vel
    uint32_t input_vel;
    uint32_t input_torque_ff;
    // set_input_torque
    uint32_t input_torque;
    // set_traj_vel_limit
    uint32_t traj_vel_limit;
    // set_traj_accel_limits
    uint32_t traj_accel_limit;
    uint32_t traj_decel_limit;
    // set_traj_inertia
    uint32_t traj_inertia;

    struct timeval timestamp;

} axisRegSettings;

/**
 * @brief Struct for containing information about gains
 */
typedef struct
{
    // set_pos_gain
    uint32_t pos_gain;
    // set_vel_gains
    uint32_t vel_gain;
    uint32_t vel_integrator_gain;
} gains;

/**
 * @brief Struct for containing information about temperature measurements
 */
typedef struct
{
    // get_temperature
    uint32_t fet_temperature;
    uint32_t motor_temperature;
    struct timeval timestamp;
} tempStruct;

/**
 * @brief Struct for containing information about encoder estimates
 */
typedef struct
{
    // Get_Encoder_Estimates
    uint32_t pos_estimate;
    uint32_t vel_estimate;
    struct timeval timestamp;
} encoderStruct;

/**
 * @brief Struct for containing information about iq values
 */
typedef struct
{
    // get_iq
    uint32_t iq_setpoint;
    uint32_t iq_measured;
    struct timeval timestamp;
} iqStruct;

/**
 * @brief Struct for containing information about bus voltage and bus current
 */
typedef struct
{
    // Get_Bus_Voltage_Current
    uint32_t bus_voltage;
    uint32_t bus_current;
    struct timeval timestamp;
} busUI;

/**
 * @brief Struct for containing information about the state of the axis
 * stores information recieved in heartbeat message but the errors (see axisErrors)
 */
typedef struct
{
    uint8_t ax_state;
    bool    trajectory_done_flag;
    uint8_t procedure_result;
    struct timeval timestamp;
} axisState;

/**
 * @brief Struct for containing information about ADC voltage
 */
typedef struct
{
    // ADC voltage
    uint32_t adc_voltage;
    struct timeval timestamp;
} adcVoltage;
