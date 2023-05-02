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
    float input_pos;
    float vel_ff;
    float torque_ff;
    // set_input_vel
    float input_vel;
    float input_torque_ff;
    // set_input_torque
    float input_torque;
    // set_traj_vel_limit
    float traj_vel_limit;
    // set_traj_accel_limits
    float traj_accel_limit;
    float traj_decel_limit;
    // set_traj_inertia
    float traj_inertia;

    struct timeval timestamp;

} axisRegSettings;

/**
 * @brief Struct for containing information about gains
 */
typedef struct
{
    // set_pos_gain
    float pos_gain;
    // set_vel_gains
    float vel_gain;
    float vel_integrator_gain;
} gains;

/**
 * @brief Struct for containing information about temperature measurements
 */
typedef struct
{
    // get_temperature
    float fet_temperature;
    float motor_temperature;
    struct timeval timestamp;
} tempStruct;

/**
 * @brief Struct for containing information about encoder estimates
 */
typedef struct
{
    // Get_Encoder_Estimates
    float pos_estimate;
    float vel_estimate;
    struct timeval timestamp;
} encoderStruct;

/**
 * @brief Struct for containing information about iq values
 */
typedef struct
{
    // get_iq
    float iq_setpoint;
    float iq_measured;
    struct timeval timestamp;
} iqStruct;

/**
 * @brief Struct for containing information about bus voltage and bus current
 */
typedef struct
{
    // Get_Bus_Voltage_Current
    float bus_voltage;
    float bus_current;
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
    float adc_voltage;
    struct timeval timestamp;
} adcVoltage;
