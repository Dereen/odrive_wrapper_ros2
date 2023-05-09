/**
 * @file datastruct.h
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Structures for storing odrive settings info
 * @version 0.1
 * @date 2023-05-09
 *
 # @copyright (c) JettyVision s.r.o in Prague 2023 - All Rights Reserved
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
    uint8_t hw_version_major {0};
    uint8_t hw_version_minor {0};

    uint8_t hw_version_variant {0};

    uint8_t fw_version_major {0};
    uint8_t fw_version_minor {0};

    uint8_t fw_version_revision {0};

    struct timeval timestamp {-1, -1};

} axisVersion;

/**
 * @brief Struct for containing information recieved by CAN about active errors 
 * Concatenates responces from heartbeat, get_error and get_controller_error
 */
typedef struct
{
    uint32_t axis_error {0};       // heartbeat
    uint32_t disarm_reason {0};    // get_error
    uint32_t active_errors {0};    // get_error
    uint32_t controller_error {0}; // Get_Controller_Error
    struct timeval timestamp {-1, -1};
} axisErrors;

/**
 * @brief Struct for containing information about regulator settings
 */
typedef struct
{
    // set_controller_mode
    uint32_t control_mode {0};
    uint32_t input_mode {0};
    // set_limits
    uint32_t velocity_limit {0};
    uint32_t current_limit {0};
    struct timeval  anticogging_timestamp {-1, -1};
    // set_input_pos
    float input_pos {0};
    float vel_ff {0};
    float torque_ff {0};
    // set_input_vel
    float input_vel {0};
    float input_torque_ff {0};
    // set_input_torque
    float input_torque {0};
    // set_traj_vel_limit
    float traj_vel_limit {0};
    // set_traj_accel_limits
    float traj_accel_limit {0};
    float traj_decel_limit {0};
    // set_traj_inertia
    float traj_inertia {0};

    struct timeval timestamp {-1, -1};

} axisRegSettings;

/**
 * @brief Struct for containing information about gains
 */
typedef struct
{
    // set_pos_gain
    float pos_gain {0};
    // set_vel_gains
    float vel_gain {0};
    float vel_integrator_gain {0};
} gains;

/**
 * @brief Struct for containing information about temperature measurements
 */
typedef struct
{
    // get_temperature
    float fet_temperature {0};
    float motor_temperature {0};
    struct timeval timestamp {-1, -1};
} tempStruct;

/**
 * @brief Struct for containing information about encoder estimates
 */
typedef struct
{
    // Get_Encoder_Estimates
    float pos_estimate {0};
    float vel_estimate {0};
    struct timeval timestamp {-1, -1};
} encoderStruct;

/**
 * @brief Struct for containing information about iq values
 */
typedef struct
{
    // get_iq
    float iq_setpoint {0};
    float iq_measured {0};
    struct timeval timestamp {-1, -1};
} iqStruct;

/**
 * @brief Struct for containing information about bus voltage and bus current
 */
typedef struct
{
    // Get_Bus_Voltage_Current
    float bus_voltage {0};
    float bus_current {0};
    struct timeval timestamp {-1, -1};
} busUI;

/**
 * @brief Struct for containing information about the state of the axis
 * stores information recieved in heartbeat message but the errors (see axisErrors)
 */
typedef struct
{
    uint8_t ax_state {0};
    bool    trajectory_done_flag {false};
    uint8_t procedure_result {0};
    struct timeval timestamp {-1, -1};
} axisState;

/**
 * @brief Struct for containing information about ADC voltage
 */
typedef struct
{
    // ADC voltage
    float adc_voltage {0};
    struct timeval timestamp {-1, -1};
} adcVoltage;
