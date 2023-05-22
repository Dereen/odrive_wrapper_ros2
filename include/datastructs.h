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

#include "odriveenums.h"

/**
 * @brief Struct for containing information recieved in CAN AxisX_Get_Version
 */
typedef struct {
    uint8_t hw_version_major{0};
    uint8_t hw_version_minor{0};

    uint8_t hw_version_variant{0};

    uint8_t fw_version_major{0};
    uint8_t fw_version_minor{0};

    uint8_t fw_version_revision{0};

    struct timeval timestamp{0, 0};

} axisVersion;

/**
 * @brief Struct for containing information recieved by CAN about active errors 
 * Concatenates responces from heartbeat, get_error and get_controller_error
 */
typedef struct {
    AxisError axis_error{AxisError::AXIS_ERROR_NONE};
    DisarmReason disarm_reason{DisarmReason::NONE};
    uint32_t active_errors{0};    // get_error
    ControllerError controller_error{ControllerError::CONTROLLER_ERROR_NONE}; // Get_Controller_Error
    struct timeval timestamp{0, 0};
} axisErrors;

/**
 * @brief Struct for containing information about regulator settings
 */
typedef struct {
    // set_controller_mode
    ControlMode control_mode{ControlMode::CONTROL_MODE_UNKNOWN};
    InputMode input_mode{InputMode::INPUT_MODE_INACTIVE};
    // set_limits
    uint32_t velocity_limit{0};
    uint32_t current_limit{0};

    // set_input_pos
    float input_pos{0};
    float vel_ff{0};
    float torque_ff{0};
    // set_input_vel
    float input_vel{0};
    float input_torque_ff{0};
    // set_input_torque
    float input_torque{0};
    // set_traj_vel_limit
    float traj_vel_limit{0};
    // set_traj_accel_limits
    float traj_accel_limit{0};
    float traj_decel_limit{0};
    // set_traj_inertia
    float traj_inertia{0};

    // set_pos_gain
    float pos_gain{0};
    // set_vel_gains
    float vel_gain{0};
    float vel_integrator_gain{0};

    struct timeval anticogging_timestamp{0, 0};
    struct timeval timestamp{0, 0};

} axisRegSettings;

/**
 * @brief Struct for containing information about temperature measurements
 */
typedef struct {
    // get_temperature
    float fet_temperature{0};
    float motor_temperature{0};
    struct timeval timestamp{0, 0};
} tempStruct;

/**
 * @brief Struct for containing information about encoder estimates
 */
typedef struct {
    // Get_Encoder_Estimates
    float pos_estimate{0};
    float vel_estimate{0};
    struct timeval timestamp{0, 0};
} encoderStruct;

/**
 * @brief Struct for containing information about motor current values
 */
typedef struct {
    // get_iq
    float iq_setpoint{0};
    float iq_measured{0};
    struct timeval timestamp{0, 0};
} iqStruct;

/**
 * @brief Struct for containing information about bus voltage and bus current
 */
typedef struct {
    // Get_Bus_Voltage_Current
    float bus_voltage{0};
    float bus_current{0};
    struct timeval timestamp{0, 0};
} busUI;

/**
 * @brief Struct for containing information about the state of the axis
 * stores information recieved in heartbeat message but the errors (see axisErrors)
 */
typedef struct {
    AxisState axis_state {AxisState::AXIS_STATE_UNDEFINED};
    bool trajectory_done_flag{false};
    ProcedureResult procedure_result{ProcedureResult::SUCCESS};
    struct timeval timestamp{0, 0};
} axisState;

/**
 * @brief Struct for containing information about ADC voltage
 */
typedef struct {
    // ADC voltage
    float adc_voltage{0};
    struct timeval timestamp{0, 0};
} adcVoltage;
