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
#include "datastructs.h"

class OdriveAxis
{
    friend std::ostream &operator<<(std::ostream &out, const OdriveAxis *obj);

private:
    // set_axis_node_id
    int id; /*!< Axis ID */

    std::shared_ptr<axisVersion> ver;       /*!< stores version information */
    std::shared_ptr<axisErrors> err;        /*!< stores errors information */
    std::shared_ptr<gains> gain;            /*!< stores gains*/
    std::shared_ptr<tempStruct> temp;       /*!< stores temperature measurements*/
    std::shared_ptr<encoderStruct> encoder; /*!< stores encoder estimates */
    std::shared_ptr<iqStruct> iq;           /*!< stores iq measurements*/
    std::shared_ptr<busUI> ui;              /*!< stores bus's voltage and current measurements*/
    std::shared_ptr<axisState> state;       /*!< stores axis state*/
    std::shared_ptr<adcVoltage> adc;        /*!< stores ADC voltage measurements*/
    std::unique_ptr<axisRegSettings> reg;   /*!< stores regulator settings*/

    // set_axis_state
    uint32_t axis_requested_state;

    // set_absolute_position
    uint32_t position;

    // reboot
    struct timeval reboot_timestamp;

    // estop
    bool estop;

    /**
     * @brief Function used to default initialization of OdriveAxis
     * Creates containers for individual structs and inits their timestamp to -1
     */
    void init();

public:
    /**
     * @brief Construct a new Odrive Axis:: Odrive Axis object
     *
     */
    OdriveAxis() : id(0)
    {
        this->init();
    };

    /**
     * @brief Construct a new Odrive Axis:: Odrive Axis object
     *
     * @param[in] id number of axes connected to odrive
     */
    OdriveAxis(int id) : id(id)
    {
        this->init();
    };

    /**
     * @brief Set the axis version object.
     * Fills corresponding structures of OdriveAxis upon recieving the get_version message
     *
     * @param[in] hw_version_major
     * @param[in]  hw_version_minor
     * @param[in]  hw_version_variant
     * @param[in]  fw_version_major
     * @param[in]  fw_version_minor
     * @param[in]  fw_version_revision
     * @param[in]  timestamp of the recieved message
     */
    void set_axis_version(uint8_t hw_version_major, uint8_t hw_version_minor,
                          uint8_t hw_version_variant, uint8_t fw_version_major,
                          uint8_t fw_version_minor, uint8_t fw_version_revision,
                          struct timeval timestamp);

    /**
     * @brief Set the axis state object.
     * Fills the corresponding structures of OdriveAxis upon recieving the heartbeat message
     *
     * @param[in] error axis_error
     * @param[in] state axis_state
     * @param[in] procedure_result
     * @param[in] trajectory_done_flag
     * @param[in] timestamp
     */
    void set_axis_state(uint8_t error, uint8_t state, uint8_t procedure_result,
                        bool trajectory_done_flag, struct timeval timestamp);

    /**
     * @brief Set the error structure.
     * Fills the corresponding structures of OdriveAxis upon recieving the get_error message
     *
     * @param[in] errors active_errors
     * @param[in] disarm_reason
     * @param[in] timestamp
     */
    void update_error(uint32_t errors, uint32_t disarm_reason, struct timeval timestamp);

    /**
     * @brief Set the encoder's position and velocity estimates upon recieving get_encoders_estimates.
     *
     * @param[in] pos pos_estimate
     * @param[in] vel vel_estimate
     * @param[in] timestamp
     */
    void update_estimates(uint32_t pos, uint32_t vel, struct timeval timestamp);

    /**
     * @brief Set the iq estimates upon recieving get_iq response.
     *
     * @param[in] setpoint iq_setpoint
     * @param[in] measured iq_measured
     * @param[in] timestamp
     */
    void update_iq(uint32_t setpoint, uint32_t measured, struct timeval timestamp);

    /**
     * @brief Updates the stored temperature readings upon recieving get_temperature response.
     *
     * @param[in] fet FET_temperature
     * @param[in] motor motor_temperature
     * @param[in] timestamp
     */
    void update_temp(uint32_t fet, uint32_t motor, struct timeval timestamp);

    /**
     * @brief updates stored bus's voltage and current apon recieving response to get_bus_voltage_current.
     *
     * @param[in] voltage bus_voltage
     * @param[in] current bus_current
     * @param[in] timestamp
     */
    void update_ui(uint32_t voltage, uint32_t current, struct timeval timestamp);

    /**
     * @brief updates stored ADC value after recieving get_adc_voltage reponse.
     *
     * @param[in] voltage ADC_voltage
     * @param[in] timestamp
     */
    void update_adc(uint32_t voltage, struct timeval timestamp);

    /**
     * @brief updates stored controller error upn recieving get_controller_error response
     *
     * @param[in] error controller_error
     * @param[in] timestamp
     */
    void update_controller_err(uint32_t error, struct timeval timestamp);

    void update_axis_requested_state(uint32_t state);

    uint32_t get_axis_requested_state();

    void update_controller_mode(uint32_t control_mode, uint32_t input_mode);

    uint32_t get_controll_mode();

    uint32_t get_input_mode();

    void update_input_pos(uint32_t input_pos, float vel_ff, float torque_ff);

    void update_input_vel(uint32_t input_vel, uint32_t input_torque_ff);

    void update_input_torque(uint32_t input_torque);

    void update_limits(uint32_t velocity, uint32_t current);

    void update_traj_vel_limit(uint32_t lim);

    void update_traj_accel_limit(uint32_t accel, uint32_t decel);

    void update_traj_inertia(uint32_t inertia);

    void update_absolut_pos(uint32_t pos);

    void update_pos_gain(uint32_t gain);

    void update_vel_gains(uint32_t gain, uint32_t integrator_gain);

    void update_anticogging(struct timeval b);

    void update_estop(bool b);

    void update_reboot_timestamp(struct timeval stamp);
};
