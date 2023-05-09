/**
 * @file odriveaxis.h
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Structure for storing odrive settings info
 * @version 0.1
 * @date 2023-05-09
 *
 # @copyright (c) JettyVision s.r.o in Prague 2023 - All Rights Reserved
 *
 */

#pragma once
#include <stdint.h>
#include <memory>
#include <cstring>
#include <iostream>
#include "candevice.h"
#include "datastructs.h"
#include "odriveenums.h"
#include "helpers.h"

/**
 * @brief A struct for storing Odrive's simpleCAN mesasge data and communication via CAN interface
 *
 */
struct OdriveAxis
{
    // set_axis_node_id
    int id {0}; /*!< Axis ID */

    axisVersion ver;       /*!< stores version information */
    axisErrors err;        /*!< stores errors information */
    gains gain;            /*!< stores gains*/
    tempStruct temp;       /*!< stores temperature measurements*/
    encoderStruct encoder; /*!< stores encoder estimates */
    iqStruct iq;           /*!< stores iq measurements*/
    busUI ui;              /*!< stores bus's voltage and current measurements*/
    axisState state;       /*!< stores axis state*/
    adcVoltage adc;        /*!< stores ADC voltage measurements*/
    axisRegSettings reg;   /*!< stores regulator settings*/

    // set_axis_state
    uint32_t axis_requested_state {0};

    // set_absolute_position
    float position {0.0f};

    // reboot
    struct timeval reboot_timestamp {-1, -1};

    // estop
    struct timeval estop {-1, -1};

    // can communication is active
    struct timeval can_active{-1};

    friend std::ostream& operator<<(std::ostream &out, const OdriveAxis& odrive);
};
