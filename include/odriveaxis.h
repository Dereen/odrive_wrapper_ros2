/**
 * @file odriveaxis.h
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Structure for storing odrive settings info
 * @version 0.1
 * @date 2023-03-21
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
    uint32_t axis_requested_state;

    // set_absolute_position
    float position;

    // reboot
    struct timeval reboot_timestamp;

    // estop
    struct timeval estop;

    friend std::ostream& operator<<(std::ostream &out, const OdriveAxis& odrive);
};
