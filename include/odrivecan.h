/**
 * @file OdriveCan.h
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Class for Odrive S1 (PRO) CAN messages ID 
 * @version 0.1
 * @date 2023-03-03
 * 
 * @copyright Copyright (c) 2023
 * see https://docs.odriverobotics.com/v/latest/can-protocol.html#transport-protocol
*/

#pragma once
#include <iostream>
#include <vector>
#include <memory>
 
#include "candevice.h"
#include "odriveaxis.h"

/**
 * @brief time constants at which the periodically updated data will be fetched 
 */
struct updatePeriods{
uint32_t heartbeat;    /*!<  error, state, done flag */
uint32_t busIU;        /*!<  bus current bus voltage */
}; 

#define CAN_MSGS_LEN  8, 7, 0, 8, 4, 4, 8, 8, 8, 8, 4, 8, 0, 4, 8, 4, 8, 8, 0, 8, 0, 4, 4, 8, 4, 4, 0


class OdriveCan : OdriveAxis
{
    public:
	    std::shared_ptr<struct updatePeriods> periods; /*!< time constants at which the periodically updated data will be fetched */

        const int canMsgLen[28]; 


    private:
        std::vector<std::shared_ptr<class OdriveAxis>>  axes;
        int axes_num;
        CanDevice * can_dev;

        struct can_frame * frame;

        enum{
            GET_VERSION           = 0x000,
            HEARTBEAT,
            ESTOP,
            GET_ERROR,
            SET_AXIS_NODE_ID      = 0x006,
            SET_AXIs_STATE,
            GET_ENCODER_ESTIMATES = 0x009,
            SET_CONTROLLER_MODE   = 0x00b,
            SET_INPUT_POS,
            SET_INPUT_VEL,
            SET_INPUT_TORQUE,
            SET_LIMITS,
            START_ANTICOGGING,
            SET_TRAJ_VEL_LIMIT,
            SET_TRAJ_ACCEL_LIMITS,
            SET_TRAJ_INERTIA,
            GET_IQ,
            GET_TEMP,
            REBOOT,
            GET_BUS_VOLTAGE_CURRENT,
            CLEAR_ERRORS,
            SET_ABSOLUTE_POSITION,
            SET_POS_GAIN,
            SET_VEL_GAINS,
            GET_ADC_VOLTAGE,
            GET_CONTROLLER_ERROR,
            ENTER_DFU_MODE
        };
        

    public:

        bool set_regulator(){};

        // struct axisErrors get_errors(){};

        OdriveCan(): axes_num(6), canMsgLen{CAN_MSGS_LEN}{

            // create shared pointers for each Odrive axis and init the instances of class Axis
            for (int i =0; i < axes_num; i++)
                    axes.push_back(std::make_shared<class OdriveAxis>(new OdriveAxis(i)));

            // init update periods
            periods  = std::make_shared<struct updatePeriods>();
			set_periods(100, 100);
        };

        OdriveCan(int axes_num) : axes_num(axes_num), canMsgLen{CAN_MSGS_LEN}{

            // create shared pointers for each Odrive axis and init the instances of class Axis
            for (int i =0; i < axes_num; i++)
                    axes.push_back(std::make_shared<class OdriveAxis>(new OdriveAxis(i)));

            //init update periods
			periods  = std::make_shared<struct updatePeriods>();
			set_periods(100, 100);
        };

        ~OdriveCan(){};

        /**
		 * @brief Set the periods object
		 * 
		 * @param[in] heartbeat period at which heartbeat msg is processed
		 * @param[in] busUI period at which bus voltage and current are updated
		 */
		void set_periods(int32_t heartbeat, int32_t busUI){

			periods->heartbeat = heartbeat;
			periods->busIU = busUI;
		}

        struct axisVersion get_version(int axisID){

        // construct can message
        int msg_id = (axisID * 32) + GET_VERSION; // axis ID + can msg name
        char buf[canMsgLen[GET_VERSION]] ;

        int ret = can_dev->send(msg_id, buf, canMsgLen[GET_VERSION], true);

        if (ret < 0)
            throw std::runtime_error("The wrong number of bytes were written to CAN");

        // set filter to response
        can_dev->set_filter(msg_id);

        // recieve response
        ret = can_dev->recieve(frame, 1);
        if (ret)
            throw std::runtime_error("Nothing to read on CAN");

        // parse response
        int response_id;
        int cmd_id;
        parse_header(frame->can_id, &response_id, &cmd_id);

        // store recieved data

        }
/**
 * @brief parses recieved can header
 * 
 * @param[in] header can_id from can_frame
 * @param[out] axisID corresponding odrive axis id
 * @param[out] cmdID command id
 */
        void parse_header(uint32_t header, int * axisID, int * cmdID ) {
            uint32_t id = header & ((1u << 29) - 1);// lower 29 bits

            bool error_msg = header & ((1u << 30) - 1);

            // parse axis ID, mesgs of axis 0 start at 0x00, axis 1 - 0x20, axis 2- 0x40 .. axis 6- 0xc0
            *axisID = (id & 0x10) / 2;

            *cmdID = (header & 0x11) - (header & 0x10);

        }

};