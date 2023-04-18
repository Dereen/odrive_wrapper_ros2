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
#include <boost/circular_buffer.hpp>

#include <thread>
#include <mutex>

/**
 * @brief time constants at which the periodically updated data will be fetched
 */
struct updatePeriods
{
    uint32_t heartbeat; /*!<  error, state, done flag */
    uint32_t busIU;     /*!<  bus current bus voltage */
};

#define CAN_MSGS_LEN 8, 7, 0, 8, 4, 4, 8, 8, 8, 8, 4, 8, 0, 4, 8, 4, 8, 8, 0, 8, 0, 4, 4, 8, 4, 4, 0

class OdriveCan : OdriveAxis
{
friend OdriveAxis;
    typedef struct 
    {
        can_frame      frame;
        struct timeval timestamp;
    }canMsg;

public:
    std::shared_ptr<struct updatePeriods> periods; /*!< time constants at which the periodically updated data will be fetched */

    const int canMsgLen[28];

private:
    int axes_num;
    std::vector<std::shared_ptr<class OdriveAxis>> axes;

    int buffer_len;
    typedef boost::circular_buffer<canMsg> can_circ_buffer;
    std::vector<std::shared_ptr<can_circ_buffer>> input_buffer; // make a circular buffer for each axis

    std::unique_ptr<CanDevice> can_dev;

    bool run;

    std::mutex msg_mutex;

    std::thread th_recieve;
    std::thread th_process;

    enum
    {
        GET_VERSION           = 0x000,
        HEARTBEAT,
        ESTOP,
        GET_ERROR,
        SET_AXIS_NODE_ID      = 0x006,
        SET_AXIS_STATE,
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

    void init(){
        // create shared pointers for each Odrive axis and init the instances of class Axis
        for (int i = 0; i < axes_num; i++){
            axes.push_back(std::make_shared<class OdriveAxis>(i));

            // init circular buffer
            input_buffer.push_back(std::make_shared<can_circ_buffer>(buffer_len));
        }

        // init update periods
        periods = std::make_shared<struct updatePeriods>();
        set_periods(100, 100);

        // init can listener
        can_dev = std::make_unique<class CanDevice>();
        std::cout << "[OdriveCan] start threads" << std::endl;
        th_recieve = std::thread(&OdriveCan::receive_msgs, this);
        th_process = std::thread(&OdriveCan::process_msgs, this);

        can_dev->init_connection();
    }

public:
    //bool set_regulator(){};

    // struct axisErrors get_errors(){};

    OdriveCan() :  canMsgLen{CAN_MSGS_LEN}, axes_num(6), buffer_len(10), run(1)
    {
        std::cout << "[OdriveCAN] Init Ordive can constructor" << std::endl;
        this->init();
        
    };

    OdriveCan(int axes_num) : canMsgLen{CAN_MSGS_LEN},axes_num(axes_num), buffer_len(10), run(1)
    {
        this->init();
    };

    ~OdriveCan(){
        th_recieve.join();
        th_process.join();
    };

    /**
     * @brief Set the periods object
     *
     * @param[in] heartbeat period at which heartbeat msg is processed
     * @param[in] busUI period at which bus voltage and current are updated
     */
    void set_periods(int32_t heartbeat, int32_t busUI)
    {
        periods->heartbeat = heartbeat;
        periods->busIU = busUI;
    }

    int get_version(int axisID)
    {
        std::cout << "[GetVersion] ASK for version " << std::endl;
        // construct can message
        int msg_id = (axisID * 32) + GET_VERSION; // axis ID + can msg name
        std::cout << "[GetVersion] CAN msg ID" << msg_id << std::endl;

        //char buf[canMsgLen[GET_VERSION]];

        int ret = can_dev->send(msg_id, canMsgLen[GET_VERSION], NULL, true);

        if (ret < 0)
            throw std::runtime_error("The wrong number of bytes were written to CAN");

        return 0;

    }

    /**
     * @brief parses recieved can header
     *
     * @param[in] header can_id from can_frame
     * @param[out] axisID corresponding odrive axis id
     * @param[out] cmdID command id
     */
    void parse_header(uint32_t header, int *axisID, int *cmdID)
    {

        bool error_msg = header & ((1u << 30) - 1);
        if (!error_msg){
            // parse axis ID, mesgs of axis 0 start at 0x00, axis 1 - 0x20, axis 2- 0x40 .. axis 6- 0xc0
            *axisID = axis_from_header(header);

            *cmdID = can_id_from_header(header);
        }
    }

    /**
     * @brief Returns CAN message ID from header
     *
     * @param[in] header recieved CAN message header
     * @return int CAN message ID
     */
    int can_id_from_header(uint32_t header)
    {
        return (header & 0x1F);
    }

    /**
     * @brief returns axis ID from can_frame header
     *
     * @param[in] header CAN frame header
     * @return int axis ID
     */
    int axis_from_header(uint32_t header)
    {

        uint32_t id = header & ((1u << 29) - 1); // lower 29 bits
        return (id & 0x10) / 2;
    }
    /**
     * @brief recieves can messages and stores them in circluar buffers for each axis
     *
     */
    void receive_msgs()
    {

        std::cout << "[RECEIVE] Start recieve thread" << std::endl;
        int ax_id;
        struct timeval timestamp;
        canMsg msg;

        struct can_frame frame;

        while (run)
        {
            // read meessages
            //std::cout << "[RECEIVE] Recieve" << std::endl;
            can_dev->recieve(&frame, &timestamp, 1);
            ax_id = axis_from_header(frame.can_dlc);
            //std::cout << "recieved message from axis " << ax_id << std::endl;
            // store message with timestamp
            msg = {frame, timestamp};
            msg_mutex.lock();
            input_buffer[ax_id]->push_back(msg); // not sure if will work
            msg_mutex.unlock();

            if (input_buffer[ax_id]->full())
                throw std::runtime_error("Circular buffer is full");
        }

        std::cout << "[RECEIVE] End recieve thread" << std::endl;
    };

    void process_msgs()
    {
        std::cout << "[PROCESS] start processing thread" << std::endl;
        canMsg msg;
        while (run)
        {
            for (int i = 0; i < axes_num; i++)
            { // check buffer for each axis

                //if (!input_buffer[i]->empty()) std::cout << "[PROCESS] Input buffer of axis " << i << " is not empty" << std::endl;
                //else std::cout << "[PROCESS] Input buffer of axis  " << i <<" is empty" << std::endl;


                if (!input_buffer[i]->empty())
                    { // if buffer for given axis is not empty
                        //std::cout << "[PROCESS] Process message on axis " << i << std::endl;
                        msg_mutex.lock();
                        msg = input_buffer[i]->front();
                        input_buffer[i]->pop_front();
                        msg_mutex.unlock();
                    
                    int canID = can_id_from_header(msg.frame.can_id);

                    switch (canID)
                    {
                    case GET_VERSION:
                        parse_version(i, msg);
                        std::cout << "got version reply\n";
                        break;
                    case HEARTBEAT:
                        parse_heartbeat(i, msg);
                        break;
                    case ESTOP:
                        break;
                    case GET_ERROR:
                        parse_error(i, msg);
                        break;
                    case GET_ENCODER_ESTIMATES:
                        parse_encoder_estimates(i, msg);
                        break;
                    case GET_IQ:
                        parse_iq(i, msg);
                        break;
                    case GET_TEMP:
                        parse_temp(i, msg);
                        break;
                    case GET_BUS_VOLTAGE_CURRENT:
                        parse_ui(i, msg);
                        break;
                    case GET_ADC_VOLTAGE:
                        parse_adc(i, msg);
                        break;
                    case GET_CONTROLLER_ERROR:
                        parse_controller_error(i, msg);
                        break;
                    }
                }
            }
            usleep(100);
        }
    }

void parse_version(int axisID, canMsg msg){

   uint8_t hw_maj = uint8_t(msg.frame.data[1]);
   uint8_t hw_min = uint8_t(msg.frame.data[2]);
   uint8_t hw_var = uint8_t(msg.frame.data[3]);
   uint8_t fw_maj = uint8_t(msg.frame.data[4]);
   uint8_t fw_min = uint8_t(msg.frame.data[5]);
   uint8_t fw_rev = uint8_t(msg.frame.data[6]);

   axes[axisID]->set_axis_version(hw_maj, hw_min, hw_var,fw_maj, fw_min, fw_rev, msg.timestamp);
}

void parse_heartbeat(int axisID, canMsg msg){

    uint32_t err = get32from8(msg.frame.data, 0);

    axes[axisID]->set_axis_state(err, msg.frame.data[4], msg.frame.data[5], msg.frame.data[6], msg.timestamp);
    
}

void parse_error(int axisID, canMsg msg){
    uint32_t err = get32from8(msg.frame.data, 0);
    uint32_t reason = get32from8(msg.frame.data, 4);
    axes[axisID]->update_error(err, reason, msg.timestamp);
}

void parse_encoder_estimates(int axisID, canMsg msg){
    uint32_t err =  get32from8(msg.frame.data, 0);
    uint32_t reason =  get32from8(msg.frame.data, 4);
    axes[axisID]->update_estimates(err, reason, msg.timestamp);
}

void parse_iq(int axisID, canMsg msg){
    uint32_t setpoint =  get32from8(msg.frame.data, 0);
    uint32_t measured =  get32from8(msg.frame.data, 4);
    axes[axisID]->update_iq(setpoint, measured, msg.timestamp);
}

void parse_temp(int axisID, canMsg msg){
    uint32_t fet =  get32from8(msg.frame.data, 0);
    uint32_t motor =  get32from8(msg.frame.data, 4);
    axes[axisID]->update_temp(fet, motor, msg.timestamp);
}

void parse_ui(int axisID, canMsg msg){
    uint32_t voltage =  get32from8(msg.frame.data, 0);
    uint32_t current =  get32from8(msg.frame.data, 4);
    axes[axisID]->update_ui(voltage, current, msg.timestamp);
}

void parse_adc(int axisID, canMsg msg){
    
    uint32_t voltage =  get32from8(msg.frame.data, 0);
    axes[axisID]->update_adc(voltage, msg.timestamp);
}

void parse_controller_error(int axisID, canMsg msg){
    uint32_t err =  get32from8(msg.frame.data, 0);
    axes[axisID]->update_controller_err(err, msg.timestamp);
    
}

uint32_t get32from8( uint8_t * data, int startIdx){
    return  data[startIdx] | data[startIdx + 1] << 8 |data[startIdx + 2] << 16 |data[startIdx + 3] << 24;
}

};
