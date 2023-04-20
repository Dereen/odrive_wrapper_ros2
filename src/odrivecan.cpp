/**
 * @file OdriveCan.c
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Class for Odrive S1 (PRO) CAN messages ID
 * @version 0.1
 * @date 2023-03-03
 *
 * @copyright Copyright (c) 2023
 * see https://docs.odriverobotics.com/v/latest/can-protocol.html#transport-protocol
 */

#include <iostream>
#include <vector>
#include <memory>

#include "candevice.h"
#include "odriveaxis.h"
#include "odrivecan.h"
#include <boost/circular_buffer.hpp>

#include <thread>
#include <mutex>
#include <unordered_map>

#include "odrivecan.h"

int OdriveCan::get_axes_num()
{
    return this->axes_num;
}

void OdriveCan::init()
{
    // create shared pointers for each Odrive axis and init the instances of class Axis
    for (int i = 0; i < axes_num; i++)
    {
        axes.push_back(std::make_shared<class OdriveAxis>(i));
        axes_ids[i] = i;

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

void OdriveCan::set_periods(int32_t heartbeat, int32_t busUI)
{
    periods->heartbeat = heartbeat;
    periods->busIU = busUI;
}

void OdriveCan::parse_header(uint32_t header, int *axisID, int *cmdID)
{

    bool error_msg = header & ((1u << 30) - 1);
    if (!error_msg)
    {
        // parse axis ID, mesgs of axis 0 start at 0x00, axis 1 - 0x20, axis 2- 0x40 .. axis 6- 0xc0
        *axisID = axis_from_header(header);

        *cmdID = can_id_from_header(header);
    }
}

int OdriveCan::can_id_from_header(uint32_t header)
{
    return (header & 0x1F);
}

int OdriveCan::axis_from_header(uint32_t header)
{
    uint32_t id = header & ((1u << 29) - 1); // lower 29 bits as per struct can_frame
    return id >> 5;
}

bool OdriveCan::check_msg_error(uint32_t header)
{
    return CAN_ERR_FLAG & header;
}

void OdriveCan::receive_msgs()
{

    std::cout << "[RECEIVE] Start recieve thread" << std::endl;
    int ax_id;
    struct timeval timestamp;
    canMsg msg;

    struct can_frame frame;

    while (run)
    {
        // read meessages
        can_dev->recieve(&frame, &timestamp, 1);
        ax_id = axis_from_header(frame.can_id);

        //  store message with timestamp
        msg = {frame, timestamp};
        msg_mutex.lock();
        input_buffer[axes_ids[ax_id]]->push_back(msg);
        msg_mutex.unlock();

        if (input_buffer[axes_ids[ax_id]]->full())
            throw std::runtime_error("Circular buffer is full");
    }

    std::cout << "[RECEIVE] End recieve thread" << std::endl;
};

void OdriveCan::process_msgs()
{
    std::cout << "[PROCESS] start processing thread" << std::endl;
    canMsg msg;
    while (run)
    {
        for (auto &it : this->axes_ids) // iterate over list of axis IDs and its corresponding buffers
        {                               // check buffer for each axis

            if (!input_buffer[it.second]->empty())
            { // if buffer for given axis is not empty
                msg_mutex.lock();
                msg = input_buffer[it.second]->front();
                input_buffer[it.second]->pop_front();
                msg_mutex.unlock();

                int canID = can_id_from_header(msg.frame.can_id);

                switch (canID)
                {
                case GET_VERSION:
                    if (check_msg_error(msg.frame.can_id))
                        std::cout << "[ERROR] got version response" << std::endl;
                    else
                        std::cout << "recieved can header in get version " << msg.frame.can_id << std::endl;
                    parse_version(it.first, msg);
                    std::cout << "got version response" << std::endl;
                    break;
                case HEARTBEAT:
                    parse_heartbeat(it.first, msg);
                    break;
                case ESTOP:
                    break;
                case GET_ERROR:
                    parse_error(it.first, msg);
                    break;
                case GET_ENCODER_ESTIMATES:
                    parse_encoder_estimates(it.first, msg);
                    break;
                case GET_IQ:
                    parse_iq(it.first, msg);
                    break;
                case GET_TEMP:
                    parse_temp(it.first, msg);
                    break;
                case GET_BUS_VOLTAGE_CURRENT:
                    parse_ui(it.first, msg);
                    break;
                case GET_ADC_VOLTAGE:
                    parse_adc(it.first, msg);
                    break;
                case GET_CONTROLLER_ERROR:
                    parse_controller_error(it.first, msg);
                    break;

                case SET_AXIS_NODE_ID:
                    throw std::runtime_error("Got unexpected response from set SET_AXIS_NODE_ID");
                    break;
                case SET_AXIS_STATE:
                    throw std::runtime_error("Got unexpected response from set SET_AXIS_STATE");
                    break;
                case SET_CONTROLLER_MODE:
                    throw std::runtime_error("Got unexpected response from set SET_CONTROLLER_MODE");
                    break;
                case SET_INPUT_POS:
                    throw std::runtime_error("Got unexpected response from set SET_INPUT_POS");
                    break;
                case SET_INPUT_VEL:
                    throw std::runtime_error("Got unexpected response from set SET_INPUT_VEL");
                    break;
                case SET_INPUT_TORQUE:
                    throw std::runtime_error("Got unexpected response from set SET_INPUT_TORQUE");
                    break;
                case SET_LIMITS:
                    throw std::runtime_error("Got unexpected response from set SET_LIMITS");
                    break;
                case START_ANTICOGGING:
                    throw std::runtime_error("Got unexpected response from set START_ANTICOGGING");
                    break;
                case SET_TRAJ_VEL_LIMIT:
                    throw std::runtime_error("Got unexpected response from set SET_TRAJ_VEL_LIMIT");
                    break;
                case SET_TRAJ_ACCEL_LIMITS:
                    throw std::runtime_error("Got unexpected response from set SET_TRAJ_ACCEL_LIMITS");
                    break;
                case SET_TRAJ_INERTIA:
                    throw std::runtime_error("Got unexpected response from set SET_TRAJ_INERTIA");
                    break;
                case REBOOT:
                    throw std::runtime_error("Got unexpected response from set REBOOT");
                    break;
                case CLEAR_ERRORS:
                    throw std::runtime_error("Got unexpected response from set CLEAR_ERRORS");
                    break;
                case SET_ABSOLUTE_POSITION:
                    throw std::runtime_error("Got unexpected response from set SET_ABSOLUTE_POSITION");
                    break;
                case SET_POS_GAIN:
                    throw std::runtime_error("Got unexpected response from set SET_POS_GAIN");
                    break;
                case SET_VEL_GAINS:
                    throw std::runtime_error("Got unexpected response from set SET_VEL_GAINS");
                    break;
                case ENTER_DFU_MODE:
                    throw std::runtime_error("Got unexpected response from set ENTER_DFU_MODE");
                    break;
                default:
                    throw std::runtime_error("Unknown can message ID");
                }
            }
        }
        usleep(100);
    }
}

void OdriveCan::parse_version(int axisID, canMsg msg)
{
    std::cout << "Recieved data " << msg.frame.data << std::endl;
    uint8_t hw_maj = uint8_t(msg.frame.data[1]);
    uint8_t hw_min = uint8_t(msg.frame.data[2]);
    uint8_t hw_var = uint8_t(msg.frame.data[3]);
    uint8_t fw_maj = uint8_t(msg.frame.data[4]);
    uint8_t fw_min = uint8_t(msg.frame.data[5]);
    uint8_t fw_rev = uint8_t(msg.frame.data[6]);

    axes[axisID]->set_axis_version(hw_maj, hw_min, hw_var, fw_maj, fw_min, fw_rev, msg.timestamp);
}

void OdriveCan::parse_heartbeat(int axisID, canMsg msg)
{
    uint32_t err = get32from8(msg.frame.data, 0);
    axes[axisID]->set_axis_state(err, msg.frame.data[4], msg.frame.data[5], msg.frame.data[6], msg.timestamp);
}

void OdriveCan::parse_error(int axisID, canMsg msg)
{
    uint32_t err = get32from8(msg.frame.data, 0);
    uint32_t reason = get32from8(msg.frame.data, 4);
    axes[axisID]->update_error(err, reason, msg.timestamp);
}

void OdriveCan::parse_encoder_estimates(int axisID, canMsg msg)
{
    uint32_t err = get32from8(msg.frame.data, 0);
    uint32_t reason = get32from8(msg.frame.data, 4);
    axes[axisID]->update_estimates(err, reason, msg.timestamp);
}

void OdriveCan::parse_iq(int axisID, canMsg msg)
{
    uint32_t setpoint = get32from8(msg.frame.data, 0);
    uint32_t measured = get32from8(msg.frame.data, 4);
    axes[axisID]->update_iq(setpoint, measured, msg.timestamp);
}

void OdriveCan::parse_temp(int axisID, canMsg msg)
{
    uint32_t fet = get32from8(msg.frame.data, 0);
    uint32_t motor = get32from8(msg.frame.data, 4);
    axes[axisID]->update_temp(fet, motor, msg.timestamp);
}

void OdriveCan::parse_ui(int axisID, canMsg msg)
{
    uint32_t voltage = get32from8(msg.frame.data, 0);
    uint32_t current = get32from8(msg.frame.data, 4);
    axes[axisID]->update_ui(voltage, current, msg.timestamp);
}

void OdriveCan::parse_adc(int axisID, canMsg msg)
{

    uint32_t voltage = get32from8(msg.frame.data, 0);
    axes[axisID]->update_adc(voltage, msg.timestamp);
}

void OdriveCan::parse_controller_error(int axisID, canMsg msg)
{
    uint32_t err = get32from8(msg.frame.data, 0);
    axes[axisID]->update_controller_err(err, msg.timestamp);
}

uint32_t OdriveCan::get32from8(uint8_t *data, int startIdx)
{
    return data[startIdx + 3] | data[startIdx + 2] << 8 | data[startIdx + 1] << 16 | data[startIdx] << 24;
}

void OdriveCan::get_char_from_uint(char *arr, uint32_t var)
{
    memcpy(arr, &var, sizeof(uint32_t));
}

void OdriveCan::get_char_from_uints(char *arr, uint32_t var1, uint32_t var2)
{
    this->get_char_from_uint(arr, var1)
    this->get_char_from_uint(arr+sizeof(uint32_t), var2);
}

void OdriveCan::get_char_from_uints(char *arr, uint32_t var1, float var2, float var3)
{
    this->get_char_from_uint(arr, &var1);
    char * ptr = arr+sizeof(uint32_t);
    memcpy(ptr, &var2, sizeof(float));
    ptr += sizeof(float);
    memcpy(ptr, &var3, sizeof(float));
}

std::ostream &operator<<(std::ostream &out, OdriveCan const *oc)
{
    for (auto &it : oc->axes_ids)
    {
        out << "\n~~~~Axis ID " << it.first << " ~~~~" << std::endl;
        out << oc->axes[it.second];
    }

    return out;
}

int OdriveCan::call_get_version(int axisID)
{
    int msg_id = axisID << 5 | GET_VERSION; // axis ID + can msg name
    std::cout << "[GetVersion] Ask for version - CAN msg ID " << std::hex << msg_id << std::dec << std::endl;

    int ret = can_dev->send(msg_id, canMsgLen[GET_VERSION], true);

    if (ret < 0)
    {
        throw std::runtime_error("The wrong number of bytes were written to CAN");
        return ret;
    }

    return 0;
}

int OdriveCan::call_estop(int axisID)
{
    int msg_id = axisID << 5 | ESTOP; // axis ID + can msg name
    std::cout << "[ESTOP] Call estop - CAN msg ID" << std::hex
              << msg_id << std::dec << std::endl;

    int ret = can_dev->send(msg_id, canMsgLen[ESTOP]);

    if (ret < 0)
    {
        throw std::runtime_error("The wrong number of bytes were written to CAN");
        return ret;
    }

    return 0;
}

int OdriveCan::call_get_error(int axisID)
{
    int msg_id = axisID << 5 | GET_ERROR; // axis ID + can msg name
    std::cout << "[GetError] Ask for get_error - CAN msg ID" << std::hex << msg_id << std::dec << std::endl;

    int ret = can_dev->send(msg_id, canMsgLen[GET_ERROR], true);

    if (ret < 0)
    {
        throw std::runtime_error("The wrong number of bytes were written to CAN");
        return ret;
    }

    return 0;
}

int OdriveCan::call_set_axis_node_id(int oldID, uint32_t newID)
{
    int msg_id = oldID << 5 | SET_AXIS_NODE_ID; // axis ID + can msg name
    std::cout << "[SetAxisNodeID] Ask for set_axis_node_id - CAN msg ID"
              << std::hex << msg_id << std::dec << std::endl;

    char data[4];
    get_char_from_uint(data, newID);

    int ret = can_dev->send(msg_id, canMsgLen[SET_AXIS_NODE_ID], data, true);

    if (ret < 0)
    {
        throw std::runtime_error("The wrong number of bytes were written to CAN");
        return ret;
    }

    // store corresponding index to old id
    int tmp = axes_ids[oldID];
    // delete old reference
    this->axes_ids.erase(oldID);

    // create new instance
    this->axes_ids[newID] = tmp;

    return 0;
}

int OdriveCan::call_set_axis_state(int axisID, uint32_t req_state)
{

    int msg_id = axisID << 5 | SET_AXIS_STATE; // axis ID + can msg name
    std::cout << "[SetAxisState]  Ask for set_axis_state - CAN msg ID"
              << std::hex << msg_id << std::dec << std::endl;

    char data[4];
    get_char_from_uint(data, req_state);

    int ret = can_dev->send(msg_id, canMsgLen[SET_AXIS_STATE], data, true);

    if (ret < 0)
    {
        throw std::runtime_error("The wrong number of bytes were written to CAN");
        return ret;
    }

    this->axes[axes_ids[axisID]]->update_axis_requested_state(req_state);
    return 0;
};

int OdriveCan::call_get_encoder_estimates(int axisID)
{
    int msg_id = (axisID * 0x20) + GET_ENCODER_ESTIMATES; // axis ID + can msg name
    std::cout << "[GetEncoderEstimates] Ask for encoder estimates -  CAN msg ID "
              << std::hex << msg_id << std::dec << std::endl;

    int ret = can_dev->send(msg_id, canMsgLen[GET_ENCODER_ESTIMATES], true);

    if (ret < 0)
    {
        throw std::runtime_error("The wrong number of bytes were written to CAN");
        return ret;
    }

    return 0;
};

int OdriveCan::call_set_encoder_mode(int axisID, uint32_t control_mode, uint32_t input_mode)
{
    // construct can message
    int msg_id = axisID << 5 | SET_CONTROLLER_MODE; // axis ID + can msg name
    std::cout << "[SetEncoderMode] Ask to set encoder mode - CAN msg ID"
              << std::hex << msg_id << std::dec << std::endl;

    char data[8];
    this->get_char_from_uints(data, control_mode, input_mode);

    int ret = can_dev->send(msg_id, canMsgLen[SET_CONTROLLER_MODE], data, true);

    if (ret < 0)
    {
        throw std::runtime_error("The wrong number of bytes were written to CAN");
        return ret;
    }

    this->axes[axes_ids[axisID]]->update_axis_requested_state(req_state);
    return 0;
};

int OdriveCan::call_set_input_pos(int axisID, uint32_t input_pos, float vel_ff, float torque_ff) { 
        // construct can message
    int msg_id = axisID << 5 | SET_INPUT_POS; // axis ID + can msg name
    std::cout << "[SetInputPos] Ask to set encoder mode - CAN msg ID"
              << std::hex << msg_id << std::dec << std::endl;

    char data[8];
    this->get_char_from_uints(data, input_pos, vel_ff, torque_ff);

    int ret = can_dev->send(msg_id, canMsgLen[SET_INPUT_POS], data, true);

    if (ret < 0)
    {
        throw std::runtime_error("The wrong number of bytes were written to CAN");
        return ret;
    }

    this->axes[axes_ids[axisID]]->update_axis_requested_state(req_state);
    return 0;
 };

int OdriveCan::call_set_input_vel(int axisID) { return 0; };

int OdriveCan::call_set_input_torque(int axisID) { return 0; };

int OdriveCan::call_set_limits(int axisID) { return 0; };

int OdriveCan::call_start_anticogging(int axisID) { return 0; };

int OdriveCan::call_set_teaj_vel_limit(int axisID) { return 0; };

int OdriveCan::call_set_traj_accel_limits(int axisID) { return 0; };

int OdriveCan::call_set_traj_inertia(int axisID) { return 0; };

int OdriveCan::call_get_iq(int axisID)
{

    std::cout << "[GetIQ] Ask for IQ " << std::endl;
    // construct can message
    int msg_id = (axisID * 0x20) + GET_IQ; // axis ID + can msg name
    std::cout << "[GetIQ] CAN msg ID " << std::hex << msg_id << std::dec << std::endl;

    int ret = can_dev->send(msg_id, canMsgLen[GET_IQ], true);

    if (ret < 0)
    {
        throw std::runtime_error("The wrong number of bytes were written to CAN");
        return ret;
    }

    return 0;
};

int OdriveCan::call_get_tempterature(int axisID)
{
    std::cout << "[GetTemp] Ask for temperature " << std::endl;
    // construct can message
    int msg_id = (axisID * 0x20) + GET_TEMP; // axis ID + can msg name
    std::cout << "[GetTemp] CAN msg ID " << std::hex << msg_id << std::dec << std::endl;

    int ret = can_dev->send(msg_id, canMsgLen[GET_TEMP], true);

    if (ret < 0)
    {
        throw std::runtime_error("The wrong number of bytes were written to CAN");
        return ret;
    }

    return 0;
};

int OdriveCan::call_reboot(int axisID) { return 0; };

int OdriveCan::call_get_bus_ui(int axisID)
{

    std::cout << "[GetBusUI] Ask for UI " << std::endl;
    // construct can message
    int msg_id = (axisID * 0x20) + GET_BUS_VOLTAGE_CURRENT; // axis ID + can msg name
    std::cout << "[GetBusUI] CAN msg ID " << std::hex << msg_id << std::dec << std::endl;

    int ret = can_dev->send(msg_id, canMsgLen[GET_BUS_VOLTAGE_CURRENT], true);

    if (ret < 0)
    {
        throw std::runtime_error("The wrong number of bytes were written to CAN");
        return ret;
    }

    return 0;
};

int OdriveCan::call_clear_errors(int axisID) { return 0; };

int OdriveCan::call_set_absolute_position(int axisID) { return 0; };

int OdriveCan::call_set_pos_gain(int axisID) { return 0; };

int OdriveCan::call_set_vel_gains(int axisID) { return 0; };

int OdriveCan::call_get_adc_voltage(int axisID)
{

    std::cout << "[GetADC] Ask for ADC voltage " << std::endl;
    // construct can message
    int msg_id = (axisID * 0x20) + GET_ADC_VOLTAGE; // axis ID + can msg name
    std::cout << "[GetADC] CAN msg ID " << std::hex << msg_id << std::dec << std::endl;

    int ret = can_dev->send(msg_id, canMsgLen[GET_ADC_VOLTAGE], true);

    if (ret < 0)
    {
        throw std::runtime_error("The wrong number of bytes were written to CAN");
        return ret;
    }

    return 0;
};

int OdriveCan::call_get_controller_Error(int axisID)
{
    std::cout << "[GetControllerError] Ask if any errors are in controller " << std::endl;
    // construct can message
    int msg_id = (axisID * 0x20) + GET_CONTROLLER_ERROR; // axis ID + can msg name
    std::cout << "[GetControllerError] CAN msg ID " << std::hex << msg_id << std::dec << std::endl;

    int ret = can_dev->send(msg_id, canMsgLen[GET_CONTROLLER_ERROR], true);

    if (ret < 0)
    {
        throw std::runtime_error("The wrong number of bytes were written to CAN");
        return ret;
    }

    return 0;
};

int OdriveCan::call_enter_dfu_mode(int axisID) { return 0; };
