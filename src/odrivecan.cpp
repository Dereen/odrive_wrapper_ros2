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
#include <sys/time.h>

#include "odrivecan.h"
#include "helpers.h"

int OdriveCan::get_axes_num()
{
    return this->axes_num;
}

void OdriveCan::init()
{
    canMsgLen = {{GET_VERSION, 8},
                 {HEARTBEAT, 7},
                 {ESTOP, 0},
                 {GET_ERROR, 8},
                 {SET_AXIS_NODE_ID, 4},
                 {SET_AXIS_STATE, 4},
                 {GET_ENCODER_ESTIMATES, 8},
                 {SET_CONTROLLER_MODE, 8},
                 {SET_INPUT_POS, 8},
                 {SET_INPUT_VEL, 8},
                 {SET_INPUT_TORQUE, 4},
                 {SET_LIMITS, 8},
                 {START_ANTICOGGING, 0},
                 {SET_TRAJ_VEL_LIMIT, 4},
                 {SET_TRAJ_ACCEL_LIMITS, 8},
                 {SET_TRAJ_INERTIA, 4},
                 {GET_IQ, 8},
                 {GET_TEMP, 8},
                 {REBOOT, 0},
                 {GET_BUS_VOLTAGE_CURRENT, 8},
                 {CLEAR_ERRORS, 0},
                 {SET_ABSOLUTE_POSITION, 4},
                 {SET_POS_GAIN, 4},
                 {SET_VEL_GAINS, 8},
                 {GET_ADC_VOLTAGE, 4},
                 {GET_CONTROLLER_ERROR, 4},
                 {ENTER_DFU_MODE, 0}};

    // create unique pointers for each Odrive axis and init the instances of class Axis
    for (int i = 0; i < axes_num; i++)
    {
        axes.push_back(std::make_shared<class OdriveAxis>(i));
        axes_ids[i] = i;

        // init circular buffer
        input_buffer.push_back(std::make_unique<can_circ_buffer>(buffer_len));
    }

    // init update periods
    periods = std::make_unique<struct updatePeriods>();
    set_periods(100, 100);

    // init can listener
    can_dev = std::make_unique<class CanDevice>();
    can_dev->init_connection();

    std::cout << "[OdriveCan] start threads" << std::endl;
    th_recieve = std::thread(&OdriveCan::receive_msgs, this);
    th_process = std::thread(&OdriveCan::process_msgs, this);
    th_send = std::thread(&OdriveCan::ask_for_current_values, this);
    th_errors = std::thread(&OdriveCan::get_errors, this);
}

void OdriveCan::set_periods(int32_t status_time, int32_t data_time)
{
    periods->axis_status = status_time;
    periods->data = data_time;
}
void OdriveCan::set_update_period(time_t new_period)
{
    this->periods->data = new_period;
}

void OdriveCan::set_status_update_period(time_t new_period)
{
    this->periods->axis_status = new_period;
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
    else
        perror("Got CAN mesage with error flag");
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


/**
 * @brief Sends CAN requests for temperature readings, encoder estimates, motor current, and ADC voltage
 *
 */
void OdriveCan::ask_for_current_values()
{
    std::cout << "[SEND] Start send thread" << std::endl;
    int id;

    while (run)
    {

        for (auto &it : this->axes)
        {
            id = it->get_axis_id();
            // std::cout << "ask for temp" << std::endl;
            call_get_tempterature(id);
            /// std::cout << "ask for bus ui" << std::endl;
            call_get_bus_ui(id);
            // std::cout << "ask for encoder estimates" << std::endl;
            call_get_encoder_estimates(id);
            // std::cout << "ask for iq" << std::endl;
            call_get_iq(id);
            //  std::cout << "ask for adc voltage" << std::endl;
            call_get_adc_voltage(id);
        }
        usleep(periods->data*1000);
    }
}

void OdriveCan::get_errors()
{
    std::cout << "[INFO] Start error catching thread" << std::endl;
    int id;

    while (run)
    {

        for (auto &it : this->axes)
        {
            id = it->get_axis_id();
            call_get_error(id);
            call_get_controller_error(id);
            
        }
        usleep(periods->axis_status*1000);
    }
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
        if (!key_present(axes_ids, ax_id)) // key is not present in unordered map
        {
            std::cout << "[ERROR] recieved message from unexpected axis ID " << get_axis_id() << std::endl;
            continue;
        }
        else
        {
            buffer_mutex.lock();
            input_buffer[axes_ids[ax_id]]->push_back(msg);
            buffer_mutex.unlock();
        }

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
                buffer_mutex.lock();
                msg = input_buffer[it.second]->front();
                input_buffer[it.second]->pop_front();
                buffer_mutex.unlock();

                int canID = can_id_from_header(msg.frame.can_id);
                if (check_msg_error(msg.frame.can_id))
                { // Odrive does not use ERR bit
                    std::cout << "[ERROR] got message response with ERR flag " << msg.frame.can_id << std::endl;
                }

                switch (canID)
                {
                case GET_VERSION:
                    parse_version(it.first, msg);
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
                std::cout << "got response to controller error" << std::endl;
                    parse_controller_error(it.first, msg);
                    break;

                case SET_AXIS_NODE_ID:
                    perror("Got unexpected response from set SET_AXIS_NODE_ID");
                    break;
                case SET_AXIS_STATE:
                    perror("Got unexpected response from set SET_AXIS_STATE");
                    break;
                case SET_CONTROLLER_MODE:
                    perror("Got unexpected response from set SET_CONTROLLER_MODE");
                    break;
                case SET_INPUT_POS:
                    perror("Got unexpected response from set SET_INPUT_POS");
                    break;
                case SET_INPUT_VEL:
                    perror("Got unexpected response from set SET_INPUT_VEL");
                    break;
                case SET_INPUT_TORQUE:
                    perror("Got unexpected response from set SET_INPUT_TORQUE");
                    break;
                case SET_LIMITS:
                    perror("Got unexpected response from set SET_LIMITS");
                    break;
                case START_ANTICOGGING:
                    perror("Got unexpected response from set START_ANTICOGGING");
                    break;
                case SET_TRAJ_VEL_LIMIT:
                    perror("Got unexpected response from set SET_TRAJ_VEL_LIMIT");
                    break;
                case SET_TRAJ_ACCEL_LIMITS:
                    perror("Got unexpected response from set SET_TRAJ_ACCEL_LIMITS");
                    break;
                case SET_TRAJ_INERTIA:
                    perror("Got unexpected response from set SET_TRAJ_INERTIA");
                    break;
                case REBOOT:
                    perror("Got unexpected response from set REBOOT");
                    break;
                case CLEAR_ERRORS:
                    perror("Got unexpected response from set CLEAR_ERRORS");
                    break;
                case SET_ABSOLUTE_POSITION:
                    perror("Got unexpected response from set SET_ABSOLUTE_POSITION");
                    break;
                case SET_POS_GAIN:
                    perror("Got unexpected response from set SET_POS_GAIN");
                    break;
                case SET_VEL_GAINS:
                    perror("Got unexpected response from set SET_VEL_GAINS");
                    break;
                case ENTER_DFU_MODE:
                    perror("Got unexpected response from set ENTER_DFU_MODE");
                    break;
                default:
                    perror("Unknown can message ID");
                    break;
                }
            }
        }
        usleep(100);
    }
}

void OdriveCan::parse_version(int axisID, canMsg msg)
{
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
    uint32_t err = get32from8(msg.frame.data, 0, lsb);
    axes[axisID]->set_axis_state(err, msg.frame.data[4], msg.frame.data[5], msg.frame.data[6], msg.timestamp);
}

void OdriveCan::parse_error(int axisID, canMsg msg)
{
    uint32_t err = get32from8(msg.frame.data, 0, lsb);
    uint32_t reason = get32from8(msg.frame.data, 4, lsb);
    axes[axisID]->update_error(err, reason, msg.timestamp);
}

void OdriveCan::parse_encoder_estimates(int axisID, canMsg msg)
{
    float pos = get_float(get32from8(msg.frame.data, 0, lsb));
    float vel = get_float(get32from8(msg.frame.data, 4, lsb));
    axes[axisID]->update_estimates(pos, vel, msg.timestamp);
}

void OdriveCan::parse_iq(int axisID, canMsg msg)
{
    float setpoint = get_float(get32from8(msg.frame.data, 0, lsb));
    float measured = get_float(get32from8(msg.frame.data, 4, lsb));
    axes[axisID]->update_iq(setpoint, measured, msg.timestamp);
}

void OdriveCan::parse_temp(int axisID, canMsg msg)
{
    // for (int i = 0; i < 8; i++)
    //     std::cout << unsigned(msg.frame.data[i]) << " ";
    // std::cout << std::endl;
    float fet = get_float(get32from8(msg.frame.data, 0, lsb));
    float motor = get_float(get32from8(msg.frame.data, 4, lsb));
    axes[axisID]->update_temp(fet, motor, msg.timestamp);
}

void OdriveCan::parse_ui(int axisID, canMsg msg)
{
    float voltage = get_float(get32from8(msg.frame.data, 0, lsb));
    float current = get_float(get32from8(msg.frame.data, 4, lsb));
    axes[axisID]->update_ui(voltage, current, msg.timestamp);
}

void OdriveCan::parse_adc(int axisID, canMsg msg)
{

    float voltage = get_float(get32from8(msg.frame.data, 0, lsb));
    axes[axisID]->update_adc(voltage, msg.timestamp);
}

void OdriveCan::parse_controller_error(int axisID, canMsg msg)
{
    uint32_t err = get32from8(msg.frame.data, 0, lsb);
    axes[axisID]->update_controller_err(err, msg.timestamp);
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
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | GET_VERSION; // axis ID + can msg name
#ifndef DEBUG
        std::cout << "[GetVersion] Ask for version - CAN msg ID " << std::hex << msg_id << std::dec << std::endl;
#endif
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[GET_VERSION], true);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR]  on axis" << axisID << " - GET_VERSION. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        return 0;
    }
    else
        return 1;
}

int OdriveCan::call_estop(int axisID)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | ESTOP; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[ESTOP] Call estop - CAN msg ID" << std::hex
                  << msg_id << std::dec << std::endl;
#endif
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[ESTOP]);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - ESTOP. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        this->axes[axes_ids[axisID]]->update_estop(tv);

        return 0;
    }
    else
        return 1;
}

int OdriveCan::call_get_error(int axisID)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | GET_ERROR; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[GetError] Ask for get_error - CAN msg ID" << std::hex << msg_id << std::dec << std::endl;
#endif
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[GET_ERROR], true);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - GET_ERROR. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        return 0;
    }
    else
        return 1;
}

int OdriveCan::call_set_axis_node_id(int oldID, uint32_t newID)
{
    if (key_present(axes_ids, oldID))
    {
        int msg_id = oldID << 5 | SET_AXIS_NODE_ID; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[SetAxisNodeID] Ask for set_axis_node_id - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[4];
        get_char_from_num<uint32_t>(data, newID, lsb);
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[SET_AXIS_NODE_ID], data);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << oldID << " - SET_AXIS_NODE_ID. The wrong number of bytes were written to CAN" << std::endl;
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
    else
        return 1;
}

int OdriveCan::call_set_axis_state(int axisID, uint32_t req_state)
{
    if (key_present(axes_ids, axisID))
    {

        int msg_id = axisID << 5 | SET_AXIS_STATE; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[SetAxisState]  Ask for set_axis_state - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[4];
        get_char_from_num<uint32_t>(data, req_state, lsb);
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[SET_AXIS_STATE], data);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - SET_AXIS_STATE. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        this->axes[axes_ids[axisID]]->update_axis_requested_state(req_state);
        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_get_encoder_estimates(int axisID)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = (axisID * 0x20) + GET_ENCODER_ESTIMATES; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[GetEncoderEstimates] Ask for encoder estimates -  CAN msg ID "
                  << std::hex << msg_id << std::dec << std::endl;
#endif
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[GET_ENCODER_ESTIMATES], true);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - GET_ENCODER_ESTIMATES. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_set_controller_mode(int axisID, uint32_t control_mode, uint32_t input_mode)
{
    if (key_present(axes_ids, axisID))
    {
        // construct can message
        int msg_id = axisID << 5 | SET_CONTROLLER_MODE; // axis ID + can msg name
#ifndef DEBUG
        std::cout << "[SetControlerMode] Ask to set controller mode - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[8];
        get_char_from_nums<uint32_t>(data, control_mode, input_mode, lsb);
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[SET_CONTROLLER_MODE], data);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - SET_CONTROLLER_MODE. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        this->axes[axes_ids[axisID]]->update_controller_mode(control_mode, input_mode);

        struct timeval tv;
        gettimeofday(&tv, NULL);
        this->axes[axes_ids[axisID]]->update_controller_timestamp(tv);
        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_set_input_pos(int axisID, float input_pos, float vel_ff, float torque_ff)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | SET_INPUT_POS; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[SetInputPos] Ask to set input position - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[8];
        get_char_from_nums<float>(data, input_pos, vel_ff, torque_ff, lsb);
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[SET_INPUT_POS], data);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - SET_INPUT_POS. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        this->axes[axes_ids[axisID]]->update_input_pos(input_pos, vel_ff, torque_ff);
        struct timeval tv;
        gettimeofday(&tv, NULL);
        this->axes[axes_ids[axisID]]->update_controller_timestamp(tv);
        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_set_input_vel(int axisID, float input_vel, float input_torque_ff)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | SET_INPUT_VEL; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[SetInputVel] Ask to set input velocity - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[8];
        get_char_from_nums<float>(data, input_vel, input_torque_ff, lsb);
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[SET_INPUT_VEL], data);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - SET_INPUT_VEL. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        };
        this->axes[axes_ids[axisID]]->update_input_vel(input_vel, input_torque_ff);
        struct timeval tv;
        gettimeofday(&tv, NULL);
        this->axes[axes_ids[axisID]]->update_controller_timestamp(tv);
        return 0;
    }
    else
        return 1;
}

int OdriveCan::call_set_input_torque(int axisID, float torque)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | SET_INPUT_TORQUE; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[SetInputTorque] Ask to set input torque - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[4];
        get_char_from_num<float>(data, torque, lsb);
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[SET_INPUT_TORQUE], data);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - SET_INPUT_TORQUE. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        };

        this->axes[axes_ids[axisID]]->update_input_torque(torque);
        struct timeval tv;
        gettimeofday(&tv, NULL);
        this->axes[axes_ids[axisID]]->update_controller_timestamp(tv);

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_set_limits(int axisID, float velocity, float current)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | SET_LIMITS; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[SetLimits] Ask to set velocity and current limits - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[8];
        get_char_from_nums<float>(data, velocity, current, lsb);
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[SET_LIMITS], data);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - SET_LIMITS. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        };

        this->axes[axes_ids[axisID]]->update_limits(velocity, current);
        struct timeval tv;
        gettimeofday(&tv, NULL);
        this->axes[axes_ids[axisID]]->update_controller_timestamp(tv);

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_start_anticogging(int axisID)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | START_ANTICOGGING; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[Anticogging] Ask to start anticogging" << std::hex
                  << msg_id << std::dec << std::endl;
#endif
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[START_ANTICOGGING]);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - START_ANTICOGGING. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        this->axes[axes_ids[axisID]]->update_anticogging(tv);
        this->axes[axes_ids[axisID]]->update_controller_timestamp(tv);

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_set_traj_vel_limit(int axisID, float lim)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | SET_TRAJ_VEL_LIMIT; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[SetLimits] Ask to set velocity and current limits - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[4];
        get_char_from_num<float>(data, lim, lsb);
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[SET_TRAJ_VEL_LIMIT], data);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - SET_TRAJ_VEL_LIMIT. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        };

        this->axes[axes_ids[axisID]]->update_traj_vel_limit(lim);
        struct timeval tv;
        gettimeofday(&tv, NULL);
        this->axes[axes_ids[axisID]]->update_controller_timestamp(tv);

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_set_traj_accel_limits(int axisID, float accel, float decel)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | SET_TRAJ_ACCEL_LIMITS; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[SetAccelLimits] Ask to set trajectory acceleration and decellerations limits - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[8];
        get_char_from_nums<float>(data, accel, decel, lsb);
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[SET_TRAJ_ACCEL_LIMITS], data);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - SET_TRAJ_ACCEL_LIMITS. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        };

        this->axes[axes_ids[axisID]]->update_traj_accel_limit(accel, decel);
        struct timeval tv;
        gettimeofday(&tv, NULL);
        this->axes[axes_ids[axisID]]->update_controller_timestamp(tv);

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_set_traj_inertia(int axisID, float inertia)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | SET_TRAJ_INERTIA; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[SetAccelLimits] Ask to set trajectory acceleration and decellerations limits - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[4];
        get_char_from_num<float>(data, inertia, lsb);
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[SET_TRAJ_INERTIA], data);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - SET_TRAJ_INERTIA. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        };

        this->axes[axes_ids[axisID]]->update_traj_inertia(inertia);
        struct timeval tv;
        gettimeofday(&tv, NULL);
        this->axes[axes_ids[axisID]]->update_controller_timestamp(tv);

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_get_iq(int axisID)
{
    if (key_present(axes_ids, axisID))
    {
        // construct can message
        int msg_id = (axisID * 0x20) + GET_IQ; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[GetIQ] Ask for IQ  CAN msg ID " << std::hex << msg_id << std::dec << std::endl;
#endif
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[GET_IQ], true);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - GET_IQ. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_get_tempterature(int axisID)
{
    if (key_present(axes_ids, axisID))
    {
        // construct can message
        int msg_id = (axisID * 0x20) + GET_TEMP; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[GetTemp] Ask for temperature CAN msg ID " << std::hex << msg_id << std::dec << std::endl;
#endif
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[GET_TEMP], true);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - GET_TEMP. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_reboot(int axisID)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | REBOOT; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[Reboot] Ask to reboot " << std::hex
                  << msg_id << std::dec << std::endl;
#endif
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[REBOOT]);
        send_mutex.unlock();
        if (ret < 0)
        {
            std::cerr << "[ERROR] on axis" << axisID << " - REBOOT. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        this->axes[axes_ids[axisID]]->update_reboot_timestamp(tv);

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_get_bus_ui(int axisID)
{
    if (key_present(axes_ids, axisID))
    {
        // construct can message
        int msg_id = (axisID * 0x20) + GET_BUS_VOLTAGE_CURRENT; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[GetBusUI] Ask for UI  CAN msg ID " << std::hex << msg_id << std::dec << std::endl;
#endif
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[GET_BUS_VOLTAGE_CURRENT], true);
        send_mutex.unlock();
        if (ret < 0)
        {

            std::cerr << "[ERROR] on axis" << axisID << " - GET_BUS_VOLTAGE_CURRENT. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_clear_errors(int axisID)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | CLEAR_ERRORS; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[Reboot] Ask to clear errors" << std::hex
                  << msg_id << std::dec << std::endl;
#endif
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[CLEAR_ERRORS]);
        send_mutex.unlock();
        if (ret < 0)
        {

            std::cerr << "[ERROR] on axis" << axisID << " - CLEAR_ERRORS. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_set_absolute_position(int axisID, float pos)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | SET_ABSOLUTE_POSITION; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[SetAbsolutePos] Ask to set absolute position to " << (unsigned)pos << " - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[4];
        get_char_from_num<float>(data, pos, lsb);
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[SET_ABSOLUTE_POSITION], data);
        send_mutex.unlock();
        if (ret < 0)
        {

            std::cerr << "[ERROR] on axis" << axisID << " - SET_ABSOLUTE_POSITION. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        };

        this->axes[axes_ids[axisID]]->update_absolut_pos(pos);
        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_set_pos_gain(int axisID, float gain)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | SET_POS_GAIN; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[SetPositionGain] Ask to set position gain " << (unsigned)gain << " - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[4];
        get_char_from_num<float>(data, gain, lsb);
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[SET_POS_GAIN], data);
        send_mutex.unlock();
        if (ret < 0)
        {

            std::cerr << "[ERROR] on axis" << axisID << " - SET_POS_GAIN. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        };

        this->axes[axes_ids[axisID]]->update_pos_gain(gain);

        return 0;
    }
    else

        return 1;
};

int OdriveCan::call_set_vel_gains(int axisID, float gain, float integrator)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | SET_VEL_GAINS; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[SetVelocityGain] Ask to set velocity gain " << (unsigned)gain << ", integrator gain "
                  << unsigned(integrator) << " - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[8];
        get_char_from_nums<float>(data, gain, integrator, lsb);
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[SET_VEL_GAINS], data);
        send_mutex.unlock();
        if (ret < 0)
        {

            std::cerr << "[ERROR] on axis" << axisID << " - SET_VEL_GAINS. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        };

        this->axes[axes_ids[axisID]]->update_vel_gains(gain, integrator);

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_get_adc_voltage(int axisID)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = (axisID * 0x20) + GET_ADC_VOLTAGE; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[GetADC] Ask for ADC voltage - CAN msg ID " << std::hex << msg_id << std::dec << std::endl;
#endif
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[GET_ADC_VOLTAGE], true);
        send_mutex.unlock();
        if (ret < 0)
        {

            std::cerr << "[ERROR] on axis" << axisID << " - GET_ADC_VOLTAGE. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_get_controller_error(int axisID)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = (axisID * 0x20) + GET_CONTROLLER_ERROR; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[GetControllerError] Ask if any errors are in controller - CAN msg ID "
                  << std::hex << msg_id << std::dec << std::endl;
#endif
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[GET_CONTROLLER_ERROR], true);
        send_mutex.unlock();
        if (ret < 0)
        {

            std::cerr << "[ERROR] on axis" << axisID << " - GET_CONTROLLER_ERROR. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        return 0;
    }
    else
        return 1;
};

int OdriveCan::call_enter_dfu_mode(int axisID)
{
    if (key_present(axes_ids, axisID))
    {
        int msg_id = axisID << 5 | ENTER_DFU_MODE; // axis ID + can msg name
#ifdef DEBUG
        std::cout << "[DFU] Ask to enter DFU mode" << std::hex
                  << msg_id << std::dec << std::endl;
#endif
        send_mutex.lock();
        int ret = can_dev->send(msg_id, canMsgLen[ENTER_DFU_MODE]);
        send_mutex.unlock();
        if (ret < 0)
        {

            std::cerr << "[ERROR] on axis" << axisID << " - ENTER_DFU_MODE. The wrong number of bytes were written to CAN" << std::endl;
            return ret;
        }

        return 0;
    }
    else
        return 1;
};
