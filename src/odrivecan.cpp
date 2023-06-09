/**
 * @file odrivecan.cpp
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Class for Odrive S1 (PRO) CAN messages ID
 * @version 0.1
 * @date 2023-05-09
 *
 # @copyright (c) JettyVision s.r.o in Prague 2023 - All Rights Reserved
 * see https://docs.odriverobotics.com/v/latest/can-protocol.html#transport-protocol
 */

#include <iostream>
#include <vector>
#include <memory>

#include <boost/circular_buffer.hpp>

#include <thread>
#include <mutex>
#include <unordered_map>
#include <sys/time.h>

#include "candevice.h"
#include "odriveaxis.h"
#include "odrivecan.h"
#include "helpers.h"

void OdriveCan::init() {
    assert(axes_num != 0);
    assert(output_stream != nullptr);
    assert(error_stream != nullptr);

    can_msg_len = {{GET_VERSION,             8},
                 {HEARTBEAT,               7},
                 {ESTOP,                   0},
                 {GET_ERROR,               8},
                 {SET_AXIS_NODE_ID,        4},
                 {SET_AXIS_STATE,          4},
                 {GET_ENCODER_ESTIMATES,   8},
                 {SET_CONTROLLER_MODE,     8},
                 {SET_INPUT_POS,           8},
                 {SET_INPUT_VEL,           8},
                 {SET_INPUT_TORQUE,        4},
                 {SET_LIMITS,              8},
                 {START_ANTICOGGING,       0},
                 {SET_TRAJ_VEL_LIMIT,      4},
                 {SET_TRAJ_ACCEL_LIMITS,   8},
                 {SET_TRAJ_INERTIA,        4},
                 {GET_IQ,                  8},
                 {GET_TEMP,                8},
                 {REBOOT,                  0},
                 {GET_BUS_VOLTAGE_CURRENT, 8},
                 {CLEAR_ERRORS,            0},
                 {SET_ABSOLUTE_POSITION,   4},
                 {SET_POS_GAIN,            4},
                 {SET_VEL_GAINS,           8},
                 {GET_TORQUE,              8},
                 {GET_CONTROLLER_ERROR,    4},
                 {ENTER_DFU_MODE,          0}};

    // create unique pointers for each Odrive axis and init the instances of class Axis
    {
        std::lock_guard<std::mutex> guard(data_mutex);
        for (int i = 0; i < axes_num; i++) {
            axes_ids[i] = i;
            OdriveAxis tmp;
            axes.push_back(tmp);
            axes[i].id = i;
            //*output_stream << tmp << std::endl;

            // init circular buffer
            input_buffer.push_back(make_unique<can_circ_buffer>(buffer_len));
        }
    }

    // init can listener
    can_dev = make_unique<class CanDevice>(dev_name, output_stream, error_stream);
}

void OdriveCan::start(void) {

    can_dev->init_connection();

    // reset can timestamp
    struct timeval timestamp;
    gettimeofday(&timestamp, NULL);
    {
        std::lock_guard<std::mutex> guard(data_mutex);
        for (int i = 0; i < axes_num; i++) {
            axes[axes_ids[i]].can_active = timestamp;
        }
    }

    *output_stream << "[OdriveCan] start threads" << std::endl;
    th_recieve = std::thread(&OdriveCan::receive_msgs, this);
    th_process = std::thread(&OdriveCan::process_msgs, this);
    th_send = std::thread(&OdriveCan::ask_for_current_values, this);
    th_errors = std::thread(&OdriveCan::get_errors, this);
}

void OdriveCan::stop(void) {
    run = false;
    th_recieve.join();
    th_process.join();
    th_send.join();
    th_errors.join();
}

void OdriveCan::parse_header(uint32_t header, int *axisID, int *cmdID) {

    bool error_msg = header & ((1u << 30) - 1);
    if (!error_msg) {
        // parse axis ID, mesgs of axis 0 start at 0x00, axis 1 - 0x20, axis 2- 0x40 .. axis 6- 0xc0
        *axisID = axis_from_header(header);

        *cmdID = can_id_from_header(header);
    } else
        *error_stream << "Got CAN mesage with error flag";
}

int OdriveCan::can_id_from_header(uint32_t header) {
    return (header & 0x1F);
}

int OdriveCan::axis_from_header(uint32_t header) {
    uint32_t id = header & ((1u << 29) - 1); // lower 29 bits as per struct can_frame
    return id >> 5;
}

/**
 * @brief Sends CAN requests for temperature readings, encoder estimates, motor current, and ADC voltage
 *
 */
void OdriveCan::ask_for_current_values() {
    *output_stream << "[SEND] Start send thread" << std::endl;

    std::vector<int> ids;
    {
        std::lock_guard<std::mutex> guard(data_mutex);
        std::transform(this->axes.begin(), this->axes.end(),
                       std::back_inserter(ids), [](OdriveAxis axis) { return axis.id; });
    }

    while (run) {

        for (auto &id: ids) {
            // *output_stream << "ask for temp" << std::endl;
            call_get_tempterature(id);
            // *output_stream << "ask for bus ui" << std::endl;
            call_get_bus_ui(id);
            // *output_stream << "ask for encoder estimates" << std::endl;
            //call_get_encoder_estimates(id);
            call_get_torque(id);
            // *output_stream << "ask for iq" << std::endl;
            call_get_iq(id);
            //  *output_stream << "ask for adc voltage" << std::endl;
            //call_get_adc_voltage(id);
        }
        usleep(data_update_ms * 1000);
    }
}

void OdriveCan::get_errors() {
    *output_stream << "[INFO] Start error catching thread" << std::endl;

    std::vector<int> ids;

    {
        std::lock_guard<std::mutex> guard(data_mutex);
        std::transform(this->axes.begin(), this->axes.end(),
                       std::back_inserter(ids), [](OdriveAxis axis) { return axis.id; });
    }

    while (run) {
        for (auto &id : ids) {
            call_get_error(id);
            call_get_controller_error(id);
        }
        usleep(axis_status_update_ms * 1000);
    }
}

void OdriveCan::receive_msgs() {

    *output_stream << "[RECEIVE] Start recieve thread" << std::endl;
    int ax_id;
    struct timeval timestamp;
    canMsg msg;

    struct can_frame frame;

    while (run) {
        // read messages
        can_dev->recieve(&frame, &timestamp, 1);
        ax_id = axis_from_header(frame.can_id);

        //  store message with timestamp
        msg = {frame, timestamp};
        if (!key_present(axes_ids, ax_id)) // key is not present in unordered map
        {
            *output_stream << "[ERROR] recieved message from unexpected axis ID " << ax_id << std::endl;
            continue;
        } else {
            {
                std::lock_guard<std::mutex> guard(data_mutex);
                axes[axes_ids[ax_id]].can_active = timestamp;
            }
            if (!input_buffer[axes_ids[ax_id]]->full()) {
                std::lock_guard<std::mutex> guard(buffer_mutex);
                input_buffer[axes_ids[ax_id]]->push_back(msg);
            } else {
                *error_stream << "[ERROR] Circular buffer is full" << std::endl;
            }
        }
    }

    *output_stream << "[RECEIVE] End recieve thread" << std::endl;
};

void OdriveCan::process_msgs() {
    *output_stream << "[PROCESS] start processing thread" << std::endl;
    canMsg msg;
    while (run) {
        for (auto &it: this->axes_ids) // iterate over list of axis IDs and its corresponding buffers
        {                               // check buffer for each axis

            // if buffer for given axis is not empty
            if (!input_buffer[it.second]->empty()) {

                {
                    std::lock_guard<std::mutex> guard(buffer_mutex);
                    msg = input_buffer[it.second]->front();
                    input_buffer[it.second]->pop_front();
                }

                int canID = can_id_from_header(msg.frame.can_id);
                if (check_msg_error(msg.frame.can_id)) { // Odrive does not use ERR bit
                    *output_stream << "[ERROR] got message response with ERR flag " << msg.frame.can_id << std::endl;
                }

                switch (canID) {
                    case GET_VERSION:
                        parse_version(it.first, msg);
                        break;
                    case HEARTBEAT:
                        parse_heartbeat(it.first, msg);
                        break;
                    case ESTOP:
                        *output_stream << "[PROCESS] Estop request confirned" << std::endl;
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
                    case GET_TORQUE:
                        parse_torque(it.first, msg);
                        break;
                    case GET_CONTROLLER_ERROR:
                        *output_stream << "got response to controller error" << std::endl;
                        parse_controller_error(it.first, msg);
                        break;

                    case SET_AXIS_NODE_ID:
                        *error_stream << "Got unexpected response from set SET_AXIS_NODE_ID" << std::endl;
                        break;
                    case SET_AXIS_STATE:
                        *error_stream << "Got unexpected response from set SET_AXIS_STATE" << std::endl;
                        break;
                    case SET_CONTROLLER_MODE:
                        *error_stream << "Got unexpected response from set SET_CONTROLLER_MODE" << std::endl;
                        break;
                    case SET_INPUT_POS:
                        *error_stream << "Got unexpected response from set SET_INPUT_POS" << std::endl;
                        break;
                    case SET_INPUT_VEL:
                        *error_stream << msg.frame.data << std::endl;
                        break;
                    case SET_INPUT_TORQUE:
                        *error_stream << "Got unexpected response from set SET_INPUT_TORQUE" << std::endl;
                        break;
                    case SET_LIMITS:
                        *error_stream << "Got unexpected response from set SET_LIMITS" << std::endl;
                        break;
                    case START_ANTICOGGING:
                        *error_stream << "Got unexpected response from set START_ANTICOGGING" << std::endl;
                        break;
                    case SET_TRAJ_VEL_LIMIT:
                        *error_stream << "Got unexpected response from set SET_TRAJ_VEL_LIMIT" << std::endl;
                        break;
                    case SET_TRAJ_ACCEL_LIMITS:
                        *error_stream << "Got unexpected response from set SET_TRAJ_ACCEL_LIMITS" << std::endl;
                        break;
                    case SET_TRAJ_INERTIA:
                        *error_stream << "Got unexpected response from set SET_TRAJ_INERTIA" << std::endl;
                        break;
                    case REBOOT:
                        *error_stream << "Got unexpected response from set REBOOT" << std::endl;
                        break;
                    case CLEAR_ERRORS:
			*output_stream << "[PROCESS] Clear errors request confirned." << std::endl;
                        break;
                    case SET_ABSOLUTE_POSITION:
                        *error_stream << "Got unexpected response from set SET_ABSOLUTE_POSITION" << std::endl;
                        break;
                    case SET_POS_GAIN:
                        *error_stream << "Got unexpected response from set SET_POS_GAIN" << std::endl;
                        break;
                    case SET_VEL_GAINS:
                        *error_stream << "Got unexpected response from set SET_VEL_GAINS" << std::endl;
                        break;
                    case ENTER_DFU_MODE:
                        *error_stream << "Got unexpected response from set ENTER_DFU_MODE";
                        break;
                    default:
                        *error_stream << "Unknown can message ID" << std::endl;
                        break;
                }
            }
        }
        usleep(100);
    }
}

void OdriveCan::parse_version(int axisID, canMsg msg) {
    uint8_t hw_maj = uint8_t(msg.frame.data[1]);
    uint8_t hw_min = uint8_t(msg.frame.data[2]);
    uint8_t hw_var = uint8_t(msg.frame.data[3]);
    uint8_t fw_maj = uint8_t(msg.frame.data[4]);
    uint8_t fw_min = uint8_t(msg.frame.data[5]);
    uint8_t fw_rev = uint8_t(msg.frame.data[6]);

    {
        std::lock_guard<std::mutex> guard(data_mutex);
        axes[axisID].ver.hw_version_major = hw_maj;
        axes[axisID].ver.hw_version_minor = hw_min;
        axes[axisID].ver.hw_version_variant = hw_var;
        axes[axisID].ver.fw_version_major = fw_maj;
        axes[axisID].ver.fw_version_minor = fw_min;
        axes[axisID].ver.fw_version_revision = fw_rev;
        axes[axisID].ver.timestamp = msg.timestamp;
    }
}

void OdriveCan::parse_heartbeat(int axisID, canMsg msg) {
    uint32_t err = get32from8(msg.frame.data, 0, lsb);

    {
        std::lock_guard<std::mutex> guard(data_mutex);
        axes[axisID].err.axis_error = (AxisError) err;
        axes[axisID].err.timestamp = msg.timestamp;

        axes[axisID].state.axis_state = (AxisState)msg.frame.data[4];
        axes[axisID].state.procedure_result = (ProcedureResult)msg.frame.data[5];
        axes[axisID].state.trajectory_done_flag = msg.frame.data[6];
        axes[axisID].state.timestamp = msg.timestamp;
    }
}

void OdriveCan::parse_error(int axisID, canMsg msg) {
    uint32_t err = get32from8(msg.frame.data, 0, lsb);
    uint32_t reason = get32from8(msg.frame.data, 4, lsb);
    {
        std::lock_guard<std::mutex> guard(data_mutex);
        axes[axisID].err.active_errors = err;
        axes[axisID].err.disarm_reason = (DisarmReason) reason;
        axes[axisID].err.timestamp = msg.timestamp;
    }
}

void OdriveCan::parse_encoder_estimates(int axisID, canMsg msg) {
    float pos = get_float(get32from8(msg.frame.data, 0, lsb));
    float vel = get_float(get32from8(msg.frame.data, 4, lsb));
    {
        std::lock_guard<std::mutex> guard(data_mutex);
        axes[axisID].encoder.pos_estimate = pos;
        axes[axisID].encoder.vel_estimate = vel;
        axes[axisID].encoder.timestamp = msg.timestamp;
    }
}

void OdriveCan::parse_iq(int axisID, canMsg msg) {
    float setpoint = get_float(get32from8(msg.frame.data, 0, lsb));
    float measured = get_float(get32from8(msg.frame.data, 4, lsb));
    {
        std::lock_guard<std::mutex> guard(data_mutex);
        axes[axisID].iq.iq_measured = measured;
        axes[axisID].iq.iq_setpoint = setpoint;
        axes[axisID].iq.timestamp = msg.timestamp;
    }
}

void OdriveCan::parse_temp(int axisID, canMsg msg) {
    float fet = get_float(get32from8(msg.frame.data, 0, lsb));
    float motor = get_float(get32from8(msg.frame.data, 4, lsb));
    {
        std::lock_guard<std::mutex> guard(data_mutex);
        axes[axisID].temp.fet_temperature = fet;
        axes[axisID].temp.motor_temperature = motor;
        axes[axisID].temp.timestamp = msg.timestamp;
    }
}

void OdriveCan::parse_ui(int axisID, canMsg msg) {
    float voltage = get_float(get32from8(msg.frame.data, 0, lsb));
    float current = get_float(get32from8(msg.frame.data, 4, lsb));
    {
        std::lock_guard<std::mutex> guard(data_mutex);
        axes[axisID].ui.bus_voltage = voltage;
        axes[axisID].ui.bus_current = current;
        axes[axisID].ui.timestamp = msg.timestamp;
    }
}

void OdriveCan::parse_torque(int axisID, canMsg msg) {
    float setpoint = get_float(get32from8(msg.frame.data, 0, lsb));
    float estimate = get_float(get32from8(msg.frame.data, 4, lsb));
    {
        std::lock_guard<std::mutex> guard(data_mutex);
        axes[axisID].torque.torque_setpoint = setpoint;
        axes[axisID].torque.torque_estimate = estimate;
        axes[axisID].torque.timestamp = msg.timestamp;
    }
}

void OdriveCan::parse_controller_error(int axisID, canMsg msg) {
    uint32_t err = get32from8(msg.frame.data, 0, lsb);
    {
        std::lock_guard<std::mutex> guard(data_mutex);
        axes[axisID].err.controller_error = (ControllerError) err;
        axes[axisID].err.timestamp = msg.timestamp;
    }
}

std::ostream &operator<<(std::ostream &out, const OdriveCan &odrive) {
    for (auto &it: odrive.axes_ids) {
        out << "\n~~~~Axis ID " << it.first << " ~~~~" << std::endl;
        out << odrive[it.second];
    }

    return out;
}

int OdriveCan::call_get_version(int axisID) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | GET_VERSION; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[GetVersion] Ask for version - CAN msg ID " << std::hex << msg_id << std::dec << std::endl;
#endif
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[GET_VERSION], true);
        }
        if (ret < 0) {
            *error_stream << "[ERROR]  on axis" << axisID << " - GET_VERSION. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_estop(int axisID) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | ESTOP; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[ESTOP] Call estop - CAN msg ID" << std::hex
                  << msg_id << std::dec << std::endl;
#endif
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[ESTOP]);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - ESTOP. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        {
            std::lock_guard<std::mutex> guard(data_mutex);
            this->axes[axes_ids[axisID]].estop = tv;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_get_error(int axisID) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | GET_ERROR; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[GetError] Ask for get_error - CAN msg ID" << std::hex << msg_id << std::dec << std::endl;
#endif
        int ret = -1;
        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[GET_ERROR], true);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - GET_ERROR. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_set_axis_node_id(int oldID, uint32_t newID) {
    if (key_present(axes_ids, oldID)) {
        int msg_id = oldID << 5 | SET_AXIS_NODE_ID; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[SetAxisNodeID] Ask for set_axis_node_id - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[4];
        get_char_from_num<uint32_t>(data, newID, lsb);

        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[SET_AXIS_NODE_ID], data);
        }

        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << oldID << " - SET_AXIS_NODE_ID. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        // store corresponding index to old id
        int tmp = axes_ids[oldID];
        // delete old reference
        this->axes_ids.erase(oldID);

        // create new instance
        this->axes_ids[newID] = tmp;

        return 0;
    } else
        return 1;
}

int OdriveCan::call_set_axis_state(int axisID, AxisState req_state) {
    if (key_present(axes_ids, axisID)) {

        int msg_id = axisID << 5 | SET_AXIS_STATE; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[SetAxisState]  Ask for set_axis_state - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[4];
        get_char_from_num<uint32_t>(data, req_state, lsb);

        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[SET_AXIS_STATE], data);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - SET_AXIS_STATE. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        {
            std::lock_guard<std::mutex> guard(data_mutex);
            this->axes[axes_ids[axisID]].axis_requested_state = req_state;
        }
        return 0;
    } else
        return 1;
}

int OdriveCan::call_get_encoder_estimates(int axisID) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = (axisID * 0x20) + GET_ENCODER_ESTIMATES; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[GetEncoderEstimates] Ask for encoder estimates -  CAN msg ID "
                  << std::hex << msg_id << std::dec << std::endl;
#endif
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[GET_ENCODER_ESTIMATES], true);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - GET_ENCODER_ESTIMATES. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_set_controller_mode(int axisID, ControlMode control_mode, InputMode input_mode) {
    if (key_present(axes_ids, axisID)) {
        // construct can message
        int msg_id = axisID << 5 | SET_CONTROLLER_MODE; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[SetControlerMode] Ask to set controller mode - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[8];
        get_char_from_nums<uint32_t>(data, control_mode, input_mode, lsb);

        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[SET_CONTROLLER_MODE], data);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - SET_CONTROLLER_MODE. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        {
            std::lock_guard<std::mutex> guard(data_mutex);
            this->axes[axes_ids[axisID]].reg.control_mode = control_mode;
            this->axes[axes_ids[axisID]].reg.input_mode = input_mode;
            this->axes[axes_ids[axisID]].reg.timestamp = tv;
        }
        return 0;
    } else
        return 1;
}

int OdriveCan::call_set_input_pos(int axisID, float input_pos, float vel_ff, float torque_ff) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | SET_INPUT_POS; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[SetInputPos] Ask to set input position - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        if ((std::abs(vel_ff) > 65504)| (std::abs(torque_ff) > 65504)){
            std::cerr << "The absolute value of vel_ff or torque_ff was too big (should be less than 65 504). The Set_input_position message will not be called" << std::endl;
            return 1;
        }

        char data[8];
        get_char_from_nums<float, ushort>(data, input_pos, float_to_half(vel_ff), float_to_half(torque_ff), lsb);

        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);

            ret = can_dev->send(msg_id, can_msg_len[SET_INPUT_POS], data);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - SET_INPUT_POS. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        {
            this->axes[axes_ids[axisID]].reg.input_pos = input_pos;
            this->axes[axes_ids[axisID]].reg.vel_ff = half_to_float(float_to_half(vel_ff));
            this->axes[axes_ids[axisID]].reg.torque_ff = half_to_float(float_to_half(torque_ff));
            this->axes[axes_ids[axisID]].reg.timestamp = tv;
        }
        return 0;
    } else
        return 1;
}

int OdriveCan::call_set_input_vel(int axisID, float input_vel, float input_torque_ff) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | SET_INPUT_VEL; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[SetInputVel] Ask to set input velocity - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[8];
        get_char_from_nums<float>(data, input_vel, input_torque_ff, lsb);
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[SET_INPUT_VEL], data);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - SET_INPUT_VEL. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        {
            std::lock_guard<std::mutex> guard(data_mutex);
            this->axes[axes_ids[axisID]].reg.input_vel = input_vel;
            this->axes[axes_ids[axisID]].reg.input_torque_ff = input_torque_ff;
            this->axes[axes_ids[axisID]].reg.timestamp = tv;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_set_input_torque(int axisID, float torque) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | SET_INPUT_TORQUE; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[SetInputTorque] Ask to set input torque - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[4];
        get_char_from_num<float>(data, torque, lsb);
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[SET_INPUT_TORQUE], data);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - SET_INPUT_TORQUE. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        {
            std::lock_guard<std::mutex> guard(data_mutex);
            this->axes[axes_ids[axisID]].reg.input_torque = torque;
            this->axes[axes_ids[axisID]].reg.timestamp = tv;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_set_limits(int axisID, float velocity, float current) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | SET_LIMITS; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[SetLimits] Ask to set velocity and current limits - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[8];
        get_char_from_nums<float>(data, velocity, current, lsb);
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[SET_LIMITS], data);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - SET_LIMITS. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        {
            std::lock_guard<std::mutex> guard(data_mutex);
            this->axes[axes_ids[axisID]].reg.velocity_limit = velocity;
            this->axes[axes_ids[axisID]].reg.current_limit = current;
            this->axes[axes_ids[axisID]].reg.timestamp = tv;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_start_anticogging(int axisID) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | START_ANTICOGGING; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[Anticogging] Ask to start anticogging" << std::hex
                  << msg_id << std::dec << std::endl;
#endif
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[START_ANTICOGGING]);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - START_ANTICOGGING. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);

        {
            std::lock_guard<std::mutex> guard(data_mutex);
            this->axes[axes_ids[axisID]].reg.anticogging_timestamp = tv;
            this->axes[axes_ids[axisID]].reg.timestamp = tv;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_set_traj_vel_limit(int axisID, float lim) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | SET_TRAJ_VEL_LIMIT; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[SetLimits] Ask to set velocity and current limits - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[4];
        get_char_from_num<float>(data, lim, lsb);
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[SET_TRAJ_VEL_LIMIT], data);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - SET_TRAJ_VEL_LIMIT. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        {
            std::lock_guard<std::mutex> guard(data_mutex);
            this->axes[axes_ids[axisID]].reg.traj_vel_limit = lim;
            this->axes[axes_ids[axisID]].reg.timestamp = tv;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_set_traj_accel_limits(int axisID, float accel, float decel) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | SET_TRAJ_ACCEL_LIMITS; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[SetAccelLimits] Ask to set trajectory acceleration and decellerations limits - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[8];
        get_char_from_nums<float>(data, accel, decel, lsb);
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[SET_TRAJ_ACCEL_LIMITS], data);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - SET_TRAJ_ACCEL_LIMITS. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);

        {
            std::lock_guard<std::mutex> guard(data_mutex);
            this->axes[axes_ids[axisID]].reg.traj_accel_limit = accel;
            this->axes[axes_ids[axisID]].reg.traj_decel_limit = decel;
            this->axes[axes_ids[axisID]].reg.timestamp = tv;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_set_traj_inertia(int axisID, float inertia) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | SET_TRAJ_INERTIA; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[SetAccelLimits] Ask to set trajectory acceleration and decellerations limits - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[4];
        get_char_from_num<float>(data, inertia, lsb);
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[SET_TRAJ_INERTIA], data);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - SET_TRAJ_INERTIA. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        };

        struct timeval tv;
        gettimeofday(&tv, NULL);

        {
            std::lock_guard<std::mutex> guard(data_mutex);
            this->axes[axes_ids[axisID]].reg.traj_inertia = inertia;
            this->axes[axes_ids[axisID]].reg.timestamp = tv;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_get_iq(int axisID) {
    if (key_present(axes_ids, axisID)) {
        // construct can message
        int msg_id = (axisID * 0x20) + GET_IQ; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[GetIQ] Ask for IQ  CAN msg ID " << std::hex << msg_id << std::dec << std::endl;
#endif
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[GET_IQ], true);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - GET_IQ. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_get_tempterature(int axisID) {
    if (key_present(axes_ids, axisID)) {
        // construct can message
        int msg_id = (axisID * 0x20) + GET_TEMP; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[GetTemp] Ask for temperature CAN msg ID " << std::hex << msg_id << std::dec << std::endl;
#endif
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[GET_TEMP], true);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - GET_TEMP. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_reboot(int axisID) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | REBOOT; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[Reboot] Ask to reboot " << std::hex
                  << msg_id << std::dec << std::endl;
#endif
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[REBOOT]);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - REBOOT. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        {
            std::lock_guard<std::mutex> guard(data_mutex);
            this->axes[axes_ids[axisID]].reboot_timestamp = tv;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_get_bus_ui(int axisID) {
    if (key_present(axes_ids, axisID)) {
        // construct can message
        int msg_id = (axisID * 0x20) + GET_BUS_VOLTAGE_CURRENT; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[GetBusUI] Ask for UI  CAN msg ID " << std::hex << msg_id << std::dec << std::endl;
#endif
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[GET_BUS_VOLTAGE_CURRENT], true);
        }
        if (ret < 0) {

            *error_stream << "[ERROR] on axis" << axisID << " - GET_BUS_VOLTAGE_CURRENT. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_clear_errors(int axisID) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | CLEAR_ERRORS; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[Reboot] Ask to clear errors" << std::hex
                  << msg_id << std::dec << std::endl;
#endif
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[CLEAR_ERRORS]);
        }
        if (ret < 0) {

            *error_stream << "[ERROR] on axis" << axisID << " - CLEAR_ERRORS. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_set_absolute_position(int axisID, float pos) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | SET_ABSOLUTE_POSITION; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[SetAbsolutePos] Ask to set absolute position to " << (unsigned)pos << " - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[4];
        get_char_from_num<float>(data, pos, lsb);
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[SET_ABSOLUTE_POSITION], data);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - SET_ABSOLUTE_POSITION. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        };

        {
            std::lock_guard<std::mutex> guard(data_mutex);
            this->axes[axes_ids[axisID]].position = pos;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_set_pos_gain(int axisID, float gain) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | SET_POS_GAIN; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[SetPositionGain] Ask to set position gain " << (unsigned)gain << " - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[4];
        get_char_from_num<float>(data, gain, lsb);
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[SET_POS_GAIN], data);
        }
        if (ret < 0) {
            *error_stream << "[ERROR] on axis" << axisID << " - SET_POS_GAIN. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        };

        struct timeval tv;
        gettimeofday(&tv, NULL);

        {
            std::lock_guard<std::mutex> guard(data_mutex);
            this->axes[axes_ids[axisID]].reg.pos_gain = gain;
            this->axes[axes_ids[axisID]].reg.timestamp = tv;
        }

        return 0;
    } else

        return 1;
}

int OdriveCan::call_set_vel_gains(int axisID, float gain, float integrator) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | SET_VEL_GAINS; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[SetVelocityGain] Ask to set velocity gain " << (unsigned)gain << ", integrator gain "
                  << unsigned(integrator) << " - CAN msg ID"
                  << std::hex << msg_id << std::dec << std::endl;
#endif

        char data[8];
        get_char_from_nums<float>(data, gain, integrator, lsb);
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[SET_VEL_GAINS], data);
        }
        if (ret < 0) {

            *error_stream << "[ERROR] on axis" << axisID << " - SET_VEL_GAINS. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        };

        struct timeval tv;
        gettimeofday(&tv, NULL);

        {
            std::lock_guard<std::mutex> guard(data_mutex);
            this->axes[axes_ids[axisID]].reg.vel_gain = gain;
            this->axes[axes_ids[axisID]].reg.vel_integrator_gain = integrator;
            this->axes[axes_ids[axisID]].reg.timestamp = tv;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_get_torque(int axisID) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = (axisID * 0x20) + GET_TORQUE; // axis ID + can msg name
#ifdef DEBUG
       *output_stream << "[GetTorque] Ask for Torque - CAN msg ID " << std::hex << msg_id << std::dec << std::endl;
#endif
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[GET_TORQUE], true);
        }
        if (ret < 0) {

            *error_stream << "[ERROR] on axis" << axisID << " - GET_TORQUE. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_get_controller_error(int axisID) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = (axisID * 0x20) + GET_CONTROLLER_ERROR; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[GetControllerError] Ask if any errors are in controller - CAN msg ID "
                  << std::hex << msg_id << std::dec << std::endl;
#endif
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[GET_CONTROLLER_ERROR], true);
        }
        if (ret < 0) {

            *error_stream << "[ERROR] on axis" << axisID << " - GET_CONTROLLER_ERROR. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        return 0;
    } else
        return 1;
}

int OdriveCan::call_enter_dfu_mode(int axisID) {
    if (key_present(axes_ids, axisID)) {
        int msg_id = axisID << 5 | ENTER_DFU_MODE; // axis ID + can msg name
#ifdef DEBUG
        *output_stream << "[DFU] Ask to enter DFU mode" << std::hex
                  << msg_id << std::dec << std::endl;
#endif
        int ret = -1;

        {
            std::lock_guard<std::mutex> guard(send_mutex);
            ret = can_dev->send(msg_id, can_msg_len[ENTER_DFU_MODE]);
        }
        if (ret < 0) {

            *error_stream << "[ERROR] on axis" << axisID << " - ENTER_DFU_MODE. The wrong number of bytes were written to CAN. Written: " << ret << ", expected: " << can_msg_len[SET_INPUT_POS] << std::endl;
            return ret;
        }

        return 0;
    } else
        return 1;
}

OdriveAxis OdriveCan::operator[](int index) const {
    {
        std::lock_guard<std::mutex> guard(data_mutex);
        return axes[index];
    }
}

bool OdriveCan::is_axis_active(int axisID) {
    if (key_present(axes_ids, axisID)) {
        struct timeval now;
        gettimeofday(&now, NULL);

        struct timeval can_msg{-1, -1};

        {
            std::lock_guard<std::mutex> guard(data_mutex);
            can_msg = axes[axes_ids[axisID]].can_active;
        }

        int seconds = now.tv_sec - can_msg.tv_sec;
        int miliseconds = seconds * 1000 + ((now.tv_usec - can_msg.tv_usec) / 1000);

        return miliseconds < this->can_timeout_ms;
    }
    return false;
}

bool OdriveCan::is_connected(void) {
    return can_dev->is_connected();
}
