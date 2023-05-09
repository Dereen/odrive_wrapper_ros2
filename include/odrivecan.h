/**
 * @file odrivecan.h
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Class for Odrive S1 (PRO) CAN messages ID
 * @version 0.1
 * @date 2023-05-04
 *
 # @copyright (c) JettyVision s.r.o in Prague 2023 - All Rights Reserved
 * see https://docs.odriverobotics.com/v/latest/can-protocol.html#transport-protocol
 */

#pragma once
#include <vector>
#include <memory>
#include <cstring>

#include "candevice.h"
#include "odriveaxis.h"
#include <boost/circular_buffer.hpp>

#include <thread>
#include <mutex>
#include <unordered_map>

struct OdriveCanParams
{
    boost::optional<uint32_t> axis_status_update_ms; /*!<  error, state, done flag */
    boost::optional<uint32_t> data_update_ms;        /*!<  bus UI, temperature, encoders ADC */
    boost::optional<int> buffer_len;                 /*!< Buffer length for storing messages for each axis */
    boost::optional<int> axes_num;                   /*!< Number of initialized axes */
    boost::optional<std::vector<int>> axes_IDs;      /*!< Maps axis ID to index in axes vector */
    boost::optional<std::string> dev_name;           /*!< conencted device's system name, e.g. can0... */
    boost::optional<bool> lsb;                       /*!<  defines is architecture is Least Significant Bit */
};

/**
 * @brief A class for communicating with odrive axes by CAN messges (ver. 0.6.6)
 *
 * In the basic configuration the class expects the axes to be enumerated continuously from 0.
 *       So that if four axes are connected the axes_num = 4 , the expected axes ID are {0, 1, 2, 3}.
 *       Any other ID's are ignored. If your axes have different ids, you can use OdriveCan(int axes_num, std::vector<int> ax_ids).
 *
 * Note: in Odrive's ver. 0.6.4. simpleCAN implementation, the CAN's error message is incorrectly  implemented.
 *       Instead of sending the return message with CAN_ERR_FLAG, Odrive returns message filled with 0's,
 *       which can be easily misinterpreted as the response of Axis0 to get_version.
 *       It is highly recommended to update the Firmware to version 0.6.6
 *       If you insist on ver. 0.6.4, do not use the axis0, set axes_num = number of used axes + 1 and ignore data stored at axis0.
 *
 * Note: The simpleCAN protocol does not provide timestamps, so the OdriveAxis is filled by timestamp retrieved from socket when
 *       message is recieved
 */
class OdriveCan
{
    typedef struct
    {
        can_frame frame;
        struct timeval timestamp;
    } canMsg;

public:
    friend std::ostream &operator<<(std::ostream &out, const OdriveCan& odrive); // function for nice data printing

private:
    std::unordered_map<int, int> canMsgLen; /*!< Map data's part of CAN message length for individual messages */

    int axes_num;                                        /*!< Number of initialized axes */
    std::vector<OdriveAxis> axes; /*!< Array of initialized axes */
    std::unordered_map<int, int> axes_ids;               /*!< Maps axis ID to index in axes vector */

    int buffer_len; /*!< Buffer length for storing messages for each axis */
    typedef boost::circular_buffer<canMsg> can_circ_buffer;
    std::vector<std::unique_ptr<can_circ_buffer>> input_buffer; /*!<  A circular buffer for each axis info storage */

    std::unique_ptr<CanDevice> can_dev; /*!< Class for low level can operations */

    bool run;                /*!< Flag for finishing threads */

    std::mutex buffer_mutex; /*!< Mutex for reading messages from buffers */
    std::mutex send_mutex;   /*!< Mutex for sending messages */

    std::thread th_recieve; /*!< Thread that reads messages from CAN interface and stores them into buffers */
    std::thread th_process; /*!< Thread for processing messages from buffers */
    std::thread th_send;    /*!< Thread for requesting periodic actuator readings */
    std::thread th_errors;  /*!< Thread for fetching errors */

    std::string dev_name;   /*!< conencted device's system name */

    bool lsb;                       /*!< defines is architecture is Least Significant Bit */
    uint32_t axis_status_update_ms; /*!<  error, state, done flag */
    uint32_t data_update_ms;        /*!<  bus UI, temperature, encoders ADC */

    enum
    {
        GET_VERSION = 0x000,
        HEARTBEAT,
        ESTOP,
        GET_ERROR,
        SET_AXIS_NODE_ID = 0x006,
        SET_AXIS_STATE,
        GET_ENCODER_ESTIMATES = 0x009,
        SET_CONTROLLER_MODE = 0x00b,
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

    void init();

public:
    OdriveCan(int axes_num = 6) : axes_num(axes_num), buffer_len(10), run(1), lsb(false), axis_status_update_ms(100), data_update_ms(100)
    {
        this->init();
    };

    OdriveCan(struct OdriveCanParams param) : axes_num(8), buffer_len(10), run(1), lsb(false), axis_status_update_ms(100), data_update_ms(100)
    {
        if (param.axes_num != boost::none)
            this->axes_num = *param.axes_num;
        if (param.axis_status_update_ms != boost::none)
            this->axis_status_update_ms = *param.axis_status_update_ms;
        if (param.data_update_ms != boost::none)
            this->data_update_ms = *param.data_update_ms;
        if (param.buffer_len != boost::none)
            this->buffer_len = *param.buffer_len;
        if (param.dev_name != boost::none)
            this->dev_name = *param.dev_name;
        if (param.lsb != boost::none)
            this->lsb = *param.lsb;

        this->init();

        if ((param.axes_IDs != boost::none))
        {
            std::vector<int> arr = *param.axes_IDs;
            if  (arr.size() == this->axes_ids.size()){
                // delete all map
                this->axes_ids.clear();

                for (int i = 0; i < axes_num; i++)
                {
                    // create new instance
                    this->axes_ids[arr[i]] = i;

                    this->axes[i].id = arr[i];
                }
            }   else
            {
                std::cerr << "[ERROR] Got incorrect number of new ID's for the pre-allocated amount of axes" << std::endl;
            }
        }
    };


    ~OdriveCan()
    {
        run = false;
        th_recieve.join();
        th_process.join();
        th_send.join();
        th_errors.join();
    };

    /**
     * @brief Get the number of initalized axis containers
     *
     * @return int number of initialized axis data containers
     */
    int get_axes_num();

    /**
     * @brief Set the data update period
     *
     * @param[in] new_period new update period for new temperature, encoderStruct, busUI, iqStruct  and ADC data requests in ms
     */
    void set_update_period(time_t new_period);

    /**
     * @brief Set the status update period
     *
     * @param[in] new_period update the period for axis status data requests in ms
     */
    void set_status_update_period(time_t new_period);

    /**
     * @brief Set the periods object
     *
     * @param[in] status period at which heartbeat msg is processed and errors are fetched
     * @param[in] data_time period at which bus voltage and current, ADC voltage, temperature, encoder estimates are updated
     */
    void set_periods(int32_t status_time, int32_t data_time);

    /**
     * @brief parses recieved can header
     *
     * @param[in] header can_id from can_frame
     * @param[out] axisID corresponding odrive axis id
     * @param[out] cmdID command id
     */
    void parse_header(uint32_t header, int *axisID, int *cmdID);

    /**
     * @brief Returns CAN message ID from header
     *
     * @param[in] header recieved CAN message header
     * @return int CAN message ID
     */
    int can_id_from_header(uint32_t header);

    /**
     * @brief returns axis ID from can_frame header
     *
     * @param[in] header CAN frame header
     * @return int axis ID
     */
    int axis_from_header(uint32_t header);

    /**
     * @brief Gets the errors. calls Get_Error and Get_Controller_Error periodically
     *
     */
    void get_errors();

    /**
     * @brief Periodiaclly asks for temperature readings, encoder estimation, bus UI, motor current, ADC voltage
     *
     */
    void ask_for_current_values();

    /**
     * @brief recieves can messages and stores them in circluar buffers for each axis
     *
     */
    void receive_msgs();

    /**
     * @brief processes can messages from circluar buffers for each axis
     *
     */
    void process_msgs();

    /**
     * @brief Parses response to Get_Version message
     *
     * @param[in] axisID axis id to which the message corresponds
     * @param[in] msg recieved data
     */
    void parse_version(int axisID, canMsg msg);

    /**
     * @brief Parses Heartbeat message
     *
     * @param[in] axisID axis id to which the message corresponds
     * @param[in] msg recieved data
     */
    void parse_heartbeat(int axisID, canMsg msg);

    /**
     * @brief Parses Get_Error message
     *
     * @param[in] axisID axis id to which the message corresponds
     * @param[in] msg recieved data
     */
    void parse_error(int axisID, canMsg msg);

    /**
     * @brief Parses Get_Controller_Error message
     *
     * @param[in] axisID axis id to which the message corresponds
     * @param[in] msg recieved data
     */
    void parse_controller_error(int axisID, canMsg msg);

    /**
     * @brief Parses response to Get_Encoder_Estimates message
     *
     * @param[in] axisID axis id to which the message corresponds
     * @param[in] msg recieved data
     */
    void parse_encoder_estimates(int axisID, canMsg msg);

    /**
     * @brief Parses response to Get_Iq message
     *
     * @param[in] axisID axis id to which the message corresponds
     * @param[in] msg recieved data
     */
    void parse_iq(int axisID, canMsg msg);

    /**
     * @brief Parses response to Get_Temp message
     *
     * @param[in] axisID axis id to which the message corresponds
     * @param[in] msg recieved data
     */
    void parse_temp(int axisID, canMsg msg);

    /**
     * @brief Parses response to Get_Bus_Voltage_Current message
     *
     * @param[in] axisID axis id to which the message corresponds
     * @param[in] msg recieved data
     */
    void parse_ui(int axisID, canMsg msg);

    /**
     * @brief Parses response to Get_ADC_Voltage message
     *
     * @param[in] axisID axis id to which the message corresponds
     * @param[in] msg recieved data
     */
    void parse_adc(int axisID, canMsg msg);

    // USER called functions ---------------------------------------------------------
    /**
     * @brief Gets the odrive version by calling get_version message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_get_version(int axisID);

    /**
     * @brief Calls ESTOP
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_estop(int axisID);

    /**
     * @brief Gets active errors and disarm reason by calling get_error message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_get_error(int axisID);

    /**
     * @brief Sets new axis ID via CAN
     *
     * @param[in] oldID old axis ID
     * @param[in] newID new axis ID
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_axis_node_id(int oldID, uint32_t newID);

    /**
     * @brief Sets axis state by calling set_axis_state message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @param[in] req_state corresponds to AxisState in odriveenums.h
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_axis_state(int axisID, uint32_t req_state);

    /**
     * @brief Gets encoder position and velocity estimates by calling get_encoder_estimates message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_get_encoder_estimates(int axisID);

    /**
     * @brief Sets controller mode by calling set_controller_mode message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @param[in] control_mode corresponds to ControlMode in odriveenums.h
     * @param[in] input_mode corresponds to InputMode in odriveenums.h
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_controller_mode(int axisID, uint32_t control_mode, uint32_t input_mode);

    /**
     * @brief Sets input position parameters by calling set_input_pos message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @param[in] input_pos input position
     * @param[in] vel_ff velocity feed forward
     * @param[in] torque_ff torque feed forward
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_input_pos(int axisID, float input_pos, float vel_ff, float torque_ff);

    /**
     * @brief Sets input velocity parameters by calling set_input_vel message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @param[in] input_vel input velocity
     * @param[in] input_torque_ff input torque feed forward
     * @param[in] torque_ff torque feed forward
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_input_vel(int axisID, float input_vel, float input_torque_ff);

    /**
     * @brief Sets input torque parameters by calling set_input_torque message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @param[in] torque input torque
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_input_torque(int axisID, float torque);

    /**
     * @brief Sets velocity and current limits by calling set_limits message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @param[in] velocity velocity limit
     * @param[in] current current limit
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_limits(int axisID, float velocity, float current);

    /**
     * @brief Calls start_anticogging message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_start_anticogging(int axisID);

    /**
     * @brief Sets trajectory velocity limit by calling set_traj_vel_limit message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @param[in] lim trajectory velocity limit
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_traj_vel_limit(int axisID, float lim);

    /**
     * @brief Sets trajectory acceleration and deceleration limits by calling set_traj_acel_limit message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @param[in] accel trajectory acceleration limit
     * @param[in] deccel trajectory deceleration limit
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_traj_accel_limits(int axisID, float accel, float decel);

    /**
     * @brief Sets trajectory inertia by calling set_traj_inertia message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @param[in] inertia trajectory inertia
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_traj_inertia(int axisID, float inertia);

    /**
     * @brief Fetches commanded and measured motor currents by calling get_iq message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_get_iq(int axisID);

    /**
     * @brief Fetches FET temperature by calling get_temperature message. Odrive does not respond with motor temperature
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_get_tempterature(int axisID); // OK  -returns FET temperature

    /**
     * @brief reboots odrive by calling reboot message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_reboot(int axisID);

    /**
     * @brief Fetches bus current and voltage by calling get_bus_voltage_current message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_get_bus_ui(int axisID); // OK - returns voltage

    /**
     * @brief Clears odrive errors by calling clear_errors message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_clear_errors(int axisID);

    /**
     * @brief Sets absolute position reference inertia by calling set_absolute_position message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @param[in] pos absolute position
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_absolute_position(int axisID, float pos);

    /**
     * @brief Sets position gain by calling set_pos_gain message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @param[in] gain position gain
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_pos_gain(int axisID, float gain);

    /**
     * @brief Sets velocity gain and velocity integrator gain by calling set_vel_gains message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @param[in] gain velocity gain
     * @param[in] integrator velocity integrator gain
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_vel_gains(int axisID, float gain, float integrator);

    /**
     * @brief Fetches ADC voltage by calling get_adc_voltage message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_get_adc_voltage(int axisID);

    /**
     * @brief Fetches controller error by calling get_controller_error message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_get_controller_error(int axisID);

    /**
     * @brief sets odrive to software update mode by calling enter_dfu_mode message
     *
     * @param[in] axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_enter_dfu_mode(int axisID);

    const OdriveAxis& operator[](int index);

};
