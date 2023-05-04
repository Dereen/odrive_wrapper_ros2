/**
 * @file OdriveCan.h
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Class for Odrive S1 (PRO) CAN messages ID
 * @version 0.1
 * @date 2023-05-04
 *
 * @copyright Copyright (c) 2023
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

/**
 * @brief Time constants at which the periodically updated data will be fetched
 */
struct updatePeriods
{
    uint32_t axis_status; /*!<  error, state, done flag */
    uint32_t data;        /*!<  bus UI, temperature, encoders ADC */
};

/**
 * @brief A class for communicating with odrive axes by CAN messges in ver. 0.6.4
 *
 * In the basic configuration the class expects the axes to be enumerated continuously from 0.
 *       So that if four axes are connected the axes_num = 4 , the expected axes ID are {0, 1, 2, 3}.
 *       Any other ID's are ignored. If your axes have different ids, you can use OdriveCan(int axes_num, std::vector<int> ax_ids).
 *
 * Note: in Odrive's simpleCAN implementation, the CAN's error message is incorrectly  implemented.
 *       Instead of sending the return message with CAN_ERR_FLAG, Odrive returns message filled with 0's,
 *       which can be easily misinterpreted as the response of Axis0 to get_version.
 *       Therefore, do not use the axis0, set axes_num = number of used axes + 1 and ignore data stored at axis0.
 */
class OdriveCan : OdriveAxis
{
    friend OdriveAxis;
    typedef struct
    {
        can_frame frame;
        struct timeval timestamp;
    } canMsg;

public:
    std::unique_ptr<struct updatePeriods> periods; /*!< time constants in ms at which the periodically updated data will be fetched */

    friend std::ostream &operator<<(std::ostream &out, OdriveCan const *oc); // function for nice data printing

private:
    std::unordered_map<int, int> canMsgLen; /*!< Map data's part of CAN message length for individual messages */

    int axes_num;                                        /*!< Number of initialized axes */
    std::vector<std::shared_ptr<class OdriveAxis>> axes; /*!< Array of initialized axes */
    std::unordered_map<int, int> axes_ids;               /*!< Maps axis ID to index in axes vector */

    int buffer_len; /*!< Buffer length for storing messages for each axis */
    typedef boost::circular_buffer<canMsg> can_circ_buffer;
    std::vector<std::unique_ptr<can_circ_buffer>> input_buffer; /*!<  A circular buffer for each axis info storage */

    std::unique_ptr<CanDevice> can_dev; /*!< Class for low level can operations */

    bool run; /*!< Flag for finishing threads */

    std::mutex buffer_mutex; /*!< Mutex for reading messages from buffers */
    std::mutex send_mutex;   /*!< Mutex for sending messages */

    std::thread th_recieve; /*!< Thread that reads messages from CAN interface and stores them into buffers */
    std::thread th_process; /*!< Thread for processing messages from buffers */
    std::thread th_send;    /*!< Thread for requesting periodic actuator readings */
    std::thread th_errors;  /*!< Thread for fetching errors */

    long bit_rate; // can bitrate

    std::string dev_name; // conencted device's system name

    bool lsb; // defines is architecture is Least Significant Bit

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
    OdriveCan() : axes_num(6), buffer_len(10), run(1), lsb(true)
    {
        std::cout << "[OdriveCAN] Init Ordive can constructor" << std::endl;
        this->init();
    };

    OdriveCan(int axes_num) : axes_num(axes_num), buffer_len(10), run(1), lsb(false)
    {
        this->init();
    };

    OdriveCan(int axes_num, std::vector<int> ax_ids) : axes_num(axes_num), buffer_len(10), run(1), lsb(true)
    {
        this->init();

        // check if correct number of indexes was passed
        if (ax_ids.size() == this->axes_ids.size())
        {
            // delete all map
            this->axes_ids.clear();

            for (int i = 0; i < axes_num; i++)
            {
                // create new instance
                this->axes_ids[ax_ids[i]] = i;

                this->axes[i]->set_axis_id(ax_ids[i]);
            }
        }
        else
        {
            std::cerr << "[ERROR] Got incorrect number of new ID's for the pre-allocated amount of axes" << std::endl;
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
     * @param[in] new_period new update period for new temperature, encoder, busUI, motor current  and ADC data requests in ms
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
     * @brief Parses resopnse to Get_Iq message
     *
     * @param[in] axisID axis id to which the message corresponds
     * @param[in] msg recieved data
     */
    void parse_iq(int axisID, canMsg msg);

    /**
     * @brief Parses resopnse to Get_Temp message
     *
     * @param[in] axisID axis id to which the message corresponds
     * @param[in] msg recieved data
     */
    void parse_temp(int axisID, canMsg msg);

    /**
     * @brief Parses resopnse to Get_Bus_Voltage_Current message
     *
     * @param[in] axisID axis id to which the message corresponds
     * @param[in] msg recieved data
     */
    void parse_ui(int axisID, canMsg msg);

    /**
     * @brief Parses resopnse to Get_ADC_Voltage message
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
     * @param[in] axisID axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_estop(int axisID);

    /**
     * @brief Gets active errors and disarm reason by calling get_error message
     *
     * @param[in] axisID axisID ID that corresponds to message's destination axis
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
     * @param[in] axisID axisID ID that corresponds to message's destination axis
     * @param[in] req_state corresponds to AxisState in odriveenums.h
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_axis_state(int axisID, uint32_t req_state);

    /**
     * @brief Gets encoder position and velocity estimates by calling get_encoder_estimates message
     *
     * @param[in] axisID axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_get_encoder_estimates(int axisID);

    /**
     * @brief Sets controller mode by calling set_controller_mode message
     *
     * @param[in] axisID axisID ID that corresponds to message's destination axis
     * @param[in] control_mode corresponds to ControlMode in odriveenums.h
     * @param[in] input_mode corresponds to InputMode in odriveenums.h
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_controller_mode(int axisID, uint32_t control_mode, uint32_t input_mode);

    /**
     * @brief Sets input position parameters by calling set_input_pos message
     *
     * @param[in] axisID axisID ID that corresponds to message's destination axis
     * @param[in] input_pos input position
     * @param[in] vel_ff velocity feed forward
     * @param[in] torque_ff torque feed forward
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_input_pos(int axisID, float input_pos, float vel_ff, float torque_ff);

    /**
     * @brief Sets input velocity parameters by calling set_input_vel message
     *
     * @param[in] axisID axisID ID that corresponds to message's destination axis
     * @param[in] input_vel input velocity
     * @param[in] input_torque_ff input torque feed forward
     * @param[in] torque_ff torque feed forward
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_input_vel(int axisID, float input_vel, float input_torque_ff);

    /**
     * @brief Sets input torque parameters by calling set_input_torque message
     *
     * @param[in] axisID axisID ID that corresponds to message's destination axis
     * @param[in] torque input torque
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_input_torque(int axisID, float torque);

    /**
     * @brief Sets velocity and current limits by calling set_limits message
     *
     * @param[in] axisID axisID ID that corresponds to message's destination axis
     * @param[in] velocity velocity limit
     * @param[in] current current limit
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_set_limits(int axisID, float velocity, float current);

    int call_start_anticogging(int axisID);

    int call_set_traj_vel_limit(int axisID, float lim);

    int call_set_traj_accel_limits(int axisID, float accel, float decel);

    int call_set_traj_inertia(int axisID, float inertia);

    /**
     * @brief Fetches commanded and measured motor currents by calling get_iq message
     * 
     * @param[in] axisID axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_get_iq(int axisID);

    /**
     * @brief Fetches FET temperature by calling get_temperature message. Odrive does not respond with motor temperature
     * 
     * @param[in] axisID axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_get_tempterature(int axisID); // OK  -returns FET temperature

    /**
     * @brief reboots odrive by calling reboot message
     * 
     * @param[in] axisID axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_reboot(int axisID);

    int call_get_bus_ui(int axisID); // OK - returns voltage

    int call_clear_errors(int axisID);

    int call_set_absolute_position(int axisID, float pos);

    int call_set_pos_gain(int axisID, float gain);

    int call_set_vel_gains(int axisID, float gain, float integrator);

    int call_get_adc_voltage(int axisID);

    int call_get_controller_error(int axisID);

    /**
     * @brief sets odrive to software update mode by calling enter_dfu_mode message
     * 
     * @param[in] axisID axisID ID that corresponds to message's destination axis
     * @return int returns -1 at bus write failure, 0 at sucess
     */
    int call_enter_dfu_mode(int axisID);
};
