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
    uint32_t heartbeat; /*!<  error, state, done flag */
    uint32_t busIU;     /*!<  bus current bus voltage */
};


/**
 * @brief A class for communicating with odrive axes by CAN messges.
 * 
 * The class expects the axes to be enumerated continuously from 0. 
 *       So that if four axes are connected the axes_num = 4 , the expected axes ID are {0, 1, 2, 3}.
 *       Any other ID's are ignored.
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
    std::unique_ptr<struct updatePeriods> periods; /*!< time constants at which the periodically updated data will be fetched */

    friend std::ostream &operator<<(std::ostream &out, OdriveCan const *oc);

    int get_axes_num();

private:
    std::unordered_map<int, int> canMsgLen;
    int axes_num;
    std::vector<std::shared_ptr<class OdriveAxis>> axes;
    std::unordered_map<int, int> axes_ids;  /*!< Maps axis ID to index in axes vector */

    int buffer_len;
    typedef boost::circular_buffer<canMsg> can_circ_buffer;
    std::vector<std::unique_ptr<can_circ_buffer>> input_buffer; // make a circular buffer for each axis

    std::unique_ptr<CanDevice> can_dev;

    bool run;

    std::mutex msg_mutex;
    std::mutex send_mutex;

    std::thread th_recieve;
    std::thread th_process;
    std::thread th_send;
    std::thread th_errors;

    int update_period_us;
    int err_update_period_us;

    long bit_rate;  // can bitrate

    std::string dev_name;  // conencted device's system name

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

    OdriveCan() : axes_num(6), buffer_len(10), run(1), update_period_us(100000), err_update_period_us(10000)
    {
        std::cout << "[OdriveCAN] Init Ordive can constructor" << std::endl;
        this->init();
    };

    OdriveCan(int axes_num) :  axes_num(axes_num), buffer_len(10), run(1), update_period_us(100000),  err_update_period_us(10000)
    {
        this->init();
    };

    ~OdriveCan()
    {
        th_recieve.join();
        th_process.join();
        th_send.join();
        th_errors.join();
    };

    /**
     * @brief Set the periods object
     *
     * @param[in] heartbeat period at which heartbeat msg is processed
     * @param[in] busUI period at which bus voltage and current are updated
     */
    void set_periods(int32_t heartbeat, int32_t busUI);

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

    bool check_msg_error(uint32_t header);

    void get_errors();

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

    void parse_version(int axisID, canMsg msg);

    void parse_heartbeat(int axisID, canMsg msg);

    void parse_error(int axisID, canMsg msg);

    void parse_encoder_estimates(int axisID, canMsg msg);

    void parse_iq(int axisID, canMsg msg);

    void parse_temp(int axisID, canMsg msg);

    void parse_ui(int axisID, canMsg msg);

    void parse_adc(int axisID, canMsg msg);

    void parse_controller_error(int axisID, canMsg msg);

    uint32_t get32from8(uint8_t *data, int startIdx);

   // void get_char_from_uint(char* arr, uint32_t var);

    template <typename T> 
    void get_char_from_num(char *arr, T var);

    template <typename T> 
    void get_char_from_nums(char *arr, T var1, T var2);

    template <typename T, typename F>
    void get_char_from_nums(char *arr, T var1, F var2, F var3);

    // USER called functions ---------------------------------------------------------
    /**
     * @brief Gets the odrive version by calling get_version message
     * 
     * @param[in] axisID 
     * @return int returns -1 at failure, 0 at sucess
     */
    int call_get_version(int axisID); // NOK returns error - probably not implemented in odrive

    int call_estop(int axisID);  // OK

    int call_get_error(int axisID);

    int call_set_axis_node_id(int oldID, uint32_t newID);

    int call_set_axis_state(int axisID, uint32_t req_state);

    int call_get_encoder_estimates(int axisID);

    int call_set_controller_mode(int axisID, uint32_t control_mode, uint32_t input_mode);

    int call_set_input_pos(int axisID, uint32_t input_pos, float vel_ff, float torque_ff) ;

    int call_set_input_vel(int axisID, uint32_t input_vel, uint32_t input_torque_ff);

    int call_set_input_torque(int axisID, uint32_t torque) ;

    int call_set_limits(int axisID, uint32_t velocity, uint32_t current);

    int call_start_anticogging(int axisID); 

    int call_set_traj_vel_limit(int axisID,  uint32_t lim);

    int call_set_traj_accel_limits(int axisID, uint32_t accel, uint32_t decel);

    int call_set_traj_inertia(int axisID, uint32_t inertia);

    int call_get_iq(int axisID);

    int call_get_tempterature(int axisID); // OK  -returns FET temperature

    int call_reboot(int axisID);

    int call_get_bus_ui(int axisID);  // OK - returns voltage

    int call_clear_errors(int axisID);

    int call_set_absolute_position(int axisID, uint32_t pos);

    int call_set_pos_gain(int axisID, uint32_t gain);

    int call_set_vel_gains(int axisID, uint32_t gain, uint32_t integrator);

    int call_get_adc_voltage(int axisID);

    int call_get_controller_error(int axisID);

    int call_enter_dfu_mode(int axisID);

};
