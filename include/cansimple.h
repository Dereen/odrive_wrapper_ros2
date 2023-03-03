/**
 * @file cansimple.h
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Class for Odrive S1 (PRO) CAN messages ID 
 * @version 0.1
 * @date 2023-03-03
 * 
 * @copyright Copyright (c) 2023
 * see https://docs.odriverobotics.com/v/latest/can-protocol.html#transport-protocol
*/
class CANSimple {
   public:
    enum {
        GET_VERSION = 0x000,  // CANOpen NMT Message REC
        HEARTBEAT,
        ESTOP,
        GET_ERROR,  // Errors
        SET_AXIS_NODE_ID,
        SET_AXIS_STATE,
        GET_ENCODER_ESTIMATES,
        SET_CONTROLLER_MODE = 0x00B,
        SET_INPUT_POS,
        SET_INPUT_VEL,
        SET_INPUT_TORQUE,
        SET_LIMITS,
        START_ANTICOGGING,
        SET_TRAJ_VEL_LIMIT,
        SET_TRAJ_ACCEL_LIMITS,
        SET_TRAJ_INERTIA,
        GET_IQ,
        GET_TEMPERATURE,
        REBOOT,
        GET_BUS_VOLTAGE_CURRENT,
        CLEAR_eRRORS,
        SET_ABSOLUTE_POSITION,
        SET_POS_GAIN,
        SET_VEL_GAINS,
        GET_ADC_VOLTAGE,
        GET_CONTROLLER_ERROR,
        ENTER_DFU_MODE,
    };
};