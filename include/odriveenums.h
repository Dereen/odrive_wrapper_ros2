/**
 * @file odriveenums.h
 * @author samuelsadok
 * @brief Odrive enums
 * @version 0.1
 * @date 2022-03-30
 *
 * see odriverobotics/ODrive/Arduino/ODriveArduino/ODriveEnums.h
 */

#ifndef ODriveEnums_h
#define ODriveEnums_h

#include <map>

/* TODO: This file is dangerous because the enums could potentially change between API versions. Should transmit as part of the JSON.
** To regenerate this file, nagivate to the top level of the ODrive repository and run:
** python Firmware/interface_generator_stub.py --definitions Firmware/odrive-interface.yaml --template tools/arduino_enums_template.j2 --output Arduino/ODriveArduino/ODriveEnums.h
*/

// ODrive.GpioMode
enum GpioMode {
    GPIO_MODE_DIGITAL = 0,
    GPIO_MODE_DIGITAL_PULL_UP = 1,
    GPIO_MODE_DIGITAL_PULL_DOWN = 2,
    GPIO_MODE_ANALOG_IN = 3,
    GPIO_MODE_UART_A = 4,
    GPIO_MODE_UART_B = 5,
    GPIO_MODE_UART_C = 6,
    GPIO_MODE_CAN_A = 7,
    GPIO_MODE_I2C_A = 8,
    GPIO_MODE_SPI_A = 9,
    GPIO_MODE_PWM = 10,
    GPIO_MODE_ENC0 = 11,
    GPIO_MODE_ENC1 = 12,
    GPIO_MODE_ENC2 = 13,
    GPIO_MODE_MECH_BRAKE = 14,
    GPIO_MODE_STATUS = 15,
};

// ODrive.StreamProtocolType
enum StreamProtocolType {
    STREAM_PROTOCOL_TYPE_FIBRE = 0,
    STREAM_PROTOCOL_TYPE_ASCII = 1,
    STREAM_PROTOCOL_TYPE_STDOUT = 2,
    STREAM_PROTOCOL_TYPE_ASCII_AND_STDOUT = 3,
};

// ODrive.Can.Protocol
enum Protocol {
    PROTOCOL_SIMPLE = 0x00000001,
};

static std::map<uint8_t, std::string> can_protocol_map = {{PROTOCOL_SIMPLE,                         "protocol simple"}};

// ODrive.Axis.AxisState
enum AxisState {
    AXIS_STATE_UNDEFINED = 0,
    AXIS_STATE_IDLE = 1,
    AXIS_STATE_STARTUP_SEQUENCE = 2,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
    AXIS_STATE_MOTOR_CALIBRATION = 4,
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
    AXIS_STATE_LOCKIN_SPIN = 9,
    AXIS_STATE_ENCODER_DIR_FIND = 10,
    AXIS_STATE_HOMING = 11,
    AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12,
    AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13,
};

static std::map<uint8_t, std::string> axis_state_map = {
        {AXIS_STATE_UNDEFINED,                         "undefined"},
        {AXIS_STATE_IDLE,                              "idle"},
        {AXIS_STATE_STARTUP_SEQUENCE,                  "startup sequence"},
        {AXIS_STATE_FULL_CALIBRATION_SEQUENCE,         "callibration sequence"},
        {AXIS_STATE_MOTOR_CALIBRATION,                 "motor callibration"},
        {AXIS_STATE_ENCODER_INDEX_SEARCH,              "encoder index search"},
        {AXIS_STATE_ENCODER_OFFSET_CALIBRATION,        "encoder offset callibration"},
        {AXIS_STATE_CLOSED_LOOP_CONTROL,               "closed loop control"},
        {AXIS_STATE_LOCKIN_SPIN,                       "lockin spin"},
        {AXIS_STATE_ENCODER_DIR_FIND,                  "encoder dir find"},
        {AXIS_STATE_HOMING,                            "homing"},
        {AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION, "encoder polarity calibration"},
        {AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION,    "encoder hall phase calibration"}};

// ODrive.Encoder.Mode
enum EncoderMode {
    ENCODER_MODE_INCREMENTAL = 0,
    ENCODER_MODE_HALL = 1,
    ENCODER_MODE_SINCOS = 2,
    ENCODER_MODE_SPI_ABS_CUI = 256,
    ENCODER_MODE_SPI_ABS_AMS = 257,
    ENCODER_MODE_SPI_ABS_AEAT = 258,
    ENCODER_MODE_SPI_ABS_RLS = 259,
    ENCODER_MODE_SPI_ABS_MA732 = 260,
};

// ODrive.Controller.ControlMode
enum ControlMode {
    CONTROL_MODE_UNKNOWN = -1,
    CONTROL_MODE_VOLTAGE_CONTROL = 0,
    CONTROL_MODE_TORQUE_CONTROL = 1,
    CONTROL_MODE_VELOCITY_CONTROL = 2,
    CONTROL_MODE_POSITION_CONTROL = 3,
};

static std::map<ControlMode, std::string> control_mode_map = {
        {CONTROL_MODE_VOLTAGE_CONTROL,  "voltage control"},
        {CONTROL_MODE_TORQUE_CONTROL,   "torque control"},
        {CONTROL_MODE_VELOCITY_CONTROL, "velocity control"},
        {CONTROL_MODE_POSITION_CONTROL, "position control"}};

// ODrive.Controller.InputMode
enum InputMode {
    INPUT_MODE_INACTIVE = 0,
    INPUT_MODE_PASSTHROUGH = 1,
    INPUT_MODE_VEL_RAMP = 2,
    INPUT_MODE_POS_FILTER = 3,
    INPUT_MODE_MIX_CHANNELS = 4,
    INPUT_MODE_TRAP_TRAJ = 5,
    INPUT_MODE_TORQUE_RAMP = 6,
    INPUT_MODE_MIRROR = 7,
    INPUT_MODE_TUNING = 8,
};

static std::map<InputMode, std::string> input_mode_map = {
        {INPUT_MODE_INACTIVE,     "Inactive"},
        {INPUT_MODE_PASSTHROUGH,  "Passthrough"},
        {INPUT_MODE_VEL_RAMP,     "Velocity ramp"},
        {INPUT_MODE_POS_FILTER,   "Position filter"},
        {INPUT_MODE_MIX_CHANNELS, "Mix channels"},
        {INPUT_MODE_TRAP_TRAJ,    "Trap trajectory"},
        {INPUT_MODE_TORQUE_RAMP,  "Torque ramp"},
        {INPUT_MODE_MIRROR,       "Mirror"},
        {INPUT_MODE_TUNING,       "Tuning"}};

// ODrive.Motor.MotorType
enum MotorType {
    MOTOR_TYPE_HIGH_CURRENT = 0,
    MOTOR_TYPE_GIMBAL = 2,
    MOTOR_TYPE_ACIM = 3,
};

// ODrive.Error
enum ODriveError {
    ODRIVE_ERROR_NONE = 0x00000000,
    ODRIVE_ERROR_CONTROL_ITERATION_MISSED = 0x00000001,
    ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE = 0x00000002,
    ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE = 0x00000004,
    ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT = 0x00000008,
    ODRIVE_ERROR_DC_BUS_OVER_CURRENT = 0x00000010,
    ODRIVE_ERROR_BRAKE_DEADTIME_VIOLATION = 0x00000020,
    ODRIVE_ERROR_BRAKE_DUTY_CYCLE_NAN = 0x00000040,
    ODRIVE_ERROR_INVALID_BRAKE_RESISTANCE = 0x00000080,
};

static std::map<int, std::string> odrive_error_map = {
        {ODRIVE_ERROR_NONE,                      "none"},
        {ODRIVE_ERROR_CONTROL_ITERATION_MISSED,  "control iteration missed"},
        {ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE,      "DC bus undervoltage"},
        {ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE,       "DC bus overvoltage"},
        {ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT, "DC bus over regen current"},
        {ODRIVE_ERROR_DC_BUS_OVER_CURRENT,       "DC bus over current"},
        {ODRIVE_ERROR_BRAKE_DEADTIME_VIOLATION,  "brake deadtime violation"},
        {ODRIVE_ERROR_BRAKE_DUTY_CYCLE_NAN,      "breake duty cycle NaN"},
        {ODRIVE_ERROR_INVALID_BRAKE_RESISTANCE,  "invalid brake resistance"}};

// ODrive.Can.Error
enum CanError {
    CAN_ERROR_NONE = 0x00000000,
    CAN_ERROR_DUPLICATE_CAN_IDS = 0x00000001,
};

enum DisarmReason {
    NONE = 0x0,
    INITIALIZING  = 0x1,
    SYSTEM_LEVEL  = 0x2,
    TIMING_ERROR  = 0x4,
    MISSING_ESTIMATE  = 0x8,
    BAD_CONFIG  = 0x10,
    DRV_FAULT  = 0x20,
    MISSING_INPUT  = 0x40,
    DC_BUS_OVER_VOLTAGE  = 0x100,
    DC_BUS_UNDER_VOLTAGE  = 0x200,
    DC_BUS_OVER_CURRENT  = 0x400,
    DC_BUS_OVER_REGEN_CURRENT  = 0x800,
    CURRENT_LIMIT_VIOLATION  = 0x1000,
    MOTOR_OVER_TEMP  = 0x2000,
    INVERTER_OVER_TEMP  = 0x4000,
    VELOCITY_LIMIT_VIOLATION  = 0x8000,
    POSITION_LIMIT_VIOLATION  = 0x10000,
    WATCHDOG_TIMER_EXPIRED  = 0x1000000,
    ESTOP_REQUESTED  = 0x2000000,
    SPINOUT_DETECTED  = 0x4000000,
    OTHER_DEVICE_FAILED  = 0x8000000,
    CALIBRATION_ERROR  = 0x40000000,
};

static std::map<DisarmReason, std::string> disarm_reason_map = {
        {INITIALIZING,                              "Initializing"},
        {SYSTEM_LEVEL,                              "System level"},
        {TIMING_ERROR,                              "Timing error"},
        {MISSING_ESTIMATE,                          "Missing estimate"},
        {BAD_CONFIG,                                "Bad config"},
        {DRV_FAULT,                                 "Drv fault"},
        {MISSING_INPUT,                             "Missing input"},
        {DC_BUS_OVER_VOLTAGE,                       "DC bus over voltage"},
        {DC_BUS_UNDER_VOLTAGE,                      "DC bus under voltage"},
        {DC_BUS_OVER_CURRENT,                       "DC bus over current"},
        {DC_BUS_OVER_REGEN_CURRENT,                 "DC bus over regen current"},
        {CURRENT_LIMIT_VIOLATION,                   "Current limit violation"},
        {MOTOR_OVER_TEMP,                           "Motor over temp"},
        {INVERTER_OVER_TEMP,                        "Inverter over temp"},
        {VELOCITY_LIMIT_VIOLATION,                  "Velocity limit violation"},
        {POSITION_LIMIT_VIOLATION,                  "Position limit violation"},
        {WATCHDOG_TIMER_EXPIRED,                    "Watchdog timer expired"},
        {ESTOP_REQUESTED,                           "Emergency stop requested"},
        {SPINOUT_DETECTED,                          "Spinout detected"},
        {OTHER_DEVICE_FAILED,                       "Other device failed"},
        {CALIBRATION_ERROR,                         "Calibration error"}};

// ODrive.Axis.Error
enum AxisError {
    AXIS_ERROR_NONE = 0x00000000,
    AXIS_ERROR_INVALID_STATE = 0x00000001,
    AXIS_ERROR_MOTOR_FAILED = 0x00000040,
    AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED = 0x00000080,
    AXIS_ERROR_ENCODER_FAILED = 0x00000100,
    AXIS_ERROR_CONTROLLER_FAILED = 0x00000200,
    AXIS_ERROR_WATCHDOG_TIMER_EXPIRED = 0x00000800,
    AXIS_ERROR_MIN_ENDSTOP_PRESSED = 0x00001000,
    AXIS_ERROR_MAX_ENDSTOP_PRESSED = 0x00002000,
    AXIS_ERROR_ESTOP_REQUESTED = 0x00004000,
    AXIS_ERROR_HOMING_WITHOUT_ENDSTOP = 0x00020000,
    AXIS_ERROR_OVER_TEMP = 0x00040000,
    AXIS_ERROR_UNKNOWN_POSITION = 0x00080000,
};

static std::map<AxisError, std::string> axis_error_map = {
        {AXIS_ERROR_NONE,                        "None"},
        {AXIS_ERROR_INVALID_STATE,               "Invalid state"},
        {AXIS_ERROR_MOTOR_FAILED,                "Motor failed"},
        {AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED, "Sensorless estimator failed"},
        {AXIS_ERROR_ENCODER_FAILED,              "Encoder failed"},
        {AXIS_ERROR_CONTROLLER_FAILED,           "Controller failed"},
        {AXIS_ERROR_WATCHDOG_TIMER_EXPIRED,      "Watchdog timer expired"},
        {AXIS_ERROR_MIN_ENDSTOP_PRESSED,         "Min endstop pressed"},
        {AXIS_ERROR_MAX_ENDSTOP_PRESSED,         "Max endstop pressed"},
        {AXIS_ERROR_ESTOP_REQUESTED,             "Estop requested"},
        {AXIS_ERROR_HOMING_WITHOUT_ENDSTOP,      "Homing without endstop"},
        {AXIS_ERROR_OVER_TEMP,                   "Over temp"},
        {AXIS_ERROR_UNKNOWN_POSITION,            "Unknown position"}};

// ODrive.Motor.Error
enum MotorError {
    MOTOR_ERROR_NONE = 0x00000000,
    MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 0x00000001,
    MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 0x00000002,
    MOTOR_ERROR_DRV_FAULT = 0x00000008,
    MOTOR_ERROR_CONTROL_DEADLINE_MISSED = 0x00000010,
    MOTOR_ERROR_MODULATION_MAGNITUDE = 0x00000080,
    MOTOR_ERROR_CURRENT_SENSE_SATURATION = 0x00000400,
    MOTOR_ERROR_CURRENT_LIMIT_VIOLATION = 0x00001000,
    MOTOR_ERROR_MODULATION_IS_NAN = 0x00010000,
    MOTOR_ERROR_MOTOR_THERMISTOR_OVER_TEMP = 0x00020000,
    MOTOR_ERROR_FET_THERMISTOR_OVER_TEMP = 0x00040000,
    MOTOR_ERROR_TIMER_UPDATE_MISSED = 0x00080000,
    MOTOR_ERROR_CURRENT_MEASUREMENT_UNAVAILABLE = 0x00100000,
    MOTOR_ERROR_CONTROLLER_FAILED = 0x00200000,
    MOTOR_ERROR_I_BUS_OUT_OF_RANGE = 0x00400000,
    MOTOR_ERROR_BRAKE_RESISTOR_DISARMED = 0x00800000,
    MOTOR_ERROR_SYSTEM_LEVEL = 0x01000000,
    MOTOR_ERROR_BAD_TIMING = 0x02000000,
    MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE = 0x04000000,
    MOTOR_ERROR_UNKNOWN_PHASE_VEL = 0x08000000,
    MOTOR_ERROR_UNKNOWN_TORQUE = 0x10000000,
    MOTOR_ERROR_UNKNOWN_CURRENT_COMMAND = 0x20000000,
    MOTOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT = 0x40000000,
    MOTOR_ERROR_UNKNOWN_VBUS_VOLTAGE = 0x80000000,
    MOTOR_ERROR_UNKNOWN_VOLTAGE_COMMAND = 0x100000000,
    MOTOR_ERROR_UNKNOWN_GAINS = 0x200000000,
    MOTOR_ERROR_CONTROLLER_INITIALIZING = 0x400000000,
    MOTOR_ERROR_UNBALANCED_PHASES = 0x800000000,
};

static std::map<MotorError, std::string> motor_error_map = {
        {MOTOR_ERROR_NONE,                              "None"},
        {MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE,     "Phase resistance out of range"},
        {MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE,     "Inductance out of range"},
        {MOTOR_ERROR_DRV_FAULT,                         "Drv fault"},
        {MOTOR_ERROR_CONTROL_DEADLINE_MISSED,           "Control deadline missed"},
        {MOTOR_ERROR_MODULATION_MAGNITUDE,              "Modulation magnitude"},
        {MOTOR_ERROR_CURRENT_SENSE_SATURATION,          "Current sense saturation"},
        {MOTOR_ERROR_CURRENT_LIMIT_VIOLATION,           "Limit violation"},
        {MOTOR_ERROR_MODULATION_IS_NAN,                 "Modulation is nan"},
        {MOTOR_ERROR_MOTOR_THERMISTOR_OVER_TEMP,        "Motor thermistor over temp"},
        {MOTOR_ERROR_FET_THERMISTOR_OVER_TEMP,          "Fet thermistor over temp"},
        {MOTOR_ERROR_TIMER_UPDATE_MISSED,               "Timer update missed"},
        {MOTOR_ERROR_CURRENT_MEASUREMENT_UNAVAILABLE,   "Current measurement unavailable"},
        {MOTOR_ERROR_CONTROLLER_FAILED,                 "Controller failed"},
        {MOTOR_ERROR_I_BUS_OUT_OF_RANGE,                "I bus out of range"},
        {MOTOR_ERROR_BRAKE_RESISTOR_DISARMED,           "Brake resistor disarmed"},
        {MOTOR_ERROR_SYSTEM_LEVEL,                      "System level"},
        {MOTOR_ERROR_BAD_TIMING,                        "Bad timing"},
        {MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE,            "Unknown phase estimate"},
        {MOTOR_ERROR_UNKNOWN_PHASE_VEL,                 "Unknown phase vel"},
        {MOTOR_ERROR_UNKNOWN_TORQUE,                    "Unknown torque"},
        {MOTOR_ERROR_UNKNOWN_CURRENT_COMMAND,           "Unknown current command"},
        {MOTOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT,       "Unknown current measurement"},
        {MOTOR_ERROR_UNKNOWN_VBUS_VOLTAGE,              "Unknown VBus voltage"},
        {MOTOR_ERROR_UNKNOWN_VOLTAGE_COMMAND,           "Unknown voltage command"},
        {MOTOR_ERROR_UNKNOWN_GAINS,                     "Unknown gains"},
        {MOTOR_ERROR_CONTROLLER_INITIALIZING,           "Controller initializing"},
        {MOTOR_ERROR_UNBALANCED_PHASES,                 "Unbalanced phases"}};

// ODrive.Controller.Error
enum ControllerError {
    CONTROLLER_ERROR_NONE = 0x00000000,
    CONTROLLER_ERROR_OVERSPEED = 0x00000001,
    CONTROLLER_ERROR_INVALID_INPUT_MODE = 0x00000002,
    CONTROLLER_ERROR_UNSTABLE_GAIN = 0x00000004,
    CONTROLLER_ERROR_INVALID_MIRROR_AXIS = 0x00000008,
    CONTROLLER_ERROR_INVALID_LOAD_ENCODER = 0x00000010,
    CONTROLLER_ERROR_INVALID_ESTIMATE = 0x00000020,
    CONTROLLER_ERROR_INVALID_CIRCULAR_RANGE = 0x00000040,
    CONTROLLER_ERROR_SPINOUT_DETECTED = 0x00000080,
};

static std::map<ControllerError, std::string> controller_error_map = {
        {CONTROLLER_ERROR_NONE,                   "None"},
        {CONTROLLER_ERROR_OVERSPEED,              "Overspeed"},
        {CONTROLLER_ERROR_INVALID_INPUT_MODE,     "Invalid input mode"},
        {CONTROLLER_ERROR_UNSTABLE_GAIN,          "Unstable gain"},
        {CONTROLLER_ERROR_INVALID_MIRROR_AXIS,    "Invalid mirror axis"},
        {CONTROLLER_ERROR_INVALID_LOAD_ENCODER,   "Invalid load encoder"},
        {CONTROLLER_ERROR_INVALID_ESTIMATE,       "Invalid estimate"},
        {CONTROLLER_ERROR_INVALID_CIRCULAR_RANGE, "Invalid circular range"},
        {CONTROLLER_ERROR_SPINOUT_DETECTED,       "Spinout detected"}};

// ODrive.Encoder.Error
enum EncoderError {
    ENCODER_ERROR_NONE = 0x00000000,
    ENCODER_ERROR_UNSTABLE_GAIN = 0x00000001,
    ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH = 0x00000002,
    ENCODER_ERROR_NO_RESPONSE = 0x00000004,
    ENCODER_ERROR_UNSUPPORTED_ENCODER_MODE = 0x00000008,
    ENCODER_ERROR_ILLEGAL_HALL_STATE = 0x00000010,
    ENCODER_ERROR_INDEX_NOT_FOUND_YET = 0x00000020,
    ENCODER_ERROR_ABS_SPI_TIMEOUT = 0x00000040,
    ENCODER_ERROR_ABS_SPI_COM_FAIL = 0x00000080,
    ENCODER_ERROR_ABS_SPI_NOT_READY = 0x00000100,
    ENCODER_ERROR_HALL_NOT_CALIBRATED_YET = 0x00000200,
};

// ODrive.SensorlessEstimator.Error
enum SensorlessEstimatorError {
    SENSORLESS_ESTIMATOR_ERROR_NONE = 0x00000000,
    SENSORLESS_ESTIMATOR_ERROR_UNSTABLE_GAIN = 0x00000001,
    SENSORLESS_ESTIMATOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT = 0x00000002,
};

enum ProcedureResult {
    SUCCESS = 0x0,
    BUSY = 0x1,
    CANCELLED = 0x2,
    DISARMED = 0x3,
    NO_RESPONSE = 0x4,
    POLE_PAIR_CPR_MISMATCH = 0x5,
    PHASE_RESISTANCE_OUT_OF_RANGE = 0x6,
    PHASE_INDUCTANCE_OUT_OF_RANGE = 0x7,
    UNBALANCED_PHASES = 0x8,
    INVALID_MOTOR_TYPE = 0x9,
    ILLEGAL_HALL_STATE = 0xA,
    TIMEOUT = 0xB,
    HOMING_WITHOUT_ENDSTOP = 0xC,
    INVALID_STATE = 0xD,
    NOT_CALIBRATED = 0xE
};

static std::map<ProcedureResult, std::string> procedure_result_map = {
        {SUCCESS,                           "Success"},
        {BUSY,                              "Busy"},
        {CANCELLED,                         "Cancelled"},
        {DISARMED,                          "Disarmed"},
        {NO_RESPONSE,                       "No response"},
        {POLE_PAIR_CPR_MISMATCH,            "Pole pair cpr mismatch"},
        {PHASE_RESISTANCE_OUT_OF_RANGE,     "Phase resistance out of range"},
        {UNBALANCED_PHASES,                 "Unbalanced"},
        {PHASE_INDUCTANCE_OUT_OF_RANGE,     "Phase inductance out of range"},
        {INVALID_MOTOR_TYPE,                "Invalid motor type"},
        {ILLEGAL_HALL_STATE,                "Illegal hall state"},
        {TIMEOUT,                           "Timeout"},
        {HOMING_WITHOUT_ENDSTOP,            "Homing without endstop"},
        {INVALID_STATE,                     "Invalid state"},
        {NOT_CALIBRATED,                    "Not calibrated"}};


#endif