/**
 * @file odriveaxis.cpp
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Structure for fetching and storing odrive settings info
 * @version 0.1
 * @date 2023-03-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "odriveaxis.h"

std::map<uint8_t, std::string> axis_state_map = {
    {AXIS_STATE_UNDEFINED, "undefined"},
    {AXIS_STATE_IDLE, "idle"},
    {AXIS_STATE_STARTUP_SEQUENCE, "startup sequence"},
    {AXIS_STATE_FULL_CALIBRATION_SEQUENCE, "callibration sequence"},
    {AXIS_STATE_MOTOR_CALIBRATION, "motor callibration"},
    {AXIS_STATE_ENCODER_INDEX_SEARCH, "encoder index search"},
    {AXIS_STATE_ENCODER_OFFSET_CALIBRATION, "encoder offset callibration"},
    {AXIS_STATE_CLOSED_LOOP_CONTROL, "closed loop control"},
    {AXIS_STATE_LOCKIN_SPIN, "lockin spin"},
    {AXIS_STATE_ENCODER_DIR_FIND, "encoder dir find"},
    {AXIS_STATE_HOMING, "homing"},
    {AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION, "encoder polarity calibration"},
    {AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION, "encoder hall phase calibration"}};

std::map<uint8_t, std::string> control_mode_map = {
    {CONTROL_MODE_VOLTAGE_CONTROL, "voltage control"},
    {CONTROL_MODE_TORQUE_CONTROL, "torque control"},
    {CONTROL_MODE_VELOCITY_CONTROL, "velocity control"},
    {CONTROL_MODE_POSITION_CONTROL, "position control"}};

std::map<uint8_t, std::string> input_mode_map = {
    {INPUT_MODE_INACTIVE, "inactive"},
    {INPUT_MODE_PASSTHROUGH, "passthrough"},
    {INPUT_MODE_VEL_RAMP, "velocity ramp"},
    {INPUT_MODE_POS_FILTER, "position filter"},
    {INPUT_MODE_MIX_CHANNELS, "mix channels"},
    {INPUT_MODE_TRAP_TRAJ, "trap trajectory"},
    {INPUT_MODE_TORQUE_RAMP, "torque ramp"},
    {INPUT_MODE_MIRROR, "mirror"},
    {INPUT_MODE_TUNING, "tuning"}};

std::map<uint8_t, std::string> odrive_error_map = {
    {ODRIVE_ERROR_NONE, "none"},
    {ODRIVE_ERROR_CONTROL_ITERATION_MISSED, "control iteration missed"},
    {ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE, "DC bus undervoltage"},
    {ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE, "DC bus overvoltage"},
    {ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT, "DC bus over regen current"},
    {ODRIVE_ERROR_DC_BUS_OVER_CURRENT, "DC bus over current"},
    {ODRIVE_ERROR_BRAKE_DEADTIME_VIOLATION, "brake deadtime violation"},
    {ODRIVE_ERROR_BRAKE_DUTY_CYCLE_NAN, "breake duty cycle NaN"},
    {ODRIVE_ERROR_INVALID_BRAKE_RESISTANCE, "invalid brake resistance"}};

std::map<uint8_t, std::string> axis_error_map = {
    {AXIS_ERROR_NONE, "none"},
    {AXIS_ERROR_INVALID_STATE, "invalid state"},
    {AXIS_ERROR_MOTOR_FAILED, "motor failed"},
    {AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED, "sensorless estimator failed"},
    {AXIS_ERROR_ENCODER_FAILED, "encoder failed"},
    {AXIS_ERROR_CONTROLLER_FAILED, "controller failed"},
    {AXIS_ERROR_WATCHDOG_TIMER_EXPIRED, "watchdog timer expired"},
    {AXIS_ERROR_MIN_ENDSTOP_PRESSED, "min endstop pressed"},
    {AXIS_ERROR_MAX_ENDSTOP_PRESSED, "max endstop pressed"},
    {AXIS_ERROR_ESTOP_REQUESTED, "estop requested"},
    {AXIS_ERROR_HOMING_WITHOUT_ENDSTOP, "homing without endstop"},
    {AXIS_ERROR_OVER_TEMP, "over temp"},
    {AXIS_ERROR_UNKNOWN_POSITION, "unknown position"}};

void OdriveAxis::init()
{
    estop.tv_sec = -1;

    gain = std::make_unique<gains>();

    ver = std::make_unique<axisVersion>();
    ver->timestamp.tv_sec = -1;

    err = std::make_unique<axisErrors>();
    err->timestamp.tv_sec = -1;

    state = std::make_unique<axisState>();
    state->timestamp.tv_sec = -1;

    iq = std::make_unique<iqStruct>();
    iq->timestamp.tv_sec = -1;

    temp = std::make_unique<tempStruct>();
    temp->timestamp.tv_sec = -1;

    ui = std::make_unique<busUI>();
    ui->timestamp.tv_sec = -1;

    encoder = std::make_unique<encoderStruct>();
    encoder->timestamp.tv_sec = -1;

    adc = std::make_unique<adcVoltage>();
    adc->timestamp.tv_sec = -1;

    reg = std::make_unique<axisRegSettings>();
    reg->timestamp.tv_sec = -1;
}

void OdriveAxis::set_axis_version(uint8_t hw_version_major, uint8_t hw_version_minor,
                                  uint8_t hw_version_variant, uint8_t fw_version_major,
                                  uint8_t fw_version_minor, uint8_t fw_version_revision,
                                  struct timeval timestamp)
{

    this->ver->hw_version_major = hw_version_major;
    this->ver->hw_version_minor = hw_version_minor;
    this->ver->hw_version_variant = hw_version_variant;
    this->ver->fw_version_major = fw_version_major;
    this->ver->fw_version_minor = fw_version_minor;
    this->ver->fw_version_revision = fw_version_revision;
    this->ver->timestamp = timestamp;
}

void OdriveAxis::set_axis_state(uint8_t error, uint8_t state, uint8_t procedure_result,
                                bool trajectory_done_flag, struct timeval timestamp)
{
    this->err->axis_error = error;
    this->err->timestamp = timestamp;

    this->state->ax_state = state;
    this->state->trajectory_done_flag = trajectory_done_flag;
    this->state->procedure_result = procedure_result;
    this->state->timestamp = timestamp;
}

void OdriveAxis::update_error(uint32_t errors, uint32_t disarm_reason, struct timeval timestamp)
{
    this->err->active_errors = errors;
    this->err->disarm_reason = disarm_reason;
    this->err->timestamp = timestamp;
}

void OdriveAxis::update_estimates(uint32_t pos, uint32_t vel, struct timeval timestamp)
{
    this->encoder->pos_estimate = pos;
    this->encoder->vel_estimate = vel;
    this->encoder->timestamp = timestamp;
}

void OdriveAxis::update_iq(uint32_t setpoint, uint32_t measured, struct timeval timestamp)
{
    this->iq->iq_measured = measured;
    this->iq->iq_setpoint = setpoint;
    this->iq->timestamp = timestamp;
}

void OdriveAxis::update_temp(uint32_t fet, uint32_t motor, struct timeval timestamp)
{
    this->temp->fet_temperature = fet;
    this->temp->motor_temperature = motor;
    this->temp->timestamp = timestamp;
}

void OdriveAxis::update_ui(uint32_t voltage, uint32_t current, struct timeval timestamp)
{
    this->ui->bus_voltage = voltage;
    this->ui->bus_current = current;
    this->ui->timestamp = timestamp;
}

void OdriveAxis::update_adc(uint32_t voltage, struct timeval timestamp)
{
    this->adc->adc_voltage = voltage;
    this->adc->timestamp = timestamp;
}

void OdriveAxis::update_controller_err(uint32_t error, struct timeval timestamp)
{
    this->err->controller_error = error;
    this->err->timestamp = timestamp;
}

std::string italic(std::string s){
    return "\e[3m" + s + "\e[0m";
}

std::ostream &operator<<(std::ostream &out, OdriveAxis const *ax)
{
    // print Odrive version
    if (ax->ver->timestamp.tv_sec > -1)
    {
        out << "Odrive version [" << ax->ver->timestamp << "]:" << std::endl;
        out << "\tHardware version " << unsigned(ax->ver->hw_version_major) << "."
            << unsigned(ax->ver->hw_version_minor) << "." << (unsigned)ax->ver->hw_version_variant << std::endl;
        out << "\tFirmware version " << unsigned(ax->ver->fw_version_major) << "."
            << unsigned(ax->ver->fw_version_minor) << "." << (unsigned)ax->ver->fw_version_revision << std::endl;
    }
    // print Axis state
    out << "Axis status [" << ax->state->timestamp << "]:" << std::endl;
    if (ax->state->timestamp.tv_sec > -1)
    {
        out << "\tAxis state: " << axis_state_map[(unsigned)ax->state->ax_state] << std::endl;
        out << "\tProcedure result: " << (unsigned)ax->state->procedure_result << std::endl;
        out << "\tTrajectory done: " << (ax->state->trajectory_done_flag ? "yes" : "no") << std::endl;

        if (ax->err->timestamp.tv_sec > -1)
        {
            out << "Errors [" << ax->err->timestamp << "]:\n";
            out << "\tActive errors: " << odrive_error_map[ax->err->active_errors] << std::endl;
            out << "\tDisarm reason: " << (unsigned)ax->err->disarm_reason << std::endl;
            out << "\tAxis errors: " << axis_error_map[ax->err->axis_error] << std::endl;
            out << "\tController error: " << (unsigned)ax->err->controller_error << std::endl;
        }
    }

    // print Encoder estimates
    if (ax->encoder->timestamp.tv_sec > -1)
    {
        out << "Encoder estimates [" << ax->encoder->timestamp << "]:" << std::endl;
        out << "\tPosition estimate: " << (unsigned)ax->encoder->pos_estimate << std::endl;
        out << "\tVelocity estimate: " << (unsigned)ax->encoder->vel_estimate << std::endl;
    }

    // current temperature
    if (ax->temp->timestamp.tv_sec > -1)
    {
        out << "Temperature [" << ax->temp->timestamp << "]:" << std::endl;
        out << "\tFET temperature: " << (unsigned)ax->temp->fet_temperature << std::endl;
        out << "\tMotor temperature: " << (unsigned)ax->temp->motor_temperature << std::endl;
    }

    // current, voltage
    if (ax->ui->timestamp.tv_sec > -1)
    {
        out << "Bus voltage and current [" << ax->ui->timestamp << "]:" << std::endl;
        out << "\tBus current: " << (unsigned)ax->ui->bus_current << std::endl;
        out << "\tBus voltage: " << (unsigned)ax->ui->bus_voltage << std::endl;
    }

        if (ax->iq->timestamp.tv_sec > -1)
    {
        out << "Motor current [" << ax->iq->timestamp << "]:" << std::endl;
        out << "\tCommanded: " << (unsigned)ax->iq->iq_setpoint << std::endl;
        out << "\tMeasured: " << (unsigned)ax->iq->iq_measured << std::endl;
    }

    // ADC voltage
    if (ax->adc->timestamp.tv_sec > -1)
        out << "ADC voltage[" << ax->adc->timestamp << "]: " << (unsigned)ax->adc->adc_voltage << std::endl;

    // Regulator settings
    if (ax->reg->timestamp.tv_sec > -1)
    {
        out << "Regulator settings [" << ax->reg->timestamp << "]: " << std::endl;
        out << "\tController mode: *"<< italic("Control mode: ") << control_mode_map[ax->reg->control_mode] << ", *"<< italic("Input mode: ")
            << input_mode_map[ax->reg->input_mode] << std::endl;
        out << "\tLimits: *" << italic("Velocity: ") << (unsigned)ax->reg->velocity_limit << ", *" << italic("Current: ")
            << (unsigned)ax->reg->current_limit << std::endl;
        out << "\tAnticogging is  " << (ax->reg->anticogging_timestamp.tv_sec > 0 ? "activated" : "deactivated") << std::endl;
        out << "\tInput position: *" << italic("Input position: ") << (unsigned)ax->reg->input_pos << ", *" << italic("Vel FF: ")
            << ax->reg->vel_ff << " *" << italic("Torque FF: ") << ax->reg->torque_ff << std::endl;
        out << "\tInput velocity: *" << italic("Input vel: ") << (unsigned)ax->reg->input_vel << ", *" << italic("Input torque FF: ")
            << (unsigned)ax->reg->input_torque_ff << std::endl;
        out << "\tInput torque: " << (unsigned)ax->reg->input_torque << std::endl;
        out << "\tTrajectory limits: *"<<italic("Velocity: ") << (unsigned)ax->reg->traj_vel_limit << ", *" << italic("Acceleration: ")
            << (unsigned)ax->reg->traj_accel_limit << ", *" << italic("Deceleration: ") << (unsigned)ax->reg->traj_decel_limit << std::endl;
        out << "\tTrajectory inertia: " << (unsigned)ax->reg->traj_inertia << std::endl;
    }

    if (ax->estop.tv_sec > 0)
        out << "Estop was called [" << ax->estop << "]: " << std::endl;

    return out;
}

void OdriveAxis::update_axis_requested_state(uint32_t state)
{
    this->axis_requested_state = state;
};

uint32_t OdriveAxis::get_axis_requested_state()
{
    return this->axis_requested_state;
};

void OdriveAxis::update_controller_mode(uint32_t control_mode, uint32_t input_mode)
{
    this->reg->control_mode = control_mode;
    this->reg->input_mode = input_mode;
};

uint32_t OdriveAxis::get_controll_mode()
{
    return this->reg->control_mode;
};

uint32_t OdriveAxis::get_input_mode()
{
    return this->reg->input_mode;
};

void OdriveAxis::update_input_pos(uint32_t input_pos, float vel_ff, float torque_ff)
{
    this->reg->input_pos = input_pos;
    this->reg->vel_ff = vel_ff;
    this->reg->torque_ff = torque_ff;
};

void OdriveAxis::update_input_vel(uint32_t input_vel, uint32_t input_torque_ff)
{
    this->reg->input_vel = input_vel;
    this->reg->input_torque_ff = input_torque_ff;
}

void OdriveAxis::update_input_torque(uint32_t input_torque)
{
    this->reg->input_torque = input_torque;
}

void OdriveAxis::update_limits(uint32_t velocity, uint32_t current)
{
    this->reg->velocity_limit = velocity;
    this->reg->current_limit = current;
}

void OdriveAxis::update_traj_vel_limit(uint32_t lim)
{
    this->reg->traj_vel_limit = lim;
}

void OdriveAxis::update_traj_accel_limit(uint32_t accel, uint32_t decel)
{
    this->reg->traj_accel_limit = accel;
    this->reg->traj_decel_limit = decel;
}

void OdriveAxis::update_traj_inertia(uint32_t inertia)
{
    this->reg->traj_inertia = inertia;
}

void OdriveAxis::update_absolut_pos(uint32_t pos)
{
    this->position = pos;
}

void OdriveAxis::update_pos_gain(uint32_t gain)
{
    this->gain->pos_gain = gain;
}

void OdriveAxis::update_vel_gains(uint32_t gain, uint32_t integrator_gain)
{
    this->gain->vel_gain = gain;
    this->gain->vel_integrator_gain = integrator_gain;
}

void OdriveAxis::update_estop(struct timeval b)
{
    this->estop = b;
}

void OdriveAxis::update_anticogging(struct timeval b)
{
    this->reg->anticogging_timestamp = b;
}

void OdriveAxis::update_reboot_timestamp(struct timeval stamp)
{
    this->reboot_timestamp = stamp;
}

int OdriveAxis::get_axis_id()
{
    return this->id;
}

void OdriveAxis::update_controller_timestamp(struct timeval t){
    this->reg->timestamp = t;
}
