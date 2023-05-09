/**
 * @file odriveaxis.cpp
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Structure for fetching and storing odrive settings info
 * @version 0.1
 * @date 2023-03-21
 *
 # @copyright (c) JettyVision s.r.o in Prague 2023 - All Rights Reserved
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

std::string italic(std::string s){
    return "\e[3m" + s +  "\e[0m";
}

std::ostream& operator<<(std::ostream &out, const OdriveAxis& odrive)
{
    // print Odrive version
    if (odrive.ver.timestamp.tv_sec > -1)
    {
        out << "Odrive version [" << odrive.ver.timestamp << "]:" << std::endl;
        out << "\tHardware version " << unsigned(odrive.ver.hw_version_major) << "."
            << unsigned(odrive.ver.hw_version_minor) << "." << (unsigned)odrive.ver.hw_version_variant << std::endl;
        out << "\tFirmware version " << unsigned(odrive.ver.fw_version_major) << "."
            << unsigned(odrive.ver.fw_version_minor) << "." << (unsigned)odrive.ver.fw_version_revision << std::endl;
    }
    // print Axis state
     
    if (odrive.state.timestamp.tv_sec > -1)
    {
        out << "Axis status [" << odrive.state.timestamp << "]:" << std::endl;
        out << "\tAxis state: " << axis_state_map[(unsigned)odrive.state.ax_state] << std::endl;
        out << "\tProcedure result: " << (unsigned)odrive.state.procedure_result << std::endl;
        out << "\tTrajectory done: " << (odrive.state.trajectory_done_flag ? "yes" : "no") << std::endl;

        if (odrive.err.timestamp.tv_sec > -1)
        {
            out << "Errors [" << odrive.err.timestamp << "]:\n";
            out << "\tActive errors: " << odrive_error_map[odrive.err.active_errors] << std::endl;
            out << "\tDisarm reason: " << (unsigned)odrive.err.disarm_reason << std::endl;
            out << "\tAxis errors: " << axis_error_map[odrive.err.axis_error] << std::endl;
            out << "\tController error: " << (unsigned)odrive.err.controller_error << std::endl;
        }
    } else{
        out << "Axis status : CAN INTERFACE INACTIVE" << std::endl;
    }

    // print Encoder estimates
    if (odrive.encoder.timestamp.tv_sec > -1)
    {
        out << "Encoder estimates [" << odrive.encoder.timestamp << "]:" << std::endl;
        out << "\tPosition estimate: " << odrive.encoder.pos_estimate << std::endl;
        out << "\tVelocity estimate: " << odrive.encoder.vel_estimate << std::endl;
    }

    // current temperature
    if (odrive.temp.timestamp.tv_sec > -1)
    {
        out << "Temperature [" << odrive.temp.timestamp << "]:" << std::endl;
        out << "\tFET temperature: " << odrive.temp.fet_temperature << std::endl;
        out << "\tMotor temperature: " << odrive.temp.motor_temperature << std::endl;
    }

    // current, voltage
    if (odrive.ui.timestamp.tv_sec > -1)
    {
        out << "Bus voltage and current [" << odrive.ui.timestamp << "]:" << std::endl;
        out << "\tBus current: " << odrive.ui.bus_current << std::endl;
        out << "\tBus voltage: " << odrive.ui.bus_voltage << std::endl;
    }

    if (odrive.iq.timestamp.tv_sec > -1)
    {
        out << "Motor current [" << odrive.iq.timestamp << "]:" << std::endl;
        out << "\tCommanded: " << odrive.iq.iq_setpoint << std::endl;
        out << "\tMeasured: " << odrive.iq.iq_measured << std::endl;
    }

    // ADC voltage
    if (odrive.adc.timestamp.tv_sec > -1)
        out << "ADC voltage[" << odrive.adc.timestamp << "]: " << odrive.adc.adc_voltage << std::endl;

    // Regulator settings
    out << "Absolute position: " << odrive.position << std::endl;
    if (odrive.reg.timestamp.tv_sec > -1)
    {
        out << "Regulator settings [" << odrive.reg.timestamp << "]: " << std::endl;
        out << "\tController mode: \t*"<< italic("Control mode: ") << control_mode_map[odrive.reg.control_mode] << ", \t*"<< italic("Input mode: ")
            << input_mode_map[odrive.reg.input_mode] << std::endl;
        out << "\tLimits: \t\t*" << italic("Velocity: ") << odrive.reg.velocity_limit << ", \t*" << italic("Current: ")
            << odrive.reg.current_limit << std::endl;
        out << "\tInput position: \t*" << italic("Position: ") << odrive.reg.input_pos << ", \t*" << italic("Vel FF: ")
            << odrive.reg.vel_ff << " \t\t*" << italic("Torque FF: ") << odrive.reg.torque_ff << std::endl;
        out << "\tInput velocity: \t*" << italic("Input vel: ") << odrive.reg.input_vel << ", \t*" << italic("Input torque FF: ")
            << odrive.reg.input_torque_ff << std::endl;
        out << "\tTrajectory limits: \t*"<<italic("Velocity: ") << odrive.reg.traj_vel_limit << ", \t*" << italic("Acceleration: ")
            << odrive.reg.traj_accel_limit << ", \t*" << italic("Deceleration: ") << odrive.reg.traj_decel_limit << std::endl;
        out << "\tTrajectory inertia: " << odrive.reg.traj_inertia << std::endl;
        out << "\tInput torque: " << odrive.reg.input_torque << std::endl;
        out << "\tAnticogging is " << (odrive.reg.anticogging_timestamp.tv_sec > 0 ? "activated" : "deactivated") << std::endl;
    }

    if (odrive.estop.tv_sec > 0)
        out << "Estop was called [" << odrive.estop << "] " << std::endl;

    if (odrive.reboot_timestamp.tv_sec > -1)
        out << "Odrive was rebooted [" << odrive.reboot_timestamp << "]" << std::endl;


    return out;
}
