#pragma once

#include "motor_mixer_base.h"
#include <algorithm>
#include <array>

inline float clamp(float value, float min, float max)
{
#if (__cplusplus >= 202002L)
    return std::clamp(value, min, max);
#else
    return (value < min) ? min : (value > max) ? max : value;
#endif
}

float mix_quad_x(std::array<float, 4>& motor_outputs, const motor_mixer_commands_t& commands, motor_mixer_parameters_t& params);
float mix_hex_x (std::array<float, 6>& motor_outputs, const motor_mixer_commands_t& commands, motor_mixer_parameters_t& params);
float mix_octo_quad_x(std::array<float, 8>& motor_outputs, const motor_mixer_commands_t& commands, motor_mixer_parameters_t& params);

float mix_bicopter(std::array<float, 4>& motor_outputs, const motor_mixer_commands_t& commands, motor_mixer_parameters_t& params);

float mix_tricopter(std::array<float, 4>& motor_outputs, const motor_mixer_commands_t& commands, motor_mixer_parameters_t& params);

float mix_wing(std::array<float, 3>& motor_outputs, const motor_mixer_commands_t& commands, motor_mixer_parameters_t& params);
float mix_airplane(std::array<float, 5>& motor_outputs, const motor_mixer_commands_t& commands, motor_mixer_parameters_t& params);
