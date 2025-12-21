#pragma once

#include "MotorMixerBase.h"
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

float mixQuadX(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, MotorMixerBase::parameters_t& params);
float mixHexX (std::array<float, 6>& motorOutputs, const MotorMixerBase::commands_t& commands, MotorMixerBase::parameters_t& params);
float mixOctoQuadX(std::array<float, 8>& motorOutputs, const MotorMixerBase::commands_t& commands, MotorMixerBase::parameters_t& params);

float mixBicopter(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, MotorMixerBase::parameters_t& params);

float mixTricopter(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, MotorMixerBase::parameters_t& params);

float mixWing(std::array<float, 3>& motorOutputs, const MotorMixerBase::commands_t& commands, MotorMixerBase::parameters_t& params);
float mixAirplane(std::array<float, 5>& motorOutputs, const MotorMixerBase::commands_t& commands, MotorMixerBase::parameters_t& params);
