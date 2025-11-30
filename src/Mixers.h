#pragma once

#include "MotorMixerBase.h"
#include <array>

float mixQuadX(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin);
// variant with `undershoot` and `overshoot` parameters added for test code
float mixQuadX(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin, float& undershoot, float& overshoot); // NOLINT(readability-redundant-declaration)
float mixHexX (std::array<float, 6>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin);
float mixOctoX(std::array<float, 8>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin);

float mixBicopter(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin);

float mixTricopter(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float maxServoAngleRadians, float motorOutputMin);
// variant with `undershoot` and `overshoot` parameters added for test code
float mixTricopter(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float maxServoAngleRadians, float motorOutputMin, float& undershoot, float& overshoot); // NOLINT(readability-redundant-declaration)

float mixWing(std::array<float, 3>& motorOutputs, const MotorMixerBase::commands_t& commands);
float mixAirplane(std::array<float, 5>& motorOutputs, const MotorMixerBase::commands_t& commands);
