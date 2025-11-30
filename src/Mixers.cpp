#include "Mixers.h"
#include <algorithm>


float mixWing(std::array<float, 3>& motorOutputs, const MotorMixerBase::commands_t& commands)
{
    enum { THROTTLE, LEFT_FLAPERON, RIGHT_FLAPERON };

    motorOutputs[THROTTLE]       =  commands.throttle;
    motorOutputs[LEFT_FLAPERON]  =  commands.roll + commands.pitch;
    motorOutputs[RIGHT_FLAPERON] = -commands.roll + commands.pitch;

    return commands.throttle;
}

float mixAirplane(std::array<float, 5>& motorOutputs, const MotorMixerBase::commands_t& commands)
{
    enum { THROTTLE, LEFT_AILERON, RIGHT_AILERON, ELEVATOR, RUDDER };

    motorOutputs[THROTTLE]      =  commands.throttle;
    motorOutputs[LEFT_AILERON]  =  commands.roll;
    motorOutputs[RIGHT_AILERON] = -commands.roll;
    motorOutputs[ELEVATOR]      =  commands.pitch;
    motorOutputs[RUDDER]        =  commands.yaw;

    return commands.throttle;
}

float mixBicopter(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin)
{
    (void)motorOutputMin;
    enum { MOTOR_LEFT, MOTOR_RIGHT, SERVO_LEFT, SERVO_RIGHT };

    const float throttle = commands.throttle;
    motorOutputs[MOTOR_LEFT]   =  throttle + commands.roll;
    motorOutputs[MOTOR_RIGHT]  =  throttle - commands.roll;
    motorOutputs[SERVO_LEFT]   =  commands.pitch - commands.yaw;
    motorOutputs[SERVO_RIGHT]  =  commands.pitch + commands.yaw;

    return throttle;
}

float mixTricopter(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float maxServoAngleRadians, float motorOutputMin)
{
    float undershoot = 0.0F;
    float overshoot = 0.0F;
    return mixTricopter(motorOutputs, commands, maxServoAngleRadians, motorOutputMin, undershoot, overshoot);
}

float mixTricopter(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float maxServoAngleRadians, float motorOutputMin, float& undershoot, float& overshoot)
{
    enum { FL = 0, FR = 1, REAR = 2, S0 = 3};
    constexpr float TWO_THIRDS = 2.0F / 3.0F;
    constexpr float FOUR_THIRDS = 4.0F / 3.0F;

    const float throttle = commands.throttle;
    motorOutputs[FL]   = throttle + commands.roll - TWO_THIRDS*commands.pitch;
    motorOutputs[FR]   = throttle - commands.roll - TWO_THIRDS*commands.pitch;
    motorOutputs[REAR] = (throttle + FOUR_THIRDS*commands.pitch)/cosf(commands.yaw*maxServoAngleRadians);

    constexpr float motorOutputMax = 1.0F;
    // check for rear overshoot, front motors unlikely to overshoot since there are two of them and there is no yaw-related attenuation
    overshoot = motorOutputs[REAR] - motorOutputMax;
    if (overshoot > 0.0F) {
        // rear motor is saturated, so reduce its output to motorOutputMax and reduce front motors similarly
        motorOutputs[REAR] = motorOutputMax;
        motorOutputs[FL] = std::max(motorOutputMin, motorOutputs[FL] - overshoot);
        motorOutputs[FR] = std::max(motorOutputMin, motorOutputs[FR] - overshoot);
    }

    // check for front undershoot
    undershoot = motorOutputMin - std::min(motorOutputs[FL], motorOutputs[FR]);
    if (undershoot > 0.0F) {
        motorOutputs[REAR] = motorOutputMin;
        motorOutputs[FL] = std::min(motorOutputMax, motorOutputs[FL] + undershoot);
        motorOutputs[FR] = std::min(motorOutputMax, motorOutputs[FR] + undershoot);
    }

    motorOutputs[S0] = commands.yaw;

    return throttle;
}

/*!
Calculate the "mix" for the QuadX motor configuration.
Motor rotation is "propellers out".
Assumes motors are on.
Motor numbering is:
4 2
3 1
*/
float mixQuadX(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin)
{
    float undershoot = 0.0F;
    float overshoot = 0.0F;
    return mixQuadX(motorOutputs, commands, motorOutputMin, undershoot, overshoot);
}

float mixQuadX(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin, float& undershoot, float& overshoot) // NOLINT(readability-function-cognitive-complexity)
{
    enum { MOTOR_COUNT = 4 };
    enum { BACK_RIGHT = 0, FRONT_RIGHT = 1, BACK_LEFT = 2, FRONT_LEFT = 3 };

    // calculate the motor outputs without yaw applied
    float throttle = commands.throttle;
    motorOutputs[BACK_RIGHT]  = throttle - commands.roll + commands.pitch;// + commands.yaw;
    motorOutputs[FRONT_RIGHT] = throttle - commands.roll - commands.pitch;// - commands.yaw;
    motorOutputs[BACK_LEFT]   = throttle + commands.roll + commands.pitch;// - commands.yaw;
    motorOutputs[FRONT_LEFT]  = throttle + commands.roll - commands.pitch;// + commands.yaw;

    // deal with yaw undershoot and overshoot
    // High values of yaw can cause motor outputs to go outside range [motorOutputMin, motorOutputMax]
    // If this happens, we reduce the magnitude of the yaw command.
    // This reduces yaw authority, but avoids "yaw jumps"
    static constexpr float motorOutputMax = 1.0F;
    overshoot = 0.0F;
    undershoot = 0.0F;
    float commandYaw = commands.yaw;
    if (commands.yaw > 0.0F) {
        // check if M1 or M2 will have output less than motorOutputMin
        const float m1 = motorOutputs[1] - commands.yaw;
        if (m1 < motorOutputMin) {
            undershoot = m1 - motorOutputMin;
        }
        const float m2 = motorOutputs[2] - commands.yaw;
        if (m2 < motorOutputMin) {
            undershoot = std::min(undershoot, m2 - motorOutputMin);
        }
        // check if M0 or M3 will have output greater than motorOutputMax
        const float m0 = motorOutputs[0] + commands.yaw;
        if (m0 > motorOutputMax) {
            overshoot = m0 - motorOutputMax;
        }
        const float m3 = motorOutputs[3] + commands.yaw;
        if (m3 > motorOutputMax) {
            overshoot = std::max(overshoot, m3 - motorOutputMax);
        }
        if (commandYaw + (undershoot - overshoot) > 0.0F) {
            commandYaw += (undershoot - overshoot);
            throttle -= (undershoot + overshoot);
        }
    } else {
        // check if M0 or M3 will have output less than motorOutputMin
        const float m0 = motorOutputs[0] + commands.yaw;
        if (m0 < motorOutputMin) {
            undershoot = m0 - motorOutputMin;
        }
        const float m3 = motorOutputs[3] + commands.yaw;
        if (m3 < motorOutputMin) {
            undershoot = std::min(undershoot, m3 - motorOutputMin);
        }
        // check if M1 or M2 will have output greater than motorOutputMax
        const float m1 = motorOutputs[1] - commands.yaw;
        if (m1 > motorOutputMax) {
            overshoot = m1 - motorOutputMax;
        }
        const float m2 = motorOutputs[2] - commands.yaw;
        if (m2 > motorOutputMax) {
            overshoot = std::max(overshoot, m2 - motorOutputMax);
        }
        if (commandYaw - (undershoot - overshoot) < 0.0F) {
            commandYaw -= (undershoot - overshoot);
            throttle -= (undershoot + overshoot);
        }
    }

    motorOutputs[BACK_RIGHT]  += commandYaw;
    motorOutputs[FRONT_RIGHT] -= commandYaw;
    motorOutputs[BACK_LEFT]   -= commandYaw;
    motorOutputs[FRONT_LEFT]  += commandYaw;

    float maxOutput = motorOutputs[0];
    float output = motorOutputs[1];
    if (output > maxOutput) { maxOutput = output; }
    output = motorOutputs[2];
    if (output > maxOutput) { maxOutput = output; }
    output = motorOutputs[3];
    if (output > maxOutput) { maxOutput = output; }

    if (maxOutput > motorOutputMax) {
        const float correction = maxOutput - motorOutputMax;
        throttle -= correction;
        for (auto& motorOutput : motorOutputs) {
            motorOutput -= correction; // cppcheck-suppress useStlAlgorithm
        }
    }

    return throttle;
}

/*!
Calculate the "mix" for the HexX motor configuration.
Motor rotation is "propellers out".
Assumes motors are on.
*/
float mixHexX(std::array<float, 6>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin) // NOLINT(readability-function-cognitive-complexity)
{
    (void)motorOutputMin;

    enum { MOTOR_COUNT = 6 };

    // calculate the motor outputs without yaw applied
    float throttle = commands.throttle;
    static constexpr float sin30 = 0.5F;
    static constexpr float sin60 = 0.86602540378F;
    motorOutputs[0] = throttle - sin30*commands.roll + sin60*commands.pitch;// + commands.yaw; // back right
    motorOutputs[1] = throttle - sin30*commands.roll - sin60*commands.pitch;// - commands.yaw; // front right
    motorOutputs[2] = throttle + sin30*commands.roll + sin60*commands.pitch;// - commands.yaw; // back left
    motorOutputs[3] = throttle + sin30*commands.roll - sin60*commands.pitch;// + commands.yaw; // front left
    motorOutputs[4] = throttle -       commands.roll;//                        + commands.yaw; // center right
    motorOutputs[5] = throttle +       commands.roll;//                        - commands.yaw; // center left

    // deal with yaw undershoot and overshoot
    // High values of yaw can cause motor outputs to go outside range [motorOutputMin, motorOutputMax]
    // If this happens, we reduce the magnitude of the yaw command.
    // This reduces yaw authority, but avoids "yaw jumps"
    static constexpr float motorOutputMax = 1.0F;
    float overshoot = 0.0F;
    float undershoot = 0.0F;
    float commandYaw = commands.yaw;
    if (commands.yaw > 0.0F) {
        // check if M1, M2, or M5 will have output less than motorOutputMin
        const float m1 = motorOutputs[1] - commands.yaw;
        if (m1 < motorOutputMin) {
            undershoot = m1 - motorOutputMin;
        }
        const float m2 = motorOutputs[2] - commands.yaw;
        if (m2 < motorOutputMin) {
            undershoot = std::min(undershoot, m2 - motorOutputMin);
        }
        const float m5 = motorOutputs[5] - commands.yaw;
        if (m5 < motorOutputMin) {
            undershoot = std::min(undershoot, m5 - motorOutputMin);
        }
        // check if M0, M3, or M4 will have output greater than motorOutputMax
        const float m0 = motorOutputs[0] + commands.yaw;
        if (m0 > motorOutputMax) {
            overshoot = m0 - motorOutputMax;
        }
        const float m3 = motorOutputs[3] + commands.yaw;
        if (m3 > motorOutputMax) {
            overshoot = std::max(overshoot, m3 - motorOutputMax);
        }
        const float m4 = motorOutputs[4] + commands.yaw;
        if (m4 > motorOutputMax) {
            overshoot = std::max(overshoot, m4 - motorOutputMax);
        }
        if (commandYaw + (undershoot - overshoot) > 0.0F) {
            commandYaw += (undershoot - overshoot);
            throttle -= (undershoot + overshoot);
        }
    } else {
        // check if M0, M3, or M4 will have output less than motorOutputMin
        const float m0 = motorOutputs[0] + commands.yaw;
        if (m0 < motorOutputMin) {
            undershoot = m0 - motorOutputMin;
        }
        const float m3 = motorOutputs[3] + commands.yaw;
        if (m3 < motorOutputMin) {
            undershoot = std::min(undershoot, m3 - motorOutputMin);
        }
        const float m4 = motorOutputs[4] + commands.yaw;
        if (m4 < motorOutputMin) {
            undershoot = std::min(undershoot, m4 - motorOutputMin);
        }
        // check if M1, M2, or M5 will have output greater than motorOutputMax
        const float m1 = motorOutputs[1] - commands.yaw;
        if (m1 > motorOutputMax) {
            overshoot = m1 - motorOutputMax;
        }
        const float m2 = motorOutputs[2] - commands.yaw;
        if (m2 > motorOutputMax) {
            overshoot = std::max(overshoot, m2 - motorOutputMax);
        }
        const float m5 = motorOutputs[5] - commands.yaw;
        if (m5 > motorOutputMax) {
            overshoot = std::max(overshoot, m5 - motorOutputMax);
        }
        if (commandYaw - (undershoot - overshoot) < 0.0F) {
            commandYaw -= (undershoot - overshoot);
            throttle -= (undershoot + overshoot);
        }
    }

    // adjust commands.yaw to remove undershoot and overshoot
    motorOutputs[0] += commandYaw;
    motorOutputs[1] -= commandYaw;
    motorOutputs[2] -= commandYaw;
    motorOutputs[3] += commandYaw;
    motorOutputs[4] += commandYaw;
    motorOutputs[5] -= commandYaw;

    const float maxOutput = *std::max_element(motorOutputs.begin(), motorOutputs.end());

    if (maxOutput > motorOutputMax) {
        const float correction = maxOutput - motorOutputMax;
        throttle -= correction;
        for (auto& motorOutput : motorOutputs) {
            motorOutput -= correction; // cppcheck-suppress useStlAlgorithm
        }
    }

    return throttle;
}

/*!
Calculate the "mix" for the OctoX motor configuration.
Motor rotation is "propellers out".
Assumes motors are on.
*/
float mixOctoX(std::array<float, 8>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin) // NOLINT(readability-function-cognitive-complexity)
{
    (void)motorOutputMin;

    enum { MOTOR_COUNT = 8 };

    float throttle = commands.throttle;

    motorOutputs[0] = throttle - commands.roll + commands.pitch;// + commands.yaw; // back right
    motorOutputs[1] = throttle - commands.roll - commands.pitch;// - commands.yaw; // front right
    motorOutputs[2] = throttle + commands.roll + commands.pitch;// - commands.yaw; // back left
    motorOutputs[3] = throttle + commands.roll - commands.pitch;// + commands.yaw; // front left

    motorOutputs[4] = throttle - commands.roll + commands.pitch;// - commands.yaw; // under back right
    motorOutputs[5] = throttle - commands.roll - commands.pitch;// + commands.yaw; // under front right
    motorOutputs[6] = throttle + commands.roll + commands.pitch;// + commands.yaw; // under back left
    motorOutputs[7] = throttle + commands.roll - commands.pitch;// - commands.yaw; // under front left

    static constexpr float motorOutputMax = 1.0F;
    float overshoot = 0.0F;
    float undershoot = 0.0F;
    float commandYaw = commands.yaw;
    if (commands.yaw > 0.0F) {
        // check if M1, M2, M4, or M7 will have output less than motorOutputMin
        const float m1 = motorOutputs[1] - commands.yaw;
        if (m1 < motorOutputMin) {
            undershoot = m1 - motorOutputMin;
        }
        const float m2 = motorOutputs[2] - commands.yaw;
        if (m2 < motorOutputMin) {
            undershoot = std::min(undershoot, m2 - motorOutputMin);
        }
        const float m4 = motorOutputs[4] - commands.yaw;
        if (m4 < motorOutputMin) {
            undershoot = std::min(undershoot, m4 - motorOutputMin);
        }
        const float m7 = motorOutputs[7] - commands.yaw;
        if (m7 < motorOutputMin) {
            undershoot = std::min(undershoot, m7 - motorOutputMin);
        }
        // check if M0, M3, M5, or M6 will have output greater than motorOutputMax
        const float m0 = motorOutputs[0] + commands.yaw;
        if (m0 > motorOutputMax) {
            overshoot = m0 - motorOutputMax;
        }
        const float m3 = motorOutputs[3] + commands.yaw;
        if (m3 > motorOutputMax) {
            overshoot = std::max(overshoot, m3 - motorOutputMax);
        }
        const float m5 = motorOutputs[5] + commands.yaw;
        if (m5 > motorOutputMax) {
            overshoot = std::max(overshoot, m5 - motorOutputMax);
        }
        const float m6 = motorOutputs[6] + commands.yaw;
        if (m6 > motorOutputMax) {
            overshoot = std::max(overshoot, m6 - motorOutputMax);
        }
        if (commandYaw + (undershoot - overshoot) > 0.0F) {
            commandYaw += (undershoot - overshoot);
            throttle -= (undershoot + overshoot);
        }
    } else {
        // check if M0, M3, M5, or M6  will have output less than motorOutputMin
        const float m0 = motorOutputs[0] + commands.yaw;
        if (m0 < motorOutputMin) {
            undershoot = m0 - motorOutputMin;
        }
        const float m3 = motorOutputs[3] + commands.yaw;
        if (m3 < motorOutputMin) {
            undershoot = std::min(undershoot, m3 - motorOutputMin);
        }
        const float m5 = motorOutputs[5] + commands.yaw;
        if (m5 < motorOutputMin) {
            undershoot = std::min(undershoot, m5 - motorOutputMin);
        }
        const float m6 = motorOutputs[6] + commands.yaw;
        if (m6 < motorOutputMin) {
            undershoot = std::min(undershoot, m6 - motorOutputMin);
        }
        // check if M1, M2, M4, or M7 will have output greater than motorOutputMax
        const float m1 = motorOutputs[1] - commands.yaw;
        if (m1 > motorOutputMax) {
            overshoot = m1 - motorOutputMax;
        }
        const float m2 = motorOutputs[2] - commands.yaw;
        if (m2 > motorOutputMax) {
            overshoot = std::max(overshoot, m2 - motorOutputMax);
        }
        const float m4 = motorOutputs[4] - commands.yaw;
        if (m4 > motorOutputMax) {
            overshoot = std::max(overshoot, m4 - motorOutputMax);
        }
        const float m7 = motorOutputs[7] - commands.yaw;
        if (m7 > motorOutputMax) {
            overshoot = std::max(overshoot, m7 - motorOutputMax);
        }
        if (commandYaw - (undershoot - overshoot) < 0.0F) {
            commandYaw -= (undershoot - overshoot);
            throttle -= (undershoot + overshoot);
        }
    }

    motorOutputs[0] += commandYaw;
    motorOutputs[1] -= commandYaw;
    motorOutputs[2] -= commandYaw;
    motorOutputs[3] += commandYaw;
    motorOutputs[4] -= commandYaw;
    motorOutputs[5] += commandYaw;
    motorOutputs[6] += commandYaw;
    motorOutputs[7] -= commandYaw;

    const float maxOutput = *std::max_element(motorOutputs.begin(), motorOutputs.end());

    if (maxOutput > motorOutputMax) {
        const float correction = maxOutput - motorOutputMax;
        throttle -= correction;
        for (auto& motorOutput : motorOutputs) {
            motorOutput -= correction; // cppcheck-suppress useStlAlgorithm
        }
    }

    return throttle;
}
