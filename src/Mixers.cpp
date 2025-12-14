#include "Mixers.h"
#include <algorithm>


float mixWing(std::array<float, 3>& motorOutputs, const MotorMixerBase::commands_t& commands, MotorMixerBase::parameters_t& params)
{
    (void)params;
    enum { THROTTLE, LEFT_FLAPERON, RIGHT_FLAPERON };

    motorOutputs[THROTTLE]       =  commands.throttle; // throttle may be controlled by a servo for a wing with an internal combustion engine
    motorOutputs[LEFT_FLAPERON]  =  commands.roll + commands.pitch;
    motorOutputs[RIGHT_FLAPERON] = -commands.roll + commands.pitch;

    return commands.throttle;
}

float mixAirplane(std::array<float, 5>& motorOutputs, const MotorMixerBase::commands_t& commands, MotorMixerBase::parameters_t& params)
{
    (void)params;
    enum { THROTTLE, LEFT_AILERON, RIGHT_AILERON, ELEVATOR, RUDDER };

    motorOutputs[THROTTLE]      =  commands.throttle; // throttle may be controlled by a servo for a wing with an internal combustion engine
    motorOutputs[LEFT_AILERON]  =  commands.roll;
    motorOutputs[RIGHT_AILERON] = -commands.roll;
    motorOutputs[ELEVATOR]      =  commands.pitch;
    motorOutputs[RUDDER]        =  commands.yaw;

    return commands.throttle;
}

float mixBicopter(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, MotorMixerBase::parameters_t& params)
{
    (void)params;
    enum { MOTOR_LEFT, MOTOR_RIGHT, SERVO_LEFT, SERVO_RIGHT };

    const float throttle = commands.throttle;
    motorOutputs[MOTOR_LEFT]   =  throttle + commands.roll;
    motorOutputs[MOTOR_RIGHT]  =  throttle - commands.roll;
    motorOutputs[SERVO_LEFT]   =  commands.pitch - commands.yaw;
    motorOutputs[SERVO_RIGHT]  =  commands.pitch + commands.yaw;

    return throttle;
}

/*!
Motor numbering is the same as Betaflight

CW = clockwise
CC = counter clockwise


    front
  vCC^   ^CWv
    3     2
     \   /
      |Y|
       |
       1
      vCW^


"Mix" calculation
                                        m
Roll right              (left+  right-)  0-+
Pitch up (stick back)   (front+ back-)   -++
Yaw clockwise           (CC+    CW-)     --+

For coordinated flight the aircraft's nose is aligned with the direction of the turn, ie we yaw right when we roll right,
so we want the front right motor to turn clockwise.
*/
float mixTricopter(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, MotorMixerBase::parameters_t& params)
{
    enum { REAR = 0, FR = 1, FL = 2, S0 = 3};
    constexpr float TWO_THIRDS = 2.0F / 3.0F;
    constexpr float FOUR_THIRDS = 4.0F / 3.0F;

    const float throttle = commands.throttle;
    const float pivotAngleRadians = commands.yaw*params.maxServoAngleRadians;
    motorOutputs[REAR] = (throttle                 - FOUR_THIRDS * commands.pitch) / cosf(pivotAngleRadians);
    motorOutputs[FR]   =  throttle - commands.roll + TWO_THIRDS  * commands.pitch;
    motorOutputs[FL]   =  throttle + commands.roll + TWO_THIRDS  * commands.pitch;
    motorOutputs[S0]   = commands.yaw;


#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING)
    // check for rear params.overshoot, front motors unlikely to params.overshoot since there are two of them and there is no yaw-related attenuation
    params.overshoot = motorOutputs[REAR] - params.motorOutputMax;
    if (params.overshoot > 0.0F) {
        // rear motor is saturated, so reduce its output to params.motorOutputMax and reduce front motors similarly
        motorOutputs[REAR] = params.motorOutputMax;
        motorOutputs[FR] = std::max(params.motorOutputMin, motorOutputs[FR] - params.overshoot);
        motorOutputs[FL] = std::max(params.motorOutputMin, motorOutputs[FL] - params.overshoot);
    }

    // check for front params.undershoot
    params.undershoot = params.motorOutputMin - std::min(motorOutputs[FL], motorOutputs[FR]);
    if (params.undershoot > 0.0F) {
        motorOutputs[REAR] = params.motorOutputMin;
        motorOutputs[FR] = std::min(params.motorOutputMax, motorOutputs[FR] + params.undershoot);
        motorOutputs[FL] = std::min(params.motorOutputMax, motorOutputs[FL] + params.undershoot);
    }
#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING)

    return throttle;
}

/*!
Motor numbering is the same as Betaflight.
Motor rotation is "propellers out" (ie Betaflight "yaw reversed").

CW = clockwise
CC = counter clockwise


       front
 vCC^ 4     2 ^CWv
       \   /
        |X|
       /   \
 ^CWv 3     1 vCC^
    

"Mix" calculation
                                        m
Roll right              (left+  right-)  --++
Pitch up (stick back)   (front+ back-)   -+-+
Yaw clockwise           (CC+    CW-)     +--+
*/
float mixQuadX(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, MotorMixerBase::parameters_t& params) // NOLINT(readability-function-cognitive-complexity)
{
    enum { MOTOR_COUNT = 4 };
    enum { BACK_RIGHT = 0, FRONT_RIGHT = 1, BACK_LEFT = 2, FRONT_LEFT = 3 };

    // calculate the motor outputs without yaw applied
    float throttle = commands.throttle;
    motorOutputs[BACK_RIGHT]  = throttle - commands.roll - commands.pitch + commands.yaw;
    motorOutputs[FRONT_RIGHT] = throttle - commands.roll + commands.pitch - commands.yaw;
    motorOutputs[BACK_LEFT]   = throttle + commands.roll - commands.pitch - commands.yaw;
    motorOutputs[FRONT_LEFT]  = throttle + commands.roll + commands.pitch + commands.yaw;

#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING)
    // High values of yaw can cause motor outputs to go outside range [motorOutputMin, params.motorOutputMax]
    // If this happens, we reduce the magnitude of the yaw command.
    // This reduces yaw authority, but avoids "yaw jumps"
    // NOTE: array indices are zero-based, whereas motor numbering in the diagram above is one-based
    params.overshoot = 0.0F;
    params.undershoot = 0.0F;
    float yawDelta = 0.0F;
    if (commands.yaw > 0.0F) {
        // check if m1 or m2 will have output less than params.motorOutputMin
        params.undershoot = std::min(params.undershoot, motorOutputs[1] - params.motorOutputMin);
        params.undershoot = std::min(params.undershoot, motorOutputs[2] - params.motorOutputMin);
        // check if m0 or m3 will have output greater than params.motorOutputMax
        params.overshoot = std::max(params.overshoot, motorOutputs[0] - params.motorOutputMax);
        params.overshoot = std::max(params.overshoot, motorOutputs[3] - params.motorOutputMax);
        if (commands.yaw + (params.undershoot - params.overshoot) > 0.0F) {
            yawDelta =  (params.undershoot - params.overshoot);
            throttle -= (params.undershoot + params.overshoot);
        }
    } else {
        // check if m0 or m3 will have output less than params.motorOutputMin
        params.undershoot = std::min(params.undershoot, motorOutputs[0] - params.motorOutputMin);
        params.undershoot = std::min(params.undershoot, motorOutputs[3] - params.motorOutputMin);
        // check if m1 or m2 will have output greater than params.motorOutputMax
        params.overshoot = std::max(params.overshoot, motorOutputs[1] - params.motorOutputMax);
        params.overshoot = std::max(params.overshoot, motorOutputs[2] - params.motorOutputMax);
        if (commands.yaw - (params.undershoot - params.overshoot) < 0.0F) {
            yawDelta = -(params.undershoot - params.overshoot);
            throttle -= (params.undershoot + params.overshoot);
        }
    }

    motorOutputs[BACK_RIGHT]  += yawDelta;
    motorOutputs[FRONT_RIGHT] -= yawDelta;
    motorOutputs[BACK_LEFT]   -= yawDelta;
    motorOutputs[FRONT_LEFT]  += yawDelta;

    float maxOutput = motorOutputs[0];
    if (motorOutputs[1] > maxOutput) { maxOutput = motorOutputs[1]; }
    if (motorOutputs[2] > maxOutput) { maxOutput = motorOutputs[2]; }
    if (motorOutputs[3] > maxOutput) { maxOutput = motorOutputs[3]; }

    if (maxOutput > params.motorOutputMax) {
        const float correction = maxOutput - params.motorOutputMax;
        throttle -= correction;
        motorOutputs[0] -= correction;
        motorOutputs[1] -= correction;
        motorOutputs[2] -= correction;
        motorOutputs[3] -= correction;
    }
#endif //LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING)

    return throttle;
}

/*!
Motor numbering is the same as Betaflight
Motor rotation is "propellers out" (ie Betaflight "yaw reversed").

CW = clockwise
CCW = counter clockwise


        front
  vCC^ 4     2 ^CWv
        \   /
^CWv 6---|*|---5 vCC^
        /   \
  vCC^ 3     1 ^CWv


"Mix" calculation
                                        m
Roll right              (left+  right-)  --++-+
Pitch up (stick back)   (front+ back-)   -+-+00
Yaw clockwise           (CC+    CW-)     --+++-

*/
float mixHexX(std::array<float, 6>& motorOutputs, const MotorMixerBase::commands_t& commands, MotorMixerBase::parameters_t& params) // NOLINT(readability-function-cognitive-complexity)
{
    enum { MOTOR_COUNT = 6 };

    // calculate the motor outputs without yaw applied
    float throttle = commands.throttle;
    static constexpr float sin30 = 0.5F;
    static constexpr float sin60 = 0.86602540378F;
    motorOutputs[0] = throttle - sin30*commands.roll - sin60*commands.pitch - commands.yaw; // back right
    motorOutputs[1] = throttle - sin30*commands.roll + sin60*commands.pitch - commands.yaw; // front right
    motorOutputs[2] = throttle + sin30*commands.roll - sin60*commands.pitch + commands.yaw; // back left
    motorOutputs[3] = throttle + sin30*commands.roll + sin60*commands.pitch + commands.yaw; // front left
    motorOutputs[4] = throttle -       commands.roll                        + commands.yaw; // center right
    motorOutputs[5] = throttle +       commands.roll                        - commands.yaw; // center left

#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING)
    // deal with yaw params.undershoot and params.overshoot
    // High values of yaw can cause motor outputs to go outside range [motorOutputMin, params.motorOutputMax]
    // If this happens, we reduce the magnitude of the yaw command.
    // This reduces yaw authority, but avoids "yaw jumps"
    // NOTE: array indices are zero-based, whereas motor numbering in the diagram above is one-based
    params.overshoot = 0.0F;
    params.undershoot = 0.0F;
    float yawDelta = 0.0F;
    if (commands.yaw > 0.0F) {
        // check if m0, m1, or m5 will have output less than params.motorOutputMin
        params.undershoot = std::min(params.undershoot, motorOutputs[0] - params.motorOutputMin);
        params.undershoot = std::min(params.undershoot, motorOutputs[1] - params.motorOutputMin);
        params.undershoot = std::min(params.undershoot, motorOutputs[5] - params.motorOutputMin);
        // check if m2, m3, or m4 will have output greater than params.motorOutputMax
        params.overshoot = std::max(params.overshoot, motorOutputs[2] - params.motorOutputMax);
        params.overshoot = std::max(params.overshoot, motorOutputs[3] - params.motorOutputMax);
        params.overshoot = std::max(params.overshoot, motorOutputs[4] - params.motorOutputMax);
        if (commands.yaw + (params.undershoot - params.overshoot) > 0.0F) {
            yawDelta =  (params.undershoot - params.overshoot);
            throttle -= (params.undershoot + params.overshoot);
        }
    } else {
        // check if m2, m3, or m4 will have output less than params.motorOutputMin
        params.undershoot = std::min(params.undershoot, motorOutputs[2] - params.motorOutputMin);
        params.undershoot = std::min(params.undershoot, motorOutputs[3] - params.motorOutputMin);
        params.undershoot = std::min(params.undershoot, motorOutputs[4] - params.motorOutputMin);
        // check if m0, m1, or m5 will have output greater than params.motorOutputMax
        params.overshoot = std::max(params.overshoot, motorOutputs[0] - params.motorOutputMax);
        params.overshoot = std::max(params.overshoot, motorOutputs[1] - params.motorOutputMax);
        params.overshoot = std::max(params.overshoot, motorOutputs[5] - params.motorOutputMax);
        if (commands.yaw - (params.undershoot - params.overshoot) < 0.0F) {
            yawDelta = -(params.undershoot - params.overshoot);
            throttle -= (params.undershoot + params.overshoot);
        }
    }

    // adjust commands.yaw to remove params.undershoot and params.overshoot
    motorOutputs[0] -= yawDelta;
    motorOutputs[1] -= yawDelta;
    motorOutputs[2] += yawDelta;
    motorOutputs[3] += yawDelta;
    motorOutputs[4] += yawDelta;
    motorOutputs[5] -= yawDelta;

    const float maxOutput = *std::max_element(motorOutputs.begin(), motorOutputs.end());

    if (maxOutput > params.motorOutputMax) {
        const float correction = maxOutput - params.motorOutputMax;
        throttle -= correction;
        for (auto& motorOutput : motorOutputs) {
            motorOutput -= correction; // cppcheck-suppress useStlAlgorithm
        }
    }
#endif //LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING)

    return throttle;
}

/*!
Motor numbering is the same as Betaflight.
Motor directions are the same as Betaflight.

CW = clockwise
CC = counter clockwise

       front
 vCC^ 8     6 ^CWv
 ^CWv 4     2 vCC^
       \   /
        |X|
       /   \
 vCC^ 3     1 ^CWv
 ^CWv 7     5 vCC^

"Mix" calculation
                                        m 5678
Roll right              (left+  right-)  --++ --++
Pitch up (stick back)   (front+ back-)   -+-+ -+-+
Yaw clockwise           (CC+    CW-)     -++- +--+
*/
float mixOctoQuadX(std::array<float, 8>& motorOutputs, const MotorMixerBase::commands_t& commands, MotorMixerBase::parameters_t& params) // NOLINT(readability-function-cognitive-complexity)
{
    enum { MOTOR_COUNT = 8 };

    float throttle = commands.throttle;

    motorOutputs[0] = throttle - commands.roll - commands.pitch - commands.yaw; // back right
    motorOutputs[1] = throttle - commands.roll + commands.pitch + commands.yaw; // front right
    motorOutputs[2] = throttle + commands.roll - commands.pitch + commands.yaw; // back left
    motorOutputs[3] = throttle + commands.roll + commands.pitch - commands.yaw; // front left

    motorOutputs[4] = throttle - commands.roll - commands.pitch + commands.yaw; // under back right
    motorOutputs[5] = throttle - commands.roll + commands.pitch - commands.yaw; // under front right
    motorOutputs[6] = throttle + commands.roll - commands.pitch - commands.yaw; // under back left
    motorOutputs[7] = throttle + commands.roll + commands.pitch + commands.yaw; // under front left

#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING)
    // deal with yaw params.undershoot and params.overshoot
    // High values of yaw can cause motor outputs to go outside range [motorOutputMin, params.motorOutputMax]
    // If this happens, we reduce the magnitude of the yaw command.
    // This reduces yaw authority, but avoids "yaw jumps"
    // NOTE: array indices are zero-based, whereas motor numbering in the diagram above is one-based
    params.overshoot = 0.0F;
    params.undershoot = 0.0F;
    float yawDelta = 0.0F;
    if (commands.yaw > 0.0F) {
        // check if m0, m3, m5, or m6 will have output less than params.motorOutputMin
        params.undershoot = std::min(params.undershoot, motorOutputs[0] - params.motorOutputMin);
        params.undershoot = std::min(params.undershoot, motorOutputs[3] - params.motorOutputMin);
        params.undershoot = std::min(params.undershoot, motorOutputs[5] - params.motorOutputMin);
        params.undershoot = std::min(params.undershoot, motorOutputs[6] - params.motorOutputMin);
        // check if m1, m2, m4, or m7 will have output greater than params.motorOutputMax
        params.overshoot = std::max(params.overshoot, motorOutputs[1] - params.motorOutputMax);
        params.overshoot = std::max(params.overshoot, motorOutputs[2] - params.motorOutputMax);
        params.overshoot = std::max(params.overshoot, motorOutputs[4] - params.motorOutputMax);
        params.overshoot = std::max(params.overshoot, motorOutputs[7] - params.motorOutputMax);
        if (commands.yaw + (params.undershoot - params.overshoot) > 0.0F) {
            yawDelta =  (params.undershoot - params.overshoot);
            throttle -= (params.undershoot + params.overshoot);
        }
    } else {
        // check if m1, m2, m4, or m7  will have output less than params.motorOutputMin
        params.undershoot = std::min(params.undershoot, motorOutputs[1] - params.motorOutputMin);
        params.undershoot = std::min(params.undershoot, motorOutputs[2] - params.motorOutputMin);
        params.undershoot = std::min(params.undershoot, motorOutputs[4] - params.motorOutputMin);
        params.undershoot = std::min(params.undershoot, motorOutputs[7] - params.motorOutputMin);
        // check if m0, m3, m5, or m6 will have output greater than params.motorOutputMax
        params.overshoot = std::max(params.overshoot, motorOutputs[0] - params.motorOutputMax);
        params.overshoot = std::max(params.overshoot, motorOutputs[3] - params.motorOutputMax);
        params.overshoot = std::max(params.overshoot, motorOutputs[5] - params.motorOutputMax);
        params.overshoot = std::max(params.overshoot, motorOutputs[6] - params.motorOutputMax);
        if (commands.yaw - (params.undershoot - params.overshoot) < 0.0F) {
            yawDelta = -(params.undershoot - params.overshoot);
            throttle -= (params.undershoot + params.overshoot);
        }
    }

    motorOutputs[0] -= yawDelta;
    motorOutputs[1] += yawDelta;
    motorOutputs[2] += yawDelta;
    motorOutputs[3] -= yawDelta;
    motorOutputs[4] += yawDelta;
    motorOutputs[5] -= yawDelta;
    motorOutputs[6] -= yawDelta;
    motorOutputs[7] += yawDelta;

    const float maxOutput = *std::max_element(motorOutputs.begin(), motorOutputs.end());

    if (maxOutput > params.motorOutputMax) {
        const float correction = maxOutput - params.motorOutputMax;
        throttle -= correction;
        for (auto& motorOutput : motorOutputs) {
            motorOutput -= correction; // cppcheck-suppress useStlAlgorithm
        }
    }
#endif //LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING)

    return throttle;
}
