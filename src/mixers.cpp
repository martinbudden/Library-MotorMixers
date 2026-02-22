#include "mixers.h"


float mix_wing(std::array<float, 3>& motor_outputs, const motor_mixer_commands_t& commands, motor_mixer_parameters_t& params)
{
    (void)params;
    enum { THROTTLE, LEFT_FLAPERON, RIGHT_FLAPERON };

    motor_outputs[THROTTLE]       =  commands.throttle; // throttle may be controlled by a servo for a wing with an internal combustion engine
    motor_outputs[LEFT_FLAPERON]  =  commands.roll + commands.pitch;
    motor_outputs[RIGHT_FLAPERON] = -commands.roll + commands.pitch;

    return commands.throttle;
}

float mix_airplane(std::array<float, 5>& motor_outputs, const motor_mixer_commands_t& commands, motor_mixer_parameters_t& params)
{
    (void)params;
    enum { THROTTLE, LEFT_AILERON, RIGHT_AILERON, ELEVATOR, RUDDER };

    motor_outputs[THROTTLE]      =  commands.throttle; // throttle may be controlled by a servo for a wing with an internal combustion engine
    motor_outputs[LEFT_AILERON]  =  commands.roll;
    motor_outputs[RIGHT_AILERON] = -commands.roll;
    motor_outputs[ELEVATOR]      =  commands.pitch;
    motor_outputs[RUDDER]        =  commands.yaw;

    return commands.throttle;
}

float mix_bicopter(std::array<float, 4>& motor_outputs, const motor_mixer_commands_t& commands, motor_mixer_parameters_t& params)
{
    (void)params;
    enum { MOTOR_LEFT, MOTOR_RIGHT, SERVO_LEFT, SERVO_RIGHT };

    const float throttle = commands.throttle;
    motor_outputs[MOTOR_LEFT]   =  throttle + commands.roll;
    motor_outputs[MOTOR_RIGHT]  =  throttle - commands.roll;
    motor_outputs[SERVO_LEFT]   =  commands.pitch - commands.yaw;
    motor_outputs[SERVO_RIGHT]  =  commands.pitch + commands.yaw;

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
float mix_tricopter(std::array<float, 4>& motor_outputs, const motor_mixer_commands_t& commands, motor_mixer_parameters_t& params)
{
    enum { REAR = 0, FR = 1, FL = 2, S0 = 3};
    constexpr float TWO_THIRDS = 2.0F / 3.0F;
    constexpr float FOUR_THIRDS = 4.0F / 3.0F;

    const float throttle = commands.throttle;
    const float pivotAngleRadians = commands.yaw*params.max_servo_angle_radians;
    motor_outputs[REAR] = (throttle                 - FOUR_THIRDS * commands.pitch) / cosf(pivotAngleRadians);
    motor_outputs[FR]   =  throttle - commands.roll + TWO_THIRDS  * commands.pitch;
    motor_outputs[FL]   =  throttle + commands.roll + TWO_THIRDS  * commands.pitch;
    motor_outputs[S0]   = commands.yaw;

#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_TRICOPTER)
    // check for rear overshoot, front motors unlikely to overshoot since there are two of them and there is no yaw-related attenuation
    params.overshoot = motor_outputs[REAR] - params.motor_output_max;
    if (params.overshoot > 0.0F) {
        // rear motor is saturated, so reduce its output to params.motor_output_max and reduce front motors similarly
        // !!TODO: should also increase yaw
        motor_outputs[REAR] = params.motor_output_max;
        motor_outputs[FR] = std::max(params.motor_output_min, motor_outputs[FR] - params.overshoot);
        motor_outputs[FL] = std::max(params.motor_output_min, motor_outputs[FL] - params.overshoot);
    }

    // check for front undershoot
    params.undershoot = std::min(motor_outputs[FL], motor_outputs[FR]) - params.motor_output_min;
    if (params.undershoot < 0.0F) {
        motor_outputs[REAR] = params.motor_output_min;
        motor_outputs[FR] = std::min(params.motor_output_max, motor_outputs[FR] - params.undershoot);
        motor_outputs[FL] = std::min(params.motor_output_max, motor_outputs[FL] - params.undershoot);
    }
#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_TRICOPTER)

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
float mix_quad_x(std::array<float, 4>& motor_outputs, const motor_mixer_commands_t& commands, motor_mixer_parameters_t& params) // NOLINT(readability-function-cognitive-complexity)
{
    // NOTE: motor array indices are zero-based, whereas motor numbering in the diagram above is one-based

    enum { MOTOR_COUNT = 4 };
    enum { BACK_RIGHT = 0, FRONT_RIGHT = 1, BACK_LEFT = 2, FRONT_LEFT = 3 };

    // calculate the motor outputs without yaw applied
    float throttle = commands.throttle;
    motor_outputs[BACK_RIGHT]  = throttle - commands.roll - commands.pitch; // + commands.yaw;
    motor_outputs[FRONT_RIGHT] = throttle - commands.roll + commands.pitch; // - commands.yaw;
    motor_outputs[BACK_LEFT]   = throttle + commands.roll - commands.pitch; // - commands.yaw;
    motor_outputs[FRONT_LEFT]  = throttle + commands.roll + commands.pitch; // + commands.yaw;

    params.overshoot = 0.0F;
    params.undershoot = 0.0F;

#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_ROLL_PITCH)
    // Check for overshoot caused by roll and pitch.
    // If there is overshoot, we can just clamp the output, since this will just reduce the magnitude of the command
    // without without affecting the other axes (because of the symmetry of the QuadX).
    for (auto& motorOutput : motor_outputs) {
        motorOutput = std::clamp(motorOutput, params.motor_output_min, params.motor_output_max);
    }
#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_ROLL_PITCH

    motor_outputs[BACK_RIGHT]  += commands.yaw;
    motor_outputs[FRONT_RIGHT] -= commands.yaw;
    motor_outputs[BACK_LEFT]   -= commands.yaw;
    motor_outputs[FRONT_LEFT]  += commands.yaw;

#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_YAW)
    // Now check if there is overshoot due to yaw
    // We cannot simply clamp the offending outputs, since this may cause result in a change in the overall
    // vertical thrust (ie a "yaw jump").
    // For example, if m1 and m2 have their undershoot clamped without a corresponding clamping of the m0 and m3
    // then the overall vertical thrust will increase and the quadcopter will "jump" upwards.
    // So instead of clamping individual motors, we reduce the magnitude of the yaw command.
    if (commands.yaw > 0.0F) {
        // check if m1 or m2 will have output less than params.motor_output_min
        params.undershoot = std::min(params.undershoot, motor_outputs[1] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[2] - params.motor_output_min);
        // check if m0 or m3 will have output greater than params.motor_output_max
        params.overshoot = std::max(params.overshoot, motor_outputs[0] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[3] - params.motor_output_max);
        if (commands.yaw + (params.undershoot - params.overshoot) > 0.0F) {
            throttle -= (params.undershoot + params.overshoot);
            const float yawDelta =  (params.undershoot - params.overshoot);
            motor_outputs[BACK_RIGHT]  += yawDelta;
            motor_outputs[FRONT_RIGHT] -= yawDelta;
            motor_outputs[BACK_LEFT]   -= yawDelta;
            motor_outputs[FRONT_LEFT]  += yawDelta;
        }
    } else if (commands.yaw < 0.0F) {
        // check if m0 or m3 will have output less than params.motor_output_min
        params.undershoot = std::min(params.undershoot, motor_outputs[0] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[3] - params.motor_output_min);
        // check if m1 or m2 will have output greater than params.motor_output_max
        params.overshoot = std::max(params.overshoot, motor_outputs[1] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[2] - params.motor_output_max);
        if (commands.yaw - (params.undershoot - params.overshoot) < 0.0F) {
            throttle -= (params.undershoot + params.overshoot);
            const float yawDelta = -(params.undershoot - params.overshoot);
            motor_outputs[BACK_RIGHT]  += yawDelta;
            motor_outputs[FRONT_RIGHT] -= yawDelta;
            motor_outputs[BACK_LEFT]   -= yawDelta;
            motor_outputs[FRONT_LEFT]  += yawDelta;
        }
    }
#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_YAW)

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
float mix_hex_x(std::array<float, 6>& motor_outputs, const motor_mixer_commands_t& commands, motor_mixer_parameters_t& params) // NOLINT(readability-function-cognitive-complexity)
{
    // NOTE: motor array indices are zero-based, whereas motor numbering in the diagram above is one-based

    enum { MOTOR_COUNT = 6 };

    // calculate the motor outputs without yaw applied
    float throttle = commands.throttle;
    static constexpr float sin30 = 0.5F;
    static constexpr float sin60 = 0.86602540378F;
    motor_outputs[0] = throttle - sin60*commands.pitch; // back right
    motor_outputs[1] = throttle + sin60*commands.pitch; // front right
    motor_outputs[2] = throttle - sin60*commands.pitch; // back left
    motor_outputs[3] = throttle + sin60*commands.pitch; // front left
    motor_outputs[4] = throttle; // center right
    motor_outputs[5] = throttle; // center left

    params.overshoot = 0.0F;
    params.undershoot = 0.0F;

#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_ROLL_PITCH)
    // Check for overshoot caused by pitch.
    // If there is overshoot, we can just clamp the output, since this will just reduce the magnitude of the command
    // without without affecting the other axes (because of the symmetry of the HexX).
    // NOTE: motors motor_outputs[4] and motor_outputs[5] are not clamped, since they have no effect on pitch.
    motor_outputs[0] = std::clamp(motor_outputs[0], params.motor_output_min, params.motor_output_max);
    motor_outputs[1] = std::clamp(motor_outputs[1], params.motor_output_min, params.motor_output_max);
    motor_outputs[2] = std::clamp(motor_outputs[2], params.motor_output_min, params.motor_output_max);
    motor_outputs[3] = std::clamp(motor_outputs[3], params.motor_output_min, params.motor_output_max);
#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_ROLL_PITCH

    motor_outputs[0] -= sin30*commands.roll;
    motor_outputs[1] -= sin30*commands.roll;
    motor_outputs[2] += sin30*commands.roll;
    motor_outputs[3] += sin30*commands.roll;
    motor_outputs[4] -=       commands.roll;
    motor_outputs[5] +=       commands.roll;

#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_ROLL_PITCH)
    // If we have overshoot caused by roll we cannot just clamp the output, since this will affect the yaw
    if (commands.roll > 0.0F) {
        // check if m2, m3, or m5 will have output less than params.motor_output_min
        params.undershoot = std::min(params.undershoot, motor_outputs[2] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[3] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[5] - params.motor_output_min);
        // check if m0, m1, or m4 will have output greater than params.motor_output_max
        params.overshoot = std::max(params.overshoot, motor_outputs[0] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[1] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[4] - params.motor_output_max);
        if (commands.roll + (params.undershoot - params.overshoot) > 0.0F) {
            throttle -= (params.undershoot + params.overshoot);
            const float rollDelta =  (params.undershoot - params.overshoot);
            motor_outputs[0] -= rollDelta;
            motor_outputs[1] -= rollDelta;
            motor_outputs[2] += rollDelta;
            motor_outputs[3] += rollDelta;
            motor_outputs[4] -= rollDelta;
            motor_outputs[5] += rollDelta;
        }
    } else {
        // check if m2, m3, or m5 will have output less than params.motor_output_min
        params.undershoot = std::min(params.undershoot, motor_outputs[2] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[3] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[5] - params.motor_output_min);
        // check if m0, m1, or m4 will have output greater than params.motor_output_max
        params.overshoot = std::max(params.overshoot, motor_outputs[0] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[1] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[4] - params.motor_output_max);
        if (commands.yaw - (params.undershoot - params.overshoot) < 0.0F) {
            throttle -= (params.undershoot + params.overshoot);
            const float rollDelta = -(params.undershoot - params.overshoot);
            motor_outputs[0] -= rollDelta;
            motor_outputs[1] -= rollDelta;
            motor_outputs[2] += rollDelta;
            motor_outputs[3] += rollDelta;
            motor_outputs[4] -= rollDelta;
            motor_outputs[5] += rollDelta;
        }
    }
#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_ROLL_PITCH

    motor_outputs[0] -= commands.yaw;
    motor_outputs[1] -= commands.yaw;
    motor_outputs[2] += commands.yaw;
    motor_outputs[3] += commands.yaw;
    motor_outputs[4] += commands.yaw;
    motor_outputs[5] -= commands.yaw;

#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_YAW)
    if (commands.yaw > 0.0F) {
        // check if m0, m1, or m5 will have output less than params.motor_output_min
        params.undershoot = std::min(params.undershoot, motor_outputs[0] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[1] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[5] - params.motor_output_min);
        // check if m2, m3, or m4 will have output greater than params.motor_output_max
        params.overshoot = std::max(params.overshoot, motor_outputs[2] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[3] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[4] - params.motor_output_max);
        if (commands.yaw + (params.undershoot - params.overshoot) > 0.0F) {
            throttle -= (params.undershoot + params.overshoot);
            const float yawDelta =  (params.undershoot - params.overshoot);
            motor_outputs[0] -= yawDelta;
            motor_outputs[1] -= yawDelta;
            motor_outputs[2] += yawDelta;
            motor_outputs[3] += yawDelta;
            motor_outputs[4] += yawDelta;
            motor_outputs[5] -= yawDelta;
        }
    } else {
        // check if m2, m3, or m4 will have output less than params.motor_output_min
        params.undershoot = std::min(params.undershoot, motor_outputs[2] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[3] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[4] - params.motor_output_min);
        // check if m0, m1, or m5 will have output greater than params.motor_output_max
        params.overshoot = std::max(params.overshoot, motor_outputs[0] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[1] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[5] - params.motor_output_max);
        if (commands.yaw - (params.undershoot - params.overshoot) < 0.0F) {
            throttle -= (params.undershoot + params.overshoot);
            const float yawDelta = -(params.undershoot - params.overshoot);
            motor_outputs[0] -= yawDelta;
            motor_outputs[1] -= yawDelta;
            motor_outputs[2] += yawDelta;
            motor_outputs[3] += yawDelta;
            motor_outputs[4] += yawDelta;
            motor_outputs[5] -= yawDelta;
        }
    }
#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_YAW

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
float mix_octo_quad_x(std::array<float, 8>& motor_outputs, const motor_mixer_commands_t& commands, motor_mixer_parameters_t& params) // NOLINT(readability-function-cognitive-complexity)
{
    // NOTE: motor array indices are zero-based, whereas motor numbering in the diagram above is one-based
    enum { MOTOR_COUNT = 8 };

    float throttle = commands.throttle;

    motor_outputs[0] = throttle - commands.roll - commands.pitch; // - commands.yaw; // back right
    motor_outputs[1] = throttle - commands.roll + commands.pitch; // + commands.yaw; // front right
    motor_outputs[2] = throttle + commands.roll - commands.pitch; // + commands.yaw; // back left
    motor_outputs[3] = throttle + commands.roll + commands.pitch; // - commands.yaw; // front left

    motor_outputs[4] = throttle - commands.roll - commands.pitch; // + commands.yaw; // under back right
    motor_outputs[5] = throttle - commands.roll + commands.pitch; // - commands.yaw; // under front right
    motor_outputs[6] = throttle + commands.roll - commands.pitch; // - commands.yaw; // under back left
    motor_outputs[7] = throttle + commands.roll + commands.pitch; // + commands.yaw; // under front left

    params.overshoot = 0.0F;
    params.undershoot = 0.0F;

#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_ROLL_PITCH)
    // Check for overshoot caused by roll and pitch.
    // If there is overshoot, we can just clamp the output, since this will just reduce the magnitude of the command
    // without without affecting the other axes (because of the symmetry of the QuadX).
    for (auto& motorOutput : motor_outputs) {
        motorOutput = std::clamp(motorOutput, params.motor_output_min, params.motor_output_max);
    }
#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_ROLL_PITCH

    motor_outputs[0] -= commands.yaw; // back right
    motor_outputs[1] += commands.yaw; // front right
    motor_outputs[2] += commands.yaw; // back left
    motor_outputs[3] -= commands.yaw; // front left

    motor_outputs[4] += commands.yaw; // under back right
    motor_outputs[5] -= commands.yaw; // under front right
    motor_outputs[6] -= commands.yaw; // under back left
    motor_outputs[7] += commands.yaw; // under front left

#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_YAW)
    if (commands.yaw > 0.0F) {
        // check if m0, m3, m5, or m6 will have output less than params.motor_output_min
        params.undershoot = std::min(params.undershoot, motor_outputs[0] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[3] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[5] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[6] - params.motor_output_min);
        // check if m1, m2, m4, or m7 will have output greater than params.motor_output_max
        params.overshoot = std::max(params.overshoot, motor_outputs[1] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[2] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[4] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[7] - params.motor_output_max);
        if (commands.yaw + (params.undershoot - params.overshoot) > 0.0F) {
            throttle -= (params.undershoot + params.overshoot);
            const float yawDelta =  (params.undershoot - params.overshoot);
            motor_outputs[0] -= yawDelta;
            motor_outputs[1] += yawDelta;
            motor_outputs[2] += yawDelta;
            motor_outputs[3] -= yawDelta;
            motor_outputs[4] += yawDelta;
            motor_outputs[5] -= yawDelta;
            motor_outputs[6] -= yawDelta;
            motor_outputs[7] += yawDelta;
        }
    } else {
        // check if m1, m2, m4, or m7  will have output less than params.motor_output_min
        params.undershoot = std::min(params.undershoot, motor_outputs[1] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[2] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[4] - params.motor_output_min);
        params.undershoot = std::min(params.undershoot, motor_outputs[7] - params.motor_output_min);
        // check if m0, m3, m5, or m6 will have output greater than params.motor_output_max
        params.overshoot = std::max(params.overshoot, motor_outputs[0] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[3] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[5] - params.motor_output_max);
        params.overshoot = std::max(params.overshoot, motor_outputs[6] - params.motor_output_max);
        if (commands.yaw - (params.undershoot - params.overshoot) < 0.0F) {
            throttle -= (params.undershoot + params.overshoot);
            const float yawDelta = -(params.undershoot - params.overshoot);
            motor_outputs[0] -= yawDelta;
            motor_outputs[1] += yawDelta;
            motor_outputs[2] += yawDelta;
            motor_outputs[3] -= yawDelta;
            motor_outputs[4] += yawDelta;
            motor_outputs[5] -= yawDelta;
            motor_outputs[6] -= yawDelta;
            motor_outputs[7] += yawDelta;
        }
    }
#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_YAW

    return throttle;
}
