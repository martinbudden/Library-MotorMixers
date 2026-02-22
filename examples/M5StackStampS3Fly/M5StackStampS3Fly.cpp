#include <Arduino.h>
#include <motor_mixer_quadx_pwm.h>
#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <esp32-hal-ledc.h>
#endif

#define USE_MOTOR_MIXER
MotorMixerQuadXPwm* motorMixerPtr = nullptr;
MotorMixerQuadBase::motor_pins_t pins = MotorMixerQuadBase::MOTOR_PINS;


void setup()
{
    enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7};

    Serial.begin(115200);
    delay(1000); // delay to allow serial port to initialize before first print

    Serial.printf("**** Ready ****\r\n\r\n");
#if defined(USE_MOTOR_MIXER)
    static MotorMixerQuadXPwm motorMixer(MotorMixerQuadBase::MOTOR_PINS);
    motorMixerPtr = &motorMixer;
#else
    // Motor PWM Frequency
    static constexpr int frequency_hz = 150000;
    // PWM Resolution
    static constexpr int resolutionBits = 8;
// -D MOTOR_PINS=motor_pins_t{.m0=41,.m1=42,.m2=10,.m3=5} ; BR, FR, BL, FL
#if defined(FRAMEWORK_ARDUINO_ESP32_V2)
    pins = MotorMixerQuadBase::MOTOR_PINS;
    ledcSetup(0, frequency_hz, resolutionBits);
    ledcSetup(1, frequency_hz, resolutionBits);
    ledcSetup(2, frequency_hz, resolutionBits);
    ledcSetup(3, frequency_hz, resolutionBits);
    ledcAttachPin(pins.m0, 0); // back right
    ledcAttachPin(pins.m1, 1); // front right
    ledcAttachPin(pins.m2, 2); // back left
    ledcAttachPin(pins.m3, 3); // front left
#else
    ledcAttach(pins.m0, frequency_hz, resolutionBits);
    ledcAttach(pins.m1, frequency_hz, resolutionBits);
    ledcAttach(pins.m2, frequency_hz, resolutionBits);
    ledcAttach(pins.m3, frequency_hz, resolutionBits);
#endif
#endif

}

void loop()
{
    static size_t loopCounter = 0;
    static uint8_t motor_index = 0;
    Serial.printf("Writing to motor %d\r\n", motor_index);
#if defined(USE_MOTOR_MIXER)
    motorMixerPtr->write_motor(motor_index, 0.05F);
#else
#if defined(FRAMEWORK_ARDUINO_ESP32_V2)
    ledcWrite(motor_index, 10);
#else
    switch(motor_index) {
    case 0:
        ledcWrite(pins.m0, 10);
        break;
    case 1:
        ledcWrite(pins.m1, 10);
        break;
    case 2:
        ledcWrite(pins.m2, 10);
        break;
    case 3:
        ledcWrite(pins.m3, 10);
        break;
    default:
        break;
    }
#endif
#endif
    delay(500);
    ++loopCounter;
    if (loopCounter == 10) {
        loopCounter = 0;
#if defined(USE_MOTOR_MIXER)
        motorMixerPtr->write_motor(motor_index, 0.0F);
#else
#if defined(FRAMEWORK_ARDUINO_ESP32_V2)
        ledcWrite(motor_index, 0);
#else
        switch(motor_index) {
        case 0:
            ledcWrite(pins.m0, 0);
            break;
        case 1:
            ledcWrite(pins.m1, 0);
            break;
        case 2:
            ledcWrite(pins.m2, 0);
            break;
        case 3:
            ledcWrite(pins.m3, 0);
            break;
        default:
            break;
        }
#endif
#endif
        ++motor_index;
        if (motor_index == 4) {
            motor_index = 0;
        }
    }
}
