#include <Arduino.h>
#include <MotorMixerQuadX_PWM.h>
#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <esp32-hal-ledc.h>
#endif

#define USE_MOTOR_MIXER
MotorMixerQuadX_PWM* motorMixerPtr = nullptr;
MotorMixerQuadBase::motor_pins_t pins;


void setup()
{
    enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7};

    Serial.begin(115200);
    delay(1000); // delay to allow serial port to initialize before first print

    Serial.printf("**** Ready ****\r\n\r\n");
#if defined(USE_MOTOR_MIXER)
    static MotorMixerQuadX_PWM motorMixer(MotorMixerQuadBase::MOTOR_PINS);
    motorMixerPtr = &motorMixer;
#else
    // Motor PWM Frequency
    static constexpr int frequencyHz = 150000;
    // PWM Resolution
    static constexpr int resolutionBits = 8;

    pins = MotorMixerQuadBase::MOTOR_PINS;
    ledcSetup(0, frequencyHz, resolutionBits);
    ledcSetup(1, frequencyHz, resolutionBits);
    ledcSetup(2, frequencyHz, resolutionBits);
    ledcSetup(3, frequencyHz, resolutionBits);
// -D MOTOR_PINS=motor_pins_t{.m0=41,.m1=42,.m2=10,.m3=5} ; BR, FR, BL, FL

    ledcAttachPin(pins.m0, 0); // back right
    ledcAttachPin(pins.m1, 1); // front right
    ledcAttachPin(pins.m2, 2); // back left
    ledcAttachPin(pins.m3, 3); // front left
#endif

}

void loop()
{
    static size_t loopCounter = 0;
    static uint8_t motorIndex = 0;
    Serial.printf("Writing to motor %d\r\n", motorIndex);
#if defined(USE_MOTOR_MIXER)
    motorMixerPtr->writeMotor(motorIndex, 0.05F);
#else
    ledcWrite(motorIndex, 10);
#endif
    delay(500);
    ++loopCounter;
    if (loopCounter == 10) {
        loopCounter = 0;
#if defined(USE_MOTOR_MIXER)
        motorMixerPtr->writeMotor(motorIndex, 0.0F);
#else
        ledcWrite(motorIndex, 0);
#endif
        ++motorIndex;
        if (motorIndex == 4) {
            motorIndex = 0;
        }
    }
}
