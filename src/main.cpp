/*
   Self balancing robot
   By Reko Meriö
   github.com/rekomerio
*/

#include "Arduino.h"
#include <Wire.h>
#include "PID.h"
#include "Channel.h"
#include "defines.h"

template <class T>
struct axis
{
    T x, y, z;
};

void stopTimer();
void startTimer();
void computeAngle();
void setPulseDuration(float pulseTime);
void blinkLed(uint8_t blinks, uint8_t pin);
void setBraking();
void setYaw();
void gyroCalibrate(struct axis<int32_t> &calibration);
void setInitialAngle();
float pulseTime(float speed);

/* P,   I,   D,   min,  max */
PID anglePID(19.0, 0.5, 30.0, -MAX_SPEED, MAX_SPEED);                         // Controls the motors to achieve desired angle
PID speedPID(0.0165f, 0.0f, 0.00425f, -MAX_ANGLE + 10.0f, MAX_ANGLE - 10.0f); // Adjusts the angle, so that speed is minimal
PID positionPID(0.3f, 0.0f, 9.0f, -400, 400);

struct axis<int16_t> gyro;
struct axis<int16_t> acc;
struct axis<int32_t> gyroOffset =
{
    0, 0, 0
};

Channel pitch;
Channel yaw;

float angle;
float targetAngle = 0.0f;

bool isFallen = true;
bool shouldHoldPosition = true;
bool hasTargetChanged = false;
bool hasVehicleStopped = false;

/* Used for controlling the yaw */
uint16_t numInterrupts = 0;
uint16_t rightPulseToSkip = 0;
uint16_t leftPulseToSkip = 0;

int32_t actualPosition = 0;
int32_t targetPosition = 0;
uint32_t stickLastUsedAt = 0;

void setup()
{
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 0xFFFF;                                            // When timer is equal to this value, interrupt is triggered
    TIMSK1 |= (1 << OCIE1A);                                   // Enable interrupt for OCR1A match
    TCCR1B |= (1 << WGM12);                                    // CTC mode - reset timer on match
    DDRD |= (1 << RMP) | (1 << LMP) | (1 << LMD) | (1 << RMD); // Set pins to output

    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(11, INPUT_PULLUP); // Button 1
    pinMode(12, INPUT_PULLUP); // Button 2
    pinMode(7, INPUT);         // Rx channel yaw
    pinMode(10, INPUT);        // Rx channel pitch

    // Enable interrupts for pin state changes for all digital pins
    PCICR = (1 << PCIE0 | 1 << PCIE2);

    // Enable interrupts only for D10 state changes
    PCMSK0 = 0;
    PCMSK0 = (1 << PCINT2);
    // Enable interrupts only for D7 state changes
    PCMSK2 = 0;
    PCMSK2 = (1 << PCINT23);

#if DEBUG
    Serial.begin(9600);
#endif
    Wire.begin();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0); // Wake up MPU
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B); // Gyro
    Wire.write(0b00010000);
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);       // Accelerometer
    Wire.write(0b00001000); // 4G scale
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);
    Wire.write(0x03); // Set digital low pass filter to ~43Hz
    Wire.endTransmission();

    blinkLed(4, RED_LED_PIN);
    gyroCalibrate(gyroOffset);
    setInitialAngle();
    blinkLed(6, GREEN_LED_PIN);
}

uint8_t counter = 0; // Used for debugging

void loop()
{
    computeAngle();

    if (abs(angle) > MAX_ANGLE)
    {
        isFallen = true;
        targetAngle = 0.0f;
        anglePID.reset();
        speedPID.reset();
        actualPosition = 0;
        targetPosition = 0;
    }

    if (abs(angle) < START_ANGLE)
    {
        isFallen = false;
    }

    setYaw();
    int16_t receivedSpeed = pitch.getStickPosition();
    if (abs(receivedSpeed) > 25)
    {
        stickLastUsedAt = millis();
        hasTargetChanged = true;
        hasVehicleStopped = false;
    }

    if (isFallen)
    {
        stopTimer();
    }
    else
    {
        float vehicleSpeed = anglePID.compute(angle - targetAngle);

        if (shouldHoldPosition && hasVehicleStopped)
        {
            if (hasTargetChanged)
            {
                stopTimer();
                targetPosition = actualPosition;
                startTimer();
                hasTargetChanged = false;
            }
            receivedSpeed += positionPID.compute(actualPosition - targetPosition);
        }

        targetAngle = speedPID.compute((float)receivedSpeed * 1.5f - vehicleSpeed);

        if (vehicleSpeed > 0)
        {
            PORTD &= ~(1 << LMD); // Normal direction
            PORTD |= (1 << RMD);
        }
        else
        {
            PORTD |= (1 << LMD); // Reversed direction
            PORTD &= ~(1 << RMD);
        }

        if (abs(vehicleSpeed) > MIN_SPEED)
        {
            setPulseDuration(pulseTime(vehicleSpeed)); // Set motor speed
            SET_GREEN_LED_OFF;
        }
        else
        {
            stopTimer(); // Stop the motors
            SET_GREEN_LED_ON;
            if ((uint32_t)(millis() - stickLastUsedAt) > 100)
                hasVehicleStopped = true;
        }
    }
#if DEBUG
    adjustPid();
#else
    setBraking();
#endif
}

void computeAngle()
{
    static uint32_t previousTime = 0;
    /*
    We wait here a while if necessary, because gyro angle is measured in degrees traveled per second,
    so we need to know exactly how long one loop takes to be able to calculate the current angle.
  */
    constexpr uint16_t loopTime = 1000000 / REFRESH_RATE;
    while ((uint32_t)(micros() - previousTime) < loopTime)
    {
    }
    previousTime = micros();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Address of acc x
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    acc.x = Wire.read() << 8 | Wire.read();
    acc.y = Wire.read() << 8 | Wire.read();
    acc.z = Wire.read() << 8 | Wire.read();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); // Address of gyro x
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    gyro.x = (Wire.read() << 8 | Wire.read()) - gyroOffset.x;
    gyro.y = (Wire.read() << 8 | Wire.read()) - gyroOffset.y;
    gyro.z = (Wire.read() << 8 | Wire.read()) - gyroOffset.z;

    constexpr float complementGyro = 1.0f - (1.0f / REFRESH_RATE);
    constexpr float complementAcc = 1.0f / REFRESH_RATE;

    float accelerationY = atan(-1 * (acc.x / ACC_RATE) / sqrt(pow((acc.y / ACC_RATE), 2) + pow((acc.z / ACC_RATE), 2))) * RAD_TO_DEG - CG; // Calculate accelerometer angle from y-axis
    angle += (float)(gyro.y) / GYRO_RATE / REFRESH_RATE;                                                                                   // Calculate the angular rotation gyro has measured from this loop
    angle = complementGyro * angle + complementAcc * accelerationY;                                                                        // Compensate for gyro drift with complementary filter
}
/*
  This function is used only in the setup to set the initial angle of the robot
*/
void setInitialAngle()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Address of acc.x
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    acc.x = Wire.read() << 8 | Wire.read();
    acc.y = Wire.read() << 8 | Wire.read();
    acc.z = Wire.read() << 8 | Wire.read();
    angle = atan(-1 * (acc.x / ACC_RATE) / sqrt(pow((acc.y / ACC_RATE), 2) + pow((acc.z / ACC_RATE), 2))) * RAD_TO_DEG - CG;
}
/*
  Read samples from gyro and calculate average
*/
void gyroCalibrate(struct axis<int32_t> &calibration)
{
    for (uint8_t i = 0; i < CALIBRATION_ROUNDS; i++)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 6, true);
        calibration.x += Wire.read() << 8 | Wire.read();
        calibration.y += Wire.read() << 8 | Wire.read();
        calibration.z += Wire.read() << 8 | Wire.read();
        delay(1000 / REFRESH_RATE); // Simulate actual loop time
    }
    calibration.x /= CALIBRATION_ROUNDS;
    calibration.y /= CALIBRATION_ROUNDS;
    calibration.z /= CALIBRATION_ROUNDS;
}

void blinkLed(uint8_t blinks, uint8_t pin)
{
    for (uint8_t i = 0; i < blinks; i++)
    {
        digitalWrite(pin, HIGH);
        delay(50);
        digitalWrite(pin, LOW);
        delay(50);
    }
}

void setYaw()
{
    int16_t receivedYaw = yaw.getStickPosition();

    if (receivedYaw < -25)
    {
        leftPulseToSkip = (1000 / (-receivedYaw));
    }
    else
    {
        leftPulseToSkip = 0;
    }

    if (receivedYaw > 25)
    {
        rightPulseToSkip = (1000 / receivedYaw);
    }
    else
    {
        rightPulseToSkip = 0;
    }
}

void setBraking()
{
    if (LEFT_BTN_IS_PRESSED)
    { // Slow braking
        //anglePID.setPID(19.0, 0.5, 30.0);
        //speedPID.setPID(0.0165, 0.0, 0.00425);
        shouldHoldPosition = false;
        SET_RED_LED_ON;
    }
    else if (RIGHT_BTN_IS_PRESSED)
    { // Fast braking
        //anglePID.setPID(15.0, 0.5, 25.0);
        //speedPID.setPID(0.0265, 0.0, 0.0095);
        shouldHoldPosition = true;
        SET_RED_LED_ON;
    }
    else
    {
        SET_RED_LED_OFF;
    }
}
/*
  For debugging purposes
*/
void adjustPid()
{
#if SPEED_P_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            speedPID.setP(speedPID.getP() - 0.00025);
        if (RIGHT_BTN_IS_PRESSED)
            speedPID.setP(speedPID.getP() + 0.00025);
        Serial.println(speedPID.getP(), 10);
    }
#endif
#if SPEED_I_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            speedPID.setI(speedPID.getI() - 0.00001);
        if (RIGHT_BTN_IS_PRESSED)
            speedPID.setI(speedPID.getI() + 0.00001);
        Serial.println(speedPID.getI(), 10);
    }
#endif

#if SPEED_D_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            speedPID.setD(speedPID.getD() - 0.0005);
        if (RIGHT_BTN_IS_PRESSED)
            speedPID.setD(speedPID.getD() + 0.0005);
        Serial.println(speedPID.getD(), 10);
    }
#endif

#if ANGLE_P_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            anglePID.setP(anglePID.getP() - 1);
        if (RIGHT_BTN_IS_PRESSED)
            anglePID.setP(anglePID.getP() + 1);
        Serial.println(anglePID.getP());
    }
#endif

#if ANGLE_I_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            anglePID.setI(anglePID.getI() - 0.05);
        if (RIGHT_BTN_IS_PRESSED)
            anglePID.setI(anglePID.getI() + 0.05);
        Serial.println(anglePID.getI());
    }
#endif

#if ANGLE_D_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            anglePID.setD(anglePID.getD() - 1);
        if (RIGHT_BTN_IS_PRESSED)
            anglePID.setD(anglePID.getD() + 1);
        Serial.println(anglePID.getD());
    }
#endif
#if POSITION_P_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            positionPID.setP(positionPID.getP() - 0.05);
        if (RIGHT_BTN_IS_PRESSED)
            positionPID.setP(positionPID.getP() + 0.05);
        Serial.println(positionPID.getP(), 10);
    }
#endif
#if POSITION_I_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            positionPID.setI(positionPID.getI() - 0.00001);
        if (RIGHT_BTN_IS_PRESSED)
            positionPID.setI(positionPID.getI() + 0.00001);
        Serial.println(positionPID.getI(), 10);
    }
#endif

#if POSITION_D_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            positionPID.setD(positionPID.getD() - 0.5);
        if (RIGHT_BTN_IS_PRESSED)
            positionPID.setD(positionPID.getD() + 0.5);
        Serial.println(positionPID.getD(), 10);
    }
#endif
}
/*
  Set duration for the pulse generated for stepper motors
*/
void setPulseDuration(float pulseTime)
{
    if (pulseTime < 1)
        return;
    stopTimer();
    OCR1A = MINIMUM_TICKS * pulseTime;
    TCNT1 = (TCNT1 < OCR1A) ? TCNT1 : 0; // Set the counter to 0, if OCR1A is smaller than TCNT1. - Risk of corrupting TCNT1, if timer is not disabled!
    startTimer();
}
/*
  Sets prescaler to 0 to stop the timer
*/
inline void stopTimer()
{
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
}
/*
  Sets prescaler to 64 to start the timer
*/
inline void startTimer()
{
    TCCR1B |= (1 << CS11) | (1 << CS10);
}

float pulseTime(float speed) 
{
    // Do this to avoid overflowing 16-bit register when going slow speed
    // 1000 / 65535 = 0.01525...
    // Keep the limit a bit higher because of Atmega328p's inaccurate floating point calculations
    if (speed < 0.016f && speed > -0.016f) 
    {
        return MAX_SPEED / 0.016f;
    }
    return fabs(MAX_SPEED / speed);
}
/*
  Pulse generation for stepper motors.
  Yawing of the robot is done by skipping some pulses
*/
#define RMP_HIGH (PIND >> RMP) & 1
#define LMP_HIGH (PIND >> LMP) & 1
#define RMD_HIGH (PIND >> RMD) & 1

ISR(TIMER1_COMPA_vect)
{
    // Drive right motor
    if (rightPulseToSkip == 0 || numInterrupts % rightPulseToSkip != 0)
    {
        if (RMP_HIGH)
        {
            /* Set pins low */
            PORTD &= ~(1 << RMP);
        }
        else
        {
            /* Set pins high */
            PORTD |= (1 << RMP);
        }
    }

    // Drive left motor
    if (leftPulseToSkip == 0 || numInterrupts % leftPulseToSkip != 0)
    {
        if (LMP_HIGH)
        {
            /* Set pins low */
            PORTD &= ~(1 << LMP);
        }
        else
        {
            /* Set pins high */
            PORTD |= (1 << LMP);
        }
    }
    numInterrupts++;

    if (numInterrupts % 2 == 0)
        actualPosition += (RMD_HIGH) ? -1 : 1;
}

// Receiver channel 4, pin D7
#define RX1_IS_HIGH (PIND >> 7) & 1
// Receiver channel 2, pin D10
#define RX2_IS_HIGH (PINB >> 2) & 1
/*
  Capture the pulsewidth of rx channel
*/
ISR(PCINT0_vect)
{
    if (RX2_IS_HIGH)
    {
        pitch.setStartTime(micros());
    }
    else
    {
        pitch.setEndTime(micros());
    }
}

ISR(PCINT2_vect)
{
    if (RX1_IS_HIGH)
    {
        yaw.setStartTime(micros());
    }
    else
    {
        yaw.setEndTime(micros());
    }
}