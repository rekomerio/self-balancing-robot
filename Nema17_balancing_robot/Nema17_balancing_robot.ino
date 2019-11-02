/*
   Self balancing robot
   By Reko Meriö
   github.com/rekomerio
*/

#include <Wire.h>
#include <PID.h>

#define DEBUG              0

#define ANGLE_P_ADJ        0
#define ANGLE_I_ADJ        0
#define ANGLE_D_ADJ        0

#define SPEED_P_ADJ        0
#define SPEED_I_ADJ        0
#define SPEED_D_ADJ        0

/* GYRO */
#define MPU                0x68
#define REFRESH_RATE       250.0
#define ACC_RATE           8192.0
#define GYRO_RATE          32.8
#define CALIBRATION_ROUNDS 50
/*
   Used for pulse generation
   1/1 step: 62
   1/2 step: 31
   1/4 step: 16
*/
#define MINIMUM_TICKS      16
/* ROBOT */
#define CG                 0
#define MAX_SPEED          1000.0
#define MIN_SPEED          1
#define MAX_ANGLE          25
#define START_ANGLE        1
/*
  Pins dedicated for the motors
  L: LEFT
  R: RIGHT
  M: MOTOR
  P: PULSE
  D: DIRECTION
*/
#define LMP                2
#define LMD                3
#define RMP                4
#define RMD                5

#define GREEN_LED_PIN      8
#define RED_LED_PIN        9

#define SET_GREEN_LED_ON   PORTB |=  (1 << 0)
#define SET_GREEN_LED_OFF  PORTB &= ~(1 << 0)
#define SET_RED_LED_ON     PORTB |=  (1 << 1)
#define SET_RED_LED_OFF    PORTB &= ~(1 << 1)

#define pulseTime(S) (abs(MAX_SPEED / S))

template <class T>
struct axis {
  T x, y, z;
};

/* P,   I,   D,   min,  max */
PID anglePID(22.0, 0.5, 30.0, -MAX_SPEED, MAX_SPEED);    // Controls the motors to achieve desired angle
PID speedPID(0.02, 0.0, 0.00425, -MAX_ANGLE, MAX_ANGLE); // Adjusts the angle, so that speed is minimal

struct axis<int16_t> gyro;
struct axis<int16_t> acc;
struct axis<int32_t> gyroOffset = {0, 0, 0};

float angle;
float targetAngle = 0;

bool fallen = true;

void setup() {
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
#if DEBUG
  Serial.begin(9600);
#endif
  Wire.begin();

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);                                         // Wake up MPU
  Wire.endTransmission();

  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                                      // Gyro
  Wire.write(0b00010000);
  Wire.endTransmission();

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                                      // Accelerometer
  Wire.write(0b00001000);                                // 4G scale
  Wire.endTransmission();

  Wire.beginTransmission(MPU);
  Wire.write(0x1A);
  Wire.write(0x03);                                      // Set digital low pass filter to ~43Hz
  Wire.endTransmission();

  blinkLed(4, RED_LED_PIN);
  gyroCalibrate(gyroOffset);
  setInitialAngle();
  blinkLed(6, GREEN_LED_PIN);
}

uint8_t counter = 0; // Used for debugging

void loop() {
  computeAngle();

  if (angle > MAX_ANGLE || angle < -MAX_ANGLE) {
    fallen = true;
    targetAngle = 0;
    anglePID.reset();
    speedPID.reset();
  }

  if (angle < START_ANGLE && angle > -START_ANGLE) {
    fallen = false;
  }

  if (fallen) {
    stopTimer();
  } else {
    float vehicleSpeed = anglePID.compute(angle - targetAngle);
    targetAngle = speedPID.compute(-vehicleSpeed);

    if (vehicleSpeed > 0) {
      PORTD &= ~(1 << LMD);                                       // Normal direction
      PORTD |=  (1 << RMD);
    } else {
      PORTD |=  (1 << LMD);                                       // Reversed direction
      PORTD &= ~(1 << RMD);
    }

    if (vehicleSpeed > MIN_SPEED || vehicleSpeed < -MIN_SPEED) {
      setPulseDuration(pulseTime(vehicleSpeed));                 // Set motor speed
      SET_GREEN_LED_OFF;
    } else {
      stopTimer();                                               // Stop the motors
      SET_GREEN_LED_ON;
    }
  }
#if DEBUG
  readButtons();
#endif
}

void computeAngle() {
  static uint32_t previousTime = 0;
  /*
    We wait here a while if necessary, because gyro angle is measured in degrees traveled per second,
    so we need to know exactly how long one loop takes to be able to calculate the current angle.
  */
  while ((uint32_t)(micros() - previousTime) < (1000000 / REFRESH_RATE)) {}
  previousTime = micros();

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);                  // Address of acc x
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  acc.x = Wire.read() << 8 | Wire.read();
  acc.y = Wire.read() << 8 | Wire.read();
  acc.z = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU);
  Wire.write(0x43);                 // Address of gyro x
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  gyro.x = (Wire.read() << 8 | Wire.read()) - gyroOffset.x;
  gyro.y = (Wire.read() << 8 | Wire.read()) - gyroOffset.y;
  gyro.z = (Wire.read() << 8 | Wire.read()) - gyroOffset.z;

  float acc_y = atan(-1 * (acc.x / ACC_RATE) / sqrt(pow((acc.y / ACC_RATE), 2) + pow((acc.z / ACC_RATE), 2))) * RAD_TO_DEG - CG;  // Calculate accelerometer angle from y-axis
  angle += (float)(gyro.y) / GYRO_RATE / REFRESH_RATE;                                                                            // Calculate the angular rotation gyro has measured from this loop
  angle = (1.0 - (1.0 / REFRESH_RATE)) * angle + (1.0 / REFRESH_RATE) * acc_y;                                                    // Compensate for gyro drift with complementary filter
}
/*
  This function is used only in the setup to set the initial angle of the robot
*/
void setInitialAngle() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);                  // Address of acc.x
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  acc.x = Wire.read() << 8 | Wire.read();
  acc.y = Wire.read() << 8 | Wire.read();
  acc.z = Wire.read() << 8 | Wire.read();
  angle = atan(-1 * (acc.x / ACC_RATE) / sqrt(pow((acc.y / ACC_RATE), 2) + pow((acc.z / ACC_RATE), 2))) * RAD_TO_DEG - CG;
}
/*
  Read samples from gyro and calculate average
*/
void gyroCalibrate(struct axis<int32_t> &calibration) {
  for (uint8_t i = 0; i < CALIBRATION_ROUNDS; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    calibration.x += Wire.read() << 8 | Wire.read();
    calibration.y += Wire.read() << 8 | Wire.read();
    calibration.z += Wire.read() << 8 | Wire.read();
    delay(1000 / REFRESH_RATE);                           // Simulate actual loop time
  }
  calibration.x /= CALIBRATION_ROUNDS;
  calibration.y /= CALIBRATION_ROUNDS;
  calibration.z /= CALIBRATION_ROUNDS;
}

void blinkLed(uint8_t blinks, uint8_t pin) {
  for (uint8_t i = 0; i < blinks; i++) {
    digitalWrite(pin, HIGH);
    delay(65);
    digitalWrite(pin, LOW);
    delay(65);
  }
}

void readButtons() {
#if SPEED_P_ADJ
  if (counter++ % 50 == 0) {
    if (!digitalRead(11)) speedPID.setP(speedPID.getP() - 0.00025);
    if (!digitalRead(12)) speedPID.setP(speedPID.getP() + 0.00025);
    Serial.println(speedPID.getP(), 10);
  }
#endif
#if SPEED_I_ADJ
  if (counter++ % 50 == 0) {
    if (!digitalRead(11)) speedPID.setI(speedPID.getI() - 0.00001);
    if (!digitalRead(12)) speedPID.setI(speedPID.getI() + 0.00001);
    Serial.println(speedPID.getI(), 10);
  }
#endif

#if SPEED_D_ADJ
  if (counter++ % 50 == 0) {
    if (!digitalRead(11)) speedPID.setD(speedPID.getD() - 0.0005);
    if (!digitalRead(12)) speedPID.setD(speedPID.getD() + 0.0005);
    Serial.println(speedPID.getD(), 10);
  }
#endif

#if ANGLE_P_ADJ
  if (counter++ % 50 == 0) {
    if (!digitalRead(11)) anglePID.setP(anglePID.getP() - 1);
    if (!digitalRead(12)) anglePID.setP(anglePID.getP() + 1);
    Serial.println(anglePID.getP());
  }
#endif

#if ANGLE_I_ADJ
  if (counter++ % 50 == 0) {
    if (!digitalRead(11)) anglePID.setI(anglePID.getI() - 0.05);
    if (!digitalRead(12)) anglePID.setI(anglePID.getI() + 0.05);
    Serial.println(anglePID.getI());
  }
#endif

#if ANGLE_D_ADJ
  if (counter++ % 50 == 0) {
    if (!digitalRead(11)) anglePID.setD(anglePID.getD() - 1);
    if (!digitalRead(12)) anglePID.setD(anglePID.getD() + 1);
    Serial.println(anglePID.getD());
  }
#endif
}
/*
  Set duration for the pulse generated for stepper motors
*/
void setPulseDuration(float pulseTime) {
  if (pulseTime < 1) return;
  stopTimer();
  OCR1A = MINIMUM_TICKS * pulseTime;
  TCNT1 = (TCNT1 < OCR1A) ? TCNT1 : 0; // Set the counter to 0, if OCR1A is smaller than TCNT1. - Risk of corrupting TCNT1, if timer is not disabled!
  startTimer();
}
/*
  Sets prescaler to 0 to stop the timer
*/
void stopTimer() {
  TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
}
/*
  Sets prescaler to 64 to start the timer
*/
void startTimer() {
  TCCR1B |= (1 << CS11) | (1 << CS10);
}
/*
  Pulse generation for stepper motors
*/
#define RMP_HIGH (PIND >> RMP) & 1

ISR(TIMER1_COMPA_vect) {
  if (RMP_HIGH) {
    /* Set pins low */
    PORTD &= ~((1 << LMP) | (1 << RMP));
  } else {
    /* Set pins high */
    PORTD |= (1 << LMP) | (1 << RMP);
  }
}
