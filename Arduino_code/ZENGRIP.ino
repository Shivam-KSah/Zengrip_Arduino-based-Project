#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <IBusBM.h>

// Initialize objects
IBusBM IBus;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Motor Driver Pins
#define ENA 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 10
#define ENB 11

// Servo Channels on PCA9685
#define SERVO_1 0
#define SERVO_2 1

// Servo PWM Range (MG995 - Full 0° to 180°)
#define SERVO_MIN 150  // 0°
#define SERVO_MAX 600  // 180°

int throttle = 0, steering = 0;
int targetServo1 = 90, targetServo2 = 90; // Target positions for smooth movement
int currentServo1 = 90, currentServo2 = 90; // Current positions

void setup() {
    Serial.begin(115200);
    IBus.begin(Serial);

    pwm.begin();
    pwm.setPWMFreq(50); // Standard 50Hz for servos

    // Set motor control pins as OUTPUT
    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
}

void loop() {
    // Read iBus Channels
    throttle = map(IBus.readChannel(1), 1000, 2000, -255, 255); // CH3: Forward/Backward
    steering = map(IBus.readChannel(3), 1000, 2000, -255, 255); // CH1: Left/Right

    // **Use SWB (CH5) and SWC (CH6) for servo control**
    targetServo1 = map(IBus.readChannel(4), 1000, 2000, 0, 180); // SWB → Servo 1 (0° to 180°)
    targetServo2 = map(IBus.readChannel(5), 1000, 2000, 0, 180); // SWC → Servo 2 (0° to 180°)

    // Debugging Servo Inputs
    Serial.print("THROTTLE"); Serial.print(throttle);
    Serial.print("STEERING"); Serial.println(steering);
     // Deb
    // **Smooth Servo Movements**
    moveServoSmoothly(&currentServo1, targetServo1, SERVO_1);
    moveServoSmoothly(&currentServo2, targetServo2, SERVO_2);

    // Motor Control Logic
    if (throttle > 50) MoveForward(250);
    else if (throttle < -50) MoveBackward(250);
    else if (steering > 50) TurnRight(250);
    else if (steering < -50) TurnLeft(250);
    else Stop();
}

// 🚗 Movement Functions
void MoveBackward(int speed) {
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Moving Front");

}

void MoveForward(int speed) {
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Moving Back");
    
}

void TurnLeft(int speed) {
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
}

void TurnRight(int speed) {
       analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void Stop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

// **Smooth Servo Movement Function**
void moveServoSmoothly(int *currentPos, int targetPos, int servoChannel) {
    if (*currentPos < targetPos) (*currentPos)++; // Move up step-by-step
    if (*currentPos > targetPos) (*currentPos)--; // Move down step-by-step

    pwm.setPWM(servoChannel, 0, map(*currentPos, 0, 180, SERVO_MIN, SERVO_MAX));
    delay(0); // Small delay for smooth movement
}