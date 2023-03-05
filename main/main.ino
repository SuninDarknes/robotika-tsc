#include <AccelStepper.h>

#define RIGHT_STEPPER_DIR 11
#define RIGHT_STEPPER_STEP 3
#define LEFT_STEPPER_STEP 6
#define LEFT_STEPPER_DIR 5

#define RIGHT_OUT_IR_SENSOR 23
#define RIGHT_IN_IR_SENSOR 25
#define LEFT_IN_IR_SENSOR 27
#define LEFT_OUT_IR_SENSOR 29

void setupPins() {
  pinMode(RIGHT_STEPPER_DIR, OUTPUT);
  pinMode(RIGHT_STEPPER_STEP, OUTPUT);
  pinMode(LEFT_STEPPER_STEP, OUTPUT);
  pinMode(LEFT_STEPPER_DIR, OUTPUT);

  pinMode(RIGHT_IN_IR_SENSOR, INPUT);
  pinMode(LEFT_IN_IR_SENSOR, INPUT);
  pinMode(RIGHT_OUT_IR_SENSOR, INPUT);
  pinMode(LEFT_OUT_IR_SENSOR, INPUT);
}



AccelStepper LStepper(1, LEFT_STEPPER_STEP, LEFT_STEPPER_DIR);
AccelStepper RStepper(1, RIGHT_STEPPER_STEP, RIGHT_STEPPER_DIR);
class Steppers {
public:
  static int setup() {
    TCCR3A = (1 << WGM31) | (1 << COM3C1);
    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS32);
    TCCR4A = (1 << WGM41) | (1 << COM4A1);
    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS42);
  }

  static void stopRight() {
    digitalWrite(6, LOW);
  }
  static void stopLeft() {
    digitalWrite(3, LOW);
  }
  static void stop() {
    digitalWrite(6, LOW);
    digitalWrite(3, LOW);
    ICR3 = 100;
    ICR4 = 100;
  }
  static bool followLine() {
    if (digitalRead(RIGHT_IN_IR_SENSOR)) {
      ICR3 = 100;
      ICR4 = 110;

      return true;
    }
    if (digitalRead(LEFT_IN_IR_SENSOR)) {
      ICR3 = 110;
      ICR4 = 100;
      return true;
    }
    return false;
  }

  static void followUntilLeftTurn() {
    Steppers::setup();

    while (!digitalRead(LEFT_OUT_IR_SENSOR))
      followLine();
    Serial.println("Left line found!");

    delay(600);

    ICR3 = 5000;
    delay(800);
    while (!digitalRead(RIGHT_IN_IR_SENSOR)) {}
    Steppers::stop();
    delay(1000);
  }
  static void followUntilRightTurn() {
    Steppers::setup();

    while (!digitalRead(RIGHT_OUT_IR_SENSOR))
      followLine();
    delay(600);

    ICR4 = 5000;
    delay(800);
    while (!digitalRead(LEFT_IN_IR_SENSOR)) {}
    Steppers::stop();
    delay(1000);
  }
  static void followUntilEnd() {
    while (!(digitalRead(LEFT_IN_IR_SENSOR) && digitalRead(RIGHT_IN_IR_SENSOR)))
      followLine();
  }
  static void Rotate(int degree) {
    LStepper.setMaxSpeed(1000);
    RStepper.setMaxSpeed(1000);

    LStepper.setAcceleration(300);
    RStepper.setAcceleration(300);
    LStepper.setSpeed(1200);
    RStepper.setSpeed(1200);
    //3195 je pun krug
    //3195/360 = 8.875
    LStepper.move((int)(8.875 * degree));
    RStepper.move((int)(8.875 * degree));
    while (LStepper.distanceToGo() != 0 && RStepper.distanceToGo() != 0) {
      LStepper.run();
      RStepper.run();
    }
    digitalWrite(RIGHT_STEPPER_DIR, LOW);
  }
};


void setup() {
  setupPins();
  delay(5000);
  ICR3 = 100;
  ICR4 = 100;
  digitalWrite(RIGHT_STEPPER_DIR, LOW);
  Serial.begin(9600);

  Steppers::followUntilLeftTurn();
  Steppers::setup();
  Steppers::followUntilEnd();
  ICR3 = 100;
  ICR4 = 100;
  delay(2000);
  Steppers::stop();
  Steppers::Rotate(180);

  Steppers::followUntilLeftTurn();
  Steppers::followUntilRightTurn();
  Steppers::setup();
  Steppers::followUntilEnd();
  ICR3 = 100;
  ICR4 = 100;
  delay(2000);
  Steppers::stop();
  Steppers::Rotate(180);

  Steppers::followUntilRightTurn();
  Steppers::followUntilLeftTurn();
  
  Steppers::setup();
  Steppers::followUntilEnd();
  ICR3 = 100;
  ICR4 = 100;
  delay(2000);
  Steppers::stop();
  Steppers::Rotate(180);

  Steppers::followUntilLeftTurn();
  Steppers::followUntilRightTurn();
  Steppers::setup();
  Steppers::followUntilEnd();
  ICR3 = 100;
  ICR4 = 100;
  delay(2000);
  Steppers::stop();
  Steppers::Rotate(180);

  Steppers::followUntilRightTurn();
  Steppers::setup();
  Steppers::followUntilEnd();


  Steppers::stop();
}
void loop() {
}
