#include <AccelStepper.h>
#include <Wire.h>
#include <Math.h>
#include <Servo.h>

#define DISTANCE_TO_BALL 15



#define RIGHT_STEPPER_DIR 11
#define RIGHT_STEPPER_STEP 3
#define LEFT_STEPPER_STEP 6
#define LEFT_STEPPER_DIR 5

#define RIGHT_OUT_IR_SENSOR 23
#define RIGHT_IN_IR_SENSOR 25
#define LEFT_IN_IR_SENSOR 27
#define LEFT_OUT_IR_SENSOR 29

#define LEFT_US_SENSOR_TRIG 31
#define LEFT_US_SENSOR_ECHO 33
#define RIGHT_US_SENSOR_TRIG 35
#define RIGHT_US_SENSOR_ECHO 37
#define FRONT_US_SENSOR_TRIG 39
#define FRONT_US_SENSOR_ECHO 41

#define BOTTOM_SERVO 11
#define MID_SERVO 12
#define GRIPPER_SERVO 13
#define BRUSHLESS_MOTOR 45


#define SensorAddressWrite 0x5A  //
#define SensorAddressRead 0x5A   //29
#define EnableAddress 0xD1       // register address + command bits
#define ATimeAddress 0xd2        // register address + command bits
#define WTimeAddress 0xd4        // register address + command bits
#define ConfigAddress 0xDE       // register address + command bits
#define ControlAddress 0xE0      // register address + command bits
#define IDAddress 0xe3           // register address + command bits
#define ColorAddress 0xe6        // register address + command bits



Servo bottomServo, midServo, gripperServo, brushlessMotor;

void setupPins() {
  pinMode(RIGHT_STEPPER_DIR, OUTPUT);
  pinMode(RIGHT_STEPPER_STEP, OUTPUT);
  pinMode(LEFT_STEPPER_STEP, OUTPUT);
  pinMode(LEFT_STEPPER_DIR, OUTPUT);

  pinMode(RIGHT_IN_IR_SENSOR, INPUT);
  pinMode(LEFT_IN_IR_SENSOR, INPUT);
  pinMode(RIGHT_OUT_IR_SENSOR, INPUT);
  pinMode(LEFT_OUT_IR_SENSOR, INPUT);

  bottomServo.attach(BOTTOM_SERVO);
  midServo.attach(MID_SERVO);
  gripperServo.attach(GRIPPER_SERVO);
  brushlessMotor.attach(BRUSHLESS_MOTOR);
}

class colorSensor {
public:
  byte i2cWriteBuffer[10];
  byte i2cReadBuffer[10];



  unsigned long count = 0, t_start = 0, t_end = 0, t_sum = 0;

  unsigned int clear_color = 0;
  unsigned int red_color = 0;
  unsigned int green_color = 0;
  unsigned int blue_color = 0;
  /*  
      Send register address and the byte value you want to write the magnetometer and 
      loads the destination register with the value you send
      */
  void Writei2cRegisters(byte numberbytes, byte command) {
    byte i = 0;

    Wire.beginTransmission(SensorAddressWrite);  // Send address with Write bit set
    Wire.write(command);                         // Send command, normally the register address
    for (i = 0; i < numberbytes; i++)            // Send data
      Wire.write(i2cWriteBuffer[i]);
    Wire.endTransmission();

    delayMicroseconds(100);  // allow some time for bus to settle
  }

  /*  
      Send register address to this function and it returns byte value
      for the magnetometer register's contents 
      */
  byte Readi2cRegisters(int numberbytes, byte command) {
    byte i = 0;

    Wire.beginTransmission(SensorAddressWrite);  // Write address of read to sensor
    Wire.write(command);
    Wire.endTransmission();

    delayMicroseconds(100);  // allow some time for bus to settle

    Wire.requestFrom(SensorAddressRead, numberbytes);  // read data
    for (i = 0; i < numberbytes; i++)
      i2cReadBuffer[i] = Wire.read();

    Wire.endTransmission();

    delayMicroseconds(100);  // allow some time for bus to settle
  }

  void init_TCS34725(void) {
    i2cWriteBuffer[0] = 0x00;
    //i2cWriteBuffer[0] = 0xff;
    Writei2cRegisters(1, ATimeAddress);  // RGBC timing is 256 - contents x 2.4mS =
    i2cWriteBuffer[0] = 0x00;
    Writei2cRegisters(1, ConfigAddress);  // Can be used to change the wait time
    i2cWriteBuffer[0] = 0x00;
    Writei2cRegisters(1, ControlAddress);  // RGBC gain control
    i2cWriteBuffer[0] = 0x03;
    Writei2cRegisters(1, EnableAddress);  // enable ADs and oscillator for sensor
  }

  void get_TCS34725ID(void) {
    Readi2cRegisters(1, IDAddress);
    if (i2cReadBuffer[0] = 0x44)
      Serial.println("TCS34725 is present");
    else
      Serial.println("TCS34725 not responding");
  }

  /*
      Reads the register values for clear, red, green, and blue.
      */

  void get_Colors(void) {

    Readi2cRegisters(8, ColorAddress);
    clear_color = (unsigned int)(i2cReadBuffer[1] << 8) + (unsigned int)i2cReadBuffer[0];
    red_color = (unsigned int)(i2cReadBuffer[3] << 8) + (unsigned int)i2cReadBuffer[2];
    green_color = (unsigned int)(i2cReadBuffer[5] << 8) + (unsigned int)i2cReadBuffer[4];
    blue_color = (unsigned int)(i2cReadBuffer[7] << 8) + (unsigned int)i2cReadBuffer[6];

    //t_end = millis();

    //t_sum = t_sum + t_end - t_start;

    // send register values to the serial monitor


    if (count == 100) {
      count = 0;
      t_end = millis();

      Serial.print("t_end - t_start:");
      Serial.println(t_end - t_start, DEC);

      t_start = t_end;
      t_sum = 0;
    }
    count++;

    Serial.print("clear color=");
    Serial.print(clear_color, DEC);
    Serial.print(" red color=");
    Serial.print(red_color, DEC);
    Serial.print(" green color=");
    Serial.print(green_color, DEC);
    Serial.print(" blue color=");
    Serial.println(blue_color, DEC);
  }
};

int
getUSDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.println(distance);
  return distance;
};


AccelStepper LStepper(1, LEFT_STEPPER_STEP, LEFT_STEPPER_DIR);
AccelStepper RStepper(1, RIGHT_STEPPER_STEP, RIGHT_STEPPER_DIR);
class Steppers {
public:
  static int setup() {
    Wire.begin();




    TCCR3A = (1 << WGM31) | (1 << COM3C1);
    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS32);
    TCCR4A = (1 << WGM41) | (1 << COM4A1);
    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS42);
  }
  static bool isLeftIn() {
    return digitalRead(LEFT_IN_IR_SENSOR);
  }
  static bool isRightIn() {
    return digitalRead(RIGHT_IN_IR_SENSOR);
  }
  static bool isEnd() {
    return isLeftIn() && isRightIn();
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
    if (isRightIn()) {
      ICR3 = 100;
      ICR4 = 110;

      return true;
    }
    if (isLeftIn()) {
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
    while (!isRightIn()) {}
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
    while (!isLeftIn()) {}
    Steppers::stop();
    delay(1000);
  }
  static void followUntilEnd() {
    while (!isEnd())
      followLine();
  }
  static void Rotate(int degree) {
    LStepper.setMaxSpeed(1000);
    RStepper.setMaxSpeed(1000);

    LStepper.setAcceleration(500);
    RStepper.setAcceleration(500);
    LStepper.setSpeed(1200);
    RStepper.setSpeed(1200);
    //3195 je pun krug
    //3195/360 = 8.875
    LStepper.move((long)(8.875 * degree));
    RStepper.move((long)(8.875 * degree));
    while (LStepper.distanceToGo() != 0 && RStepper.distanceToGo() != 0) {
      LStepper.run();
      RStepper.run();
    }
    digitalWrite(RIGHT_STEPPER_DIR, LOW);
  }

  static bool alignArm() {
    LStepper.setSpeed(1200);
    RStepper.setSpeed(1200);
    LStepper.move(100);
    RStepper.move(100);
    int leftD = getUSDistance(LEFT_US_SENSOR_TRIG, LEFT_US_SENSOR_ECHO), rightD = getUSDistance(RIGHT_US_SENSOR_TRIG, RIGHT_US_SENSOR_ECHO);

    while (leftD != DISTANCE_TO_BALL && rightD != DISTANCE_TO_BALL) {
      LStepper.run();
      RStepper.run();
    }
  }
};

class Servos{
  public:
  bool hasBall=0;
  int bottomPos=0;
  int midPos=0;
  int gripperPos=0;
  bool pickupBall(){
    
      for (midPos = 00; midPos <= 30; midPos += 1) {
    bottomServo.write(90 - midPos * 2);
    midServo.write(40 - midPos);
    delay(15);  // waits 15 ms for the servo to reach the position
  }
  for (gripperPos = 00; gripperPos <= 100; gripperPos += 1) {  // goes from 0 degrees to 180 degrees
    gripperServo.write(gripperPos);
    delay(15);  // waits 15 ms for the servo to reach the position
  }
  for (bottomPos = 00; bottomPos <= 30 * 5.5; bottomPos += 1) {  // goes from 0 degrees to 180 degrees
    bottomServo.write(30 + bottomPos * 1.6 / 5.5);
    midServo.write(10 + bottomPos);
    delay(15);  // waits 15 ms for the servo to reach the position
  }
  }
};

void setup() {
  setupPins();



  /*
  delay(5000);
  ICR3 = 100;
  ICR4 = 100;
  digitalWrite(RIGHT_STEPPER_DIR, LOW);
  Serial.begin(9600);

  colorSensor::init_TCS34725();
  colorSensor::get_TCS34725ID();




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
*/
  

  brushlessMotor.write(50);
  midServo.write(0);
  bottomServo.write(60);
  gripperServo.write(0);

  return;

  for (pos = 00; pos <= 100; pos += 1) {  // goes from 0 degrees to 180 degrees
    gripperServo.write(100 - pos);
    delay(15);  // waits 15 ms for the servo to reach the position
  }


  delay(2000);
}
void loop() {
}
