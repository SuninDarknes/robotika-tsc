#include <AccelStepper.h>
#include <Wire.h>
#include <Math.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#define DISTANCE_TO_BALL 15



#define RIGHT_STEPPER_DIR 5
#define RIGHT_STEPPER_STEP 6
#define LEFT_STEPPER_STEP 3
#define LEFT_STEPPER_DIR 2

#define RIGHT_OUT_IR_SENSOR 23
#define RIGHT_IN_IR_SENSOR 25
#define LEFT_IN_IR_SENSOR 39
#define LEFT_OUT_IR_SENSOR 41

#define LEFT_US_SENSOR_TRIG 35
#define LEFT_US_SENSOR_ECHO 37
#define RIGHT_US_SENSOR_TRIG 31
#define RIGHT_US_SENSOR_ECHO 33
#define FRONT_US_SENSOR_TRIG 29
#define FRONT_US_SENSOR_ECHO 27

#define BALL_SWITCH 43

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
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x3F, 16, 2);

int ballsCollected = 0;


void setupPins() {
  pinMode(RIGHT_STEPPER_DIR, OUTPUT);
  pinMode(RIGHT_STEPPER_STEP, OUTPUT);
  pinMode(LEFT_STEPPER_STEP, OUTPUT);
  pinMode(LEFT_STEPPER_DIR, OUTPUT);

  pinMode(RIGHT_IN_IR_SENSOR, INPUT);
  pinMode(LEFT_IN_IR_SENSOR, INPUT);
  pinMode(RIGHT_OUT_IR_SENSOR, INPUT);
  pinMode(LEFT_OUT_IR_SENSOR, INPUT);

  pinMode(RIGHT_US_SENSOR_ECHO, INPUT);
  pinMode(LEFT_US_SENSOR_ECHO, INPUT);
  pinMode(FRONT_US_SENSOR_ECHO, INPUT);

  pinMode(RIGHT_US_SENSOR_TRIG, OUTPUT);
  pinMode(LEFT_US_SENSOR_TRIG, OUTPUT);
  pinMode(FRONT_US_SENSOR_TRIG, OUTPUT);

  pinMode(10, OUTPUT);

  pinMode(BALL_SWITCH, INPUT_PULLUP);

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

int getUSDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
};


AccelStepper LStepper(1, LEFT_STEPPER_STEP, LEFT_STEPPER_DIR);
AccelStepper RStepper(1, RIGHT_STEPPER_STEP, RIGHT_STEPPER_DIR);
float spMp = 1;
class Steppers {
public:
  static int setup() {
    LStepper.setMaxSpeed(1000 * spMp);
    RStepper.setMaxSpeed(1000 * spMp);

    LStepper.setAcceleration(5000);
    RStepper.setAcceleration(5000);
    LStepper.setSpeed(1000 * spMp);
    RStepper.setSpeed(1000);
  }
  static void run() {
    LStepper.run();
    RStepper.run();
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

  static void move(int step) {
    LStepper.move(step);
    RStepper.move(step);
  }
  static bool followLine() {
    LStepper.move(10000);
    RStepper.move(10000);
    if (digitalRead(LEFT_IN_IR_SENSOR)) {
      LStepper.setMaxSpeed(750 * spMp);
      RStepper.setMaxSpeed(1000 * spMp);
    }
    if (digitalRead(RIGHT_IN_IR_SENSOR)) {
      LStepper.setMaxSpeed(1000 * spMp);
      RStepper.setMaxSpeed(750 * spMp);
    }

    run();
  }
  static bool followLine(int ignoreSideLines) {
    move(10000);
    if (isLeftIn()) {
      LStepper.setMaxSpeed(750 * spMp);
      RStepper.setMaxSpeed(1000 * spMp);
    }
    if (isRightIn()) {
      LStepper.setMaxSpeed(1000 * spMp);
      RStepper.setMaxSpeed(750 * spMp);
    }

    run();
  }
  static void lineFollowStop() {

    LStepper.move(0);
    RStepper.move(0);
    LStepper.setMaxSpeed(1000 * spMp);
    RStepper.setMaxSpeed(1000 * spMp);
  }
  static void runUntilEnd() {
    while (LStepper.distanceToGo() != 0 && RStepper.distanceToGo() != 0) {
      LStepper.run();
      RStepper.run();
    }
  }

  static void followUntilLeftTurn() {
    while (!digitalRead(LEFT_OUT_IR_SENSOR))
      followLine();
    lineFollowStop();
    move(900);
    runUntilEnd();
    RotateUntilSens(-1);
  }
  static void followUntilRightTurn() {
    while (!digitalRead(RIGHT_OUT_IR_SENSOR))
      followLine();
    lineFollowStop();
    move(900);
    runUntilEnd();
    RotateUntilSens(1);
  }

  static void followUntilEnd() {
    while (!(digitalRead(LEFT_IN_IR_SENSOR) && digitalRead(RIGHT_IN_IR_SENSOR)))
      followLine();
    lineFollowStop();
    LStepper.moveTo(0);
    LStepper.setCurrentPosition(0);
    RStepper.moveTo(0);
    RStepper.setCurrentPosition(0);
  }
  static void goUntilEnd() {
    unsigned long cooldownn = millis();
    while (true) {
      followLine();
      if (millis() > cooldownn + 500) {
        cooldownn = millis();
        lcd.clear();
        int sen = getUSDistance(LEFT_US_SENSOR_TRIG, LEFT_US_SENSOR_ECHO);
        lcd.print(sen);
        if (sen < 30) break;
      }
    }
    lcd.print("OUT");

    followUntilEnd();
  }
  static void Rotate(int degree) {
    //3195 je pun krug
    //3195/360 = 8.875
    LStepper.move((long)(8.85 * degree));
    RStepper.move((long)(8.85 * -degree));

    runUntilEnd();
    if (degree == abs(180)) {
      move(-1000);
      runUntilEnd();
    }
  }

  static void RotateUntilSens(int degree) {
    Rotate(degree * 20);

    LStepper.setMaxSpeed(400 * spMp);
    RStepper.setMaxSpeed(400 * spMp);
    LStepper.move(10000 * degree);
    RStepper.move(10000 * -degree);
    if (degree > 0)
      while (!digitalRead(RIGHT_IN_IR_SENSOR))
        run();
    else
      while (!digitalRead(LEFT_IN_IR_SENSOR))
        run();

    LStepper.setMaxSpeed(1000 * spMp);
    RStepper.setMaxSpeed(1000 * spMp);
  }
  static void RotateUntilSensEnd(int degree) {
    Rotate(degree * 20);

    LStepper.setMaxSpeed(400 * spMp);
    RStepper.setMaxSpeed(400 * spMp);
    LStepper.move(10000 * degree);
    RStepper.move(10000 * -degree);
    if (degree < 0)
      while (!digitalRead(RIGHT_IN_IR_SENSOR))
        run();
    else
      while (!digitalRead(LEFT_IN_IR_SENSOR))
        run();

    LStepper.setMaxSpeed(1000 * spMp);
    RStepper.setMaxSpeed(1000 * spMp);
  }
  static bool alignWithLine() {
    LStepper.setMaxSpeed(100);
    RStepper.setMaxSpeed(100);
    forceStop();
    LStepper.moveTo(1000);
    RStepper.moveTo(1000);
    while (digitalRead(LEFT_IN_IR_SENSOR) && digitalRead(RIGHT_IN_IR_SENSOR)) {
      run();
    }

    if (!digitalRead(LEFT_IN_IR_SENSOR)) {
      while (digitalRead(RIGHT_IN_IR_SENSOR)) {
        RStepper.run();
      }
    } else if (!digitalRead(RIGHT_IN_IR_SENSOR)) {
      while (digitalRead(LEFT_IN_IR_SENSOR)) {
        LStepper.run();
      }
    }
    while (!digitalRead(LEFT_IN_IR_SENSOR) || !digitalRead(RIGHT_IN_IR_SENSOR)) {
      run();
    }

    if (digitalRead(LEFT_IN_IR_SENSOR)) {
      while (!digitalRead(RIGHT_IN_IR_SENSOR)) {
        RStepper.run();
      }
    } else if (digitalRead(RIGHT_IN_IR_SENSOR)) {
      while (!digitalRead(LEFT_IN_IR_SENSOR)) {
        LStepper.run();
      }
    }
    while (digitalRead(LEFT_IN_IR_SENSOR) && digitalRead(RIGHT_IN_IR_SENSOR)) {
      run();
    }

    if (!digitalRead(LEFT_IN_IR_SENSOR)) {
      while (digitalRead(RIGHT_IN_IR_SENSOR)) {
        RStepper.run();
      }
    } else if (!digitalRead(RIGHT_IN_IR_SENSOR)) {
      while (digitalRead(LEFT_IN_IR_SENSOR)) {
        LStepper.run();
      }
    }


    LStepper.setMaxSpeed(1000);
    RStepper.setMaxSpeed(1000);
    forceStop();
  }
  static void forceStop() {
    LStepper.setCurrentPosition(0);
    RStepper.setCurrentPosition(0);
    LStepper.moveTo(0);
    RStepper.moveTo(0);
  }
};



bool hasBall = 0;
float bottomPos = 40,
      midPos = 0,
      gripperPos = 0;
class Servos {
public:
  static void setup(int a, int b, int c) {

    bottomPos = a;
    midPos = b;
    gripperPos = c;

    bottomServo.write(bottomPos);
    midServo.write(midPos);
    gripperServo.write(gripperPos);
  }
  static void move(float a, float b, float c) {
    a -= bottomPos;
    b -= midPos;
    c -= gripperPos;

    int highest = max(abs(a), max(abs(b), abs(c)));
    a /= highest;

    b /= highest;

    c /= highest;
    for (int pos = 0; pos <= highest; pos += 1) {
      bottomPos += a;
      midPos += b;
      gripperPos += c;
      bottomServo.write((long)bottomPos);
      midServo.write((long)midPos);
      gripperServo.write((long)gripperPos);
      delay(15);
    }
  }
  static bool pickupBall() {
    Servos::move(32, 0, 0);
    Servos::move(32, 0, 110);
    hasBall = !digitalRead(BALL_SWITCH);
    if (hasBall) {
      Servos::move(60, 0, 110);
      ballsCollected++;
    } else Servos::move(60, 0, 0);
    return hasBall;
  }
  static void shoot() {
    Servos::move(60, 0, 100);
    Servos::move(130, 0, 100);
    Servos::move(80, 180, 100);
    brushlessMotor.write(80);
    Servos::move(80, 180, 0);
    delay(2000);
    brushlessMotor.write(50);
    Servos::move(130, 0, 0);
    Servos::move(60, 0, 0);
  }
  static void release() {
    //naciljati kam pustiti lopticu
    Servos::move(60, 0, 100);
    Servos::move(60, 0, 0);
  }
};


bool parnaStanica = false;
//            
//              /$$$$$$  /$$$$$$$$ /$$$$$$$$ /$$   /$$ /$$$$$$$ 
//             /$$__  $$| $$_____/|__  $$__/| $$  | $$| $$__  $$
//            | $$  \__/| $$         | $$   | $$  | $$| $$  \ $$
//            |  $$$$$$ | $$$$$      | $$   | $$  | $$| $$$$$$$/
//             \____  $$| $$__/      | $$   | $$  | $$| $$____/ 
//             /$$  \ $$| $$         | $$   | $$  | $$| $$      
//            |  $$$$$$/| $$$$$$$$   | $$   |  $$$$$$/| $$      
//             \______/ |________/   |__/    \______/ |__/      
                                                  
                                                  
                                              



void setup() {
  setupPins();
  Serial.begin(9600);

  brushlessMotor.write(50);
  Steppers::setup();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("Hello, world!");

  //Servos::setup(60, 0, 0);
  Servos::setup(95, 180, 0);
  Servos::move(120, 0, 0);
  Servos::move(60, 0, 0);
  //colorSensor::init_TCS34725();
  //colorSensor::get_TCS34725ID();
  
    Steppers::followUntilLeftTurn();


    do {
      Steppers::followUntilLeftTurn();
      Steppers::followUntilEnd();
      Steppers::alignWithLine();
      if (Servos::pickupBall()) break;
      Steppers::Rotate(180);
/*
      parnaStanica = !parnaStanica;
      Steppers::followUntilLeftTurn();
      Steppers::followUntilRightTurn();
      Steppers::followUntilEnd();
      Steppers::alignWithLine();
      if (Servos::pickupBall()) break;
      Steppers::Rotate(180);

      parnaStanica = !parnaStanica;
      Steppers::followUntilRightTurn();
      Steppers::followUntilLeftTurn();
      Steppers::followUntilEnd();

      Steppers::alignWithLine();
      if (Servos::pickupBall()) break;
      Steppers::Rotate(180);

      parnaStanica = !parnaStanica;
      Steppers::followUntilLeftTurn();
      Steppers::followUntilRightTurn();
      Steppers::followUntilEnd();
      Steppers::alignWithLine();
      if (Servos::pickupBall()) break;
      Steppers::Rotate(180);

      Steppers::followUntilRightTurn();
      Steppers::followUntilEnd();
      Steppers::alignWithLine();
  */
    } while (false);

    Steppers::Rotate(180);
    if (parnaStanica) Steppers::followUntilRightTurn();
    else Steppers::followUntilLeftTurn();
    lcd.setCursor(0, 0);

    lcd.print("goUntilEnd");
    Steppers::goUntilEnd();
    lcd.print("alignWLine");
    Steppers::alignWithLine();
    lcd.print("release");
    Servos::release();
    do {
      do {
        Steppers::Rotate(180);
        parnaStanica = false;

        Steppers::followUntilLeftTurn();
        Steppers::followUntilEnd();
        if (Servos::pickupBall()) break;
        Steppers::Rotate(180);
        parnaStanica = !parnaStanica;

        Steppers::followUntilLeftTurn();
        Steppers::followUntilRightTurn();
        Steppers::followUntilEnd();
        if (Servos::pickupBall()) break;
        Steppers::Rotate(180);
        parnaStanica = !parnaStanica;

        Steppers::followUntilRightTurn();
        Steppers::followUntilLeftTurn();
        Steppers::followUntilEnd();
        if (Servos::pickupBall()) break;
        Steppers::Rotate(180);
        parnaStanica = !parnaStanica;

        Steppers::followUntilLeftTurn();
        Steppers::followUntilRightTurn();
        Steppers::followUntilEnd();
        if (Servos::pickupBall()) break;
        Steppers::Rotate(180);
        parnaStanica = !parnaStanica;

      } while (false);
      Steppers::Rotate(180);
      if (!parnaStanica) Steppers::followUntilRightTurn();
      else Steppers::followUntilLeftTurn();

      lcd.print("goUntilEnd");
      Steppers::goUntilEnd();
      lcd.print("alignWLine");
      lcd.print("release");
      Servos::release();
    } while (ballsCollected < 2);

  ballsCollected = 0;
  Steppers::Rotate(-100);
  Steppers::followUntilRightTurn();
  do {
    Steppers::followUntilEnd();
    Steppers::alignWithLine();
    while (true)
      if (Servos::pickupBall()) break;
    Steppers::Rotate(-90);
    Steppers::forceStop();
    LStepper.setMaxSpeed(300);
    RStepper.setMaxSpeed(300);
    LStepper.move(-10000);
    RStepper.move(10000);
    while (!digitalRead(RIGHT_IN_IR_SENSOR)) {
      Steppers::run();
    }

    LStepper.setMaxSpeed(1000);
    RStepper.setMaxSpeed(1000);

    Steppers::followUntilEnd();
    Steppers::alignWithLine();

    Steppers::Rotate(-90);
    Steppers::forceStop();
    LStepper.setMaxSpeed(200);
    RStepper.setMaxSpeed(200);
    LStepper.move(-10000);
    RStepper.move(10000);
    while (!digitalRead(RIGHT_IN_IR_SENSOR)) {
      Steppers::run();
    }


    LStepper.setMaxSpeed(100);
    RStepper.setMaxSpeed(100);
    LStepper.move(10000);
    RStepper.move(-10000);
    unsigned long lMil = millis();
    int dist = 10000, stage = 0, lastDist = 0;
    for (int i = 0; i < 3000; i++) {
      if (millis() > lMil + 100) {
        dist = getUSDistance(FRONT_US_SENSOR_TRIG, FRONT_US_SENSOR_ECHO);
        lMil = millis();
      }
      lcd.clear();
      lcd.print(dist);
      if (dist < 100 && dist > 2 && stage == 0) {
        stage++;
        lastDist = dist;
      }
      if (dist < 100 && dist > 2 && stage == 1 && dist < lastDist - 4) {
        lcd.print("PIJU!");
        stage++;
        break;
      }

      Steppers::run();
    }
    delay(1000000);

    Servos::shoot();
  } while (ballsCollected < 3);
}


void loop() {
  analogWrite(10,0);
    delay(150);
  analogWrite(10,150);
    delay(150);
  
}
