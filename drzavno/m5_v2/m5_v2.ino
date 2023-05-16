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

//Inicijalizacija pinova
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
//klasa za sve varijable i funkcije senzora za boje
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

  bool isColor(int r_in, int g_in, int b_in, int c_in) {
    int r, g, b;
    r = red_color;
    g = green_color;
    b = blue_color;

    return (abs(r - r_in) < 1000 && abs(g - g_in) < 1000 && abs(b - b_in) < 1000 && clear_color > c_in);
  }
  void printColor() {
    get_Colors();
    lcd.clear();
    lcd.print(red_color);
    lcd.print(green_color);
    lcd.setCursor(0, 1);
    lcd.print(blue_color);
    lcd.print(clear_color);
    delay(100);
  }
};
//Vraca vrijednost u cm od danog ultrasoniconog senzora.
int getUSDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.34 / 2;
  return distance;
};

//Kreiranje objekata za koračne motore
AccelStepper LStepper(1, LEFT_STEPPER_STEP, LEFT_STEPPER_DIR);
AccelStepper RStepper(1, RIGHT_STEPPER_STEP, RIGHT_STEPPER_DIR);
float spMp = 1.25;

//Statična klasa sa funkcijamam za pokretanje robota pomocu koračnih motora
class Steppers {
public:

  //Inicijalizacija za objekte koji kontroliraju koračne motore pomoću AccelStepper biblioteke
  static int setup() {
    LStepper.setMaxSpeed(1000 * spMp);
    RStepper.setMaxSpeed(1000 * spMp);

    LStepper.setAcceleration(5000);
    RStepper.setAcceleration(5000);
    LStepper.setSpeed(1000 * spMp);
    RStepper.setSpeed(1000 * spMp);
  }
  //oba motora naprave 1 korak
  static void run() {
    LStepper.run();
    RStepper.run();
  }
  static void runSpeed() {
    LStepper.runSpeed();
    RStepper.runSpeed();
  }
  //dodavanje broj koraka u oba stepera
  static void move(int step) {
    LStepper.move(step);
    RStepper.move(step);
  }
  //Praćenje linije
  static bool followLine() {
    LStepper.move(1000);
    RStepper.move(1000);
    if (digitalRead(LEFT_IN_IR_SENSOR)) {
      LStepper.setMaxSpeed(750 * spMp);
      RStepper.setMaxSpeed(1250 * spMp);
      LStepper.setSpeed(750 * spMp);
      RStepper.setSpeed(1250 * spMp);
    }
    if (digitalRead(RIGHT_IN_IR_SENSOR)) {
      LStepper.setMaxSpeed(1250 * spMp);
      RStepper.setMaxSpeed(750 * spMp);
      LStepper.setSpeed(1250 * spMp);
      RStepper.setSpeed(750 * spMp);
    }

    runSpeed();
  }

  //Praćenje linije i korekcija u slučaju da dođe na ulicu
  static bool followLine(int ignoreSides) {
    LStepper.move(100);
    RStepper.move(100);
    if (digitalRead(LEFT_IN_IR_SENSOR) && !digitalRead(LEFT_OUT_IR_SENSOR)) {
      LStepper.setMaxSpeed(750 * spMp);
      RStepper.setMaxSpeed(1100 * spMp);
      LStepper.setSpeed(750 * spMp);
      RStepper.setSpeed(1100 * spMp);
    }
    if (digitalRead(RIGHT_IN_IR_SENSOR) && !digitalRead(RIGHT_OUT_IR_SENSOR)) {
      LStepper.setMaxSpeed(1100 * spMp);
      RStepper.setMaxSpeed(750 * spMp);
      LStepper.setSpeed(1100 * spMp);
      RStepper.setSpeed(750 * spMp);
    }
    run();
  }
  //Postavljanje jednake brzine na oba motora
  static void resetSpeed() {
    LStepper.setCurrentPosition(0);
    RStepper.setCurrentPosition(0);
    LStepper.moveTo(0);
    RStepper.moveTo(0);
    LStepper.setMaxSpeed(1000 * spMp);
    RStepper.setMaxSpeed(1000 * spMp);
    LStepper.setSpeed(1000 * spMp);
    RStepper.setSpeed(1000 * spMp);
  }
  //Pokretanje motora do prije određenog broja koraka
  static void runUntilEnd() {
    while (LStepper.distanceToGo() != 0 && RStepper.distanceToGo() != 0) {
      LStepper.run();
      RStepper.run();
    }
  }
  //Pokretanje motora do prve linije
  static void runUntilLine() {
    resetSpeed();
    move(10000);
    while (!digitalRead(LEFT_IN_IR_SENSOR) && !digitalRead(RIGHT_IN_IR_SENSOR)) {
      LStepper.run();
      RStepper.run();
    }
  }
  //Pokretanje motora do prvog očitanja ljevog vanjskog IR senzora
  static void followUntilLeftTurn() {
    while (!digitalRead(LEFT_OUT_IR_SENSOR))
      followLine();
    resetSpeed();
    move(800);
    runUntilEnd();
    RotateUntilSens(-1);
  }
  //Pokretanje motora do prvog očitanja desnog vanjskog IR senzora
  static void followUntilRightTurn() {
    while (!digitalRead(RIGHT_OUT_IR_SENSOR))
      followLine();
    resetSpeed();
    move(800);
    runUntilEnd();
    RotateUntilSens(1);
  }

  //Prati liniju dok ne dođe do kraja ulice.
  static void followUntilEnd() {
    while (!(digitalRead(LEFT_IN_IR_SENSOR) && digitalRead(RIGHT_IN_IR_SENSOR)))
      followLine();
    resetSpeed();
    LStepper.moveTo(0);
    LStepper.setCurrentPosition(0);
    RStepper.moveTo(0);
    RStepper.setCurrentPosition(0);
  }
  //Prati liniju dok ne dođe do kraja ulice, ignorira sporedne ulice.
  static void followUntilEndIgnoreOut() {
    while (!(digitalRead(LEFT_IN_IR_SENSOR) && digitalRead(RIGHT_IN_IR_SENSOR)) || digitalRead(LEFT_OUT_IR_SENSOR) || digitalRead(RIGHT_OUT_IR_SENSOR)) {


      while (!(digitalRead(LEFT_IN_IR_SENSOR) && digitalRead(RIGHT_IN_IR_SENSOR)) || digitalRead(LEFT_OUT_IR_SENSOR) || digitalRead(RIGHT_OUT_IR_SENSOR))
        followLine(1);

      move(40);
      runUntilEnd();
    }
  }
  //Prati liniju dok ljevi US senzor vidi objekt bliže od 30 cm
  static void goUntilEnd() {
    unsigned long cooldownn = millis();
    while (true) {
      followLine();
      if (millis() > cooldownn + 500) {
        cooldownn = millis();
        lcd.clear();
        int sen = getUSDistance(LEFT_US_SENSOR_TRIG, LEFT_US_SENSOR_ECHO);
        lcd.print(sen);
        if (sen < 300) break;
      }
    }
    lcd.print("OUT");

    followUntilEnd();
  }
  //Okreče so na mjestu za postavljeni kut u smjeru kazaljke na satu
  static void Rotate(int degree) {
    //3195 je pun krug
    //3195/360 = 8.875
    resetSpeed();
    float jedanStupanjStep = 3195.0 / 360.0;
    float okretanje = jedanStupanjStep * degree;
    LStepper.move((long)okretanje);
    RStepper.move((long)-okretanje);

    runUntilEnd();
  }
  static void Rotate(int degree, bool rotateAroundCenter) {
    //3195 je pun krug
    //3195/360 = 8.875
    resetSpeed();
    LStepper.move((long)(8.85 * degree));
    RStepper.move((long)(8.85 * -degree));

    runUntilEnd();
    move(-900);
    runUntilEnd();
  }
  //Okretanje u postavljenom smjeru do očitanje linije
  static void RotateUntilSens(int degree) {
    Rotate(degree * 50);

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

    resetSpeed();
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

  //Poravnanje s linijom na kraju ulice
  static bool alignWithLine() {
    LStepper.setMaxSpeed(100);
    RStepper.setMaxSpeed(100);
    forceStop();

    ////LOOP za poravnavanje
    for (int i = 0; i < 3; i++) {


      LStepper.moveTo(-1000);
      RStepper.moveTo(-1000);
      while (!(digitalRead(LEFT_IN_IR_SENSOR) && digitalRead(RIGHT_IN_IR_SENSOR))) {
        run();
      }


      lcd.clear();
      lcd.print("Poravnavanje ");
      lcd.print(i);
      lcd.setCursor(0, 1);
      lcd.print("S kraj linije");
      lcd.setCursor(0, 0);

      LStepper.moveTo(1000);
      RStepper.moveTo(1000);
      while (digitalRead(LEFT_IN_IR_SENSOR) && digitalRead(RIGHT_IN_IR_SENSOR)) {
        run();
      }


      lcd.clear();
      lcd.print("Poravnavanje ");
      lcd.print(i);
      lcd.setCursor(0, 1);
      lcd.print("Jednostrano ravnanje");
      lcd.setCursor(0, 0);

      if (!digitalRead(LEFT_IN_IR_SENSOR)) {
        while (digitalRead(RIGHT_IN_IR_SENSOR)) {
          RStepper.run();
        }
      } else if (!digitalRead(RIGHT_IN_IR_SENSOR)) {
        while (digitalRead(LEFT_IN_IR_SENSOR)) {
          LStepper.run();
        }
      }
    }
  }

  //Zaustavljanje motora
  static void
  forceStop() {
    LStepper.setCurrentPosition(0);
    RStepper.setCurrentPosition(0);
    LStepper.moveTo(0);
    RStepper.moveTo(0);
  }
  //Poravnava se sa zidom ili objektom i približi mu se do navedene udaljenosti
  static void korekcija(int needed_dist) {
    forceStop();
    Rotate(-10);
    LStepper.setMaxSpeed(200);
    RStepper.setMaxSpeed(200);
    LStepper.move(10000);
    RStepper.move(-10000);
    int LDist = 1000;
    int RDist = 1000;
    unsigned long lMil = millis();
    bool foundTeglin = false;
    while (true) {
      if (millis() > lMil + 50) {
        LDist = getUSDistance(LEFT_US_SENSOR_TRIG, LEFT_US_SENSOR_ECHO);
        RDist = getUSDistance(RIGHT_US_SENSOR_TRIG, RIGHT_US_SENSOR_ECHO);
        lMil = millis();
        lcd.clear();
        lcd.print(LDist);
        lcd.print("  ");
        lcd.print(RDist);
      }
      if (RDist < 200) foundTeglin = true;

      if (LDist - RDist < -2 && foundTeglin) {

        forceStop();
        break;
      }

      run();
    }

    LStepper.move(10000);
    RStepper.move(10000);
    for (int i = 0; i < 2; i++) {
      unsigned long timeout = millis();
      delay(100);
      while (abs(LDist - needed_dist) > 5 && abs(RDist - needed_dist) > 5 && timeout + 4000 > millis()) {

        LDist = getUSDistance(LEFT_US_SENSOR_TRIG, LEFT_US_SENSOR_ECHO);
        RDist = getUSDistance(LEFT_US_SENSOR_TRIG, LEFT_US_SENSOR_ECHO);

        lcd.clear();
        lcd.print(LDist);
        lcd.print("  ");
        lcd.print(RDist);
        if (LDist - needed_dist > 0) {
          LStepper.setSpeed(400);
          RStepper.setSpeed(400);
        } else {
          LStepper.setSpeed(-400);
          RStepper.setSpeed(-400);
        }
        runSpeed();
      }
    }
    LStepper.setSpeed(-400);
    RStepper.setSpeed(400);
    while (!digitalRead(RIGHT_IN_IR_SENSOR)) {
      runSpeed();
    }

    LStepper.setSpeed(400);
    RStepper.setSpeed(-400);
    while (!digitalRead(LEFT_IN_IR_SENSOR)) {
      runSpeed();
    }

    LStepper.setMaxSpeed(1000);
    RStepper.setMaxSpeed(1000);

    LStepper.setSpeed(1000);
    RStepper.setSpeed(1000);
  }

  static void korekcija_ciljanje(int needed_dist) {
    forceStop();
    Rotate(-10);
    LStepper.setMaxSpeed(200);
    RStepper.setMaxSpeed(200);
    LStepper.move(10000);
    RStepper.move(-10000);
    int LDist = 1000;
    int RDist = 1000;
    unsigned long lMil = millis();
    bool foundTeglin = false;
    while (true) {
      if (millis() > lMil + 50) {
        LDist = getUSDistance(LEFT_US_SENSOR_TRIG, LEFT_US_SENSOR_ECHO);
        RDist = getUSDistance(RIGHT_US_SENSOR_TRIG, RIGHT_US_SENSOR_ECHO);
        lMil = millis();
        lcd.clear();
        lcd.print(LDist);
        lcd.print("  ");
        lcd.print(RDist);
      }
      if (RDist < 200) foundTeglin = true;

      if (LDist - RDist < -2 && foundTeglin) {

        forceStop();
        break;
      }

      run();
    }

    LStepper.move(10000);
    RStepper.move(10000);
    for (int i = 0; i < 2; i++) {
      unsigned long timeout = millis();
      delay(100);
      while (abs(LDist - needed_dist) > 5 && abs(RDist - needed_dist) > 5 && timeout + 4000 > millis()) {

        LDist = getUSDistance(LEFT_US_SENSOR_TRIG, LEFT_US_SENSOR_ECHO);
        RDist = getUSDistance(LEFT_US_SENSOR_TRIG, LEFT_US_SENSOR_ECHO);

        lcd.clear();
        lcd.print(LDist);
        lcd.print("  ");
        lcd.print(RDist);
        if (LDist - needed_dist > 0) {
          LStepper.setSpeed(400);
          RStepper.setSpeed(400);
        } else {
          LStepper.setSpeed(-400);
          RStepper.setSpeed(-400);
        }
        runSpeed();
      }
    }
    LStepper.setSpeed(-200);
    RStepper.setSpeed(200);
    while (!digitalRead(RIGHT_IN_IR_SENSOR)) {
      runSpeed();
    }

    LStepper.setMaxSpeed(1000);
    RStepper.setMaxSpeed(1000);

    LStepper.setSpeed(1000);
    RStepper.setSpeed(1000);
  }
};



bool hasBall = 0;
float bottomPos = 40,
      midPos = 0,
      gripperPos = 0;

//Statična klasa za kontrolu servo motora
class Servos {
public:
  //Inicijalizacija servo sa postavljenim vrijednostima
  static void setup(int a, int b, int c) {

    bottomPos = a;
    midPos = b;
    gripperPos = c;

    bottomServo.write(bottomPos);
    midServo.write(midPos);
    gripperServo.write(gripperPos);
  }

  //Linerno pomiče servo u zadanu poziciju
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
      delay(10);
    }
  }
  static void forceMove(float a, float b, float c) {
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
      delay(1);
    }
  }
  //Ako je loptica pokupljena vraca true
  static bool pickupBall() {
    Servos::move(60, 0, 0);
    Servos::move(20, 0, 0);
    Servos::move(20, 0, 110);
    hasBall = !digitalRead(BALL_SWITCH);
    if (hasBall) {
      Servos::move(60, 0, 110);
      ballsCollected++;
    } else Servos::move(60, 0, 110);
    return hasBall;
  }


  //pucanje tako da ruka digne lopticu, aktivra se brushless motor i loptica se ispušta u cijev
  static void shoot() {
    Servos::move(60, 0, 100);
    Servos::move(60, 180, 100);
    brushlessMotor.write(100);
    Servos::move(60, 180, 0);
    delay(2000);

    delay(1000);
    brushlessMotor.write(50);
    Servos::move(60, 0, 0);
  }

  //Ispuštanje loptice
  static void release() {
    //naciljati kam pustiti lopticu
    Servos::move(60, 0, 110);
    Servos::move(22, 0, 110);
    Servos::move(22, 0, 0);

    Servos::move(60, 0, 110);
  }
  //Ispuštanje loptice
  static void releaseHigh() {
    //naciljati kam pustiti lopticu
    Servos::move(45, 0, 110);
    Servos::move(45, 0, 0);
    Servos::move(60, 0, 0);
  }
  //Cilja dok ne nađe najbljiži čunj i tada puca.
  //AZ: Postavit ga za 20* stupnjeva u ljevo od središnjeg čunja i puca kad vidi prvi cunj.
  static void aimAndShoot() {
    int distance = 0;
    for (int i = 0; i < 30; i++) {
      Steppers::Rotate(4);
      int passed = 0;
      for (int j = 0; j < 5; j++) {
        distance = getUSDistance(FRONT_US_SENSOR_TRIG, FRONT_US_SENSOR_ECHO);
        while (distance == 0) {
          distance = getUSDistance(FRONT_US_SENSOR_TRIG, FRONT_US_SENSOR_ECHO);
        }
        lcd.clear();
        lcd.print(distance);
        if (distance < 100) passed++;
      }

      lcd.setCursor(0, 1);
      lcd.print(passed == 5 ? "Pucaj" : "Mjerenje");
      lcd.setCursor(0, 0);
      if (passed != 5) continue;
      lcd.setCursor(0, 1);
      lcd.print("Pucaj");
      lcd.setCursor(0, 0);

      double distanceBetweenCenterAndSonic = 8.0;
      double UltrasonicAngleRad = 0.6435011;
      double cosUltrasonicAngle = 0.8;
      double sinUltrasonicAngle = 0.6;
      double radToDeg = 180.0 / PI;
      double c = distance;
      double b = cosUltrasonicAngle * c;
      double a = sinUltrasonicAngle * c;
      double angle = atan(a / (b + distanceBetweenCenterAndSonic)) * radToDeg;

      Steppers::Rotate((int)angle / 2);
      shoot();
      delay(1000);
      break;
    }
  }
};

colorSensor cs;

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
  lcd.print("Vektor 2.0");

  Servos::setup(90, 90, 90);
  Servos::move(60, 0, 0);


  cs.init_TCS34725();
  cs.get_TCS34725ID();
  cs.get_Colors();

  bool lijevaStanica = false;



  Steppers::runUntilLine();
  Steppers::move(1100);
  Steppers::runUntilEnd();

  Steppers::Rotate(-90);
  //     /$$$$$$  /$$$$$$$$ /$$$$$$  /$$$$$$$  /$$$$$$$$
  //    /$$__  $$|__  $$__//$$__  $$| $$__  $$|__  $$__/
  //   | $$  \__/   | $$  | $$  \ $$| $$  \ $$   | $$
  //   |  $$$$$$    | $$  | $$$$$$$$| $$$$$$$/   | $$
  //    \____  $$   | $$  | $$__  $$| $$__  $$   | $$
  //    /$$  \ $$   | $$  | $$  | $$| $$  \ $$   | $$
  //   |  $$$$$$/   | $$  | $$  | $$| $$  | $$   | $$
  //    \______/    |__/  |__/  |__/|__/  |__/   |__/

  int brojUlica = 0;
  for (brojUlica = 0; brojUlica < 3; brojUlica++) {
    Steppers::followUntilLeftTurn();
    Steppers::followUntilEnd();
    Steppers::alignWithLine();
    Steppers::korekcija(110);
    Steppers::korekcija(110);
    lijevaStanica = !lijevaStanica;
    if (Servos::pickupBall()) break;  //Lopta 1
    Steppers::RotateUntilSens(3);
    Steppers::followUntilEnd();
    Steppers::alignWithLine();
    Steppers::korekcija(110);
    Steppers::korekcija(110);
    lijevaStanica = !lijevaStanica;
    if (Servos::pickupBall()) break;  //Lopta 2
    Steppers::Rotate(180, true);
    Steppers::followUntilRightTurn();
  }
  Steppers::Rotate(180, true);
  if (lijevaStanica) Steppers::followUntilLeftTurn();
  else Steppers::followUntilRightTurn();

  for (brojUlica; brojUlica < 3; brojUlica++) {
    lcd.clear();
    lcd.print("desilo");
    Steppers::followUntilEnd();
    Steppers::move(200);
    Steppers::runUntilEnd();
  }
  brojUlica = 0;

  Steppers::korekcija(130);
  Servos::releaseHigh();


  //     /$$$$$$            /$$        /$$$$$$   /$$$$$$  /$$$$$$$
  //    /$$__  $$          | $$       /$$__  $$ /$$__  $$| $$__  $$
  //   |__/  \ $$          | $$      | $$  \ $$| $$  \ $$| $$  \ $$
  //     /$$$$$$/          | $$      | $$  | $$| $$  | $$| $$$$$$$/
  //    /$$____/           | $$      | $$  | $$| $$  | $$| $$____/
  //   | $$                | $$      | $$  | $$| $$  | $$| $$
  //   | $$$$$$$$ /$$      | $$$$$$$$|  $$$$$$/|  $$$$$$/| $$
  //   |________/|__/      |________/ \______/  \______/ |__/
  int brojProvjerenih = 0;
  Steppers::Rotate(180);
  lijevaStanica = true;
  for (brojUlica = 0; brojUlica < 3; brojUlica++) {
    Steppers::followUntilLeftTurn();
    Steppers::followUntilEnd();
    Steppers::alignWithLine();
    Steppers::korekcija(110);
    Steppers::korekcija(110);
    lijevaStanica = false;
    brojProvjerenih++;
    if (Servos::pickupBall()) break;
    Steppers::Rotate(170, true);
    Steppers::followUntilEnd();
    Steppers::move(400);
    Steppers::runUntilEnd();
    Steppers::followUntilEnd();
    Steppers::alignWithLine();
    Steppers::korekcija(110);
    Steppers::korekcija(110);
    lijevaStanica = true;
    brojProvjerenih++;
    if (Servos::pickupBall()) break;
    if (brojUlica == 2) break;
    Steppers::Rotate(180, true);
    Steppers::followUntilRightTurn();
  }
  Steppers::Rotate(180, true);
  if (lijevaStanica) Steppers::followUntilLeftTurn();
  else Steppers::followUntilRightTurn();

  for (brojUlica; brojUlica >= 0; brojUlica--) {
    Steppers::followUntilEnd();
    Steppers::move(200);
    Steppers::runUntilEnd();
  }
  brojUlica = 0;
  Steppers::korekcija(130);
  Servos::releaseHigh();

  Steppers::Rotate(180);
  lijevaStanica = true;


  //     /$$$$$$            /$$        /$$$$$$   /$$$$$$  /$$$$$$$
  //    /$$__  $$          | $$       /$$__  $$ /$$__  $$| $$__  $$
  //   |__/  \ $$          | $$      | $$  \ $$| $$  \ $$| $$  \ $$
  //     /$$$$$$/          | $$      | $$  | $$| $$  | $$| $$$$$$$/
  //    /$$____/           | $$      | $$  | $$| $$  | $$| $$____/
  //   | $$                | $$      | $$  | $$| $$  | $$| $$
  //   | $$$$$$$$ /$$      | $$$$$$$$|  $$$$$$/|  $$$$$$/| $$
  //   |________/|__/      |________/ \______/  \______/ |__/
  ///////3


  int prosaoPraznih = 0;
  for (int i = 0; i < brojProvjerenih / 2; i++) {
    Steppers::followUntilEnd();
    Steppers::move(200);
    Steppers::runUntilEnd();
  }

  if (brojProvjerenih % 2 == 1) {
    Steppers::followUntilRightTurn();
    Steppers::followUntilEnd();
    Steppers::alignWithLine();
    Steppers::korekcija(110);
    Steppers::korekcija(110);
    lijevaStanica = true;
    prosaoPraznih++;
    if (Servos::pickupBall()) {
      if (brojProvjerenih / 2 == 1) brojProvjerenih = 200;
      else brojProvjerenih = 100;
    }
    if (brojProvjerenih < 4) {
      Steppers::Rotate(180, true);
      Steppers::followUntilRightTurn();
    }
  }


  brojUlica = brojProvjerenih / 2;
  for (brojUlica; brojUlica < 3 - brojProvjerenih % 2; brojUlica++) {
    Steppers::followUntilLeftTurn();
    Steppers::followUntilEnd();
    Steppers::alignWithLine();
    Steppers::korekcija(110);
    Steppers::korekcija(110);
    lijevaStanica = false;
    prosaoPraznih++;
    if (Servos::pickupBall()) break;
    Steppers::Rotate(180, true);
    Steppers::followUntilEnd();
    Steppers::move(400);
    Steppers::runUntilEnd();
    Steppers::followUntilEnd();
    Steppers::alignWithLine();
    Steppers::korekcija(110);
    Steppers::korekcija(110);
    lijevaStanica = true;
    prosaoPraznih++;
    if (Servos::pickupBall()) break;
    if (brojUlica == 2) break;
    Steppers::Rotate(180, true);
    Steppers::followUntilRightTurn();
  }
  Steppers::Rotate(180, true);
  if (lijevaStanica) Steppers::followUntilLeftTurn();
  else Steppers::followUntilRightTurn();

  if (brojProvjerenih == 100) brojUlica = 0;

  if (brojProvjerenih == 200) brojUlica = 1;
  if (prosaoPraznih == 5) brojUlica++;

  for (brojUlica; brojUlica >= 0; brojUlica--) {
    Steppers::followUntilEnd();
    Steppers::move(200);
    Steppers::runUntilEnd();
  }
  brojUlica = 0;

  Steppers::korekcija_ciljanje(130);
  Servos::releaseHigh();

  //     /$$$$$$$   /$$$$$$  /$$$$$$$  /$$$$$$$$         /$$          /$$$$$$   /$$$$$$  /$$$$$$$$ /$$$$$$  /$$    /$$
  //    | $$__  $$ /$$__  $$| $$__  $$|__  $$__/       /$$$$         /$$__  $$ /$$__  $$|__  $$__//$$__  $$| $$   | $$
  //    | $$  \ $$| $$  \ $$| $$  \ $$   | $$         |_  $$        | $$  \__/| $$  \ $$   | $$  | $$  \ $$| $$   | $$
  //    | $$$$$$$/| $$$$$$$$| $$$$$$$/   | $$           | $$        | $$ /$$$$| $$  | $$   | $$  | $$  | $$|  $$ / $$/
  //    | $$____/ | $$__  $$| $$__  $$   | $$           | $$        | $$|_  $$| $$  | $$   | $$  | $$  | $$ \  $$ $$/
  //    | $$      | $$  | $$| $$  \ $$   | $$           | $$        | $$  \ $$| $$  | $$   | $$  | $$  | $$  \  $$$/
  //    | $$      | $$  | $$| $$  | $$   | $$          /$$$$$$      |  $$$$$$/|  $$$$$$/   | $$  |  $$$$$$/   \  $/
  //    |__/      |__/  |__/|__/  |__/   |__/         |______/       \______/  \______/    |__/   \______/     \_/



  Steppers::Rotate(-90);
  Steppers::move(2000);
  Steppers::runUntilEnd();
  Steppers::runUntilLine();
  Steppers::alignWithLine();
  Steppers::resetSpeed();
  Steppers::move(900);
  Steppers::runUntilEnd();
  Steppers::RotateUntilSens(-1);

  Steppers::followUntilEnd();
  Steppers::alignWithLine();
  Steppers::move(-20);
  Steppers::runUntilEnd();
  Steppers::Rotate(-1);
  Servos::pickupBall();

  Steppers::Rotate(180);



  Steppers::followUntilEnd();
  Steppers::alignWithLine();
  Steppers::resetSpeed();
  Steppers::move(-100);
  Steppers::runUntilEnd();
  Steppers::korekcija_ciljanje(110);
  Servos::shoot();
  while (true) {

    Steppers::korekcija_ciljanje(110);

    while (!Servos::pickupBall()) {}
    Servos::shoot();
  }
}


void loop() {
}