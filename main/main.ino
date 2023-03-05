

#define RIGHT_STEPPER_DIR 2
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
  }
  static void followLine() {
    if (digitalRead(LEFT_IN_IR_SENSOR)) {
      ICR3 = 100;
      ICR4 = 130;
    }
    if (digitalRead(RIGHT_IN_IR_SENSOR)) {
      ICR3 = 130;
      ICR4 = 100;
    }
  }

  static void followUntilTurn() {
    do {
      followLine();
    } while (!digitalRead(LEFT_OUT_IR_SENSOR) && !digitalRead(RIGHT_OUT_IR_SENSOR));
    if (digitalRead(LEFT_OUT_IR_SENSOR)) {
      ICR3 = 800;
      ICR4 = 60;
    } else {
      ICR3 = 60;
      ICR4 = 800;
    }
    delay(500);
  }
};


void setup() {
  setupPins();
  Steppers::setup();

  ICR3 = 100;
  ICR4 = 100;

  Serial.begin(9600);
}

void loop() {
  
  while (true) {Steppers::followUntilTurn();
  }
}