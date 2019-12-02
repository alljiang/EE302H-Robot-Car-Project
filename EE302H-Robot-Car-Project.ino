
//  Motor Drivers: Dual TB6612FNG 
//  Wall IR Sensors: Sharp GP2Y0A21YK0F
//  Line IR Sensors: QRE1113

// HIGH LEVEL
char EXIT = 'B';
char currentExit = 64;

// DIO
int PIN_DIO_PWMA = 5; // Motor Driver, Motor Left PWM
int PIN_DIO_PWMB = 6; // Motor Driver, Motor Right PWM
int PIN_DIO_AIN2 = 8; // Motor Driver, Motor Left H-Bridge B
int PIN_DIO_AIN1 = 9; // Motor Driver, Motor Left H-Bridge A
int PIN_DIO_BIN1 = 11; // Motor Driver, Motor Right H-Bridge A
int PIN_DIO_BIN2 = 12; // Motor Driver, Motor Right H-Bridge B
int PIN_DIO_LED = 3; // LED on shield

//  Analog
int PIN_ANALOG_WALL_IR_LEFT = 0;  // Wall IR, Vout
int PIN_ANALOG_WALL_IR_FRONT = 1;  // Wall IR, Vout
int PIN_ANALOG_WALL_IR_RIGHT = 2;  // Wall IR, Vout
int PIN_ANALOG_LINE_RIGHT = 5;  // Line IR, Vout
int PIN_ANALOG_LINE_MIDDLE = 4;  // Line IR, Vout
int PIN_ANALOG_LINE_LEFT = 3;  // Line IR, Vout

//  Threshold values for sensors, trial/error defined
int IR_WALL_THRESHOLD = 200;
int IR_WALL_NEAR_THRESHOLD = 500;
int IR_LINE_THRESHOLD = 850;

// Control states
boolean followLine;  // follows line if true, follows wall otherwise
boolean testingExit;
boolean takeTurn;
boolean exiting;
boolean followWall;
boolean completedExit1;
boolean completedExit2;
boolean turnedAround;

long turnWindowCloseTime = millis();
long followRightCloseTime = millis();
long followLeftCloseTime = millis();

//PID
double lastLineError = 0;
double lastWallError = 0;

//Speedup
long lastLargeCorrectionMillis = millis();

void setup() {
  Serial.begin(230400);

  //  Pin Initializations
  pinMode(PIN_DIO_PWMA, OUTPUT);
  pinMode(PIN_DIO_PWMB, OUTPUT);
  pinMode(PIN_DIO_AIN2, OUTPUT);
  pinMode(PIN_DIO_AIN1, OUTPUT);
  pinMode(PIN_DIO_BIN2, OUTPUT);
  pinMode(PIN_DIO_BIN2, OUTPUT); 
  pinMode(PIN_DIO_LED, OUTPUT);

  digitalWrite(PIN_DIO_AIN1, LOW);
  digitalWrite(PIN_DIO_AIN2, LOW);
  digitalWrite(PIN_DIO_BIN2, LOW);
  digitalWrite(PIN_DIO_BIN1, LOW);
  digitalWrite(PIN_DIO_PWMA, LOW);
  digitalWrite(PIN_DIO_PWMB, LOW);
  digitalWrite(PIN_DIO_LED, LOW);

  followLine = true;
  followWall = false;
}

void loop() {
  LEDControl();
  
  //  Read IR sensor values
  float lineLeftRaw = lineLeftRaw * 0. + 1 * analogRead(PIN_ANALOG_LINE_LEFT);
  float lineMiddleRaw = lineMiddleRaw * 0. + 1 * analogRead(PIN_ANALOG_LINE_MIDDLE);
  float lineRightRaw = lineRightRaw * 0. + 1 * analogRead(PIN_ANALOG_LINE_RIGHT);

  int wallFrontRaw = analogRead(PIN_ANALOG_WALL_IR_FRONT);
  int wallLeftRaw = analogRead(PIN_ANALOG_WALL_IR_LEFT);
  int wallRightRaw = analogRead(PIN_ANALOG_WALL_IR_RIGHT);

  bool lineLeft = lineLeftRaw > IR_LINE_THRESHOLD;
  bool lineMiddle = lineMiddleRaw > IR_LINE_THRESHOLD;
  bool lineRight = lineRightRaw > IR_LINE_THRESHOLD;
  bool wallFront = wallFrontRaw > IR_WALL_NEAR_THRESHOLD;
  bool wallLeft = wallLeftRaw > IR_WALL_THRESHOLD;
  bool wallRight = wallRightRaw > IR_WALL_THRESHOLD;

  String db = "";

  db += "   Line Left: " + String(lineLeftRaw) + "\n";
  db += " Line Middle: " + String(lineMiddleRaw) + "\n";
  db += "  Line Right: " + String(lineRightRaw) + "\n\n";
  db += "   Wall Left: " + String(wallLeftRaw) + "\n";
  db += "  Wall Front: " + String(wallFrontRaw) + "\n";
  db += "  Wall Right: " + String(wallRightRaw) + "\n\n";

  double motorLeftOutput = 0;
  double motorRightOutput = 0;

  if(followLine) {
  
    if(followRightCloseTime > millis()) {
      if(lineMiddle) {
        motorLeftOutput = 0.8;
        motorRightOutput = -0.3;
      }
      else {
        motorLeftOutput = -0.3;
        motorRightOutput = 0.8;
      }
      lastLargeCorrectionMillis = millis();
    }
    else if(followLeftCloseTime > millis()) {
      if(lineMiddle) {
        motorLeftOutput = -0.3;
        motorRightOutput = 0.8;
      }
      else {
        motorLeftOutput = 0.8;
        motorRightOutput = -0.3;
      }
      lastLargeCorrectionMillis = millis();
    }
    else {
      if(completedExit1 && completedExit2 && !lineLeft && !lineMiddle && !lineRight && wallLeft && wallRight) {
        followWall = true;
        followLine = false;
        return;
      }
  
      db += "line\n";
  
      double baseSpeed = 0.24;
      long straightTime = millis() - lastLargeCorrectionMillis;
      double boost = straightTime * 0.0001;
      motorLeftOutput = baseSpeed + boost;
      motorRightOutput = baseSpeed + boost;
  
      double kP = 1.4;
      double kD = 0;
  
      double error = 0;
      double lines = 0;
      if(lineLeft) {
        lines++;
        error += 1;
      }
      if(lineMiddle) {
        lines++;
      }
      if(lineRight) {
        lines++;
        error -= 1;
      }

      if(completedExit1 && !completedExit2 && wallFrontRaw && !turnedAround) {
        turnedAround = true;
        Serial.println("Turning around");
        if(EXIT % 2 == 0) {
          // exit is A or C, right side
          drive(-0.3, 0.3);
          delay(300);
          while(analogRead(PIN_ANALOG_LINE_LEFT) < IR_LINE_THRESHOLD) {}
          drive(0, 0);
          
          followRightCloseTime = millis() + 2000;
        }
        else {
          // exit is B, left side
          followLeftCloseTime = millis() + 2000;
        }
      }
  
      if(!completedExit2 && turnWindowCloseTime < millis()) {
        exiting = false;
        if(!testingExit && lines == 2) {
          drive(0.3, 0.3);
          delay(500);
          drive(0, 0);
          testingExit = true;
        }
        else if(testingExit) {
          testingExit = false;
          if(lines == 0) {
            Serial.println("Corner");
            drive(-0.3, -0.3);
            delay(800);
            drive(0, 0);
            lastLargeCorrectionMillis = millis();
            turnWindowCloseTime = millis() + 1000;
            LEDFlash(100, 1);
          }
          else {
            Serial.println("Exit");
            currentExit++;
            if(currentExit == EXIT) {
              drive(-0.3, -0.3);
              delay(1000);
              drive(0, 0);
              if(EXIT % 2 == 1) {
                // exit is A or C, right side
                followRightCloseTime = millis() + 3000;
                Serial.println("GOING RIGHT");
              }
              else {
                // exit is B, left side
                followLeftCloseTime = millis() + 3000;
                Serial.println("GOING LEFT");
              }
              if(!completedExit1) completedExit1 = true; 
              else completedExit2 = true;
              EXIT++;
            }
            LEDFlash(3000, 1);
          }
        } 
        
      }

      if(lines == 0) {
        error = 0;
        lines = 1;
      }
      error /= lines;
  
      double P = kP * error;
      double D = kD * (lastLineError - error);
  
      double output = P + D;
  
      motorLeftOutput += -output;
      motorRightOutput += output;
  
      if(abs(error) < 1 && abs(error) > 0) lastLargeCorrectionMillis = millis();
    }
  }
  else if(followWall) {

    db += "wall\n";

    double baseSpeed = 0.3;
    motorLeftOutput = baseSpeed;
    motorRightOutput = baseSpeed;
    
    double kP = 0.0007;
    double kD = 0.0002;
    
    double error = wallRightRaw - wallLeftRaw;

    double P = kP * error;
    double D = kD * (error - lastWallError);

    double output = P + D;

    lastWallError = error;

    motorLeftOutput += -output;
    motorRightOutput += output;
  }
  db += "--------------------------------------------\n";
//  Serial.print(db);

  drive(motorLeftOutput, motorRightOutput);
}

float motorMultiplier = 1;

/*
 * Input float between [-1, 1], Forward is positive
 */
void setMotorLeft(float val) {
  val *= motorMultiplier;
  if(val < 0) {
    digitalWrite(PIN_DIO_BIN2, LOW);
    digitalWrite(PIN_DIO_BIN1, HIGH);
  }
  else {
    digitalWrite(PIN_DIO_BIN2, HIGH);
    digitalWrite(PIN_DIO_BIN1, LOW);
  }
  int dutyCycle = (int)(abs(val) * 255);
  analogWrite(PIN_DIO_PWMB, dutyCycle);

//  Serial.println("  Motor Left Output: " + String(val));
}

/*
 * Input float between [-1, 1], Forward is positive
 */
void setMotorRight(float val) {
  val *= motorMultiplier;
  if(val < 0) {
    digitalWrite(PIN_DIO_AIN1, LOW);
    digitalWrite(PIN_DIO_AIN2, HIGH);
  }
  else {
    digitalWrite(PIN_DIO_AIN1, HIGH);
    digitalWrite(PIN_DIO_AIN2, LOW);
  }
  if(abs(val) > 1) {
    val = 1;
  }
  analogWrite(PIN_DIO_PWMA, (int)(abs(val) * 255));
  
//  Serial.println(" Motor Right Output: " + String(val));
}

void drive(float l, float r) {
  setMotorLeft(l);
  setMotorRight(r);
}

double LED_period = 0;
int times_remaining = 0;
long lastToggle = 0;
boolean lastLEDState = false;

void LEDControl() {
  if(millis() - lastToggle > LED_period && (times_remaining > 0 || lastLEDState)) {
    lastLEDState = !lastLEDState;
    if(lastLEDState) digitalWrite(PIN_DIO_LED, HIGH);
    else digitalWrite(PIN_DIO_LED, LOW);
    lastToggle = millis();
    
    if(lastLEDState) times_remaining--;
  }
}

void LEDFlash(double time_m, int times) {
  LED_period = time_m;
  times_remaining = times;
}
