
//  Motor Drivers: Dual TB6612FNG 
//  Wall IR Sensors: Sharp GP2Y0A21YK0F
//  Line IR Sensors: QRE1113

// HIGH LEVEL
int EXIT = 1;  // A: 1  B: 3  C: 5
int currentExit = 0;

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
int IR_WALL_NEAR_THRESHOLD = 600;
int IR_LINE_THRESHOLD = 780;
int IR_LINE_THRESHOLD_STRICT = 930;
int IR_RED_LOW_THRESHOLD = 0;   //TUNE
int IR_RED_HIGH_THRESHOLD = 600;  //TUNE

// Control states
boolean followLine;  // follows line if true, follows wall otherwise
boolean testingExit;
boolean takeTurn;
boolean exiting;
boolean followWall;
boolean completedExit1;
boolean completedExit2;
boolean turnedAround;
boolean completedTunnel;
boolean followLeft;
boolean onRed;

long turnWindowCloseTime = millis();
long followRightCloseTime = millis();
long followLeftCloseTime = millis();

//PID
double lastLineError = 0;
double lastWallError = 0;

//Speedup
long lastLargeCorrectionMillis = millis();
double boost_straight = 0.2;
double requiredAvg = 0.9;

// Rolling average
double avg_history_size = 250;
int history[250];
int avg_pointer = 0;
int avg_sum = 0;
int lastMode = 0;
int historySum[250];

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

  historySum[0] = 99;
}

void loop() {
  LEDControl();

  if(onRed) {
    drive(0,0);
    return;
  }
  
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
  bool lineLeft_strict = lineLeftRaw > IR_LINE_THRESHOLD_STRICT;
  bool lineMiddle_strict = lineMiddleRaw > IR_LINE_THRESHOLD_STRICT;
  bool lineRight_strict = lineRightRaw > IR_LINE_THRESHOLD_STRICT;
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

  // rolling average
  int currentMode = 0;
  if(lineLeft && !lineMiddle) currentMode = 1;
  else if(!lineLeft && !lineMiddle) currentMode = 2;

  int changed = 0;
  if(currentMode != lastMode) changed = 1;
  lastMode = currentMode;
  
  avg_sum += changed;
  avg_sum -= history[avg_pointer];
  if(lineLeft && lineRight && lineMiddle) historySum[0] = 999;
  
  history[avg_pointer] = changed;
  if(avg_pointer + 1 >= avg_history_size) {
    avg_pointer = 0;
  }
  else avg_pointer++;

  int lowChange = 999;
  int highChange = 0;

  for(int i = avg_history_size-1; i >= 0; i--) {
    if(i == 0) {
      historySum[i] = avg_sum;
    }
    else {
      historySum[i] = historySum[i-1];
    }
    if(historySum[i] < lowChange) lowChange = historySum[i];
    if(historySum[i] > highChange) highChange = historySum[i];
  }

//  Serial.println(highChange - lowChange);

  if(followLine) {
    if(completedTunnel) {
        int red = 0;
        if(lineLeftRaw < IR_RED_HIGH_THRESHOLD && lineLeftRaw > IR_RED_LOW_THRESHOLD) red++;
        if(lineMiddleRaw < IR_RED_HIGH_THRESHOLD && lineMiddleRaw > IR_RED_LOW_THRESHOLD) red++;
        if(lineRightRaw < IR_RED_HIGH_THRESHOLD && lineRightRaw > IR_RED_LOW_THRESHOLD) red++;
        if(red == 3) {
          drive(0,0);
          onRed = true;
          drive(1,1);
          delay(300);
          return;
        }
        else if(red == 2) drive(0.5, 0.2); //TUNE
    }
    if(followRightCloseTime < millis() && turnedAround && !completedExit2) {
      drive(-0.4, 0.4);
      if(!lineMiddle) return;
      followLeft = true;
        
      completedExit2 = true;
    }
    if(followRightCloseTime > millis()) {
      if(lineMiddle) {
        motorLeftOutput = 0.8;
        motorRightOutput = -0.3;
      }
      else {
        motorLeftOutput = -0.3;
        motorRightOutput = 0.8;
      }
    }
    else {
      
      if(completedExit2 && wallLeft && wallRight && !lineMiddle && !lineRight && !lineLeft && !completedTunnel) {
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

      if(!completedExit2 && wallFront && !turnedAround) {
        turnedAround = true;
        Serial.println("Turning around");
        drive(-0.3, 0.3);
        delay(500);
        while(analogRead(PIN_ANALOG_LINE_LEFT) < IR_LINE_THRESHOLD) {}
        drive(0,0);
        
        followRightCloseTime = millis() + 1500;   // TUNE
        return;
      }
  
//      if(!turnedAround && !completedExit2 && turnWindowCloseTime < millis()) {
//        if(!testingExit && lines == 2) {
//          drive(0.3, 0.3);
//          delay(700);
//          drive(0, 0);
//          testingExit = true;
//        }
//        else if(testingExit) {
//          testingExit = false;
//          if(lines == 0) {
//            Serial.println("Corner");
//            drive(-0.3, -0.3);
//            delay(800);
//            drive(0, 0);
//            lastLargeCorrectionMillis = millis();
//            turnWindowCloseTime = millis() + 1000;
//            LEDFlash(100, 1);
//          }
//          else {
//            Serial.println("Exit: " + String(currentExit));
//            currentExit++;
//            if(currentExit == EXIT) {
//              LEDFlash(3000, 1);
//              drive(-0.3, -0.3);
//              delay(1000);
//              drive(0, 0);
//              if(EXIT != 3) {
//                // exit is A or C, right side
//                followRightCloseTime = millis() + 3000;
//                Serial.println("GOING RIGHT");
//              }
//              else {
//                // exit is B, left side
//                followLeftCloseTime = millis() + 3000;
//                Serial.println("GOING LEFT");
//              }
//              if(!completedExit1) completedExit1 = true; 
//              else completedExit2 = true;
//              EXIT += 2;
//              currentExit = EXIT - 1;
//            }
//          }
//        } 
//      }
      if(!followLeft) {
        if(lineLeft && !lineMiddle) {
          motorLeftOutput = 0.24;
          motorRightOutput = 0.24;
          if(highChange - lowChange < 4) {
            motorLeftOutput += boost_straight;
            motorRightOutput += boost_straight;
          }
        }
        else if(lineMiddle) {
          motorLeftOutput = 0.6;
          motorRightOutput = -0.6;
        }
        else if(!lineLeft && !lineMiddle) {
          motorLeftOutput = -0.6;
          motorRightOutput = 0.6;
        }
      }
      else {
        if(lineLeft && lineMiddle || lineLeft && !lineMiddle) {
          motorLeftOutput = -0.5;
          motorRightOutput = 0.5;
        }
        else if(!lineLeft && lineMiddle) {
          motorLeftOutput = 0.24;
          motorRightOutput = 0.24;
        }
        else if(!lineLeft && !lineMiddle) {
          motorLeftOutput = 0.5;
          motorRightOutput = -0.5;
        }
      }
      

//      if(lines == 0) {
//        error = 0;
//        lines = 1;
//      }
//      error /= lines;
//  
//      double P = kP * error;
//      double D = kD * (lastLineError - error);
//  
//      double output = P + D;
//
//      motorLeftOutput += -output;
//      motorRightOutput += output;
  
//      if(abs(error) < 1 && abs(error) > 0) lastLargeCorrectionMillis = millis();
    }
  }
  else if(followWall) {

    LEDFlash(20, 1);

    if(lineLeft_strict || lineMiddle_strict || lineRight_strict) {
      // sweep left then right until you see the line
//      long sweepLeftEndTime = millis() + 200; //  TUNE
//      while(analogRead(PIN_ANALOG_LINE_LEFT) < IR_LINE_THRESHOLD && PIN_ANALOG_LINE_MIDDLE < IR_LINE_THRESHOLD 
//            && analogRead(PIN_ANALOG_LINE_RIGHT) < IR_LINE_THRESHOLD && millis() < sweepLeftEndTime) drive(0.3, -0.3);  // TUNE
//      while(analogRead(PIN_ANALOG_LINE_LEFT) < IR_LINE_THRESHOLD && PIN_ANALOG_LINE_MIDDLE < IR_LINE_THRESHOLD 
//            && analogRead(PIN_ANALOG_LINE_RIGHT) < IR_LINE_THRESHOLD) drive(0.5, 0.3);  // TUNE
      completedTunnel = true;
      followWall = false;
      followLine = true;
      followLeft = false;
    }

    db += "wall\n";

    double baseSpeed = 0.25;
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
//  delay(500);
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
