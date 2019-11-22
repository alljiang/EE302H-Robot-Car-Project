
//  Motor Drivers: Dual TB6612FNG 
//  Wall IR Sensors: Sharp GP2Y0A21YK0F
//  Line IR Sensors: QRE1113

// DIO
int PIN_DIO_PWMA = 5; // Motor Driver, Motor Left PWM
int PIN_DIO_PWMB = 6; // Motor Driver, Motor Right PWM
int PIN_DIO_AIN2 = 8; // Motor Driver, Motor Left H-Bridge B
int PIN_DIO_AIN1 = 9; // Motor Driver, Motor Left H-Bridge A
int PIN_DIO_BIN1 = 11; // Motor Driver, Motor Right H-Bridge A
int PIN_DIO_BIN2 = 12; // Motor Driver, Motor Right H-Bridge B

//  Analog
int PIN_ANALOG_WALL_IR_LEFT = 0;  // Wall IR, Vout
int PIN_ANALOG_WALL_IR_FRONT = 1;  // Wall IR, Vout
int PIN_ANALOG_WALL_IR_RIGHT = 2;  // Wall IR, Vout
int PIN_ANALOG_LINE_RIGHT = 5;  // Line IR, Vout
int PIN_ANALOG_LINE_MIDDLE = 4;  // Line IR, Vout
int PIN_ANALOG_LINE_LEFT = 3;  // Line IR, Vout

//  Threshold values for sensors, trial/error defined
int IR_WALL_THRESHOLD = 600;
int IR_LINE_THRESHOLD = 900;

// Control states
boolean followLine;  // follows line if true, follows wall otherwise

void setup() {
  Serial.begin(115200);

  //  Pin Initializations
  pinMode(PIN_DIO_PWMA, OUTPUT);
  pinMode(PIN_DIO_PWMB, OUTPUT);
  pinMode(PIN_DIO_AIN2, OUTPUT);
  pinMode(PIN_DIO_AIN1, OUTPUT);
  pinMode(PIN_DIO_BIN2, OUTPUT);
  pinMode(PIN_DIO_BIN2, OUTPUT); 

  digitalWrite(PIN_DIO_AIN1, LOW);
  digitalWrite(PIN_DIO_AIN2, LOW);
  digitalWrite(PIN_DIO_BIN2, LOW);
  digitalWrite(PIN_DIO_BIN1, LOW);
  digitalWrite(PIN_DIO_PWMA, LOW);
  digitalWrite(PIN_DIO_PWMB, LOW);
}

void loop() {
  //  Read IR sensor values
  int lineLeftRaw = lineLeftRaw * 0. + 1 * analogRead(PIN_ANALOG_LINE_LEFT);
  int lineMiddleRaw = lineMiddleRaw * 0. + 1 * analogRead(PIN_ANALOG_LINE_MIDDLE);
  int lineRightRaw = lineRightRaw * 0. + 1 * analogRead(PIN_ANALOG_LINE_RIGHT);

  int wallFrontRaw = analogRead(PIN_ANALOG_WALL_IR_FRONT);
  int wallLeftRaw = analogRead(PIN_ANALOG_WALL_IR_LEFT);
  int wallRightRaw = analogRead(PIN_ANALOG_WALL_IR_RIGHT);

  bool lineLeft = lineLeftRaw > IR_LINE_THRESHOLD;
  bool lineMiddle = lineMiddleRaw > IR_LINE_THRESHOLD;
  bool lineRight = lineRightRaw > IR_LINE_THRESHOLD;
  bool wallFront = wallFrontRaw > IR_WALL_THRESHOLD;
  bool wallLeft = wallLeftRaw > IR_WALL_THRESHOLD;
  bool wallRight = wallRightRaw > IR_WALL_THRESHOLD;

  Serial.println("   Line Left: " + String(lineLeft));
  Serial.println(" Line Middle: " + String(lineMiddle));
  Serial.println("  Line Right: " + String(lineRight));
  Serial.println();
  Serial.println("   Wall Left: " + String(wallLeft));
  Serial.println("  Wall Front: " + String(wallFront));
  Serial.println("  Wall Right: " + String(wallRight));
  Serial.println();

  followLine = true;
  
  if(followLine) {
     /*
     *  Line-following state machine
     *  L M R   Left  Right   Description
     *  0 0 0   .4   -.4      see nothing, ur screwed
     *  0 0 1   .4   -.4      line on right, turn right
     *  0 1 0   .4    .4      line middle, go straight
     *  1 0 0  -.4    .4      line on left, turn left
     *  0 1 1   .4   -.4      hard right, pivot
     *  1 1 0  -.4    .4      hard left, pivot
     *  1 0 1   .4   -.4     wtf is happening
     *  1 1 1   .4   -.4     RIP
     */

    double motorLeftOutput = 0;
    double motorRightOutput = 0;
    // 0 = white, 1 = black
    if( !lineLeft && !lineMiddle && !lineRight ) {
      //  0 0 0
      motorLeftOutput = 0.3;
      motorRightOutput = 0.3;
    }
    else if ( !lineLeft && !lineMiddle && lineRight ) {
      //  0 0 1
      motorLeftOutput = 0.3;
      motorRightOutput = 0.;
    }
    else if ( !lineLeft && lineMiddle && !lineRight ) {
      // 0 1 0      
      motorLeftOutput = 0.3;
      motorRightOutput = 0.3;
    }
    else if ( lineLeft && !lineMiddle && !lineRight ) {
      // 1 0 0
      motorLeftOutput = 0.;
      motorRightOutput = 0.3;
    }
    else if ( !lineLeft && lineMiddle && lineRight ) {
      // 0 1 1
      motorLeftOutput = 0.4;
      motorRightOutput = 0.;
    }
    else if ( lineLeft && lineMiddle && !lineRight ) {
      // 1 1 0
      motorLeftOutput = 0.;
      motorRightOutput = 0.4;
    }
    else if ( lineLeft && !lineMiddle && lineRight ) {
      // 1 0 1
      motorLeftOutput = 0.2;
      motorRightOutput = 0.3;
    }
    else if ( lineLeft && lineMiddle && lineRight ) {
      // 1 1 1
      motorLeftOutput = 0.3;
      motorRightOutput = 0.3;
    }
    setMotorLeft(motorLeftOutput);
    setMotorRight(motorRightOutput);
  }
  Serial.println("--------------------------------------------");
//  delay(0);
}

float motorMultiplier = 3;

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

  Serial.println("  Motor Left Output: " + String(val));
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
  
  Serial.println(" Motor Right Output: " + String(val));
}
