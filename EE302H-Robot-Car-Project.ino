
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
int PIN_ANALOG_WALL_IR_BACK = 0;  // Wall IR, Vout
int PIN_ANALOG_WALL_IR_FRONT = 1;  // Wall IR, Vout
int PIN_ANALOG_LINE_RIGHT = 3;  // Line IR, Vout
int PIN_ANALOG_LINE_MIDDLE = 4;  // Line IR, Vout
int PIN_ANALOG_LINE_LEFT = 5;  // Line IR, Vout

//  Threshold values for sensors, trial/error defined
int IR_WALL_THRESHOLD = 300;
int IR_LINE_THRESHOLD = 600;

void setup() {
  Serial.begin(115200);

  //  Pin Initializations
  pinMode(PIN_DIO_PWMA, OUTPUT);
  pinMode(PIN_DIO_PWMB, OUTPUT);
  pinMode(PIN_DIO_AIN2, OUTPUT);
  pinMode(PIN_DIO_AIN1, OUTPUT);
  pinMode(PIN_DIO_BIN2, OUTPUT);
  pinMode(PIN_DIO_BIN2, OUTPUT); 

  digitalWrite(PIN_DIO_AIN1, HIGH);
  digitalWrite(PIN_DIO_AIN2, LOW);
  digitalWrite(PIN_DIO_BIN2, HIGH);
  digitalWrite(PIN_DIO_BIN1, LOW);
}

void loop() {
  int lineLeftRaw = analogRead(PIN_ANALOG_LINE_LEFT);
  int lineMiddleRaw = analogRead(PIN_ANALOG_LINE_MIDDLE);
  int lineRightRaw = analogRead(PIN_ANALOG_LINE_RIGHT);

  int wallFrontRaw = analogRead(PIN_ANALOG_WALL_IR_FRONT);
  int wallBackRaw = analogRead(PIN_ANALOG_WALL_IR_BACK);

  bool lineLeft = lineLeftRaw > IR_LINE_THRESHOLD;
  bool lineMid = lineMidRaw > IR_LINE_THRESHOLD;
  bool lineRight = lineRightRaw > IR_LINE_THRESHOLD;
  bool wallFront = wallFrontRaw < IR_WALL_THRESHOLD;
  bool wallBack = wallBackRaw < IR_WALL_THRESHOLD;
}

/*
 * Input float between [-1, 1], Forward is positive
 */
void setMotorLeft(float val) {
  if(val < 0) {
    digitalWrite(PIN_DIO_AIN1, LOW);
    digitalWrite(PIN_DIO_AIN2, HIGH);
  }
  else {
    digitalWrite(PIN_DIO_AIN1, HIGH);
    digitalWrite(PIN_DIO_AIN2, LOW);
  }
  analogWrite(PIN_DIO_PWMA, (int)(abs(val) * 255));
}

/*
 * Input float between [-1, 1], Forward is positive
 */
void setMotorRight(float val) {
  if(val < 0) {
    digitalWrite(PIN_DIO_BIN2, LOW);
    digitalWrite(PIN_DIO_BIN1, HIGH);
  }
  else {
    digitalWrite(PIN_DIO_BIN2, HIGH);
    digitalWrite(PIN_DIO_BIN1, LOW);
  }
  analogWrite(PIN_DIO_PWMB, (int)(abs(val) * 255));
}
