#define encoderFRPinA  20
#define encoderFRPinB  21
#define encoderFLPinA  18
#define encoderFLPinB  19
#define encoderRCPinA  2
#define encoderRCPinB  3
#define motorSpeed 255
#define dirControl 13         
#define pwmControl 12
#define reardirControl 9
#define rearpwmControl 8
#define PULL 16
#define DIRL 15
#define ENAL 14
#define PULR 5
#define DIRR 6
#define ENAR 7
#define width 600
#define angle 300
#define opticalR A0
#define opticalL A1
#define opticalRear A2
#define button_ctr 35
#define button_rear 39
#define button_front 31
#define HEIGHT 33
#define WIDTH 24
#define CPTF 1024
#define CPTR 90

volatile unsigned int encoderFRPos = 0;
volatile unsigned int encoderFLPos = 0;
volatile unsigned int encoderRCPos = 0;
volatile unsigned int encoderFRAng = 0;
volatile unsigned int encoderFLAng = 0;
volatile unsigned int encoderRCAng = 0;
volatile int tim = 0;
volatile int deg = 0;
volatile int dir = 0;
volatile int per = 0;
volatile int rwm = 0;

boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars];

//-------------------------------------------------------------
//SETUP
//-------------------------------------------------------------
void setup() {
  pinMode (PULL, OUTPUT);      //left front pulse
  pinMode (PULR, OUTPUT);     //right front pulse
  pinMode (DIRL, OUTPUT);     //left front direction
  pinMode (DIRR, OUTPUT);     //right front direction
  pinMode (ENAL, OUTPUT);     //left front enable
  pinMode (ENAR, OUTPUT);     //right front enable
  pinMode (31, INPUT);         //button to perform the drive motion of the car
  pinMode(35, INPUT);          //button to turn the rear wheel CW to center it
  pinMode(39, INPUT);          //button to turn the rear wheel CCW to center it
  pinMode (dirControl, OUTPUT);     //rear wheel turning direction pin
  pinMode(pwmControl, OUTPUT);      //rear wheel turning speed control pin
  pinMode(reardirControl, OUTPUT);  //rear wheel forward or backward direction pin
  pinMode(rearpwmControl, OUTPUT);  //rear wheel forward or backward speed control pin
  pinMode(encoderFRPinA, INPUT);
  pinMode(button_front, INPUT);       //button to perform the drive motion of the car
  pinMode(button_ctr, INPUT);          //button to turn the rear wheel CW to center it
  pinMode(button_rear, INPUT);         //button to turn the rear wheel CCW to center it
  home();

  Serial.begin (9600);
  Serial.println("start");                // a personal quirk
  encoderFRPos = 0;
  encoderFLPos = 0;
  encoderRCPos = 0;
  encoderFRAng = 0;
  encoderFLAng = 0;
  encoderRCAng = 0;
}

void home() {
  digitalWrite(DIRR, HIGH);
  digitalWrite(DIRL, HIGH);
  digitalWrite(ENAR, HIGH);
  digitalWrite(ENAL, HIGH);
  while (analogRead(opticalR) < 750) {
    digitalWrite(PULR, HIGH);
    delayMicroseconds(width);
    digitalWrite(PULR, LOW);
    delayMicroseconds(width);
  }
  while (analogRead(opticalL) < 750) {
    digitalWrite(PULL, HIGH);
    delayMicroseconds(width);
    digitalWrite(PULL, LOW);
    delayMicroseconds(width);
  }
  attachInterrupt(digitalPinToInterrupt(encoderFLPinA), doEncoderFLA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderFLPinA), doEncoderFLB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderFRPinA), doEncoderFRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderFRPinA), doEncoderFRB, CHANGE);

  Serial.println("now perparing homing the rear wheel (OP: watch the cables)");
  Serial.println("*********************************************************************");
  Serial.println("To home the wheel by turning CLOCKWISE, press the MIDDLE button.");
  Serial.println("To home the wheel by turning COUNTERCLOCKWISE, press the REAR button.");
  Serial.println("*********************************************************************");
  while (digitalRead(button_rear) == 1 && digitalRead(button_ctr) == 1) {}
  digitalWrite(reardirControl, (digitalRead(button_ctr) == 0) ? 0 : 1);
  analogWrite(rearpwmControl, motorSpeed);
  while (analogRead(opticalRear) < 750) {}
  analogWrite(rearpwmControl, 0);
  attachInterrupt(digitalPinToInterrupt(encoderRCPinA), doEncoderRCA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRCPinA), doEncoderRCB, CHANGE);
  while (digitalRead(button_rear) == 0 || digitalRead(button_ctr) == 0) {}
  Serial.println("steering homing complete");
}

//-------------------------------------------------------------
//Main program
//-------------------------------------------------------------

void loop() 
{
  int cur = per;
  int rid = dir;
  recvWithStartEndMarkers();
  int rad = (tan(deg) / HEIGHT);
  int alpha = atan(HEIGHT/(rad + (WIDTH / 2)));
  int beta = atan(HEIGHT/(rad - (WIDTH / 2)));
  if (rvm == 1){
    while(encoderRCAng < 90){
    rearmotorCW();
    }
  }else if (encoderRCAng > 50) {
    while(encoderRCAng > 0){
      rearmotorCCW();
    }
  }else{
  if (abs(encoderFRAng - beta > 5)) {
      if (encoderFRAng > beta) {
        digitalWrite(DIR_R, LOW);
      }
      if (encoderFRAng < beta) {
        digitalWrite(DIRR, HIGH);
      }
      while (encoderFRAng < beta) {
        digitalWrite(PULR, HIGH);
        delayMicroseconds(width);
        digitalWrite(PULR, LOW);
        delayMicroseconds(width);
      }
    }
  
    if (abs(encoderFLAng - alpha > 5)) {
      if (encoderFLAng > alpha) {
        digitalWrite(DIRL, LOW);
      }
      if (leftEnc < alpha) {
        digitalWrite(DIRL, HIGH);
      }
      while (encoderFLAng < alpha) {
        digitalWrite(PULL, HIGH);
        delayMicroseconds(width);
        digitalWrite(PULL, LOW);
        delayMicroseconds(width);
      }
    } 
  }
  if(dir == 1){
  digitalWrite(dirControl,HIGH);
  }else{
    digitalWrite(dirControl,LOW);
  }
  analogWrite(pwmControl, per);
    delay(tim);
}
//-------------------------------------------------------------
//API extraction
//-------------------------------------------------------------
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
  if (Serial.available() > 0) {
    newData = false;
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
                parseData();
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
  }
}
//-------------------------------------------------------------
//Data correction
//-------------------------------------------------------------
void parseData() {

    // split the data into its parts
    
  char * strtokIndx; 

  strtokIndx = strtok(receivedChars, ","); //For how long
  tim = atoi(strtokIndx);
  
  strtokIndx = strtok(NULL,","); //deg of rotation
  deg = ((atoi(strtokIndx)*255)/100); //0 -> 100
  
  strtokIndx = strtok(NULL, ","); 
  dir = atoi(strtokIndx); //0-1
  
  strtokIndx = strtok(NULL, ",");
  //deg = (((atoi(strtokIndx)+50)*500)/100)+1250; //-50 -> 50
  per =  ((atoi(strtokIndx)*255)/100); //0 -> 100

  strtokIndx = strtok(NULL, ">"); 
  rwm = atoi(strtokIndx); //0-1 
  
}


//-------------------------------------------------------------
//Turn the rear wheel clockwise or counterclockwise
//-------------------------------------------------------------

  void rearmotorCW()      //rearwheel clockwise
  {
      digitalWrite(reardirControl, LOW);
  analogWrite(rearpwmControl, abs(deg));
  delay(10);
  }

  void rearmotorCCW()     //rearwheel counterclockwise
  {
  if( deg > 0){
      digitalWrite(reardirControl, HIGH);
    }
  analogWrite(rearpwmControl, abs(deg));
  delay(10);
  }
  
//-------------------------------------------------------------
//Handles Count for Encoders
//-------------------------------------------------------------

void doEncoderFLA() {
  if (digitalRead(encoderFRPinA) == HIGH) {
    if (digitalRead(encoderFRPinB) == LOW) {
      encoderFRPos++;
    }
    else {
      encoderFRPos--;
    }
  }
  else
  {
    if (digitalRead(encoderFRPinB) == HIGH) {
      encoderFRPos++;
    }
    else {
      encoderFRPos--;
    }
  }
  encoderFRAng = (encoderFRPos % CPTF) * (360) / CPTF;
}

void doEncoderFLB() {
  if (digitalRead(encoderFRPinB) == HIGH) {
    if (digitalRead(encoderFRPinA) == LOW) {
      encoderFRPos++;
    }
    else {
      encoderFRPos--;
    }
  }
  else
  {
    if (digitalRead(encoderFRPinA) == HIGH) {
      encoderFRPos++;
    }
    else {
      encoderFRPos--;
    }
  }
  encoderFRAng = (encoderFRPos % CPTF) * (360) / CPTF;
}
void doEncoderFLA() {
  if (digitalRead(encoderFLPinA) == HIGH) {
    if (digitalRead(encoderFLPinB) == LOW) {
      encoderFLPos++;
    }
    else {
      encoderFLPos--;
    }
  }
  else
  {
    if (digitalRead(encoderFLPinB) == HIGH) {
      encoderFLPos++;
    }
    else {
      encoderFLPos--;
    }
  }
  encoderFLAng = (encoderFLPos % CPTF) * (360) / CPTF;
}

void doEncoderFLB() {
  if (digitalRead(encoderFLPinB) == HIGH) {
    if (digitalRead(encoderFLPinA) == LOW) {
      encoderFLPos++;
    }
    else {
      encoderFLPos--;
    }
  }
  else
  {
    if (digitalRead(encoderFLPinA) == HIGH) {
      encoderFLPos++;
    }
    else {
      encoderFLPos--;
    }
  }
  encoderFLAng = (encoderFLPos % CPTF) * (360) / CPTF;
}

void doEncoderRCA() {
  if (digitalRead(encoderRCPinA) == HIGH) {
    if (digitalRead(encoderRCPinB) == LOW) {
      encoderRCPos++;
    }
    else {
      encoderRCPos--;
    }
  }
  else
  {
    if (digitalRead(encoderRCPinB) == HIGH) {
      encoderRCPos++;
    }
    else {
      encoderRCPos--;
    }
  }
  encoderRCAng = (encoderRCPos % CPTR) * (360) / CPTR;
}

void doEncoderRCB() {
  if (digitalRead(encoderRCPinB) == HIGH) {
    if (digitalRead(encoderRCPinA) == LOW) {
      encoderRCPos++;
    }
    else {
      encoderRCPos--;
    }
  }
  else
  {
    if (digitalRead(encoderRCPinA) == HIGH) {
      encoderRCPos++;
    }
    else {
      encoderRCPos--;
    }
  }
  encoderRCAng = (encoderRCPos % CPTR) * (360) / CPTR;
}


