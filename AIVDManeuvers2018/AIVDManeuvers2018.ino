#define encoderFRPinA  18
#define encoderFRPinB  19
#define encoderFLPinA  20
#define encoderFLPinB  21
#define encoderRCPinA  2
#define encoderRCPinB  3
#define motorSpeed 255
#define dirControl 13         
#define pwmControl 12
#define reardirControl 9
#define rearpwmControl 8
#define PULL 6
#define DIRL 5
#define ENAL 4
#define PULR 7
#define DIRR 10
#define ENAR 11
#define width 600
#define opticalR 0
#define opticalL 1
#define opticalRear 2
#define button_ctr 35
#define button_rear 39
#define button_front 31
#define HEIGHT 33
#define carwidth 24
#define CPTF 1024
#define CPTR 90

unsigned int encoderFRPos = 0;
unsigned int encoderFLPos = 0;
unsigned int encoderRCPos = 0;
unsigned int encoderFRAng = 0;
unsigned int encoderFLAng = 0;
unsigned int encoderRCAng = 0;
int tim = 0;
int deg = 0;
int dir = 0;
int per = 0;
int rwm = 0;

boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars];

//-------------------------------------------------------------
//SETUP
//-------------------------------------------------------------
void setup() {
  Serial.begin (9600);
  pinMode (PULL, OUTPUT);      //left front pulse
  pinMode (PULR, OUTPUT);     //right front pulse
  pinMode (DIRL, OUTPUT);     //left front direction
  pinMode (DIRR, OUTPUT);     //right front direction
  pinMode (ENAL, OUTPUT);     //left front enable
  pinMode (ENAR, OUTPUT);     //right front enable
  pinMode (dirControl, OUTPUT);     //rear wheel turning direction pin
  pinMode(pwmControl, OUTPUT);      //rear wheel turning speed control pin
  pinMode(reardirControl, OUTPUT);  //rear wheel forward or backward direction pin
  pinMode(rearpwmControl, OUTPUT);  //rear wheel forward or backward speed control pin
  pinMode(encoderFRPinA, INPUT);
  pinMode(encoderFRPinB, INPUT);
  pinMode(encoderFLPinA, INPUT);
  pinMode(encoderFLPinB, INPUT);
  pinMode(encoderRCPinA, INPUT);
  pinMode(encoderRCPinB, INPUT);
  pinMode(button_front, INPUT);       //button to perform the drive motion of the car
  pinMode(button_ctr, INPUT);          //button to turn the rear wheel CW to center it
  pinMode(button_rear, INPUT);         //button to turn the rear wheel CCW to center it
  Serial.println("start");                // a personal quirk
  homer();
  
  encoderFRPos = 0;
  encoderFLPos = 0;
  encoderRCPos = 0;
  encoderFRAng = 0;
  encoderFLAng = 0;
  encoderRCAng = 0;
}

void homer() {
  digitalWrite(DIRR, HIGH);
  
  digitalWrite(ENAR, HIGH);
  
  while (analogRead(opticalR) < 750) {
    digitalWrite(PULR, HIGH);
    delayMicroseconds(width);
    digitalWrite(PULR, LOW);
    delayMicroseconds(width);
  }

  digitalWrite(DIRL, HIGH);
  digitalWrite(ENAL, HIGH);
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
  //int rad = (tan(deg) / HEIGHT);
  //int alpha = atan(HEIGHT/(rad + (carwidth / 2)));
  //int beta = atan(HEIGHT/(rad - (carwidth / 2)));
  int alpha = deg;
  int beta = deg;
  if (rwm == 1){
    while(encoderRCAng < 90){
    rearmotorCW();
    }
  }else if (encoderRCAng > 50) {
    while(encoderRCAng > 0){
      rearmotorCCW();
    }
  }else{
      if (beta % encoderFRAng > 360 % encoderFRAng) {
        digitalWrite(DIRR, LOW);
      }
      if (beta % encoderFRAng < 360 % encoderFRAng) {
        digitalWrite(DIRR, HIGH);
      }
      if (alpha % encoderFLAng > 360 % encoderFLAng) {
        digitalWrite(DIRL, LOW);
      }
      if (alpha % encoderFLAng < 360 % encoderFLAng) {
        digitalWrite(DIRL, HIGH);
      }

      if(deg > 0){
       while (encoderFRAng != beta || encoderFLAng != alpha) {
        if(encoderFRAng != beta){
        digitalWrite(PULR, HIGH);
        delayMicroseconds(width);
        digitalWrite(PULR, LOW);
        delayMicroseconds(width);
        }
        Serial.println(encoderFRAng);
        if(encoderFLAng != alpha){
        digitalWrite(PULL, HIGH);
        delayMicroseconds(width);
        digitalWrite(PULL, LOW);
        delayMicroseconds(width);
        }
        Serial.println(encoderFLAng);
      }
      }
      if(deg < 0){
       while (encoderFRAng != (beta + 360) || encoderFLAng != (alpha + 360)) {
        if(encoderFRAng != beta){
        digitalWrite(PULR, HIGH);
        delayMicroseconds(width);
        digitalWrite(PULR, LOW);
        delayMicroseconds(width);
        }
        Serial.println(encoderFRAng);
        if(encoderFLAng != alpha){
        digitalWrite(PULL, HIGH);
        delayMicroseconds(width);
        digitalWrite(PULL, LOW);
        delayMicroseconds(width);
        }
        Serial.println(encoderFLAng);
      }
        
      }
      


  }
  
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
  Serial.println(tim);
  
  strtokIndx = strtok(NULL,","); //deg of rotation
  deg = atoi(strtokIndx); //0 -> 100
  Serial.println(deg);
  strtokIndx = strtok(NULL, ","); 
  dir = atoi(strtokIndx); //0-1
  Serial.println(dir);
  strtokIndx = strtok(NULL, ",");
  //deg = (((atoi(strtokIndx)+50)*500)/100)+1250; //-50 -> 50
  per =  ((atoi(strtokIndx)*255)/100); //0 -> 100
  Serial.println(per);
  strtokIndx = strtok(NULL, ">"); 
  rwm = atoi(strtokIndx); //0-1 
  Serial.println(rwm);
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

void doEncoderFRA() {
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
  encoderFRAng = map(abs(encoderFRPos % CPTF),0,1023,0,360);
}

void doEncoderFRB() {
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
  encoderFRAng = map(abs(encoderFRPos % CPTF),0,1023,0,360);
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
  encoderFLAng = map(abs(encoderFRPos % CPTF),0,1023,0,360);
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
  encoderFLAng = map(abs(encoderFRPos % CPTF),0,1023,0,360);
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
  encoderRCAng = (encoderRCPos % CPTR) * 360 / CPTR;
}


