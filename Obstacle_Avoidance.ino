/********************************************************************
  Original by T.Basic, 3/29/2021
  - Program seperated into three parts
  (1) Lynxbot moving down corridor
  - proportional control used for moving down corridor
  - After blocker is removed moves at a deltaF, for delay(1200)
  (2) First sweep for light with servo motor, followed by car turning
  towards light
  (3) Lynxbot moving towards light
  - Proportional control used between photoresistor and motors
  - Servomotor constantly sweeps 75 degrees to 105 degrees
  - Depending on what angle light is detected, motors turn a certain way
  - When photoresistor detects light 20 cm away, 180 degrees turn
  to neutralize light
  
 ***********************************************************************/
// Pin Assignments
int RED = 4;
int YLW = 5;
int GRN = 6;
int BUTTON_B = 8;
int MOTOR_L = 10;  // left motor signal
int MOTOR_R = 11;  // right motor signal
int BUMPER = 13;
int SHARP1 = A2;   // sharp side
int SHARP2 = A3; //Sharp forward
int result1;
int result2;
int sensor1;
int sensor2;
int servoPin = A1;     //servo pin
int servoAngle;
int lightAngle;
int LIGHT = A0;     //photoresistor pin (can't be a3)
// recommended initial values 
int delta  = 15;   // 15 as low speed, 25 as high speed
int HYS    = 150; // sharp hysterisis (default 50)     

// adjust stop speed and target distance as appropriate
int WALL = 786;        // 700 mv gives 40cm
int STOP_SPEED = 144;  // tuned for Big Louie

int DOOR = 1400;
float doorLIGHT = 4200;
// proportional control int
int deltaF = 25; 
int deltaL = 15;
const float KP = 0.35; // for sharp
const float KS = 2.0; // for light
int error;
int deltaV;
int dummy;
float result;          //A to D value from photoresistor
float mvresult;        //millivolt value for photoresistor
float maxmv; 
unsigned long time;
unsigned long start;
int lightTimeFound;
void setup() {
  // put your setup code here, to run once:
 // initialize the digital led pins as outputs.
  pinMode(RED, OUTPUT);
  pinMode(YLW, OUTPUT);
  pinMode(GRN, OUTPUT);
  //initialize buttons and bumper pins as inputs
  pinMode(BUMPER, INPUT);
  pinMode(BUTTON_B, INPUT);
  //initialize motor control pins as outputs
  pinMode(MOTOR_L, OUTPUT);
  pinMode(MOTOR_R, OUTPUT);
  
  //setup serial debug
  Serial.begin(9600);
 
  Serial.println("Press Button B to start.");
  do {
    toggleLED(GRN);         //motors stopped, Green LED flashing
  } while(digitalRead(BUTTON_B)== HIGH);
  
  sensor1 = map(analogRead(SHARP1),0,1023,0,5000); // initialize sensor
  sensor2 = map(analogRead(SHARP2),0,1023,0,5000); 
  result = analogRead(LIGHT);               //read the value of photoresistor
  mvresult = map(result, 0, 1024, 0, 5000);  //convert value to millivolts
  runMotors(0,0);  // check motors are stopped
  delay(500);
  runMotors(deltaF,deltaF);
  delay(5);
  runMotors(0,0);
  delay(5);
  servoPulse(servoPin,105); 
  delay(500);
  Serial.println("Program Running. Press bumper to stop");
}


void loop() {
  sensor1 = map(analogRead(SHARP1),0,1023,0,5000); 
  sensor2 = map(analogRead(SHARP2),0,1023,0,5000);
  result = analogRead(LIGHT);               //read the value of photoresistor
  mvresult = map(result, 0, 1024, 0, 5000);  //convert value to millivolts
  // Running down corridor section
  do{
  sensor1 = map(analogRead(SHARP1),0,1023,0,5000);
  sensor2 = map(analogRead(SHARP2),0,1023,0,5000);
  int error = WALL - sensor1;
  int dummy = min(abs(error)*KP, 100);  
  int deltaV = deltaF*(100-dummy)/100;

  if(error>0){          // too far from the wall
    //Serial.println(sensor1);
    //delay(100);
    turnOnLED(YLW);
    runMotors(deltaV, deltaF); 
  }else if(error < 0){    // too close to the wall
    //Serial.println(sensor1);
    //delay(100);
    turnOnLED(RED);
    runMotors(deltaF,deltaV); 
  }else{                             // at target distance
    //Serial.println(sensor1);
    //delay(100);
    turnOnLED(GRN);
    runMotors(deltaF,deltaF);
  }
 }while(sensor2 < DOOR);
 sensor1 = map(analogRead(SHARP1),0,1023,0,5000); 
 sensor2 = map(analogRead(SHARP2),0,1023,0,5000);
 do {  
    turnOnLED(YLW);
    runMotors(0,0);
    delay(100);
    sensor1 = map(analogRead(SHARP1),0,1023,0,5000); // twice is good
    sensor2 = map(analogRead(SHARP2),0,1023,0,5000);
    }while(sensor2 > DOOR);

 runMotors(0,0);
 delay(1000);
 runMotors(deltaF, deltaF);
 delay(1200);
 runMotors(0,0);
 delay(1000);

 // Initial sweep with servomotor, followed by car turning
 // Servo Sweep 90 degrees and find light
   for(servoAngle = 195; servoAngle >= 15; servoAngle = servoAngle - 1){
      turnOnLED(GRN); 
      result = analogRead(LIGHT);               //read the value of photoresistor
      mvresult = map(result, 0, 1024, 0, 5000);
      servoPulse(servoPin, servoAngle);
      delay(15);           // change to 40 ms 
      if (mvresult > maxmv){ 
        lightAngle = servoAngle - 15;
        maxmv = mvresult;
        turnOnLED(RED);
      }  
} 
// Sweep servo from 0 deg to light angle
for(servoAngle = 15; servoAngle <= lightAngle + 15; servoAngle = servoAngle + 1){
      turnOnLED(GRN); 
      result = analogRead(LIGHT);               //read the value of photoresistor
      mvresult = map(result, 0, 1024, 0, 5000);
      servoPulse(servoPin, servoAngle);
      delay(15);           // change to 40 ms 
      }  
    runMotors(0,0);
    delay(500);
    // turn lynxbot toward light
    for(float oo = 0; oo <= 12 - (lightAngle/7.5); oo++){
    if(lightAngle < 90){  
      turnOnLED(YLW);  
      result = analogRead(LIGHT);               //read the value of photoresistor
      mvresult = map(result, 0, 1024, 0, 5000); 
      delay(40);
      runMotors(25,-25);
      }
  }
runMotors(0,0);
delay(500);
  for(float mm = 0; mm <= (lightAngle/7.5) - 12 ; mm++){
      if(lightAngle >=90){
      turnOnLED(RED);  
      result = analogRead(LIGHT);               //read the value of photoresistor
      mvresult = map(result, 0, 1024, 0, 5000);
      delay(40);        
      runMotors(-25,25); 
  }
 
}
runMotors(0,0);
delay(500); 
// Final section, lynxbot moving towards light
do{
  for(servoAngle = 125; servoAngle >= 70; servoAngle = servoAngle - 1){
      turnOnLED(GRN); 
      result = analogRead(LIGHT);               //read the value of photoresistor
      mvresult = map(result, 0, 1024, 0, 5000);
      servoPulse(servoPin, servoAngle);
      

      if (mvresult > maxmv){ 
        lightAngle = servoAngle - 15;
        maxmv = mvresult;
        turnOnLED(RED);   
        }
        int error = lightAngle - (servoAngle - 15);
        int dummy = min(abs(error)*KS, 100);  
        int deltaV = deltaL*(100-dummy)/100;
        if (error < 0){
            turnOnLED(RED);
            runMotors(deltaL,deltaV);
        }else if(error > 0){
          turnOnLED(YLW);
              runMotors(deltaV,deltaL);
            }
            else{
              turnOnLED(GRN);
              runMotors(deltaL,deltaL);
            }
            mvresult = map(analogRead(LIGHT), 0, 1024, 0, 5000);
         if (mvresult > doorLIGHT){
          runMotors(0,0);
          break;          
         }       
       mvresult = map(analogRead(LIGHT), 0, 1024, 0, 5000);
        }
         mvresult = map(analogRead(LIGHT), 0, 1024, 0, 5000);  

    for(servoAngle = 70; servoAngle <= 125; servoAngle = servoAngle + 1){
      turnOnLED(GRN); 
      result = analogRead(LIGHT);               //read the value of photoresistor
      mvresult = map(result, 0, 1024, 0, 5000);

      servoPulse(servoPin, servoAngle);
      
                 // change to 40 ms 
      if (mvresult > maxmv){ 
        lightAngle = servoAngle - 15;   
        maxmv = mvresult;
        turnOnLED(RED); 
      } 
      int error = lightAngle - (servoAngle - 15);
      int dummy = min(abs(error)*KS, 100);  
      int deltaV = deltaL*(100-dummy)/100;
      if (error <0 ){
        turnOnLED(RED);
        runMotors(deltaL,deltaV);
        }else if(error >0){
          turnOnLED(YLW);
          runMotors(deltaV,deltaL);
            }
            else{
               turnOnLED(YLW);
              runMotors(deltaL,deltaL);
            }
          mvresult = map(analogRead(LIGHT), 0, 1024, 0, 5000);
          if (mvresult > doorLIGHT){
          runMotors(0,0);
          break;          
         } 
         mvresult = map(analogRead(LIGHT), 0, 1024, 0, 5000);
}  
 }while(mvresult < doorLIGHT);
// Stop, turn 180 degrees, reverse
 do{
  turnOnLED(RED);
  runMotors(0,0);
  delay(1000);
  runMotors(45,-45);
  delay(650);
  runMotors(0,0);
  delay(500);
//  runMotors(-deltaF,-deltaF);
//  delay(250);
  runMotors(0,0);
  delay(1000);
  break;
 }while(mvresult > doorLIGHT);
 do{
  flashAllLEDs();
  runMotors(0,0);
 }while(1);
 }



//**********FUNCTIONS (subroutines)******************
//Turn on a single LED, and all other off
void turnOnLED(int colour){
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW); 
  digitalWrite(colour, HIGH);
}

//Toggle an LED on/off
void toggleLED(int colour){
  digitalWrite(colour, HIGH);
  delay(250);
  digitalWrite(colour, LOW);
  delay(250); 
}

//flash all LEDs
void flashAllLEDs(){
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
  delay(250);
  digitalWrite(GRN, HIGH);
  digitalWrite(YLW, HIGH);
  digitalWrite(RED, HIGH);
  delay(250);
}

// left and right motor commands
void runMotors(int delta_L, int delta_R){
  int pulse_L = (STOP_SPEED + delta_L)*10;  //determines length of pulse in microsec
  int pulse_R = (STOP_SPEED + delta_R)*10;
  for(int i=0; i<3; i++){
    pulseOut(MOTOR_L, pulse_L);    //send pulse to left motors
    pulseOut(MOTOR_R, pulse_R);    //send pulse to right motors
  }
}

// single motor pulsewidth command
void pulseOut(int motor, int pulsewidth){
  digitalWrite(motor, HIGH);         
  delayMicroseconds(pulsewidth);  //send pulse of desired pulsewidth      
  digitalWrite(motor, LOW);
}

void servoPulse(int servoPin, int myAngle){
  int pulseWidth = (myAngle * 9) + 350; 
  digitalWrite(servoPin, HIGH);         //set servo high
  delayMicroseconds(pulseWidth);        //microsecond pause
  digitalWrite(servoPin, LOW);          //set servo low
}
