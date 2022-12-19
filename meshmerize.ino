


#include <QTRSensors.h>
const int buttonPin = 2;  
#define NUM_SENSORS 7     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN    // emitter is controlled by digital pin 2
int buttonState = digitalRead(buttonPin);  
#define ir_sensor A6
#define Kp .1
#define Kd .09
#define Ki .01
#define MaxSpeed 200

#define BaseSpeed 200
// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {8,A5,A4,A3,A2,A1,A0},NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
#define leapTime 100
#define pwm1 11//pwma
#define rightMotor1 3//inb1
#define rightMotor2 4   //inb2
#define pwm2 10  //pwmb
#define leftMotor1  5//ina1
#define leftMotor2  6//
#define led 13
int ir_reading=analogRead(ir_sensor);
   unsigned int position = qtrrc.readLine(sensorValues);
int leftNudge;
int replaystage;
int rightNudge;
char path[100] = {};
int pathLength;
int readLength;
void setup(){
  pinMode(buttonPin, INPUT);
 pinMode(ir_sensor,INPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(pwm1,OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  digitalWrite(led, LOW);
    delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 20; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();
      analogWrite(pwm1,100);
  digitalWrite(rightMotor1,HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(pwm2,100);
  digitalWrite(leftMotor1,LOW);
  digitalWrite(leftMotor2, HIGH);// reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
    for (int i = 0; i < 30; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();
      analogWrite(pwm1,100);
  digitalWrite(rightMotor1,LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(pwm2,100);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);// reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
    for (int i = 0; i < 18; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();
      analogWrite(pwm1,100);
  digitalWrite(rightMotor1,HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(pwm2,100);
  digitalWrite(leftMotor1,LOW);
  digitalWrite(leftMotor2, HIGH);// reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
 digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);

 delay(3000);
   pinMode(buttonPin, INPUT);
}
void loop(){
 
readSensors();
if( sensorValues[3]<600 && sensorValues[0]>600 && sensorValues[6]>600 && analogRead(ir_sensor)>600){
  straight();
  readSensors();
}
else
{
  readSensors();
  turns();
}

}
 void turns(){
 if( sensorValues[0]<600 && sensorValues[6]<600 && sensorValues[4]<600){//to check maze is finishef or not
     analogWrite(pwm1,120);
  digitalWrite(rightMotor1,HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);
    delay(leapTime);
    readSensors();
    
    if(sensorValues[0]<600 && sensorValues[6]<600 && sensorValues[3]<600){
      done();
    }
    if(sensorValues[0]>600 && sensorValues[6]>600){ 
      turnLeft();
    }
    
  }
  
 if( sensorValues[0]>600 && sensorValues[6]>600 && sensorValues[1]<600 && sensorValues[2]<600 && sensorValues[3]<600 &&sensorValues[4]<600 && sensorValues[5]<600  ){//to check maze is finishef or not
     analogWrite(pwm1,120);
  digitalWrite(rightMotor1,HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);
    delay(30);
    readSensors();
    if (sensorValues[3]>600)
    { delay(10);
      turnTleft();
    }
    
  }
  
  if(sensorValues[0]<600 && sensorValues[3]<600){ // if you can turn left then turn left
         analogWrite(pwm1,120);
  digitalWrite(rightMotor1,HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);
    delay(leapTime);
    readSensors();
      
      if(sensorValues[0]>600 ){
        turnLeft();
      }
     /* else{
        done();
      }
*/  }
     if(sensorValues[0]<600 &&sensorValues[3]>600 && sensorValues[2]>600 && sensorValues[5]>600 ){//135 left
      {
        analogWrite(pwm1,120);
        digitalWrite(rightMotor1,HIGH);
        digitalWrite(rightMotor2, LOW);
        analogWrite(pwm2,120);
        digitalWrite(leftMotor1,HIGH);
        digitalWrite(leftMotor2, LOW);  
        delay(40);
        turnSmallLeft();
      }
      
    }
  if(sensorValues[6]<600){//right
     analogWrite(pwm1,120);
  digitalWrite(rightMotor1,HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);
    delay(30);
    readSensors();
    
    if(sensorValues[0]<600 && sensorValues[3]>600){//left
     // delay(40);
      readSensors();
      if(sensorValues[6]<600 &&sensorValues[3]<600 && sensorValues[0]<600){
        done();
      }
     
      else{
        turnLeft();
        return;
      }
    }
    if(sensorValues[0]<600 ){//left
      delay(leapTime-40);
      readSensors();
      if(sensorValues[6]<600 &&sensorValues[3]<600 && sensorValues[0]<600){
        done();
      }
     
      else{
        turnLeft();
        return;
      }
    }
    if (sensorValues[6]<600 && sensorValues[3]>600 && sensorValues[2]>600 && sensorValues[1]>600)
    {
      turnRight();
      return;
    }
    delay(leapTime-30);
    readSensors();
    if(sensorValues[0]>600 && sensorValues[2]>600 &&
      sensorValues[3]>600 && sensorValues[5]>600){
      turnRight();
      return;
    }
    path[pathLength]='S';
    Serial.println("s");
    pathLength++;
    Serial.print("Path length: ");
    Serial.println(pathLength);
    if(path[pathLength-2]=='B'){
    Serial.println("shortening path");
    shortPath();
     }
    straight();
  }
  readSensors();
  if(sensorValues[0]>600 && sensorValues[2]>600 && sensorValues[3]>600 
    && sensorValues[6]>600 && sensorValues[1]>600 && sensorValues[4]>600)
    {
    turnAround();
  }
  
  if(analogRead(ir_sensor)<600)
    {
    turnOAround();
  }

}

void readSensors(){
  
   unsigned int  line_position = qtrrc.readLine(sensorValues); 

}



void done(){
  digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
  replaystage=1;
  path[pathLength]='D';
  pathLength++;
 while(sensorValues[0]<600){
   digitalWrite(led, HIGH);
   
    readSensors();
  
 }
 
 buttonState=digitalRead(buttonPin);
 while(true){
   if(digitalRead(buttonPin)==HIGH){
  delay(600);
  replay();
   }
else{
  continue;
}
   
 }
}

void turnTleft()
{ 
while(sensorValues[2]<600 ||sensorValues[4]<600){
  analogWrite(pwm1,120);
  digitalWrite(rightMotor1,LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);

    delay(2);
     digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    readSensors();
  }
    
  while(sensorValues[4]>600 ) {
   analogWrite(pwm1,120);
  digitalWrite(rightMotor1,LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);

    delay(2);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    readSensors();
  
  }
  
  
  if(replaystage==0){
    path[pathLength]='t';
    Serial.println("t");
    pathLength++;
    Serial.print("Path length: ");
    Serial.println(pathLength);
      
  }

    
  
}
void turnSmallLeft()
{ 
while(sensorValues[2]<600 ||sensorValues[4]<600){
  analogWrite(pwm1,120);
  digitalWrite(rightMotor1,LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);

    delay(2);
     digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(3);
    readSensors();
  }
    
  while(sensorValues[4]>600 ) {
   analogWrite(pwm1,120);
  digitalWrite(rightMotor1,LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);

    delay(2);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(3);
    readSensors();
  
  }
  
  
  if(replaystage==0){
    path[pathLength]='m';
    Serial.println("m");
    pathLength++;
    Serial.print("Path length: ");
    Serial.println(pathLength);
      
  }

    
  
}
void turnLeft()
{ 
while(sensorValues[2]<600 ||sensorValues[4]<600){
  analogWrite(pwm1,120);
  digitalWrite(rightMotor1,LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);

    delay(2);
     digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    readSensors();
  }
    
  while(sensorValues[4]>600 ) {
   analogWrite(pwm1,120);
  digitalWrite(rightMotor1,LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);

    delay(2);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    readSensors();
  
  }
  
 

    
  if(replaystage==0){
    path[pathLength]='L';
    Serial.println("l");
    pathLength++;
    Serial.print("Path length: ");
    Serial.println(pathLength);
      if(path[pathLength-2]=='B'){
    Serial.println("shortening path");
        shortPath();
      }
  }
}

void turnRight(){

    
while(sensorValues[4]<600){
  analogWrite(pwm1,120);
  digitalWrite(rightMotor1,HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,LOW);
  digitalWrite(leftMotor2, HIGH);
    delay(2);
     digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    readSensors();
  }
   while(sensorValues[4]>600){
       analogWrite(pwm1,120);
  digitalWrite(rightMotor1,HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,LOW);
  digitalWrite(leftMotor2, HIGH);
    delay(2);
     digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    readSensors();
  }
   while(sensorValues[2]>600){
  analogWrite(pwm1,120);
  digitalWrite(rightMotor1,HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,LOW);
  digitalWrite(leftMotor2, HIGH);
    delay(2);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    readSensors();
  }
  if(replaystage==0){
  path[pathLength]='R';
  Serial.println("r");
  pathLength++;
  Serial.print("Path length: ");
  Serial.println(pathLength);
    if(path[pathLength-2]=='B'){
      Serial.println("shortening path");
      shortPath();
    }
  }
 
}

void turnSmallRight(){

    
while(sensorValues[4]<600){
  analogWrite(pwm1,120);
  digitalWrite(rightMotor1,HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,LOW);
  digitalWrite(leftMotor2, HIGH);
    delay(2);
     digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    readSensors();
  }
   while(sensorValues[4]>600){
       analogWrite(pwm1,120);
  digitalWrite(rightMotor1,HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,LOW);
  digitalWrite(leftMotor2, HIGH);
    delay(2);
     digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    readSensors();
  }
   while(sensorValues[2]>600){
  analogWrite(pwm1,120);
  digitalWrite(rightMotor1,HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(pwm2,120);
  digitalWrite(leftMotor1,LOW);
  digitalWrite(leftMotor2, HIGH);
    delay(2);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    readSensors();
  }
  if(replaystage==0){
  path[pathLength]='q';
  Serial.println("q");
  pathLength++;
  Serial.print("Path length: ");
  Serial.println(pathLength);
    
  }
 
}
void straight(){
    digitalWrite(rightMotor1,HIGH);
    digitalWrite(rightMotor2, LOW);
    digitalWrite(leftMotor1,HIGH);
    digitalWrite(leftMotor2, LOW);                                                                                                                                                                                                                                                                                          
    int lastError = 0;
    int line_position = qtrrc.readLine(sensorValues);
    int error = line_position - 3000;
    int error1 = error - lastError;
    int error2 = (2.0 / 3.0) * error2 + error ;
    int motorSpeed = Kp * error + Kd * error1 + Ki * error2;
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
    if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
    if (rightMotorSpeed < 0)rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)leftMotorSpeed = 0;
    analogWrite(pwm1, rightMotorSpeed);
    analogWrite(pwm2, leftMotorSpeed);

    lastError = error;
  
    qtrrc.readLine(sensorValues);
  
  
 
}
void turnAround(){
  analogWrite(pwm1,140);
  digitalWrite(rightMotor1,HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(pwm2,140);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);
    delay(80);
   while(sensorValues[4]>600){
      analogWrite(pwm1,160);
  digitalWrite(rightMotor1,LOW);
  digitalWrite(rightMotor2,HIGH);
  analogWrite(pwm2,140);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);
    delay(2);
     digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    readSensors();
  }
   /*while(ir_reading <200){
      analogWrite(pwm1,160);
  digitalWrite(rightMotor1,LOW);
  digitalWrite(rightMotor2,HIGH);
  analogWrite(pwm2,160);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);
    delay(2);
     digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    readSensors();
  }*/
  path[pathLength]='B';
  pathLength++;
  straight();
  Serial.println("b");
  Serial.print("Path length: ");
  Serial.println(pathLength);
}
void turnOAround(){
    
   while(sensorValues[3]>600|| analogRead(ir_sensor)<600){
      analogWrite(pwm1,160);
      digitalWrite(rightMotor1,LOW);
      digitalWrite(rightMotor2,HIGH);
      analogWrite(pwm2,140);
      digitalWrite(leftMotor1,HIGH);
      digitalWrite(leftMotor2, LOW);
    delay(2);
     digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    readSensors();
   }
   /*while(ir_reading <200){
      analogWrite(pwm1,160);
  digitalWrite(rightMotor1,LOW);
  digitalWrite(rightMotor2,HIGH);
  analogWrite(pwm2,160);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2, LOW);
    delay(2);
     digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    delay(1);
    readSensors();
  }*/
  path[pathLength]='B';
  pathLength++;
  straight();
  Serial.println("b");
  Serial.print("Path length: ");
  Serial.println(pathLength);
}

void shortPath(){
 int shortDone=0;
  if(path[pathLength-3]=='L' && path[pathLength-1]=='R'  ){
    pathLength-=3;
    path[pathLength]='B';
  Serial.println("test1");
    shortDone=1;
  }
   
  if(path[pathLength-3]=='L' && path[pathLength-1]=='S'  && shortDone==0){
    pathLength-=3;
    path[pathLength]='R';
  Serial.println("test2");
    shortDone=1;
  }
   
  if(path[pathLength-3]=='R' && path[pathLength-1]=='L'  && shortDone==0){
    pathLength-=3;
    path[pathLength]='B';
  Serial.println("test3");
    shortDone=1;
  }
  
   
  if(path[pathLength-3]=='S' && path[pathLength-1]=='L'&& shortDone==0){
    pathLength-=3;
    path[pathLength]='R';
  Serial.println("test4");
    shortDone=1;
  }
     
  if(path[pathLength-3]=='S' && path[pathLength-1]=='S'   &&shortDone==0){
    pathLength-=3;
    path[pathLength]='B';
  Serial.println("test5");
    shortDone=1;
  }
    if(path[pathLength-3]=='L' && path[pathLength-1]=='L' && shortDone==0){
    pathLength-=3;
    path[pathLength]='S';
  Serial.println("test6");
    shortDone=1;
  }
  
  path[pathLength+1]='D';
  path[pathLength+2]='D';
  pathLength++;
  Serial.print("Path length: ");
  Serial.println(pathLength);
  printPath();
}




void printPath(){
  Serial.println("+++++++++++++++++");
  int x;
  while(x<=pathLength){
  Serial.println(path[x]);
  x++;
  }
  Serial.println("+++++++++++++++++");
}


void replay(){
   readSensors();
 if(sensorValues[0]>600 &&sensorValues[6]>600){
    straight();
  }
      
 
  else{
    if(path[readLength]=='D'){
      analogWrite(pwm1,200);
       digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
     analogWrite(pwm2,200);
    digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
    delay(80);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
      endMotion();
    }

 
    if(path[readLength]=='L'){
       analogWrite(pwm1,120);
     digitalWrite(leftMotor1,HIGH);
     digitalWrite(leftMotor2, LOW);
      analogWrite(pwm2,120);
    analogWrite(rightMotor1, HIGH);
    analogWrite(rightMotor2,LOW);
    delay(leapTime);
      turnLeft();
    }
    if(path[readLength]=='R'){
             analogWrite(pwm1,120);
     digitalWrite(leftMotor1,HIGH);
     digitalWrite(leftMotor2, LOW);
      analogWrite(pwm2,120);
    analogWrite(rightMotor1, HIGH);
    analogWrite(rightMotor2,LOW);
    delay(30);
    if (sensorValues[6]<600 && sensorValues[3]>600 && sensorValues[2]>600 && sensorValues[1]>600)
    {
      turnRight();
      
    }
    else{
      delay(leapTime-30);
    turnRight();
    }
    }
    if(path[readLength]=='S'){
           analogWrite(pwm1,200);
     digitalWrite(leftMotor1,HIGH);
     digitalWrite(leftMotor2, LOW);
      analogWrite(pwm2,200);
    analogWrite(rightMotor1, HIGH);
    analogWrite(rightMotor2,LOW);
    delay(leapTime);
    straight();
    }
 
    if(path[readLength]=='m'){
           analogWrite(pwm1,120);
     digitalWrite(leftMotor1,HIGH);
     digitalWrite(leftMotor2, LOW);
      analogWrite(pwm2,120);
    analogWrite(rightMotor1, HIGH);
    analogWrite(rightMotor2,LOW);
    delay(50);
    turnSmallLeft();
    }
    
    if(path[readLength]=='q'){
           analogWrite(pwm1,120);
     digitalWrite(leftMotor1,HIGH);
     digitalWrite(leftMotor2, LOW);
      analogWrite(pwm2,120);
    analogWrite(rightMotor1, HIGH);
    analogWrite(rightMotor2,LOW);
    delay(50);
    turnSmallRight();
    }
    
        if(path[readLength]=='t'){
           analogWrite(pwm1,120);
     digitalWrite(leftMotor1,HIGH);
     digitalWrite(leftMotor2, LOW);
      analogWrite(pwm2,120);
    analogWrite(rightMotor1, HIGH);
    analogWrite(rightMotor2,LOW);
    delay(30);
    turnTleft();
    }

  
    
    readLength++;
  }
    
  replay();

}

void endMotion(){
    digitalWrite(led, LOW);
    delay(500);
    digitalWrite(led, HIGH);
    delay(200);
      digitalWrite(led, LOW);
    delay(200);
    digitalWrite(led, HIGH);
    delay(500);
  endMotion();
}
