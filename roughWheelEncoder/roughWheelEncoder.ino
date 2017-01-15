
//Using the encoder sample code provided by dfrobot.com

#include <PID_v1.h>

const byte encoder0pinA = 2;// interrupt pin 0 // Don't know the usage
const byte encoder0pinB = 3;// digital pin 3
int E_left =5; //The enabling of L298PDC motor driver board connection to the digital interface port 5
int M_left =4; //The enabling of L298PDC motor driver board connection to the digital interface port 4

byte encoder0PinALast; // Don't understand the usage yet
double duration,abs_duration;//the number of the pulses
boolean result;
boolean Direction; // the rotation direction

double val_output;//Power supplied to the motor PWM value.
double Setpoint;
double Kp=0.6, Ki=5, Kd=0;  
PID myPID(&abs_duration, &val_output, &Setpoint, Kp, Ki, Kd, DIRECT); 
 
void setup() {
  Serial.begin(9600);

  //Assumption it is using only one motor first...
  pinMode(M_left, OUTPUT);   //L298P Control port settings DC motor driver board for the output mode
  pinMode(E_left, OUTPUT); 

  //Assume that setting different output value will change the rpm
  Setpoint =80;  //Set the output value of the PID

  //Need the droid to test out... Don't know what is this...
  myPID.SetMode(AUTOMATIC);//PID is set to automatic mode
  myPID.SetSampleTime(100);//Set PID sampling frequency is 100ms
  EncoderInit();
}

void loop() {
  //advance();//Motor Forward
  abs_duration=abs(duration);
  result=myPID.Compute();//PID conversion is complete and returns 1
  if(result)
  {
    Serial.print("Pulse: ");
    Serial.println(duration); 
    duration = 0; //Count clear, wait for the next count
  }
}

void EncoderInit(){
  Direction = true;
  pinMode(encoder0pinB, INPUT);
  attachInterrupt(0, wheelSpeed, CHANGE);
}

void wheelSpeed(){
  int Lstate = digitalRead(encoder0pinA);
  if ((encoder0PinALast == LOW) && Lstate == HIGH){
    int val = digitalRead(encoder0pinB);
    if (val == LOW && Direction){
      Direction = false; // Reverse
    }
    else if (val == HIGH && !Direction){
      Direction = true; // Forward
    }
    encoder0PinALast = Lstate;
    if (!Direction) duration++;
    else duration --;
}
}

void advance(){
     //Motor Forward
     digitalWrite(M_left,LOW);
     analogWrite(E_left,val_output);
}

void back(){
     //Motor reverse
     digitalWrite(M_left,HIGH);
     analogWrite(E_left,val_output);
}

void Stop(){
     //Motor stops
     digitalWrite(E_left, LOW); 
}

  

