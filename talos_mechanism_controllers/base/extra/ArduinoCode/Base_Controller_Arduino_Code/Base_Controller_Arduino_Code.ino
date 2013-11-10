
#include <parser.h>
#include <Encoder.h>
#include <PID_v1.h> // Include PID library for closed loop control
#include <basemtrcontrol.h>
  
#define INPUT_PULLUP 0x2
  
///////////////////////////////////////////////////////////////////////////////////Variables/////////////////////////////////////////////////////////////////////////////////////////////////
  
double error = 0;
  
/////////////////////////////////////////////////
  
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete


///////////////////////////////////////////////////////
  
//Thethered control: ANALOG PINS
const int        twistRemotePin = 3, linearRemotePin = 4, RemotePowerPin = 46;
const int        rightKillPin = 50, leftKillPin = 48;
int              _RightMotorKillValue = 0;
int              _LeftMotorKillValue = 0;
  
  
double val0 = 200; // Duty Cycle val = 0 gives 0% DUTY, val = 254 gives 100% DUTY
double rightPotValue = 200; // Duty Cycle val = 0 gives 0% DUTY, val = 254 gives 100% DUTY
  
const int        INA1 = 47; // These pins control the state of 
const int        INB1 = 53; // the bridge in normal operation: 
const int        INA2 = 39;
const int        INB2 = 45;
  
//  [HIGH HIGH = BRAKE to Vcc] 
//  [HIGH LOW = CLOCKWISE]
//  [LOW HIGH = Counter Clock Wise]
//  [LOW LOW = BRAKE to ground]
 
  const int        ENA1 = 49; //LOW Disables Half Bridge A HIGH Enables half bridge A
  const int        ENB1 = 51; //LOW Disables Half Bridge B HIGH Enables half bridge B
  const int        ENA2 = 41;
  const int        ENB2 = 43;
  const int        PWMpin1 = 7;
  const int        PWMpin2 = 6;
  
  //Encoder count variables:   
  double           _leftMotorRPM = 0;
  double           _rightMotorRPM = 0;
  long             _newLeftEncoderPosition = 0;
  long             _newRightEncoderPosition = 0;
  long             _oldLeftEncoderPosition = 0;
  long             _oldRightEncoderPosition  = 0;
  float            _leftCount = 0;
  float            _rightCount = 0;
  
  const float      CountsPerRotation = 4680;
  const float      MultiplicationFactor = 6; //counts per encoder impulse
  
  
  //PID Variables:
  double _rightMotorRPMset = 0, _rightInput = 0, _rightOutput = 0;
  double _leftMotorRPMset = 0, _leftInput = 0, _leftOutput = 0;
  const float SampleTime = 50;
  int Kp = 160, Kd = 0, Ki = 1000;
  int KpIN = 0, KdIN = 0, KiIN = 0;
  int KpPin = 5, KdPin = 6, KiPin = 7;
  double MaxRPM = 1.45;
  
  //Joystick Control Variables:
  double linearXHigh = 1023; 
  double linearXLow = 135;
  double linearXCentre = 560;
  double linearAnalogPin = 3;
  double twistXHigh = 1023;
  double twistXLow = 125;
  double twistXCentre = 480;
  double test = 0;

  // serial msg variables:
  double _interLeftMotorRPMSet;
  double _interRightMotorRPMSet;
  
  //Initialize encoder inputs
  Encoder LeftEncoder(2, 3);
  Encoder RightEncoder(18,19);
  
  // Initialize PID
  PID rightPID(&_rightInput, &_rightOutput, &_rightMotorRPMset, Kp, Ki, Kd, DIRECT);
  PID leftPID(&_leftInput, &_leftOutput, &_leftMotorRPMset, Kp, Ki, Kd, DIRECT);


void setup()
{    
/////////////////////////////////////////////////////////////////////////////////void setup //////////////////////////////////////////////////////////////////////////////////////////////////  
  Serial.begin(9600);
  
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);  
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(ENA1, OUTPUT);
  pinMode(ENB1, OUTPUT);
  pinMode(ENA2, OUTPUT);
  pinMode(ENB2, OUTPUT);
  pinMode(rightKillPin,INPUT_PULLUP);
  pinMode(leftKillPin,INPUT_PULLUP);
  pinMode(RemotePowerPin,INPUT);
  
   inputString.reserve(100);
    
  // Initialize Velocity Setpoint
  _rightMotorRPMset = 1;  
  
  //turn the PID on
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(SampleTime);
  
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(SampleTime);
    
}

void loop()
{
/////////////////////////////////////////////////////////////////////////////////void loop///////////////////////////////////////////////////////////////////////////////////////////////////
      //SetPIDValues(); //Set Kp, Ki, Kd
      
      // TODO ?should? be able to move to setup
      rightPID.SetTunings(Kp, Ki, Kd);
      leftPID.SetTunings(Kp, Ki, Kd);
      
     _LeftMotorKillValue = digitalRead(leftKillPin);
     _RightMotorKillValue = digitalRead(rightKillPin);      //print analog pin 3 input value
     
     KillMotor(_LeftMotorKillValue, ENA1, ENB1);      //enable or disable H-bridge 2 based on input from analog input. HIGH = OFF, LOW = ON
     KillMotor(_RightMotorKillValue, ENA2, ENB2);           //Kill Right motor when kill cable is pulled
     
     //calculates setRPM based on analog joystick signals
     //TODO can be can only when new information available.
  
     _leftMotorRPMset = _interLeftMotorRPMSet; // TODO I think this is where to put twist messagesbnb
     _rightMotorRPMset = _interRightMotorRPMSet; 
     
     setMotorDirection(_leftMotorRPMset, INA1, INB1);  //Sets Pin outs for Pololu 705 
     setMotorDirection(_rightMotorRPMset, INA2, INB2);
     
     _leftMotorRPMset = abs(_leftMotorRPMset);
     _rightMotorRPMset = abs(_rightMotorRPMset);
     
     // calculates speed wheels are rotating at
     
     // this part must run each loop
     // down to delay()
     computeLeftRPM();   
     computeRightRPM();                                     //RIGHT motorSpeed compute
     
     _leftInput = _leftMotorRPM;
     _rightInput = _rightMotorRPM;
  
     leftPID.Compute();                              //Compute new PID values given input = _rightMotorRPM
     rightPID.Compute();
     
     analogWrite(PWMpin1, _leftOutput); 
     analogWrite(PWMpin2, _rightOutput); 
     
     delay(SampleTime); //loop DELAY
     
     
   
  ///////////////////////////////////////////////////////////////////////////////// end void loop/////////////////////////////////////////////////////////////////////////////////////////////
}

//////////////////////////////////////////////////////////////////////////////////////Functions///////////////////////////////////////////////////////////////////////////////////////////////
//
//FUNCTION: SetPIDValues: This function is used to set the PID values of the MCU 
//using three potentiometers. Each potentiometer is power between 5V and GND. 
//The gains Kp, Kd, and Ki, are changed through the analog signals is input into 
//the Arduino Mega pins: 5, 6, and 7 respectively. 

void SetPIDValues()
{
  //read PID input values from potentiometers
  KpIN = analogRead(KpPin);
  KdIN = analogRead(KdPin);
  KiIN = analogRead(KiPin);
    
  if(KpIN/2 != Kp){
    Kp = KpIN;
    rightPID.SetTunings(Kp, Ki, Kd);
    leftPID.SetTunings(Kp, Ki, Kd);
  }
  if(KdIN/2 != Kd){
    Kd = KdIN/2;
    rightPID.SetTunings(Kp, Ki, Kd);
    leftPID.SetTunings(Kp, Ki, Kd);
  }
  if(KiIN/2 != Ki){
    Ki = KiIN;
    rightPID.SetTunings(Kp, Ki, Kd);
    leftPID.SetTunings(Kp, Ki, Kd);
  }
}


//FUNCTION: getCount: This function is used to determine the count of the encoders after rotation of one loop
double getCount( long _oldEncoderPosition, long _newEncoderPosition)
{
  float _Count = 0;
  _Count = _newEncoderPosition - _oldEncoderPosition; //leftCount is the # of (encoder pulses)*(N=6) per "SampleTime"
  
  return _Count;
}

//FUNCTION: getRPM returns ABSOLUTE Value of the shaft velocity

float getRPM(double _Count,  double SampleTime, int CountsPerRotation, int MultiplicationFactor)
{
  double _MotorRPM = 0;
  _MotorRPM = _Count/((SampleTime/1000)*CountsPerRotation*MultiplicationFactor); //Calculates actual RPM of motor shaft
  
  if(_MotorRPM < 0){
    _MotorRPM = -_MotorRPM;
  }
  
  return _MotorRPM;
}

int KillMotor(int analogPinValue, int ENA, int ENB )
{
   if (analogPinValue == LOW)
   {
       digitalWrite(ENA, HIGH);
       digitalWrite(ENB, HIGH);
   }
   else
   {  
       digitalWrite(ENA, LOW);
       digitalWrite(ENB, LOW);
   }
   
   return 0;
}


void computeRightRPM()
{
  _newRightEncoderPosition = RightEncoder.read();
  _rightCount = getCount( _oldRightEncoderPosition, _newRightEncoderPosition);
  _rightMotorRPM = getRPM(_rightCount, SampleTime, CountsPerRotation, MultiplicationFactor);
  
  if (_newRightEncoderPosition != _oldRightEncoderPosition) 
  {
  _oldRightEncoderPosition = _newRightEncoderPosition;
  }
}


    void computeLeftRPM()
{
    _newLeftEncoderPosition = LeftEncoder.read();  
    _leftCount = getCount( _oldLeftEncoderPosition, _newLeftEncoderPosition);
 
     if (_newLeftEncoderPosition != _oldLeftEncoderPosition) 
     {
     _leftMotorRPM = getRPM(_leftCount, SampleTime, CountsPerRotation, MultiplicationFactor);
     _oldLeftEncoderPosition = _newLeftEncoderPosition;
  }
}


double CalcRPMSet( double XHigh, double XLow, double XCentre, double AnalogInput, int PowerPin)
{
  double SlopeHigh = 0, SlopeLow = 0;
  double FHigh = 0, FLow = 0;
  double MinOut = 0.3, MaxOut = (0.9*MaxRPM), output = 0; 
  double X = 0;
  double power = 0;
  X = analogRead(AnalogInput);
  power = digitalRead(PowerPin);
  if(power == HIGH){
  if( X > XCentre){
    SlopeHigh = (MaxRPM/(XHigh-XCentre));
    FHigh = SlopeHigh*(X-XCentre);
    if((FHigh >= MinOut)&&(FHigh <= MaxOut)){
      output = -FHigh;
    }
    else if(FHigh < MinOut){
      output = 0;
    }
    else if(FHigh > MaxOut){
      output = -MaxOut;
    }
  }
  else if( X <= XCentre){
    SlopeLow = (MaxRPM/(XLow-XCentre));
    FLow = SlopeLow*(X-XCentre);
    if((FLow >= MinOut)&&(FLow <= MaxOut)){
      output = FLow;
    }
    else if(FLow < MinOut){
      output = 0;
    }
    else if(FLow > MaxOut){
      output = MaxOut;
    }
  }
  }
  else{
    output = 0;
  }
  return output;
}

int setMotorDirection(double Velocity, int INA, int INB)
{
  if(Velocity > 0){
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
  }
  else if (Velocity < 0){
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
  }
  else{
    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);
    Velocity = 0;
  }
  return 1;
}


double getLeftTwistRPM()
{
  double twistRPM = 0;
  double leftTwistRPM = 0;
  //double rightTwistRPM = 0;
  twistRPM = CalcRPMSet( twistXHigh, twistXLow , twistXCentre, twistRemotePin, RemotePowerPin);
  leftTwistRPM = -twistRPM;
  
  return leftTwistRPM;
   
}


double getRightTwistRPM()
{
  double twistRPM = 0;
  //double leftTwistRPM = 0;
  double rightTwistRPM = 0;
  twistRPM = CalcRPMSet( twistXHigh, twistXLow , twistXCentre, twistRemotePin, RemotePowerPin);
  rightTwistRPM = twistRPM;
  
  return rightTwistRPM;

}

double getLinearRPM()
{
  double linearRPM = 0;
  linearRPM = CalcRPMSet( linearXHigh, linearXLow , linearXCentre, linearRemotePin, RemotePowerPin);
  
  return linearRPM;
  
}
    
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      
    if (stringComplete) {
      OnReceived(inputString);

    // clear the string:
    inputString = "";
      stringComplete = false;
      }  
    } 
  }
}

void PrintDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

    Serial.print (int(val));  //prints the int part
    Serial.print("."); // print the decimal point
    unsigned int frac;
    if(val >= 0)
        frac = (val - int(val)) * precision;
    else
        frac = (int(val)- val ) * precision;
    Serial.println(frac,DEC) ;
}

void ProcessTwist(double * twistvals) {
 _interLeftMotorRPMSet = CalcRPS(twistvals, 'L');
 Serial.print("Left Wheel: ");
 Serial.println(_interLeftMotorRPMSet);
 
 _interRightMotorRPMSet = CalcRPS(twistvals, 'R');
 Serial.print("Right Wheel: ");
 Serial.println(_interRightMotorRPMSet);
 
}

void OnReceived(String s) {
  if (s == "initializing\n") {
    Serial.print(s);
  } else {
    Serial.print(s); // TODO debug
      //copy String object's contents into char array
      char carr[s.length()];
      s.toCharArray(carr, s.length());
      
      //results container.
      //double array contains parsed values.
      double parsedvals[3];
      
      //parse character array into results container
      ParseTwist((const char*) carr, parsedvals);
      
      //handle results
      ProcessTwist(parsedvals);
  }
}
