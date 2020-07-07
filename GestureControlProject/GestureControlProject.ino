
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
//#include "I2Cdev.h" <- this is included in the path
#include <MPU6050.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO


// This definition is essential for Calibration and Gesture Recognition
#define CALIBRATION

#ifdef CALIBRATION
  #define CALIBRATED_PRINT(x) Serial.print(x)
  #define CALIBRATED_PRINTLN(x) Serial.println(x)
  const int AX = 0;
  const int AY = 1;
  const int AZ = 2;
#endif

#define ACC_AXIS 3
#define ZERO 0

// Uncomment to see a flood of comments on the Serial COM port indicating the exact point of failure and TimeOuts 
// of each state. Recommended to only be uncommented for debugging
//#define DEBUG_OUT

#define RECORD_VALIDATE

const int Axis = ACC_AXIS;
const int DataLength = 22;

int32_t DataQueue[Axis][DataLength];
int32_t Calibrated[Axis];
int32_t CurrentIndex[Axis];
int32_t TotalValue[Axis];

// These are threshold values that we vary to fine tune gesture recognition
int16_t X_pos_Threshold = 1700;
int16_t Y_pos_Threshold = 1900;
int16_t Z_pos_Threshold = 2000;

boolean StartRecordingFlag = false;
unsigned long Starttime = 0; 
unsigned long Endtime = 0;
unsigned long intermidiatetime = 0;
int16_t counter = 0;

unsigned long DurationofGesture = 0;
unsigned long DurationofPerformedGesture = 0;
boolean PerformGesture = false;
boolean Calculated = false;
unsigned long validationstart = 0;
unsigned long validationintermediate = 0;
unsigned long validationend = 0; 
unsigned long timertemp = 0;
int16_t Tcounts = 0;
int16_t XTcnts = 0;
int16_t YTcnts = 0;
int16_t ZTcnts = 0;
int16_t XFcnts = 0;
int16_t YFcnts = 0;
int16_t ZFcnts = 0;
int16_t Fcounts = 0;
int16_t counter2 = 0;
int8_t TrialCnt = 0;

const int8_t Trials = 3;
const int16_t TimeOut_Level = 330;
const int16_t Gesture_TimeOut = 3000;
const int buttonPin = PIN_SW0;    // the number of the pushbutton pin
const int ledPin    = PIN_LED_13; // the number of the LED pin

// Initialize container variables for recording accelerometer data
void InitDataHolders()
{
  for(int i=0 ;i<Axis;i++)
  {
    for(int j=0;j<DataLength;j++)
    {
        DataQueue[i][j] = 0;   
    }  

    Calibrated[i] = 0;
    CurrentIndex[i] = 0;
    TotalValue[i] = 0;
  }  
}

// Calibrate the AX AY AZ by averaging previous values
void CalibrateData(int Axis, int32_t val)
{ 
    TotalValue[Axis] -=  DataQueue[Axis][CurrentIndex[Axis]];
    DataQueue[Axis][CurrentIndex[Axis]] = val;
    TotalValue[Axis] += val;
    CurrentIndex[Axis]++;
    if(CurrentIndex[Axis]>=DataLength)
    {
      CurrentIndex[Axis] = 0;
    } 
    Calibrated[Axis] = TotalValue[Axis]/DataLength;  
}


int32_t CapturedDataQ[Axis][DataLength];
boolean TheMotionResult[Axis][DataLength];

// Initialise the Array
void InitializeCapturedDataQ()
{
  for(int i=0 ;i<Axis;i++)
  {
    for(int j=0;j<DataLength;j++)
    {
        CapturedDataQ[i][j] = 0;
        TheMotionResult[i][j] = false;   
    }   
  }
}


// Record the sequence of the input Recording Gesture!
void RecordGesSeq(int32_t val_Ax, int32_t val_Ay, int32_t val_Az)
{
  if(StartRecordingFlag == true)
  {
    timertemp = (millis()-intermidiatetime);
      if(timertemp > TimeOut_Level)
      {   
          intermidiatetime = millis();
          CapturedDataQ[AX][counter] = val_Ax;
          CapturedDataQ[AY][counter] = val_Ay;
          CapturedDataQ[AZ][counter] = val_Az;
          counter++;       
      }
      timertemp = millis() - Starttime; 
      if(timertemp > Gesture_TimeOut)
      {
        #ifdef DEBUG_OUT
          Serial.println("counter");
          Serial.println(counter);
        #endif
        Endtime = millis();
        StartRecordingFlag = false;
      }
  }
  
  if(!StartRecordingFlag && !Calculated)
  {
    Serial.println("Stopped Recording Data");
    DurationofGesture = Starttime - Endtime;
    Calculated = true;
    Serial.println("The System has recorded The Gesure and has entered into LOCKED STATE");
    Serial.println(">>>>>>>>LOCKED<<<<<<<<<");
    Serial.println("You can perform your gesture in 5 secs");
    delay(5000);
    PerformGesture = true;
    Serial.println("perform Key Gesture..:");
    validationstart = millis();
    validationintermediate = validationstart;
  }
}

// Validate the gesture using thresholds 
void ValidateGesture(int32_t val_Ax, int32_t val_Ay, int32_t val_Az)
{
  
    if(PerformGesture == true)
    {
       if(counter2 <counter)
       {
        timertemp = millis() - validationintermediate;
        if(timertemp > TimeOut_Level)
        {
          validationintermediate = millis();
          int16_t Value_diff = 0;
          if(val_Ax < (CapturedDataQ[AX][counter2] + X_pos_Threshold)  && val_Ax > CapturedDataQ[AX][counter2] - X_pos_Threshold)
          {
            TheMotionResult[AX][counter2] = true;
            Tcounts++;
            XTcnts++;
          } 
          else
          {
            TheMotionResult[AX][counter2] = false;
            Fcounts++;
            XFcnts++;
          }
          #ifdef VARIABLE_CHANGE
             Value_diff = val_Ax - CapturedDataQ[AX][counter2];
             Serial.println("Value Diff for Ax");
             Serial.println(Value_diff);
          #endif
          if(val_Ay < (CapturedDataQ[AY][counter2] + Y_pos_Threshold)  && val_Ay > CapturedDataQ[AY][counter2] - Y_pos_Threshold)
          {
            TheMotionResult[AY][counter2] = true;
            Tcounts++;
            YTcnts++;
          } 
          else
          {
            TheMotionResult[AY][counter2] = false;
             Fcounts++;
             YFcnts++;
          }
          #ifdef VARIABLE_CHANGE
            Value_diff = val_Ay - CapturedDataQ[AY][counter2];
            Serial.println("Value Diff for Ay");
            Serial.println(Value_diff);
          #endif
          if(val_Az < (CapturedDataQ[AZ][counter2] + Z_pos_Threshold)  && val_Az > CapturedDataQ[AZ][counter2] - Z_pos_Threshold)
          {
            TheMotionResult[AZ][counter2] = true;
            Tcounts++;
            ZTcnts++;
          } 
          else
          {
            TheMotionResult[AZ][counter2] = false;
            Fcounts++;
            ZFcnts++;
          }
          #ifdef VARIABLE_CHANGE
            Value_diff = val_Az - CapturedDataQ[AZ][counter2];
            Serial.println("Value Diff for Az");
            Serial.println(Value_diff);
          #endif 
          counter2++;
          
        }
       }
       else
       {
          PerformGesture = false;
          #ifdef DEBUG_OUT
            Serial.println("Trues");
            Serial.println(Tcounts);
            Serial.println("Fales");
            Serial.println(Fcounts);
            Serial.print("x");Serial.print("\t");
            Serial.print(XTcnts);Serial.print("\t");Serial.print(XFcnts);
            Serial.println();
            Serial.print("y");Serial.print("\t");
            Serial.print(YTcnts);Serial.print("\t");Serial.print(YFcnts);
            Serial.println();
            Serial.print("z");Serial.print("\t");
            Serial.print(ZTcnts);Serial.print("\t");Serial.print(ZFcnts);
            Serial.println(); 
          #endif
          if(Tcounts > Fcounts || (XTcnts > 3 && Tcounts == Fcounts))
          {
            Serial.println("Gesture Matched");
            Serial.println(">>>>>>>>>UNLOCKING SUCCESSFUL<<<<<<<<<");
            digitalWrite(ledPin, LOW);
            Serial.println("LED is ON");
            delay(3000); 
            digitalWrite(ledPin, HIGH);
            Serial.println("LED is OFF");
          }
          else
          {
            Serial.println("Gesture Match Failed");
          }
          TrialCnt--;           
          resetVar();
          DurationofPerformedGesture = 0;
          if(TrialCnt>0)
          {
             Serial.println("You can perform your gesture in 5 secs to Check again");
             delay(5000); 
             //Serial.println("perform gesture");
             Serial.println("Recording Motion"); 
             PerformGesture = true;
             validationstart = millis();
             validationintermediate = validationstart;   
          }
          else
          {
            Serial.println("Press Button to Record new Sequence!");
            counter = 0;
          }
       }
    }
}

void resetVar()
{
  Tcounts = 0;
  Fcounts = 0;
  counter2 = 0;
  XTcnts = 0;
  XFcnts = 0;
  YTcnts = 0;
  YFcnts = 0;
  ZTcnts = 0;
  ZFcnts = 0;
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    //Initialize Calibration Array
    InitDataHolders();
    
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // initialize the LED pin as an output:
    pinMode(ledPin, OUTPUT);
    // initialize the pushbutton pin as an input:
    pinMode(buttonPin, INPUT_PULLUP);

    //The recording counter
    counter = 0;

    Serial.println("Hit the button to start recording"); 
}

void loop() {
    
    // read raw accel measurements from device
    accelgyro.getAcceleration(&ax, &ay, &az);
    
    if(buttonPressed(buttonPin) == true)
    {
       digitalWrite(ledPin, HIGH);
       StartRecordingFlag = !StartRecordingFlag;
       if(TrialCnt == 0)
       {
        if(StartRecordingFlag == true)
        {
          Calculated = false;
          Serial.println("The Device will start recording Sequence in 7 secs,BE READY!");
          delay(5000);
          TrialCnt = Trials;
          Serial.println("Recording...");
          Starttime = millis();
          intermidiatetime = Starttime;
        }
       }
    }
  
    #ifdef CALIBRATION
         CalibrateData(AX, ax);
         CalibrateData(AY, ay);
         CalibrateData(AZ, az);
    #endif
    
    #ifdef OUTPUT_READABLE_ACCELGYRO
        #ifndef RECORD_VALIDATE
            Serial.print("a/g:\t");
            Serial.print(Calibrated[AX]); Serial.print("\t");
            Serial.print(Calibrated[AY]); Serial.print("\t");
            Serial.print(Calibrated[AZ]); Serial.print("\t");
            Serial.println();
        #endif
    #endif

    #ifdef RECORD_VALIDATE
        RecordGesSeq(Calibrated[AX], Calibrated[AY], Calibrated[AZ]);
        ValidateGesture(Calibrated[AX], Calibrated[AY], Calibrated[AZ]);
    #endif
}


// This is imported function for button press
int buttonPressed(uint8_t button) 
{
  static uint16_t lastStates = 0;
  uint8_t state = digitalRead(button);
  if (state != ((lastStates >> button) & 1)) 
  {
    lastStates ^= 1 << button;
    return state == HIGH;
  }
  return false;
}
