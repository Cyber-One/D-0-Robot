/*
 * D-0 control system V 0.0.1
 *  Use: 
 *  FS-iA6B receiver to Serial1.
 *  Mini MP3 Player
 *  Adafruit I2C 16ch servo driver
 *  
 *  Created by Ray Edgley
 */

#include <FlySkyiBus.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <DFMiniMp3.h>

// implement a notification class,
// its member methods will get called 
//
class Mp3Notify
{
public:
  static void OnError(uint16_t errorCode)
  {
    // see DfMp3_Error for code meaning
    Serial.println();
    Serial.print("Com Error ");
    Serial.println(errorCode);
  }

  static void OnPlayFinished(uint16_t globalTrack)
  {
    Serial.println();
    Serial.print("Play finished for #");
    Serial.println(globalTrack);   
  }

  static void OnCardOnline(uint16_t code)
  {
    Serial.println();
    Serial.print("Card online ");
    Serial.println(code);     
  }

  static void OnUsbOnline(uint16_t code)
  {
    Serial.println();
    Serial.print("USB Disk online ");
    Serial.println(code);     
  }

  static void OnCardInserted(uint16_t code)
  {
    Serial.println();
    Serial.print("Card inserted ");
    Serial.println(code); 
  }

  static void OnUsbInserted(uint16_t code)
  {
    Serial.println();
    Serial.print("USB Disk inserted ");
    Serial.println(code); 
  }

  static void OnCardRemoved(uint16_t code)
  {
    Serial.println();
    Serial.print("Card removed ");
    Serial.println(code);  
  }

  static void OnUsbRemoved(uint16_t code)
  {
    Serial.println();
    Serial.print("USB Disk removed ");
    Serial.println(code);  
  }
};

// software serial #2: RX = digital pin 8, TX = digital pin 9
// on the Mega, use other pins instead, since 8 and 9 don't work on the Mega
SoftwareSerial portTwo(8, 9);

FlySkyiBus iBus(10,10); //rx and tx pins

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Some arduino boards only have one hardware serial port, so a software serial port is needed instead.
// comment out the above definition and uncomment these lines
//SoftwareSerial secondarySerial(10, 11); // RX, TX
DFMiniMp3<SoftwareSerial, Mp3Notify> mp3(portTwo);

#define RX_Min 1000
#define RX_Max 2000

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  200 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  450 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  611 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2441 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define FREQ 50 // Analog servos run at ~60 Hz updates

#define Servo0_Range 30 // Left Wheel Max speed
#define Servo1_Range 30 // Right Wheel Max speed
#define Servo2_Range 50 // Head Tilt Max
#define Servo3_Range 150 // Head Tilt Max
#define Servo4_Range 40 // Head Rotate Max
#define Servo5_Range 100 // Main Bar Max Tilt

unsigned long CycleCounter, LastCycle;
bool RunCycle;
unsigned int RC_Last[10];
unsigned int Servo_Last[10];
bool RC_Changed[10];
int MP3track;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Startup Controller.");
  Serial.flush();

  Wire.begin();
  Serial.println("I2C (Wire) Started.");

// Start each software serial port
  portTwo.begin(9600);
  Serial.println("Software Serial for MP3 Player Started.");

  MP3track = 1;
  mp3.begin();
  Serial.println("MP3 Started.");

  pwm.begin();
  pwm.setPWMFreq(FREQ);  // Analog servos run at ~60 Hz updates
  Serial.println("I2C PWM Controler Started.");

  iBus.begin(115200);
  Serial.println("iBus Started.");

  mp3.playMp3FolderTrack(MP3track);  // sd:/mp3/0001.mp3

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

//  Serial.print(".");
  iBus.read_serial();
  for (int x=0; x<10; x++){
    bool RCchanged = false;
//    Serial.print(".");
    if(RC_Last[x] != iBus.get_channel(x)){
      RC_Last[x] = iBus.get_channel(x);
      RC_Changed[x] = true;
      RCchanged = true;
    } else RC_Changed[x] = false;
    if(RCchanged){ 
      PrintRCvalues();
      SetPWM();
    }
  }
  if (RC_Changed[9]){
    switch (RC_Last[6]) {
      case 1000:
        MP3track = 1;
        break;
      case 1100:
        MP3track = 2;
        break;
      case 1300:
        MP3track = 3;
        break;
      case 1400:
        MP3track = 4;
        break;
      case 1500:
        MP3track = 5;
        break;
      case 1600:
        MP3track = 6;
        break;
      case 1700:
        MP3track = 7;
        break;
      case 1800:
        MP3track = 8;
        break;
      case 1900:
        MP3track = 9;
        break;
      default:
        MP3track = 1;
    }
    if (RC_Last[9] > 1500) {
      RC_Changed[9] = false;
      mp3.playMp3FolderTrack(MP3track);
    }
  }
}

void SetPWM(){
  if (RC_Last[7] <  1500) {
    digitalWrite(2, HIGH);
  } else {
    digitalWrite(2, LOW);
  }
  unsigned int Left = RC_Last[1] + (RC_Last[0] - 1500);
  unsigned int Right = RC_Last[1] - (RC_Last[0] - 1500);
  if (RC_Last[0] == 1500 && RC_Last[1] == 1500){
    Servo_Last[0] = 0;
    Servo_Last[1] = 0;
  } else {
    Servo_Last[0] = map(constrain(Left, 1000, 2000), RX_Min, RX_Max, 319-Servo0_Range, 319+Servo0_Range);
    Servo_Last[1] = map(constrain(Right, 1000, 2000), RX_Min, RX_Max, 319+Servo1_Range, 319-Servo1_Range);
  }
  Servo_Last[2] = map(RC_Last[2], RX_Min, RX_Max, 320+Servo2_Range, 320-Servo2_Range);
  Servo_Last[3] = map(RC_Last[3], RX_Min, RX_Max, 320+Servo3_Range, 320-Servo3_Range);
  Servo_Last[4] = map(RC_Last[4], RX_Min, RX_Max, 280-Servo4_Range, 280+Servo4_Range);
  Servo_Last[5] = map(RC_Last[5], RX_Min, RX_Max, 280-Servo5_Range, 280+Servo5_Range);
  pwm.setPWM(15, 0, Servo_Last[0]);  // Left Wheel.
  pwm.setPWM(14, 0, Servo_Last[1]);  // Right Wheel.
  pwm.setPWM(13, 0, Servo_Last[5]);  // Main Arm.
  pwm.setPWM(12, 0, Servo_Last[2]);  // Nod Bar.
  pwm.setPWM(10, 0, Servo_Last[3]);  // Head Rotate.
  pwm.setPWM(11, 0, Servo_Last[4]);  // Head Tilt.
}

void PrintRCvalues(){
  for (int x=0; x<10; x++){
    Serial.print(RC_Last[x], DEC);
    Serial.print(" (");
    Serial.print(Servo_Last[x], DEC);
    Serial.print(")\t");
  }
  Serial.println();
  Serial.flush();
}
