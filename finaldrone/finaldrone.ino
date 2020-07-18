#include <TinyGPS++.h>
#include <SoftwareSerial.h>
String Latitude;
String Longitude;
String Altitude;
String Velocity;
String Direction;
String Distance;

 const int voltageSensor = A0;
int sensorPin = A1; 
float vOUT = 0.0;
float vIN = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
int value = 0;

// (tony vars)
#include <avr/sleep.h>//this AVR library contains the methods that controls the sleep modes
#define interruptPin 2 //Pin we are going to use to wake up the Arduino

int buttonPin = 6;
int buttonNew;
int buttonOld = 1;
int debounce = 100;
volatile int arduino_state = 1;

// (tony vars end)
/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
SoftwareSerial mySerial(8, 9); 
void setup()
{
  Serial.begin(115200);
  //ss.begin(GPSBaud);

  //(tony setup)
  pinMode(interruptPin, INPUT_PULLUP);//Set pin d2 to input using the buildin pullup resistor
  pinMode(buttonPin, INPUT);

  // (tony setup end)

  Serial.println(F("FullExample.ino"));
  Serial.println(F("An extensive example of many interesting TinyGPS++ features"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  Serial.println(F("Satso0iu8 HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));
}

void loop()
{ 
  //(tomny loop)

  buttonNew = digitalRead(buttonPin);
  //Serial.print("Sleep Button state : ");
  //Serial.println(buttonNew);
  
  if(buttonOld == 1 && buttonNew==0){
    Serial.println("\nSending the arduino to sleep mode...");
    
    //Send the sms alert to the admin..
    mySerial.println("AT"); 
    updateSerial();
    mySerial.println("AT+CMGF=1"); 
    updateSerial();
    //mySerial.println("AT+CMGS=\"+254707314476\"");
    mySerial.println("AT+CMGS=\"+254722534687\"");
    updateSerial();
    mySerial.print("The Arduino is going to sleep in 5 Seconds..."); 
    updateSerial();
    mySerial.write(26);
    delay(5000);

    // Call the sleep function
    go_to_sleep();
  }
  buttonOld = buttonNew;
  delay(debounce);

  //(tony loop end)

  
   int reading = analogRead(sensorPin);  
   if(reading>=1){
      ss.begin(9600);
  voltage();
  beitian();
  delay(500);
  ss.end();
  delay(1000);
  if(Latitude=="0.0000000"){
  
    Serial.println("not ready");
    
    }
    else{
        mySerial.begin(9600);

  Serial.println("Initializing..."); 
  delay(1000);

  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
 
  mySerial.println("AT+CMGF=1");// Configuring TEXT mode
  updateSerial();
  mySerial.println("AT+CMGS=\"20384\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  //mySerial.println("AT+CMGS=\"+254717861596\"");
  updateSerial();
  mySerial.print("utm " + Longitude + "@" + Latitude + "@" + Altitude + "@" + Velocity  + "@" + vIN); //text content
  updateSerial();
  mySerial.write(26);
  mySerial.end();
  delay(500);
      
      }

    }
else{
  ss.end();
        mySerial.begin(9600);

  Serial.println("Initializing..."); 
  delay(1000);

  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
 
  mySerial.println("AT+CMGF=1");// Configuring TEXT mode
  updateSerial();
  //mySerial.println("AT+CMGS=\"20384\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  mySerial.println("AT+CMGS=\"+254707314476\"");
  updateSerial();
  mySerial.print("Device tamper"); //text content
  updateSerial();
  mySerial.write(26);
  mySerial.end();
  delay(500);
  
  mySerial.end();
  Serial.print("DEVICE TAMPER");
  
  }
}
//gsm
void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}




void beitian(){
  static const double LONDON_LAT = 32.867599, LONDON_LON = -80.037022;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

    Longitude = String(gps.location.lng(), 7);
Latitude = String(gps.location.lat(), 7);
Altitude = String(gps.altitude.meters()*3.281);
Velocity = String(gps.speed.kmph() / 3.6);
Direction = String(cardinalToLondon);
Distance = String(courseToLondon);

  
  }
// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

//VOLTAGE
void voltage(){
  
     value = analogRead(voltageSensor);
  vOUT = (value * 5.0) / 1024.0;
  vIN = vOUT / (R2/(R1+R2));
  Serial.print("Input = ");
  Serial.println(vIN);}


//(tony funtions)
void go_to_sleep(){
  /*
  This function puts the arduino to sleep after sending an sms alert to the admin.
  */  

  // Putting the Arduino to sleep
  sleep_enable();//Enabling sleep mode
  attachInterrupt(0, wake_from_sleep, LOW);//attaching a interrupt to pin d2
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);//Setting the sleep mode, in our case full sleep
  delay(1000); //wait a second to allow the led to be turned off before going to sleep
  sleep_cpu();//activating sleep mode
  Serial.println("Ardunino is up");//next line of code executed after the interrupt 
  
}

void wake_from_sleep(){
  Serial.println("Waking up (from trigger)");//Print message to serial monitor
  sleep_disable();//Disable sleep mode
  detachInterrupt(0); //Removes the interrupt from pin 2;
}

//(tony functions end)
