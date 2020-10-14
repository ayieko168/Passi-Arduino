#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <VoltageReference.h>
#include <TinyGPS.h>

//VARIABLE
String Latitude;
String Longitude;
String Altitude;
String Velocity;
String End;
String Elevation;
String vIN;
//END
//---Bicolor Status LED
int RED_LED = 8;
int GREEN_LED = 9;

SoftwareSerial Debug(6, 5); // Debugging (RX TX)
SoftwareSerial GPS_Serial(1, 0);   // Connected to GPS
//SoftwareSerial GSM_Serial (10,11);// connected to GSM
 SoftwareSerial mySerial(10, 11);  
   
Adafruit_BMP280 bmp;
VoltageReference vRef = VoltageReference();
TinyGPS gps;
//gsm variable
String apn = "safaricom";                       //APN
String apn_u = "saf";                     //APN-Username
String apn_p = "data";                     //APN-Password
String url = "http://utm.karimu.co.ke/iot-save.php";  //URL for HTTP-POST-REQUEST
String data1;   //String for the first Paramter (e.g. Sensor1)
String data2;   //String for the second Paramter (e.g. Sensor2)
//end
void setup() {

  pinMode(RED_LED, OUTPUT); // Status LEDs
  pinMode(GREEN_LED, OUTPUT); // Status LEDs
  
  vRef.begin();              //Vcc read library
//-----------------------------BMP 280------------------------------------
  if (!bmp.begin()) {
    //Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

//-----------------------------GPS------------------------------------
   Debug.begin(9600);
  
}


void loop() 
{
  
   GPS_Serial.begin(9600);
  parameters();
    GPS_Serial.end();
     mySerial.begin(9600);
 End = "0";
  mySerial.println("AT");
  runsl();//Print GSM Status an the Serial Output;
  delay(100);
  mySerial.println("AT+SAPBR=3,1,Contype,GPRS");
  runsl();
  delay(100);
  mySerial.println("AT+SAPBR=3,1,APN," + apn);
  runsl();
  delay(100);
 
  mySerial.println("AT+SAPBR =1,1");
  runsl();
  delay(500);
  mySerial.println("AT+SAPBR=2,1");
  runsl();
  delay(500);
  mySerial.println("AT+HTTPINIT");
  runsl();
  delay(500);
  mySerial.println("AT+HTTPPARA=CID,1");
  runsl();
  delay(100);
  mySerial.println("AT+HTTPPARA=URL," + url);
  runsl();
  delay(100);
  mySerial.println("AT+HTTPPARA=CONTENT,application/x-www-form-urlencoded");
  runsl();
  delay(100);
    mySerial.println("AT+HTTPDATA=192,10000");
  runsl();
  delay(100);
  mySerial.print("api_token=nCEdlyIEl95E2DkignU2XCLSSkzaPqphfWmeydcA6qiJvRxFcAwEEAD1izAG2fQ&longitude=" + Longitude + "&latitude=" + Latitude + "&altitude=" + Altitude + "&battery=" + vIN +"&end=" + End);
  //mySerial.print("api_token=vI89s84tfxMkjarIKy4mRmZNOwxF8F7pLST5QQxZkSSyZihtCdaWv2y5uIF6&longitude=" + Longitude + "&latitude=" + Latitude + "&altitude=" + Altitude + "&velocity=" + Velocity  + "&battery=" + vIN + "&End=" + End);
 //mySerial.println("api_token=vI89s84tfxMkjarIKy4mRmZNOwxF8F7pLST5QQxZkSSyZihtCdaWv2y5uIF6");
  runsl();
  delay(9000);
  mySerial.println("AT+HTTPACTION=1");
  runsl();
  delay(5000);
 mySerial.end();
 

}



void parameters(){
  

  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (GPS_Serial.available())
    {
      char c = GPS_Serial.read();
      Serial.println(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

    if (newData)
  {
    float flat, flon, GPS_altitude, Travel_speed;
    unsigned long age;
    GPS_altitude = gps.f_altitude();
    Travel_speed = gps.f_speed_kmph();
    
    gps.f_get_position(&flat, &flon, &age);

   Debug.println();
    Debug.println("Values:- ");
    Debug.print("LAT=");
    Serial.print(flat , 6);
    Debug.print(" LON=");
    Debug.print(flon , 6);
    Debug.print(" Speed=");
    Debug.print(Travel_speed , 6);
    Debug.print(" GPS Altitude=");
    Debug.print(GPS_altitude , 6);
    Debug.print(" SAT=");
    Debug.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Debug.print(" PREC=");
    Debug.println(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());

delay(500);
      Latitude = String(flat);
  Longitude = String(flon);
  //Altitude = String((GPS_altitude , 6));


 
  }
int Satellites =0;
Satellites = int(gps.satellites());

if(Satellites > 4 && Satellites <= 22 )
{
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
  }
else
{
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  }
  
  float vcc = vRef.readVcc()/1000.00;
  Debug.print("Battery Voltage: ");
  Debug.println(String(vcc) + " volts");


  
//    mySerial.print(bmp.readTemperature());
//    mySerial.println(" *C");
//
      Debug.print(F("Pressure = "));
      Debug.print(bmp.readPressure());
      Debug.println(" Pa");
      Debug.print(F("Approx altitude = "));
      Debug.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
      Debug.println(" m");


  Altitude = String((bmp.readAltitude(1013.25)));
  vIN = String(vcc);
 delay(1000);


  
  
  }


    
void runsl() {
  while (mySerial.available()) {
     Debug.print(mySerial.read());
  }

}
