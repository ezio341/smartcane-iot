#include <TinyGPS++.h> // library for GPS module
#include <SoftwareSerial.h> // library to convert signal from serial
#include <ESP8266WiFi.h>
#include "FirebaseESP8266.h"  // Install Firebase ESP8266 library
#include <Time.h>
#include <NTPClient.h>// library to get date and time from pool.ntp.org
#include <WiFiUdp.h>

#define FIREBASE_HOST "YOUR_REALTIMEDATABASE_NAME" //Without http:// or https:// schemes
#define FIREBASE_AUTH "YOUR_PROJECT_DATABASESECRET"

FirebaseData firebaseData;

TinyGPSPlus gps;                          // The TinyGPS++ object
static const int RXPin = 4, TXPin = 5;    // define the serial receiver and transfer
SoftwareSerial ss(RXPin, TXPin);          // The serial connection to the GPS device

const char* ssid = "YOUR_WIFI_SSID";            //ssid of your wifi
const char* password = "YOUR_WIFI_PASSWORD";      //password of your wifi

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

//attribute to store data from gps, sensor, and NTP server
float latitude , longitude, distance;
String date_str , time_str , lat_str , lng_str, datetime_str, dist_risk;

const int pingPin = D6; // Trigger Pin of Ultrasonic Sensor
const int echoPin = D7; // Echo Pin of Ultrasonic Sensor
int piezoPin = D5;      // output pin for buzzer piezo

//============================================================================================================

void setup()
{
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(piezoPin, OUTPUT);
  Serial.begin(115200);
  ss.begin(9600);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password); //connecting to wifi
  while (WiFi.status() != WL_CONNECTED)// while wifi not connected
  {
    delay(500);
    Serial.print("."); //print "...."
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());  // Print the IP address

  timeClient.begin();
  timeClient.setTimeOffset(25200); // Time offset of west part of Indonesia

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

}

//================================================================================================================

void getTime(){
  timeClient.update();

  unsigned long epochTime = timeClient.getEpochTime();
  
  String formattedTime = timeClient.getFormattedTime();

  //Get a time structure
  struct tm *ptm = gmtime ((time_t *)&epochTime); 
  int monthDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon+1;
  int currentYear = ptm->tm_year+1900;

  //Print complete date:
  String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay);
  date_str = currentDate;
  time_str = formattedTime;
  datetime_str = currentDate+"/"+formattedTime;
  delay(1);
}

//==============================================================================================================

void pushFirebase(){
  if(Firebase.setDouble(firebaseData, "/rtLocation/latitude", latitude));

  if(Firebase.setDouble(firebaseData, "/rtLocation/longitude", longitude));
  if(Firebase.setDouble(firebaseData, "/rtLocation/longitude", longitude));
  if(Firebase.setDouble(firebaseData, "/rtProxSensor/distance/", distance));
  if(Firebase.setString(firebaseData, "/rtProxSensor/risk/", dist_risk));

  String timeNow = datetime_str;

  if(Firebase.setDouble(firebaseData, "/history/"+timeNow+"/latitude", latitude));
  if(Firebase.setDouble(firebaseData, "/history/"+timeNow+"/longitude", longitude));
  if(Firebase.setDouble(firebaseData, "/history/"+timeNow+"/distance/", distance));
  if(Firebase.setString(firebaseData, "/history/"+timeNow+"/risk/", dist_risk));
  delay(1);
}

//======================================================================================================================
void senseDistance(){
  long duration, inches, cm;
  digitalWrite(pingPin, LOW);
  delayMicroseconds(10);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  distance = cm;
  if(cm>50 && cm < 100){
    tone(piezoPin, 500, 500);
    dist_risk="MEDIUM";
  }else if(cm<50){
    tone(piezoPin, 500, 1000);
    dist_risk="HIGH";
  }else{
    tone(piezoPin, 0, 0);
    dist_risk="LOW";
  }
  
  delay(1);
}

long microsecondsToInches(long microseconds)      // convert microseconds from ultrasonic sensor to inches
{
return microseconds / 74 / 2;
}
 
long microsecondsToCentimeters(long microseconds) // convert microseconds from ultrasonic sensor to centimeters
{
return microseconds / 29 / 2;
}

//============================================================================================================

void loop()
{
  senseDistance();
  while (ss.available() > 0) //while data is available
    if (gps.encode(ss.read())) //read gps data
    {
      if (gps.location.isUpdated() && gps.location.isValid()) //check whether gps location is valid
      {
        getTime();
        latitude = gps.location.lat();  // get latitude
        lat_str = String(latitude , 6); // latitude location is stored in a string
        longitude = gps.location.lng(); // get longitude
        lng_str = String(longitude , 6); //longitude location is stored in a string
        pushFirebase();
      }
      delay(1);
    }
  delay(50);
}
