#include <TinyGPSPlus.h>


#include <SoftwareSerial.h>

// MPU9250 code below unti hastages end

#include <MPU9250_WE.h>

#include <Wire.h>


#define MPU9250_ADDR 0x68

/*


   This sample code demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.


   It requires the use of SoftwareSerial, and assumes that you have a


   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).


*/


static const int RXPin = 4, TXPin = 3;


static const uint32_t GPSBaud = 9600;


float waypoint_lat_x[] = {32.323232, 13.234, 19.232};

float waypoint_lng_y[] = {93.31, 14.414, 13.13};


// The TinyGPSPlus object


TinyGPSPlus gps;


 


// The serial connection to the GPS device


SoftwareSerial ss(RXPin, TXPin);


 

// MPU9250 code below


MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);



// Rotary Encoder Inputs
#define CLK 12
#define DT 11
#define SW 10

int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";
unsigned long lastButtonPress = 0;


void setup()


{


  Serial.begin(115200);


  ss.begin(GPSBaud);


 


  Serial.println(F("FullExample.ino"));


  Serial.println(F("An extensive example of many interesting TinyGPSPlus features"));


  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());


  Serial.println(F("by Mikal Hart"));


  Serial.println();


  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));


  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));


  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));







// MPU9250 code below


  Wire.begin();

  if(!myMPU9250.init()){

    Serial.println("MPU9250 does not respond");

  }

  else{

    Serial.println("MPU9250 is connected");

  }

  if(!myMPU9250.initMagnetometer()){

    Serial.println("Magnetometer does not respond");

  }

  else{

    Serial.println("Magnetometer is connected");

  }


  /* You can choose the following operational modes

   * AK8963_PWR_DOWN            power down (default)

   * AK8963_CONT_MODE_8HZ       continuous at 8Hz sample rate

   * AK8963_CONT_MODE_100HZ     continuous at 100Hz sample rate 

   * 

   * In trigger mode the AK8963 goes into power down after the measurement

   */

  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

  

  /* In continuous mode you need to wait for the first data to be available. If you 

   * comment the line below you will probably obtain zero. 

   */

  delay(200);


// rotary encoder inouts
 // Set encoder pins as inputs
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  pinMode(SW, INPUT_PULLUP);

  // Setup Serial Monitor
  Serial.begin(115200);

  // Read the initial state of CLK
  lastStateCLK = digitalRead(CLK);

}




 


void loop()


{


  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;


 


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


  smartDelay(20);


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


  smartDelay(20);


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


  smartDelay(20);


}


 


static void printStr(const char *str, int len)


{


  int slen = strlen(str);


  for (int i=0; i<len; ++i)


    Serial.print(i<slen ? str[i] : ' ');


  smartDelay(20);






// MPU9250 code below

 xyzFloat magValue = myMPU9250.getMagValues(); // returns magnetic flux density [µT] 


  Serial.println("Magnetometer Data in µTesla: ");

  Serial.print(magValue.x);

  Serial.print("   ");

  Serial.print(magValue.y);

  Serial.print("   ");

  Serial.println(magValue.z);


  delay(1000);
float Pi = 3.14159265358979323846264338327950288419716939937510;
float Heading = (atan2(magValue.x ,magValue.y) * 180) / Pi;
Serial.print(Heading);


// rotatry encoder inouts
// Read the current state of CLK
  currentStateCLK = digitalRead(CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK) {
      counter --;
      currentDir ="CCW";
    } else {
      // Encoder is rotating CW so increment
      counter ++;
      currentDir ="CW";
    }

    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.println(counter);
    
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;

  // Read the button state
  int btnState = digitalRead(SW);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //if 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50) {
      Serial.println("Button pressed!");
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

  // Put in a slight delay to help debounce the reading
  delay(10);
  float WindDirection = (counter * 24);
  Serial.println(WindDirection);
  smartDelay(20);
}
