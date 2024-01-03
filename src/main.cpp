/***********************************************************
 * Smart Helmet project by Evyatar Cohen & Lior Kestelbaum *
 *               Supervised by Mony Orbach                 *
 *                   HS DSL Lab, Technion                  *
 *                        01/2024                          *
************************************************************/

#include "Audio.h"
#include "SD.h"
#include "FS.h"
#include <Wire.h>


#include <TinyGPS++.h>
#include <SoftwareSerial.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MPU6050.h>

#include <BluetoothSerial.h>

//check if Bluetooth is properly enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// Digital I/O used
#define SD_CS          5
#define SPI_MOSI      23    // SD Card
#define SPI_MISO      19
#define SPI_SCK       18

#define I2S_DOUT     25
#define I2S_BCLK     26     // I2S
#define I2S_LRC      27

#define NUM_OF_ITER_AUDIO 5000 // for playing audion in a while loop
#define NAME_LEN 40
#define SOS_SIGNAL 101 // this sequence is sent when hit is detected

//Emergency
String  EmergencyLocation;
bool EmergencyFlag = false;
bool EmergencyDetectionFlag = false;

Audio audio;

// PINs name - number
//LED, Haptic feedback
const int Left_LED_Pin = 13 ;
const int Right_LED_Pin = 14 ;
const int Left_Haptic_Pin = 4 ;
const int Right_Haptic_Pin = 16 ;

// const variable 
static const uint32_t GPSBaud = 9600;

//compass
float compass[4]; //Heading, X, Y, Z

//ROUTE & DISTENTION
bool initialized = false;
const int scannerLength = 2;
double distanceToNextDest[scannerLength]; 
String routeData = "";
static const int MaxRouteSize = 1000;
float route[2][MaxRouteSize];
int routeLength = 0;
int nextDest = 1;
static const int ArrivalRadius = 20; //Radius of the next destination is 20 meters
double mostRecentLatt;
double mostRecentLon;
double minDest;
double a_threshold = 8; //acceleration threshold [g], set to small value for testing. Should be 4g

int last_navDecision = 0 ; //straight


//Navigation algorithm improvement using triangles to "predict" turns
int POI_Turns[2][MaxRouteSize];
bool first_time_right = true ; //at the first time we don't want to hear the turn right sound, because it comes from default variables

// function declarations
void play_sound(const char* file_name) ;
void config_pinout();
void mpu_setup();
void compass_setup();
bool get_new_GPS_compass_data();
void get_route();
void gps_error();
void get_compass();
int decideDirections(float compass[], float posLat, float posLon, float destLat, float destLon);
void ActivateNavigationLEDs(int decision, int last_decision);
void turn_left() ;
void turn_right();
static float CalcAngleBetweenVectors(float v1_x, float v1_y, float v2_x, float v2_y);
static void GetPointsOfInterest(int POI_Turns[2][MaxRouteSize] , float route[2][MaxRouteSize], int routeLength);
static void ConstructRouteArray(String routeData, float route[2][MaxRouteSize], int index);
void getCompass(float *compass);
void finish();


// The TinyGPS++ object
TinyGPSPlus gps;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); //Compass
Adafruit_MPU6050 mpu;  //Accelerometer


 
void setup() {
    Serial.begin(9600);
    while (!Serial);             //wait for serial monitor
    Serial.println("Starting setup");
    Serial.println("[APP] Free memory: " + String(esp_get_free_heap_size()) + " bytes");
    config_pinout();
    
    pinMode(SD_CS, OUTPUT);      
    digitalWrite(SD_CS, HIGH);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    if(!SD.begin(SD_CS))
    {
      Serial.println("Error talking to SD card!");
      while(true);  // end program
    }

    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(21); // 0...21
    
    Serial.println("End setup");
}



bool first_gps_msg = true;

void loop()
{
  //Serial.println("loop start");
  if (!initialized) {
    play_sound("turn_on.wav");
    get_route();
    play_sound("route_received.wav");
    delay(500);
    

    initialized = true;
  }
  
  /*****************************Navigation***********************************/
  
  if(nextDest < routeLength) //Still haven't finished the route.
  {
    while (SerialBT.available() > 0)
    {
        if (first_gps_msg) {
          first_gps_msg = false;
          //play_sound("car_engine.wav");
        }
        if (get_new_GPS_compass_data()) /*LIOR*/
        {
					//Printing next destination for debugging
          Serial.println(F("The Next Point is: "));
          Serial.print(nextDest);
          Serial.println();
          
          int MaxScannerLength = 0; //Maximum scanner length allowed 
          //(important only when approaching the end of the route array, in order not to go out of bounds)
          
          //Clac distance of current pos and current next dest, and with 3 point distance scanner
          for(int i = 0;i < scannerLength; i++)
          {
            if(nextDest + i < routeLength)
            {
							//Calculate the distance from the rider's location to next destination
              distanceToNextDest[i] = TinyGPSPlus::distanceBetween(mostRecentLatt,
                                  mostRecentLon, route[0][nextDest + i], route[1][nextDest + i]); 
              Serial.print(F("Distance nextdest "));
              Serial.print(i);
              Serial.print("  : ");
              Serial.print(distanceToNextDest[i]);
              Serial.println();
              MaxScannerLength++;
            }
          }
          if(POI_Turns[0][nextDest] == 1)
          {
            MaxScannerLength = 1;
          }
          //Get min distnce from scanner (this checks if some points were skipped)
          minDest = distanceToNextDest[0];
          int skipIndex = 0;
          if(nextDest != routeLength - 1)
          {
            Serial.print(F("MaxScannerLength =  "));
            Serial.print(MaxScannerLength);
            Serial.println();
            for(int i = 0;i < MaxScannerLength /*MaxScanner Length (in order not to go out of bounds)*/ ; i++)
            {
              if(distanceToNextDest[i] <= minDest)
              {
                minDest = distanceToNextDest[i];
                skipIndex = i;
              }
            }
          }
          else
          {
            skipIndex = 0;
          }
          Serial.print(F("SkipIndex =  "));
          Serial.print(skipIndex);
          Serial.println();
          //Check if we arrived at current next dest, and check if skipped in order to update nextdest accordingly
          if(skipIndex == 0)
          {
            if(minDest < ArrivalRadius)
            {
              nextDest++;
              Serial.println(F("Next Dest Updated: "));
              Serial.print(nextDest);
              Serial.println(F("   NO SKIP"));
              Serial.println();
            }
          }
          else //(if skipindex !=0) if we skipped the original next dest.
          {
            double distanceSkip = TinyGPSPlus::distanceBetween(route[0][nextDest + skipIndex], //distance between dest to skip to, and nextdest to skip to
              route[1][nextDest + skipIndex], route[0][nextDest + skipIndex + 1], route[1][nextDest + skipIndex + 1]);
            if(distanceSkip < distanceToNextDest[skipIndex + 1])//meaning position is now behind nextskip
            {
              Serial.print(F("NextDest: "));
              Serial.print(nextDest);
              Serial.print(F("    SkipIndex: "));
              Serial.print(skipIndex);
              nextDest = nextDest + skipIndex;
              Serial.print(F("    Next Dest + SkipIndex: "));
              Serial.print(nextDest);
              Serial.println();
            }
            else //position is closer to nextskip + 1 (skip an extra dest)
            {
              nextDest = nextDest + skipIndex + 1;
              Serial.print(F("Next Dest Updated With Skip +1: "));
              Serial.print(nextDest);
              Serial.println();
            }
          }
        }
        else //Gps is not working properly (invalid data)
        {
          gps_error();
        }
    }
      
    //Calculate where to go
    
    //If the next dest is a point of interest (indicates a junction)
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(POI_Turns[0][nextDest] == 1)
    {
      Serial.print(F("MIN DEST = "));
      Serial.print(minDest);
      Serial.println();
      if(minDest < 35) //indicate a turn a bit before arriving to the junction 
      {
        nextDest++;
      }
    }

    int navDecision = decideDirections(compass, mostRecentLatt, mostRecentLon, route[0][nextDest], route[1][nextDest]);
    ActivateNavigationLEDs(navDecision , last_navDecision);
    last_navDecision = navDecision ;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  } 
  else //Finished the route.
  {
    finish();
  }
  /*****************************Navigation End********************************/

  /*****************************Emergency*************************************/ 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

	//Check if an accident happened (acceleration passed the threshold)
  if (abs(a.acceleration.x) > a_threshold || abs(a.acceleration.y) > a_threshold  || abs(a.acceleration.z - 10) > a_threshold)
  {
    if(EmergencyDetectionFlag == false)
    {
      digitalWrite(Left_LED_Pin, HIGH);   // turn the LED on 
      digitalWrite(Right_LED_Pin, HIGH);   // turn the LED on
      EmergencyFlag = true; //Flag to start SOS
      EmergencyDetectionFlag = true; //Flag to indicate an accident (only occurs once)
    }  
  }

  //Sending the accident location through bluetooth (breaking latt and long into bytes)
  if(EmergencyFlag)
  {
    //Breaking the lattitude and longitude into 2 digits long packets (in order to send a single byte through BT)
    SerialBT.write(SOS_SIGNAL);
    EmergencyFlag = false;
    finish();
  }
  /*****************************Emergency End*********************************/

}

void finish()
{
    digitalWrite (Left_LED_Pin, HIGH);
    digitalWrite (Right_LED_Pin, HIGH);
    digitalWrite (Left_Haptic_Pin, HIGH);
    digitalWrite (Right_Haptic_Pin, HIGH);
    delay(2000);

    digitalWrite (Left_Haptic_Pin, LOW);
    digitalWrite (Right_Haptic_Pin, LOW);
    audio.connecttoFS(SD, "car_engine.wav");
    ESP.restart();

}
// function definitions:
void config_pinout()
{

  SerialBT.begin("Helmet_Bluetooth", false);
  //Define LEDs and Haptic feedback as OUTPUT
  pinMode(Left_LED_Pin, OUTPUT);
  pinMode(Right_LED_Pin, OUTPUT);
  pinMode(Left_Haptic_Pin, OUTPUT);
  pinMode(Right_Haptic_Pin, OUTPUT);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); 
  //Accelerometer  
  mpu_setup();

  //Compass
  compass_setup();
}

void mpu_setup(void) {
  while (!Serial)
    //delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      //delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(false);
}

void compass_setup() {
  //Wire.begin(21, 22);  // Initialize I2C communication on pins 21 (SDA) and 22 (SCL)
  
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  Serial.println("HMC5883 Found!");
}

bool get_new_GPS_compass_data()
{
  //  Serial.println(SerialBT.readString());

  //GPS coordinates.
  String incomingData = SerialBT.readStringUntil('\n'); // Read until newline character
  if (incomingData.length() > 0) {
      mostRecentLatt = atof(incomingData.c_str()); // Convert to float
      Serial.print("Received latitude: ");
      Serial.println(mostRecentLatt, 4); // Print the float value with 4 decimal places
  }
  incomingData = SerialBT.readStringUntil('\n'); // Read until newline character
  if (incomingData.length() > 0) {
      mostRecentLon = atof(incomingData.c_str()); // Convert to float
      Serial.print("Received longitude: ");
      Serial.println(mostRecentLon, 4); // Print the float value with 4 decimal places
  }

  //compass degress angle
  incomingData = SerialBT.readStringUntil('\n');
  if (incomingData.length() > 0) {
    //if the angle is in radians 
    //float headingDegrees = atof(incomingData.c_str()) * 180/M_PI; 
    //if the angle is in degrees
    float headingDegrees = atof(incomingData.c_str()) ;
    compass[0] = headingDegrees;
    Serial.print("compass heading degrees is: ");
    Serial.println(compass[0], 4); // Print the float value with 4 decimal places
  }

  return true;
}

void play_sound(const char* file_name) 
{
  Serial.println("playing audio...");
  if(!audio.connecttoFS(SD, file_name))
    Serial.println("FAILED AUDIO!");
  else {
    Serial.print("connected to audio file ");
    Serial.println(file_name);
    int total = audio.getFileSize();
    Serial.println("Duration of audio: " + String(total));
    Serial.println("Started audio");

    Serial.println("[APP] Free memory: " + String(esp_get_free_heap_size()) + " bytes");
  
    // while (audio.getFilePos() < total){
    for (int i=0; i < NUM_OF_ITER_AUDIO; i++){
      audio.loop();
    }
    Serial.println("Finished audio");
  }
}

void get_route() {
  ReceiveRoute: //Receiving the route via bluetooth (in case of a mistake, redo this process)
  //Initializing the route array to -1
  for(int i = 0; i < MaxRouteSize; i++)
  {
    route[0][i] = -1;
    route[1][i] = -1;
  }
  //Receiving the route via bluetooth
	
	//Continiously check for bluetooth data (route from the application) and turn on LEDs sequence 
  /*in our project we have only 2 Leds so the sequence will be different.*/
  bool first_iter = true;
  while(!SerialBT.available())
  {
    if(first_iter)
    {
      digitalWrite (Left_Haptic_Pin, HIGH);
      digitalWrite (Right_Haptic_Pin, HIGH) ;
      delay(333);
      first_iter = false;
    }
    digitalWrite (Left_LED_Pin, HIGH);
    digitalWrite (Right_LED_Pin, LOW);


    delay(333);
    digitalWrite (Left_LED_Pin, LOW);
    digitalWrite (Right_LED_Pin, HIGH);

    delay(333);
    digitalWrite (Left_Haptic_Pin, LOW);
    digitalWrite (Right_Haptic_Pin, LOW);
  }

  play_sound("connected.wav");

	//Bluetooth data is being received
  if (SerialBT.available()) 
  {
    Serial.println("Receiving RouteData");
    routeData = (SerialBT.readString()); //Needs checking for large routes
    Serial.println("received " + String(routeData.length()) + " bytes");

  }
  
  String receivedRouteLength = "";
  int index = 0;
  while(routeData[index] != '!')  //get RouteLength of received route, to check if we received the whole route
  {
    receivedRouteLength = receivedRouteLength + String(routeData[index]);
    index++;
    if(index > routeData.length()) //means that the bluetooth sending was not done successfully
    {
      goto ReceiveRoute; //return to receive the route again
    }
  }
  //Serial.println("receivedRouteLength: " + String(receivedRouteLength));



  //Construct the route array
  Serial.println(routeData); //Printing received route data for debugging
  ConstructRouteArray(routeData, route, index);
  
  //Getting route length (number of points)
  for(int i = 0; i < MaxRouteSize; i++)
  {
    if(route[0][i] == -1 || route[1][i] == -1)
    {
      routeLength = i;
      i = MaxRouteSize;
    }
  }
  Serial.println("routeLength: " + String(routeLength));

  //delay(100);

  //check if the route we received is correct
  if(routeLength != receivedRouteLength.toInt()) //means that the bluetooth sending was not done successfully
  {
    Serial.println(F("Route failed"));
    goto ReceiveRoute; //return to receive the route again
  }

  //Printing the route (can be removed after testing)
  Serial.println(F("Route: "));
  for(int i = 0; i < routeLength; i++)
  {
    Serial.print(route[1][i],7);
    Serial.print(";");
    Serial.print(route[0][i],7);
    Serial.println();
  }
  
  Serial.println();
  Serial.println(F("System Navigation"));
  
  //delay(1000);
  //Construct the Points of interests array (Points that indicate turns in the route)
  GetPointsOfInterest(POI_Turns, route, routeLength);

  //Printing points of interest (remove after testing)
  for(int i = 0; i < routeLength; i++)
  {
    if(POI_Turns[0][i] == 1)
    {
      Serial.print(POI_Turns[0][i]);
      Serial.print(";");
      Serial.print(POI_Turns[1][i]);
      Serial.print("  ");
    }
  }
  
}

void gps_error(){
//turn on LEDs sequence to tell the rider
    digitalWrite (Left_LED_Pin, HIGH);

    digitalWrite (Left_Haptic_Pin, HIGH);
    digitalWrite (Right_Haptic_Pin, HIGH) ;

    //delay(333);
    digitalWrite (Left_LED_Pin, LOW);
    digitalWrite (Right_LED_Pin, HIGH);

    digitalWrite (Left_Haptic_Pin, LOW);
    digitalWrite (Right_Haptic_Pin, LOW);

    //delay(333);
    digitalWrite (Left_LED_Pin, HIGH);
    digitalWrite (Right_LED_Pin, LOW);
     
    //delay(333);
    digitalWrite (Left_LED_Pin, LOW);
    digitalWrite (Right_LED_Pin, HIGH);

    //delay(333);
    digitalWrite (Left_LED_Pin, LOW);
    digitalWrite (Right_LED_Pin, LOW);

}

/// @brief This function should get the last compass data from phone
/// @param compass 
void getCompass_from_bluetooth(float *compass)
{
  compass[0]= 30;
  compass[1]= 30;
  compass[2]= 30;
  compass[3]= 30;
}
void getCompass_from_sensor(float *compass)
{
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  //Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  //Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  //Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
  
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  float declinationAngle = 0.08787; //Depends on the location and time of the year
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  //Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  compass[0]= headingDegrees;
  compass[1]= event.magnetic.x;
  compass[2]= event.magnetic.y;
  compass[3]= event.magnetic.z;
 
}

//wrapper function!
void getCompass(float *compass)
{
  getCompass_from_bluetooth(compass);
}


//Calculate directions (old solution) (REPALCED BY NEW SOLUTION)
int decideDirections(float compass[], float posLat, float posLon, float destLat, float destLon)
{
  int decision = 0; //-1 = Left, 0 = Straight, 1 = Right
  float heading = compass[0];
  float destY = destLat - posLat , destX = destLon - posLon;
  float northX = 0, northY = 1;
	
	//creating the heading vector by rotating (0,1) vector with phase = heading
	float headingX = cos((PI / 2)-(PI * heading / 180)); 
	float headingY = sin((PI / 2)-(PI * heading / 180));
	
	float thetaX = CalcAngleBetweenVectors(destX, destY, headingX, headingY);
	float crossProductZ = headingX * destY - headingY * destX; //Cross product of heading and destination
	
	if(abs(thetaX) <= 25)
	{
		decision = 0; //Go Straight
	}
	else if(crossProductZ > 0)
	{
		decision = -1; //Go Left
	}
	else
	{
		decision = 1; //Go Right
	}
	
	return decision;
}

//Turn on LEDs according to the direction decision, (haptic feedback and sound as well)
void ActivateNavigationLEDs(int decision, int last_decision)
{
  if(decision == -1)
  {
    digitalWrite (Left_LED_Pin, HIGH); //Go left
    digitalWrite (Right_LED_Pin, LOW);
    if(last_decision != -1){ //First time we recognize a left turn 
      turn_left();
    }
  }
  else if(decision == 0) //Go straight
  {
    digitalWrite (Left_LED_Pin, LOW);
    digitalWrite (Right_LED_Pin, LOW);
  }
  else 
  {
    digitalWrite (Left_LED_Pin, LOW);
    digitalWrite (Right_LED_Pin, HIGH); //Go right
    if(last_decision != 1){ //First time we recognize a right turn 
      if( first_time_right == false ){
        turn_right();
      }
      first_time_right = false ; 
    }
  }
}

// haptic feedback and sound 
void turn_left()
{
  digitalWrite (Left_Haptic_Pin, HIGH);
  digitalWrite (Right_Haptic_Pin, HIGH);
  delay(500); //half a second 
  digitalWrite (Left_Haptic_Pin, LOW);
  digitalWrite (Right_Haptic_Pin, LOW);

  play_sound("turn_left.wav");
}


void turn_right()
{
  digitalWrite (Left_Haptic_Pin, HIGH);
  digitalWrite (Right_Haptic_Pin, HIGH);
  delay(500); //half a second 
  digitalWrite (Left_Haptic_Pin, LOW);
  digitalWrite (Right_Haptic_Pin, LOW);

  play_sound("turn_right.wav");
}

//Constructs the Points of interests array (Points that indicate turns in the route)
static void GetPointsOfInterest(int POI_Turns[2][MaxRouteSize] , float route[2][MaxRouteSize], int routeLength)
{
  float v1_x, v1_y, v2_x, v2_y, theta, vh_x, vh_y, heading[4];
  int decision;
  for(int i = 0; i < routeLength - 2; i++)
  {
    //First vector of the triangle
    v1_x = route[0][i] - route[0][i+1];
    v1_y = route[1][i] - route[1][i+1];
    
    //Second vector of the triangle
    v2_x = route[0][i+2] - route[0][i+1];
    v2_y = route[1][i+2] - route[1][i+1];
    theta = CalcAngleBetweenVectors(v1_x, v1_y, v2_x, v2_y);
    
    if(theta < 110) //triangle indicates a turn
    {
      POI_Turns[0][i+1] = 1;
      vh_x = route[0][i+1] - route[0][i];
      vh_y = route[1][i+1] - route[1][i];
      heading[0] = CalcAngleBetweenVectors(vh_x, vh_y, 0, 1);
      decision = decideDirections(heading, route[0][i], route[1][i], route[0][i+2], route[1][i+2]);
      POI_Turns[1][i+1] = decision;
    }
  }
}

//Calculates the angle between two vectors (V1_x,V1_y) and (V2_x, V2_y) 
static float CalcAngleBetweenVectors(float v1_x, float v1_y, float v2_x, float v2_y)
{
  float v1_length = sqrt(pow(v1_x,2) + pow(v1_y,2));
  float v2_length = sqrt(pow(v2_x,2) + pow(v2_y,2));
  //Dot product
  float dp = v1_x*v2_x + v1_y*v2_y;

  //Calculate the angle between the vectors to determine if it represents a turn
  float thetaRad = acos(dp/(v1_length*v2_length));
  return (thetaRad * 360)/(2 * PI);
}

//Construct the route array by segmenting the received data string (points latitude and longitude)
static void ConstructRouteArray(String routeData, float route[2][MaxRouteSize], int index)
{
	String tempLat;
  String tempLon;
  int cnt = 0; //cnt to tell how many points there are (=cnt/2). if cnt is even, we're on the lat of the current point. else, we're on the lon of the current point.
  int i = 0;
  for(i = index+1 ; i < routeData.length() ; i++)
  {
    if(routeData[i] != ',' && (cnt % 2) == 0) //check if we're still on the lon (cnt is even)
    {
      tempLon = tempLon + routeData[i];
    }
    else if((cnt % 2) == 1) //check if we're now on the lat of the current point (cnt is odd)
    {
      tempLat = tempLat + routeData[i];
    }
    if(routeData[i] == ',') //finished lon, or the whole point.
    {
      if(cnt % 2 ==0) //finished lon
      {
        route[1][cnt/2] = tempLon.toFloat();


        tempLon = "";


      }
      else //finished lat (the whole point).
      {
        route[0][cnt/2] = tempLat.toFloat();

        tempLat = "";
      }
      cnt++;

    }
  }
}

//Display GPS information //HAVE no use for now 
void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}





















/*

#include "Arduino.h"
#include "Audio.h"
#include "SD.h"
#include "SPI.h"
#include "FS.h"
#include "Ticker.h"

// Digital I/O used
#define SD_CS          5
#define SPI_MOSI      23
#define SPI_MISO      19
#define SPI_SCK       18
#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRC       26

Audio audio;
Ticker ticker;
struct tm timeinfo;
time_t now;

uint8_t hour    = 6;
uint8_t minute  = 59;
uint8_t sec     = 45;

bool f_time     = false;
int8_t timefile = -1;
char chbuf[100];

void tckr1s(){
    sec++;
    if(sec > 59)   {sec = 0;     minute++;}
    if(minute > 59){minute = 0; hour++;}
    if(hour > 23)  {hour = 0;}
    if(minute == 59 && sec == 50) f_time = true;  // flag will be set 10s before full hour
    Serial.printf("%02d:%02d:%02d\n", hour, minute, sec);
}

void setup() {
    Serial.begin(9600);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    SD.begin(SD_CS);
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(10); // 0...21
    ticker.attach(1, tckr1s);
    Serial.printf("setup success");

}

void loop(){
    audio.loop();
    //I added
    audio.connecttoFS(SD, chbuf);



    if(f_time == true){
        f_time = false;
        timefile = 3;
        uint8_t next_hour = hour + 1;
        if(next_hour == 25) next_hour = 1;
        sprintf(chbuf, "/voice_time/%03d.mp3", next_hour);
        audio.connecttoFS(SD, chbuf);
    }
}

void audio_eof_mp3(const char *info){  //end of file
    Serial.printf("file :%s\n", info);
    if(timefile>0){
        if(timefile==1){audio.connecttoFS(SD, "/voice_time/080.mp3");     timefile--;}  // stroke
        if(timefile==2){audio.connecttoFS(SD, "/voice_time/200.mp3");     timefile--;}  // precisely
        if(timefile==3){audio.connecttoFS(SD, "/voice_time/O'clock.mp3"); timefile--;}
    }
}

*/


/*
#include <Arduino.h>
#include <I2S.h>

#define I2S_SAMPLE_RATE     (44100)
#define I2S_SAMPLE_BITS     16
#define PIN_LRC             8
#define PIN_BCLK            7
#define PIN_DOUT            25

int count = 0;

void setup() 
{
  I2S.setSckPin(PIN_BCLK);
  I2S.setFsPin(PIN_LRC);
  I2S.setDataPin(PIN_DOUT);
  if (!I2S.begin(I2S_PHILIPS_MODE, I2S_SAMPLE_RATE, I2S_SAMPLE_BITS))
  {

    Serial.println("Failed to initialize I2S!");

    while (1)
      ; // do nothing
  }
}
int16_t GenerateSineWave()
{
    double rad = 2 * M_PI * 1000 * count++ / I2S_SAMPLE_RATE;
    int16_t sineVal = 32767 * sin(rad);
    return sineVal;
}
void loop()
{
  I2S.write(GenerateSineWave());
  I2S.write(GenerateSineWave());
}

*/