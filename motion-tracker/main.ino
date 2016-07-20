#include "Adafruit_GPS.h"
#include "Adafruit_LIS3DH.h"
#include "Adafruit_Sensor.h"
#include "GPS_Math.h"

#include <math.h>
#include "math.h"
#include <ctype.h>

#define knotsToMPH 1.152
#define radians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define sq(x) ((x)*(x))

#define gpsSerial Serial1

Adafruit_GPS GPS(&gpsSerial);
Adafruit_LIS3DH accel = Adafruit_LIS3DH(A2, A5, A4, A3);
FuelGauge fuel;

#define MY_NAME "TRIKE"

#define CLICKTHRESHHOLD 20

int lastSecond = 0;
bool ledState = false;

STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

unsigned long lastMotion = 0;
unsigned long lastPublish = 0;
time_t lastIdleCheckin = 0;
double lastGPSlat = 0;
double lastGPSlong = 0;

#define PUBLISH_DELAY (15 * 60 * 1000)

// if no motion for 30 minutes, sleep! (milliseconds)
#define NO_MOTION_IDLE_SLEEP_DELAY (30 * 60 * 1000)

// lets wakeup every 6 hours and check in (seconds)
#define HOW_LONG_SHOULD_WE_SLEEP (6 * 60 * 60)

// when we wakeup from deep-sleep not as a result of motion,
// how long should we wait before we publish our location?
// lets set this to less than our sleep time, so we always idle check in.
// (seconds)
#define MAX_IDLE_CHECKIN_DELAY (HOW_LONG_SHOULD_WE_SLEEP - 60)

void setup()
{
  lastMotion = 0;
  lastPublish = 0;
  initAccel();

  // electron asset tracker shield needs this to enable the power to the gps module.
  pinMode(D6, OUTPUT);
  digitalWrite(D6, LOW);

  // for blinking.
  pinMode(D7, OUTPUT);
  digitalWrite(D7, LOW);

  gpsSerial.begin(9600);
  GPS.begin(9600);
  Serial.begin(9600);

  Particle.function("TrikeLoc", publishGPS);

  //# request a HOT RESTART, in case we were in standby mode before.
  GPS.sendCommand("$PMTK101*32");
  delay(250);

  // request everything!
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  delay(250);

  // turn off antenna updates
  GPS.sendCommand(PGCMD_NOANTENNA);
  delay(250);
}

void loop()
{
  if (lastIdleCheckin == 0)
  {
    lastIdleCheckin = Time.now();
  }

  unsigned long now = millis();

  if (lastMotion > now)
  {
    lastMotion = now;
  }
  if (lastPublish > now)
  {
    lastPublish = now;
  }

  checkGPS();

  // we'll be awaken by motion, lets keep listening for more motion.
  // if we get two in a row, then we'll connect to the internet and start reporting in.
  bool hasMotion = digitalRead(WKP);
  digitalWrite(D7, (hasMotion) ? HIGH : LOW);
  if (hasMotion)
  {
    Serial.println("BUMP!");
    lastMotion = now;
  }

  // use the real-time-clock here, instead of millis.
  if ((Time.now() - lastIdleCheckin) >= MAX_IDLE_CHECKIN_DELAY)
  {
    // it's been too long!  Lets say hey!
    Serial.println("IDLE TIMER CHECK-IN");
    Particle.publish(MY_NAME + String("_status"), "IDLE TIMER CHECK-IN", PRIVATE, NO_ACK);
    lastIdleCheckin = Time.now();
  }

  // have we published recently?
  if (((millis() - lastPublish) > PUBLISH_DELAY) || (lastPublish == 0))
  {
    lastPublish = millis();
    if(locationChange)
    {
      publishGPS("LOOP");
    }
  }

  // use "now" instead of millis...  If it takes us a REALLY long time to connect, we don't want to accidentally idle out.
  if ((now - lastMotion) > NO_MOTION_IDLE_SLEEP_DELAY)
  {
    Particle.publish(MY_NAME + String("_status"), "NO MOTION", PRIVATE, NO_ACK);
    //Serial.println("NO MOTION");
    lastPublish = 0;
    lastMotion = 0;
  }

  delay(10);
}

void checkGPS()
{
    // process and dump everything from the module through the library.
  while (gpsSerial.available())
  {
    //If GPS.read is not called, the library never reads the GPS data.
    GPS.read();
    if (GPS.newNMEAreceived())
    {
      GPS.parse(GPS.lastNMEA());
    }
  }
}

void initAccel()
{
  accel.begin(LIS3DH_DEFAULT_ADDRESS);

  // Default to 5kHz low-power sampling
  accel.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ);

  // Default to 4 gravities range
  accel.setRange(LIS3DH_RANGE_4_G);

  //uint8_t c, uint8_t clickthresh, uint8_t timelimit, uint8_t timelatency, uint8_t timewindow
  accel.setClick(1, CLICKTHRESHHOLD);//, 0, 100, 50);
}

int locationChange() //how to we make this prevent publishing location if the location has not changed appreciably?
{
  if(distanceBetween(lastGPSlat, lastGPSlong, GPS.latitude, GPS.longitude) > 100)//location has changed
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
double distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);

  return delta * 6372795;
}

int publishGPS(String caller)
{
  int speed = GPS.speed * knotsToMPH;
  int batt = fuel.getSoC();

  if(caller == "LOOP") //publish to Google Sheets for the map.
  {
    String gps_push = String(convertDegMinToDecDeg(GPS.latitude));
    gps_push += ", -";
    gps_push += String(convertDegMinToDecDeg(GPS.longitude));
    gps_push += "||| ";
    gps_push += String(speed);
    gps_push += "mph. ||| Battery:";
    gps_push += String(batt);
    gps_push += "%";

    Particle.publish(MY_NAME + String("_loc"), gps_push, PRIVATE, NO_ACK);
  }
  else //send a text message with Google Maps location.
  {
    String gps_call = "http://maps.google.com/maps?z=12&t=m&q=loc:";
    gps_call += String(convertDegMinToDecDeg(GPS.latitude));
    gps_call += "+-";
    gps_call += String(convertDegMinToDecDeg(GPS.longitude));
    Serial.println(gps_call);
    Particle.publish(MY_NAME + String("_call"), gps_call, 60);
  }

  lastGPSlat = GPS.latitude;
  lastGPSlong = GPS.longitude;
  return 0;
}
