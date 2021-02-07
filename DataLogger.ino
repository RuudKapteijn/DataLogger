/****************************************************************************
 * Data Logger v30
 * 28-Jan-21
 * Ruud Kapteijn
 * Available under GPL3+
 * 
 * Uses the AHRS Madgwick code from Kris Winer (ref AHRS.h)
 * Original code by Kris Winer
 * date: Nov 1, 2014
 * code: https://github.com/j-mcc1993/LSM9DS1/blob/master/LSM9DS1_BasicAHRS_Nano33.ino
 * discussion threa: https://github.com/kriswiner/LSM9DS1/issues/14
 */
#include <Arduino_LSM9DS1.h>
#include <Arduino_LPS22HB.h> //Include library to read Pressure
#include <SPI.h>
#include <SD.h>
// #include <RF24.h>

#include "Ublox_NMEA_GPS.h"
#include "AHRS.h"
#include "AHRS_gvars.h"

#define LEDR        (22u)
#define LEDG        (23u)
#define LEDB        (24u)
#define LEDPWR      (25u)

// GPS Vars
Ublox_NMEA_GPS myGPS;
boolean fix = false;
int satellites, prev_satellites;

// potentionmeter
int potmeter = 0;

// IMU Vars
// float ax, ay, az;
// float gx, gy, gz;
// float mx, my, mz;

// Baro vars
float pressure;

// SD Vars
File dataFile;
int chipSelect = 10;
int seqnr = 0;
String fileName;
int counter = 0;
int byteCounter = 0;
String dataString;
long last_write = 0;

/*
// transmitter vars
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";     //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.
long last_write = 0;
char tx_buffer[256];
String dataString;
int counter = 0;
int stringLength = 0;
const char ping[] = "ping";
*/

// statistic vars
boolean first_fix = true;
long    start_loops;
long    count_loops=0;
long    count_RMC = 0;
long    count_GGA = 0;

void setup() {                                    // switch buildin LED off
  digitalWrite(LEDPWR, HIGH); digitalWrite(LEDR, HIGH); digitalWrite(LEDG, HIGH);  digitalWrite(LEDB, HIGH);
  digitalWrite(LEDR, LOW);                        // switch buildin LED to red -> not initialized.

  Serial.begin(9600);                             // Open serial communications and wait for port to open:
  // while (!Serial);                                // wait until monitor is opened
  Serial.println("INFO: ** Telemetry V10 **");

  myGPS.init();
  Serial.println("INFO: GPS Started");
  
  if (!IMU.begin()) {
    Serial.println("ERROR: Failed to initialize IMU! halted.");
    while (true);
  }
  Serial.println("INFO: IMU Started!");
  
  if (!BARO.begin()) {        //Initialize Pressure sensor
    Serial.println("ERROR: Failed to initialize Pressure Sensor! halted.");
    while (1);
  }
  Serial.println("INFO: BARO Started!");

  if (AHRS_setup())
    Serial.println("INFO: AHRS setup ok");
  else {
    Serial.println("ERROR AHRS setup failed. Halted!");
    while(true);
  }

/*
  radio.begin();                  //Starting the Wireless communication
  radio.openWritingPipe(address); //Setting the address where we will send the data
  radio.setPALevel(RF24_PA_MIN);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.stopListening();          //This sets the module as transmitter
  Serial.println("INFO: Transmitter started");
*/

  if (!SD.begin(chipSelect)) {              // Setup SD Card
   Serial.println("ERROR: SD Card failed or not present! Halted.");
   while (1);
  }
  Serial.println("INFO: SD card initialized");
  seqnr = 1;                              // Check on existing files
  fileName = "DL" + String(seqnr) + ".CSV";
  while(SD.exists(fileName)) {
    seqnr++;
    fileName = "DL" + String(seqnr) + ".CSV";
    delay(200); // Make sure file access is done
  }
  dataFile = SD.open(fileName, FILE_WRITE);   // Create new CSV file with appropriate headers
  dataFile.println("Nr,Mills,Date,UTC,Lat,Lon,Sog,Cog,Alt,Sat,Fix,Pm,Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz,Pres,Pitch,Roll,Yaw");
  Serial.println("INFO: new data file created, ready for logging.");      // Done with SD Card Init

  digitalWrite(LEDR, HIGH);                       // switch buildin LED off
  digitalWrite(LEDB, LOW);                        // switch buildin LED to blue -> looking for satellites.
  Serial.println("Initialization completed");
}

void loop() {                                     // run continuously
  myGPS.update();
  prev_satellites = satellites;
  satellites = myGPS.getSIV().toInt();
  if (satellites != prev_satellites)
    Serial.println("SIV: " + String(satellites) + " Pitch: " + String(pitch));

  if (satellites > 2) {                          // return TRUE if fix
    if (!fix) Serial.println("INFO: Fix created!");
    fix = true;
    digitalWrite(LEDB, HIGH);                       // switch buildin LED off
    digitalWrite(LEDG, LOW);                        // switch buildin LED to green -> (potential) fix.
    if (first_fix) {
      first_fix = false;
      start_loops = millis();
      count_loops = 0;
      count_RMC = 0;
      count_GGA = 0;
    }
  } else {
    if (fix) Serial.println("INFO: Fix lost!");
    fix = false;
    digitalWrite(LEDG, HIGH);                       // switch buildin LED off
    digitalWrite(LEDB, LOW);                        // switch buildin LED to blue -> looking for satellites.
  }

  potmeter = analogRead(7);                           // read value of potential meter via ADC pin 7
  getIMUData();
  pressure = BARO.readPressure();

  AHRS_update();

  // write line to datafile
  if (dataFile && fix && millis() - last_write > 150) {
    counter += 1;
    dataString = createDataString();
    // dataString = "dit is een lange test data string met een hele boel bytes";    
    // Serial.print("INFO: write to SD card: ");
    // Serial.println(dataString);
    dataFile.println(dataString);
    byteCounter += dataString.length();
    if (byteCounter > 1024) {
      // Serial.println("INFO: flush buffer to SD card");
      dataFile.flush();
      byteCounter = 0;
    }
  }

/*
  if (fix && millis() - last_write > 250) {     // send data string
    last_write = millis();
    counter += 1;
    dataString = createDataString() + '\n';
    // Serial.print("INFO: write to SD card: ");
    for (int i = 0; i < dataString.length(); i+=32) {
      String section = dataString.substring(i, i + 32);
      section.toCharArray(tx_buffer, sizeof(tx_buffer)); 
      radio.write(&tx_buffer, section.length());           //Sending the message to receiver
    }
  }
*/
  
  count_loops++;
  if (count_loops % 1000 == 0) {
    Serial.println("loops: "+String(count_loops)+", satellites: "+myGPS.getSIV()+", pitch: "+String(pitch));
  }
  if (count_loops % 10000 ==0) {
    Serial.println("loops: "+String(count_loops)+", avg loop: "+String((millis() - start_loops)/count_loops)+" ms, avg RMC: "+String((millis() - start_loops)/count_RMC)+" ms, avg GGA: "+String((millis() - start_loops)/count_GGA));
    // radio.write(&ping, sizeof(ping));               //Sending the message to receiver
  }
}

boolean getIMUData() {
  //Accelerometer values
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }
  //Gyroscope values 
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }
  //Magnetometer values 
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
  }
}

String createDataString() {
  String ds = "";
  
  ds += String(counter) + ",";
  ds += String(millis()) + ",";
  ds += myGPS.getDAT() + ",";
  ds += myGPS.getTIM() + ",";
  ds += myGPS.getLAT() + ",";
  ds += myGPS.getLON() + ",";
  ds += myGPS.getSOG() + ",";
  ds += myGPS.getCOG() + ",";
  ds += myGPS.getALT() + ",";
  ds += myGPS.getSIV() + ",";
  ds += myGPS.getFIX() + ",";

  ds += String(potmeter) + ",";

  ds += String(ax) + ",";
  ds += String(ay) + ",";
  ds += String(az) + ",";
  ds += String(gx) + ",";
  ds += String(gy) + ",";
  ds += String(gz) + ",";
  ds += String(mx) + ",";
  ds += String(my) + ",";
  ds += String(mz) + ",";

  ds += String(pressure) + ",";

  ds += String(pitch) + ",";
  ds += String(roll) + ",";
  ds += String(yaw);

  return(ds);
}
