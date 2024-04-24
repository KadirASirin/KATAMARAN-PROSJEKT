#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

/* KOBLINGER ARDUINO MEGA

  BNO055 (KOBLING ARDUINO MEGA) 
    SCL til 21
    SDA til 20
    Vin til 3.3-5V DC
    GND til common ground
  
  GPS BREAKOUTV3 (KOBLING ARDUINO MEGA) 
    TX til RX1 19
    RX til TX1 18
    Vin til 3.3-5V DC
    GND til common ground

  RFM95W LoRa (KOBLING ARDUINO MEGA)
    G0 til D2 (interrupt pin)
    RST til D9    
    MISO til D50 (SPI)
    MOSI til D51 (SPI)
    SCK til D52 (SPI)
    CS til D53 (SPI)
    Vin til 3.3-5V DC
    GND til common ground 
*/

//IMU (BNO055)

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//LoRa

// Outgoing message variable
String outMessage;

//Message to UNO
String targetMessage;

// Message counter
byte msgCount = 0;
 
String target = "0,0"; // Global variable to store incoming message

// Source and destination addresses
byte localAddress = 0xB0;  // Katamaran
byte destination = 0xB1;   // Base/remote pc

double x = -1000000, y = -1000000, z = -1000000; 

//GPS

Adafruit_GPS GPS(&Serial1);
HardwareSerial mySerial = Serial1;

#define GPSECHO false


void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600);
  delay(2000);

//GPS----------------------------------------------
  //Serial.println("");
  //Serial.println("Adafruit GPS library basic parsing test!"); Serial.println("");
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  delay(2000);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);

//IMU----------------------------------------------

  while (!Serial) delay(1000);
  //Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

//LoRa--------------------------------------------

  while (!Serial); delay(1000);
  //Serial.println("LoRa Duplex with callback"); Serial.println("");
 
  // Setup LoRa module
  LoRa.setPins(53, 9, 2);
 
  // Start LoRa module at local frequency
  // 433E6 for Asia
  // 866E6 for Europe
  // 915E6 for North America

  if (!LoRa.begin(866E6)) {             // initialize ratio at 866 MHz
    //Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  delay(1000);
  LoRa.onReceive(onReceive);
  LoRa.receive();
  //Serial.println("LoRa init succeeded."); Serial.println("");

  delay(2000);
}


uint32_t timer = millis();

void loop() {
  char c = GPS.read();
  if ((c) && (GPSECHO))
    Serial.write(c);

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (millis() - timer > 900) {
    timer = millis();
    if (GPS.fix) {
      targetMessage = target;
      targetMessage += ",";
      targetMessage += String(GPS.speed); // speed in knots
      targetMessage += ",";

      outMessage = String(GPS.latitude, 5); // latitude (DDMM.MMMM)
      outMessage += ",";
      outMessage += String(GPS.longitude, 5); // longitude (DDDMM.MMMM)
      outMessage += ",";
      outMessage += String(GPS.speed); // speed in knots
      outMessage += ",";
      outMessage += String(GPS.angle); // Course in degrees from true north
      
      

      // Add IMU data
      sensors_event_t event;
      bno.getEvent(&event);
      targetMessage += String(event.orientation.x);
      outMessage += ",";
      outMessage += String(event.orientation.x);
      outMessage += ",";
      outMessage += String(event.orientation.y);
      outMessage += ",";
      outMessage += String(event.orientation.z);
    } else {
      // If there is no GPS fix, create a message indicating no fix
      outMessage = "GPS: no fix";
    }
    delay(600);
    Serial.println(targetMessage);
    Serial2.println(targetMessage);
    Serial.println(outMessage);
    sendMessage(outMessage); // Send the message using LoRa
    LoRa.receive();
  }
}


//IMU----------------------------------------------
void printEvent(sensors_event_t* event) {
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
}

//LoRa----------------------------------------------

// Send LoRa Packet
void sendMessage(String outgoing) {
  LoRa.beginPacket();             // start packet
  LoRa.write(destination);        // add destination address
  LoRa.write(localAddress);       // add sender address
  LoRa.write(msgCount);           // add message ID
  LoRa.write(outgoing.length());  // add payload length
  LoRa.print(outgoing);           // add payload
  LoRa.endPacket();               // finish packet and send it
  msgCount++;                     // increment message ID
}
 
// Receive Callback Function
void onReceive(int packetSize) {
  if (packetSize == 0) return;

  int recipient = LoRa.read();
  byte sender = LoRa.read();
  byte incomingMsgId = LoRa.read();
  byte incomingLength = LoRa.read();
  
  target = ""; // Clear the previous content of the target variable

  while (LoRa.available()) {
    target += (char)LoRa.read();
  }

  if (incomingLength != target.length()) {
    Serial.println("error: message length does not match length");
    return;
  }

  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;
  }

  Serial.println("Received LoRa message: " + target);
}
