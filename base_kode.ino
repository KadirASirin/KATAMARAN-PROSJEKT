#include <SPI.h>
#include <LoRa.h>

// Outgoing message variable
String outMessage;

// Message counter
byte msgCount = 0;
 
// Receive message variables
String contents = "";

// Source and destination addresses
byte localAddress = 0xB1;  // Katamaran
byte destination = 0xB0;   // Base/remote pc

String incomingMessage = "";
 
void setup() {
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  delay(1000);
 
  while (!Serial);
  //Serial.println("LoRa Duplex with callback");
 
  // Setup LoRa module
  LoRa.setPins(10, 9, 2);
 
  // Start LoRa module at local frequency
  // 433E6 for Asia
  // 866E6 for Europe
  // 915E6 for North America

  if (!LoRa.begin(866E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  LoRa.onReceive(onReceive);
  LoRa.receive();
  //Serial.println("LoRa init succeeded.");
}


uint32_t timer = millis();

void loop() {
  // Check if data is available to read from serial
  if (Serial.available() > 0) {
    // Read incoming data
    incomingMessage = Serial.readStringUntil('\n');

    // Print received message
    Serial.print("Arduino received the target speed: ");
    Serial.println(incomingMessage);

    sendMessage(incomingMessage);
    delay(100);
    sendMessage(incomingMessage);// sender to ganger med kort mellomrom for å være sikker på at den mottas
    LoRa.receive();

  }
}

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
  if (packetSize == 0) return;  // if there's no packet, return
 
  // Read packet header bytes:
  int recipient = LoRa.read();        // recipient address
  byte sender = LoRa.read();          // sender address
  byte incomingMsgId = LoRa.read();   // incoming msg ID
  byte incomingLength = LoRa.read();  // incoming msg length
 
  String incoming = "";  // payload of packet
 
  while (LoRa.available()) {        // can't use readString() in callback, so
    incoming += (char)LoRa.read();  // add bytes one by one
  }
 
  if (incomingLength != incoming.length()) {  // check length for error
    //Serial.println("error: message length does not match length");
    return;  // skip rest of function
  }
 
  // If the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    //Serial.println("This message is not for me.");
    return;  // skip rest of function
  }
 
  // If message is for this device, or broadcast, print details:
  //Serial.println("Received from: 0x" + String(sender, HEX));
  //Serial.println("Sent to: 0x" + String(recipient, HEX));
  //Serial.println("Message ID: " + String(incomingMsgId));
  //Serial.println("Message length: " + String(incomingLength));
  Serial.println(incoming);
  //Serial.println("RSSI: " + String(LoRa.packetRssi()));
  //Serial.println("Snr: " + String(LoRa.packetSnr()));
  //Serial.println();
}
