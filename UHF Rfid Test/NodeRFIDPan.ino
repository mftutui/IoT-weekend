#include <SoftwareSerial.h>
#include <XBee.h>
#include <MTI_UHF_Reader.h>

#define TEST
#define DEBUG Serial
#define XBEE Serial1
#define UHFReaderSerial Serial3

unsigned int version = 3;


// Initializing the RFID module instance and serial connection
MTI_UHF_Reader rfidModule;
RFIDdata tag;

// Initializing the Xbee instance
XBee xbee = XBee();
ZBRxResponse rx = ZBRxResponse();
XBeeAddress64 addr64gw = XBeeAddress64();
XBeeAddress64 addr64in = XBeeAddress64();
XBeeAddress64 addr64bcast = XBeeAddress64(0x00000000, 0x0000FFFF);

#define LED_pin 6

void setup() {

  /* Debug console NODE RFID serial baudRate */
  DEBUG.begin(250000);

  while (!DEBUG); //Wait for the serial port to come online

  /* Setup MTI RFID Module */


  Serial1.begin(230400); //Serial do Zigbee
  while (!Serial1);
  if (Serial1.available()){
    Serial1.println(F("Serial XBee rfid ok!"));
  }
  else{
    Serial1.println(F("Serial XBee rfid not ok!"));    
  }


  if (setupUHFReader(115200) == false)
  {
    DEBUG.println(F("UHF Reader Module failed to respond. Please check wiring."));
    while (1); //Freeze!
  }


  Serial.println(F("Press a key to begin scanning for tags."));
  while (!Serial.available()); //Wait for user to send a character
  Serial.read(); //Throw away the user's character

  //nano.startReading(); //Begin scanning for tags
}

void loop() {
  if (rfidModule.check() == true) //Check to see if any new data has come in from module
  {
    byte responseType = rfidModule.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp

    if (responseType == RESPONSE_IS_KEEPALIVE)
    {
      Serial.println(F("Scanning"));
    }
    else if (responseType == RESPONSE_IS_TAGFOUND)
    {
      //If we have a full record we can pull out the fun bits
      int rssi = rfidModule.getTagRSSI(); //Get the RSSI for this tag read

      long freq = rfidModule.getTagFreq(); //Get the frequency this tag was detected at

      long timeStamp = rfidModule.getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message

      byte tagEPCBytes = rfidModule.getTagEPCBytes(); //Get the number of bytes of EPC from response

      Serial.print(F(" rssi["));
      Serial.print(rssi);
      Serial.print(F("]"));

      Serial.print(F(" freq["));
      Serial.print(freq);
      Serial.print(F("]"));

      Serial.print(F(" time["));
      Serial.print(timeStamp);
      Serial.print(F("]"));

      //Print EPC bytes, this is a subsection of bytes from the response/msg array
      Serial.print(F(" epc["));
      for (byte x = 0 ; x < tagEPCBytes ; x++)
      {
        if (rfidModule.msg[31 + x] < 0x10) Serial.print(F("0")); //Pretty print
        Serial.print(rfidModule.msg[31 + x], HEX);
        Serial.print(F(" "));
      }
      Serial.print(F("]"));

      Serial.println();
    }
    else if (responseType == ERROR_CORRUPT_RESPONSE)
    {
      Serial.println("Bad CRC");
    }
    else
    {
      //Unknown response
      Serial.print("Unknown error");
    }
  }
}

boolean setupUHFReader(long baudRate) {

  rfidModule.enableDebugging(DEBUG);

  UHFReaderSerial.begin(baudRate); //For this test, assume module is already at our desired baud rate
  while (!UHFReaderSerial); //Wait for port to open


  
  rfidModule.begin(UHFReaderSerial); //Tell the library to communicate over software serial port


  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  //UHFReaderSerial.begin(baudRate); //For this test, assume module is already" at our desired baud rate
  //while (!UHFReaderSerial); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while (UHFReaderSerial.available()) UHFReaderSerial.read();

  

  rfidModule.getVersion();
  if (rfidModule.msg[0] != ALL_GOOD) return (false); //Something is not right

 // rfidModule.setTagProtocol(); //Set protocol to GEN2
 // rfidModule.setAntennaPort(); //Set TX/RX antenna ports to 1

  //rfidModule.setRegion(REGION_NORTHAMERICA); //Set to North America

//  rfidModule.setReadPower(500); //5.00 dBm. Higher values may caues USB port to brown out
  //Max Read TX Power is 27.00 dBm and may cause temperature-limit throttling

  return (true); //We are ready to rock

}
