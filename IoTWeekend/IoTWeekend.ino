#include <TheThingsNetwork.h>
#include "HardwareSerial.h"
#include <SoftwareSerial.h>

#define PRESENCE_SENSOR  A0
#define WAIT_TIME        2000
#define LORA_SERIAL      Serial1
#define DEBUG_SERIAL     Serial
#define FREQ_PLAN        TTN_FP_US915   // Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define VALIDATION_FIELD_SIZE  3
#define VALIDATION_ID          "B4A5A1"
#define TAG_ID_SIZE   12
#define PAYLOAD_SIZE  14
#define DISTANCE_TRIGGER 30 // cm
uint8_t spread_factor = 7;
const char *appEui = "70B3D57ED0013955";
const char *appKey = "FB40EE13D7C7F0C736FE137E3A025916";
char tag_rfid[TAG_ID_SIZE];
char payload[PAYLOAD_SIZE];

TheThingsNetwork ttn(LORA_SERIAL, DEBUG_SERIAL, FREQ_PLAN, spread_factor);
SoftwareSerial rfid(7, 6); // Rx, Tx

void read_tag_rfid() {
    int i = 0;
    while(rfid.available()) {
        tag_rfid[i] = (char)rfid.read();
        i++;
    }
}

byte validation_tag() {
    char val_field[VALIDATION_FIELD_SIZE];
    memcpy(val_field, tag_rfid, VALIDATION_FIELD_SIZE);
//    for(int i = 0; i < VALIDATION_FIELD_SIZE; i++){
//        val_field[i] = tag_rfid[i];
//    }

    if(strcmp(val_field, VALIDATION_ID) == 0) {
      return 1;
    } else {
      return 0;
    }
}

void create_payload(byte sensor, byte validation) {
    memcpy(payload, tag_rfid, TAG_ID_SIZE);
    payload[PAYLOAD_SIZE-2] = validation;
    payload[PAYLOAD_SIZE-1] = sensor;


  
}

byte read_sensor() {
    int sensor_value = analogRead(PRESENCE_SENSOR);
    // TODO conversão 
    sensor_value = (sensor_value*0.2027)-82;
    
    DEBUG_SERIAL.print("value: ");
    DEBUG_SERIAL.print(sensor_value);

    if(sensor_value <= DISTANCE_TRIGGER) {
      return 1;
    } else {
      return 0;
    }
  
}


void mensagem(const uint8_t *payload, size_t size, port_t port) {
  if (payload[0] == 1) {
//    digitalWrite(ledVermelho, HIGH);
//    digitalWrite(ledVerde, LOW);
//    delay(3000);
//    digitalWrite(ledVermelho, LOW);
//    digitalWrite(ledVerde, HIGH);

  } else {
//    digitalWrite(ledVermelho, LOW);
//    digitalWrite(ledVerde, HIGH);
//    delay(300);
//    digitalWrite(ledVermelho, HIGH);
//    digitalWrite(ledVerde, LOW);
  }
}



void setup()
{

  LORA_SERIAL.begin(57600);
  DEBUG_SERIAL.begin(9600);

  // Wait a maximum of 10s for Serial Monitor
  while (!DEBUG_SERIAL && millis() < 10000);

  DEBUG_SERIAL.println("-- STATUS");
  ttn.showStatus();

  DEBUG_SERIAL.println("-- JOIN");
  ttn.join(appEui, appKey);

  pinMode(PRESENCE_SENSOR, INPUT); //Sensor

  ttn.onMessage(mensagem);

}


void loop() {
    //read_tag_rfid();
    memcpy(tag_rfid, "B4A5A11234567", TAG_ID_SIZE);
    byte validation  = validation_tag();
    byte sensor_read = read_sensor();
    create_payload(sensor_read, validation);

    ttn.sendBytes(payload, sizeof(payload));

    delay(WAIT_TIME);
}
