#include <TheThingsNetwork.h>
#include <Thermistor.h>

// Set your AppEUI and AppKey
const char *appEui = "70B3D57ED001217A";
const char *appKey = "7BD24D26BAE357284DB0667F672FF4DC";

#define loraSerial Serial1
#define debugSerial Serial

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_US915

uint8_t sf = 8;

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan, sf);

#define ledVerde 12
#define ledVermelho 13

#define PinoSensor A0

void setup()
{

  loraSerial.begin(57600);
  debugSerial.begin(9600);

  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000);

  debugSerial.println("-- STATUS");
  ttn.showStatus();

  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);

  pinMode(ledVerde, OUTPUT); //Pino do led será saída
  pinMode(ledVermelho, OUTPUT); //Pino do led será saída
  pinMode(PinoSensor, INPUT); //Sensor

  ttn.onMessage(mensagem);

}

void loop()
{
  debugSerial.println("-- LOOP");

  int ValorSensor = analogRead(PinoSensor);
  int ValorCel = (ValorSensor*0.2027)-82;

  debugSerial.print("Temperatura: ");
  debugSerial.print(ValorCel);
  debugSerial.println("ºC");

  byte payload[2];
  payload[0] = highByte(ValorCel);
  payload[1] = lowByte(ValorCel);
//
  ttn.sendBytes(payload, sizeof(payload));

  delay(10000);
}

void mensagem(const uint8_t *payload, size_t size, port_t port)
{
  if (payload[0] == 1)
  {
    digitalWrite(ledVermelho, HIGH);
    digitalWrite(ledVerde, LOW);
    delay(3000);
    digitalWrite(ledVermelho, LOW);
    digitalWrite(ledVerde, HIGH);

  }
  else
  {
    digitalWrite(ledVermelho, LOW);
    digitalWrite(ledVerde, HIGH);
    delay(300);
    digitalWrite(ledVermelho, HIGH);
    digitalWrite(ledVerde, LOW);
  }
}
