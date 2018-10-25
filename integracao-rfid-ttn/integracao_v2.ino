/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the (early prototype version of) The Things Network.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * Change DEVADDR to a unique address!
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>



/********************************************************************************
 * RFID INIT
 */ 

#define SEND_TAG_INTERVAL  30 // em segundos 
enum RFID_Commands
{
  cmd_RFID_RadioSetDeviceID = 0x00,
  cmd_RFID_RadioGetDeviceID,
  cmd_RFID_RadioSetOperationMode,
  cmd_RFID_RadioGetOperationMode,
  cmd_RFID_RadioSetCurrentLinkProfile,
  cmd_RFID_RadioGetCurrentLinkProfile,

  cmd_RFID_RadioWriteRegister,
 
  cmd_RFID_RadioReadRegister,
  cmd_RFID_RadioWriteBankedRegister,
  cmd_RFID_RadioReadBankedRegister,
  cmd_RFID_RadioReadRegisterInfo,

  cmd_RFID_AntennaPortSetState = 0x10,
  cmd_RFID_AntennaPortGetState,
  cmd_RFID_AntennaPortSetConfiguration,
  cmd_RFID_AntennaPortGetConfiguration,
  cmd_RFID_AntennaPortSetSenseThreshold,
  cmd_RFID_AntennaPortGetSenseThreshold,

  cmd_RFID_18K6CSetActiveSelectCriteria = 0x20,
  cmd_RFID_18K6CGetActiveSelectCriteria,
  cmd_RFID_18K6CSetSelectCriteria,
  cmd_RFID_18K6CGetSelectCriteria,
  cmd_RFID_18K6CSetSelectMaskData,
  cmd_RFID_18K6CGetSelectMaskData,
  cmd_RFID_18K6CSetPostMatchCriteria,
  cmd_RFID_18K6CGetPostMatchCriteria,
  cmd_RFID_18K6CSetPostMatchMaskData,
  cmd_RFID_18K6CGetPostMatchMaskData,

  cmd_RFID_18K6CSetQueryTagGroup = 0x30,
  cmd_RFID_18K6CGetQueryTagGroup,
  cmd_RFID_18K6CSetCurrentSingulationAlgorithm,
  cmd_RFID_18K6CGetCurrentSingulationAlgorithm,
  cmd_RFID_18K6CSetSingulationAlgorithmParameters,
  cmd_RFID_18K6CGetSingulationAlgorithmParameters,
  cmd_RFID_18K6CSetTagAccessPassword,
  cmd_RFID_18K6CGetTagAccessPassword,
  cmd_RFID_18K6CSetTagWriteDataBuffer,
  cmd_RFID_18K6CGetTagWriteDataBuffer,
  cmd_RFID_18K6CGetGuardBuffTagNum,
  cmd_RFID_18K6CGetGuardBuffTagInfo,

  cmd_RFID_18K6CTagInventory = 0x40,
  cmd_RFID_18K6CTagRead,
  cmd_RFID_18K6CTagWrite,
  cmd_RFID_18K6CTagKill,
  cmd_RFID_18K6CTagLock,
  cmd_RFID_18K6CTagMultipleWrite,
  cmd_RFID_18K6CTagBlockWrite,
  cmd_RFID_18K6CTagBlockErase,

  cmd_RFID_ControlCancel = 0x50,

  cmd_RFID_ControlPause = 0x52,
  cmd_RFID_ControlResume,
  cmd_RFID_ControlSoftReset,
  cmd_RFID_ControlResetToBootloader,
  cmd_RFID_ControlSetPowerState,
  cmd_RFID_ControlGetPowerState,

  cmd_RFID_TestSetAntenaPortConfiguration = 0x80,
  cmd_RFID_TestGetAntenaPortConfiguration,

  cmd_RFID_EngGEtTemperature = 0x91,

  cmd_RFID_EngGetRFPower = 0x93,

  cmd_RFID_BAUD_RATE = 0xA4,

};

enum EEPROM_Parameters
{
  EEPROM_RFID_stdpwr = 0x00,
  EEPROM_RFID_inventory_cycles,
  EEPROM_RFID_inventory_antennas,
  EEPROM_RFID_tag_delay

};
struct ring_s
{
  unsigned int capacity;
  unsigned int position = 0;
  unsigned int size = 0;
  uint8_t *data;
};
enum TAG_STATUS
{
  OFF = 0x00,
  READED,
  STILLON,
  JUSTLEFT
};
struct tag_s
{
  uint8_t valid = 0;
  uint8_t size = 0;
  uint8_t data[64];
  uint16_t rssi;
  uint8_t antena = 0xff;
  bool active = false;
  uint8_t readed = OFF;
  long seenAt;
  long storedAt;
};
#define LAST_TAG_LIST_SIZE 25
#define INTERNAL_TAG_BUFFER 25
struct tag_ring_s
{
  unsigned int capacity = LAST_TAG_LIST_SIZE;
  unsigned int position = 0;
  unsigned int size = 0;
  tag_s tagList[LAST_TAG_LIST_SIZE];
};

struct tag_s lastTags[LAST_TAG_LIST_SIZE];
struct tag_s internalTags[INTERNAL_TAG_BUFFER];

#define DFT_TIMEOUT 300
#define CMD_PACKET_SIZE 16
#define rxBuffSize 512

int version = 201;

struct tag_ring_s TAGring;
uint8_t RXdataBuff[rxBuffSize];
struct ring_s RXring;
uint8_t cmdBuff[16];
uint8_t devId;
uint16_t retPWR;
uint16_t paPWR;
int tagInterval = 10;


int sendTagInterval = 1000 * SEND_TAG_INTERVAL;
unsigned long lastSendTag; 

uint32_t init_time = 0;

uint8_t Reg = 0;

unsigned char ringSize(struct ring_s x);
unsigned char ringPut(struct ring_s *x, uint8_t ch);
unsigned char ringGet(struct ring_s *x);

//states
void justPass();
void getDeviceId();
void getDeviceIdrecv();
void setOperationMode();
void setOperationModerecv();
void setAntenaPort();
void setAntenaPortrecv();
void setSingulationAlgo();
void setSingulationAlgorecv();
void setSingulationParameters();
void setSingulationParametersrecv();
void startInventory();
void startInventoryrecv();
void trataIninicio();
void getDeviceBaud();


//state pointer
void (*statefunc)() = getDeviceId;


uint8_t data[70];

bool rising = false;
bool showing = false;
uint8_t tag_index = 0;

enum OPERATIONS
{
  fnc_Roll = 0x00,
  fnc_Antenna_test,
  fnc_PowerRoll_test,
};

enum CONFIGURATIONS
{
  RU_861 = 0x00,
  M_03,
};

struct antenna_struct
{
  unsigned int power = 100;
  unsigned int cycles = 2;
  unsigned int interval = 1000;
  bool active = false;
} antenna_config[4];

// MODO RFID Debug
// Quanto TRUE o firmware entra em MODO debug do RFID via serial MONITOR
bool debugRFIDMode = true;

uint8_t std_power = 280;
uint8_t std_inventory_cycle = 0x15;
uint8_t std_inventory_antennas = 2;
uint8_t std_tag_interval = 2000;
//uint8_t std_config = M_03; // Para o Spitfire essa linha deve ser descomentada
uint8_t std_config = RU_861;  // Para o RU-861 essa linha deve ser descomentada
uint8_t newCommand = fnc_Roll;

uint8_t antenna = 0;
uint16_t power = 300;

char string[50];

uint16_t u16TopicSubID[10];
uint16_t u16TopicPubID[10];
uint8_t u8Counter;
#define DEBUG Serial // RX, TX




// Configura os parametros padrões do reader RFID para usar em modo DEBUG
void setReaderParametersDebugMode()
{
    unsigned int antennasQtd = 1;

    if (std_config == RU_861)
      antennasQtd = 2;
    else
      antennasQtd = 4;

    for (int i = 0; i < antennasQtd; i++)
    {
      antenna_config[i].power = std_power;
      antenna_config[i].cycles = std_inventory_cycle;
      antenna_config[i].interval = std_tag_interval;

      if (debugRFIDMode){
        Serial.print("Config Antenna: ");
        Serial.println(i);
        Serial.print("Power: ");
        Serial.print(antenna_config[i].power);
        Serial.print(" | Cycles: ");
        Serial.print(antenna_config[i].cycles);
        Serial.print(" | Tag interval: ");
        Serial.println(antenna_config[i].interval);
      }
    }
}

long previousMillis = 0; // Variável de controle do tempo   // Tempo em ms do intervalo a ser executado
long tags = 0;
long wait = 0;
int fsmErros = 0;
unsigned char lendo = 0;
unsigned long currentMillis = millis();
unsigned long last = millis();




/*
 * RFID END
 * *****************************************************************
 */ 


//Serial.println(LMIC.freq);

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
static const PROGMEM u1_t NWKSKEY[16] = { 0x12, 0x5D, 0xB4, 0x59, 0x0E, 0x01, 0x0A, 0x73, 0x72, 0xBA, 0xB7, 0xAD, 0xD6, 0x64, 0x7B, 0x8C };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
static const u1_t PROGMEM APPSKEY[16] = { 0xCE, 0xBD, 0x65, 0x42, 0x5D, 0x85, 0x01, 0x46, 0xA3, 0x74, 0xD0, 0xA8, 0xB4, 0xED, 0xC1, 0x87 } ;

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
static const u4_t DEVADDR = 0x2603191C ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "1234567890123456789012345678";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
        Serial.println(LMIC.freq);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial.println(F("Starting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif


    RXring.data = RXdataBuff;
    RXring.capacity = rxBuffSize;
  
  
    if (debugRFIDMode)
    {
      DEBUG.println("######  RFID DEBUB MODE STARTED ########");
    } 

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    //LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    //LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

  for (int channel=0; channel<8; ++channel) {
    LMIC_disableChannel(channel);
  }
  for (int channel=9; channel<72; ++channel) {
     LMIC_disableChannel(channel);
  }

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
    
}

void loop() {
    os_runloop_once();
}





////void loop()
//{
//  //buffer de recepcao da serial do módulo RFID
//  while (Serial1.available())
//  {
//    //Serial.println("test");
//    ringPut(&RXring, Serial1.read());
//    
//  }
//  //delay(1000);
// //Serial.println("test1");
//  //define a funcão a ser executada na maquina de estados do ciclo de vida
//  (*statefunc)();
//
//  //Tempo atual em ms
//  currentMillis = millis();
//
////  ZBTxRequest zbTx;
//
//  if (currentMillis - previousMillis > tagInterval)
//  {
//    previousMillis = currentMillis; // Salva o tempo atual
//    digitalWrite(13, !digitalRead(13));
//    decreaseTagTimeLIST();
//  }
//  decreaseTagTime();
//
//  //manda tag pra fora
//  if (ringSize(RXring) < (rxBuffSize / 10))
//  {
//    if (tagRingSize(TAGring))
//    {
//     // ZBTxRequest zbTx;
//      trataTAG(tagRingGet(&TAGring));
//    }
//  }
//  PrintTAGS();
//}


//########################################################################################
//########################################################################################
//##########################    RU-861 Finite State Machine  #############################
//########################################################################################
//########################################################################################


void decreaseTagTimeLIST()
{

    for (int i = 0; i < INTERNAL_TAG_BUFFER; i++)
    {

        if (millis() - internalTags[i].storedAt > antenna_config[internalTags[i].antena].interval)
        {
            if (internalTags[i].readed != OFF)
            {
                internalTags[i].active = true;
                internalTags[i].readed = JUSTLEFT;
            }
        }
    }
}


void decreaseTagTime()
{
  for (int i = 0; i < LAST_TAG_LIST_SIZE; i++)
  {
    if (lastTags[i].valid != 0)
    {
      if (newCommand == fnc_PowerRoll_test)
      {
        if (millis() - lastTags[i].seenAt > 100)
        {
          lastTags[i].valid = 0;
        }
      }
      else
      {
        if (millis() - lastTags[i].seenAt > 1)
        {

          lastTags[i].valid = 0;
        }
      }
    }
  }
}



String  getEpcByTag(struct tag_s t){
    Serial.println("getEpcByTag start");
    String teste;
  
    for (int in = 2; in < t.size - 2; in++)
    {
      if (t.data[in] <= 0x0f)
      {
        teste += String("0");
      }
      teste += String(t.data[in], HEX);
    }
    Serial.print("EPC: ");
    Serial.println(teste);
    char buf2[30];
    teste.toCharArray(buf2, sizeof(buf2));
    //çSerial.println("getEpcByTag end");
    return buf2;
}



void trataTAG(struct tag_s t)
{
  int i;
  for (i = 0; i < LAST_TAG_LIST_SIZE; i++)
  {
    if (lastTags[i].valid != 0)
    {
      if (t.size == lastTags[i].size)
      {
        //Serial.println("bateu tamanho");
        if (memcmp(t.data, &lastTags[i], t.size))
        {
          //Serial.println("Bateu EPC");
          return; //tag ainda estava na lista
        }
      }
    }
  }

  //tag não esta na lista
  for (i = 0; i < LAST_TAG_LIST_SIZE; i++)
  {
    if (lastTags[i].valid == 0)
    { //insere na lista
      lastTags[i] = t;
      lastTags[i].valid = 1;
      lastTags[i].seenAt = millis();
      lastTags[i].active = true;

      break;
    }
  }

  tags = 0;
  int index_tag_on_list = 0;
  bool sameTAG = false;
  bool sameantenna = false;
  for (int j = 0; j < INTERNAL_TAG_BUFFER; j++)
  {
    if (memcmp(t.data, internalTags[j].data, t.size) == 0)
    {
      sameTAG = true;
      if (t.antena == internalTags[j].antena)
      {
        sameantenna = true;
        internalTags[j].storedAt = millis();
      }
    }
  }

  if (sameTAG == false) //nova tag encontrada, guarda na lista
  {
    internalTags[tag_index] = t;
    internalTags[tag_index].active = true;
    internalTags[tag_index].readed = READED;
    internalTags[tag_index].storedAt = millis();
    tag_index++;
    if (tag_index > INTERNAL_TAG_BUFFER)
      tag_index = 0;
    // Serial.print("Tag nova, index:");
    //Serial.println(tag_index);
  }
  else if (sameTAG == true && sameantenna == true) //Tag já na lista, renova o tempo
  {
    internalTags[index_tag_on_list].storedAt = millis();
    // FABIANO

    if(millis()- lastSendTag > 1000*SEND_TAG_INTERVAL){
      Serial.println("PERIODIC");
      lastSendTag = millis();
    }

    //Serial.print("Tag ON:");
    // Serial.print(index_tag_on_list);
    //Serial.println(" - Time Renovado ");
  }
  else if (sameTAG == true && sameantenna == false) //Tag já na lista, renova o tempo
  {
    internalTags[tag_index] = t;
    internalTags[tag_index].active = true;
    internalTags[tag_index].readed = READED;
    internalTags[tag_index].storedAt = millis();
    tag_index += 2;
    //tag_index--;
    //Serial.print("EPC igual: ");
    //Serial.print(tag_index);
    //Serial.println(" Different antennas ");
  }
}

void PrintTAGS()
{
  // internalTags[tag_index] = t;
  for (int k = 0; k < INTERNAL_TAG_BUFFER; k++)
  {
    if (internalTags[k].active != true)
    { 
      //String epc_active = getEpcByTag(internalTags[k]);
      //Serial.println(epc_active);
        continue;
    }

    if (internalTags[k].readed == READED)
    {
      tags = 0;
      String epc = getEpcByTag(internalTags[k]);
      //epc = epc + "in";
      lastSendTag = millis();
      Serial.println("IN TAG");
      Serial.println(epc);
      
      internalTags[k].active = false;
    }
    else if (internalTags[k].readed == JUSTLEFT)
    {
      String epc = getEpcByTag(internalTags[k]);
      Serial.println("OUT TAG");
      Serial.println(epc);
      internalTags[k].active = false;
      internalTags[k].readed = OFF;
      internalTags[k].antena = 0xff;

      tag_index--;
      //Serial.print("Tag saiu, index:");
      //Serial.println(tag_index);
      memset(internalTags[k].data, 0x00, internalTags[k].size);
    }
  }
}

//########################################################################################
//########################################################################################
//##########################    RU-861 / M-03 Finite State Machine  ######################
//########################################################################################
//########################################################################################

void countError()
{
  fsmErros++;
  //Serial.print("E");
  if (fsmErros > 60000)
  {
    Serial.println("# fsmErros OVERFLOW! ");
    statefunc = rfidModuleERROR;
  }
}
void simpleGet(uint8_t id, uint8_t cmd)
{
  formatCMD(id, cmd, cmdBuff);
  sendCMD(cmdBuff);
}

void simpleSet(uint8_t id, uint8_t cmd, uint16_t newVal)
{
  formatCMD(id, cmd, cmdBuff);
  cmdBuff[6] = newVal;
  sendCMD(cmdBuff);
}

void getDeviceBaud()
{
  uint8_t buff[7];

  for (int i = 0; i < 7; i++)
  {
    buff[i] = ringGet(&RXring);
    Serial.println(buff[i], HEX);
  }
}

void getDeviceId()
{
  fsmErros = 0;
  simpleGet(0xff, cmd_RFID_RadioGetDeviceID);
  Serial.println("Getting Device ID ...");
   
  statefunc = getDeviceIdrecv;
}

void getDeviceIdrecv()
{
  Serial.println("getDeviceIdrecv");
  uint8_t buff[16];
  
  if (ringSize(RXring) >= 16)
  {
    for (int i = 0; i < 16; i++)
    {
      buff[i] = ringGet(&RXring);
    }
    if (checkCRC(CMD_PACKET_SIZE, buff) != 0)
    {
      if(debugRFIDMode)
      {
        //DEBUG.println("Error -  CRC Error to get RFID Reader device ID");
        Serial.println("Error -  CRC Error to get RFID Reader device ID");
      }
      statefunc = rfidModuleERROR;
      return;
    }
    if ((buff[6] != 0) || (buff[5] != cmd_RFID_RadioGetDeviceID))
    {
      if(debugRFIDMode)
      {
        //DEBUG.println("Error - Error to get RFID Reader ");
        Serial.println("Error - Error to get RFID Reader ");
      }
      statefunc = rfidModuleERROR;
      return;
    }
    devId = buff[7];
    Serial.print("Device ID = ");
    Serial.println(devId);
    statefunc = setOperationMode;
  }
  else
  {
    //Serial.println("try again 1");
    countError();
  }
}

void setOperationMode()
{
  fsmErros = 0;
  simpleSet(devId, cmd_RFID_RadioSetOperationMode, 1);
  //Serial.println("Setando Operation mode");
  statefunc = setOperationModerecv;
}

void setOperationModerecv()
{
  uint8_t buff[16];
  if (ringSize(RXring) >= 16)
  {
    for (int i = 0; i < 16; i++)
    {
      buff[i] = ringGet(&RXring);
    }
    if (checkCRC(CMD_PACKET_SIZE, buff) != 0)
    {
      //Serial.println("Deu Ruim CRC");
      statefunc = rfidModuleERROR;
      return;
    }
    if ((buff[6] != 0) || (buff[5] != cmd_RFID_RadioSetOperationMode))
    {
      //Serial.println("Deu Ruim");
      statefunc = rfidModuleERROR;
      return;
    }
    //Serial.println("operation mode ok");
    statefunc = setAntenaPort;
  }
  else
  {
    //Serial.println("try again 2");
    countError();
  }
}

void setAntenaPort()
{
  fsmErros = 0;
  uint8_t antennaPort = 0;
  uint16_t powerLevel = 0;
  uint16_t dwellTime = 0;
  uint16_t nCycles = 0x10;
  uint8_t phyPort = 0;
  ///Serial.println("setting port...");
  if (newCommand == fnc_Roll)
  {
    power = antenna_config[antenna].power;
    nCycles = antenna_config[antenna].cycles;
  }
  if (newCommand == fnc_PowerRoll_test)
  {
    nCycles = 0x50;

    if (power >= 0)
    {
      power += 10;

      Serial.print("Antenna: ");
      Serial.print(antenna);
      Serial.print(" - Power: ");
      Serial.println(power);
    }
    if (power >= 310)
    {
      if (antenna <= 3)
      {

        DEBUG.println("Mudando a antenna...");
        switch (antenna)
        {
        case 0:
          DEBUG.println("antena 1...");
          power = std_power;
          antenna = 1;
          break;
        case 1:
          DEBUG.println("antena 2...");
          power = std_power;
          antenna = 2;
          break;
        case 2:
          DEBUG.println("antena 3...");
          power = std_power;
          antenna = 3;
          break;
        case 3:
          antenna = 0;
          DEBUG.println("antena 4...");
          power = std_power;
          newCommand = fnc_Roll;
          break;
        default:
          antenna = 0;
          break;
        }
        // statefunc = setAntenaPort;
      }
      DEBUG.println("Antena setada...");
    }
  }

  phyPort = antenna;
  powerLevel = power;

//  Serial.print("Setando Antena: ");
//  Serial.println(antenna);
//  Serial.println(power);
  formatCMD(devId, cmd_RFID_AntennaPortSetConfiguration, cmdBuff);
  cmdBuff[6] = antennaPort;
  cmdBuff[7] = powerLevel & 0xff;
  cmdBuff[8] = powerLevel >> 8;
  cmdBuff[9] = dwellTime & 0xff;
  cmdBuff[10] = dwellTime >> 8;
  cmdBuff[11] = nCycles & 0xff;
  cmdBuff[12] = nCycles >> 8;
  cmdBuff[13] = phyPort;
  sendCMD(cmdBuff);
  //Serial.println("Setando Antena");
  statefunc = setAntenaPortrecv;
}

void setAntenaPortrecv()
{
  fsmErros = 0;
  uint8_t buff[16];
  if (ringSize(RXring) >= 16)
  {
    for (int i = 0; i < 16; i++)
    {
      buff[i] = ringGet(&RXring);
    }
    if (checkCRC(CMD_PACKET_SIZE, buff) != 0)
    {
      //Serial.println("Deu Ruim CRC");
      statefunc = rfidModuleERROR;
      return;
    }
    if ((buff[6] != 0) || (buff[5] != cmd_RFID_AntennaPortSetConfiguration))
    {
      //Serial.println("Deu Ruim");
      statefunc = rfidModuleERROR;
      return;
    }
    //Serial.println("antena conf ok");
    statefunc = setSingulationAlgo;
  }
  else
  {
    //Serial.println("try again 3");
    countError();
  }
}

void TestSetAntenaPort()
{
  fsmErros = 0;
  uint8_t antennaPort = 0;
  uint16_t powerLevel = 300;
  formatCMD(devId, cmd_RFID_TestSetAntenaPortConfiguration, cmdBuff);
  cmdBuff[6] = antennaPort;
  cmdBuff[7] = powerLevel & 0xff;
  cmdBuff[8] = powerLevel >> 8;
  sendCMD(cmdBuff);
  Serial.println("Setando Antena teste");
  statefunc = TestSetAntenaPortrecv;
}

void TestSetAntenaPortrecv()
{
  fsmErros = 0;
  uint8_t buff[16];
  if (ringSize(RXring) >= 16)
  {
    for (int i = 0; i < 16; i++)
    {
      buff[i] = ringGet(&RXring);
    }
    if (checkCRC(CMD_PACKET_SIZE, buff) != 0)
    {
      //Serial.println("Deu Ruim CRC");
      statefunc = rfidModuleERROR;
      return;
    }
    if ((buff[6] != 0) || (buff[5] != cmd_RFID_TestSetAntenaPortConfiguration))
    {
      //Serial.println("Deu Ruim");
      statefunc = rfidModuleERROR;
      return;
    }
    Serial.println("antena conf ok");
    statefunc = TestTurnOnCarrierWave;
  }
  else
  {
    //Serial.println("try again 3");
    countError();
  }
}
void TestTurnOnCarrierWave()
{

  formatCMD(devId, 0x88, cmdBuff);
  sendCMD(cmdBuff);

  statefunc = TestTurnOnCarrierWaverecv;
}
void TestTurnOnCarrierWaverecv()
{
  uint8_t buff[16];
  if (ringSize(RXring) >= 16)
  {
    for (int i = 0; i < 16; i++)
    {
      buff[i] = ringGet(&RXring);
    }
    if (checkCRC(CMD_PACKET_SIZE, buff) != 0)
    {
      //Serial.println("Deu Ruim CRC");
      statefunc = rfidModuleERROR;
      return;
    }
    if ((buff[6] != 0) || (buff[5] != 0x88))
    {
      Serial.println("Deu Ruim");
      statefunc = setAntenaPort;
      return;
    }
    //Serial.println("singulation PARAM ok");
    statefunc = EngGetRFRevPower;
  }
  else
  {
    //Serial.println("try again 5");
    countError();
  }
}
void TestTurnOffCarrierWave()
{
  formatCMD(devId, 0x89, cmdBuff);
  sendCMD(cmdBuff);
  statefunc = TestTurnOffCarrierWaverecv;
}
void TestTurnOffCarrierWaverecv()
{
  uint8_t buff[16];
  if (ringSize(RXring) >= 16)
  {
    for (int i = 0; i < 16; i++)
    {
      buff[i] = ringGet(&RXring);
    }
    if (checkCRC(CMD_PACKET_SIZE, buff) != 0)
    {
      //Serial.println("Deu Ruim CRC");
      statefunc = rfidModuleERROR;
      return;
    }
    if ((buff[6] != 0) || (buff[5] != 0x89))
    {
      //Serial.println("Deu Ruim");
      statefunc = rfidModuleERROR;
      return;
    }
    //Serial.println("singulation PARAM ok");
    statefunc = setAntenaPort;
  }
  else
  {
    //Serial.println("try again 5");
    countError();
  }
}


void EngGetRFRevPower()
{

  fsmErros = 0;
  formatCMD(devId, 0x93, cmdBuff);

  if (Reg == 0)
  {
    cmdBuff[6] = 0x01;
  }
  else
  {
    cmdBuff[6] = 0x00;
  }

  sendCMD(cmdBuff);

  //Serial.println("Coletando potencia da antena");

  statefunc = EngGetRFPowerrecv;
}

void EngGetRFPowerrecv()
{
  fsmErros = 0;
  uint8_t buff[16];
  if (ringSize(RXring) >= 16)
  {
    for (int i = 0; i < 16; i++)
    {
      buff[i] = ringGet(&RXring);
    }
    if (checkCRC(CMD_PACKET_SIZE, buff) != 0)
    {
      //Serial.println("Deu Ruim CRC");
      statefunc = rfidModuleERROR;
      return;
    }
    if ((buff[6] != 0) || (buff[5] != 0x93))
    {
      //Serial.println("Deu Ruim");
      statefunc = rfidModuleERROR;
      return;
    }
    //recebe a potencia de retorno
    if (Reg == 0)
    {
      paPWR = buff[7] | buff[8] << 8;
      Serial.print("PWR = ");
      Serial.print(paPWR);
      Serial.println(" dB");
      Serial.println("retPWR ok");
      Reg = 1;
      statefunc = EngGetRFRevPower;
    }
    else
    {
      retPWR = buff[7] | buff[8] << 8;
      Serial.print("PWR = ");
      Serial.print(retPWR);
      Serial.println(" dB");
      Serial.println("paPWR ok");

      Serial.print("Return Loss = ");
      double dbValue = (paPWR - retPWR);
      dbValue = (dbValue != 0) ? dbValue / 10 : 0;
      Serial.print(dbValue);
      Serial.println(" dB");
      Reg = 0;

      statefunc = TestTurnOffCarrierWave;
    }
  }
  else
  {
    //Serial.println("try again 3");
    countError();
  }
}
void setSingulationAlgo()
{
  fsmErros = 0;
  simpleSet(devId, cmd_RFID_18K6CSetCurrentSingulationAlgorithm, 0);
  //Serial.println("Setando Singluation algo");
  statefunc = setSingulationAlgorecv;
}

void setSingulationAlgorecv()
{
  uint8_t buff[16];
  if (ringSize(RXring) >= 16)
  {
    for (int i = 0; i < 16; i++)
    {
      buff[i] = ringGet(&RXring);
    }
    if (checkCRC(CMD_PACKET_SIZE, buff) != 0)
    {
      //Serial.println("Deu Ruim CRC");
      statefunc = rfidModuleERROR;
      return;
    }
    if ((buff[6] != 0) || (buff[5] != cmd_RFID_18K6CSetCurrentSingulationAlgorithm))
    {
      //Serial.println("Deu Ruim");
      statefunc = rfidModuleERROR;
      return;
    }
    //Serial.println("singulation algo ok");
    statefunc = setSingulationParameters;
  }
  else
  {
    //Serial.println("try again 4");
    countError();
  }
}

void setSingulationParameters()
{
  fsmErros = 0;
  uint8_t qValue = 3;
  uint8_t retryCount = 0;
  uint8_t toggleTarget = 1;
  uint8_t repeatUntilNoTags = 0;
  formatCMD(devId, cmd_RFID_18K6CSetSingulationAlgorithmParameters, cmdBuff);
  cmdBuff[6] = 0;
  cmdBuff[7] = qValue;
  cmdBuff[8] = retryCount;
  cmdBuff[9] = toggleTarget;
  cmdBuff[10] = repeatUntilNoTags;
  sendCMD(cmdBuff);
  //Serial.println("Setando Singluation PARAM");
  statefunc = setSingulationParametersrecv;
}

void setSingulationParametersrecv()
{
  uint8_t buff[16];
  if (ringSize(RXring) >= 16)
  {
    for (int i = 0; i < 16; i++)
    {
      buff[i] = ringGet(&RXring);
    }
    if (checkCRC(CMD_PACKET_SIZE, buff) != 0)
    {
      //Serial.println("Deu Ruim CRC");
      statefunc = rfidModuleERROR;
      return;
    }
    if ((buff[6] != 0) || (buff[5] != cmd_RFID_18K6CSetSingulationAlgorithmParameters))
    {
      //Serial.println("Deu Ruim");
      statefunc = rfidModuleERROR;
      return;
    }
    //Serial.println("singulation PARAM ok");
    statefunc = startInventory;
  }
  else
  {
    //Serial.println("try again 5");
    countError();
  }
}

void startInventory()
{
  fsmErros = 0;
  uint8_t performSelect = 0;
  uint8_t performPostMatch = 0;
  uint8_t performGuardMode = 0;
  formatCMD(devId, cmd_RFID_18K6CTagInventory, cmdBuff);
  cmdBuff[6] = performSelect;
  cmdBuff[7] = performPostMatch;
  cmdBuff[8] = performGuardMode;
  //Serial.println("Start Inventory...");
  sendCMD(cmdBuff);
  statefunc = startInventoryrecv;
}

void startInventoryrecv()
{
  uint8_t buff[16];
  if (ringSize(RXring) >= 16)
  {
    for (int i = 0; i < 16; i++)
    {
      buff[i] = ringGet(&RXring);
    }
    if (checkCRC(CMD_PACKET_SIZE, buff) != 0)
    {
      if(debugRFIDMode){
        Serial.println("Error 3 - CRC Error to startInventoryRecv :  ");
        }
      statefunc = rfidModuleERROR;
      return;
    }
    if ((buff[6] != 0) || (buff[5] != cmd_RFID_18K6CTagInventory))
    {
      if(debugRFIDMode){
        Serial.println("Error 4 - Error to startInventoryRecv :  ");
        }
      statefunc = rfidModuleERROR;
      return;
    }
    if(debugRFIDMode){
      //DEBUG.println("Start Inventory - OK! ");
    }
    statefunc = justPass;
  }
  else
  {
    //Serial.println("try again inv");
    countError();
  }
}

void justPass()
{
  uint8_t aux;
  fsmErros = 0;
  if (ringSize(RXring))
  {
    aux = ringGet(&RXring);
    switch (aux)
    {
    case 'A':
    {
      //Serial.write("pacote de TAG_ACESS recebido\n");
      statefunc = trataAcess;
      break;
    }
    case 'W':
    {
      //Serial.write("pacote de WORK recebido\n");
      statefunc = trataWork;
      break;
    }
    case 'B':
    {
      //Serial.write("pacote de INICIO recebido\n");
      statefunc = trataIninicio;
      break;
    }
    case 'E':
    {
      //Serial.write("pacote de FIM recebido\n");
      statefunc = trataFim;
      break;
    }
    case 'I':
    {
      //çSerial.write("pacote de TAG recebido\n");
      statefunc = trataTAG;
      break;
    }
    default:
    {
      statefunc = rfidModuleERROR;
      break;
    }
    }
  }
}

void trataIninicio()
{
  uint8_t buff[24];
  if (ringSize(RXring) >= 23)
  {
    buff[0] = 'B';
    for (int i = 0; i < 23; i++)
    {
      buff[i + 1] = ringGet(&RXring);
    }
    if (checkCRC(24, buff) != 0)
    {
      //Serial.println("Deu Ruim CRC INI");
      statefunc = rfidModuleERROR;
      return;
    }
    //Serial.println("Deu boa INI");
    lendo = 1;
    statefunc = justPass;
  }
  else
  {
    countError();
  }
}

void trataFim()
{
  uint8_t buff[24];
  if (ringSize(RXring) >= 23)
  {
    buff[0] = 'E';
    for (int i = 0; i < 23; i++)
    {
      buff[i + 1] = ringGet(&RXring);
    }
    if (checkCRC(24, buff) != 0)
    {
      //Serial.println("Deu Ruim CRC FIM");
      statefunc = rfidModuleERROR;
      return;
    }
    //Serial.println("Deu boa FIM");
    wait = millis();

    if (newCommand == fnc_Antenna_test)
    {

      Serial.println("alterando funcao!");
      statefunc = TestSetAntenaPort;
    }
    else
      statefunc = idle;
    lendo = 0;
  }
  else
  {
    countError();
  }
}

void trataWork()
{
  uint8_t buff[24];
  if (ringSize(RXring) >= 23)
  {
    buff[0] = 'W';
    for (int i = 0; i < 23; i++)
    {
      buff[i + 1] = ringGet(&RXring);
    }
    if (checkCRC(24, buff) != 0)
    {
      //Serial.println("Deu Ruim CRC WORK");
      statefunc = rfidModuleERROR;
      return;
    }
    //Serial.println("Deu boa WORK");
    statefunc = justPass;
  }
  else
  {
    countError();
  }
}

void trataAcess()
{
  uint8_t buff[64];
  if (ringSize(RXring) >= 63)
  {
    buff[0] = 'A';
    for (int i = 0; i < 63; i++)
    {
      buff[i + 1] = ringGet(&RXring);
    }
    if (checkCRC(64, buff) != 0)
    {
      //Serial.println("Deu Ruim CRC ACEss");
      statefunc = rfidModuleERROR;
      return;
    }
    //Serial.println("Deu boa ACEss");
    statefunc = justPass;
  }
  else
  {
    countError();
  }
}

void trataTAG()
{
  uint8_t buff[64];
  char str[128];
  float rssi_float;
  if (ringSize(RXring) >= 63)
  {
    buff[0] = 'I';
    for (int i = 0; i < 63; i++)
    {
      buff[i + 1] = ringGet(&RXring);
    }
    if (checkCRC(64, buff) != 0)
    {
      ////Serial.println("Deu Ruim CRC TAG");
      statefunc = rfidModuleERROR;
      return;
    }
    //çSerial.println("Deu boa TAG");
    tags++;
    //tratamento da tag
    uint16_t len, rssi, antena, i;
    len = uint16_t(buff[11] << 8) + buff[10];
    len = (len - 3);
    len = len * 4;
    rssi = buff[23] | buff[22] << 8;
    antena = uint16_t(buff[25] << 8) + buff[24];
    struct tag_s aux;
    aux.size = len;
    memcpy(aux.data, &buff[26], len);
    //Serial.println((char*)aux.data,HEX);
    aux.rssi = rssi;
    aux.antena = antenna;
    tagRingPut(&TAGring, aux);
    statefunc = justPass;
  }
  else
  {
    ////Serial.println("try again TAG");
    countError();
  }
}

void idle()
{

  unsigned long currentMillis = millis(); //Tempo atual em ms
  if (currentMillis - wait > 100)
  {
    statefunc = startInventory;
  }
  else
  {

    if (newCommand == fnc_Roll)
    {
      switch (antenna)
      {
      case 0:
        antenna = 1;
        break;
      case 1:
        if (std_config == RU_861) //861 só tem 2 canais, portanto o inventário é feito só entre essas 2 antenas
          antenna = 0;
        else //Spitfire tem 4 antenas, então continua o ciclo
          antenna = 2;
        break;
      case 2:
        antenna = 3;
        break;
      case 3:
        antenna = 0;
        break;
      default:
        antenna = 0;
        break;
      }
    }
    //Serial.print("Changing port - antenna: ");
    //Serial.println(antenna);

    statefunc = setAntenaPort;
  }
}

void rfidModuleERROR()
{
//  ZBTxRequest xBee;
//  sprintf(string, "\x02;E01;\x03");
//  Serial.println(string);
//
//  Serial.println("# Soft reseting ...");
//  digitalWrite(A12, LOW);
//  delay(500);
//  digitalWrite(A12, HIGH);
//  delay(200);
//
//  ringReset(&RXring);
//  executeSoftReset(RUN_SKETCH_ON_BOOT);
  //TODO resetar placa
  statefunc = getDeviceId;
}

//########################################################################################
//########################################################################################
//################################    RU-861 Functions   #################################
//########################################################################################
//########################################################################################

void sendCMD(uint8_t *buff)
{
  calculateCRC(CMD_PACKET_SIZE, buff);
  for (int c = 0; c < CMD_PACKET_SIZE; c++)
  {
    Serial1.write(buff[c]);
  }
}

//formatar o comando a ser enviado para o dispositivo no formato de envio de comando especificado no Command Reference Manual

void formatCMD(uint8_t id, uint8_t opp, uint8_t *buff)
{
  memset(buff, 0, 16);
  buff[0] = 'C';
  buff[1] = 'I';
  buff[2] = 'T';
  buff[3] = 'M';
  //device id
  buff[4] = id; //devId
  //operation
  buff[5] = opp;
}

void calculateCRC(uint8_t nbytes, uint8_t *buff)
{
  uint16_t crc;
  crc = ~crc16(buff, 14 * 8);
  buff[15] = crc >> 8;
  buff[14] = crc & 0xFF;
}

uint8_t checkCRC(uint8_t nbytes, uint8_t *buff)
{
  uint16_t crc;
  uint8_t low, high;
  crc = ~crc16(buff, (nbytes - 2) * 8);
  high = crc >> 8;
  low = crc & 0xff;
  if ((buff[nbytes - 1] != high) && (buff[nbytes - 2] != low))
  {
    return 1;
  }
  return 0;
}

#define POLY 0x1021

uint16_t crc16(uint8_t *buf, uint16_t bit_length)
{
  uint16_t shift, data, val, i;

  shift = 0xFFFF;

  for (i = 0; i < bit_length; i++)
  {
    if ((i % 8) == 0)
      data = (*buf++) << 8;
    val = shift ^ data;
    shift = shift << 1;
    data = data << 1;

    if (val & 0x8000)
      shift = shift ^ POLY;
  }
  return shift;
}

// ########################################################################################
// ########################################################################################
// ################################ ring buffer functions #################################
// ########################################################################################
// ########################################################################################
unsigned char ringSize(struct ring_s x)
{
  return x.size;
}

unsigned char ringPut(struct ring_s *x, uint8_t in)
{
  if (x->size < x->capacity)
  {
    //Serial.print(in );
    x->data[(x->position + x->size) % x->capacity] = in;
    x->size = x->size + 1;
    return 1;
  }
  //Serial.println("Estorou Buffer");
  rfidModuleERROR();
  return 0;
}

unsigned char ringGet(struct ring_s *x)
{
  uint8_t aux = 0;
  if (x->size > 0)
  {
    aux = x->data[x->position];
    x->position = (x->position + 1) % x->capacity;
    x->size--;
  }
  return aux;
}

void ringReset(struct ring_s *x)
{
  x->position = 0;
  x->size = 0;
}

unsigned char tagRingSize(struct tag_ring_s x)
{
  return x.size;
}

unsigned char tagRingPut(struct tag_ring_s *x, struct tag_s in)
{
  if (x->size < x->capacity)
  {
    x->tagList[(x->position + x->size) % x->capacity] = in;
    x->size = x->size + 1;
    return 1;
  }
  ////Serial.println("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
  return 0;
}

struct tag_s tagRingGet(struct tag_ring_s *x)
{
  struct tag_s aux;
  if (x->size > 0)
  {
    aux = x->tagList[x->position];
    x->position = (x->position + 1) % x->capacity;
    x->size--;
  }
  return aux;
}

//time_t requestSync()
//{
//  // Serial.write(TIME_REQUEST);
//  return 0; // the time will be sent later in response to serial mesg
//}
