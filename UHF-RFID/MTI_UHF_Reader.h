#ifndef MTI_UHF_Reader_H
#define MTI_UHF_Reader_H

#include <SoftwareSerial.h>
#include "Arduino.h"

#define MAX_MSG_SIZE 16

#define TMR_SR_OPCODE_VERSION 0x03
#define TMR_SR_OPCODE_SET_BAUD_RATE 0x06
#define TMR_SR_OPCODE_READ_TAG_ID_SINGLE 0x21
#define TMR_SR_OPCODE_READ_TAG_ID_MULTIPLE 0x22
#define TMR_SR_OPCODE_WRITE_TAG_ID 0x23
#define TMR_SR_OPCODE_WRITE_TAG_DATA 0x24
#define TMR_SR_OPCODE_KILL_TAG 0x26
#define TMR_SR_OPCODE_READ_TAG_DATA 0x28
#define TMR_SR_OPCODE_CLEAR_TAG_ID_BUFFER 0x2A
#define TMR_SR_OPCODE_MULTI_PROTOCOL_TAG_OP 0x2F
#define TMR_SR_OPCODE_GET_READ_TX_POWER 0x62
#define TMR_SR_OPCODE_GET_WRITE_TX_POWER 0x64
#define TMR_SR_OPCODE_GET_POWER_MODE 0x68
#define TMR_SR_OPCODE_GET_READER_OPTIONAL_PARAMS 0x6A
#define TMR_SR_OPCODE_GET_PROTOCOL_PARAM 0x6B
#define TMR_SR_OPCODE_SET_ANTENNA_PORT 0x91
#define TMR_SR_OPCODE_SET_TAG_PROTOCOL 0x93
#define TMR_SR_OPCODE_SET_READ_TX_POWER 0x92
#define TMR_SR_OPCODE_SET_WRITE_TX_POWER 0x94
#define TMR_SR_OPCODE_SET_REGION 0x97
#define TMR_SR_OPCODE_SET_READER_OPTIONAL_PARAMS 0x9A
#define TMR_SR_OPCODE_SET_PROTOCOL_PARAM 0x9B

#define COMMAND_TIME_OUT  2000 //Number of ms before stop waiting for response from module

//Define all the ways functions can return
#define ALL_GOOD                        0
#define ERROR_COMMAND_RESPONSE_TIMEOUT  1
#define ERROR_CORRUPT_RESPONSE          2
#define ERROR_WRONG_OPCODE_RESPONSE     3
#define ERROR_UNKNOWN_OPCODE            4
#define RESPONSE_IS_TEMPERATURE         5
#define RESPONSE_IS_KEEPALIVE           6
#define RESPONSE_IS_TEMPTHROTTLE        7
#define RESPONSE_IS_TAGFOUND            8
#define RESPONSE_IS_NOTAGFOUND          9
#define RESPONSE_IS_UNKNOWN             10
#define RESPONSE_SUCCESS            11
#define RESPONSE_FAIL               12

//Define the allowed regions - these set the internal freq of the module
#define REGION_INDIA        0x04
#define REGION_JAPAN        0x05
#define REGION_CHINA        0x06
#define REGION_EUROPE       0x08
#define REGION_KOREA        0x09
#define REGION_AUSTRALIA    0x0B
#define REGION_NEWZEALAND   0x0C
#define REGION_NORTHAMERICA 0x0D
#define REGION_OPEN         0xFF

struct RFIDdata
{
  int dataLen;
  byte chk;
  boolean valid;
  unsigned char raw[5];
};

enum RFIDType
{
  RFID_UART,
  RFID_WIEGAND
};

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

  cmd_RFID_MacGetFirmwareVersion = 0x60,

  cmd_RFID_TestSetAntenaPortConfiguration = 0x80,
  cmd_RFID_TestGetAntenaPortConfiguration,

  cmd_RFID_EngGEtTemperature = 0x91,

  cmd_RFID_EngGetRFPower = 0x93,

  cmd_RFID_BAUD_RATE = 0xA4,

};


class MTI_UHF_Reader
{
  private:
    HardwareSerial *_UHF_Reader_Serial; //The generic connection to user's chosen serial hardware
    HardwareSerial *_debug_Serial; //The stream to send debug messages to if enabled
    uint8_t _head = 0; //Tracks the length of the incoming message as we poll the software serial
    boolean _printDebug = false; //Flag to print the serial commands we are sending to the Serial port for debug

  public:
    MTI_UHF_Reader(void);

    bool begin(HardwareSerial &serialPort); //If user doesn't specify then Serial will be used

    void enableDebugging(HardwareSerial &debugPort); //Turn on command sending and response printing. If user doesn't specify then Serial will be used
    void disableDebugging(void);

    void setBaud(long baudRate);
    void getVersion(void);
    void setReadPower(int16_t powerSetting);
    void getReadPower();
    void setWritePower(int16_t powerSetting);
    void getWritePower();
    void setRegion(uint8_t region);
    void setAntennaPort();
    void setAntennaSearchList();
    void setTagProtocol(uint8_t protocol = 0x05);

    void startReading(void); //Disable filtering and start reading continuously
    void stopReading(void); //Stops continuous read. Give 1000 to 2000ms for the module to stop reading.

    void enableReadFilter(void);
    void disableReadFilter(void);

    void setReaderConfiguration(uint8_t option1, uint8_t option2);
    void getOptionalParameters(uint8_t option1, uint8_t option2);
    void setProtocolParameters(void);
    void getProtocolParameters(uint8_t option1, uint8_t option2);

    uint8_t parseResponse(void);

    uint8_t getTagEPCBytes(void); //Pull number of EPC data bytes from record response.
    uint8_t getTagDataBytes(void); //Pull number of tag data bytes from record response. Often zero.
    uint16_t getTagTimestamp(void); //Pull timestamp value from full record response
    uint32_t getTagFreq(void); //Pull Freq value from full record response
    int8_t getTagRSSI(void); //Pull RSSI value from full record response

    bool check(void);

    uint8_t readTagEPC(uint8_t *epc, uint8_t &epcLength, uint16_t timeOut = COMMAND_TIME_OUT);
    uint8_t writeTagEPC(char *newID, uint8_t newIDLength, uint16_t timeOut = COMMAND_TIME_OUT);

    uint8_t readData(uint8_t bank, uint32_t address, uint8_t *dataRead, uint8_t &dataLengthRead, uint16_t timeOut = COMMAND_TIME_OUT);
    uint8_t writeData(uint8_t bank, uint32_t address, uint8_t *dataToRecord, uint8_t dataLengthToRecord, uint16_t timeOut = COMMAND_TIME_OUT);

    uint8_t readUserData(uint8_t *userData, uint8_t &userDataLength, uint16_t timeOut = COMMAND_TIME_OUT);
    uint8_t writeUserData(uint8_t *userData, uint8_t userDataLength, uint16_t timeOut = COMMAND_TIME_OUT);

    uint8_t readKillPW(uint8_t *password, uint8_t &passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);
    uint8_t writeKillPW(uint8_t *password, uint8_t passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);

    uint8_t readAccessPW(uint8_t *password, uint8_t &passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);
    uint8_t writeAccessPW(uint8_t *password, uint8_t passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);

    uint8_t readTID(uint8_t *tid, uint8_t &tidLength, uint16_t timeOut = COMMAND_TIME_OUT);

    uint8_t killTag(uint8_t *password, uint8_t passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);

    void sendMessage(uint8_t opcode, uint8_t *data = 0, uint8_t size = 0, uint16_t timeOut = COMMAND_TIME_OUT, boolean waitForResponse = true);
    void sendCommand(uint16_t timeOut = COMMAND_TIME_OUT, boolean waitForResponse = true);

    void printMessageArray(void);

    uint16_t calculateCRC(uint8_t *u8Buf, uint8_t len);

    //Variables

    //This is our universal msg array, used for all communication
    //Before sending a command to the module we will write our command and CRC into it
    //And before returning, response will be recorded into the msg array. Default is 255 bytes.
    uint8_t msg[MAX_MSG_SIZE];

    //uint16_t tags[MAX_NUMBER_OF_TAGS][12]; //Assumes EPC won't be longer than 12 bytes
    //uint16_t tagRSSI[MAX_NUMBER_OF_TAGS];
    //uint16_t uniqueTags = 0;



};

#endif //__MTI_UHF_Reader_H__
