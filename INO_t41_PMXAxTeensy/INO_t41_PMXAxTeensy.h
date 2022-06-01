// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _INO_t41_PMXAxTeensy_H_
#define _INO_t41_PMXAxTeensy_H_
#include "Arduino.h"



// PING No action. Used for obtaining a Status Packet (0 parameter)
#define AX_CMD_PING   0x01
// Reading values in the Control Table (2 parameters)
#define AX_CMD_READ_DATA   0x02
// Writing values to the Control Table (2+ parameters)
#define AX_CMD_WRITE_DATA  0x03


// AXTeensy commands
#define AXT_CMD_SET_LED_ON      10
#define AXT_CMD_SET_LED_OFF     11
#define AXT_CMD_GET_ADC         12
#define AXT_CMD_PING_AX         20
#define AXT_CMD_READ_AX         21
#define AXT_CMD_WRITE_AX        22

#define AXT_STATUS             0x88
#define AXT_OK                  0
// Errors
#define AXT_ERROR_SEND_CMD      -20;
#define AXT_ERROR_READ_STATUS   -30;
#define AXT_ERROR_CRC           -40;

#define AXT_AX_MISSING          8



// AX12 memory address
#define P_MODEL_NUMBER        0
#define P_VERSION         2
#define P_ID            3
#define P_BAUD_RATE         4
#define P_RETURN_DELAY_TIME     5
#define P_CW_ANGLE_LIMIT      6
#define P_CCW_ANGLE_LIMIT     8
#define P_SYSTEM_DATA2        10
#define P_LIMIT_TEMPERATURE     11
#define P_DOWN_LIMIT_VOLTAGE    12
#define P_UP_LIMIT_VOLTAGE      13
#define P_MAX_TORQUE        14
#define P_RETURN_LEVEL        16
#define P_ALARM_LED         17
#define P_ALARM_SHUTDOWN      18
#define P_OPERATING_MODE      19
#define P_DOWN_CALIBRATION        20
#define P_UP_CALIBRATION      22
#define P_TORQUE_ENABLE       24
#define P_LED           25
#define P_CW_COMPLIANCE_MARGIN    26
#define P_CCW_COMPLIANCE_MARGIN   27
#define P_CW_COMPLIANCE_SLOPE   28
#define P_CCW_COMPLIANCE_SLOPE    29
#define P_GOAL_POSITION       30
#define P_GOAL_SPEED        32
#define P_TORQUE_LIMIT        34
#define P_PRESENT_POSITION      36
#define P_PRESENT_SPEED       38
#define P_PRESENT_LOAD        40
#define P_PRESENT_VOLTAGE     42
#define P_PRESENT_TEMPERATURE   43
#define P_REGISTERED_INSTRUCTION  44
#define P_PAUSE_TIME        45
#define P_MOVING          46
#define P_LOCK            47
#define P_PUNCH           48


void setup();
void loop();

int getADC(int adc);
void setLedOn(int led);
void setLedOff(int led);
void setAXTxMode(int ax);
void setAXRxMode(int ax);
int getAddressSize(int address);
uint8_t sendAX(uint8_t port, uint8_t* buffer, int packetSize, uint8_t* result, int parametersToRead);
int pingAX(uint8_t port, uint8_t id);
int readAXData(uint8_t port, uint8_t id, uint8_t address, int* err);
int writeAXData(uint8_t port, uint8_t id, uint8_t address, uint16_t data);


uint8_t computeCRC(uint8_t c, uint8_t *buf, uint16_t length);
int checkCRC(uint8_t* buf, uint16_t length);


// Callbacks.
void after_receive(size_t length, uint16_t address);
//void after_transmit(uint16_t address);
//void before_transmit(uint16_t address);

//Do not add code below this line
#endif /* _INO_t41_PMXAxTeensy_H_ */
