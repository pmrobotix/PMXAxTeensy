

// Do not remove the include below
#include "INO_t41_PMXAxTeensy.h"

#include <i2c_register_slave.h>

// Port AXT I2C_1 : Slave1
I2CSlave& slave = Slave1;

// Callbacks.
void after_receive(size_t length, uint16_t address);

// Double receive buffers to hold data from master.
const size_t slave_rx_buffer_size = 9;
uint8_t slave_rx_buffer[slave_rx_buffer_size] = {};
uint8_t buffer[slave_rx_buffer_size] = {};
volatile size_t slave_bytes_received = 0;
// Tx buffer
const size_t tx_buffer_size = 10;
uint8_t tx_buffer[tx_buffer_size] = {};




// PING No action. Used for obtaining a Status Packet (0 parameter)
#define AX_CMD_PING   0x01
// Reading values in the Control Table (2 parameters)
#define AX_CMD_READ_DATA   0x02
// Writing values to the Control Table (2+ parameters)
#define AX_CMD_WRITE_DATA  0x03

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


// Get the adc value (12bits)
// @param ADC :  1 - 4
// @returns value : 0 - 4095
int getADC(int adc) {
  if (adc < 1 || adc > 4) {
    return 0;
  }
  int x = analogRead(adc + 37);
  return x;
}


// Set the led on
// @param led :  3 - 18
void setLedOn(int led) {
  if (led == 3) {
    digitalWrite(32, HIGH);
  } else if (led == 4) {
    digitalWrite(31, HIGH);
  } else if (led == 5) {
    digitalWrite(30, HIGH);
  } else if (led == 6) {
    digitalWrite(13, HIGH);
  } else if (led == 7) {
    digitalWrite(5, HIGH);
  } else if (led == 8) {
    digitalWrite(4, HIGH);
  }
}
// Set the led on
// @param led :  3 - 18
void setLedOff(int led) {
  if (led == 3) {
    digitalWrite(32, LOW);
  } else if (led == 4) {
    digitalWrite(31, LOW);
  } else if (led == 5) {
    digitalWrite(30, LOW);
  } else if (led == 6) {
    digitalWrite(13, LOW);
  } else if (led == 7) {
    digitalWrite(5, LOW);
  } else if (led == 8) {
    digitalWrite(4, LOW);
  }
}
void setAXTxMode(int ax) {
  if (ax == 1) {
    digitalWrite(6, HIGH);
  }
}
void setAXRxMode(int ax) {
  if (ax == 1) {
    digitalWrite(6, LOW);
  }
}

int getAddressSize(int address) {
  switch (address) {
    case P_MODEL_NUMBER:
    case P_CW_ANGLE_LIMIT:
    case P_CCW_ANGLE_LIMIT:
    case P_MAX_TORQUE:
    case P_DOWN_CALIBRATION:
    case P_UP_CALIBRATION:
    case P_GOAL_POSITION:
    case P_GOAL_SPEED:
    case P_TORQUE_LIMIT:
    case P_PRESENT_POSITION:
    case P_PRESENT_SPEED:
    case P_PRESENT_LOAD:
    case P_PUNCH:
      return 2;
    default:
      return 1;
  }
}

/*
   buffer : trame à envoyer (sans checksum)
   packetSize : taille de la trame à envoyer (sans checksum)
   result : parametres à lire
   parametersToRead : nombre de parametres à lire

   retourne 0 si pas d'erreur
   250 si mauvais port
   251 si on ne trouve pas le premier octet du header à FF
   252 si le deuxieme octet du header n'est pas FF
   253 si le CRC est mauvais

*/
uint8_t sendAX(uint8_t port, uint8_t* buffer, int packetSize, uint8_t* result, int parametersToRead) {
  HardwareSerialIMXRT *uart = &Serial2;
  if(port==1){
    // AX1 : Serial 2
    uart = &Serial2;
  }else if(port==2){
    uart = &Serial3;
  }else if(port==3){
    uart = &Serial4;
  }else if(port==4){
    uart = &Serial5;
  }else{
      Serial.print("sendAX : bad port ");
      Serial.println(port);
     return 250;
  }

  int ax = 1;


  // Envoi de la trame

  uint8_t checksum = 0;

  //
  Serial.println("will write");
  for (int i = 0; i < packetSize; i++) {
    Serial.print(buffer[i]);
    Serial.print(" ");
  }
  for (int i = 2; i < packetSize; i++) {
    checksum += buffer[i];
  }
  checksum = ~checksum;
  Serial.println(checksum);
  Serial.println("sending...");
  // Write to UART
  setAXTxMode(ax);
  // delayMicroseconds(00);
  for (int i = 0; i < packetSize; i++) {
    uart->write(buffer[i]);
  }
  uart->write(checksum);

  int stop = 6 + parametersToRead;
  // fill buffer
  for (int i = 0; i < stop; i++) {
    result[i] = 0X00;
  }


  // Reception
  uart->flush();

  delayMicroseconds(1);
  setAXRxMode(ax);
  uart->clear();
  // On attend 10ms que le buffer de reception se remplisse
  delayMicroseconds(10000);

  // L' AX12 doit repondre apres les 10us, 50us par exemple

  // read until 0xFF
  int count = 0;
  int header_found = 0;

  int r0 = uart->read();
  while (r0 == -1) {
    r0 = uart->read();
    Serial.println(r0);
  }
  Serial.print("first read: ");
  Serial.println(r0);
  if (r0 == 0xFF) {
    header_found = 1;
  } else {
    while (r0 != 0xFF) {
      r0 = uart->read();
      Serial.println("other read: ");
      Serial.println(r0);
      count++;
      //nb d'essai pour sortir de la boucle
      if (r0 == 0xFF) {
        header_found = 1;
      }
      if (count > 5 && !header_found) {
        break;
        Serial.println("break >5");
      }
    }
  }
  if (header_found == 0) {
    Serial.println("no header found");
    // On a pas le début de la réponse du l'AX (FF)
    return 251;
  }

  result[0] = r0;

  for (int i = 1; i < stop; i++) {
    result[i] = uart->read();

    if (result[1] != 0xFF) {
      return 252;
    }
  }



  //verif CHECKSUM !!
  uint8_t checksumResult = 0;
  Serial.println("Read values:");
  for (int i = 2; i < stop - 1; i++) {
    checksumResult += result[i];
    Serial.println(result[i]);
  }
  checksumResult = ~checksumResult;

  if (checksumResult != result[stop - 1]) {
    return 253;
  }
  return result[4]; //on retourne l'erreur du paquet

}
/**
  @return   0 si AX présent
  @return 251 si AX absent
*/
int pingAX(uint8_t port, uint8_t id) {
  uint8_t packet[20];
  uint8_t result[20];
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  int nbParameters = 0;
  int length = nbParameters + 2;
  packet[3] = length;
  int instruction = AX_CMD_PING;
  packet[4] = instruction;
 Serial.print("PingAX port: ");
  Serial.print(port);
  Serial.print(", id:");
  Serial.println(id);
  // Send to AX
  int error = sendAX(port, packet, 5, result, 0);

  Serial.print("Error:");
  Serial.println(error);
  Serial.println();
  // Retry
  if(error!=0){
    // pas de reponse, on retry
    error = sendAX(port,packet, 5, result, 0);
  }
  return error;
}

int readAXData(uint8_t port, uint8_t id, uint8_t address, int* err) {
  int size = getAddressSize(address);
  Serial.print("Read AX port: ");
  Serial.print(port);
  Serial.print(", id:");
  Serial.print(id);
  Serial.print(", address:");
  Serial.println(address);

  uint8_t packet[20];
  uint8_t result[20];
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  int nbParameters = 2;
  int length = nbParameters + 2;
  packet[3] = length;
  int instruction = AX_CMD_READ_DATA;
  packet[4] = instruction;
  packet[5] = address;
  packet[6] = size;

  // Send to AX
  int error = sendAX(port, packet, 7, result, size);
  if (error != 0) {
    // pas de reponse, on retry
    error = sendAX(port, packet, 7, result, size);
  }

  *err = error;

  if (size == 1) {
    // 1
    return result[5];
  } else {
    // 2
    return result[5] + result[6] * 256;
  }
}

int writeAXData(uint8_t port, uint8_t id, uint8_t address, uint16_t data) {
  int size = getAddressSize(address);

  uint8_t packet[20];
  uint8_t result[20];
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  int length = size + 3;
  packet[3] = length;
  int instruction = AX_CMD_WRITE_DATA;
  packet[4] = instruction;
  packet[5] = address;
  if (size == 1) {
    packet[6] = data;
  } else {
    packet[6] = data & 0xff;
    packet[7] = (data >> 8);
  }

  // Send to AX
  int error = sendAX(port, packet, 6 + size, result, 0);
  if (error != 0) {
    // pas de reponse, on retry
    error = sendAX(port, packet, 6 + size, result, 0);
  }
  return error;
}

// the setup() method runs once, when the sketch starts

void setup() {
  // initialize LED3 to LED8 pins as an output.
  pinMode(32, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  // AX DIR
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);

  Serial2.begin(1000000);
  Serial3.begin(1000000);
  Serial4.begin(1000000);
  Serial5.begin(1000000);
  // ADC
  analogReadResolution(12);
  analogReadAveraging(1);
  Serial.begin(115200);
  Serial.println("AXTeensy starting");

  // Configure I2C Slave
  slave.after_receive(after_receive);
  slave.set_receive_buffer(slave_rx_buffer, slave_rx_buffer_size);
  //
  slave.set_transmit_buffer(tx_buffer, tx_buffer_size);

  // Start listening
  int i2cAddr = 8;
  slave.listen(i2cAddr);
  Serial.print("I2C Slave, addr : ");
  Serial.println(i2cAddr);
}

// the loop() methor runs over and over again,
// as long as the board has power
int count = 0;

void loop() {

  //
  int l = 6;
  setLedOn(l);   // set the LED on
  delay(500);                  // wait for a second
  setLedOff(l);    // set the LED off
  delay(500);                  // wait for a second
  // Serial.println(getADC(1));
  // Serial.print("---- loop ");
  // Serial.println(count);


  count++;
}


uint8_t computeCRC(uint8_t *buf, uint16_t length) {
  uint8_t result = 5;
  for (int i = 0; i < length; i++) {
    result = result * 3 + buf[i];
  }
  return result;
}

int checkCRC(uint8_t* buf, uint16_t length) {
  uint8_t crc = buf[length - 1];
  uint8_t computed = computeCRC(buf, length - 1);
  if (crc == computed) {
    return 0;
  }
  Serial.print("CRC ERROR : computed CRC is ");
  Serial.print(crc);
  Serial.print(" but should be ");
  Serial.println(computed);
  return -1;
}


// Called by the I2C interrupt service routine.
// This method must be as fast as possible.
// Do not perform IO in it.
void after_receive(size_t length, uint16_t address) {
  // This is the only time we can guarantee that the
  // receive buffer is not changing.
  // Copy the content so we can handle it in the main loop.
  memcpy(buffer, slave_rx_buffer, length);
  slave_bytes_received = length;
  if (length == 1 && buffer[0] == AXT_STATUS) {
    Serial.println("AXT_STATUS Received");
    Serial.println();
    return;
  }

  uint8_t cmd = buffer[0];
  if(cmd==AXT_CMD_SET_LED_ON){
    Serial.print("SET_LED_ON");
  }else if(cmd==AXT_CMD_SET_LED_OFF){
    Serial.print("SET_LED_OFF");
  }else if(cmd==AXT_CMD_GET_ADC){
    Serial.print("GET_ADC");
  }else if(cmd==AXT_CMD_PING_AX){
    Serial.print("PING_AX");
  }else if(cmd==AXT_CMD_READ_AX){
    Serial.print("READ_AX");
  }else if(cmd==AXT_CMD_WRITE_AX){
    Serial.print("WRITE_AX");
  }else{
    Serial.print("UNKNOWN command ");
  }
  Serial.print(" (address : ");
  Serial.print(address);
  Serial.print(") ");
  Serial.print(length);
  Serial.print(" bytes : ");
  for (uint8_t i = 0; i < length; i++) {
    Serial.print(buffer[i]);
    Serial.print(" ");
  }
  Serial.println();
  int responseSize=0;
  if (cmd == AXT_CMD_SET_LED_ON) {

    uint8_t led = buffer[1];
    responseSize=2;
    if (checkCRC(buffer, 3) != 0) {
      tx_buffer[0] = AXT_ERROR_CRC;
      Serial.println("SET_LED_ON : bad CRC");
    } else {
      setLedOn(led);
      tx_buffer[0] = AXT_OK;
    }
  } else if (cmd == AXT_CMD_SET_LED_OFF) {
    uint8_t led = buffer[1];
    responseSize=2;
    if (checkCRC(buffer, 3) != 0) {
      tx_buffer[0] = AXT_ERROR_CRC;
      Serial.println("SET_LED_OFF : bad CRC");
    } else {
      setLedOff(led);
      tx_buffer[0] = AXT_OK;
    }
    tx_buffer[1] = computeCRC(tx_buffer, 1);
  } else if (cmd == AXT_CMD_GET_ADC) {
    uint8_t adc = buffer[1];
    responseSize=4;
    if (checkCRC(buffer, 3) != 0) {
      tx_buffer[0] = AXT_ERROR_CRC;
      tx_buffer[1]=0;
      tx_buffer[2]=0;
      Serial.println("GET_ADC : bad CRC");
    } else {
      int data = getADC(adc);
      tx_buffer[0] = AXT_OK;
      uint8_t xlow = data & 0xff;
      uint8_t xhigh = (data >> 8);
      tx_buffer[1] = xlow;
      tx_buffer[2] = xhigh;
    }
  } else if (cmd == AXT_CMD_PING_AX) {
    uint8_t port = buffer[1];
    uint8_t id = buffer[2];
    responseSize=2;
    if (checkCRC(buffer, 4) != 0) {
      tx_buffer[0] = AXT_ERROR_CRC;
      Serial.println("PING_AX : bad CRC");
    } else {

      int r = pingAX(port,id);
      if (r == 0) {
        tx_buffer[0] = AXT_OK;
      } else {
        tx_buffer[0] = AXT_AX_MISSING;
      }
    }

  } else if (cmd == AXT_CMD_READ_AX) {
    uint8_t port = buffer[1];
    uint8_t id = buffer[2];
    uint8_t addr = buffer[3];
    responseSize=4;
    if (checkCRC(buffer, 5) != 0) {
      tx_buffer[0] = AXT_ERROR_CRC;
      Serial.println("READ_AX : bad CRC");
    } else {
      int err;
      int data = readAXData(port,id,addr,&err);
      tx_buffer[0] =err;
      uint8_t xlow = data & 0xff;
      uint8_t xhigh = (data >> 8);
      tx_buffer[1] = xlow;
      tx_buffer[2] = xhigh;
    }

  }else if (cmd == AXT_CMD_WRITE_AX) {
    uint8_t port = buffer[1];
    uint8_t id = buffer[2];
    uint8_t addr = buffer[3];
    uint8_t low = buffer[4];
    uint8_t high = buffer[5];
    responseSize=2;
    if (checkCRC(buffer, 7) != 0) {
      tx_buffer[0] = AXT_ERROR_CRC;
      Serial.println("WRITE AX : bad CRC");
    } else {
      int err = writeAXData(port,id,addr,high*256+low);
      tx_buffer[0] =err;
    }

  }else {
    responseSize=2;
    tx_buffer[0] = AXT_ERROR_CRC;
    Serial.print("Unsupported command ");
    Serial.println(cmd);
  }
  // CRC
  tx_buffer[responseSize-1] = computeCRC(tx_buffer, responseSize-1);
  Serial.print("TX buffer, ");
  Serial.print(responseSize);
  Serial.print(" bytes : ");
  for(int i=0;i<responseSize;i++){
    Serial.print(tx_buffer[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println();
}

