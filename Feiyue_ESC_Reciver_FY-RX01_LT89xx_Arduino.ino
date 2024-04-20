/*
 Copyright (C) 2015 Rob van der Veer, <rob.c.veer@gmail.com>
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/* Connections:
 Arduino -> LT8900
 GND        GND
 3v3        VCC
 2          PKT
 9          CS
 10         RST
 11         MOSI
 12         MISO
 13         SCK
 Connect A3 to ground for sender, leave open for receiver role.
*/

#include <SPI.h>
#include "LT8900.h"

const uint8_t PIN_NRF_RST = 10;
const uint8_t PIN_NRF_CS = 9;
const uint8_t PIN_NRF_PKT = 2;
char sbuf[32];
uint8_t buf[255];
bool bind = false;
bool setChannelRX = false;
bool setChannelBind = true;

uint8_t dataLength = 0;
uint8_t dataThrottle = 0; //Газ
uint8_t dataSteering = 0; //Руль
uint8_t dataTrimSteeringAndThrottle = 0;   
uint8_t dataTrimThrottle = 0; //Трим газа
uint8_t dataTrimSteering = 0; //Трим руль
uint8_t dataCenter = 0; // dataTrimThrottle = 0-255 - 07? FF - 00 - 06? 00 - FF - 05
uint32_t bindTime = 0;
uint32_t dataTime = 0;

uint8_t data[] = {dataLength, dataThrottle, dataSteering, dataTrimSteeringAndThrottle, dataTrimThrottle, dataTrimSteering, dataCenter};

LT8900 lt(PIN_NRF_CS, PIN_NRF_PKT, PIN_NRF_RST);

uint8_t bit_reverse(uint8_t b_in)
{
    uint8_t b_out = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        b_out = (b_out << 1) | (b_in & 1);
        b_in >>= 1;
    }
    return b_out;
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println(F("\n\nLT8900 module sample, v0.1.\n\n"));

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  pinMode(PIN_NRF_PKT, INPUT);
  delay(100);
  lt.begin();
  
  lt.setCurrentControl(15,15);
  lt.setDataRate(LT8900::LT8910_62KBPS); //LT8910_62KBPS, LT8910_125KBPS, LT8910_250KBPS, LT8900_1MBPS
  lt.setChannel(70); // Bind Channel
  //REGISTER 36 - 39
  lt.setSyncWord(0x038003805a5a0380); // f36 bind word+bit reversed
  // REGISTER 32
  lt.setPreambleLength(3);  // 1 = 1 byte, 2 = 2 byte .... 8 = 8 byte
  lt.setSyncWordLength(32); // 16 = 16 bits, 32 = 32 bits, 48 = 48 bits, 64 = 64 bits
  lt.setTrailerLength(4); // 4 = 4 bits, 6 = 6 bits, 8 = 8 bits ..... 18 = 18 bits
  lt.setDataPacketType(0); // 0 = NRZ law data, 1 = Manchester data type, 2 = 8bit/10bit line code, 3 = Interleave data type
  lt.setFECType(0); // 0 = No FEC, 1 = FEC13, 2 = FEC23
  // REGISTER 41
  lt.setCRC(1); // 0 = CRC OFF, 1 = CRC ON
  lt.setScramble(0);  // 0 = Scramble OFF, 1 = Scramble ON
  lt.setPackLengthEn(1); // 0 = Off, 1 = LT8900 regards first byte of payload as packet length descriptor byte. 
  lt.setFwTermTx(1); // 0 = FW (MCU) handles length and terminates TX., 1 = When FIFO write point equals read point, LT8900 will terminate TX when FW handle packet length.
  lt.setAutoACK(1); //  0 = After receive, do not send ACK or NACK; just go to IDLE., 1 = After receiving data, automatically send ACK/NACK.
  //lt.whatsUp(Serial);
}

void loop()
{
    if(!bind){
      binding();
    }else {
      getData();
    }
}

void binding(){
  if(setChannelBind){
    lt.setChannel(70);
    lt.setSyncWord(0x038003805a5a0380);
    lt.startListening();
    setChannelBind = false;
    setChannelRX = true;
  }

  if (lt.available()) {
    bindTime = millis();
    int packetSize = 7;
    lt.read(buf, packetSize);
    if (packetSize > 0)
    {
        lt.sendPacket(buf, sizeof(buf));
    }
    lt.startListening();
  }
  
  if(millis() - bindTime >= 500){
      getData();
  }
}

void getData(){
  if(setChannelRX){
    lt.setChannel(21);
    lt.setSyncWord(0x038003805a5a0B24);
    lt.startListening();
    setChannelRX = false;
    setChannelBind = true;
  }
  if (lt.available()) {
    dataTime = millis();
    bind = true;
    int packetSize = 7;
    lt.read(buf, packetSize);
      if (packetSize > 0)
      {
        dataLength = buf[0];
        dataThrottle = buf[1];
        dataSteering = buf[2];
        dataTrimSteeringAndThrottle = buf[3];
        dataTrimThrottle = buf[4];
        dataTrimSteering = buf[5];
        dataCenter = buf[6];
        for(int i = 0; i < packetSize; i++)
        {
          sprintf_P(sbuf, PSTR("%02x "), buf[i]);
          Serial.print(sbuf);
        }
        Serial.print(" dataLength = ");
        Serial.print(dataLength);
        Serial.print(" dataThrottle = ");
        Serial.print(dataThrottle);
        Serial.print(" dataSteering = ");
        Serial.print(dataSteering);
        Serial.print(" dataTrimSteeringAndThrottle = ");
        Serial.print(dataTrimSteeringAndThrottle);
        Serial.print(" dataTrimThrottle = ");
        Serial.print(dataTrimThrottle);
        Serial.print(" dataTrimSteering = ");
        Serial.print(dataTrimSteering);
        Serial.print(" dataCenter = ");
        Serial.print(dataCenter);
      Serial.println("");

      }
      lt.startListening();
  }

  if(millis() - dataTime >= 500){
      binding();
  }

}
