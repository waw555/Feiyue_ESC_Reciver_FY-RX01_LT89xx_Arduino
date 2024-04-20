/*
 Copyright (C) 2015 Rob van der Veer, <rob.c.veer@gmail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include <SPI.h>
#include "LT8900.h"

#define REGISTER_READ       0b10000000  //bin
#define REGISTER_WRITE      0b00000000  //bin
#define REGISTER_MASK       0b01111111  //bin

#define R_CHANNEL           7
#define CHANNEL_RX_BIT      7
#define CHANNEL_TX_BIT      8
#define CHANNEL_MASK        0b01111111  ///bin
#define DEFAULT_CHANNEL     0x30

#define R_CURRENT           9
#define CURRENT_POWER_SHIFT 12
#define CURRENT_POWER_MASK  0b1111000000000000
#define CURRENT_GAIN_SHIFT  7
#define CURRENT_GAIN_MASK   0b0000011110000000

/* LT8910S only */
#define R_DATARATE          44
#define DATARATE_MASK       0x00FF
#define DATARATE_1MBPS      0x0100
#define DATARATE_250KBPS    0x0400
#define DATARATE_125KBPS    0x0800
#define DATARATE_62KBPS     0x1000

#define R_SYNCWORD1         36
#define R_SYNCWORD2         37
#define R_SYNCWORD3         38
#define R_SYNCWORD4         39

#define R_PACKETCONFIG      41
#define PACKETCONFIG_CRC_ON             0x8000
#define PACKETCONFIG_SCRAMBLE_ON        0x4000
#define PACKETCONFIG_PACK_LEN_ENABLE    0x2000
#define PACKETCONFIG_FW_TERM_TX         0x1000
#define PACKETCONFIG_AUTO_ACK           0x0800
#define PACKETCONFIG_PKT_FIFO_POLARITY  0x0400
#define CRC_INITIAL_DATA_MASK	0b11111111 //bin
#define DEFAULT_CRC_INITIAL_DATA 0x00

#define R_STATUS            48
#define STATUS_CRC_BIT      15


#define R_FIFO              50
#define R_FIFO_CONTROL      52

LT8900::LT8900(const uint8_t cs, const uint8_t pkt, const uint8_t rst)
{
  _pin_chipselect = cs;
  _pin_pktflag = pkt;
  _pin_reset = rst;
  _channel = DEFAULT_CHANNEL;
  _isLT8910 = false;

  pinMode(_pin_chipselect, OUTPUT);
  pinMode(_pin_pktflag, INPUT);

  digitalWrite(_pin_chipselect, HIGH);
}

void LT8900::begin()
{
	if(_pin_reset > 0)
	{
		digitalWrite(_pin_reset, LOW);
		delay(200);
		digitalWrite(_pin_reset, HIGH);
		delay(200);

	}
	// SETUP DEFAULT DATA
	writeRegister(0, 0x6fe0);		//	Default 0x6fe0	/	Power Up 0x6fef
	writeRegister(1, 0x5681);		//	Default 0x5681	/	Power Up 0x5681
	writeRegister(2, 0x6617);		//	Default 0x6617	/	Power Up 0x6619
	writeRegister(4, 0x9cc9);    	//	Default 0x9cc9	/	Power Up 0x5447
	writeRegister(5, 0x6637);    	//	Default 0x6637	/	Power Up 0xf000
	//writeRegister(6, 0x0000);    		//	UNDOCUMENTED
	writeRegister(7, 0x010A);    	//	Default 0x0030	/	Power Up 0x0030
	writeRegister(8, 0x6c90);    	//	Default 0x6c90	/	Power Up 0x71af		//power (default 71af) UNDOCUMENTED
	writeRegister(9, 0x4800);    	//	Default 0x1840	/	Power Up 0x3000
	writeRegister(10, 0x7ffd);   	//	Default 0x7ffd	/	Power Up 0x7ffd		//bit 0: XTAL OSC enable
	writeRegister(11, 0x0008);   	//	Default 0x0008	/	Power Up 0x4008		//bit 8: Power down RSSI (0=  RSSI operates normal)
	//writeRegister(12, 0x0000);		//	UNDOCUMENTED
	writeRegister(13, 0x48bd);   	//	Default 0x48bd	/	Power Up 0x4855
	//writeRegister(14, 0x0000);		//	UNDOCUMENTED
	//writeRegister(15, 0x0000);		//	UNDOCUMENTED
	//writeRegister(16, 0x0000);		//	UNDOCUMENTED
	//writeRegister(17, 0x0000);		//	UNDOCUMENTED
	//writeRegister(18, 0x0000);		//	UNDOCUMENTED
	//writeRegister(19, 0x0000);		//	UNDOCUMENTED
	//writeRegister(20, 0x0000);		//	UNDOCUMENTED
	//writeRegister(21, 0x0000);		//	UNDOCUMENTED
	writeRegister(22, 0x00ff);		//	Default 0x00ff	/	Power Up 0xc0ff
	writeRegister(23, 0x8005);  	//	Default 0x8005	/	Power Up 0x8005		//bit 2: Calibrate VCO before each Rx/Tx enable
	writeRegister(24, 0x0067);		//	Default 0x0067	/	Power Up 0x307b
	writeRegister(25, 0x1659);		//	Default 0x1659	/	Power Up 0x1659
	writeRegister(26, 0x19e0);		//	Default 0x19e0	/	Power Up 0x1833
	writeRegister(27, 0x1300);  	//	Default 0x1200	/	Power Up 0x9100		//bits 5:0, Crystal Frequency adjust
	writeRegister(28, 0x1800);		//	Default 0x1800	/	Power Up 0x1800
	//writeRegister(29, 0x0000);		//	Read Only
	//writeRegister(30, 0x0000);		//	Read Only
	//writeRegister(31, 0x0000);		//	Read Only
	writeRegister(32, 0x4800);  	//	Default 0x1806	/	Power Up 0x1806 	//AAABBCCCDDEEFFFG  A preamble length, B syncword length, C trailer length, D packet type, E FEC_type, F BRCLK_SEL, G reserved //0x5000 = 0101 0000 0000 0000 = preamble 010 (3 bytes), B 10 (48 bits)
	writeRegister(33, 0x3fc7);		//	Default 0x3ff0	/	Power Up 0x6307
	writeRegister(34, 0x2000);		//	Default 0x3000	/	Power Up 0x030b
	writeRegister(35, 0x0500);		//	Default 0x0380	/	Power Up 0x1300  	//POWER mode,  bit 8/9 on = retransmit = 3x (default)
	writeRegister(36, 0x0380);		//	Default 0x0000	/	Power Up 0x0000
	writeRegister(37, 0x0380);		//	Default 0x0000	/	Power Up 0x0000
	writeRegister(38, 0x5A5A);		//	Default 0x0000	/	Power Up 0x0000
	writeRegister(39, 0x0380);		//	Default 0x0000	/	Power Up 0x0000
	writeRegister(40, 0x2102); 		//	Default 0x2107	/	Power Up 0x2107		//max allowed error bits = 0 (01 = 0 error bits)
	writeRegister(41, 0xb000);		//	Default 0xb000	/	Power Up 0xb800
	writeRegister(42, 0xfdb0);		//	Default 0xfdb0	/	Power Up 0xfd6b
	writeRegister(43, 0x000f);		//	Default 0x000f	/	Power Up 0x000f
	writeRegister(44, 0x1000);		//	Default 0x1000	/	Power Up 0x1000
	writeRegister(45, 0x0552);		//	Default 0x0552	/	Power Up 0x0552
	//writeRegister(46, 0x0000);		//	UNDOCUMENTED
	//writeRegister(47, 0x0000);		//	UNDOCUMENTED
	writeRegister(48, 0x0000);		//	Read Only
	//writeRegister(49, 0x0000);		//	UNDOCUMENTED
	writeRegister(50, 0x000f);		//	Default 0x000f	/	Power Up 0x000f
	//writeRegister(51, 0x0000);		//	UNDOCUMENTED
	writeRegister(52, 0x8080);		//	Default 0x8080	/	Power Up 0x8080

	writeRegister
	(R_PACKETCONFIG,
	  PACKETCONFIG_CRC_ON |
	  PACKETCONFIG_PACK_LEN_ENABLE |
	  PACKETCONFIG_FW_TERM_TX
	);

	//check the variant.
	_isLT8910 = setDataRate(LT8910_62KBPS);
	//return to defaults.
	setDataRate(LT8900_1MBPS);

	//15:8, 01: 1Mbps 04: 250Kbps 08: 125Kbps 10: 62.5Kbps

	writeRegister(R_FIFO, 0x0000);  //TXRX_FIFO_REG (FIFO queue)

	writeRegister(R_FIFO_CONTROL, 0x8080); //Fifo Rx/Tx queue reset

	delay(200);
	writeRegister(R_CHANNEL, _BV(CHANNEL_TX_BIT));  //set TX mode.  (TX = bit 8, RX = bit 7, so RX would be 0x0080)
	delay(2);
	writeRegister(R_CHANNEL, _channel);  // Frequency = 2402 + channel
}

void LT8900::setChannel(uint8_t channel)
{
  _channel = channel;
  writeRegister(R_CHANNEL,  (_channel & CHANNEL_MASK));
}

uint8_t LT8900::getChannel()
{
  return _channel;
}

bool LT8900::getIs8910()
{
  return _isLT8910;
}

void LT8900::setCurrentControl(uint8_t power, uint8_t gain)
{
  writeRegister(R_CURRENT,
                ((power << CURRENT_POWER_SHIFT) & CURRENT_POWER_MASK) |
                ((gain << CURRENT_GAIN_SHIFT) & CURRENT_GAIN_MASK));
}

bool LT8900::setDataRate(DataRate rate)
{
  uint16_t newValue;

  switch (rate)
  {
    case LT8900_1MBPS:
      newValue = DATARATE_1MBPS;
      break;
    case LT8910_250KBPS:
      newValue = DATARATE_250KBPS;
      break;
    case LT8910_125KBPS:
      newValue = DATARATE_125KBPS;
      break;
    case LT8910_62KBPS:
      newValue = DATARATE_62KBPS;
      break;
    default:
      return false;
  }

  writeRegister(R_DATARATE, newValue);

  //verify
  return (readRegister(R_DATARATE) == newValue);
}

LT8900::DataRate LT8900::getDataRate()
{
  uint16_t value = readRegister(R_DATARATE) & DATARATE_MASK;
  switch (value)
  {
    case DATARATE_1MBPS:
      return LT8900_1MBPS;
    case DATARATE_250KBPS:
      return LT8910_250KBPS;
    case DATARATE_125KBPS:
      return LT8910_125KBPS;
    case DATARATE_62KBPS:
      return LT8910_62KBPS;
  }
}

uint16_t LT8900::readRegister(uint8_t reg)
{
  digitalWrite(_pin_chipselect, LOW);
  SPI.transfer(REGISTER_READ | (REGISTER_MASK & reg));
  uint8_t high = SPI.transfer(0x00);
  uint8_t low = SPI.transfer(0x00);

  digitalWrite(_pin_chipselect, HIGH);

   //Serial.print("READ REGISTER: ");
   //Serial.print(reg);
   //Serial.print(" = ");
   //Serial.println(high << 8 | low);

  return (high << 8 | low);
}

uint8_t LT8900::writeRegister(uint8_t reg, uint16_t data)
{
  uint8_t high = data >> 8;
  uint8_t low = data & 0xFF;

  return writeRegister2(reg, high, low);
}

uint8_t LT8900::writeRegister2(uint8_t reg, uint8_t high, uint8_t low)
{

  //char sbuf[32];
  //sprintf_P(sbuf, PSTR("WRITE REGISTER: %d => %02x%02x"), reg, high, low);
  //Serial.println(sbuf);


  digitalWrite(_pin_chipselect, LOW);
  uint8_t result = SPI.transfer(REGISTER_WRITE | (REGISTER_MASK & reg));
  SPI.transfer(high);
  SPI.transfer(low);

  digitalWrite(_pin_chipselect, HIGH);
  return result;
}

void LT8900::sleep()
{
  //set bit 14 on register 35.
  writeRegister(35, readRegister(35) | _BV(14));
}

void LT8900::whatsUp(Stream &stream)
{
  uint16_t mode = readRegister(R_CHANNEL);
  stream.print("\nTx_EN=");
  stream.println((mode & _BV(CHANNEL_TX_BIT)) != false);
  stream.print("Rx_En=");
  stream.println((mode & _BV(CHANNEL_RX_BIT)) != false);
  stream.print("Channel=");
  stream.println(mode & CHANNEL_MASK);

  //read the status register.
  uint16_t state = readRegister(R_STATUS);

  bool crc_error = state & _BV(15);
  bool fec23_error = state & _BV(14);
  uint8_t framer_st = (state & 0b0011111100000000) >> 8;
  bool pkt_flag = state & _BV(6);
  bool fifo_flag = state & _BV(5);

  stream.print("CRC=");
  stream.println(crc_error);
  stream.print("FEC=");
  stream.println(fec23_error);
  stream.print("FRAMER_ST=");
  stream.println(framer_st);
  stream.print("PKT=");
  stream.println(pkt_flag);
  stream.print("FIFO=");
  stream.println(fifo_flag);

  uint16_t fifo = readRegister(R_FIFO_CONTROL);
  stream.print("FIFO_WR_PTR=");
  stream.println((fifo >> 8) & 0b111111);
  stream.print("FIFO_RD_PTR=");
  stream.println(fifo & 0b111111);
}

bool LT8900::available()
{
  //read the PKT_FLAG state; this can also be done with a hard wire.

  if (digitalRead(_pin_pktflag) != 0)
  {
    return true;
  }

  return false;
}

int LT8900::read(uint8_t *buffer, size_t maxBuffer)
{
  uint16_t value = readRegister(R_STATUS);
  u8 result = (bitRead(value, STATUS_CRC_BIT) == 0);

    //CRC ok

    uint16_t data;  /*= readRegister(R_FIFO);
    uint8_t packetSize = data >> 8;
    if(maxBuffer < packetSize+1)
    {
        //BUFFER TOO SMALL
        return -2;
    }*/

    uint8_t pos=0;
    //buffer[pos++] = (data & 0xFF);
    while (pos < maxBuffer /*packetSize*/)
    {
      data = readRegister(R_FIFO);
      buffer[pos++] = data >> 8;
      buffer[pos++] = data & 0xFF;
    }

    return 1; //result ? packetSize : -1;
}

void LT8900::startListening()
{
  writeRegister(R_CHANNEL, _channel & CHANNEL_MASK);   //turn off rx/tx
  delay(3);
  writeRegister(R_FIFO_CONTROL, 0x0080);  //flush rx
  writeRegister(R_CHANNEL,  (_channel & CHANNEL_MASK) | _BV(CHANNEL_RX_BIT));   //enable RX
  delay(5);
}

/* set the BRCLK_SEL value */
void LT8900::setClock(uint8_t clock)
{
  //register 32, bits 3:1.
  uint16_t val = readRegister(35);
  val &= 0b1111111111110001;
  val |= ((clock & 0x07) << 1);;

  writeRegister(35, val);
}

bool LT8900::sendPacket(uint8_t *data, size_t packetSize)
{
  if (packetSize < 1 || packetSize > 255)
  {
    return false;
  }

  writeRegister(R_CHANNEL, 0x0000);
  writeRegister(R_FIFO_CONTROL, 0x8000);  //flush tx

  //packets are sent in 16bit words, and the first word will be the packet size.
  //start spitting out words until we are done.

  uint8_t pos = 0;
  writeRegister2(R_FIFO, packetSize, data[pos++]);
  while (pos < packetSize)
  {
    uint8_t msb = data[pos++];
    uint8_t lsb = data[pos++];

    writeRegister2(R_FIFO, msb, lsb);
  }

  writeRegister(R_CHANNEL,  (_channel & CHANNEL_MASK) | _BV(CHANNEL_TX_BIT));   //enable TX

  //Wait until the packet is sent.
  while (digitalRead(_pin_pktflag) == 0)
  {
      //do nothing.
  }

  return true;
}

void LT8900::setSyncWord(uint64_t syncWord)
{
  writeRegister(R_SYNCWORD1, syncWord);
  writeRegister(R_SYNCWORD2, syncWord >> 16);
  writeRegister(R_SYNCWORD3, syncWord >> 32);
  writeRegister(R_SYNCWORD4, syncWord >> 48);
}

//REGISTER 32 PREAMBLE_LEN 15:13
void LT8900::setPreambleLength(uint8_t option)
{
	uint16_t _option = 0b0100000000000000;
	switch (option)
	{
		case 1:
			_option = 0b0000000000000000;
			break;
		case 2:
			_option = 0b0010000000000000;
			break;
		case 3:
			_option = 0b0100000000000000;
			break;
		case 4:
			_option = 0b0110000000000000;
			break;
		case 5:
			_option = 0b1000000000000000;
			break;
		case 6:
			_option = 0b1010000000000000;
			break;
		case 7:
			_option = 0b1100000000000000;
			break;
		case 8:
			_option = 0b1110000000000000;
			break;
		default:
			_option = 0b0100000000000000;
			break;
	}
	writeRegister(32, (readRegister(32) & ~0b1110000000000000) | _option);
}

//REGISTER 32 SYNCWORD_LEN 12:11
void LT8900::setSyncWordLength(uint8_t option)
{
	uint16_t _option = 0b0001100000000000;
	switch (option)
	{
		case 16:
			_option = 0b0000000000000000;
			break;
		case 32:
			_option = 0b0000100000000000;
			break;
		case 48:
			_option = 0b0001000000000000;
			break;
		case 64:
			_option = 0b0001100000000000;
			break;
		default:
			_option = 0b0001100000000000;
			break;
	}
	writeRegister(32, (readRegister(32) & ~0b0001100000000000) | _option);
}

//REGISTER 32 TRAILER_LEN 10:8
void LT8900::setTrailerLength(uint8_t option)
{
	uint16_t _option = 0b0000011100000000;
	switch (option)
	{
		case 4:
			_option = 0b0000000000000000;
			break;
		case 6:
			_option = 0b0000000100000000;
			break;
		case 8:
			_option = 0b0000001000000000;
			break;
		case 10:
			_option = 0b0000001100000000;
			break;
		case 12:
			_option = 0b0000010000000000;
			break;
		case 14:
			_option = 0b0000010100000000;
			break;
		case 16:
			_option = 0b0000011000000000;
			break;
		case 18:
			_option = 0b0000011100000000;
			break;
		default:
			_option = 0b0000000000000000;
			break;
	}
	writeRegister(32, (readRegister(32) & ~0b0000011100000000) | _option);
}

//REFISTER 32 DATA_PACKET_TYPE 7:6

void LT8900::setDataPacketType(uint8_t type)
{
	uint16_t _type = 0b0000000011000000;
	switch (type)
	{
		case 0:
			_type = 0b0000000000000000;
			break;
		case 1:
			_type = 0b0000000001000000;
			break;
		case 2:
			_type = 0b0000000010000000;
			break;
		case 3:
			_type = 0b0000000011000000;
			break;
		default:
			_type = 0b0000000000000000;
			break;
	}
	writeRegister(32, (readRegister(32) & ~0b0000000011000000) | _type);
}

//REFISTER 32 FEC_TYPE 5:4

void LT8900::setFECType(uint8_t type)
{
	uint16_t _type = 0b0000000000110000;
	switch (type)
	{
		case 0:
			_type = 0b0000000000000000;
			break;
		case 1:
			_type = 0b0000000000010000;
			break;
		case 2:
			_type = 0b0000000000100000;
			break;
		default:
			_type = 0b0000000000000000;
			break;
	}
	writeRegister(32, (readRegister(32) & ~0b0000000000110000) | _type);
}

//REFISTER 41 CRC 15

void LT8900::setCRC(uint8_t type)
{
	uint16_t _type = 0b1000000000000000;
	switch (type)
	{
		case 0:
			_type = 0b0000000000000000;
			break;
		case 1:
			_type = 0b1000000000000000;
			break;
		default:
			_type = 0b1000000000000000;
			break;
	}
	writeRegister(R_PACKETCONFIG, (readRegister(R_PACKETCONFIG) & ~0b1000000000000000) | _type);
}

void LT8900::setScramble(uint8_t type)
{
	uint16_t _type = 0b0100000000000000;
	switch (type)
	{
		case 0:
			_type = 0b0000000000000000;
			break;
		case 1:
			_type = 0b0100000000000000;
			break;
		default:
			_type = 0b0000000000000000;
			break;
	}
	writeRegister(R_PACKETCONFIG, (readRegister(R_PACKETCONFIG) & ~0b0100000000000000) | _type);
}

void LT8900::setPackLengthEn(uint8_t type)
{
	uint16_t _type = 0b0010000000000000;
	switch (type)
	{
		case 0:
			_type = 0b0000000000000000;
			break;
		case 1:
			_type = 0b0010000000000000;
			break;
		default:
			_type = 0b0010000000000000;
			break;
	}
	writeRegister(R_PACKETCONFIG, (readRegister(R_PACKETCONFIG) & ~0b0010000000000000) | _type);
}

void LT8900::setFwTermTx(uint8_t type)
{
	uint16_t _type = 0b0001000000000000;
	switch (type)
	{
		case 0:
			_type = 0b0000000000000000;
			break;
		case 1:
			_type = 0b0001000000000000;
			break;
		default:
			_type = 0b0001000000000000;
			break;
	}
	writeRegister(R_PACKETCONFIG, (readRegister(R_PACKETCONFIG) & ~0b0001000000000000) | _type);
}

void LT8900::setAutoACK(uint8_t type)
{
	uint16_t _type = 0b0000100000000000;
	switch (type)
	{
		case 0:
			_type = 0b0000000000000000;
			break;
		case 1:
			_type = 0b0000100000000000;
			break;
		default:
			_type = 0b0000100000000000;
			break;
	}
	writeRegister(R_PACKETCONFIG, (readRegister(R_PACKETCONFIG) & ~0b0000100000000000) | _type);
}

void LT8900::setPKTFIFOPolarity(uint8_t type)
{
	uint16_t _type = 0b0000010000000000;
	switch (type)
	{
		case 0:
			_type = 0b0000000000000000;
			break;
		case 1:
			_type = 0b0000010000000000;
			break;
		default:
			_type = 0b0000000000000000;
			break;
	}
	writeRegister(R_PACKETCONFIG, (readRegister(R_PACKETCONFIG) & ~0b0000010000000000) | _type);
}

void LT8900::setCRCInitialData(uint8_t data)
{
  _data = data;
  writeRegister(R_PACKETCONFIG,  (readRegister(R_PACKETCONFIG) & ~0b0000000011111111) | (_data & CRC_INITIAL_DATA_MASK));
}

//REGISTER 35 BIT 15
void LT8900::setPowerDown(uint8_t data)
{
	uint16_t _data = 0b0000000000000000;
	switch (data)
	{
		case 0:
			_data = 0b0000000000000000;
			break;
		case 1:
			_data = 0b1000000000000000;
			break;
		default:
			_data = 0b0000000000000000;
			break;
	}
	writeRegister(35, (readRegister(35) & ~0b1000000000000000) | _data);
}

//REGISTER 35 BIT 14
void LT8900::setSleepMode(uint8_t data)
{
	uint16_t _data = 0b0000000000000000;
	switch (data)
	{
		case 0:
			_data = 0b0000000000000000;
			break;
		case 1:
			_data = 0b0100000000000000;
			break;
		default:
			_data = 0b0000000000000000;
			break;
	}
	writeRegister(35, (readRegister(35) & ~0b0100000000000000) | _data);
}

//REGISTER 35 BIT 12
void LT8900::setBRCLKOnSleep(uint8_t data)
{
	uint16_t _data = 0b0000000000000000;
	switch (data)
	{
		case 0:
			_data = 0b0000000000000000;
			break;
		case 1:
			_data = 0b0001000000000000;
			break;
		default:
			_data = 0b0000000000000000;
			break;
	}
	writeRegister(35, (readRegister(35) & ~0b0001000000000000) | _data);
}

//REGISTER 35 BITS 11:8
void LT8900::setReTransmitTimes(uint8_t data)
{
	uint16_t _data = data & 0x0F;
	_data <<= 8;

	writeRegister(35, (readRegister(35) & ~0b0000111100000000) | _data);
}

//REGISTER 35 BITS 6:0
void LT8900::setScrambleData(uint8_t data)
{
  uint8_t _data = data;
  writeRegister(35,  (readRegister(35) & ~0b0000000001111111) | (_data & 0b01111111));
}

//REGISTER 40 SYNCWORD_THRESHOLD 5:0
void LT8900::setSyncwordThreshold(uint8_t option)
{
	uint16_t _option = option;
	writeRegister(40,  (readRegister(40) & ~0b0000000000111111) | (_option & 0b00111111));	
}

uint8_t LT8900::getRSSI()
{
    //RSSI: 15:10
    uint16_t value = readRegister(6);

    return (value >> 10);
}

void LT8900::scanRSSI(uint16_t *buffer, uint8_t start_channel, uint8_t num_channels)
{
    // writeRegister(R_CHANNEL, _BV(CHANNEL_RX_BIT));
    //
    // //add read mode.
    writeRegister(R_FIFO_CONTROL, 0x8080);  //flush rx
    // writeRegister(R_CHANNEL, 0x0000);

    //set number of channels to scan.
    writeRegister(42, (readRegister(42) & 0b0000001111111111) | ((num_channels-1 & 0b111111) << 10));

    //set channel scan offset.
    writeRegister(43, (readRegister(43) & 0b0000000011111111) | ((start_channel & 0b1111111) << 8));
    writeRegister(43, (readRegister(43) & 0b0111111111111111) | _BV(15));

    while(digitalRead(_pin_pktflag) == 0)
    {
    }


    //read the results.
     uint8_t pos = 0;
    while(pos < num_channels)
    {
      uint16_t data = readRegister(R_FIFO);
      buffer[pos++] = data >> 8;
    }
}
