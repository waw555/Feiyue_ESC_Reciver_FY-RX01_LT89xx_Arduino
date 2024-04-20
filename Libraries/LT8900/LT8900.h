/*
 Copyright (C) 2015 Rob van der Veer, <rob.c.veer@gmail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#ifndef LT8900_H
#define LT8900_H

class LT8900
{

  public:
    /** Data Data
    * While the LT8900 only has support for 1MPBS speed, the LT8910 offers different speeds.
    * @see getIs8910
    */
    enum DataRate
    {
      LT8900_1MBPS,     /** default transmit rate */
      LT8910_250KBPS,   /** 250 Kpbs, only on lt8910 */
      LT8910_125KBPS,   /** 125 Kbps, only on lt8910 */
      LT8910_62KBPS     /** 62 Kbps, only on lt8910 */
    };

  private:
    uint8_t _pin_chipselect;
    uint8_t _pin_pktflag;
    uint8_t _pin_reset;
    uint8_t _channel;
	uint8_t _data;
    bool _isLT8910;

  public:
    /** Construct a new instance
     * @param cs Chip Select, this is the SLAVE SELECT pin. Please note that it is a different location than the NRF24L01+ SS pin.
     * @param pkt PKT_flag input signal pin. Comparable to the NRF24L01+ IRQ pin.
     * @param rst RESET pin. Use 0 to disable.
    */
    LT8900(const uint8_t cs, const uint8_t pkt, const uint8_t rst);

    /** Configures the module for initial use */
    void begin();

    /** 设置信道
    * @param channel has significant 7 bits
    */
    void setChannel(uint8_t channel);
    /** Retrieve the current channel */
    uint8_t getChannel();

    /** Set power and gain
    * @param power 0-0xf
    * @param gain 0-0xf
    */
    void setCurrentControl(uint8_t power, uint8_t gain);

    /** Sets the data rate
    * @param rate the transmission/reception speed
    * @returns true when the data rate was succesfully changed.
    */
    bool setDataRate(DataRate rate);

    /** Returns the data rate
    * @returns the active data rate.
    */
    DataRate getDataRate();

    /** Read the value of a register
    * @param reg
    */
    uint16_t readRegister(uint8_t reg);

    /** Writes to a register
    * @param reg The register to write to
    * @param data 16bits of data, MSB first */
    uint8_t writeRegister(uint8_t reg, uint16_t data);

    /** Writes to a register, one byte at a time.
    * This is a convenience function, because the LT8900 uses 16 bits registers with all 16 bits written at once.
    * @param reg The register to write to
    * @param high bits 15..8
    * @param low bits 7..0  */
    uint8_t writeRegister2(uint8_t reg, uint8_t high, uint8_t low);

    /** Put the LT8900 module to sleep mode.
     * In contrast to POWER DOWN, this mode will keep the register values
     * Any SPI call will awaken the module automatically */
    void sleep();

    /** Dumps debug information on the selected port
    * @param stream the output stream to use as output, eg. `whatsUp(&Serial);`
    */
    void whatsUp(Stream &stream);

    /** Signals a packet ready in the FIFO queue */
    bool available();

    /** Checks the module type
    * The LT8910 has a few extra features one of it being the extended options for data rate.
    * @returns false for a regular LT8900 and true for a LT8910
    * @see setDataRate
    */
    bool getIs8910();

    /** Read a packet into the buffer
    * @param buffer a pointer to a buffer large enough to hold the packet
    * @param maxBuffer the maximum size of the buffer. Any bytes left over in the buffer will be dropped.
    * @returns the size of the packet that was read, -1 for CRC error
    */
    int read(uint8_t *buffer, size_t maxBuffer);

    /** Switch the module to RX mode */
    void startListening();

    /** Sets the internal clock divider
   * @param clock
   */
    void setClock(uint8_t clock);

    /** Sends a packet
    * This call blocks until the packet was sent and the Tx Buffer has been completed
    * @param data
    * @param packetSize
    * @returns true when the packet was sent, or false when the packetSize was invalid.
    */
    bool sendPacket(uint8_t *data, size_t packetSize);

    /** Set Syncword
    * @param syncWord 64 bits of sync word settings.
    * @see setSyncwordLength
    */
    void setSyncWord(uint64_t syncWord);
	

    /** Sets the length of the sync word used
     * @param length:
     *          64 = 64 bits, 48 = 48 bits, 32 = 32 bits, 16 = 16 bits
     * @see Check the datasheet for which bits are actually used.
     */
    void setSyncWordLength(uint8_t length);
	
	/** Set the Preamble Lenght
    * @param lenght:
		1 = 1 byte, 2 = 2 byte, 3 = 3 byte .... 8 = 8 byte
    * @see Check the datasheet for which bits are actually used.
    */
    void setPreambleLength(uint8_t length);
	
	/** Set the Trailer Lenght
    * @param lenght:
		4 = 4 bits, 6 = 6 bits, 8 = 8 bits .... 18 = 18 bits
    * @see Check the datasheet for which bits are actually used.
    */
	void setTrailerLength(uint8_t length);
    
	/** Set the Data Packet Type
    * @param type:
		0 = NRZ law data, 1 = Manchester data type, 2 = 8bit/10bit line code, 3 = Interleave data type
    * @see Check the datasheet for which bits are actually used.
    */
	void setDataPacketType(uint8_t type);
	
	/** Set the FEC Type
    * @param type:
		0 = No FEC, 1 = FEC13, 2 = FEC23
    * @see Check the datasheet for which bits are actually used.
    */
	void setFECType(uint8_t type);
	
	/** Set the CRC
    * @param type:
		0 = CRC OFF, 1 = CRC ON
    * @see Check the datasheet for which bits are actually used.
    */
	void setCRC(uint8_t type);
	
	/** Set the Scramble
    * @param type:
		0 = Scramble OFF, 1 = Scramble ON
    * @see Check the datasheet for which bits are actually used.
    */
	void setScramble(uint8_t type);
	
	/** Set the PACK_LENGTH_EN
    * @param type:
		0 = First byte Pack Length Enable OFF, 1 = First byte Pack Length Enable ON
    * @see Check the datasheet for which bits are actually used.
    */
	void setPackLengthEn(uint8_t type);
	
	/** Set the FW_TERM_TX
    * @param type:
		0 = FW (MCU) handles length and terminates TX., 1 = When FIFO write point equals read point, LT8900 will terminate TX when FW handle packet length.
    * @see Check the datasheet for which bits are actually used.
    */
	void setFwTermTx(uint8_t type);
	
	/** Set the Register 41 AUTO_ACK
    * @param type:
		0 = After receive, do not send ACK or NACK; just go to IDLE., 1 = After receiving data, automatically send ACK/NACK.
    * @see Check the datasheet for which bits are actually used.
    */
	void setAutoACK(uint8_t type);
	
	/** Set the Register 41 PKT_FIFO_POLARITY
    * @param type:
		0 = Active high, 1 = PKT flag, FIFO flag Active low. (Default = 0)
    * @see Check the datasheet for which bits are actually used.
    */
	void setPKTFIFOPolarity(uint8_t data);
	
	/** Set the Register 41 CRC_INITIAL_DATA 7:0
    * @param type:
		Initialization constant for CRC calculation. (Default = 0x00)
    * @see Check the datasheet for which bits are actually used.
    */
	void setCRCInitialData(uint8_t data);
	
	/** Set the Register 35 SCRAMBLE_DATA 6:0
    * @param type:
		Whitening seed for data scramble. Must be set the same at both ends of radio link (Tx and Rx). (Default = 0x00)
    * @see Check the datasheet for which bits are actually used.
    */
	void setScrambleData(uint8_t data);
	
	/** Set the Register 35 POWER_DOWN 15
    * @param type:
		0: Leave power on. 1: First set crystal off, then set LDO low-power mode (register values will be lost).
    * @see Check the datasheet for which bits are actually used.
    */
	void setPowerDown(uint8_t data);
	
	/** Set the Register 35 SLEEP_MODE 14
    * @param type:
		0: Normal (IDLE) state. 1: Enter SLEEP state (set crystal gain block to off. Keep LDO regulator on (register values will be preserved). Wakeup begins when SPI_SS goes low. This will restart the on-chip clock oscillator to begin normal operation.
    * @see Check the datasheet for which bits are actually used.
    */
	void setSleepMode(uint8_t data);
	
	/** Set the Register 35 BRCLK_ON_SLEEP 12
    * @param type:
		0: crystal stops during sleep mode. Saves current but takes longer to wake up. 1: crystal running at sleep mode. Draws more current but enables fast wakeup.
    * @see Check the datasheet for which bits are actually used.
    */
	void setBRCLKOnSleep(uint8_t data);
	
	/** Set the Register 35 RE-TRANSMIT_TIMES 11:8
    * @param type:
		Max. re-transmit packet attempts when auto_ack function is enabled. (Default = 3)
    * @see Check the datasheet for which bits are actually used.
    */
	void setReTransmitTimes(uint8_t data);
	
	/** Set the Register 40 SYNCWORD_THRESHOLD 5:0
    * @param type:
		The minimum allowable error bits of SYNCWORD, 6 = 6 bits，0 = 0 bit (Default = 6)
    * @see Check the datasheet for which bits are actually used.
    */
	void setSyncwordThreshold(uint8_t data);
    

    /** Scan the signal strength for one or more channels
    * @param buffer points to a buffer to store the signal strengths, at least num_channels
    * @param start_channel starting channel to scan, where Frequency = 2402 + channel
    * @param num_channels number of channels to scan in this batch, for example scanning 4 channels
    *                     with start 2480 will scan 2480, 2481, 2482, 2483
    */
    void scanRSSI(uint16_t *buffer, uint8_t start_channel, uint8_t num_channels);

    /** retrieve the analog signal strength for the current channel */
    uint8_t getRSSI();
};


#endif //LT8900_H
