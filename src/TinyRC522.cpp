/*****************************************************/
/*                                                   */
/*                     TinyRC522                     */
/*      (c) Yvan Régeard - All rights reserved       */
/*                                                   */
/* Licensed under the MIT license. See LICENSE file  */
/* in the project root for full license information. */
/*                                                   */
/*****************************************************/



// Platform specific: ESP8266
#ifdef ESP8266

  // Includes
  #include <Arduino.h>
  #include <SPI.h>
  #include "TinyRC522.h"

#endif

// Platform specific: Raspberry pi
#ifdef __arm__

  // Types
  typedef unsigned char uint8_t;
  typedef unsigned short uint16_t;
  typedef unsigned int uint32_t;

  // Includes
  #include <stdio.h>
  #include <string.h>
  #include <wiringPi.h>
  #include <wiringPiSPI.h>
  #include "TinyRC522.h"

#endif



// Constants

  // Block size
  #define RC522_BLOCK_SIZE          18             // 16 bytes block + 2 bytes CRC_A

  // RC522 registers
  #define CommandReg                (0x01<<1)
  #define ComIrqReg                 (0x04<<1)
  #define ErrorReg                  (0x06<<1)
  #define Status2Reg                (0x08<<1)
  #define FIFODataReg               (0x09<<1)
  #define FIFOLevelReg              (0x0A<<1)
  #define ControlReg                (0x0C<<1)
  #define BitFramingReg             (0x0D<<1)
  #define CollReg                   (0x0E<<1)
  #define ModeReg                   (0x11<<1)
  #define TxModeReg                 (0x12<<1)
  #define RxModeReg                 (0x13<<1)
  #define TxControlReg              (0x14<<1)
  #define TxASKReg                  (0x15<<1)
  #define RFCfgReg                  (0x26<<1)
  #define TModeReg                  (0x2A<<1)
  #define TPrescalerReg             (0x2B<<1)
  #define TReloadRegH               (0x2C<<1)
  #define TReloadRegL               (0x2D<<1)
  #define VersionReg                (0x37<<1)



// Tag public functions

  // Constructor
  Tag::Tag(RC522* p_rc522):
    m_p_rc522(p_rc522)
  {
  }



  // Get UID
  uint32_t Tag::get_uid()
  {
    // Return UID
    return (m_uid[3]<<24)|(m_uid[2]<<16)|(m_uid[1]<<8)|(m_uid[0]<<0);
  }



  // Set protection
  bool Tag::set_protection(uint8_t* new_key,uint8_t* key)
  {
    // Set default key
    uint8_t factory_key[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    if (!new_key) new_key=factory_key;
    if (!key) key=factory_key;

    // Set protection key on all trailer blocks (key A and key B)
    for(uint8_t block_address=3; block_address<64; block_address+=4)
    {
      // Authenticate sector
      uint8_t auth_command[12]={0x60,block_address,key[0],key[1],key[2],key[3],key[4],key[5],m_uid[0],m_uid[1],m_uid[2],m_uid[3]};
      if (!m_p_rc522->authenticate(auth_command,12)) return false;

      // Send write command
      uint8_t write_command[4]={0xA0,block_address,0x00,0x00};
      if (!m_p_rc522->write(write_command,4)) return false;

      // Send data block to write
      uint8_t trailer_block[RC522_BLOCK_SIZE]={new_key[0],new_key[1],new_key[2],new_key[3],new_key[4],new_key[5],0xFF,0x07,0x80,0x69,new_key[0],new_key[1],new_key[2],new_key[3],new_key[4],new_key[5],0x00,0x00};
      if (!m_p_rc522->write(trailer_block,RC522_BLOCK_SIZE)) return false;
    }

    // Return
    return true;
  }



  // Remove protection
  bool Tag::remove_protection(uint8_t* key)
  {
    // Remove protection by setting a protection with the factory key
    return set_protection(NULL,key);
  }



  // Read data
  bool Tag::read(uint8_t* buffer,uint16_t buffer_size,uint8_t* key)
  {
    // Exit on wrong buffer
    if ((!buffer)||(buffer_size==0)||(buffer_size>TAG_MAX_SIZE)||(buffer_size%16)) return false;

    // Set default key
    uint8_t factory_key[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    if (key==NULL) key=factory_key;

    // Read data blocks
    uint16_t buffer_index=0;
    bool sector_authenticated=false;
    for(uint8_t block_address=1; block_address<64; block_address++)
    {
      // Clear authentication flag on each sector trailer
      if ((block_address%4)==3) sector_authenticated=false;
      else
      {
        // Authenticate sector
        if (!sector_authenticated)
        {
          uint8_t auth_command[12]={0x60,(uint8_t)((block_address/4)*4+3),key[0],key[1],key[2],key[3],key[4],key[5],m_uid[0],m_uid[1],m_uid[2],m_uid[3]};
          if (!m_p_rc522->authenticate(auth_command,12)) return false;
          sector_authenticated=true;
        }

        // Send read command
        uint8_t read_command[4]={0x30,block_address,0x00,0x00};
        if (!m_p_rc522->write(read_command,4)) return false;

        // Get block
        uint8_t block_buffer[RC522_BLOCK_SIZE];
        uint8_t block_buffer_size=RC522_BLOCK_SIZE;
        if (!m_p_rc522->read(block_buffer,block_buffer_size)) return false;

        // Append block to read data
        memcpy(&buffer[buffer_index],block_buffer,16);

        // End of buffer
        buffer_index+=16;
        if (buffer_index>=buffer_size) return true;
      }
    }

    // Return
    return true;
  }



  // Write data
  bool Tag::write(uint8_t* buffer,uint16_t buffer_size,uint8_t* key)
  {
    // Exit on wrong buffer
    if ((!buffer)||(buffer_size==0)||(buffer_size>TAG_MAX_SIZE)||(buffer_size%16)) return false;

    // Set default key
    uint8_t factory_key[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    if (key==NULL) key=factory_key;

    // Write data blocks
    uint16_t buffer_index=0;
    bool sector_authenticated=false;
    for(uint8_t block_address=1; block_address<64; block_address++)
    {
      // Clear authentication flag on each sector trailer
      if ((block_address%4)==3) sector_authenticated=false;
      else
      {
        // Authenticate sector
        if (!sector_authenticated)
        {
          uint8_t auth_command[12]={0x60,(uint8_t)((block_address/4)*4+3),key[0],key[1],key[2],key[3],key[4],key[5],m_uid[0],m_uid[1],m_uid[2],m_uid[3]};
          if (!m_p_rc522->authenticate(auth_command,12)) return false;
          sector_authenticated=true;
        }

        // Send write command
        uint8_t write_command[4]={0xA0,block_address,0x00,0x00};
        if (!m_p_rc522->write(write_command,4)) return false;

        // Extract data block
        uint8_t block_buffer[RC522_BLOCK_SIZE];
        memcpy(block_buffer,&buffer[buffer_index],16);

        // Send data block to write
        if (!m_p_rc522->write(block_buffer,RC522_BLOCK_SIZE)) return false;

        // End of buffer
        buffer_index+=16;
        if (buffer_index>=buffer_size) return true;
      }
    }

    // Return
    return true;
  }



  // Release
  bool Tag::release()
  {
    // Send HALTA command
    uint8_t halta_command[2]={0x50,0x00};
    return m_p_rc522->write(halta_command,sizeof(halta_command),0,false);
  }



// Tag private functions

  // Initialize
  void Tag::initialize()
  {
    // Initialize UID
    for(uint8_t i=0; i<7; i++) m_uid[i]=0x00;
    m_uid_size=0;
  }



  // Add UID byte
  void Tag::add_uid_byte(uint8_t uid_byte)
  {
    // Store UID byte
    m_uid[m_uid_size++]=uid_byte;
  }



// RC522 public functions

  // Initialize
  void RC522::initialize(uint8_t sda_pin,uint32_t frequency)
  {
    // Initialize attributes
    m_sda_pin=sda_pin;
    m_frequency=frequency;

    // Platform specific: Raspberry pi
    #ifdef __arm__

      // Initialize GPIO
      wiringPiSetupGpio();

      // Initialize SPI bus
      if (wiringPiSPISetup(sda_pin,frequency)<0) return;

    #endif

    // Initialize pins
    pinMode(m_sda_pin,OUTPUT);
    digitalWrite(m_sda_pin,HIGH);

    // Soft reset
    spi_write_register(CommandReg,0x0F);

    // Wait for oscillator start-up
    delayMicroseconds(200);

    // Set timer
    spi_write_register(TModeReg,0x80);
    spi_write_register(TPrescalerReg,0xA9);
    spi_write_register(TReloadRegH,0x00);
    spi_write_register(TReloadRegL,0xF0);

    // Force 100% ASK
    spi_write_register(TxASKReg,0x40);

    // Clear bits received after collision
    spi_clear_register_bits(CollReg,0x80);

    // Enable the antenna driver pins TX1 and TX2
    spi_set_register_bits(TxControlReg,0x03);

    // Initialize tag
    m_tag.initialize();
  }



    // Set antenna gain
  void RC522::set_antenna_gain(uint8_t gain)
  {
    // Set antenna gain
    if (gain)
    {
      spi_write_register(RFCfgReg,gain);
      spi_set_register_bits(TxControlReg,0x03);
    }
    else spi_clear_register_bits(TxControlReg,0x03);
  }



  // Get tag
  Tag* RC522::get_tag()
  {
    // Initialize tag
    m_tag.initialize();

    // Clear MIFARE crypto1 unit
    spi_clear_register_bits(Status2Reg,0x08);

    // Send WUPA command (7 bits framing / no CRC control)
    uint8_t wupa_command[1]={0x52};
    if (write(wupa_command,1,7,false))
    {
      // Get ATQA response
      uint8_t atqa[2]={0x00,0x00};
      uint8_t atqa_size=2;
      if (read(atqa,atqa_size,false))
      {
        // Check tag type (ATQA=0x0400: Mifare Classic 1K (4-bytes UID))
        if ((atqa_size==2)&&(atqa[0]==0x04)&&(atqa[1]==0x00))
        {
          // Send Anticollision CL1 command
          uint8_t anticollision_cl1_command[2]={0x93,0x20};
          if (write(anticollision_cl1_command,2,0,false))
          {
            // Get UID + BCC (5 bytes)
            uint8_t uid[5];
            uint8_t uid_size=5;
            if (read(uid,uid_size,false))
            {
              // Check Block Check Character
              if ((uid[0]^uid[1]^uid[2]^uid[3])==uid[4])
              {
                // Store UID bytes received (Cascade Tag is not controlled and considered as UID byte)
                for(uint8_t i=0; i<4; i++) m_tag.add_uid_byte(uid[i]);

                // Send Select CL1 command
                uint8_t select_cl1_command[9]={0x93,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
                memcpy(&select_cl1_command[2],uid,5);
                if (write(select_cl1_command,9))
                {
                  // Get SAK + CRC_A
                  uint8_t sak[3];
                  uint8_t sak_size=3;
                  if (read(sak,sak_size))
                  {
                    // Return tag when SAK is identified (SAK=0x08: Mifare Classic 1K (4-bytes UID))
                   if (sak[0]==0x08) return &m_tag;
                  }
                }
              }
            }
          }
        }
      }
    }

    // Return : error
    return NULL;
  }



  // Get RC522 module version
  uint8_t RC522::get_version()
  {
    // Return version
    return spi_read_register(VersionReg);
  }



// RC522 private functions

  // Authenticate
  bool RC522::authenticate(uint8_t* buffer,uint8_t buffer_size)
  {
    // Exit on wrong buffer
    if ((!buffer)||(buffer_size==0)) return false;

    // Initialize RC522 module
    spi_write_register(CommandReg,0x00);                    // Stop any active command
    spi_write_register(ComIrqReg,0x7F);                     // Clear all IRQs
    spi_write_register(FIFOLevelReg,0x80);                  // Initialize FIFO
    spi_write_register(FIFODataReg,buffer,buffer_size);     // Write data to send in the FIFO
    spi_write_register(BitFramingReg,0x00);                 // Set bit adjustments
    spi_write_register(CommandReg,0x0E);                    // MFAuthent command

    // Wait for the command to complete : 36 ms of timeout
    uint32_t timeout=millis()+36;
    while(true)
    {
      // Check IRQ bits
      uint8_t irq_bits=spi_read_register(ComIrqReg);
      if ((irq_bits&0x03)||(millis()>timeout)) return false;    // Return on error or timeout
      if (irq_bits&0x10) return true;                           // Return on command successfully terminated

      // Delay
      delayMicroseconds(400);
    }

    // Error : unexpected behaviour
    return false;
  }



  // Write data
  bool RC522::write(uint8_t* buffer,uint8_t buffer_size,uint8_t last_byte_size,bool add_crc_a)
  {
    // Exit on wrong buffer
    if ((!buffer)||(buffer_size==0)) return false;

    // Set CRC_A
    if (add_crc_a) calculate_crc_a(buffer,buffer_size-2,&buffer[buffer_size-2]);

    // Initialize RC522 module
    spi_write_register(CommandReg,0x00);                    // Stop any active command
    spi_write_register(ComIrqReg,0x7F);                     // Clear IRQs
    spi_write_register(FIFOLevelReg,0x80);                  // Clear FIFO
    spi_write_register(FIFODataReg,buffer,buffer_size);     // Write data to send in the FIFO
    spi_write_register(BitFramingReg,last_byte_size&0x07);  // Set bit adjustments
    spi_write_register(CommandReg,0x0C);                    // Transceive command
    spi_set_register_bits(BitFramingReg,0x80);              // Start data transmission

    // Wait for the command to complete : 36 ms of timeout
    uint32_t timeout=millis()+36;
    while(true)
    {
      // Read IRQ bits
      uint8_t irq_bits=spi_read_register(ComIrqReg);
      if ((irq_bits&0x03)||(millis()>timeout)) return false;    // Return on error or timeout
      if (irq_bits&0x30) return true;                           // Return on command successfully terminated or valid data stream received

      // Delay
      delayMicroseconds(400);
    }

    // Error : unexpected behaviour
    return false;
  }



  // Read data
  bool RC522::read(uint8_t* buffer,uint8_t& buffer_size,bool control_crc_a)
  {
    // Get number of bytes wating in FIFO
    uint8_t bytes_in_fifo=spi_read_register(FIFOLevelReg);

    // Exit on wrong buffer
    if ((!buffer)||(buffer_size==0)||(bytes_in_fifo>buffer_size)) return false;

    // Get data from FIFO
    spi_read_register(FIFODataReg,buffer,bytes_in_fifo);
    buffer_size=bytes_in_fifo;

    // Get valid bits in the last received byte
    uint8_t valid_bits=spi_read_register(ControlReg)&0x07;

    // Exit on NAK received : transmition failed
    if ((bytes_in_fifo==1)&&(valid_bits==0x04)) return false;

    // Exit on wrong data received : buffer too small (at least 2 bytes for CRC_A or ATQA) or incomplete
    if ((bytes_in_fifo<2)&&(valid_bits!=0x00)) return false;

    // Exit on wrong CRC_A
    if (control_crc_a)
    {
      uint8_t crc_a[2];
      calculate_crc_a(buffer,buffer_size-2,crc_a);
      if ((buffer[buffer_size-2]!=crc_a[0])||(buffer[buffer_size-1]!=crc_a[1])) return false;
    }

    // Return : buffer is valid
    return true;
  }


  // Calculate CRC_A : software calculation to avoid bus activity
  void RC522::calculate_crc_a(uint8_t* buffer,uint8_t buffer_size,uint8_t* result)
  {
    // Exit on wrong result buffer
    if (!result) return;

    // Calculate CRC_A
    uint32_t crc=0x6363;
    do
    {
      uint8_t data_byte=*buffer++;
      data_byte=(data_byte^((uint8_t) (crc&0x00FF)));
      data_byte=(data_byte^(data_byte<<4));
      crc=(crc>>8)^((uint32_t) (data_byte<<8))^((uint32_t) (data_byte<<3))^((uint32_t) (data_byte>>4));
    }
    while(--buffer_size);

    // Set CRC
    result[0]=(uint8_t) (crc&0x00FF);
    result[1]=(uint8_t) ((crc>>8)&0x00FF);
  }



  // Set/clear register bits
  void RC522::spi_set_register_bits(uint8_t address,uint8_t mask)
  {
    // Set register bits
    spi_write_register(address,spi_read_register(address)|mask);
  }



  // Clear register bits
  void RC522::spi_clear_register_bits(uint8_t address,uint8_t mask)
  {
    // Clear register bits
    spi_write_register(address,spi_read_register(address)&(~mask));
  }



  // Read register
  uint8_t RC522::spi_read_register(uint8_t address)
  {
    // Platform specific: ESP8266
    #ifdef ESP8266

      // Select device
      spi_select_device();

      // Send address value
      SPI.transfer(address|0x80);

      // Read returned value
      uint8_t value=SPI.transfer(0);

      // Release device
      spi_release_device();

      // Return value
      return value;

    #endif

    // Platform specific: Raspberry pi
    #ifdef __arm__

      // Read register
      uint8_t spi_buffer[2]={(uint8_t) (address|0x80),0x00};
      wiringPiSPIDataRW(0,spi_buffer,2);

      // Return value
      return spi_buffer[1];

    #endif
  }
  void RC522::spi_read_register(uint8_t address,uint8_t* buffer,uint8_t buffer_size)
  {
    // Exit on wrong buffer
    if ((!buffer)||(buffer_size==0)) return;

    // Platform specific: ESP8266
    #ifdef ESP8266

      // Select device
      spi_select_device();

      // Send register value
      SPI.transfer(address|0x80);

      // Read bytes
      uint8_t index=0;
      while(index<(buffer_size-1)) buffer[index++]=SPI.transfer(address|0x80);

      // Read final byte
      buffer[index]=SPI.transfer(0);

      // Release device
      spi_release_device();

    #endif

    // Platform specific: Raspberry pi
    #ifdef __arm__

      // Create SPI buffer
      uint8_t spi_buffer[1+buffer_size];
      memset(spi_buffer,(uint8_t) (address|0x80),1+buffer_size);

      // Read SPI buffer
      wiringPiSPIDataRW(0,spi_buffer,1+buffer_size);

      // Set buffer
      memcpy(buffer,&spi_buffer[1],buffer_size);

    #endif
  }



  // Write register
  void RC522::spi_write_register(uint8_t address,uint8_t value)
  {
    // Platform specific: ESP8266
    #ifdef ESP8266

      // Select device
      spi_select_device();

      // Send register value
      SPI.transfer(address);

      // Write value
      SPI.transfer(value);

      // Release device
      spi_release_device();

    #endif

    // Platform specific: Raspberry pi
    #ifdef __arm__

      // Write register
      uint8_t spi_buffer[2]={(uint8_t) (address),value};
      wiringPiSPIDataRW(0,spi_buffer,2);

    #endif
  }
  void RC522::spi_write_register(uint8_t address,uint8_t* buffer,uint8_t buffer_size)
  {
    // Exit on empty buffer
    if (buffer_size==0) return;

    // Platform specific: ESP8266
    #ifdef ESP8266

      // Select device
      spi_select_device();

      // Send register value
      SPI.transfer(address);

      // Write buffer
      for(uint8_t index=0; index<buffer_size; index++) SPI.transfer(buffer[index]);

      // Release device
      spi_release_device();

    #endif

    // Platform specific: Raspberry pi
    #ifdef __arm__

      // Write buffer
      uint8_t spi_buffer[1+buffer_size]={(uint8_t) (address)};
      memcpy(&spi_buffer[1],buffer,buffer_size);
      wiringPiSPIDataRW(0,spi_buffer,1+buffer_size);

    #endif
  }



  // SPI select/release device
  void RC522::spi_select_device()
  {
    // Platform specific: ESP8266
    #ifdef ESP8266

      // Enable SPI
      SPI.begin();

      // Start SPI transaction 
      SPI.beginTransaction(SPISettings(m_frequency,MSBFIRST,SPI_MODE0));

      // Select device
      digitalWrite(m_sda_pin,LOW);

    #endif
  }
  void RC522::spi_release_device()
  {
    // Platform specific: ESP8266
    #ifdef ESP8266

      // Release device
      digitalWrite(m_sda_pin,HIGH);

      // Stop SPI transaction
      SPI.endTransaction();

      // Disable SPI
      SPI.end();

    #endif
  }
