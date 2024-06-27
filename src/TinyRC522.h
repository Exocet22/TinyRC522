/*****************************************************/
/*                                                   */
/*                     TinyRC522                     */
/*      (c) Yvan Régeard - All rights reserved       */
/*                                                   */
/* Licensed under the MIT license. See LICENSE file  */
/* in the project root for full license information. */
/*                                                   */
/*****************************************************/
#ifndef _TINY_RC522_H_
#define _TINY_RC522_H_



// Constants

  // Tag maximum size
  #define TAG_MAX_SIZE              752             // 47 blocks of 16 bytes

  // Antenna gain
  #define ANTENNA_GAIN_OFF          0x00            // Antenna off
  #define ANTENNA_GAIN_18_DB        0x28            // 18 dB
  #define ANTENNA_GAIN_23_DB        0x38            // 23 dB
  #define ANTENNA_GAIN_33_DB        0x48            // 33 dB
  #define ANTENNA_GAIN_38_DB        0x58            // 38 dB
  #define ANTENNA_GAIN_43_DB        0x68            // 43 dB
  #define ANTENNA_GAIN_48_DB        0x78            // 48 dB



// RC522 class declaration
class RC522;



// Mifare 1K S50 (4-bytes UID) class definition
class Tag
{
  // Friend class access
  friend class RC522;



  // Private attributes
  private:

    // RC522 module pointer
    RC522* m_p_rc522;

    // UID
    uint8_t m_uid[7];
    uint8_t m_uid_size;



  // Public functions
  public:

    // Constructor
    Tag(RC522* p_rc522);

    // Get UID
    uint32_t get_uid();

    // Set/remove protection
    bool set_protection(uint8_t* new_key,uint8_t* key=NULL);
    bool remove_protection(uint8_t* key);

    // Read/write data
    bool read(uint8_t* buffer,uint16_t buffer_size,uint8_t* key=NULL);
    bool write(uint8_t* buffer,uint16_t buffer_size,uint8_t* key=NULL);

    // Release
    bool release();



  // Private functions
  private:

    // Initialize
    void initialize();

    // Add UID byte
    void add_uid_byte(uint8_t uid_byte);
};



// RC522 class definition
class RC522
{
  // Friend class access
  friend class Tag;



  // Private attributes
  private:

    // SDA pin
    uint8_t m_sda_pin;

    // Frequency
    uint32_t m_frequency;

    // Tag
    Tag m_tag{this};



  // Public functions
  public:

    // Initialize
    void initialize(uint8_t sda_pin,uint32_t frequency);

    // Set antenna gain
    void set_antenna_gain(uint8_t gain);

    // Get tag
    Tag* get_tag();

    // Get module version
    uint8_t get_version();



  // Private functions
  private:

    // Authenticate
    bool authenticate(uint8_t* buffer,uint8_t buffer_size);

    // Read/write data
    bool read(uint8_t* buffer,uint8_t& buffer_size,bool control_crc_a=true);
    bool write(uint8_t* buffer,uint8_t buffer_size,uint8_t last_byte_size=0,bool add_crc_a=true);

    // Calculate CRC_A
    void calculate_crc_a(uint8_t* buffer,uint8_t buffer_size,uint8_t* result);

    // SPI Set/clear register bits
    void spi_set_register_bits(uint8_t address,uint8_t mask);
    void spi_clear_register_bits(uint8_t address,uint8_t mask);

    // SPI Read register
    uint8_t spi_read_register(uint8_t address);
    void spi_read_register(uint8_t address,uint8_t* buffer,uint8_t buffer_size,uint8_t rx_align);

    // SPI Write register
    void spi_write_register(uint8_t address,uint8_t value);
    void spi_write_register(uint8_t address,uint8_t* buffer,uint8_t buffer_size);

    // SPI Select/release device
    void spi_select_device();
    void spi_release_device();
};



#endif
