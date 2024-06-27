/*****************************************************/
/*                                                   */
/*       Protect/Unprotect Test for TinyRC522        */
/*      (c) Yvan Régeard - All rights reserved       */
/*                                                   */
/* Licensed under the MIT license. See LICENSE file  */
/* in the project root for full license information. */
/*                                                   */
/*****************************************************/



//
// Alternate the test between the factory key and a private key
//
// 1) Select an unprotected Mifare Classic 1K S50 tag
// 2) Read the data using a protection key
// 3) Write random data
// 4) Compare data
// 5) Toggle protection key
//



// Includes
#include <Arduino.h>
#include "TinyRC522.h"



// Constants

  // Tag size
  #define TAG_SIZE                  64



// Global variables
RC522 g_rc522;
uint8_t g_factory_key[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t g_private_key[6]={0x1F,0x03,0x57,0xA6,0xE1,0x3A};
bool g_use_private_key=false;



// Initialization
void setup()
{
  // Wait for stable signal
  delay(150);

  // Initialize debug traces
  Serial.begin(115200);
  Serial.println("\n");

  // Debug trace
  Serial.println("****** Protect/Unprotect application for TinyRC522 Library ******");

  // Initialize RC522 module
  g_rc522.initialize(2,1000000);                  // SDA pin on GPIO2, 1MHz SPI bus speed
  g_rc522.set_antenna_gain(ANTENNA_GAIN_18_DB);   // Set antenna gain: 18 dB
}



// Main loop
void loop()
{
  // Get tag
  Tag* p_tag=g_rc522.get_tag();
  if (p_tag)
  {
    // Debug trace
    Serial.printf("\nTag %08X found\n",p_tag->get_uid());

    // Set protection keys
    uint8_t* p_current_key=g_factory_key;
    uint8_t* p_new_key=g_private_key;
    if (g_use_private_key) { p_current_key=g_private_key; p_new_key=g_factory_key; }

    // Debug trace
    Serial.println(g_use_private_key?"Use PRIVATE key":"> Use FACTORY key");

    // Try to read data
    uint8_t data_buffer[TAG_SIZE];
    if (p_tag->read(data_buffer,TAG_SIZE,p_current_key))
    {
      // Generate random data buffer
      randomSeed(millis());
      for(uint8_t i=0; i<TAG_SIZE; i++) data_buffer[i]=random(256);

      // Write data block
      if (p_tag->write(data_buffer,TAG_SIZE,p_current_key))
      {
        // Read data block again
        uint8_t control_buffer[TAG_SIZE];
        if (p_tag->read(control_buffer,TAG_SIZE,p_current_key))
        {
          // Control validity
          bool result=true;
          for (uint8_t i=0; i<TAG_SIZE; i++) if (data_buffer[i]!=control_buffer[i]) result=false;

          // Set protection and toggle keys
          if (result) result=p_tag->set_protection(p_new_key,p_current_key);
          if (result) g_use_private_key=!g_use_private_key;

          // Debug trace
          Serial.println(result?(g_use_private_key?"Test succeeded: tag is now PROTECTED":"> Test succeeded: tag is now UNPROTECTED"):"/!\\ Test failed !");
        }
      }
    }
    else
    {
      // Debug trace
      Serial.println("Authentication failed: toggle keys");

      // Toggle keys: when authentication fails, it's mandatory to release the tag and select it again prior any further action on it
      g_use_private_key=!g_use_private_key;
    }

    // Release tag
    p_tag->release();
  }

  // Delay
  delay(50);
}
