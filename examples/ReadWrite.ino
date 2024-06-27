/*****************************************************/
/*                                                   */
/*           Read/Write Test for TinyRC522           */
/*      (c) Yvan Régeard - All rights reserved       */
/*                                                   */
/* Licensed under the MIT license. See LICENSE file  */
/* in the project root for full license information. */
/*                                                   */
/*****************************************************/



//
// 1) Select an unprotected Mifare Classic 1K S50 tag
// 2) Read and display the data
// 3) Write random data
// 4) Compare data and display the new written data
//



// Includes
#include <Arduino.h>
#include "TinyRC522.h"



// Constants

  // Tag size
  #define TAG_SIZE                  64



// Global variables
RC522 g_rc522;



// Dump tag data
void dump_tag_data(uint8_t* buffer, uint16_t buffer_size)
{
  for(uint16_t i=0; i<buffer_size; i+=16)
    Serial.printf("%03d> %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
      i,
      buffer[i+0],buffer[i+1],buffer[i+2],buffer[i+3],buffer[i+4],buffer[i+5],buffer[i+6],buffer[i+7],
      buffer[i+8],buffer[i+9],buffer[i+10],buffer[i+11],buffer[i+12],buffer[i+13],buffer[i+14],buffer[i+15]);
}



// Initialization
void setup()
{
  // Wait for stable signal
  delay(150);

  // Initialize debug traces
  Serial.begin(115200);
  Serial.println("\n");

  // Debug trace
  Serial.println("****** Read/write application for TinyRC522 Library ******");

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

    // Read tag data
    uint8_t data_buffer[TAG_SIZE];
    if (p_tag->read(data_buffer,TAG_SIZE))
    {
      // Dump tag data
      dump_tag_data(data_buffer,TAG_SIZE);

      // Generate random data buffer
      randomSeed(millis());
      for(uint8_t i=0; i<TAG_SIZE; i++) data_buffer[i]=random(256);

      // Write data block
      if (p_tag->write(data_buffer,TAG_SIZE))
      {
        // Read data block again
        uint8_t control_buffer[TAG_SIZE];
        if (p_tag->read(control_buffer,TAG_SIZE))
        {
          // Dump tag data
          dump_tag_data(data_buffer,TAG_SIZE);

          // Control validity
          bool result=true;
          for (uint8_t i=0; i<TAG_SIZE; i++) if (data_buffer[i]!=control_buffer[i]) result=false;
          
          // Debug trace
          Serial.println(result?"Test succeeded":"/!\\ Test failed !");
        }
      }
    }

    // Release tag
    p_tag->release();
  }

  // Delay
  delay(50);
}
