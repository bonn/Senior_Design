/*
 * SimpleExample.ino
 *
 * This example demonstrates the usage of the MAX517 8-bit DAC library.
 * The diagram below shows the pinout of the 8 pin DIP version of the
 * device and its connections to the Arduino Uno.
 *
 *         -------.   .-------
 *        |        \_/        |
 *  A0 <--| OUT0         REF0 |<-- 5V
 *        |                   |
 * GND -->| GND   MAX517  VDD |<-- 5V
 *        |                   |
 *  A5 -->| SCL           AD0 |<-- GND
 *        |                   |
 *  A4 -->| SDA           AD1 |<-- GND
 *        |                   |
 *         -------------------
 *
 * Copyright (c) 2012 Jordan Goulder. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Wire.h>
#include <Max517Dac.h>


int         dacOutputValue    = 0;     // Value written to DAC


// Create a new DAC object
Max517Dac   dac;

void setup()
{
   // Initialize the serial interface
   Serial.begin(9600);

   // Reset the DAC output
   if(!dac.resetOutput())
   {
      Serial.println("Error talking to DAC. Check wiring.");
   }
}

void loop()
{
   // Read the value of the ADC
   if(255>dacOutputValue>0){
	   dacOutputValue = dacOutputValue+17;
   }else{
	   dacOutputValue = 0;
   }

   // Convert the analog reading to an 8-bit value
   

   // Print out the values
   Serial.print("DAC Output: ");
   Serial.print(dacOutputValue);
   
   
   // Update the DAC with a new value
   if (!dac.setOutput(dacOutputValue))
   {
      Serial.println("Error talking to DAC. Check wiring.");
   }

   // Wait a little bit for things to settle out
   delay(1500);
   
}
