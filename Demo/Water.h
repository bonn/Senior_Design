//Brandon
#include <Arduino.h>
#include <Adafruit_LPS35HW.h>

#define SensorPin A0          // the pH meter Analog output is connected with the Analog pin



static float pH_Sensor_Value,pH_Sensor_Voltage,pH_Sensor_Reading;

int flowPin = 2;    //This is the input pin on the Arduino
double flowRate;    //This is the value we intend to calculate. 
volatile int count; //This integer needs to be set as volatile to ensure it updates correctly during the interrupt process.  

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();





//This class calls upon the LPS33HW water pressure and temp sensor, flow sensor, and pH sensor's data.
class Water {
public:

Water(){

}


float get_level() {

 return lps35hw.readPressure(); //output is currently hPa and only called one time

}

float get_temp() {

return (lps35hw.readTemperature()* 9/5 + 32); //output is currently Fahrenheit and only called one time

}

 
 
uint8_t get_flow() {

  pinMode(flowPin, INPUT);           //Sets the pin as an input
  attachInterrupt(0, count++, RISING);  //Configures interrupt 0 (pin 2 on the Arduino Uno) to run the function "Flow"  


  count = 0;      // Reset the counter so we start counting from 0 again
  interrupts();   //Enables interrupts on the Arduino
  delay (1000);   //Wait 1 second 
  noInterrupts(); //Disable the interrupts on the Arduino
   
  //Start the math
  flowRate = (count * 2.25);        //Take counted pulses in the last second and multiply by 2.25mL 
  flowRate = flowRate * 60;         //Convert seconds to minutes, giving you mL / Minute
  flowRate = flowRate / 1000;       //Convert mL to Liters, giving you Liters / Minute
 
  return (flowRate);
  
}

 
 
 
float get_phlevel() 
{

  

// Sensor variables for Atlas Scientific pH Sensor
 //float pH_Sensor_Value;
 //float pH_Sensor_Voltage;
 //float pH_Sensor_Reading;
    
    pH_Sensor_Value = analogRead(A0);
    pH_Sensor_Voltage = pH_Sensor_Value * (3.3 / 1023.0);
    pH_Sensor_Reading = (( -5.6548 * pH_Sensor_Voltage) + 15.509);
              
  
  
  return (pH_Sensor_Reading);
}




int main()
{
    Water water;

    water.get_level();
    water.get_temp();
    water.get_flow();
    water.get_phlevel();


    return 0;
}
};
