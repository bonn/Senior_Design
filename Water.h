//Brandon

#include <ESP8266WiFi.h>
#include "Arduino.h"
#include <Adafruit_LPS35HW.h>

#define SensorPin A0          // the pH meter Analog output is connected with the Analog pin
#define Offset 0.00

unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10],temp;
pinMode(13,OUTPUT);  //setting pinMode for the pH sensor


int flowPin = 2;    //This is the input pin on the Arduino
double flowRate;    //This is the value we intend to calculate. 
volatile int count; //This integer needs to be set as volatile to ensure it updates correctly during the interrupt process.  
 


// Sensor variables for Atlas Scientific pH Sensor
// float pH_Sensor_Value;
// float pH_Sensor_Voltage;
// float pH_Sensor_Reading ;



//This class calls upon the LPS33HW water pressure and temp sensor, flow sensor, and pH sensor's data.
class Water {
public:

Water(){

}
Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

float get_level() {

 return lps35hw.readPressure(); //output is currently hPa and only called one time

}

float get_temp() {

return (lps35hw.readTemperature()* 9/5 + 32); //output is currently Fahrenheit and only called one time

}

 
 
uint8_t get_flow() {

  pinMode(flowPin, INPUT);           //Sets the pin as an input
  attachInterrupt(0, Flow, RISING);  //Configures interrupt 0 (pin 2 on the Arduino Uno) to run the function "Flow"  


  count = 0;      // Reset the counter so we start counting from 0 again
  interrupts();   //Enables interrupts on the Arduino
  delay (1000);   //Wait 1 second 
  noInterrupts(); //Disable the interrupts on the Arduino
   
  //Start the math
  flowRate = (count * 2.25);        //Take counted pulses in the last second and multiply by 2.25mL 
  flowRate = flowRate * 60;         //Convert seconds to minutes, giving you mL / Minute
  flowRate = flowRate / 1000;       //Convert mL to Liters, giving you Liters / Minute
 
  return flowRate();
  
}

 void Flow()
{
   count++; //Every time this function is called, increment "count" by 1
}
 
 
 
float get_phlevel() {

    // Sensor reading for Atlas Scientific pH Sensor
    // pH_Sensor_Value = analogRead(A0);
    // pH_Sensor_Voltage = pH_Sensor_Value * (3.3 / 1023.0);
    // pH_Sensor_Reading = (( -5.6548 * pH_Sensor_Voltage) + 15.509);

    // Depending on our memory usage we may have to decrease the sample size


    for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
        { 
                buf[i]=analogRead(SensorPin);
                delay(10);
        }
    for(int i=0;i<9;i++)        //sort the analog from small to large
        {
            for(int j=i+1;j<10;j++)
                {
                    if(buf[i]>buf[j])
                        {
                            temp=buf[i];
                            buf[i]=buf[j];
                            buf[j]=temp;
                        }
                    }
        }


    avgValue=0;
//take the average value of 6 center sample
  for(int i=2;i<8;i++){
      avgValue+=buf[i];
    }
    
// I think this should be *3.3 since the ESP high is 3.3V; also it should be 1023 not 1024
  float phValue=(float)avgValue*3.3/1023/6; //convert the analog into millivolt
  phValue=3.5*phValue;                      //convert the millivolt into pH value
  
  return phValue();
}


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
