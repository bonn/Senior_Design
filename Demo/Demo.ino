
#ifndef Ambient_h
#define Ambient_h

#include <ESP8266WiFi.h>                //ESP8266 Library


//Jordan Bonn - For the Ambient Class

#include <Wire.h>
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"
#include "DHTesp.h"

//Brandon Sanders - For the Water Class
#include "Arduino.h"
#include <Adafruit_LPS35HW.h>

//Jordan Bonn  -  For the External_Devices Class
#include <multi_channel_relay.h>
#include <Wire.h>
#include <Max517Dac.h>




Multi_Channel_Relay relay;

//Custom Definitions

#define SensorPin A0          // the pH meter Analog output is connected with the Analog pin
#define Offset 0.00

#define AL_ADDR_1 0x48
#define AL_ADDR_2 0x10




//Setup for Ambient Class
////////////////////////////////////////////////////////////////////////////////////////////
//Creating the device for the VEML sensor; we will use 2 sensors
SparkFun_Ambient_Light light_sens_1(AL_ADDR_1);
SparkFun_Ambient_Light light_sens_2(AL_ADDR_2);

// Possible values: .125, .25, 1, 2
// Both .125 and .25 should be used in most cases except darker rooms.
// A gain of 2 should only be used if the sensor will be covered by a dark
// glass.
float gain = .125;

// Possible integration times in milliseconds: 800, 400, 200, 100, 50, 25
// Higher times give higher resolutions and should be used in darker light.
int time_1 = 100;
long luxVal = 0;


//Creating the DHT device
DHTesp dht;

///////////////////////////////////////////////////////////////////////////////////////////////////


//Setup for the External Devices class
////////////////////////////////////////////////////////
//Creating the Relay Device

//Creating the DAC device
Max517Dac   dac;

///////////////////////////////////////////////////////


//Water class setup
//////////////////////////////////////////////////////////////////////////////////////////////////////
//unsigned long int avgValue;  //Store the average value of the sensor feedback
//float b;
//int buf[10],temp;
// pinMode(13,OUTPUT);  //setting pinMode for the pH sensor

double flowRate;    //This is the value we intend to calculate for water flow. 
volatile int count; //This integer needs to be set as volatile to ensure it updates correctly during the interrupt process.  
 
 //Sensor variables for Atlas Scientific pH Sensor
 float pH_Sensor_Value;
 float pH_Sensor_Voltage;
 float pH_Sensor_Reading ;


/////////////////////////////////////////////////////////////////////////////////////////////////////






//This class is used to create the Ambient object that we can get sensor data from
class Ambient {
public:

    Wire.begin();
    light_sens_1.begin();
    light_sens_1.setGain(gain);
    light_sens_1.setIntegTime(time_1);

    light_sens_2.begin();
    light_sens_2.setGain(gain);
    light_sens_2.setIntegTime(time_1);

    dht.setup(10, DHTesp::DHT22); // Connect DHT sensor to GPIO 10

    Ambient() {

    }

// This is used to select sensor 1 or 2
// sensor 1 is false (0) and sensor 2 is true (1)
    bool light_sensor;

//This function will return the temp in Farenheit
    float get_temp(){
//        Air_Temp = dht.getTemperature();
        return dht.toFahrenheit(dht.getTemperature());
    }

//This function will return the humidity as a percentage
    float get_humidity(){
//        Air_Humidity = dht.getHumidity();
        return dht.getHumidity();
    }

//This functin will return the light sensor read
//It takes a bool arg where false will return sensor 1 data and
    uint8_t get_light_reading(light_sensor){

      if(!light_sensor){
        return light_sens_1.readLight();
      }
        return light_sens_2.readLight();
    }

};



//This class is used to create the External Devices object
class External_Devices {
public:


   

    External_Devices() {

    }

    bool Dev_1,Dev_2,Dev_3,Dev_4;
    uint8_t device,desired_light_level;


//This function is called to turn off a device connected to the relay.
//It is passed an int between 1-4 and turns off the appropiate device
    void turn_off_dev(device){

        relay.turn_off_channel(device);

        return;
    }

//This function is called to turn on a device connected to the relay.
//It is passed an int between 1-4 and turns off the appropiate device
    void turn_on_dev(device){

        relay.turn_on_channel(device);

        return;
    }

//This function is called to return the current state of the relay
    bool get_relay_status(device){

        switch (device){

           case 1:
           return Dev_1;

           case 2:
           return Dev_2;

           case 3:
           return Dev_3;

           case 4:
           return Dev_4;

           default:
           return Dev_1;
      }

    }

//This function is called to set the dimemr level
//A value of 0 will be the lowest light ( hopefuly off)
//A valuse of 255 will turn the lights all the way up
    void set_light_level(desired_light_level){

        if (!dac.setOutput(desired_light_level)){

           Serial.println("Error talking to DAC. Check wiring.");
        }

        return;
    }
//This function is called to return the current value of the dimmer
//This can be used to show the current value on the GUI
    uint8_t get_light_level(){

        return desired_light_level;
    }

}






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


pinMode(2, INPUT);           //Sets the pin as an input
attachInterrupt(2, Flow, RISING);  //Configures interrupt 0 (pin 2) to run the function "Flow"  



  count = 1;      // Reset the counter so we start counting from 0 again
  interrupts();   //Enables interrupts on the Arduino
  delay (1000);   //Wait 1 second 
  noInterrupts(); //Disable the interrupts on the Arduino
  
   
 
  flowRate = (count * 2.25);        //Take counted pulses in the last second and multiply by 2.25mL 
  flowRate = flowRate * 60;         //Convert seconds to minutes, giving you mL / Minute
  flowRate = flowRate / 1000;       //Convert mL to Liters, giving you Liters / Minute
  return flowRate();
  
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

#endif
