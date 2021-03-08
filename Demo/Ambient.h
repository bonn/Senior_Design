
//Jordan Bonn

#include <ESP8266WiFi.h>                //ESP8266 Library

#include <Wire.h>
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"
#include "DHTesp.h"

#define AL_ADDR_1 0x48
#define AL_ADDR_2 0x10

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


//This class is used to create the pitchers and then has functions to add and remove water from the buckets
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


int main()
{
    Ambient ambient;

    ambient.get_temp();
    ambient.get_humidity();
    ambient.get_light_reading(0);
    ambient.get_light_reading(1);


    return 0;
}
