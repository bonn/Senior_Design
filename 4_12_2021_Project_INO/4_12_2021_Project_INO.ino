/*
    GAIA Software
    
    Created by Patrick Schneider, Brandon Sanders, and Jordan Bonn
*/

/*
 * NOTE:
  Serial1 is used for debugging because the Tx and Rx from USB are the same as Serial on the ESP8266

  Serial1 is port D4 on the ESP8266

  NODE MCU references: https://www.make-it.ca/nodemcu-arduino/nodemcu-details-specifications/
                       https://robu.in/a-beginner-guide-to-nodemcu/
  
*/





  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Libraries to Import                                    // 
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <ESP8266WiFi.h>
#include <Nextion.h>
#include "NexText.h"
#include "NexButton.h"
#include "NexSlider.h"

//For the Ambient Subsystem Sensors
#include <Wire.h>
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"
#include "DHTesp.h"

//For the Water Subsystem Sensors
#include "Arduino.h"
#include <Adafruit_LPS35HW.h>


//For the External Control Subsystem Sensors
#include <multi_channel_relay.h>
#include <Max517Dac.h>

//SD Card Libraries for exporting data
#include <SPI.h>
#include <SD.h>

//NTP and Wifi Libraries to get internet time
#include "NTPClient.h"
#include "ESP8266WiFi.h"
#include "WiFiUdp.h"


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Custom Definitions and Variables                                // 
///////////////////////////////////////////////////////////////////////////////////////////////////

//pH definitions
#define SensorPin A0          // the pH meter Analog output is connected with the Analog pin
#define Offset 0.00

//Defining the addresses for the lught sensors
#define AL_ADDR_1 0x48
#define AL_ADDR_2 0x10

//defining the address of the DHT
#define DHTPIN 16               // what digital pin we're connected to

//defining SD card pin
const int SD_CS = 0;   // chip select pin for the SD Card







//Variables for the alert sensors
int alerts_water_temp_min,
    alerts_water_temp_max,
    alerts_water_ph_min,
    alerts_water_ph_max,
    alerts_ambient_temp_min,
    alerts_ambient_temp_max;

//Variables for the scheduling
String schedules_water_start,
       schedules_water_end,
       schedules_air_start,
       schedules_air_end,
       schedules_light_start,
       schedules_light_end;

char buffer[100] = {0};


bool simulate_Sunlight;
uint32_t sun_state;


//Pin that is used for the water flow sensor
const byte water_flow_pin = 10;            // GPIO pin for the water flow sensor


//Strings used for logging files
String     headerString = "Time,Water Temp,Water pH,Water Level,Water Flow,Air Temp,Air Humidity,Light 1 Level,Light 2 Level";   //,Dimmer Level,Relay 1 Status,Relay 2 Status,Relay 3 Statu
String     dataString = "";
String     data_logging_file_name = "";  // we will add the time and file extension when creating the file
String     the_time;  

String formatted_Date="";
String dayStamp="";
String timeStamp="";   
long epoch;                                            



float test = 1; //testing dTOstrF
                                                  
File dataFile;  //File used for SD card
//variables for the LPS water temp and pressure sensor
float temperature;
float pressure;

//variable for the flow sensor
float flow;

//variable for the pH sensor
float pH;


//variable for the dimmer value
uint32_t dimmer_value = 0;
float dimmer_float;
char temp_dimmer_value[10] = {0};
uint8_t dimmer_float_2_int;

//variables for the DHT temp sensor
float TempF;
float humidity;
float DHT_humidity;
float DHT_temperature;

TempAndHumidity measurement;

bool device_1,device_2,device_3;



//variables for the Lux sensor
float lux1;
float lux2;

//  Gain settings for VEML light sensors
// Possible values: .125, .25, 1, 2
// Both .125 and .25 should be used in most cases except darker rooms.
// A gain of 2 should only be used if the sensor will be covered by a dark
// glass.
float gain = .125;

// Possible integration times in milliseconds: 800, 400, 200, 100, 50, 25
// Higher times give higher resolutions and should be used in darker light.
int time_1 = 100;
long luxVal = 0;


//Variables for the flow sensor
double flowRate;    //This is the value we intend to calculate for water flow.
volatile int count; //This integer needs to be set as volatile to ensure it updates correctly during the interrupt process.


//variable used for logging data
bool trigger;

//variables used to store SSID info *NEEDS TO BE CHANGED TO UPDATE IN APP
const char *ssid = "The Promised LAN";
const char *password = "einstein418";


// Sensor variables for Atlas Scientific pH Sensor
 float pH_Sensor_Value;
 float pH_Sensor_Voltage;
 float pH_Sensor_Reading ;



//char arrays that contain that values that are displayed to the LCD and used for logging
  static char pHTemp[6];
  static char temperatureWTemp[6];
  static char levelTemp[6];
  static char flowTemp[6];
  static char temperatureFTemp[6];
  static char humidityTemp[6];
  static char lux1Temp[6];
  static char lux2Temp[6];



//Offset for NTP time & date data
const long utcOffsetInSeconds = 19800;
//used for NTP
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};







  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Creating and Setting up Devices                                 // 
///////////////////////////////////////////////////////////////////////////////////////////////////


//Create NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);


//***********Setup for Ambient Class**************

//Creating the DHT device
DHTesp dht;

//Creating the device for the VEML sensor; we will use 2 sensors
SparkFun_Ambient_Light light(AL_ADDR_1);
SparkFun_Ambient_Light light_2(AL_ADDR_2);



//*******Setup for the External Devices class********

//Creating the Relay Device
Multi_Channel_Relay relay;

//Creating the DAC device
Max517Dac   dac;



//*******Setup for the Water class********

//Creating Water Objects
Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

// The pH and flow sensor do not need any objects created



  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Flow  FUNCTION                                       // 
///////////////////////////////////////////////////////////////////////////////////////////////////

//Function to obtain the flow data
uint8_t get_flow() {
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

//This function is called to process the count for the get_flow functions
//This must be a functions due to the wat the interrupt works
//the ICACHE_RAM_ATTR is needed on the ESP8266 so that this is stored in the correct location
ICACHE_RAM_ATTR void flow_interrupt(){
  count++;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 CREATE DEVICES FUNCTION                                       // 
///////////////////////////////////////////////////////////////////////////////////////////////////


//This functions creates all of the objects for the varios sensors used
void create_devices()
{
 Serial1.println("Let's start up some sensors!");

//Starting the Relay
relay.begin(0x11);
  device_1 = false;
  device_2 = false;
  device_3 = false;


//Starting DHT Device
dht.setup(DHTPIN, DHTesp::DHT22); // Connect DHT sensor to Pin defined earlier


//Starting the Light Sensors
  if(light.begin())
    Serial1.println("Ready to sense some light!"); 
  else
    Serial1.println("Could not communicate with the sensor!");

    if(light_2.begin())
    Serial1.println("Ready to sense some light!"); 
  else
    Serial1.println("Could not communicate with the sensor_2!");


//Starting LPS35HW device
 if (!lps35hw.begin_I2C()) {
    Serial1.println("Couldn't find LPS35HW chip");
    while (1);
  }
  Serial1.println("Found LPS35HW chip");
  

//Starting FLOW SENSOR
pinMode(water_flow_pin, INPUT);           //Sets the pin as an input
attachInterrupt(water_flow_pin , flow_interrupt, RISING);  //Configures interrupt 0 (pin 2 on the Arduino Uno) to run the function "Flow"





//Starting the DAC

 if(!dac.resetOutput())
   {
      Serial1.println("Error talking to DAC. Check wiring.");
   }else{
      Serial1.println("Was able to talk to DAC. No Issue.");
    }

}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                NEXTION SETUP -  TOUCH SCREEN OBJECTS (TEXT BOXES AND BUTTONS)                 // 
///////////////////////////////////////////////////////////////////////////////////////////////////



//Nextion objects - Example (page id = 0, component id = 1, component name = "b0")


/* Note that the page that is currently being viewed is that only page that can have components updated

   This means that if you viewing the settings page when the sensor data updates, when you go back to the first page the data will not be updated 
   if you just try to update the textbox on page 1

   To get around this we will update every page and have hidden variables in the settings page that
   will update then set the correct vales when leaving the settings screen
   
*/


//*************PAGE_1**********************************************************************************************
//Water Monitoring values
NexText tWater_Temp_page_1      = NexText (1, 11, "tWater_Temp");  //Text with water temperature value.
NexText tWater_Lvl_page_1       = NexText (1, 12, "tWater_Lvl");   //Text with water level value.
NexText tWater_Flow_page_1      = NexText (1, 13, "tWater_Flow");  //Text with water flow value.
NexText tWater_pH_page_1        = NexText (1, 14, "tWater_pH");         //Text with water pH value.

//Ambient Environment values
NexText tAir_Temp_page_1        = NexText (1, 15, "tAir_Temp");       //Text with air temperature value.
NexText tAir_Humidity_page_1    = NexText (1, 16, "tAir_Humidity");   //Text with humidity value.
NexText tAir_Light_1_page_1     = NexText (1, 17, "tAir_Light_1");    //Text with light level 1 value.
//NexText tAir_Light_2_page_1   = NexText (1, 40, "light2_var");    //Text with light level 2 value.


//*************PAGE_2**********************************************************************************************
//Water Monitoring values
NexText tWater_Temp_page_2       = NexText (2, 11, "tWater_Temp");  //Text with water temperature value.
NexText tWater_Lvl_page_2        = NexText (2, 12, "tWater_Lvl");   //Text with water level value.
NexText tWater_Flow_page_2       = NexText (2, 13, "tWater_Flow");  //Text with water flow value.
NexText tWater_pH_page_2         = NexText (2, 14, "tWater_pH");         //Text with water pH value.

//Ambient Environment values
NexText tAir_Temp_page_2         = NexText (2, 15, "tAir_Temp");       //Text with air temperature value.
NexText tAir_Humidity_page_2     = NexText (2, 16, "tAir_Humidity");   //Text with humidity value.
NexText tAir_Light_2_page_2      = NexText (2, 17, "tAir_Light_2");    //Text with light level 2 value.
//NexText tAir_Light_1_page_2    = NexText (2, 40, "light1_var");    //Text with light level 1 value.

//*************PAGE_3********************************************************************************************
//Water Monitoring values
NexText tWater_Temp_page_3  = NexText (3, 11, "tWater_Temp");  //Text with water temperature value.
NexText tWater_Lvl_page_3   = NexText (3, 12, "tWater_Lvl");   //Text with water level value.
NexText tWater_Flow_page_3  = NexText (3, 13, "tWater_Flow");  //Text with water flow value.
NexText tWater_pH_page_3    = NexText (3, 14, "tWater_pH");         //Text with water pH value.

//Ambient Environment values
NexText tAir_Temp_page_3        = NexText (3, 15, "tAir_Temp");       //Text with air temperature value.
NexText tAir_Humidity_page_3    = NexText (3, 16, "tAir_Humidity");   //Text with humidity value.
NexText tAir_Light_1_page_3     = NexText (3, 17, "tAir_Light_1");    //Text with light level 1 value.
//NexText tAir_Light_2_page_3   = NexText (3, 40, "light2_var");    //Text with light level 2 value.

//*************PAGE_4**********************************************************************************************
//Water Monitoring values
NexText tWater_Temp_page_4    = NexText (4, 11, "tWater_Temp");  //Text with water temperature value.
NexText tWater_Lvl_page_4     = NexText (4, 12, "tWater_Lvl");   //Text with water level value.
NexText tWater_Flow_page_4    = NexText (4, 13, "tWater_Flow");  //Text with water flow value.
NexText tWater_pH_page_4      = NexText (4, 14, "tWater_pH");         //Text with water pH value.

//Ambient Environment values
NexText tAir_Temp_page_4        = NexText (4, 15, "tAir_Temp");       //Text with air temperature value.
NexText tAir_Humidity_page_4    = NexText (4, 16, "tAir_Humidity");   //Text with humidity value.
NexText tAir_Light_2_page_4     = NexText (4, 17, "tAir_Light_2");    //Text with light level 2 value.
//NexText tAir_Light_1_page_4   = NexText (4, 40, "light1_var");    //Text with light level 1 value.


///***********************************************************************************************************************
//************SETTINGS_PAGE***********************************************************************************************
//LED 
NexText   tState1   = NexText   (9, 5,  "tState1");    //Text to show Dev1 ON/OFF status.
NexButton bDev1_On  = NexButton (9, 6,  "bDev1_On");   //Button to turn Dev1 ON (LED - channel 1)
NexButton bDev1_Off = NexButton (9, 7,  "bDev1_Off");  //Button to turn Dev1 OFF (LED - channel 1)

//Air Pump 
NexText   tState2   = NexText   (9, 10, "tState2");    //Text to show Dev2 ON/OFF status.
NexButton bDev2_On  = NexButton (9, 11, "bDev2_On");   //Button to turn Dev2 ON (AIR PUMP - channel 2)
NexButton bDev2_Off = NexButton (9, 12, "bDev2_Off");  //Button to turn Dev2 OFF (AIR PUMP - channel 2)


//Water Pump
NexText   tState3   = NexText   (9, 14, "tState3");    //Text to show Dev3 ON/OFF status.
NexButton bDev3_On  = NexButton (9, 15, "bDev3_On");   //Button to turn Dev3 ON (WATER PUMP - channel 3)
NexButton bDev3_Off = NexButton (9, 16, "bDev3_Off");  //Button to turn Dev3 OFF (WATER PUMP - channel 3)

//Dimming Controls
NexSlider sDimmer   = NexSlider (9, 3, "sDimmer");      //Slider bar to control  LED intensity.

//Upload sensor data sensors
NexButton bUpload   = NexButton(9, 18, "bUpload");   //Button to upload data (UD1)


///***********************************************************************************************************************
//************ALERTS_PAGE***********************************************************************************************
 
NexText   tempMinW       = NexText   (11, 14,  "tempMinW");    
NexText   tempMaxW       = NexText   (11, 15,  "tempMaxW");  
  
NexText   phMin          = NexText   (11, 16,  "phMin");    
NexText   phMax          = NexText   (11, 17,  "phMax");  
  
NexText   tempMinA       = NexText   (11, 19,  "tempMinA");   
NexText   tempMaxA       = NexText   (11, 18,  "tempMaxA");   

NexButton bSet_Alerts    = NexButton (11, 13,  "bSet_Alerts");   //Button to turn Dev1 ON (LED - channel 1)

///***********************************************************************************************************************
//************SCHEDULES_PAGE***********************************************************************************************
 
NexText   water_start     = NexText   (13, 11,  "water_start");    //Text to show Dev1 ON/OFF status.
NexText   water_end       = NexText   (13, 16,  "water_end");    //Text to show Dev1 ON/OFF status.

NexText   air_start       = NexText   (13, 12,  "air_start");    //Text to show Dev1 ON/OFF status.
NexText   air_end         = NexText   (13, 15,  "air_end");    //Text to show Dev1 ON/OFF status.

NexText   light_start     = NexText   (13, 13,  "light_start");    //Text to show Dev1 ON/OFF status.
NexText   light_end       = NexText   (13, 14,  "light_end");    //Text to show Dev1 ON/OFF status.

NexDSButton simulateSun   = NexDSButton(13, 17, "simulateSun");  //simulateSun.getValue(&sun_state);

NexButton bSet_Schedule   = NexButton (13, 9,  "bSet_Schedule");   //Button to turn Dev1 ON (LED - channel 1)




//*************** Register a button object to the touch event listener************
NexTouch *nex_listen_list[] = {

  &bDev1_On,
  &bDev1_Off,
  &bDev2_On,
  &bDev2_Off,
  &bDev3_On,
  &bDev3_Off,
  &sDimmer,
  &bUpload,
  &bSet_Schedule,
  &bSet_Alerts,

  NULL
};



  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //             BUTTON CALLBACKS (PUT CODE IN HERE TO RUN WHEN THE IS PUSHED)                     // 
///////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Button bDev1_On component pop callback function.
 * This button turns on channel 1 of the relay
 */
void bDev1_OnPopCallback(void *ptr) {
  
/*
 * Button bDev1_On component pop callback function.
 * This button turns on channel 1 of the relay
 */  
  tState1.setText("State: on");
  device_1 = true;
  delay(500);//debugging
 Serial1.println("Turn on Relay Channel 1");
  relay.turn_on_channel(1);
  
}
/*
 * Button bDev1_Off component pop callback function.
 * This button turns off channel 1 of the relay
 */
void bDev1_OffPopCallback(void *ptr) {
  tState1.setText("State: off");
  device_1 = false;
  Serial1.println("Turn off Relay Channel 1");
  relay.turn_off_channel(1);  

}
/*
 * Button bDev2_On component pop callback function.
  * This button turns on channel 2 of the relay
 */
void bDev2_OnPopCallback(void *ptr) {
  tState2.setText("State: on");
  device_2 = true;
  Serial1.println("Turn on Relay Channel 2");
  relay.turn_on_channel(2);
 
}
/*
 * Button bDev2_Off component pop callback function.
 * This button turns off channel 2 of the relay
 */

void bDev2_OffPopCallback(void *ptr) {
  tState2.setText("State: off");
  device_2 = false;
  Serial1.println("Turn off Relay Channel 2");
  relay.turn_off_channel(2);
  
}
/*
 * Button bDev3_On component pop callback function.
  * This button turns on channel 3 of the relay
 */
void bDev3_OnPopCallback(void *ptr) {
  tState3.setText("State: on");
  device_3 = true;
  Serial1.println("Turn on Relay Channel 3");
  Serial1.println();
  relay.turn_on_channel(3);
    
}
/*
 * Button bDev3_Off component pop callback function.
 * This button turns off channel 3 of the relay
 */
void bDev3_OffPopCallback(void *ptr) {
  tState3.setText("State: off");
  device_3 = false;
  Serial1.println("Turn off Relay Channel 3");
  relay.turn_off_channel(3);
  
}
/*
 * Slider sDimmer component pop call back function
 * Slider controls a value that is sent to DAC via ESP to update the intensity of the light.
 * The Slider Text will be updated with the value the slider holds.
 */
void sDimmerPopCallback(void *ptr){
  
  

  //Change dimmer value text with the current slider value
  sDimmer.getValue(&dimmer_value);
  dimmer_float = (255.0*(dimmer_value/100.0));

 // utoa(dimmer_value, temp_dimmer_value, 10);
 // tDval.setText(temp_dimmer_value);

dimmer_float_2_int = round(dimmer_float);

Serial1.println("We are in the Dimmer callbak function.");
Serial1.print(" The current dimmer setting is at: ");
Serial1.print(dimmer_value);
Serial1.print("% or ");
Serial1.print(dimmer_float_2_int);
Serial1.println(" of 255.");
    
  //This function sets the DAC output to the correct level( 8 bit unsinged int; 0-255 levels )
  //The slider goes from 0-100% so we make the number and turn it into a percentage to muliply by 255
  dac.setOutput(dimmer_float_2_int);


}
/*
 * button bUpload component pop callback funtion
 * This button is used to upload data to the cloud
 */
void bUploadPopCallback(void *ptr) {
  Serial1.println("Inside the upload function");
  bUpdateSensorValues();
  delay(1000);
  bUpdateDisplay();
  logData();  
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  
}



//Variables for the alert sensors
   


//function to gather all the data needed for alerts
// this is called when we are at the alerts page when the user presses the set alerts button
void getAlerts(){


    tempMinW.getText(buffer, sizeof(buffer));
    alerts_water_temp_min     = atoi(buffer);
    
    tempMaxW.getText(buffer, sizeof(buffer));
    alerts_water_temp_max     = atoi(buffer);
    
    phMin.getText(buffer, sizeof(buffer));
    alerts_water_ph_min       = atoi(buffer);
    
    phMax.getText(buffer, sizeof(buffer));
    alerts_water_ph_max       = atoi(buffer);
    
    tempMinA.getText(buffer, sizeof(buffer));
    alerts_ambient_temp_min   = atoi(buffer);
    
    tempMaxA.getText(buffer, sizeof(buffer));
    alerts_ambient_temp_max   = atoi(buffer);
}



//function to gather all the data needed for schedules
// this is called when we are at the schedule page when the user presses the set schedule button
void getSchedule(){

//Variables for the scheduling



       water_start.getText(buffer, sizeof(buffer));
       schedules_water_start  = buffer;
       
       water_end.getText(buffer, sizeof(buffer)) ;
       schedules_water_end    = buffer ;
       
       air_start.getText(buffer, sizeof(buffer));
       schedules_air_start    = buffer;
       
       air_end.getText(buffer, sizeof(buffer));
       schedules_air_end      = buffer;
       
       light_start.getText(buffer, sizeof(buffer));
       schedules_light_start  = buffer; 
       
       light_end.getText(buffer, sizeof(buffer));
       schedules_light_end    = buffer;

       simulateSun.getValue(&sun_state);

Serial1.println(schedules_water_start);
Serial1.println(sun_state);
       

}


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   UPDATE SENSOR VARIABLES                                     // 
///////////////////////////////////////////////////////////////////////////////////////////////////

// This function will update the global variables that are used to hold the sensor values
void bUpdateSensorValues(){
//
//Serial1.println("We are in the UpdateSensorValues Function ");
//    
     
     //This reads the analog input and converts it to pH
      pH_Sensor_Value = analogRead(A0);
      pH_Sensor_Voltage = pH_Sensor_Value * (3.3 / 1023.0);
      pH_Sensor_Reading = (( -5.6548 * pH_Sensor_Voltage) + 15.509);
      pH =  pH_Sensor_Reading;     

      //Getting LPS35 water Temp
      temperature = lps35hw.readTemperature()* 9/5 + 32;
      
      //Getting LPS53 Pressure data
      pressure = lps35hw.readPressure();

      //Getting Water Flow
      flow =  get_flow();

    //Getting the DHT Temp & Humidity
    measurement =dht.getTempAndHumidity();
    
    DHT_temperature = measurement.temperature;
    DHT_humidity = measurement.humidity;
 
    TempF = dht.toFahrenheit(DHT_temperature);

    //Getting the DHT Humidity
    humidity = DHT_humidity;

      //Getting the lux readings
      lux1 = light.readLight();

      lux2 = light_2.readLight();


/*
//Serial1 testlogging
Serial1.println("We completed in the pH Section");
Serial1.print("The Value is: ");
Serial1.println(pH);


Serial1.println("We completed in the water temp Section");
Serial1.print("The Value is: ");
Serial1.println(temperature);


Serial1.println("We completed in the water pressure Section");
Serial1.print("The Value is: ");
Serial1.println(pressure);



Serial1.println("flow ended");      
Serial1.println("We completed in the water flow Section");


Serial1.print("The DHT is showing a temp of: ");
Serial1.println(DHT_temperature);
Serial1.print("The DHT is showing a humidity of: ");
Serial1.println(DHT_humidity);

Serial1.println("We completed in the air temp Section");
Serial1.println(TempF);
//delay(1000);  // for debugging

Serial1.println("We completed in the air hum Section");
Serial1.println(humidity);
//delay(1000);  // for debugging
//

Serial1.println("We completed in the lux_1 temp Section");
Serial1.print("The Value is: ");
Serial1.println(lux1);
//delay(1000);


Serial1.println("We completed in the lux_2  Section");
Serial1.print("The Value is: ");
Serial1.println(lux2);
//delay(1000);  // for debugging


*/

}




  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        LOG SENSOR DATA                                        // 
///////////////////////////////////////////////////////////////////////////////////////////////////

//We need to update the data string that will be uploaded when we pull the new data
//The format of the data string is shown below
/*
CSV FORMAT:
Time,Water Temp,Water pH,Water Level,Water Flow,Air Temp,Air Humidity,Light 1 Level,Light 2 Level,Dimmer Level,Relay 1 Status,Relay 2 Status,Relay 3 Status
14:30,75,4,1400,300,81,30,500,500,50,1,1,1
15:00,75.4,5,1380,0,81,50,500,500,50,1,1,0
15:30,75.8,6,1350,290,82,20,550,550,50,1,1,1

*/

/*
  When we Log the sensor data the values will be added to the dataString and then write the dataString to the dataLog.txt
  After the dataString is written we want to erase the dataString.

  This will be written as comma separated values by doing the operation shown below.

  Values that we want to add are shown above in the comments 


   dataString =String(dataString + String(sensor) + ",");


*/

void logData(){


     the_time = timeStamp ; // make this a string of the time in 24 hour format
     
     dataString =  String(dataString) + String(the_time+","); 
   
     dataString =  String(dataString) + String(temperature)+",";
     dataString =  String(dataString) + String(pH)+","; 
     dataString =  String(dataString) + String(pressure)+","; 
     dataString =  String(dataString) + String(flow)+","; 
     
     dataString =  String(dataString) + String(TempF)+","; 
     dataString =  String(dataString) + String(DHT_humidity)+","; 
     dataString =  String(dataString) + String(lux1)+","; 
     dataString =  String(dataString) + String(lux2)+",";
/*
     dataString =  String(dataString) + String(dimmer_value+",");

     dataString =  String(dataString) + String(device_1)+",";
     dataString =  String(dataString) + String(device_2)+",";
     dataString =  String(dataString) + String(device_3)+",";
  */   
      //**CHANGE THIS STRING HERE for a new file at every startup possibly
     dataFile = SD.open("datalog.txt", FILE_WRITE);

      // if the file is available, write to it:
     if (dataFile) {
          dataFile.println(dataString);
          dataFile.close();
      // print to the serial port too:
      Serial1.println(dataString);
      dataString="";
    }
     // if the file isn't open, pop up an error:
     else {
      Serial1.println("error opening datalog.txt");
    }
  
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    UPDATE DISPLAY DATA                                        // 
///////////////////////////////////////////////////////////////////////////////////////////////////

//This funtion is used to update the sensor values being shown on the display


void bUpdateDisplay(){
//  This is how you can communicate to the display without the nextion software
//  Serial.print(F("t0.txt=""));
//  Serial.print(F("This is page "));
//  Serial.print(page);
//  Serial.print(""");
//  Serial.write(0xff);
//  Serial.write(0xff);
//  Serial.write(0xff);
//Serial1.println("We are in the UpdateDisplayData Function ");
//

       
//Updating the pH variable
      dtostrf(pH, 6,2, pHTemp);
      tWater_pH_page_1.setText(pHTemp);
      tWater_pH_page_2.setText(pHTemp); 
      tWater_pH_page_3.setText(pHTemp); 
      tWater_pH_page_4.setText(pHTemp);    

//**************************************************************************************
//Updating the LPS variables  

      dtostrf(temperature, 6, 2, temperatureWTemp);
      tWater_Temp_page_1.setText(temperatureWTemp);
      tWater_Temp_page_2.setText(temperatureWTemp);
      tWater_Temp_page_3.setText(temperatureWTemp);
      tWater_Temp_page_4.setText(temperatureWTemp);    
       
//*************************************************
      dtostrf(pressure, 6, 2, levelTemp);
      tWater_Lvl_page_1.setText(levelTemp);
      tWater_Lvl_page_2.setText(levelTemp);
      tWater_Lvl_page_3.setText(levelTemp);
      tWater_Lvl_page_4.setText(levelTemp);
      
//**************************************************************************************
//Updating the FLOW variables   
      dtostrf(flow, 6, 2, flowTemp);
      tWater_Flow_page_1.setText(flowTemp);
      tWater_Flow_page_2.setText(flowTemp);
      tWater_Flow_page_3.setText(flowTemp);
      tWater_Flow_page_4.setText(flowTemp);
            
//**************************************************************************************
//Updating the DHT variables
      dtostrf(TempF, 6, 2, temperatureFTemp);
      tAir_Temp_page_1.setText(temperatureFTemp);
      tAir_Temp_page_2.setText(temperatureFTemp);
      tAir_Temp_page_3.setText(temperatureFTemp);
      tAir_Temp_page_4.setText(temperatureFTemp);  
//*************************************************  
      dtostrf(humidity, 6, 2, humidityTemp);
      tAir_Humidity_page_1.setText(humidityTemp);
      tAir_Humidity_page_2.setText(humidityTemp);
      tAir_Humidity_page_3.setText(humidityTemp);
      tAir_Humidity_page_4.setText(humidityTemp);
      
//**************************************************************************************
//Getting the lux readings   
      dtostrf(lux1, 6, 2, lux1Temp);
      tAir_Light_1_page_1.setText(lux1Temp);
      tAir_Light_1_page_3.setText(lux1Temp);
     
      dtostrf(lux2, 6, 2, lux2Temp);
      tAir_Light_2_page_2.setText(lux2Temp);  
      tAir_Light_2_page_4.setText(lux2Temp);
//**************************************************************************************
//
////Water Monitoring values
//tWater_Temp_page_2 
//tWater_Lvl_page_2 
//tWater_Flow_page_2   
//tWater_pH_page_2     
////Ambient Environment values
//tAir_Temp_page_2       
//tAir_Humidity_page_2  
//tAir_Light_2_page_2    
//      

}


  //////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        SD SETUP                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////

void setup_SD(){

Serial1.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CS)) {
Serial1.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }

//dayStamp = timeClient.getFormattedDate();
epoch = timeClient.getEpochTime();  
data_logging_file_name = "datalog_"+String(epoch)+".txt";

Serial1.println("card initialized.");



Serial1.println("trying to open datalog.txt");
  
  if(SD.exists("datalog.txt")){

      SD.remove("datalog.txt");
   }else{
 
   }
   dataFile = SD.open("datalog.txt", FILE_WRITE);


  
Serial1.println("trying to write to datalog.txt");  
  // if the file is available, we will write the first line to the CSV which is the header info:
  if (dataFile) {
    dataFile.println(headerString);
    dataFile.close();
    // print to the serial port too:
Serial1.println(headerString);
  }
  // if the file isn't open, pop up an error:
  else {
Serial1.println("error opening datalog.txt");
  }
}

  //////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        SETUP                                                 //
//////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void) {
 
Serial1.begin(9600);

timeClient.begin();
timeClient.update();
timeStamp = timeClient.getFormattedTime();



Serial1.println("The time is : "+timeStamp);

delay(1500);
  create_devices();
  setup_SD();
//Serial1.println("Setting up Nextion Stuff");
  bUpdateSensorValues();

//NexConfig.h file in ITEADLIB_Arduino_Nextion folder to set the baudrate
  nexInit();

/*
In here we attach the PopCallback functions to the buttons themselves
Register the pop event callback function of the components
*/
//LED
  bDev1_On.attachPop(bDev1_OnPopCallback, &bDev1_On);
  bDev1_Off.attachPop(bDev1_OffPopCallback, &bDev1_Off);

//Air Pump
  bDev2_On.attachPop(bDev2_OnPopCallback, &bDev2_On);
  bDev2_Off.attachPop(bDev2_OffPopCallback, &bDev2_Off);

//Water Pump
  bDev3_On.attachPop(bDev3_OnPopCallback, &bDev3_On);
  bDev3_Off.attachPop(bDev3_OffPopCallback, &bDev3_Off);

//DIMMER
  sDimmer.attachPop(sDimmerPopCallback, &sDimmer);

//DATA UPLOAD
  bUpload.attachPop(bUploadPopCallback, &bUpload);



Serial1.println("");
Serial1.println("Nextion Stuff Should be setup");

///CUT THIS PORTION OUT TO CONNECT TO WIFI FROM A BUTTON
  WiFi.begin(ssid, password);
  while ( WiFi.status() != WL_CONNECTED ) {
      delay ( 500 );  // NOT for debugging
      Serial1.print ( "." );
  }

//END HERE
    
//  bUpdateDisplay();
  
}

  //////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        LOOP                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////
void loop(void) {
  /*
   * When a pop or push event occured every time,
   * the corresponding component[right page id and component id] in touch event list will be asked.
   */

  //updating the time client to get the current time
  timeClient.update();
  timeStamp = timeClient.getFormattedTime();
  //This is the function used to monitor the Nextion display for touch input
  nexLoop(nex_listen_list);
  


 
// This updates the sensor text on the display
//  if(timeClient.getMinutes() % 1 == 0 &&timeClient.getSeconds() ==4 && trigger){
Serial1.println("now we update data");
         bUpdateSensorValues();
Serial1.println("data should be updated, now we will update the display");
         bUpdateDisplay();
Serial1.println("display should be updated");     
//  }


  //This resets the trigger to true so that we get the next update in 5 mins 
  if(timeClient.getMinutes() % 1 == 0 && timeClient.getSeconds() ==5 && trigger==false){
      trigger = true;
Serial1.println("Reset Trigger to TRUE");
  }

  //This checks the time and then if the trigger is true it will update the sensor data and set the trigger to false
  // This ensures that we only call the function once in the desired time frame
  if(timeClient.getMinutes() % 1 == 0 &&timeClient.getSeconds() ==4 && trigger){
Serial1.println("now we update sensor data");
         bUpdateSensorValues();
Serial1.println("data should be updated, now we will update the display");
         bUpdateDisplay();
Serial1.println("display should be updated, now we will log the data");         
         logData();  
Serial1.println("data should be logged");
         trigger = false;
Serial1.println("Reset Trigger to FALSE");
      
  }

  
}
