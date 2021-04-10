/*
    GAIA Software
    
    Created by Patrick Schneider, Brandon Sanders, and Jordan Bonn
*/

/*
 * NOTE:
  Serial1 is used for debugging because the Tx and Rx from USB are the same as Serial

  Serial1 is port D4 on the ESP8266

  NODE MCU references: https://www.make-it.ca/nodemcu-arduino/nodemcu-details-specifications/
                       https://robu.in/a-beginner-guide-to-nodemcu/
  
*/


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Libraries to Import                                    // 
///////////////////////////////////////////////////////////////////////////////////////////////////
// Patrick Schneider
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
#define DHTPIN D0               // what digital pin we're connected to


// chip select pin for the SD Card
const int SD_CS = 4;

//Pin that is used for the water flow sensor
const byte water_flow_pin = 10;            // GPIO pin for the water flow sensor


//datastring created for logging files
String     dataString = "";

//variables for the LPS water temp and pressure sensor
float temperature;
float pressure;

//variable for the flow sensor
float flow;

//variable for the pH sensor
float pH;

//variables for the DHT temp sensor
float TempF;
float humidity;

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
 //                                       Custom Functions                                        // 
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

//This functions creates all of the objects for the varios sensors used
void create_devices()
{
 Serial1.println("Let's start up some sensors!");
//////////////////////////////////////
//Starting the Relay
relay.begin(0x11);

delay(500);
//////////////////////////////////////
//Starting LPS35HW device
 if (!lps35hw.begin_I2C()) {
    Serial1.println("Couldn't find LPS35HW chip");
    while (1);
  }
  Serial1.println("Found LPS35HW chip");
  
delay(500);  
//////////////////////////////////////
//Starting DHT Device
dht.setup(DHTPIN, DHTesp::DHT22); // Connect DHT sensor to Pin defined earlier

delay(500);
//////////////////////////////////////
//Starting FLOW SENSOR
pinMode(water_flow_pin, INPUT);           //Sets the pin as an input
attachInterrupt(water_flow_pin , flow_interrupt, RISING);  //Configures interrupt 0 (pin 2 on the Arduino Uno) to run the function "Flow"

delay(500);
//////////////////////////////////////
//Starting the Light Sensors
  if(light.begin())
    Serial1.println("Ready to sense some light!"); 
  else
    Serial1.println("Could not communicate with the sensor!");

    if(light_2.begin())
    Serial1.println("Ready to sense some light!"); 
  else
    Serial1.println("Could not communicate with the sensor_2!");

delay(500);
//////////////////////////////////////
//Starting the DAC

 if(!dac.resetOutput())
   {
      Serial1.println("Error talking to DAC. Check wiring.");
   }else{
      Serial1.println("Was able to talk to DAC. No Issue.");
    }

}





  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                  TOUCH SCREEN OBJECTS (TEXT BOXES AND BUTTONS)                                // 
///////////////////////////////////////////////////////////////////////////////////////////////////


//Nextion objects - Example (page id = 0, component id = 1, component name = "b0")

//Water Monitoring values
NexText tWater_Temp  = NexText (1, 11, "WaterTemp");  //Text with water temperature value.
NexText tWater_Lvl   = NexText (1, 12, "WaterLvl");   //Text with water level value.
NexText tWater_Flow  = NexText (1, 13, "WaterFlow");  //Text with water flow value.
NexText tWater_pH    = NexText (1, 14, "tWater_pH");         //Text with water pH value.

//Ambient Environment values
NexText tAir_Temp      = NexText (1, 15, "tAir_Temp");       //Text with air temperature value.
NexText tAir_Humidity  = NexText (1, 16, "tAir_Humidity");   //Text with humidity value.
NexText tAir_Light_1   = NexText (1, 17, "tAir_Light_1");          //Text with light level 1 value.
NexText tAir_Light_2   = NexText (3, 10, "tAir_Light_2");          //Text with light level 2 value.

//External Devices
NexButton bDev1_On  = NexButton (9, 6,  "bDev1_On");   //Button to turn Dev1 ON (LED - EX1)
NexButton bDev1_Off = NexButton (9, 7,  "bDev1_Off");  //Button to turn Dev1 OFF (LED - EX1)
NexText   tState1   = NexText   (9, 5,  "tState1");    //Text to show Dev1 ON/OFF status.
NexButton bDev2_On  = NexButton (9, 11, "bDev2_On");   //Button to turn Dev2 ON (AIR PUMP - EX2)
NexButton bDev2_Off = NexButton (9, 12, "bDev2_Off");  //Button to turn Dev2 OFF (AIR PUMP - EX2)
NexText   tState2   = NexText   (9, 10, "tState2");    //Text to show Dev2 ON/OFF status.
NexButton bDev3_On  = NexButton (9, 15, "bDev3_On");   //Button to turn Dev3 ON (WATER PUMP - EX3)
NexButton bDev3_Off = NexButton (9, 16, "bDev3_Off");  //Button to turn Dev3 OFF (WATER PUMP - EX3)
NexText   tState3   = NexText   (9, 14, "tState3");    //Text to show Dev3 ON/OFF status.

//Dimming Controls
NexSlider sDimmer = NexSlider (9, 3, "sDimmer");      //Slider bar to ontrol EX1 - LED intensity.
NexText   tDval   = NexText   (9,19, "tDval");        //Text to show value the slider is holding.

//Upload sensor data sensors
NexButton bUpload = NexButton(9,18, "bUpload");   //Button to upload data (UD1)

// Register a button object to the touch event list.
NexTouch *nex_listen_list[] = {

  &bDev1_On,
  &bDev1_Off,
  &bDev2_On,
  &bDev2_Off,
  &bDev3_On,
  &bDev3_Off,
  &sDimmer,
  &bUpload,

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
  Serial1.println("Turn off Relay Channel 1");
  relay.turn_off_channel(1);  

}
/*
 * Button bDev2_On component pop callback function.
  * This button turns on channel 2 of the relay
 */
void bDev2_OnPopCallback(void *ptr) {
  tState2.setText("State: on");
  Serial1.println("Turn on Relay Channel 2");
  relay.turn_on_channel(2);
 
}
/*
 * Button bDev2_Off component pop callback function.
 * This button turns off channel 2 of the relay
 */

void bDev2_OffPopCallback(void *ptr) {
  tState2.setText("State: off");
  Serial1.println("Turn off Relay Channel 2");
  relay.turn_off_channel(2);
  
}
/*
 * Button bDev3_On component pop callback function.
  * This button turns on channel 3 of the relay
 */
void bDev3_OnPopCallback(void *ptr) {
  tState3.setText("State: on");
  Serial1.println("Turn on Relay Channel 3");
  relay.turn_on_channel(3);
    
}
/*
 * Button bDev3_Off component pop callback function.
 * This button turns off channel 3 of the relay
 */
void bDev3_OffPopCallback(void *ptr) {
  tState3.setText("State: off");
  Serial1.println("Turn off Relay Channel 3");
  relay.turn_off_channel(3);
  
}
/*
 * SLider sDimmer component pop call back function
 * SLider controls a value that is sent to DAC via ESP to update the intensity of the light.
 * The Slider Text will be updated with the value the slider holds.
 */
void sDimmerPopCallback(void *ptr){
  uint32_t number = 0;
  char temp[10] = {0};

  //Change dimmer value text with the current slider value
  sDimmer.getValue(&number);
  utoa(number, temp, 10);
  tDval.setText(temp);

    Serial1.print("We are in the Dimmer callbak and the number value is: ");
    Serial1.println(255*(number/100));
    
  //This function sets the DAC output to the correct level( 8 bit unsinged int; 0-255 levels )
  //The slider goes from 0-100% so we make the number and turn it into a percentage to muliply by 255
  dac.setOutput(255*(number/100));


}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Uploading Sensor Data Button                                       // 
///////////////////////////////////////////////////////////////////////////////////////////////////

void bUploadPopCallback(void *ptr) {

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
   Serial1.println("Inside the upload function");

     bUpdateSensor();

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial1.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial1.println("error opening datalog.txt");
  }
}


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   UPDATE SENSOR VARIABLES                                     // 
///////////////////////////////////////////////////////////////////////////////////////////////////

      // We decided to update the sensor values as we update the data on the display

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    UPDATE DISPALY DATA                                        // 
///////////////////////////////////////////////////////////////////////////////////////////////////

//We need to update the data string that will be uploaded when we pull the new data
//The format of the data string is shown below
/*
CSV FORMAT:
Date/Time, pH Reading, Water Temp, Water Level, Water Flow, Ambient Temp, Ambient Hum, Lux 1, Lux 2

*/
void bUpdateSensor(){

Serial1.println("We are in the UpdateSensor Function ");
delay(500);  // for debugging  
Serial1.println("Starting the pH section");
delay(500);  // for debugging
      //This reads the analog input and converts it to pH
      pH_Sensor_Value = analogRead(A0);
      pH_Sensor_Voltage = pH_Sensor_Value * (3.3 / 1023.0);
      pH_Sensor_Reading = (( -5.6548 * pH_Sensor_Voltage) + 15.509);
      pH =  pH_Sensor_Reading;     
      static char pHTemp[6];
      dtostrf(pH, 6,2, pHTemp);
      tWater_pH.setText(pHTemp);   
// dataString =String(dataString + String(sensor) + ",");
Serial1.println("We completed in the pH Section");
delay(500);  // for debugging
Serial1.print("The Value is: ");
Serial1.println(pH);


Serial1.println("Starting the LPS TEMP section");
delay(500);  // for debugging
      //Getting LPS35 water Temp
      temperature = lps35hw.readTemperature()* 9/5 + 32;
      static char temperatureWTemp[6];
      dtostrf(temperature, 6, 2, temperatureWTemp);
      tWater_Temp.setText(temperatureWTemp);
Serial1.println("We completed in the water temp Section");
Serial1.print("The Value is: ");
Serial1.println(temperature);
delay(1500);  // for debugging



Serial1.println("Starting the LPS pressure section");
  delay(500);  // for debugging
      //Getting LPS53 Pressure data
      pressure = lps35hw.readPressure();
      static char levelTemp[6];
      dtostrf(pressure, 6, 2, levelTemp);
      tWater_Lvl.setText(levelTemp);
Serial1.println("We completed in the water pressure Section");
Serial1.print("The Value is: ");
Serial1.println(levelTemp);



Serial1.println("Starting the FLOW  section");
delay(1500);  // for debugging
      //Getting Water Flow
        flow =  get_flow();
        static char flowTemp[6];
        dtostrf(flow, 6, 2, flowTemp);
        tWater_Flow.setText(flowTemp);
Serial1.println("We completed in the water flow Section");
Serial1.println("We are starting the  flow");
delay(1500);  // for debugging

      //Ambient Environment Sensor Update
Serial1.println("We are starting the DHT TEMP");
delay(1500);  // for debugging

      //Getting the DHT Temp
      TempF = dht.toFahrenheit(dht.getTemperature()); //69.69;
      static char temperatureFTemp[6];
      dtostrf(TempF, 6, 2, temperatureFTemp);
      tAir_Temp.setText(temperatureFTemp);
Serial1.println("We completed in the air temp Section");
Serial1.println(TempF);
delay(1500);  // for debugging

Serial1.println("We are starting the DHT HUM");
delay(500);
      //Getting the DHT Humidity
      humidity = dht.getHumidity();
      static char humidityTemp[6];
      dtostrf(humidity, 6, 2, humidityTemp);
      tAir_Humidity.setText(humidityTemp);


Serial1.println("We completed in the air hum Section");

Serial1.println(humidity);
delay(1500);  // for debugging

      //Getting the lux readings
Serial1.println("We are starting the LUX 1");
delay(500);
      lux1 = light.readLight();
      static char lux1Temp[6];
      dtostrf(lux1, 6, 2, lux1Temp);
      tAir_Light_1.setText(lux1Temp);
Serial1.println("We completed in the lux_1 temp Section");


Serial1.print("The Value is: ");
Serial1.println(lux1);
delay(500);
Serial1.println("We are starting the LUX 2");    
delay(500);  // for debugging
      lux2 = light_2.readLight();
      static char lux2Temp[6];
      dtostrf(lux2, 6, 2, lux2Temp);
      tAir_Light_2.setText(lux2Temp);
Serial1.println("We completed in the lux_2  Section");

Serial1.print("The Value is: ");
Serial1.println(lux2);
delay(500);  // for debugging

}



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                     SETUP                                                //
//////////////////////////////////////////////////////////////////////////////////////////////////

void setup(void) {
 
Serial1.begin(9600);
 create_devices();
delay(500);   // for debugging
Serial1.println("Setting up Nextion Stuff");
delay(500);   // for debugging

//NexConfig.h file in ITEADLIB_Arduino_Nextion folder to
//set the baudrate
  nexInit();

//Register the pop event callback function of the components
//EX1
  bDev1_On.attachPop(bDev1_OnPopCallback, &bDev1_On);
  bDev1_Off.attachPop(bDev1_OffPopCallback, &bDev1_Off);

//EX2
  bDev2_On.attachPop(bDev2_OnPopCallback, &bDev2_On);
  bDev2_Off.attachPop(bDev2_OffPopCallback, &bDev2_Off);

//EX3
  bDev3_On.attachPop(bDev3_OnPopCallback, &bDev3_On);
  bDev3_Off.attachPop(bDev3_OffPopCallback, &bDev3_Off);

//EX1 DIMMER
  sDimmer.attachPop(sDimmerPopCallback, &sDimmer);

//DATA UPLAOD
  bUpload.attachPop(bUploadPopCallback, &bUpload);

Serial1.println("");
Serial1.println("Nextion Stuff Should be setup");
  WiFi.begin(ssid, password);
  while ( WiFi.status() != WL_CONNECTED ) {
      delay ( 500 );  // NOT for debugging
      Serial1.print ( "." );
  }
  timeClient.begin(); 
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                     LOOP                                                //
//////////////////////////////////////////////////////////////////////////////////////////////////


void loop(void) {
  /*
   * When a pop or push event occured every time,
   * the corresponding component[right page id and component id] in touch event list will be asked.
   */

  //updating the time client to get the current time
  timeClient.update();

  //This is the function used to monitor the Nextion display for touch input
  nexLoop(nex_listen_list);
  
  
  //This resets the trigger to true so that we get the next update in 5 mins 
  if(timeClient.getMinutes() % 1 == 0&&timeClient.getSeconds()==5&&trigger==false){
      trigger = true;
Serial1.println("Reset Trigger to TRUE");
  }


  //This checks the time and then if the trigger is true it will update the sensor data and set the trigger to false
  // This ensures that we only call the function once in the desired time frame
  if(timeClient.getMinutes() % 1 == 0 &&timeClient.getSeconds()==4 && trigger){
Serial1.println("now we update data");
         bUpdateSensor();
Serial1.println("data should be updated");
         trigger = false;
Serial1.println("Reset Trigger to FALSE");
      
  }

  
}
