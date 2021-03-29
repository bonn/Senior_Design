//Copyright Patrick Schneider 
#include <ESP8266WiFi.h>
#include <Nextion.h>
#include "NexText.h"
#include "NexButton.h"
#include "NexSlider.h" 

#define DHTPIN 4     // what digital pin we're connected to

int count = 0;

// Define LED, AIR PUMP, WATER PUMP pins
const int EX1 = 5;
const int EX2 = 16;
const int EX3 = 4;
const int UD1 = 15;

float temperature = 78.98;
float pressure = 75.98;
float flow = 74.98;
float pH = 77.98;
float TempF = 79.98;
float humidity = 28.98;
float lux1 = 58.98;
float lux2 = 58.98;

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                   TOUCH SCREEN OBJECTS                                       //
//////////////////////////////////////////////////////////////////////////////////////////////////

//Nextion objects - Example (page id = 0, component id = 1, component name = "b0") 

//Water Monitoring values 
NexText tWater_Temp  = NexText (1, 11, "WaterTemp");  //Text with water temperature value.
NexText tWater_Lvl   = NexText (1, 12, "WaterLvl");   //Text with water level value. 
NexText tWater_Flow  = NexText (1, 13, "WaterFlow");  //Text with water flow value. 
NexText tWater_pH    = NexText (1, 14, "pH");         //Text with water pH value. 

//Ambient Environment values 
NexText tAir_Temp      = NexText (1, 15, "tAir_Temp");       //Text with air temperature value.
NexText tAir_Humidity  = NexText (1, 16, "tAir_Humidity");   //Text with humidity value.
NexText tAir_Light_1   = NexText (1, 17, "Light1");          //Text with light level 1 value.
NexText tAir_Light_2   = NexText (3, 10, "Light2");          //Text with light level 2 value. 

//External Devices 
NexButton bDev1_On  = NexButton (9, 10, "bDev1_On");   //Button to turn Dev1 ON (LED - EX1)
NexButton bDev1_Off = NexButton (9, 11, "bDev1_off");  //Button to turn Dev1 OFF (LED - EX1)
NexText   tState1   = NexText   (9, 9,  "tState1");    //Text to show Dev1 ON/OFF status.
NexButton bDev2_On  = NexButton (9, 10, "bDev2_On");   //Button to turn Dev2 ON (AIR PUMP - EX2)
NexButton bDev2_Off = NexButton (9, 11, "bDev2_off");  //Button to turn Dev2 OFF (AIR PUMP - EX2)
NexText   tState2   = NexText   (9, 9,  "tState2");    //Text to show Dev2 ON/OFF status.
NexButton bDev3_On  = NexButton (9, 10, "bDev3_On");   //Button to turn Dev3 ON (WATER PUMP - EX3)
NexButton bDev3_Off = NexButton (9, 11, "bDev3_off");  //Button to turn Dev3 OFF (WATER PUMP - EX3)
NexText   tState3   = NexText   (9, 9,  "tState3");    //Text to show Dev3 ON/OFF status.

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
//////////////////////////////////////////////////////////////////////////////////////////////////
//                                   EXTERNAL DEVICE CONTROLS                                   //
//////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Button bDev1_On component pop callback function. 
 * When the ON button is released, the LED turns on and the tState1 text changes. 
 */
void bDev1_OnPopCallback(void *ptr) {
  tState1.setText("State: on"); 
  digitalWrite(EX1, HIGH);
}
/*
 * Button bDev1_Off component pop callback function. 
 * When the OFF button is released, the LED turns off and the tState1 text changes. 
 */
void bDev1_OffPopCallback(void *ptr) {
  tState1.setText("State: off"); 
  digitalWrite(EX1, LOW);
}
/*
 * Button bDev2_On component pop callback function. 
 * When the ON button is released, the AIR PUMP turns on and the tState2 text changes. 
 */
void bDev2_OnPopCallback(void *ptr) {
  tState2.setText("State: on"); 
  digitalWrite(EX2, HIGH);
}
/*
 * Button bDev2_Off component pop callback function. 
 * When the OFF button is released, the AIR PUMP turns OFF and the tState2 text changes. 
 */
void bDev2_OffPopCallback(void *ptr) {
  tState1.setText("State: off"); 
  digitalWrite(EX2, LOW);
}
/*
 * Button bDev3_On component pop callback function. 
 * When the ON button is released, the WATER PUMP turns on and the tState3 text changes. 
 */ 
void bDev3_OnPopCallback(void *ptr) {
  tState3.setText("State: on"); 
  digitalWrite(EX3, HIGH);
}
/*
 * Button bDev3_Off component pop callback function. 
 * When the OFF button is released, the WATER PUMP turns OFF and the tState3 text changes. 
 */
void bDev3_OffPopCallback(void *ptr) {
  tState3.setText("State: off"); 
  digitalWrite(EX3, LOW);
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
  //Signal to Changed brightness of EXTERNAL DEVICE LED1 
  analogWrite(EX1, number); 
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//                                  SENSOR DATA UPLOADING                                       //
//////////////////////////////////////////////////////////////////////////////////////////////////

void bUploadPopCallback(void *ptr) {
  digitalWrite(UD1, HIGH);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                  UPDATING SENSOR VARIABLES                                   //
////////////////////////////////////////////////////////////////////////////////////////////////// 


  




//////////////////////////////////////////////////////////////////////////////////////////////////
//                               UPDATING SENSOR DATA ON SCREEN                                 //
////////////////////////////////////////////////////////////////////////////////////////////////// 
//Water Environment Sensor Update
void bUpdateSensor(){


static char temperatureWTemp[6];
dtostrf(temperature, 6, 2, temperatureWTemp);
tWater_Temp.setText(temperatureWTemp);

static char levelTemp[6];
dtostrf(pressure, 6, 2, levelTemp);
tWater_Lvl.setText(levelTemp);

static char flowTemp[6];
dtostrf(flow, 6, 2, flowTemp);
tWater_Flow.setText(flowTemp);

static char pHTemp[6];
dtostrf(pH, 6,2, pHTemp);
tWater_pH.setText(pHTemp);

//Ambient Environment Sensor Update

static char temperatureFTemp[6];
dtostrf(TempF, 6, 2, temperatureFTemp);
tAir_Temp.setText(temperatureFTemp);

static char humidityTemp[6];
dtostrf(humidity, 6, 2, humidityTemp);
tAir_Humidity.setText(humidityTemp);

static char lux1Temp[6];
dtostrf(lux1, 6, 2, lux1Temp);
tAir_Light_1.setText(lux1Temp);

static char lux2Temp[6];
dtostrf(lux2, 6, 2, lux2Temp);
tAir_Light_2.setText(lux2Temp);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//                                        MAIN                                                  //
////////////////////////////////////////////////////////////////////////////////////////////////// 

void setup(void) {    

  Serial.begin(9600);
  
    
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

   
//Set EXTERNAL DEVICES as outputs
  pinMode(EX1, OUTPUT);
  pinMode(EX2, OUTPUT);
  pinMode(EX3, OUTPUT);
  pinMode(UD1, OUTPUT);
  
}

void loop(void) {   
  /*
   * When a pop or push event occured every time,
   * the corresponding component[right page id and component id] in touch event list will be asked.
   */
  count = count+1;
  nexLoop(nex_listen_list);


  
  // Serial.print(" The nexLoop is at ");
  // Serial.println(count);
}
