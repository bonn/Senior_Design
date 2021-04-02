/*
 * Rui Santos 
 * Complete Project Details http://randomnerdtutorials.com
 */


#include <multi_channel_relay.h>

#include <Nextion.h>
#include "NexText.h"
#include "NexButton.h"


#include "DHTesp.h"
#include <Adafruit_LPS35HW.h>
#include <multi_channel_relay.h>



//#define DHTPIN 4     // what digital pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)


// Initialize DHT sensor.
//DHT dht(DHTPIN, DHTTYPE);

int count = 0;



// LED pins
const int led1 = 5;
const int led2 = 16;



Multi_Channel_Relay relay;

// Declare your Nextion objects - Example (page id = 0, component id = 1, component name = "b0") 
/*
NexButton bOn = NexButton(0, 2, "bOn");
NexButton bOff = NexButton(0, 3, "bOff");
NexSlider h0 = NexSlider(0, 5, "h0");
NexText tSlider = NexText(0, 6, "tSlider");
NexText tTempC = NexText(1, 5, "tTempC");
NexText tTempF = NexText(1, 4, "tTempF");
NexProgressBar jHumidity = NexProgressBar(1, 8, "jHumidity");
NexText tHumidity = NexText(1, 9, "tHumidity");
*/


//Water Monitoring values 
NexText tWater_Temp  = NexText (1, 11, "WaterTemp");  //Text with water temperature value
NexText tWater_Lvl   = NexText (1, 12, "WaterLvl");   //Text with water level value 
NexText tWater_Flow  = NexText (1, 13, "WaterFlow");  //Text with water flow value 
NexText tWater_pH    = NexText (1, 14, "pH ");        //Text with water pH value 

//Ambient Environment values 
NexText tAir_Temp      = NexText (1, 15, "tAir_Temp");       //Text with air temperature value
NexText tAir_Humidity  = NexText (1, 16, "tAir_Humidity");   //Text with humidity value 
NexText tAir_Light_1   = NexText (1, 17, "Light1");        //Text with light level 1 value
NexText tAir_Light_2   = NexText (3, 10, "Light2");        //Text with light level 2 value 

//External Devices 
//FOR DEMO////
NexButton bDev_On = NexButton (9, 10, "bDev_On");
NexButton bDev_Off = NexButton (9, 11, "bDev_off");
NexText tState = NexText(9, 9, "tState"); 
//////////
NexButton bDev_1  = NexButton (9, 1, "Dev1");        // Button to turn ON/OFF (LED 1)
NexButton bDev_2  = NexButton (9, 4, "Dev2");        // Button to turn ON/OFF (AIR PUMP)
NexButton bDev_3  = NexButton (9, 5, "Dev3");        //Button to turn ON/OFF (WATER PUMP)
NexButton bDev_4  = NexButton (9, 6, "Dev4");        //Button to turn ON/OFF (OPEN)

//Update sensors
NexButton bUpdate = NexButton(1,18, "bUpdate");





// Register a button object to the touch event list.  
NexTouch *nex_listen_list[] = {
/////////////////////////////
///////////DEMO/////////////
  &bDev_On,
  &bDev_Off,
  &bUpdate,
//////////////////////////  
  &bDev_1,
  &bDev_2,
  &bDev_3,
  &bDev_4,
 
  NULL
};
 
/*
 * Button bOn component pop callback function. 
 * When the ON button is released, the LED turns on and the state text changes. 
 */
void bDev_OnPopCallback(void *ptr) {
  tState.setText("State: on"); //<-----------make text box with state

  Serial.println("I am in  bDev_On PopCallback");
  //digitalWrite(led1, HIGH);
    relay.turn_on_channel(1);  
}

/*
 * Button bOff component pop callback function. 
 * When the OFF button is released, the LED turns off and the state text changes. 
 */
void bDev_OffPopCallback(void *ptr) {
  tState.setText("State: off"); //<-------------------same text box updated
  //digitalWrite(led1, LOW);
   relay.turn_off_channel(1);
}

/*
 * Slider h0 component pop callback function. 
 * When the slider is released, the LED brightness changes and the slider text changes. 
 */
//void h0PopCallback(void *ptr) {
//  uint32_t number = 0;
 // char temp[10] = {0};
  // change text with the current slider value
 // h0.getValue(&number);
 // utoa(number, temp, 10);
 // tSlider.setText(temp);
  // change LED brightness
  //analogWrite(led2, number); 
//}

/*
 * Button bUpdate component pop callback function. 
 * When the UPDATE button is released, the temperature and humidity readings are updated. 
 */
void bUpdatePopCallback(void *ptr) {

  
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = 20.0;
  // Read temperature as Celsius (the default)
  float t = 10.0;
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = 76.45;


  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    return;
  }
  
//// Update temperature in Celsius
  //static char temperatureCTemp[6];
  //dtostrf(t, 6, 2, temperatureCTemp);
  //tTempC.setText(temperatureCTemp);


  // Update humidity percentage text and progress bar
  char hTemp[10] = {0}; 
  utoa(int(h), hTemp, 10);
  tAir_Humidity.setText(hTemp);
  

  // Update temperature in Fahrenheit
  static char temperatureFTemp[6];
  dtostrf(f, 6, 2, temperatureFTemp);
  tAir_Temp.setText(temperatureFTemp);
}

void setup(void) {    
//  dht.begin();

relay.begin(0x11); 
  Serial.begin(9600);
       

  // You might need to change NexConfig.h file in your ITEADLIB_Arduino_Nextion folder
  // Set the baudrate which is for debug and communicate with Nextion screen
  nexInit();

  // Register the pop event callback function of the components
  bDev_On.attachPop(bDev_OnPopCallback, &bDev_On);
  bDev_Off.attachPop(bDev_OffPopCallback, &bDev_Off);
  //h0.attachPop(h0PopCallback);
  bUpdate.attachPop(bUpdatePopCallback, &bUpdate);
    
  // Set LEDs as outputs
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

}

void loop(void) {   
  /*
   * When a pop or push event occured every time,
   * the corresponding component[right page id and component id] in touch event list will be asked.
   */
  count = count+1;
  
relay.begin(0x11); 
  nexLoop(nex_listen_list);
  
//Serial.print(" The nexLoop is at ");

// Serial.println(count);
}
