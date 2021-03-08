
//Jordan Bonn

#include <multi_channel_relay.h>

#include <Wire.h>
#include <Max517Dac.h>

  /**
   *  channle: 4 3 2 1
   *  state: 0b0000 -> 0x00  (all off)
   *  state: 0b1111 -> 0x0f   (all on)
  */

Multi_Channel_Relay relay;
Max517Dac   dac;

//This class is used to create the pitchers and then has functions to add and remove water from the buckets
class External_Devices {
public:

    relay.begin(0x11);

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


int main()

{



    External_Devices external;

    external.turn_on_dev(1);

    external.get_relay_status(1);

    external.turn_off_dev(1)

    external.set_light_level(255);

    external.get_light_level();

    return 0;
}
