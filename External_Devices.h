
//Jordan Bonn

#include <multi_channel_relay.h>

  /** 
   *  channle: 4 3 2 1
   *  state: 0b0000 -> 0x00  (all off)
   *  state: 0b1111 -> 0x0f   (all on)
  */ 

Multi_Channel_Relay relay;

//This class is used to create the pitchers and then has functions to add and remove water from the buckets
class External_Devices {
public:
    
    relay.begin(0x11); 

    External_Devices() {
        
    }
    
    
    bool Dev_1,Dev_2,Dev_3,Dev_4;

    float desired_light_level;
    int device;


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

//This function is called to turn on a device connected to the relay.
//It is passed an int between 1-4 and turns off the appropiate device
    void get_relay_status(device){

        relay.turn_on_channel(device);

        return;
    }

    
    uint8_t get_LightLevel_1(){

        return Light_1;
    }

    
};


int main()

{
    int x, y, z;
    
    std::cout << "Please enter the size of the first pitcher." << '\n';
    
    std::cin >> x;
    
    std::cout << "Please enter the size of the second pitcher." << '\n';
    
    std::cin >> y;
    
    std::cout << "Please enter the requested amount of water." << '\n';
    
    std::cin >> z;
    
    
    Pitchers pitchers;
    
    pitchers.inputBucketSizes(x, y);
    pitchers.inputDesiredAmount(z);
    pitchers.getDesiredResult_Depth();
    
    return 0;
}
