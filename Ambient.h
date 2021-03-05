
//Jordan Bonn

#include <iostream>
#include <utility>
#include <set>
#include <vector>

//This class is used to create the pitchers and then has functions to add and remove water from the buckets
class Ambient {
public:
    
    Ambient() {
        
    }
    
 
    float Air_Temp, Air_Humidity;

    uint8_t Light_1,Light_2



    float get_AirTemp(){


        return Air_Temp;
    }

    float get_AirHum(){

        return Air_Humidity;
    }

    uint8_t get_LightLevel_1(){

        return Light_1;
    }

    uint8_t get_LightLevel_2(){

        return Light_2;
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
