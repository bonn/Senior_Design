
//Jordan Bonn

#include <iostream>
#include <utility>
#include <set>
#include <vector>

//This class is used to create the pitchers and then has functions to add and remove water from the buckets
class Pitchers {
public:
    
    Pitchers() {
        
    }
    
    bool dontHaveResult, isPossible;
    int  smallBucketCapacity, largeBucketCapacity;
    int  desiredAmount;
    
    
    std::vector<std::pair<int, int>> solutionNodes;
    std::vector<std::pair<int, int>>::iterator itr;
    
    std::set<std::pair<int, int>> previouslyCheckedNodes;
    
    std::pair<int, int> nextNode;
    std::pair <int, int> startingNode = std::make_pair(0, 0);
    
    //This function that can be called specify the size of the buckets, the buckets are then classifed by their size and have their initial amount set to 0
    void inputBucketSizes(int bucketA, int bucketB){
        
        if (bucketA >= bucketB){
            largeBucketCapacity = bucketA;
            smallBucketCapacity = bucketB;
            
        }
        else {
            largeBucketCapacity = bucketB;
            smallBucketCapacity = bucketA;
            
        }
        
    }
    
    //This function that can be called to specify the desired amount of water and then assigns that value to the appropiate variable
    void inputDesiredAmount(int requestedAmount) {
        desiredAmount = requestedAmount;
    }
    
    //This checks the remaining capacity of the small bucket
    int getRemainingSmallCapacity(std::pair<int, int>workingNode) {
        
        int amountInSmall = workingNode.first;
        
        return (smallBucketCapacity - amountInSmall);
    }
    
    //This checks the remaining capacity of the large bucket
    int getRemainingLargeCapacity(std::pair<int, int>workingNode) {
        
        int amountInLarge = workingNode.second;
        
        return (largeBucketCapacity - amountInLarge);
    }
    
    
    //This function will fill the smaller bucket from the river
    std::pair<int, int> fillSmall(std::pair<int, int>workingNode) {
        
        workingNode.first = smallBucketCapacity;
        
        return  workingNode;
    }
    
    //This function will fill the larger bucket from the river
    std::pair<int, int>  fillLarge(std::pair<int, int>workingNode) {
        workingNode.second = largeBucketCapacity;
        
        return workingNode;
    }
    
    //This function will empty the small bucket into the river
    std::pair<int, int> emptySmall(std::pair<int, int>workingNode) {
        workingNode.first = 0;
        
        return workingNode;
    }
    
    //This function will empty the large bucket into the river
    std::pair<int, int> emptyLarge(std::pair<int, int>workingNode) {
        workingNode.second = 0;
        
        return workingNode;
    }
    
    //This function will pour from the small bucket into the large bucket until it is full
    std::pair<int, int> pourSmallToLarge(std::pair<int, int>workingNode) {
        
        int largeBucketRoom = getRemainingLargeCapacity(workingNode);
        
        if (largeBucketRoom > workingNode.first) {
            workingNode.first = workingNode.first - workingNode.first;
            workingNode.second = workingNode.second + workingNode.first;
            
        }
        else {
            workingNode.first = workingNode.first - largeBucketRoom;
            workingNode.second = workingNode.second + largeBucketRoom;
        }
        return workingNode;
    }
    
    //This function will pour from the large bucket into the small bucket until it is full
    std::pair<int, int> pourLargeToSmall(std::pair<int, int>workingNode) {
        
        int smallBucketRoom = getRemainingSmallCapacity(workingNode);
        
        if (smallBucketRoom > workingNode.second) {
            workingNode.first = workingNode.first + workingNode.second;
            workingNode.second = workingNode.second - workingNode.second;
            
        }
        else {
            workingNode.first = workingNode.first + smallBucketRoom;
            workingNode.second = workingNode.second - smallBucketRoom;
        }
        
        return workingNode;
    }
    
    //This function checks for a duplicate pair in a set of pairs
    bool isDuplicatePair(std::pair<int, int>workingNode, std::set < std::pair<int, int>> workingset) {
        
        bool isDuplicate;
        
        for (std::set<std::pair<int, int>>::iterator it = workingset.begin();
             it != workingset.end(); ++it) {
            
            if (workingNode.first == it->first&& workingNode.second == it->second) {
                isDuplicate = true;
                
                return isDuplicate;
            }
            else {
                isDuplicate = false;
            }
        }
        
        return isDuplicate;
        
    }
    
    
    //This function gets the first avaiable node that is not already been checked
    std::pair < int, int > getNextNode(std::pair<int, int> currentNode) {
        
        int currentCapacityS = currentNode.first;
        int currentCapacityL = currentNode.second;
        
        std::pair<int, int> riverToSmall = fillSmall(currentNode);
        std::pair<int, int> riverToLarge = fillLarge(currentNode);
        std::pair<int, int> smallToRiver = emptySmall(currentNode);
        std::pair<int, int> largeToRiver = emptyLarge(currentNode);
        std::pair<int, int> smallToLarge = pourSmallToLarge(currentNode);
        std::pair<int, int> largeToSmall = pourLargeToSmall(currentNode);
        
        
        if (isDuplicatePair(smallToRiver, previouslyCheckedNodes)==false) {
            return smallToRiver;
            
        }
        else if (isDuplicatePair(largeToRiver, previouslyCheckedNodes) == false) {
            return largeToRiver;
            
        }
        else if (isDuplicatePair(riverToSmall, previouslyCheckedNodes) == false) {
            return riverToSmall;
            
            
        }
        else if (isDuplicatePair(riverToLarge, previouslyCheckedNodes) == false) {
            return riverToLarge;
            
        }
        else if (isDuplicatePair(smallToLarge, previouslyCheckedNodes) == false) {
            return smallToLarge;
            
        }
        else if (isDuplicatePair(largeToSmall, previouslyCheckedNodes) == false) {
            return largeToSmall;
        }
        else {
            std::cout << "It is not possible to obtain the desired amount of water with the jugs that are available."<< '\n';
            isPossible = false;
        }
        
    }
    
    
    //This function gets the desired amount of water by using the valid moves if it is possible
    void getDesiredResult_Depth() {
        int count = 0;
        
        nextNode = startingNode;
        dontHaveResult = true;
        
        if (smallBucketCapacity + largeBucketCapacity < desiredAmount) {
            
            
            std::cout << "The solution is not possible, the desired amount is greater than the comboned total of the Jugs. " << '\n';
            isPossible = false;
            
        }
        else {
            isPossible = true;
        }
        
        
        while (dontHaveResult&&isPossible) {
            
            count++;
            
            if (nextNode.first + nextNode.second == desiredAmount) {
                
                itr = solutionNodes.end();
                solutionNodes.insert(itr, nextNode);
                
                std::cout << '\n' << "We have reached the solution!!!" << '\n' << "It will take a total of "<< count<< " steps. " << '\n' << " Below are the steps in the solutionNodes"<<'\n';
                
                for (std::vector<std::pair<int, int>>::iterator it = solutionNodes.begin();  it != solutionNodes.end(); it++) {
                    
                    std::cout << it->first << " , " << it->second << '\n' << '\n';
                    
                }
                
                dontHaveResult = false;
                break;
            }
            
            itr = solutionNodes.end();
            
            solutionNodes.insert(itr, nextNode);
            previouslyCheckedNodes.insert(nextNode);
            
            nextNode = getNextNode(nextNode);
            
        }
        
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
