#include "analysis.h"

#include <iostream> // Only here for showing the code is working
#include <sstream>
#include <vector>
using namespace std;
namespace analysis {

//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
unsigned int countCharacters(std::string sentence){
    unsigned int count = 0;
     for(int i=0; sentence[i] != '\0'; i++){
        // if(sentence[i]!=' ')// this condition is used to avoid counting space
        // {
            count++;
        //}
    }
    return count;
}

//! @todo
//! TASK 5 - Refer to README.md and the Header file for full description
int getNumber(std::string sentence){
    vector<int> numberList;
    unsigned int num = 0;
    stringstream ss;    
    /* Storing the whole string into string stream */
    ss << sentence;
    /* Running loop till the end of the stream */
    string temp;
    int found;
    while (!ss.eof()) {
  
        /* extracting word by word from stream */
        ss >> temp;
  
        /* Checking the given word is integer or not */
        if (stringstream(temp) >> found)
            {
                cout << found << " "<<endl;
                numberList.push_back(found);
            }
        /* To save from space at the end of string */
        temp = "";
    }
    if (numberList.size()>0)
    {
        num = numberList.at(0);
    }
    
    return num;

}

}
