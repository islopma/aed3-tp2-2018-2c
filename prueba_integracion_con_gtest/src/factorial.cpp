//
// Created by Christian nahuel Rivera on 22/09/2018.
//

#include "factorial.h"

int Factorial::factorial(int number) {
    if(number== 1){
        return 1;
    }else{
        return number*factorial(number-1);
    }
}
