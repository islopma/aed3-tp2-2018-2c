//
// Created by Christian nahuel Rivera on 22/09/2018.
//

#include "gtest/gtest.h"
#include "../src/factorial.h"

struct FactorialTest: testing::Test{
    Factorial *factorial;

    FactorialTest(){
        factorial = new Factorial;
    }

    ~FactorialTest(){
        delete factorial;
    }
};

TEST_F(FactorialTest, whenFactorialOf1_mustReturn1)
{
    ASSERT_EQ(1, factorial->factorial(1));
}

TEST_F(FactorialTest, whenFactorialOf4_mustReturn24)
{
    ASSERT_EQ(24, factorial->factorial(4));
}
