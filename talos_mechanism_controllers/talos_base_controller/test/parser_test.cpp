#include "talos_base_controller/parser.cpp"
#include <gtest/gtest.h>
#include <talos_base_controller/ArduinoMessage.h>

TEST(TestSuite, testExpectedParse)
{


	Parser p;
	p.parse("1, 0.34343, 0.13413919, 0.95381513, 4.13431, 8.343, 9.0");

	
}


int main(int argc, char** argv)
{

	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}	
