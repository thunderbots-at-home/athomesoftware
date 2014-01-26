COMPILER=g++
GTEST_PATH=/home/marco/catkin_ws/src/athomesoftware/gtest-1.7.0/
GTEST_HEADERS=$(GTEST_PATH)/include/
GTEST_LIBS=$(GTEST_PATH)/bin/
TEST_SRCS=test_basemtrcontrol.cpp
SRCS=../basemtrcontrol.cpp

.PHONY : test
test : $(TEST_SRCS)
	$(COMPILER) $(TEST_SRCS) -I.. -I$(GTEST_HEADERS) -L$(GTEST_LIBS) -lgtest_main -lgtest -lpthread basemtrcontrol.o -o basemtrcontroller.tests

.PHONY : build 
build: $(SRCS)
	$(COMPILER) $(SRCS) -I.. -c
