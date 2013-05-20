# Original MakeFile by Steven Erenst

#Compiler
CXX := g++

#Executable to compile
EXECUTABLE := objectTrainer 

#Compiler Flags
CFLAGS := -std=c++0x -fopenmp -g -Wall -I/usr/local/boost_1_53_0/

#Linker Flags
LDFLAGS := -g 

#Opencv pkg-config
CONFIG := `pkg-config --libs --cflags opencv`

#BOOST Libraries
BOOST := -L/usr/local/boost/stage/lib -lboost_filesystem -lboost_system

#Source files
SRC := $(wildcard src/*.cpp)

#Object Files 
OBJ := $(subst src/, obj/, $(SRC:.cpp=.o))

#Prevent echoing of commands
.SILENT :

#Rule to link executable
$(addprefix bin/, $(EXECUTABLE)) : $(OBJ)
	echo "	LD	$@"
	mkdir -p bin/ output/
	$(CXX) $(LDFLAGS) -o $@ $(OBJ) $(CONFIG) $(BOOST)
	echo "Build Succeeded"

#Rule to generate .d and .o files from .cpp files
obj/%.o : src/%.cpp
	echo "	CXX	$<"
	mkdir -p obj/
	$(CXX) $(CFLAGS) -o $@ -c -MMD -MP $<

#include dependency files
-include $(patsubst src/%.cpp, obj/%.d, $(SRC))  

#Rule to clean generated files
.PHONY : clean
clean  :
	$(RM) -r bin/ obj/
