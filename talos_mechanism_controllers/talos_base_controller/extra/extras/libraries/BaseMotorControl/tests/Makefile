CC=g++
INCLUDEPATHS=-I./ -I../
LIBRARIES=-lgtest -lpthread
LIBRARYPATHS=-L./
OUTPUT=-o tests.out
tests: test_basemtrcontrol.cpp
	$(CC) $(INCLUDEPATHS) $(OUTPUT) test_basemtrcontrol.cpp ../basemtrcontrol.cpp $(LIBRARYPATHS) $(LIBRARIES)
	./tests.out
	
