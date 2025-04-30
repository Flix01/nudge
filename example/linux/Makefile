# Makefile for Linux/MacOS

CXX = c++
CXXFLAGS = -I../../ -march=native -fno-rtti -fno-exceptions -O3 -Wall
LDFLAGS = -lglut -lGL
SOURCES = ../stdafx.cpp
TARGETS = example example01 example02 example03

all: $(TARGETS)

example: ../main.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) ../main.cpp $(SOURCES) -o $@ $(LDFLAGS)

example01: ../example01.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) ../example01.cpp -o $@

example02: ../example02.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) ../example02.cpp $(SOURCES) -o $@ $(LDFLAGS)
	
example03: ../example03.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) -DEXAMPLE03_CPP -I../example03_include/ ../example03.cpp $(SOURCES) -o $@ $(LDFLAGS)	

clean:
	rm -f $(TARGETS)
	
