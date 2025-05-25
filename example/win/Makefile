# Makefile for cross compile examples using x86_64-w64-mingw32-g++ on Linux/MacOS

CXX = x86_64-w64-mingw32-g++
CXXFLAGS = -I../../ -march=native -fno-rtti -fno-exceptions -O3 -Wall
LDFLAGS =  -static-libgcc -static-libstdc++  -lopengl32
SOURCES = ../stdafx.cpp
TARGETS = example example01 example02 example03
INCLUDE_GLUT = -I./
LINK_GLUT = -L./ -lfreeglut # library already present in the original nudge repository
INCLUDE_GLEW = -I./
LINK_GLEW = -L./ -lglew32 # library downloaded here: https://sourceforge.net/projects/glew/files/glew/2.1.0/glew-2.1.0-win32.zip/download
 
# Clarifications:
# On Ubuntu 24.04:  
# (a) I installed packages: g++-mingw-w64-x86-64 and gcc-mingw-w64-x86-64. I've ignored files with the i686 suffix (for Windows 32-bit compilations)
# (b) Windows default headers and libraries are here:
# -I/usr/x86_64-w64-mingw32/include -L/usr/x86_64-w64-mingw32/lib
# but are already included by default
# -static-libgcc -static-libstdc++ # these are used to remove dependencies to additional .dll files
# -Wl,-Bstatic -lstdc++ -lpthread -Wl,-Bdynamic # This syntax is not used (it tells compiler to use the static versions of the in-between libraries)

# -Wno-format or -D__USE_MINGW_ANSI_STDIO=1 are something I've exploring to suppress format warnings arising 
# from behavior of stdio in msvcrt.dll (in case they're present)

all: $(TARGETS)

example: ../main.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDE_GLUT) ../main.cpp $(SOURCES) -o $@ $(LINK_GLUT) $(LDFLAGS) -s

example01: ../example01.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) ../example01.cpp -o $@ -s

example02: ../example02.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDE_GLUT) ../example02.cpp $(SOURCES) -o $@ $(LINK_GLUT) $(LDFLAGS) -s
	
example03: ../example03.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) -Wno-format $(INCLUDE_GLEW) $(INCLUDE_GLUT) -DEXAMPLE03_CPP -I../example03_include/ ../example03.cpp $(SOURCES) -o $@ $(LINK_GLEW) $(LINK_GLUT) $(LDFLAGS) -s

clean:
	rm -f $(TARGETS:=.exe)
	
	

