@echo off
rem ------------------------------------------------------------------
rem Build.bat – Windows batch file for building using mingw
rem This script uses x86_64-w64-mingw32-g++ and defines compile flags
rem similar to the working Linux Makefile for mingw-w64 (in the same folder).
rem ---------------------
rem ==> NEVER TESTED <==
rem ------------------------------------------------------------------

rem Set tool and flags
rem x86_64-w64-mingw32-g++ is good for mingw from w64devkit-x64-2.2.0.7z.exe, otherwise
rem g++ should be enough for winlibs-x86_64-posix-seh-gcc-15.1.0-mingw-w64msvcrt-12.0.0-r1
set CXX=x86_64-w64-mingw32-g++
set CXXFLAGS=-I../../ -march=native -fno-rtti -fno-exceptions -O3 -Wall
set LDFLAGS=-static-libgcc -static-libstdc++ -lopengl32
set SOURCES=../stdafx.cpp

rem Set additional includes and linker options for libraries
set INCLUDE_GLUT=-I./
set LINK_GLUT=-L./ -lfreeglut
set INCLUDE_GLEW=-I./
set LINK_GLEW=-L./ -lglew32

rem Define list of targets for “all” (for display purposes)
set TARGETS=example example01 example02 example03

rem If no argument is given, build all targets
if "%1"=="" goto all

rem Else, select the target based on the first argument
if /I "%1"=="example" goto target_example
if /I "%1"=="example01" goto target_example01
if /I "%1"=="example02" goto target_example02
if /I "%1"=="example03" goto target_example03
if /I "%1"=="clean" goto clean

echo Unknown target: %1
goto end

:all
echo Building all targets: %TARGETS%
call "%~0" example
call "%~0" example01
call "%~0" example02
call "%~0" example03
goto end

:target_example
echo Compiling example...
rem Compile: example is built from ../main.cpp and SOURCES with GLUT linking.
%CXX% %CXXFLAGS% %INCLUDE_GLUT% ../main.cpp %SOURCES% -o example.exe %LINK_GLUT% %LDFLAGS% -s
goto end

:target_example01
echo Compiling example01...
rem Compile: example01 is built from ../example01.cpp and SOURCES.
%CXX% %CXXFLAGS% ../example01.cpp %SOURCES% -o example01.exe -s
goto end

:target_example02
echo Compiling example02...
rem Compile: example02 is built from ../example02.cpp and SOURCES with GLUT linking.
%CXX% %CXXFLAGS% %INCLUDE_GLUT% ../example02.cpp %SOURCES% -o example02.exe %LINK_GLUT% %LDFLAGS% -s
goto end

:target_example03
echo Compiling example03...
rem Compile: example03 is built from ../example03.cpp and SOURCES; uses both GLEW and GLUT,
rem and adds a preprocessor define -DEXAMPLE03_CPP and an extra include directory.
%CXX% %CXXFLAGS% -Wno-format %INCLUDE_GLEW% %INCLUDE_GLUT% -DEXAMPLE03_CPP -I../example03_include/ ../example03.cpp %SOURCES% -o example03.exe %LINK_GLEW% %LINK_GLUT% %LDFLAGS% -s
goto end

:clean
echo Cleaning generated executables...
del example.exe 2>nul
del example01.exe 2>nul
del example02.exe 2>nul
del example03.exe 2>nul
goto end

:end
echo Done.
pause

