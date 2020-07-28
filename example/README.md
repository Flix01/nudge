# EXAMPLE

In this folder are present:

<b>main.cpp:</b>

![example](./screenshots/example.png)

<b>main_no_ffp.cpp:</b> 

![example](./screenshots/example_no_ffp.png)

<b>main_no_ffp_with_shadows.cpp (here with 8192 rigid bodies):</b> 

![example](./screenshots/example_no_ffp_with_shadows.gif)

<b>main_no_ffp_with_shadows.cpp (emscripten-1.39.14) (here with 800 rigid bodies):</b> 

<img src="./screenshots/example_no_ffp_with_shadows_emscripten.png" width="250">

## HOW TO COMPILE

### main.cpp

This is the default stand-alone demo of the nudge upstream repository.
It can be compiled using the makefiles available in the three subfolders: linux, mac and win.

### main_no_fpp.cpp

This is a port of the default demo that does not use the fixed function pipeline (so that it can be compiled using emscripten).
It uses teapot.h from https://github.com/Flix01/Header-Only-GL-Helpers.

Compilation instructions are at the top of the file.

### main_no_fpp_with_shadows.cpp

This is an enhanced version of main.no_ffp.cpp with shadows, more objects and dynamic resolution (the resolution of the window and of the shadow map can decrease when the frame rate is low).
It uses teapot.h and dynamic_resolution.h from https://github.com/Flix01/Header-Only-GL-Helpers.

Compilation instructions are at the top of the file. 

- This demo can also be compiled with emscripten (a .html is present in the html subfolder, and can be run locally using a webserver, or emrun).



**PREREQUISITES**

* glut (or freeglut)
* glew (Windows only)

