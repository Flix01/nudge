#ifndef _STDAFX_H_
#define _STDAFX_H_

// this is the same for all the examples
#include "nudge.h"
// that's all for stdafx.h


// only example03.cpp needs some additional stuff
#ifdef EXAMPLE03_CPP
// the headers at the bottom need to know opengl, so we must pull in opengl here
//#define USE_FREEGLUT  // optional (it usually needs to link to -lfreeglut, except on Linux where it's still -lglut)
#ifndef GL_GLEXT_PROTOTYPES
#	define GL_GLEXT_PROTOTYPES	// This includes all gl functions on all systems except Windows
#endif
#ifdef _WIN32
#undef USE_GLEW
#define USE_GLEW		// Mandatory to include all gl functions
#endif
#ifdef USE_GLEW
#   ifdef __APPLE__
#       include <GLEW/glew.h>
#   elif defined(_WIN32)
#       include <GLEW/glew.h>   // <GL/glew.h> ?
#   else // linux
#       include <GL/glew.h>
#   endif
#endif //USE_GLEW
#ifndef USE_FREEGLUT
#   ifdef __APPLE__
#       include <GLUT/GLUT.h>
#   elif defined(_WIN32)
#       include <GLUT/glut.h>   // <GL/glut.h> ?
#   else // Linux.
#       include <GL/glut.h>
#   endif
#else //USE_FREEGLUT
#   include <GL/freeglut.h>
#endif //USE_FREEGLUT
#ifdef __APPLE__
#   include <OpenGL/gl.h>
#else //__APPLE__
#   include <GL/gl.h>
#endif //__APPLE__
// now opengl should be OK


// "dynamic_resolution.h" implements the first shadow mapping step and optionally dynamic resolution (that by default should keep frame rate > config.dynamic_resolution_target_fps)
//#define DYNAMIC_RESOLUTION_USE_GLSL_VERSION_330   // (Optional) Not sure it's faster... and with emscripten, it probably needs: -s USE_WEBGL2=1
//#define DYNAMIC_RESOLUTION_SHADOW_MAP_DISABLED                    // no shadow map (saves memory)
//#define DYNAMIC_RESOLUTION_SHADOW_MAP_SIZE_MULTIPLIER (1.5)            // needs a value (default is 1.5). When screen is resized, it multiplies its longest dimension to get the shadow texture size.
//#define DYNAMIC_RESOLUTION_SHADOW_MAP_SIZE_FORCE_POT              // when defined the shadow texture size is approximated by its nearest power of two
//#define DYNAMIC_RESOLUTION_SHADOW_MAP_MAX_WIDTH       (2048)      // needs a value (default 2048). It clamps: shadow_texture.width = DYNAMIC_RESOLUTION_SHADOW_MAP_SIZE_MULTIPLIER * screen.width; // (or its nearest POT)
//#define DYNAMIC_RESOLUTION_SHADOW_MAP_MAX_HEIGHT      (2048)       // needs a value (default 2048). It clamps: shadow_texture.height = DYNAMIC_RESOLUTION_SHADOW_MAP_SIZE_MULTIPLIER * screen.height); // (or its nearest POT)
//#define DYNAMIC_RESOLUTION_SHADOW_MAP_RECTANGULAR                 // WIP. When it's NOT defined then: if (shadow_texture.width<shadow_texture.height)  shadow_texture.width=shadow_texture.height; else shadow_texture.height=shadow_texture.width;
//#define DYNAMIC_RESOLUTION_SHADOW_USE_NEAREST_TEXTURE_FILTER
//#define DYNAMIC_RESOLUTION_SHADOW_USE_PCF (3)   // (Optional) Smooths shadows a bit, but its more expensive; with emscripten, it needs: -s USE_WEBGL2=1
//#define TEAPOT_SHADER_SHADOW_MAP_PCF (3)    // This is the same as before for teapot.h. I guess thay must be kept in sync... even if this def seems to do nothing...
#include "dynamic_resolution.h"

#include <math.h> // sqrt (this should be included in teapot.h, shouldn't it?)
#define TEAPOT_SHADER_SPECULAR
#define TEAPOT_SHADER_FOG       // used to mitigate bad clipping
//#define TEAPOT_SHADER_FOG_HINT_FRAMENT_SHADER   // Better fog on ground plane (but a bit expensive)
#define TEAPOT_SHADER_USE_SHADOW_MAP // needs DYNAMIC_RESOLUTION_SHADOW_MAP_DISABLED not defined
#include "teapot.h"
// ----------------------------------------
#endif //EXAMPLE03_CPP


#endif //_STDAFX_H_

