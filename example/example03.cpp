//
// Copyright (c) 2024 Flix01 (https://github.com/Flix01/nudge/tree/master/new_version)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the “Software”), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

// Based on the original (2017) example by Rasmus Barringer

// example03.cpp is currently just a cut-down, minimal version of example02.cpp,
// converting it from the fixed fuction pipeline to shader opengl,
// so that hopefully we can have:
// -> shadows
// -> dynamic resolution support
// -> emscripten build

// so there is not much about nudge.h that is not already present in example02.cpp
// => example03.cpp is less relevant regarding physics programming and it's better
// to use example02.cpp as a reference for physics code


// PLEASE COMPILE FROM THE nudge/example folder

// Linux:
//--------
// c++ -O3 -march=native -fno-rtti -fno-exceptions -fopenmp-simd -DEXAMPLE03_CPP example03.cpp stdafx.cpp -o example03 -I"./" -I"../" -I"./example03_include/" -lglut -lGL
// (Optionally with -march=haswell)

// Emscripten (with output in the ./html subfolder)
//-------------------------------------------------
// With SIMD support (see: https://emscripten.org/docs/porting/simd.html):
// em++ -O3 -msse2 -msimd128 -fno-rtti -fno-exceptions -fopenmp-simd -DEXAMPLE03_CPP -s STACK_SIZE=512kb -s ALLOW_MEMORY_GROWTH=1 -o html/nudge_example03.html ./example03.cpp ./stdafx.cpp -I"./" -I"../" -I"./example03_include/" -lglut -lGL
// This commandline 'should' work to remove SIMD support. It needs SIMDE (clone the header-only library: https://github.com/simd-everywhere/simde somewhere and add a soft link to its inner ./simde subfolder at the same level as nudge.h):
// em++ -O3 -DNUDGE_USE_SIMDE -DSIMDE_NO_NATIVE -DSIMDE_ENABLE_OPENMP -DEXAMPLE03_CPP -fopenmp-simd -fno-rtti -fno-exceptions -s STACK_SIZE=512kb -s ALLOW_MEMORY_GROWTH=1 -o html/nudge_example03.html ./example03.cpp ./stdafx.cpp -I"./" -I"../" -I"./example03_include/" -lglut -lGL
// Once successfully compiled, run html/nudge_example03.html locally through a web server (or using emrun, search the web for further info).
// As you can see for some reasons (not sure if it's related to nudge.h or more likely to teapot.h) it needs:
// -s STACK_SIZE=512kb -s ALLOW_MEMORY_GROWTH=1
// If something goes wrong try: -s STACK_SIZE=1mb or something like that

// Windows
//---------
// Never tried, but it needs GLEW to import all the required GL functions

// Mac
//----
// Never tried

// Optional definitions:
// -DAZERTY_KEYBOARD_LAYOUT for French keyboard
// -DNDEBUG to remove assertions (use it only when you see that performance increases)

#ifndef EXAMPLE03_CPP
#error Please define EXAMPLE03_CPP in your compilation options (not in a .c file!)
#endif

#include "stdafx.h"

#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h> // strtol (removable, but otherwise unsafe scanf must be used)

#ifndef __EMSCRIPTEN__
#define TOT_NUM_BOXES (1024)
#define TOT_NUM_SPHERES (512)
#else
#define TOT_NUM_BOXES (256)
#define TOT_NUM_SPHERES (128)
#endif
#define TOT_NUM_BODIES (TOT_NUM_BOXES+TOT_NUM_SPHERES)

// custom replacements of gluPerspective(...) and gluLookAt(...), so that we don't need -lglu
void glPerspective(float degfovy, float aspect, float zNear, float zFar);
void glLookAt(float eyeX,float eyeY,float eyeZ,float centerX,float centerY,float centerZ,float upX,float upY,float upZ);

// a selection of popular colors (more can be found in a lot of wiki sites)
enum ColorEnum {
    COLOR_NONE=0,COLOR_ALICEBLUE/*0xF0F8FFFF*/,COLOR_ANTIQUEWHITE/*0xFAEBD7FF*/,COLOR_AQUA/*0x00FFFFFF*/,COLOR_AQUAMARINE/*0x7FFFD4FF*/,COLOR_AZURE/*0xF0FFFFFF*/,COLOR_BEIGE/*0xF5F5DCFF*/,COLOR_BISQUE/*0xFFE4C4FF*/,COLOR_BLACK/*0x000000FF*/,COLOR_BLANCHEDALMOND/*0xFFEBCDFF*/,COLOR_BLUE/*0x0000FFFF*/,COLOR_BLUEVIOLET/*0x8A2BE2FF*/,COLOR_BROWN/*0xA52A2AFF*/,COLOR_BURLYWOOD/*0xDEB887FF*/,COLOR_CADETBLUE/*0x5F9EA0FF*/,COLOR_CHARTREUSE/*0x7FFF00FF*/,COLOR_CHOCOLATE/*0xD2691EFF*/,COLOR_CORAL/*0xFF7F50FF*/,COLOR_CORNFLOWERBLUE/*0x6495EDFF*/,COLOR_CORNSILK/*0xFFF8DCFF*/,
    COLOR_CRIMSON/*0xDC143CFF*/,COLOR_CYAN/*0x00FFFFFF*/,COLOR_DARKBLUE/*0x00008BFF*/,COLOR_DARKCYAN/*0x008B8BFF*/,COLOR_DARKGOLDENROD/*0xB8860BFF*/,COLOR_DARKGRAY/*0xA9A9A9FF*/,COLOR_DARKGREEN/*0x006400FF*/,COLOR_DARKKHAKI/*0xBDB76BFF*/,COLOR_DARKMAGENTA/*0x8B008BFF*/,COLOR_DARKOLIVEGREEN/*0x556B2FFF*/,COLOR_DARKORANGE/*0xFF8C00FF*/,COLOR_DARKORCHID/*0x9932CCFF*/,COLOR_DARKRED/*0x8B0000FF*/,COLOR_DARKSALMON/*0xE9967AFF*/,COLOR_DARKSEAGREEN/*0x8FBC8FFF*/,COLOR_DARKSLATEBLUE/*0x483D8BFF*/,COLOR_DARKSLATEGRAY/*0x2F4F4FFF*/,COLOR_DARKTURQUOISE/*0x00CED1FF*/,COLOR_DARKVIOLET/*0x9400D3FF*/,COLOR_DEEPPINK/*0xFF1493FF*/,
    COLOR_DEEPSKYBLUE/*0x00BFFFFF*/,COLOR_DIMGRAY/*0x696969FF*/,COLOR_DODGERBLUE/*0x1E90FFFF*/,COLOR_FIREBRICK/*0xB22222FF*/,COLOR_FLORALWHITE/*0xFFFAF0FF*/,COLOR_FORESTGREEN/*0x228B22FF*/,COLOR_FUCHSIA/*0xFF00FFFF*/,COLOR_GAINSBORO/*0xDCDCDCFF*/,COLOR_GHOSTWHITE/*0xF8F8FFFF*/,COLOR_GOLD/*0xFFD700FF*/,COLOR_GOLDENROD/*0xDAA520FF*/,COLOR_GRAY/*0x808080FF*/,COLOR_GREEN/*0x008000FF*/,COLOR_GREENYELLOW/*0xADFF2FFF*/,COLOR_HONEYDEW/*0xF0FFF0FF*/,
    COLOR_HOTPINK/*0xFF69B4FF*/,COLOR_INDIANRED/*0xCD5C5CFF*/,COLOR_INDIGO/*0x4B0082FF*/,COLOR_IVORY/*0xFFFFF0FF*/,COLOR_KHAKI/*0xF0E68CFF*/,COLOR_LAVENDER/*0xE6E6FAFF*/,
    COLOR_LAVENDERBLUSH/*0xFFF0F5FF*/,COLOR_LAWNGREEN/*0x7CFC00FF*/,COLOR_LEMONCHIFFON/*0xFFFACDFF*/,COLOR_LIGHTBLUE/*0xADD8E6FF*/,COLOR_LIGHTCORAL/*0xF08080FF*/,COLOR_LIGHTCYAN/*0xE0FFFFFF*/,COLOR_LIGHTGOLDENRODYELLOW/*0xFAFAD2FF*/,COLOR_LIGHTGRAY/*0xD3D3D3FF*/,COLOR_LIGHTGREEN/*0x90EE90FF*/,COLOR_LIGHTPINK/*0xFFB6C1FF*/,COLOR_LIGHTSALMON/*0xFFA07AFF*/,COLOR_LIGHTSEAGREEN/*0x20B2AAFF*/,COLOR_LIGHTSKYBLUE/*0x87CEFAFF*/,COLOR_LIGHTSLATEGRAY/*0x778899FF*/,COLOR_LIGHTSTEELBLUE/*0xB0C4DEFF*/,COLOR_LIGHTYELLOW/*0xFFFFE0FF*/,COLOR_LIME/*0x00FF00FF*/,COLOR_LIMEGREEN/*0x32CD32FF*/,COLOR_LINEN/*0xFAF0E6FF*/,COLOR_MAGENTA/*0xFF00FFFF*/,
    COLOR_MAROON/*0x800000FF*/,COLOR_MEDIUMAQUAMARINE/*0x66CDAAFF*/,COLOR_MEDIUMBLUE/*0x0000CDFF*/,COLOR_MEDIUMORCHID/*0xBA55D3FF*/,COLOR_MEDIUMPURPLE/*0x9370DBFF*/,COLOR_MEDIUMSEAGREEN/*0x3CB371FF*/,COLOR_MEDIUMSLATEBLUE/*0x7B68EEFF*/,COLOR_MEDIUMSPRINGGREEN/*0x00FA9AFF*/,COLOR_MEDIUMTURQUOISE/*0x48D1CCFF*/,COLOR_MEDIUMVIOLETRED/*0xC71585FF*/,COLOR_MIDNIGHTBLUE/*0x191970FF*/,COLOR_MINTCREAM/*0xF5FFFAFF*/,COLOR_MISTYROSE/*0xFFE4E1FF*/,COLOR_MOCCASIN/*0xFFE4B5FF*/,COLOR_NAVAJOWHITE/*0xFFDEADFF*/,COLOR_NAVY/*0x000080FF*/,COLOR_OLDLACE/*0xFFF5E6FF*/,COLOR_OLIVE/*0x808000FF*/,COLOR_OLIVEDRAB/*0x6B8E23FF*/,
    COLOR_ORANGE/*0xFFA500FF*/,COLOR_ORANGERED/*0xFF4500FF*/,
    COLOR_ORCHID/*0xDA70D6FF*/,COLOR_PALEGOLDENROD/*0xEEE8AAFF*/,COLOR_PALEGREEN/*0x98FB98FF*/,COLOR_PALETURQUOISE/*0xAFEEEEFF*/,COLOR_PALEVIOLETRED/*0xDB7093FF*/,COLOR_PAPAYAWHIP/*0xFFEFD5FF*/,COLOR_PEACHPUFF/*0xFFDAB9FF*/,COLOR_PERU/*0xCD853FFF*/,COLOR_PINK/*0xFFC0CBFF*/,COLOR_PLUM/*0xDDA0DDFF*/,COLOR_POWDERBLUE/*0xB0E0E6FF*/,COLOR_PURPLE/*0x800080FF*/,COLOR_RED/*0xFF0000FF*/,COLOR_ROSYBROWN/*0xBC8F8FFF*/,COLOR_ROYALBLUE/*0x4169E1FF*/,COLOR_SADDLEBROWN/*0x8B4513FF*/,COLOR_SALMON/*0xFA8072FF*/,COLOR_SANDYBROWN/*0xF4A460FF*/,COLOR_SEAGREEN/*0x2E8B57FF*/,COLOR_SEASHELL/*0xFFF5EEFF*/,
    COLOR_SIENNA/*0xA0522DFF*/,COLOR_SILVER/*0xC0C0C0FF*/,COLOR_SKYBLUE/*0x87CEEBFF*/,COLOR_SLATEBLUE/*0x6A5ACDFF*/,COLOR_SLATEGRAY/*0x708090FF*/,COLOR_SNOW/*0xFFFAFAFF*/,COLOR_SPRINGGREEN/*0x00FF7FFF*/,COLOR_STEELBLUE/*0x4682B4FF*/,COLOR_TAN/*0xD2B48CFF*/,COLOR_TEAL/*0x008080FF*/,COLOR_THISTLE/*0xD8BFD8FF*/,COLOR_TOMATO/*0xFF6347FF*/,COLOR_TURQUOISE/*0x40E0D0FF*/,COLOR_VIOLET/*0xEE82EEFF*/,COLOR_WHEAT/*0xF5DEB3FF*/,COLOR_WHITE/*0xFFFFFFFF*/,COLOR_WHITESMOKE/*0xF5F5F5FF*/,COLOR_YELLOW/*0xFFFF00FF*/,COLOR_YELLOWGREEN/*0x9ACD32FF*/,COLOR_COUNT
};
void Teapot_SetColorEnum(ColorEnum color); // internally calls Teapot_SetColor(...)
inline void Teapot_SetHalfScaling(float scalingX,float scalingY,float scalingZ) {
    Teapot_SetScaling(2.f*scalingX,2.f*scalingY,2.f*scalingZ);
}
namespace nudge {NUDGE_CONSTEXPR FlagMask BF_IS_SHADOW_PASS_FRUSTUM_CULLED=1<<14,BF_IS_SHADOW_AND_RENDERING_PASS_FRUSTUM_CULLED=BF_IS_SHADOW_PASS_FRUSTUM_CULLED|BF_IS_FRUSTUM_CULLED;}


// the global structure with all our data
static struct globals_t {
	nudge::context_t nudge_context;
    unsigned use_graphic_transform;
    unsigned use_character_camera;
    struct camera_t {
		// camera data:
        float targetPos[3]; // = {0,2,0};     // This demo places the ground plane below zero
		float cameraYaw; // = 2*M_PI;
		float cameraPitch; // = M_PI*0.125f;
        float cameraDistance; // = 20;
		float cameraPos[3]; // = {0,0,0};       // Derived value (do not edit)
    } camera[2];
    float lightDirection[4]; // = {1,1,2,   0};	// Will be normalized
	struct proj_t {
		// pMatrix data:
        float pMatrixFovDeg; // = 45.f;
		float pMatrixNearPlane; // = 0.5f;
        float pMatrixFarPlane; // = 100.0f;
	} proj;
    struct key_t {
        uint32_t down,down_last_frame,pressed/*_this_frame*/,released/*_this_frame*/;   // bit-mask of KeyMask values below
        int mouseX,mouseY;  // mouse position (updated only when a key or mouse button is pressed)
    } key;  // read-only
	float instantFrameTime; // = 16.2f;
    struct bodies_t {
        unsigned kinematic_body;    // we store the index of a kinematic body to move it around manually
        unsigned character_body;    // same as before
        unsigned conveyor_belt_body0,conveyor_belt_body1;
        unsigned star_body, num_stars;
        unsigned rotating_platform_animation_index;
        unsigned fix_character_sinking_effect_on_fall;   // experiment
    } bodies;
    unsigned use_frustum_culling;
    unsigned num_frustum_culled_bodies; // to monitor frustum culling
    unsigned num_shadow_pass_frustum_culled_bodies; // to monitor frustum culling
    unsigned use_shadows;
    unsigned is_shadow_pass;
    float mMatrices[TOT_NUM_BODIES*16]; // we cache body mMatrices just because otherwise we must calculate them twice each frame (one for shadow pass and one for rendering pass: see DrawPhysics(...))
} globals = {{},1,0,{{{0,2,0},2*M_PI,M_PI*0.125f,20,{0,0,0}},{{0,2,0},M_PI,M_PI*0.125f,5,{0,0,0}}},{1,1,1.5},{45.f,0.75f,75.0f},{0,0,0,(uint32_t)-1,0,0},16.2,{NUDGE_INVALID_BODY_ID,NUDGE_INVALID_BODY_ID,NUDGE_INVALID_BODY_ID,NUDGE_INVALID_BODY_ID,NUDGE_INVALID_BODY_ID,0,(unsigned)-1,0},1,0,0,1,0,{}};
static nudge::context_t* c = &globals.nudge_context;	// shorter... we'll use this


// these are mainly used to improve GLUT key handling. I'd use a uint64_t enum here, but it requires C++11 support (unless we use messy variables or #defines instead of typed enums)
enum KeyMask {KEY_REGULAR_KEY_START_INDEX=0,
              KEY_W=1<<0,KEY_A=1<<1,KEY_S=1<<2,KEY_D=1<<3,KEY_WASD=KEY_W|KEY_A|KEY_S|KEY_D,KEY_J=1<<4,KEY_SPACE=1<<5,KEY_R=1<<6,KEY_ESC=1<<7,KEY_ENTER=1<<8,KEY_H=1<<9,KEY_M=1<<10,KEY_F=1<<11,
              KEY_NUM_REGULAR_KEYS=12,
              KEY_MODIFIER_KEY_START_INDEX=12,
              KEY_SHIFT=1<<12,KEY_CTRL=1<<13,KEY_ALT=1<14,KEY_ALL_MODIFIER_KEYS=KEY_SHIFT|KEY_CTRL|KEY_ALT,
              KEY_NUM_MODIFIER_KEYS=3,
              KEY_SPECIAL_KEY_START_INDEX=15,
              KEY_LEFT=1<<15,KEY_RIGHT=1<<16,KEY_UP=1<<17,KEY_DOWN=1<<18,KEY_ALL_ARROW_KEYS=KEY_LEFT|KEY_RIGHT|KEY_UP|KEY_DOWN,
              KEY_PAGE_UP=1<<19,KEY_PAGE_DOWN=1<<20,KEY_PAGE_UP_OR_DOWN=KEY_PAGE_UP|KEY_PAGE_DOWN,KEY_HOME=1<<21,KEY_END=1<<22,
              KEY_F1=1<<23,KEY_F3=1<<24,KEY_F5=1<<25,KEY_F7=1<<26,KEY_ALL_SUPPORTED_FUNCTION_KEYS=KEY_F1|KEY_F3|KEY_F5|KEY_F7,
              KEY_NUM_SPECIAL_KEYS=26-KEY_SPECIAL_KEY_START_INDEX+1,
              KEY_MOUSE_KEY_START_INDEX=27,
              KEY_MOUSE_BUTTON_LEFT=1<<27,KEY_MOUSE_BUTTON_RIGHT=1<<28,KEY_MOUSE_BUTTON_LEFT_OR_RIGHT=KEY_MOUSE_BUTTON_LEFT|KEY_MOUSE_BUTTON_RIGHT,
              KEY_MOUSE_BUTTON_MIDDLE=1<<29,KEY_ALL_MOUSE_BUTTONS=KEY_MOUSE_BUTTON_LEFT_OR_RIGHT|KEY_MOUSE_BUTTON_MIDDLE,
              KEY_WOUSE_WHEEL_UP=1<<30,KEY_MOUSE_WHEEL_DOWN=1<<31,KEY_MOUSE_WHEEL_UP_OR_DOWN=KEY_WOUSE_WHEEL_UP|KEY_MOUSE_WHEEL_DOWN,KEY_ALL_MOUSE_BUTTONS_AND_WHEEL=KEY_ALL_MOUSE_BUTTONS|KEY_MOUSE_WHEEL_UP_OR_DOWN,
              KEY_NUM_MOUSE_KEYS=31-KEY_MOUSE_KEY_START_INDEX+1
             };


// Please note that code related to nudge physics is mostly grouped in the functions that follow
// You can usually avoid to read further. 80% of the physics code is in InitPhysics().
inline void bind_body(nudge::context_t* c,unsigned body,TeapotMeshEnum shape,ColorEnum color=COLOR_NONE) {c->bodies.infos[body].user.u8[0] = (uint8_t) shape; c->bodies.infos[body].user.u8[1] = (uint8_t) ((color!=COLOR_NONE)?color:(ColorEnum)(1+(body%(COLOR_COUNT-1))));/*(persistent) 'random' body color*/}
inline TeapotMeshEnum bodyinfo_get_shape_enum(const nudge::BodyInfo* info) {return (TeapotMeshEnum) info->user.u8[0];}
inline ColorEnum bodyinfo_get_color_enum(const nudge::BodyInfo* info) {return (ColorEnum) info->user.u8[1];}
float* getStarPosition(unsigned number); // forward declaration
void InitPhysics() {
    using namespace nudge;
    if (c->MAX_NUM_BODIES==0) {
        init_context_with(c,TOT_NUM_BOXES,TOT_NUM_SPHERES);    // first call
        // usually here users can set some c->simulation_params if necessary
        assert(c->MAX_NUM_BODIES==TOT_NUM_BODIES);
    }
    else restart_context(c); // when KEY_R is pressed

    unsigned body;
    Transform T = identity_transform;

    // Ground mesh (box)
    T.p[0]=0.f;T.p[1]=-0.3f;T.p[2]=0.0f;
    body = add_box(c,0,1.5f*6.f,0.3f,1.5f*6.f,&T);assert(body!=NUDGE_INVALID_BODY_ID);    // we should always check the return value (but we don't)
    bind_body(c,body,TEAPOT_MESH_CUBIC_GROUND,COLOR_LIGHTGREEN); // this is our GRAPHIC body link (nothing regarding physics) (TEAPOT_MESH_CUBIC_GROUND is like TEAPOT_MESH_CUBE, better targeted for a wide 'up face')
    //T = identity_transform; // we 'should' do this every time (but we don't)

    // 4 other ground meshes (boxes)
    T.p[0]=-16.f;T.p[1]=-0.3f;T.p[2]=2.5f;
    body = add_box(c,0,2.95f,0.3f,6.0f,&T);
    bind_body(c,body,TEAPOT_MESH_CUBIC_GROUND,COLOR_LIGHTGREEN);
    T.p[0]=11.f;T.p[1]=2.f;T.p[2]=-11.f;
    body = add_box(c,0,2.f,0.2f,2.,&T);
    bind_body(c,body,TEAPOT_MESH_CUBIC_GROUND,COLOR_DARKSEAGREEN);
    T.p[0]=-11.f;T.p[1]=3.8f;T.p[2]=-9.f;
    body = add_box(c,0,2.f,0.2f,4.,&T);
    bind_body(c,body,TEAPOT_MESH_CUBIC_GROUND,COLOR_DARKSEAGREEN);
    T=identity_transform;T.p[0]=-12.f;T.p[1]=4.f-0.2f;T.p[2]=5.5f;
    body = add_box(c,0.f,1.f,0.2f,1.f,&T);
    bind_body(c,body,TEAPOT_MESH_CUBIC_GROUND,COLOR_DARKSEAGREEN);

    // a dynamic cube (box)
    T.p[0]=-6.f;T.p[1]=0.5f;T.p[2]=3.0f;
    body = add_box(c,4.f,0.5f,0.5f,0.5f,&T);
    bind_body(c,body,TEAPOT_MESH_CUBE_ROUNDED,COLOR_AZURE);

    // house (box)
    T.p[0]=-3.75f;T.p[1]=1.5f;T.p[2]=-1.0f;
    body = add_box(c,0.f,1.5f,1.5f,2.25f,&T);
    c->bodies.properties[body].friction = 0.2f;
    bind_body(c,body,TEAPOT_MESH_CUBE,COLOR_IVORY);

    // house roof (box compound)
    T.p[0]=-3.75f;T.p[1]=3.5f;T.p[2]=-1.0f;
    Transform boxT[2] = {{{{-0.9f,0.f,0.f}},{0},{{0.f,0.f,0.f,1.f}}},{{{0.9f,0.f,0.f}},{0},{{0.f,0.f,0.f,1.f}}}};
    const float hsizeTriplets[6] = {1.2f,0.15f,2.75f,    1.2f,0.15f,2.75f};
    nm_QuatFromAngleAxis(boxT[0].q,M_DEG2RAD(36.f),0.f,0.f,1.f);
    nm_QuatFromAngleAxis(boxT[1].q,M_DEG2RAD(-36.f),0.f,0.f,1.f);
    body = add_compound(c,0.f,NULL,2,hsizeTriplets,boxT,0,NULL,NULL,&T);
    bind_body(c,body,TEAPOT_MESH_ROOF,COLOR_DARKRED);

    // static cylinderY (box)
    T.p[0]=-1.65f;T.p[1]=1.2f;T.p[2]=-1.5f;
    body = add_box(c,0.f,0.2f,1.25f,0.2f,&T);
    bind_body(c,body,TEAPOT_MESH_CYLINDER,COLOR_LIGHTBLUE);

    // skittles (box) [comOffset test][collision group test]
    T=identity_transform;
    float hsize[3];hsize[0]=hsize[2]=0.225f;hsize[1]=0.75f;
    T.p[1]=hsize[1]*1.25f;
    float comOffset[3]={0,-0.75f,0}; // moves center of mass down
    const float spacingX = hsize[0]*2.1f;
    const float spacingZ = hsize[2]*2.1f;
    for (int j=0;j<6;j++) {
        T.p[0]=2.4;T.p[2]=2.0+spacingZ;
        if (j==0) T.p[2]-=spacingZ*2.f; // first row (1 skittle)
        else if (j<3) {
            // second row (2 skittles)
            T.p[2]-=spacingZ;
            T.p[0]+=-spacingX*0.5f+(j-1)*spacingX;
        }
        else T.p[0]+=-spacingX+(j-3)*spacingX;   // last row (3 skittles)
        body = add_box(c,1.f,hsize[0],hsize[1],hsize[2],&T,comOffset);
        bind_body(c,body,TEAPOT_MESH_SKITTLE,COLOR_GOLDENROD);
        c->bodies.filters[body].collision_group = COLLISION_GROUP_A;   // group the body belongs to (see the flags of the kinematic teapot)
    }
    memset(comOffset,0,sizeof(float)*3);    // reset comOffset variable

    // test extra:: namespace
    T=identity_transform;T.p[0]=-3.5f;T.p[1]=0.5f;T.p[2]=6.f;
    body = extra::add_compound_cylinder(c,0.3f,0.25f,T.p[1],&T,AXIS_Y,0,0);
    bind_body(c,body,TEAPOT_MESH_CYLINDER,COLOR_PINK);

    // test extra:: namespace
    T=identity_transform;T.p[0]=-6.5f;T.p[1]=0.5f;T.p[2]=6.f;
    body = extra::add_compound_hollow_cylinder(c,0.5f,0.35f,0.5f,T.p[1],&T,AXIS_Y,0,0);    // same as the comment below
    bind_body(c,body,TEAPOT_MESH_HOLLOW_CYLINDER,COLOR_ORANGERED);

    // test extra:: namespace
    T=identity_transform;T.p[0]=-5.0f;T.p[1]=0.5f+0.25f;T.p[2]=6.f;
    body = extra::add_compound_capsule(c,0.5f,0.25f,0.5f,&T,AXIS_Y,0,0);
    bind_body(c,body,TEAPOT_MESH_CAPSULE,COLOR_AZURE);

    // test extra:: namespace
    T=identity_transform;T.p[0]=-7.0f;T.p[1]=0.5f;T.p[2]=8.f;
    body = extra::add_compound_cone(c,0.5f,0.5f,T.p[1],&T,AXIS_Y,0,0);
    bind_body(c,body,TEAPOT_MESH_CONE2,COLOR_YELLOW);

    // test extra:: namespace
    T=identity_transform;T.p[0]=-6.5f;T.p[1]=1.0f;T.p[2]=4.f;
    body = extra::add_compound_torus(c,1.f,T.p[1]*0.8f,T.p[1]*0.2f,&T,AXIS_Z,16); // the torus mesh in teapot.h if in the Z direction, and the total_radius = 0.8 (main_radius) + 0.2 (inner_radius) // the last 2 are args of add_torus(...)
    bind_body(c,body,TEAPOT_MESH_TORUS,COLOR_YELLOW);

    // kinematic teapot (box) [manual kinematic body test] [collision mask test]
    T=identity_transform;T.p[0]=1.5f;T.p[1]=0.5f;T.p[2]=8.0f;
    globals.bodies.kinematic_body = body = add_box(c,-10000.f,0.5f,0.5f,0.75f,&T);
    bind_body(c,body,TEAPOT_MESH_TEAPOT,COLOR_LIGHTSKYBLUE);
    c->bodies.filters[body].collision_group = COLLISION_GROUP_E;   // group the body belongs to
    c->bodies.filters[body].collision_mask = COLLISION_GROUP_ALL & (~COLLISION_GROUP_A);    // groups the body can collide with (all except bodies of group A)
    // (kinematic animation test: door) [automatic kinematic body test]
    {
        // -> kinematic body creation
        T=identity_transform;T.p[0]=-0.5f;T.p[1]=1.0f;T.p[2]=-8.0f;
        comOffset[0]=-0.625f;   // this is JUST to ease the open-door animation later
        body = add_box(c,-10000.f,0.625f,1.f,0.1f,&T,comOffset);
        bind_body(c,body,TEAPOT_MESH_CUBE,COLOR_DARKBLUE);
        memset(comOffset,0,sizeof(float)*3);    // reset comOffset variable

        // -> kinematic animation construction
        // ------> key frames set (key frames for all animations are all in a single global array)
        const uint32_t start_key_frame = c->kinematic_data.key_frame_count;
        kinematic_data_reserve_key_frames(&c->kinematic_data,c->kinematic_data.key_frame_count+4);    // we'll use <=4 frames in total
        assert(c->kinematic_data.key_frame_capacity>=c->kinematic_data.key_frame_count+4);
        uint32_t* total_key_frames_count = &c->kinematic_data.key_frame_count;   // for all animations
        Transform T = identity_transform; KinematicData::TimeMode mode = KinematicData::TM_NORMAL;
        Transform* pkfTransforms = &c->kinematic_data.key_frame_transforms[start_key_frame];       // at this point they're all set to identity
        KinematicData::TimeMode* pkfModes = &c->kinematic_data.key_frame_modes[start_key_frame];   // at this point they're all set to KinematicData::TM_NORMAL;
        T.time = 0.25f; mode = KinematicData::TM_ACCELERATE;
        pkfTransforms[0] = T; pkfModes[0] = mode; (*total_key_frames_count)++;   // key frame 0
        nm_QuatFromAngleAxis(T.rotation,-M_PI*0.5f,0,1,0);T.time = 1.f; mode = KinematicData::TM_DECELERATE;
        pkfTransforms[1] = T; pkfModes[1] = mode; (*total_key_frames_count)++;   // key frame 1
        T = pkfTransforms[0]; T.time = 1.f; mode = pkfModes[0];
        pkfTransforms[2] = T; pkfModes[2] = mode; (*total_key_frames_count)++;   // key frame 2
        T.time = 0.25f; T.position[2]=3.5f; mode = KinematicData::TM_NORMAL;
        pkfTransforms[3] = T; pkfModes[3] = mode; (*total_key_frames_count)++;   // key frame 3
        assert(*total_key_frames_count<=c->kinematic_data.key_frame_capacity);

        // ------> kinematic animation that use these key frames
        kinematic_data_reserve_animations(&c->kinematic_data,c->kinematic_data.animations_count+2);    // we'll use <=2 animation in total
        assert(c->kinematic_data.animations_capacity>=c->kinematic_data.animations_count+2);

        KinematicData::Animation* ka = &c->kinematic_data.animations[c->kinematic_data.animations_count++];
        ka->key_frame_start = start_key_frame; ka->key_frame_count = 4;
        ka->loop_mode = KinematicData::Animation::LM_LOOP_NORMAL;
        ka->body = body;
        ka->baseT = c->bodies.transforms[body];   // unless key frame Transforms are in the absolute space, this must always be set
        ka->use_baseT = true;   // unless key frame Transforms are in the absolute space, this must always be set
        ka->speed = 1.f;    // but too fast kinematic animations can lead to tunnelling
        ka->play_time = ka->offset_time = 0.f;  // Mainly used to time-offset one instance
        ka->playing = true;


        // (optional: second body reusing the same key frames)
        // -> kinematic body creation
        T=identity_transform;T.p[0]=-0.5f+2.5f;T.p[1]=1.0f;T.p[2]=-8.0f;
        comOffset[0]=-0.625f;   // this is JUST to ease the open-door animation later
        body = add_box(c,-10000.f,0.625f,1.f,0.1f,&T,comOffset);
        bind_body(c,body,TEAPOT_MESH_CUBE,COLOR_DARKBLUE);
        memset(comOffset,0,sizeof(float)*3);    // reset comOffset variable

        c->kinematic_data.animations[c->kinematic_data.animations_count] = *ka;
        ka = &c->kinematic_data.animations[c->kinematic_data.animations_count++];
        ka->baseT = T;nm_QuatFromAngleAxis(ka->baseT.q,M_PI,0,0,1);
        ka->body = body;

    }

    // experimental character (ehm...) controller (box)
    T=identity_transform;T.p[0]=7.5f;T.p[1]=0.65f;T.p[2]=-4.f;nm_QuatFromAngleAxis(T.q,-M_PI*0.5f,0,1,0);
    const float chdim[3] = {.3f,T.p[1],0.1f}, cmass=20.f;
    // simple box shape
    body = add_box(c,cmass,chdim[0],chdim[1],chdim[2],&T);
    // or compound shape (almost useless)
    // const float bC[3] = {chdim[0],chdim[1]*0.7f,chdim[2]};
    // const float sCs[4] = {chdim[0]*0.35f,chdim[0]*0.35f,chdim[0]*0.4f,chdim[0]*0.4f};
    // Transform cTs[5]={identity_transform,identity_transform,identity_transform,identity_transform,identity_transform};
    // cTs[0].p[1]=-sCs[2]*0.5f;cTs[3].p[1]=chdim[1]-sCs[2]*0.5f;cTs[4].p[1]=cTs[3].p[1]-sCs[3]*1.f;
    // cTs[1].p[1]=-chdim[1]+sCs[0];cTs[1].p[0]=chdim[0]*0.4f;;cTs[2].p[1]=-chdim[1]+sCs[1];cTs[2].p[0]=-chdim[0]*0.4f;
    // body = add_compound(c,cmass,NULL,1,bC,&cTs[0],4,sCs,&cTs[1],&T);
    // or a traditional capsule (wrong scaling, but I want to test the staircase with it)
    //body = extra::add_compound_capsule(c,cmass,1.5f*chdim[0],1.5f*(chdim[1]-chdim[0]),&T);
    bind_body(c,body,TEAPOT_MESH_CHARACTER,globals.bodies.num_stars<12?COLOR_YELLOW:COLOR_ORANGE);
    globals.bodies.character_body = body;assert(body==c->bodies.transforms[body].body);
    float* in_inv = c->bodies.properties[body].inertia_inverse;in_inv[0]=in_inv[1]=in_inv[2]=0.f;
    c->bodies.properties[body].friction=4.5f;  // default body friction is 1.f AFAIR
    c->bodies.filters[body].flags|=BF_IS_CHARACTER; // note that this flag does nothing in nudge.h
    c->bodies.filters[body].collision_group=COLLISION_GROUP_C;

    // star (box) [collectable test]
    if (globals.bodies.num_stars<12) {
        T=identity_transform;memcpy(T.p,getStarPosition(globals.bodies.num_stars),3*sizeof(float));
        globals.bodies.star_body = body = add_box(c,-2.f,0.5f,0.5,0.1,&T); // IMPORTANT: the absolute value of the mass will be used when we later turn this body to dynamic (when the star is collected)
        body_set_collision_group_and_mask(c,body,COLLISION_GROUP_F,COLLISION_GROUP_C);  // collides only with group_c (our character), otherwise no (kinematic) collision report (see the sensor experiment)
        bind_body(c,body,TEAPOT_MESH_STAR,COLOR_YELLOW);
        nudge::log("[H] num_stars: %u/12\n",globals.bodies.num_stars);nudge::flush();
    }
    else globals.bodies.star_body=NUDGE_INVALID_BODY_ID;

    // experimental platform (box)
    {
        // -> kinematic animation construction
        // ------> key frames set (key frames for all animations are all in a single global array)
        const uint32_t start_key_frame = c->kinematic_data.key_frame_count;
        kinematic_data_reserve_key_frames(&c->kinematic_data,c->kinematic_data.key_frame_count+2);    // we'll use <=4 frames in total
        assert(c->kinematic_data.key_frame_capacity>=c->kinematic_data.key_frame_count+2);
        uint32_t* total_key_frames_count = &c->kinematic_data.key_frame_count;   // for all animations
        Transform T = identity_transform;
        Transform* pkfTransforms = &c->kinematic_data.key_frame_transforms[start_key_frame];       // at this point they're all set to identity
        T.time = 0.f; T.position[2]=-6.5f;T.position[1]=3.f;
        pkfTransforms[0] = T; (*total_key_frames_count)++;   // key frame 0
        T.time = 6.f; T.position[2]=6.5f;T.position[1]=0.f;
        pkfTransforms[1] = T; (*total_key_frames_count)++;   // key frame 1
        assert(*total_key_frames_count<=c->kinematic_data.key_frame_capacity);

        // ------> kinematic animation that use these key frames
        kinematic_data_reserve_animations(&c->kinematic_data,c->kinematic_data.animations_count+2);    // we'll use <=2 animation in total
        assert(c->kinematic_data.animations_capacity>=c->kinematic_data.animations_count+2);

        T=identity_transform;T.p[0]=10.5f;T.p[1]=-0.5f;T.p[2]=0.f;
        body = add_box(c,-1000.f,1.5f,0.2f,2.5f,&T);
        bind_body(c,body,TEAPOT_MESH_CUBIC_GROUND,COLOR_DARKGREEN);
        c->bodies.filters[body].flags|=BF_IS_PLATFORM;     // note that this flag does nothing in nudge.h

        KinematicData::Animation* ka = &c->kinematic_data.animations[c->kinematic_data.animations_count++];
        ka->key_frame_start = start_key_frame; ka->key_frame_count = 2;
        ka->loop_mode = KinematicData::Animation::LM_LOOP_PING_PONG;
        ka->body = body;
        ka->baseT = c->bodies.transforms[body];   // unless key frame Transforms are in the absolute space, this must always be set
        ka->use_baseT = true;   // unless key frame Transforms are in the absolute space, this must always be set
        ka->speed = 1.f;    // but too fast kinematic animations can lead to tunnelling
        ka->play_time = ka->offset_time = 4.f;  // Mainly used to time-offset one instance
        ka->playing = true;

        T=identity_transform;T.p[0]=0.f;T.p[1]=1.5f;T.p[2]=-10.5f;
        body = add_box(c,-1000.f,1.5f,0.2f,2.5f,&T);
        bind_body(c,body,TEAPOT_MESH_CUBIC_GROUND,COLOR_LIGHTSEAGREEN);
        c->bodies.filters[body].flags|=BF_IS_PLATFORM;     // note that this flag does nothing in nudge.h

        c->kinematic_data.animations[c->kinematic_data.animations_count] = *ka;
        ka = &c->kinematic_data.animations[c->kinematic_data.animations_count++];
        ka->baseT = c->bodies.transforms[body];nm_QuatFromAngleAxis(ka->baseT.q,M_PI*0.5,0,1,0);
        ka->body = body; ka->play_time = ka->offset_time = 0.f;
        ka->speed = 1.2f;
    }

    // another moving platform: lift here (box)
    {
        // -> kinematic animation construction
        // ------> key frames set (key frames for all animations are all in a single global array)
        const uint32_t start_key_frame = c->kinematic_data.key_frame_count;
        kinematic_data_reserve_key_frames(&c->kinematic_data,c->kinematic_data.key_frame_count+4);    // we'll use <=4 frames in total
        assert(c->kinematic_data.key_frame_capacity>=c->kinematic_data.key_frame_count+4);
        uint32_t* total_key_frames_count = &c->kinematic_data.key_frame_count;   // for all animations
        Transform T = identity_transform, *pkfTransforms = &c->kinematic_data.key_frame_transforms[start_key_frame];       // at this point they're all set to identity
        KinematicData::TimeMode mode=KinematicData::TM_NORMAL,*pkfModes = &c->kinematic_data.key_frame_modes[start_key_frame];   // at this point they're all set to KinematicData::TM_NORMAL;

        T.time = 4.f;T.p[0]=0.f;T.p[1]=0.f;T.p[2]=0.f;mode = KinematicData::TM_NORMAL;      pkfTransforms[0]=T;pkfModes[0]=mode;(*total_key_frames_count)++;   // key frame 0
        T.time = 4.f;T.p[0]=0.f;T.p[1]=4.f;T.p[2]=0.f;mode = KinematicData::TM_NORMAL;      pkfTransforms[1]=T;pkfModes[1]=mode;(*total_key_frames_count)++;   // key frame 1
        T.time = 4.f;T.p[0]=0.f;T.p[1]=4.f;T.p[2]=0.f;mode = KinematicData::TM_NORMAL;      pkfTransforms[2]=T;pkfModes[2]=mode;(*total_key_frames_count)++;   // key frame 2
        T.time = 4.f;T.p[0]=0.f;T.p[1]=0.f;T.p[2]=0.f;mode = KinematicData::TM_ACCELERATE;  pkfTransforms[3]=T;pkfModes[3]=mode;(*total_key_frames_count)++;   // key frame 3
        assert(*total_key_frames_count<=c->kinematic_data.key_frame_capacity);

        // ------> kinematic animation that use these key frames
        kinematic_data_reserve_animations(&c->kinematic_data,c->kinematic_data.animations_count+1);    // we'll use <=2 animation in total
        assert(c->kinematic_data.animations_capacity>=c->kinematic_data.animations_count+1);

        T=identity_transform;T.p[0]=-12.f;T.p[1]=-0.2f;T.p[2]=7.5f;
        body = add_box(c,-1000.f,1.f,0.2f,1.f,&T);
        bind_body(c,body,TEAPOT_MESH_CUBIC_GROUND,COLOR_DARKGREEN);
        c->bodies.filters[body].flags|=BF_IS_PLATFORM;     // note that this flag does nothing in nudge.h

        KinematicData::Animation* ka = &c->kinematic_data.animations[c->kinematic_data.animations_count++];
        ka->key_frame_start = start_key_frame; ka->key_frame_count = 4;
        ka->loop_mode = KinematicData::Animation::LM_LOOP_NORMAL;
        ka->body = body;
        ka->use_baseT = true; ka->baseT = c->bodies.transforms[body];   // unless key frame Transforms are in the absolute space, this must always be set
        ka->speed = 1.f;    // but too fast kinematic animations can lead to tunnelling
        ka->play_time = ka->offset_time = 0.f;  // Mainly used to time-offset one instance
        ka->playing = true;
    }

    // conveyor belts experiments (box) (static boxes with a linear velocity)
    T=identity_transform;T.p[0]=-11.0275f;T.p[1]=-0.198f;T.p[2]=5.f;
    globals.bodies.conveyor_belt_body0 = body = add_box(c,0,2.025f,0.2f,1.5f,&T);
    bind_body(c,body,TEAPOT_MESH_CUBIC_GROUND,COLOR_RED);
    c->bodies.momentum[body].velocity[0]=-1.5f; // this line does not work, because we're overriding this value at runtime in UpdatePhysics(), but it usually works
    c->bodies.properties[body].friction = 2.f;
    c->bodies.filters[body].flags|=BF_IS_PLATFORM;  // test (toggle and see the difference on character)

    T.p[0]=-12.f;T.p[1]=4.f-0.2f;T.p[2]=-0.25f;
    globals.bodies.conveyor_belt_body1 = body = add_box(c,0,1.f,0.2f,4.75f,&T);
    bind_body(c,body,TEAPOT_MESH_CUBIC_GROUND,COLOR_RED);
    c->bodies.momentum[body].velocity[2]=-1.5f; // this line does not work, because we're overriding this value at runtime in UpdatePhysics(), but it usually works
    c->bodies.properties[body].friction = 2.f;
    c->bodies.filters[body].flags|=BF_IS_PLATFORM;  // test (toggle and see the difference on character)

    // another moving platform here: [rotating platform test] (box)
    {
        // -> kinematic animation construction
        // ------> key frames set (key frames for all animations are all in a single global array)
        const uint32_t start_key_frame = c->kinematic_data.key_frame_count, num_frames_to_add = 7;
        kinematic_data_reserve_key_frames(&c->kinematic_data,c->kinematic_data.key_frame_count+num_frames_to_add);    // we'll use <=4 frames in total
        assert(c->kinematic_data.key_frame_capacity>=c->kinematic_data.key_frame_count+num_frames_to_add);
        Transform T = identity_transform, *pkfTransforms = &c->kinematic_data.key_frame_transforms[start_key_frame];       // at this point they're all set to identity
        unsigned new_frames_cnt=0;
        // All times here are in seconds for the default animation speed (ka->speed=1.0f), that we don't use...
        // Warning: 'T = identity_transform;' resets T.time too!
        T = identity_transform;T.time = 2.f;/*if !=0 platform waits some time*/     pkfTransforms[new_frames_cnt++]=T;
        T.time =  8.f;nm_QuatFromAngleAxis(T.q,M_PI*(0.66f*1.f),0,1,0);T.p[1]=1.5f; pkfTransforms[new_frames_cnt++]=T;
        T.time =  8.f;nm_QuatFromAngleAxis(T.q,M_PI*(0.66f*2.f),0,1,0);T.p[1]=3.0f; pkfTransforms[new_frames_cnt++]=T;
        T.time =  8.f;nm_QuatFromAngleAxis(T.q,M_PI*(0.66f*3.f),0,1,0);T.p[1]=4.5f; pkfTransforms[new_frames_cnt++]=T;
        T.time =  8.f;nm_QuatFromAngleAxis(T.q,M_PI*(0.66f*4.f),0,1,0);T.p[1]=3.0f; pkfTransforms[new_frames_cnt++]=T;
        T.time =  8.f;nm_QuatFromAngleAxis(T.q,M_PI*(0.66f*5.f),0,1,0);T.p[1]=1.5f; pkfTransforms[new_frames_cnt++]=T;
        T = identity_transform;T.time =  8.f;                                       pkfTransforms[new_frames_cnt++]=T;

        assert(new_frames_cnt==num_frames_to_add);  // well, <= could work
        assert(c->kinematic_data.key_frame_count==start_key_frame);c->kinematic_data.key_frame_count+=new_frames_cnt;  // mandatory
        assert(c->kinematic_data.key_frame_count<=c->kinematic_data.key_frame_capacity);

        // ------> kinematic animation that use these key frames
        kinematic_data_reserve_animations(&c->kinematic_data,c->kinematic_data.animations_count+1);    // we'll use <=2 animation in total
        assert(c->kinematic_data.animations_capacity>=c->kinematic_data.animations_count+1);

        T = identity_transform;T.p[0]=-0.2f;T.p[1]=0.;T.p[2]=0.f;
        const float com_offset[3]={0,0,-20.5f};
        body = add_box(c,-1000.f,2.5f,0.2f,1.5f,&T,com_offset);
        bind_body(c,body,TEAPOT_MESH_CUBIC_GROUND,COLOR_DARKGREEN);
        c->bodies.filters[body].flags|=BF_IS_PLATFORM;     // note that this flag does nothing in nudge.h
        c->bodies.infos[body].sk_user.i8[3]=0;  // we'll use this value as a driver for platform movement direction (-1,0,1); 0 = platform stands still

        globals.bodies.rotating_platform_animation_index = c->kinematic_data.animations_count;
        KinematicData::Animation* ka = &c->kinematic_data.animations[c->kinematic_data.animations_count++];
        ka->key_frame_start = start_key_frame; ka->key_frame_count = new_frames_cnt;
        ka->loop_mode = KinematicData::Animation::LM_LOOP_NORMAL;
        ka->body = body;
        ka->use_baseT = true; ka->baseT = c->bodies.transforms[body];nm_QuatFromAngleAxis(ka->baseT.q,M_PI*1.5f,0,1,0);   // unless key frame Transforms are in the absolute space, this must always be set
        ka->speed = 0.f;    // we'll set the speed dynamically when c->bodies.infos[body].sk_user.i8[3]!=0 (we'll set that when the character is on it)
        ka->play_time = ka->offset_time = 0.f;
        ka->playing = true; // we could trigger this instead of ka->speed... but it's almost the same

        c->bodies.transforms[body] = ka->baseT; // this places 'body' at frame 0 (if it has an identity T like in our case) even if ka->playing==false or ka->speed==0; (otherwise is just useless)
    }

    // static hollow cylinder
    const float cylRadius = 0.9f;
    T=identity_transform;T.p[0]=-7.5f;T.p[2]=1.f;T.p[1]=0.75f;
    body = extra::add_compound_hollow_cylinder(c,0,cylRadius*0.2f*3.5f,cylRadius,T.p[1],&T); // inner dimensions are fixed
    bind_body(c,body,TEAPOT_MESH_HOLLOW_CYLINDER,COLOR_ORANGE);

    // sensor experiment (test)
    T.p[1]=0.1f;
    const bool correct_way_to_make_a_sensor = false;
    if (correct_way_to_make_a_sensor) {
        body = add_box(c,1000.f,cylRadius/1.9f,T.p[1],cylRadius/1.9f,&T);
        float* tmp = c->bodies.properties[body].inertia_inverse;tmp[0]=tmp[1]=tmp[2]=0;
        tmp = c->bodies.properties[body].gravity;tmp[0]=tmp[1]=tmp[2]=0;
        c->bodies.filters[body].flags|=BF_IS_SENSOR;    // note that BF_IS_SENSOR ia completely unused in nudge.h (just an empty tag)
        c->bodies.filters[body].collision_mask = 0; // don't collide with any body
        c->bodies.filters[body].flags|=BF_NEVER_SLEEPING;   // this is bad! This way c->active_bodies.count is always positive!
        bind_body(c,body,TEAPOT_MESH_COUNT,COLOR_RED);
        /*
            Considerations:
            1) of course this needs code in the [collision query] section
            2) currently sensors work only this way (e.g. dynamic, non-sleeping bodies)
            What I don't like is the line: c->bodies.filters[body].flags|=BF_NEVER_SLEEPING;
            because in the physics view we can't detect when all the bodies are out of the
            c->active_bodies list anymore (when everything becomes dark black). So this is
            probably costly...
            I was thinking of removing BF_NEVER_SLEEPING from nudge.h, but if there's
            no other way to make sensors...
            (note that both static and dynamic bodies in (my mod of) nudge.h are always kept in sleep
            mode by design). Basically I want to be able to detect zero active_bodies...

            In a typical application, we can probably mitigate the 'correct_way' by removing the 'BF_NEVER_SLEEPING'
            flag when the body is distant (but it's a solution I don't like).
        */
    }
    else {
        // This is the only way to use a kinematic body as sensor but... it's not a sensor anymore (it collides with other bodies)
        // The catch is:
        // if collision filters of body A prevent collision with some body B, then contact data between body A and body B are not reported UNLESS A is a DYNAMIC body AND A is not sleeping
        // so if we want to use static or kinematic bodies as sensors, we must enable collisions with the body we must detect => it's not a sensor anymore
        // Note that for the purpose of this demo the effect is the same
        body = add_box(c,0.f,cylRadius/1.9f,T.p[1],cylRadius/1.9f,&T);
        c->bodies.filters[body].flags|=BF_IS_SENSOR;    // note that BF_IS_SENSOR ia completely unused in nudge.h (just an empty tag)
        //c->bodies.filters[body].collision_mask = 0; // [NOPE!] don't collide with any body (if set => no more collisions reported here)
        bind_body(c,body,TEAPOT_MESH_COUNT,COLOR_RED);
    }


    // (box stack test)
    const int enable_sad_stuff = 0;
    if (enable_sad_stuff) {
        unsigned num_stacked_boxes = 10;
        const float stacked_box_hsize[3] = {0.5f,0.175f,0.25f};
        const float stacked_box_mass = 2.f;
        if (num_stacked_boxes>c->MAX_NUM_BOXES-c->colliders.boxes.count) num_stacked_boxes = c->MAX_NUM_BOXES-c->colliders.boxes.count;
        T=identity_transform;T.p[0]=-4.f;T.p[2]=2.95f;T.p[1]=stacked_box_hsize[1];
        for (unsigned j=0;j<num_stacked_boxes;j++)   {
            body = add_box(c,stacked_box_mass,stacked_box_hsize[0],stacked_box_hsize[1],stacked_box_hsize[2],&T);
            bind_body(c,body,TEAPOT_MESH_CUBE,COLOR_RED);

            // a trick to improve stack stability is to start with a big mass and/or inertia in the bottom bricks... to test
            //const float linearMass = stacked_box_mass*50.f*(num_stacked_boxes-j)/(num_stacked_boxes/*/2*/);
            //float linearMass = 50000.f*stacked_box_mass;
            //calculate_box_inertia_inverse(c->bodies.properties[body].inertia_inverse,linearMass,stacked_box_hsize[0],stacked_box_hsize[1],stacked_box_hsize[2]);
            //c->bodies.properties[body].mass_inverse=1.f/linearMass; // optional (well, it can worsen things)
            //c->bodies.properties[body].gravity[1]=-1.5f;  // what about this cheat? No!

            // the general solution to improve stacking stability is to increase c->simulation_params.num_iterations_per_substep.
            // but it's much more expensive then increasing the local inertia (or the mass) of the bricks.

            // however of course increasing the local inertia, or the mass of the bricks, has some side-effect on the dynamic response
            // of the body.
            // But usually it's a good practice and this trick is commonly used in many physic engines to improve the stability of a body.

            // Also incrementing c->simulation_params.sleeping_threshold_linear_velocity_squared and sleeping_threshold_angular_velocity_squared
            // could work, but simply because the stack goes to sleep early (however ATM there's no per-body sleeping threshold and sleeping bodies sometimes move (there's probably a bug somewhere))
            c->bodies.idle_counters[body]=0xFF; // sets the brick to sleep... this is probably good when we don't need to make tests... and it's wrong (lower bricks might fall without waking upper bricks up, if there are micro-spaces in the stack)

            T.p[1]+= stacked_box_hsize[1]*2.f;
        }
    }

    // (domino test)
    unsigned num_domino_boxes = 25;
    const float domino_box_hsize[3] = {0.3f,0.6f,0.1f};
    const float domino_spacing = domino_box_hsize[2]*2.f*3.5f;
    const float domino_box_mass = 0.4f;
    if (num_domino_boxes>c->MAX_NUM_BOXES-c->colliders.boxes.count) num_domino_boxes = c->MAX_NUM_BOXES-c->colliders.boxes.count;
    T = identity_transform;T.p[0]=8.f;T.p[1]=domino_box_hsize[1];T.p[2]=8.f;
    for (unsigned j=0;j<num_domino_boxes;j++)   {
        body = add_box(c,domino_box_mass,domino_box_hsize[0],domino_box_hsize[1],domino_box_hsize[2],&T);
        bind_body(c,body,TEAPOT_MESH_CUBE,COLOR_RED);

        // advance T
        nm_QuatRotate(T.q, M_DEG2RAD(j<num_domino_boxes/2?4.f:-6.5f), 0,1,0);
        float zAxis[3];nm_QuatGetAxisZ(zAxis,T.q);
        for (int l=0;l<3;l++) T.p[l] -= zAxis[l]*domino_spacing;
    }
    T = identity_transform;

    // (random falling objects)
    int num_falling_boxes = (int)c->MAX_NUM_BOXES-(int)colliders_get_num_remaining_boxes(c)/2;if (num_falling_boxes>24) num_falling_boxes=24;
    for (int j=0;j<num_falling_boxes;j++)   {
        float size[3];
        for (int l=0;l<3;l++)   {
            size[l] = 0.5f*(float)rand() * (1.0f/(float)RAND_MAX) + 0.5f;
            if (l!=1) T.p[l] =  (((float)rand() * (1.0f/(float)RAND_MAX)) - 0.5f) * 1.5f*10.f;
            else T.p[l] = 10.f + (float)rand() * (1.0f/(float)RAND_MAX) * 5.0f;
            T.q[l] = (float)rand() * (M_PI/(float)RAND_MAX);
        }
        nm_QuatNormalize(T.q);
        const float mass = size[0]*size[1]*size[2];
        body = add_box(c,mass,size[0]*0.5f,size[1]*0.5f,size[2]*0.5f,&T);assert(body!=NUDGE_INVALID_BODY_ID);
        bind_body(c,body,TEAPOT_MESH_CUBE,COLOR_NONE);
    }
    T = identity_transform;
    int num_falling_spheres = (int)c->MAX_NUM_SPHERES-(int)colliders_get_num_remaining_spheres(c)/2;if (num_falling_spheres>24) num_falling_spheres=24;
    for (int j=0;j<num_falling_spheres;j++)   {
        const float radius = 0.25f*((float)rand()*(1.0f/(float)RAND_MAX) + 0.5f);
        for (int l=0;l<3;l++)   {
            if (l!=1) T.p[l] =  (((float)rand() * (1.0f/(float)RAND_MAX)) - 0.5f) * 1.5f*10.f;
            else T.p[l] = 10.f + (float)rand() * (1.0f/(float)RAND_MAX) * 5.0f;
        }
        const float mass = 8.18879f*radius*radius*radius *0.5f;
        body = add_sphere(c,mass,radius,&T);assert(body!=NUDGE_INVALID_BODY_ID);
        bind_body(c,body,TEAPOT_MESH_SPHERE1,COLOR_NONE);
    }

}
void DestroyPhysics() {nudge::destroy_context(c);}

void UpdatePhysics(double elapsedSecondsFromLastCall) {
    // simulation
    const unsigned numSubSteps = nudge::pre_simulation_step(c,elapsedSecondsFromLastCall); // Mandatory
    if (numSubSteps) {
        const double granularElapsedTime = numSubSteps*c->simulation_params.time_step;

        // common quantities we'll use in this scope
        float* angle = &c->user.f32[1];    // here we use a user field of the nudge::context, so that it gets saved/load with it
        (*angle)+=(float)(granularElapsedTime*1.5);if (*angle>M_PI*2.0f) (*angle)-=M_PI*2.0f;
        const float sinAngle = sinf(*angle),cosAngle = cosf(*angle);

        // we must manually advance our 'manual' kinematic bodies by granularElapsedTime
        if (globals.bodies.kinematic_body<c->bodies.count) {
            using namespace nudge;
            const float basePosition[3] = {1.5f,0.5f,5.0};const float amplitude = 3.0f;
            const Transform* T = &c->bodies.transforms[globals.bodies.kinematic_body];  // Tip: don't change this directly!
            assert(T->body==globals.bodies.kinematic_body);    // did you know it?

            Transform newT= { {{basePosition[0]+amplitude*sinAngle,T->p[1],basePosition[2]+amplitude*cosAngle}},{},{{T->r[0],T->r[1],T->r[2],T->r[3]}} };assert(newT.r[3]!=0);
            nm_QuatFromAngleAxis(newT.r,*angle+M_PI*0.5f,0.f,1.f,0.f);   // this line rotates the body while moving in circle
            // this function moves the current body transform (i.e. T) to the new one (newT). It does two things:
            // 1) sets: T=newT
            // 2) updates the linear/angular velocities of the body (necessary to improve collision detection)
            // For this to work the passed delta time (i.e. granularElapsedTime) must be small
            TransformAssignToBody(c,T->body,newT,granularElapsedTime);
        }
        // here we change the velocity of the two conveyor belts dynamically
        if (globals.bodies.conveyor_belt_body0<c->bodies.count) {c->bodies.momentum[globals.bodies.conveyor_belt_body0].velocity[0] = -0.5f+1.5f*sinAngle;}
        if (globals.bodies.conveyor_belt_body1<c->bodies.count) {c->bodies.momentum[globals.bodies.conveyor_belt_body1].velocity[2] = 1.f*sinAngle;}
        // here we adjust the moving speed of the platform rotating around
        if (globals.bodies.rotating_platform_animation_index<c->kinematic_data.animations_count) {
            nudge::KinematicData::Animation* an = &c->kinematic_data.animations[globals.bodies.rotating_platform_animation_index];
            if (an->body<c->bodies.count) {
                if (an->playing) {
                    const float rotating_platform_max_speed = 0.45f;
                    const int8_t dir = c->bodies.infos[an->body].sk_user.i8[3];
                    const float speed_step = granularElapsedTime*rotating_platform_max_speed;
                    //if (globals.key.pressed&KEY_MOUSE_BUTTON_RIGHT && globals.key.down&KEY_CTRL) c->bodies.infos[an->body].sk_user.i8[3]=dir==0?1:-dir;
                    if (dir>0) {if (an->speed<rotating_platform_max_speed) {an->speed+=speed_step;if (an->speed>rotating_platform_max_speed) an->speed=rotating_platform_max_speed;}}
                    else if (dir<0)  {if (an->speed>-rotating_platform_max_speed) {an->speed-=speed_step;if (an->speed<-rotating_platform_max_speed) an->speed=-rotating_platform_max_speed;}}
                }
            }
        }
        // here we manually advance our character body (even if it's dynamic)
        if (globals.bodies.character_body<c->bodies.count) {
            using namespace nudge;
            Transform* T = &c->bodies.transforms[globals.bodies.character_body];assert(globals.bodies.character_body==T->body);
            const BodyInfo* info = &c->bodies.infos[T->body];

            const uint32_t state_key_down = globals.key.down;
            const uint32_t state_key_pressed = globals.key.pressed;

            // we advance T directly (note that key handling is frame rate dependent, but it's just a test):
            const int32_t aux_body = info->aux_bodies[0];assert(aux_body<(int32_t)c->bodies.count);
            //const bool character_is_on_something = aux_body>=0;
            const bool character_must_jump = aux_body>=0 && ((state_key_pressed&KEY_J) || (state_key_pressed&KEY_MOUSE_BUTTON_LEFT && globals.use_character_camera));
            const bool character_is_on_platform =  aux_body>=0 && (c->bodies.filters[aux_body].flags&BF_IS_PLATFORM);

            // experimental (needs a LOT of work: character get stuck and WSJ stop working sometimes)
            const bool character_is_against_static_obstacle = (aux_body==-1) && info->aux_bodies[1]>=0;
            //if (aux_body>=0) glprintf("character_is_against_ground\n");
            //if (character_is_against_static_obstacle) glprintf("character_is_against_static_obstacle!\n");

            if (character_must_jump) {
                const float amount = 5.f;
                c->bodies.momentum[globals.bodies.character_body].velocity[1]+=amount;
                if (c->bodies.filters[globals.bodies.character_body].flags&BF_IS_DYNAMIC) c->bodies.idle_counters[globals.bodies.character_body]=0;  // wakes up body (dynamic body only)
            }
            else if (((state_key_down&KEY_WASD) || character_is_on_platform) && !character_is_against_static_obstacle)   {
                Transform tr = *T;   // copy
                if (state_key_down&(KEY_A|KEY_D)) {
                    const float amount = 0.05f*((state_key_down&KEY_A)?1.0f:-1.0f);
                    float yaw[4];nm_QuatFromAngleAxis(yaw,amount,0,1,0);
                    nm_QuatMul(tr.rotation,T->rotation,yaw);
                    nm_QuatNormalize(tr.rotation);
                    // optional stuff (change the moving direction of the platform rotating around)
                    if (character_is_on_platform && globals.bodies.rotating_platform_animation_index<c->kinematic_data.animations_count && (unsigned)aux_body==c->kinematic_data.animations[globals.bodies.rotating_platform_animation_index].body) {
                        float ch_axis_z[3];nm_QuatGetAxisZ(ch_axis_z,tr.rotation);
                        float pl_axis_x[3];nm_QuatGetAxisX(pl_axis_x,c->bodies.transforms[aux_body].q);
                        const float dot = nm_Vec3Dot(ch_axis_z,pl_axis_x);
                        const float min_abs_dot = 0.9f;
                        if (dot>min_abs_dot) c->bodies.infos[aux_body].sk_user.i8[3]=1;
                        else if (dot<-min_abs_dot) c->bodies.infos[aux_body].sk_user.i8[3]=-1;
                    }
                    // end optional stuff
                }
                if (state_key_down&(KEY_W|KEY_S)) {
                    // Note that this allows only walking on planar surfaces... Y movement of everything about 'character ehm controller' has not be handled...
                    const float amount = 0.05f*((state_key_down&KEY_W)?1.f:-0.5f);
                    float angle,axis[3];nm_QuatToAngleAxis(tr.rotation,&angle,axis);
                    angle*=axis[1]; // axis[1] can be 1 or -1 AFAICS
                    const float sinangle=amount*sinf(angle),cosangle=amount*cosf(angle);
                    //glprintf("Angle axis:%1.3f; {%1.3f,%1.3f,%1.3f}\n",angle,axis[0],axis[1],axis[2]);
                    tr.position[0] = T->position[0] + sinangle;
                    tr.position[2] = T->position[2] + cosangle;
                    //glprintf("axis[1]=%1.2f x+=%1.4f; z+=%1.4f;\n",axis[1],sinangle,cosangle);
                }

                // 2) assign the new transform with this function [From a (separated) newT: -> T=newT and linear/angular velocities updated to improve collision detection]

                // The following is a hack line, most likely necessary because nudge::TransformAssignToBody(...) is wrong when
                // extracting the angular velocities from the two transforms (any help is welcome). UPDATE: Nope, function seems correct (test with zero gravity and see)
                // But beside that if body inertia inverse vector is [0,0,0], how can the body rotate?
                // Maybe we can just remove the line below and then make the angular velocity decrease with time
                // Easiest way is to keep the hack (character won't use its angular velocity in collisions, but that does not happen frequently... but some collision artifact might appear)
                T->rotation[0]=tr.rotation[0];T->rotation[1]=tr.rotation[1];T->rotation[2]=tr.rotation[2];T->rotation[3]=tr.rotation[3];
                float* velocity = c->bodies.momentum[globals.bodies.character_body].velocity;
                const float oldVelocity[3] = {velocity[0],velocity[1],velocity[2]};
                TransformAssignToBody(c,T->body,tr,globals.instantFrameTime,character_is_on_platform?aux_body:-1);   // This wakes up body if dynamic too
                velocity[1] = oldVelocity[1];  // velocityY is necessary to make jumping work better
            }
            // Last note:
            // I'm not sure that a character controller can be done with this
            // simple, proof-of-a-concept header-only physics sengine (although IMHO
            // it's one of the most useful single-header C file ever written!)
        }
        // here we animate the star
        if (globals.bodies.star_body<c->bodies.count && c->bodies.filters[globals.bodies.star_body].flags&nudge::BF_IS_STATIC_OR_KINEMATIC) {
            using namespace nudge;assert(globals.bodies.num_stars<12);
            const float y = getStarPosition(globals.bodies.num_stars)[1];
            Transform* T = &c->bodies.transforms[globals.bodies.star_body];T->p[1]=y+0.2f*cosAngle;
            nm_QuatFromAngleAxis(T->q,*angle,0,1,0);

            //test (to remove)
            //body_scale(c,globals.bodies.star_body,1.f+0.01f*sinAngle);
            //body_scale(c,globals.bodies.star_body,-1.f+0.1f*sinAngle);
        }
    }
    nudge::simulation_step(c);  // Mandatory


    // collision query
    if (numSubSteps>0)  {        
        // This part is all WIP, since I haven't understood how it works yet... => code is a mess!
        using namespace nudge;
        const ContactData* cd = &c->contact_data;
        assert(c && cd);
        for (unsigned i = 0; i < cd->count; i++) {
            const unsigned a = cd->bodies[i].a, b = cd->bodies[i].b;
            const BodyFilter *a_filter = &c->bodies.filters[a], *b_filter = &c->bodies.filters[b];

            //assert(a!=b && !(a_filter->flags&BF_IS_DISABLED_OR_REMOVED) && !(b_filter->flags&BF_IS_DISABLED_OR_REMOVED)); // seems to be always true in my tests (OK)

            //assert(a_filter->flags&BF_IS_DYNAMIC || b_filter->flags&BF_IS_DYNAMIC); // asserts
            //assert(((a_filter->flags | b_filter->flags) & BF_IS_DYNAMIC) == BF_IS_DYNAMIC);   // same as above (!=0 works here, because BF_IS_DYNAMIC is one-bit mask)

            //assert(!((a_filter->flags&BF_IS_STATIC_OR_KINEMATIC) && (b_filter->flags&BF_IS_STATIC_OR_KINEMATIC))); // asserts
            //assert(((a_filter->flags | b_filter->flags) & BF_IS_STATIC_OR_KINEMATIC) != BF_IS_STATIC_OR_KINEMATIC); // same as above
            //assert(((a_filter->flags&b_filter->flags)&BF_IS_STATIC_OR_KINEMATIC)==0);   // not the same as above; asserts only if a and b are both static or both kinematic
            if (
                ((a_filter->flags&BF_IS_STATIC_OR_KINEMATIC) && (b_filter->flags&BF_IS_STATIC_OR_KINEMATIC)) // this should not happen, but sometimes it happens. Why?
                || a==b || (a_filter->flags&BF_IS_DISABLED_OR_REMOVED) || (b_filter->flags&BF_IS_DISABLED_OR_REMOVED) // this never happens, but for robustness we leave it
                ) continue;
            //assert(a_filter->flags&BF_IS_DYNAMIC || b_filter->flags&BF_IS_DYNAMIC);   // now OK

            // sensor test ------------
            if (a_filter->flags&BF_IS_SENSOR || b_filter->flags&BF_IS_SENSOR)  {
                unsigned detected_body = NUDGE_INVALID_BODY_ID;
                if (a_filter->flags&BF_IS_SENSOR && b_filter->flags&BF_IS_DYNAMIC && !(b_filter->flags&BF_IS_SENSOR)) detected_body=b;
                else if (b_filter->flags&BF_IS_SENSOR && a_filter->flags&BF_IS_DYNAMIC && !(a_filter->flags&BF_IS_SENSOR)) detected_body=a;
                if (detected_body!=NUDGE_INVALID_BODY_ID) c->bodies.momentum[detected_body].velocity[1]=10.f;
            }
            // end sensor test --------

            const FlagMask a_is_character = a_filter->flags&BF_IS_CHARACTER;
            const FlagMask b_is_character = b_filter->flags&BF_IS_CHARACTER;
            if ((a_is_character||b_is_character) && !(a_is_character&&b_is_character))  // Last condition should be handled... (but how to know which character is above the other?)
            {
                // These code assigns a 'ground' to 'character' (1-to allow jumping only when is present;2-to allow correct walking on moving platforms)
                const uint32_t character = a_is_character?a:b,other_body = a_is_character?b:a;int dbg=0;
                if (other_body==globals.bodies.star_body
                        //|| (dbg=globals.key.down&KEY_SHIFT && globals.key.down&KEY_SPACE)
                        ) {
                    BodyFilter* other_body_filters;
                    if (dbg) {*( (uint32_t*) &other_body) = globals.bodies.star_body;other_body_filters = &c->bodies.filters[other_body];}
                    else other_body_filters = a_is_character ? (BodyFilter*)b_filter : (BodyFilter*)a_filter;
                    if (other_body_filters->flags&BF_IS_STATIC_OR_KINEMATIC)    {
                        ++globals.bodies.num_stars;nudge::log("[H] num_stars: %u/12\n",globals.bodies.num_stars);nudge::flush();
                        body_change_motion_state(c,other_body,BF_IS_DYNAMIC);
                        other_body_filters->collision_mask=0;   // turn off collisions with everything
                        const float *camPos = globals.camera[globals.use_character_camera].cameraPos, *starPos = c->bodies.transforms[other_body].p;
                        float vec[3]={camPos[0]-starPos[0],camPos[1]+10.f-starPos[1],camPos[2]-starPos[2]},len=nm_Vec3Normalize(vec);
                        const float amount=len*0.5f;vec[0]*=amount;vec[1]*=amount;vec[2]*=amount;
                        float *lin_vel = c->bodies.momentum[other_body].velocity, *ang_vel = c->bodies.momentum[other_body].angular_velocity;
                        memcpy(lin_vel,vec,3*sizeof(float));ang_vel[0]=0;ang_vel[1]=15.f;ang_vel[2]=0;
                    }
                }
                else {
                    int16_t* character_aux_bodies = &c->bodies.infos[character].aux_bodies[0];
                    const int detect_static_obstacle = (globals.key.down&(KEY_W|KEY_S)) && character_aux_bodies[1]==-1 && (c->bodies.filters[other_body].flags&BF_IS_STATIC_OR_KINEMATIC) && other_body!=(unsigned)character_aux_bodies[0];
                    if (character_aux_bodies[0]==-1 || detect_static_obstacle)    {
                        // Check if contact is below 'character', and set aux_body for 'character'
                        // cd->data->position and T->position are in world space
                        // we assume contact normal is in the world space... (it makes no sense, because it should depend on the order of the two bodies, but it works)
                        // actually we are just testing: contact->normal[1]<-0.95f AFAICS, so the normal points down not up...
                        // we should do extensive tests to understand if body order is random or if static/kinematic vs dynamic has a specific order, etc...
                        const Contact* contact = &cd->data[i];
                        if (character_aux_bodies[0]==-1) {
                            const float dot_down =  contact->normal[0]*0+contact->normal[1]*-1+contact->normal[2]*0;  // TODO: Better dot between contact->normal and body gravity (and normalize)
                            const int is_on_ground = (a_is_character&&dot_down>0.9f) || (b_is_character&&dot_down<-0.9f);
                            if (is_on_ground) {
                                character_aux_bodies[0]=(int16_t) other_body;
                                //glprintf("Ground! char_body=%c normal:{%1.2f,%1.2f,%1.2f}; dot_down=%1.2f;\n",a_is_character?'a':'b',contact->normal[0],contact->normal[1],contact->normal[2],dot_down);
                                // results:
                                /* On static ground (body 0): a_is_character, contact->normal:{0,-1,0}, dot_down= 1
                                 * On a dynamic cube:         b_is_character, contact->normal:{0, 1,0}, dow_down=-1
                                 * so the contact normal direction is from a_body to b_body
                                */
                            }
                        }
                        if (detect_static_obstacle) {
                            float vel_norm[3];// suboptimal, we could store it outside the loop
                            //const float* vel = c->bodies.momentum[character].velocity;
                            //const float speed = nm_Vec3Normalized(vel_norm,vel);
                            nm_QuatGetAxisZ(vel_norm,c->bodies.transforms[character].q);if (globals.key.down&KEY_S) {vel_norm[0]=-vel_norm[0];vel_norm[1]=-vel_norm[1];vel_norm[2]=-vel_norm[2];};
                            const float dot_vel =  contact->normal[0]*vel_norm[0]+contact->normal[1]*vel_norm[1]+contact->normal[2]*vel_norm[2];
                            const int is_in_direction = (a_is_character&&(dot_vel>0.15f)) || (b_is_character&&(dot_vel<-0.15f));
                            if (is_in_direction) {
                                character_aux_bodies[1]=(int16_t) other_body;
                                //glprintf("Obstacle! char_body=%c normal:{%1.2f,%1.2f,%1.2f}; dot_vel=%1.2f;\n",a_is_character?'a':'b',contact->normal[0],contact->normal[1],contact->normal[2],dot_vel);
                                // results:
                                /* On static ground (body 0): a_is_character, contact->normal:{0,-1,0}, dot_down= 1
                                 * On a dynamic cube:         b_is_character, contact->normal:{0, 1,0}, dow_down=-1
                                 * so the contact normal direction is from a_body to b_body
                                */
                            }
                            //glprintf("Obstacle! char_body=%c normal:{%1.2f,%1.2f,%1.2f}; dot_vel=%1.2f; speed=%1.3f;\n",a_is_character?'a':'b',contact->normal[0],contact->normal[1],contact->normal[2],dot_vel,speed);

                        }
                    }
                }
            }
            // Here we simply report collisions
            //#                   define TEST_CONTACT_CHECKING
#                   ifdef TEST_CONTACT_CHECKING
            if (a==body || b==body) {
                //const unsigned other_body = a==body?b:a;if (c->bodies.collision_groups[other_body]==COLLISION_GROUP_A)
                // the line above can be used to test reports on the skittles
                {
                    const Contact* contact = &cd->data[i];
                    printf("Frame %lu) Collision between bodies: %u and %u at pos {%1.3f,%1.3f,%1.3f} norm {%1.3f,%1.3f,%1.3f} depth: %1.4f\n",
                           args->num_frames,a,b,contact->position[0],contact->position[1],contact->position[2],
                            contact->normal[0],contact->normal[1],contact->normal[2],contact->penetration);
                    ++j;
                }
                /* some notes I've discovered:
                   1) 'body' can be found in 'a' or in 'b' (and probably 'contact.normal' changes its direction [no, I think all it's in world space coords])
                   2) The same body pair can reappear multiple times in a single frame, but 'contact' differs.
                   */
            }
#                   endif
        }
        //if (j>0) fflush(stdout);
        // dbg:
        //static unsigned num_collisions = cd->count;if (num_collisions!=cd->count) {num_collisions=cd->count;SDL_Log("[PhysicFrame:%llu] num_collisions_pairs=%u (num_wrong_contacts=%u)\n",c->simulation_params.num_frames,cd->count,num_wrong_contacts);}
    }
}

void CharacterControllerFixSinkingEffectOnFall(unsigned body,float* mMatrix16,int16_t aux_body);  // forward declaration
int glIsAabbVisible(const float*__restrict mMatrix16,float aabbCenterX,float aabbCenterY,float aabbCenterZ,float aabbHalfExtentX,float aabbHalfExtentY,float aabbHalfExtentZ); // forward declaration
int glIsAabbVisible(const float* __restrict Tpos3,float aabbEnlargedRadius); // forward declaration
void DrawPhysics(const int rendering_pass,const int num_rendering_passes)  {
    using namespace nudge;
    // Note: this function is part of DrawGL, and it occasionally contains gl calls

    assert(globals.is_shadow_pass == (rendering_pass<num_rendering_passes-1));
    const unsigned is_shadow_pass = globals.is_shadow_pass;

    // we use a different color for (dynamic) bodies with a center of mass offset,
    // and another for sleeping (dynamic) bodies (but only when !use_graphic_transform)
    const ColorEnum no_active_body_color = COLOR_BLACK;
    const ColorEnum sleeping_color = COLOR_DARKSLATEGRAY;
    const ColorEnum com_offset_body_color = COLOR_DARKBLUE;

    const FlagMask BF_FRUSTUM_CULLED = is_shadow_pass ? BF_IS_SHADOW_PASS_FRUSTUM_CULLED : (FlagMask) BF_IS_FRUSTUM_CULLED;
    unsigned* num_frustum_culled_bodies = is_shadow_pass ? &globals.num_shadow_pass_frustum_culled_bodies : &globals.num_frustum_culled_bodies;
    float* mMatrix=NULL,mMatrixTmp[16];unsigned num_sleeping_and_dynamic=0;globals.num_frustum_culled_bodies=globals.num_shadow_pass_frustum_culled_bodies=0;
    const int use_graphic_transform = globals.use_graphic_transform;
    for (unsigned body=0,body_count=c->bodies.count;body<body_count;body++)	{
        BodyFilter* filter = &c->bodies.filters[body];	// BodyFilter hosts per-body flags, collision masks and sleeping state
        if (filter->flags&BF_IS_DISABLED_OR_REMOVED) continue;

        const BodyInfo* info = &c->bodies.infos[body];   // BodyInfo hosts our custom per-body (user-side) data (in this example the TEAPOT_MESH_ enum and the COLOR_ enum), plus per-body aabb data
        const TeapotMeshEnum shape = bodyinfo_get_shape_enum(info);
        filter->flags&=~BF_FRUSTUM_CULLED;   // removes this flag from the body

        if (rendering_pass==0) {
            //------------------------------------------------------------
            // we exploit this loop to remove fallen bodies so that their colliders (i.e. collision shapes)
            // are freed and the body indices are reusable by nudge::add_ (this happens automatically)
            if (c->bodies.transforms[body].p[1]<-40.f && filter->flags&BF_IS_DYNAMIC) {
                // here we just remove (graphic) boxes and spheres, and respawn other bodies
                if ((shape==TEAPOT_MESH_CUBE || shape==TEAPOT_MESH_SPHERE1 || (body==globals.bodies.star_body && globals.bodies.num_stars==12)) && !(filter->flags&(BF_IS_CHARACTER|BF_IS_PLATFORM))) {
                    remove_body(c,body);
                    if (body==globals.bodies.star_body) {
                        globals.bodies.star_body=NUDGE_INVALID_BODY_ID;
                        if (globals.bodies.character_body<c->bodies.count) {bind_body(c,globals.bodies.character_body,TEAPOT_MESH_CHARACTER,COLOR_ORANGE);c->bodies.momentum[globals.bodies.character_body].velocity[1]=5.f;}
                    }
                    continue;
                }
                else if (body==globals.bodies.star_body) {
                    assert(globals.bodies.num_stars<12);// when a star falls to the bottom we must display next star
                    body_change_motion_state(c,body,BF_IS_KINEMATIC);filter->collision_mask=COLLISION_GROUP_C;  // restore flags/collision mask
                    memcpy(c->bodies.transforms[body].p,getStarPosition(globals.bodies.num_stars),3*sizeof(float));
                    assert(c->bodies.transforms[body].p[1]>-40.f);
                }
                else {Transform* T = &c->bodies.transforms[body];T->p[0]=0.f;T->p[1]=20.f;T->p[2]=0.0f;} // respawn all the other bodies
            }
            // end loop exploiting... back to basic drawing of all bodies
            //-----------------------------------------------------------
        }
        const bool is_dynamic = filter->flags&BF_IS_DYNAMIC;
        const int is_sleeping_and_dynamic = c->bodies.idle_counters[body]==0xFF && is_dynamic;num_sleeping_and_dynamic+=(is_sleeping_and_dynamic?1:0);
        const int has_com_offset = filter->flags&BF_HAS_COM_OFFSET;

        assert(info->user.u16[1]<COLOR_COUNT);// we have stuffed the per-body color enum in info->userUint16[1]
        ColorEnum color = (ColorEnum) bodyinfo_get_color_enum(info);assert(color!=COLOR_NONE); // COLOR_NONE is replaced in bind_body(...)

        mMatrix = &globals.mMatrices[16*body];
        if (rendering_pass==0) calculate_graphic_transform_for_body(c,body,mMatrix);	// what this does is: 1) smoothes the graphic transform (using DeltaTimes less than timeStep); 2) embeds the center of mass offset (so that it can be stripped from the aabb later [*])

        // frustum culling on mMatrix here
        const float* aabb_hextents = info->aabb_half_extents;   // 3-floats array
        const float aabb_center[3] = {info->aabb_center[0]+info->com_offset[0],info->aabb_center[1]+info->com_offset[1],info->aabb_center[2]+info->com_offset[2]};  // here we strip the comOffset from aabb_center, because we're culling on a mMatrix that embeds it already [*]
        if (globals.use_frustum_culling && !glIsAabbVisible(mMatrix,aabb_center[0],aabb_center[1],aabb_center[2],aabb_hextents[0],aabb_hextents[1],aabb_hextents[2])) {
            filter->flags|=BF_FRUSTUM_CULLED;
            ++(*num_frustum_culled_bodies);
            continue;
        }
        // end frustum culling on mMatrix

        if (use_graphic_transform)	{
            // experiment (to remove) ------
            if (rendering_pass==0 && (filter->flags&BF_IS_CHARACTER) && globals.bodies.fix_character_sinking_effect_on_fall && info->aux_bodies[0]>=0)
                CharacterControllerFixSinkingEffectOnFall(body,mMatrix,info->aux_bodies[0]);  // [experimental] attempt to mitigate the 'sinking effect' of characters after a jump or a fall
            // -----------------------------

            // 2) inglobes the com_offset (if present), so that we should not manually translate our graphic mesh


            // Now we just add our custom code to draw our graphic body (= the graphic representation of the body)
            // => here we don't retrieve any physic quantity at all to determine the scaling we must apply
            // => instead we retrieve the shape axis aligned bounding box extents (inside BodyInfo), which is a geometric quantity, in a suitable format
            //    Note that that quantity is NOT a physic quantity, because it's NOT USED by nudge.h at all (it was added just for user convenience)

            // Note that here we don't use info->aabb_center at all to draw the body
            // Here in our case the sum (info->aabb_center+info->com_offset) is always nearly {0,0,0}: this sum is the aabb center we have introduced in add_compund(...) calls, stripped by (=regardless of) the center of mass offset ([*])
            // In any case, even if the sum is not zero, normally we should just skip it when drawing the geometry, if our graphic mesh has the same aabb center (it should be true when we build the colliders to cover a predefined mesh).
            // In case of graphic center mismatches I suggest to: 1) center the mesh built with add_compound(...), using last argument, or 2) Calculate (info->aabb_center+info->com_offset) and use it when redering the graphic body.

            if (shape<TEAPOT_MESH_COUNT) {
                if (is_shadow_pass) {
                    if (shape==TEAPOT_MESH_CAPSULE) {
                        // For capsules, Teapot_SetScaling(x,y,z) is interpreted this way:
                        // diameter = (x+z)/2; cylinderHeight = y. So the total height is: (cylinderHeight + diameter)
                        // This was made to force uniform scaling of the two half spheres.
                        const float r = (aabb_hextents[0]+aabb_hextents[2])*0.5f;   // radius of the capsule
                        const float hh = aabb_hextents[1]-r;                        // half height of the cylinder
                        Dynamic_Resolution_Shadow_Set_Scaling(r*2.f,hh*2.f,r*2.f);
                    }
                    else if ((shape>=TEAPOT_MESH_CUBE && shape<=TEAPOT_MESH_CUBE_ROUNDED)
                             || (shape>=TEAPOT_MESH_SPHERE1 && shape<=TEAPOT_MESH_SPHERE1)) {
                        // This branch is optional (performance optimization for meshes that are originally stored inside teapot.h with a cubic aabb)
                        // It can probably be extended to a few other meshes
                        Dynamic_Resolution_Shadow_Set_Scaling(aabb_hextents[0]*2.f,aabb_hextents[1]*2.f,aabb_hextents[2]*2.f);
                    }
                    else {
                        // This should work for everything (with a few exception (e.g. capsules)
                        float graphic_mesh_half_extents[3];Teapot_GetMeshAabbHalfExtents(shape,graphic_mesh_half_extents);
                        Dynamic_Resolution_Shadow_Set_Scaling(aabb_hextents[0]/graphic_mesh_half_extents[0],
                                aabb_hextents[1]/graphic_mesh_half_extents[1],
                                aabb_hextents[2]/graphic_mesh_half_extents[2]);
                    }
                    Dynamic_Resolution_Shadow_Set_MMatrix(mMatrix);
                    Teapot_LowLevel_DrawElements(shape);
                }
                else {
                    Teapot_SetColorEnum(color); // internally this calls glColor3fv(...)
                    if (shape==TEAPOT_MESH_CAPSULE) {
                        // For capsules, Teapot_SetScaling(x,y,z) is interpreted this way:
                        // diameter = (x+z)/2; cylinderHeight = y. So the total height is: (cylinderHeight + diameter)
                        // This was made to force uniform scaling of the two half spheres.
                        const float r = (aabb_hextents[0]+aabb_hextents[2])*0.5f;   // radius of the capsule
                        const float hh = aabb_hextents[1]-r;                        // half height of the cylinder
                        Teapot_SetHalfScaling(r,hh,r);
                    }
                    else if ((shape>=TEAPOT_MESH_CUBE && shape<=TEAPOT_MESH_CUBE_ROUNDED)
                             || (shape>=TEAPOT_MESH_SPHERE1 && shape<=TEAPOT_MESH_SPHERE1)
                             || (shape>=TEAPOT_MESH_PLANE_X && shape<=TEAPOT_MESH_STAR_2D)
                             || (shape>=TEAPOT_MESH_TEXT_X)
                             ) {
                        // This branch is optional (performance optimization for meshes that are originally stored inside teapot.h with a cubic aabb)
                        // It can probably be extended to a few other meshes
                        Teapot_SetHalfScaling(aabb_hextents[0],aabb_hextents[1],aabb_hextents[2]);
                    }
                    else {
                        // This should work for everything (with a few exception (e.g. capsules)
                        float graphic_mesh_half_extents[3];Teapot_GetMeshAabbHalfExtents(shape,graphic_mesh_half_extents);
                        Teapot_SetScaling(aabb_hextents[0]/graphic_mesh_half_extents[0],
                                aabb_hextents[1]/graphic_mesh_half_extents[1],
                                aabb_hextents[2]/graphic_mesh_half_extents[2]);
                    }
                    Teapot_Draw(mMatrix,shape);
                }
            }
        }
        else {
            // Use use_graphic_transform=0 in debug mode, where you don't have a graphic mesh
            // or you want to see all the boxes/spheres that make up the body.
            // The transforms here are not smoothed.
            // Here we must use the physic layout of the body (the one used by nudge.h)
            const Transform* T1 = &c->bodies.transforms[body];
            mMatrix=mMatrixTmp;

            if (!is_shadow_pass) {
                if (is_dynamic) {
                    if (c->active_bodies.count==0) color = no_active_body_color;
                    else if (is_sleeping_and_dynamic) color = sleeping_color;
                    else if (has_com_offset) color = com_offset_body_color;
                }
                Teapot_SetColorEnum(color);
            }

            const BodyLayout* layout = &c->bodies.layouts[body];	// BodyLayout hosts the indices of the body colliders (i.e. collision shapes). The global arrays of colliders are in c->colliders.boxes and c->colliders.spheres
            if (layout->num_boxes>0) {
                assert(layout->first_box_index>=0);
                assert((uint16_t)layout->first_box_index+layout->num_boxes<=c->colliders.boxes.count);
                for (uint16_t i=layout->first_box_index,isz=layout->first_box_index+layout->num_boxes;i<isz;i++)	{
                    const Transform* T2 = &c->colliders.boxes.transforms[i];
                    const Transform T = TransformMul(*T1,*T2);
                    const BoxCollider* coll = &c->colliders.boxes.data[i];
                    TransformToMat4(mMatrix,&T);
                    if (is_shadow_pass) {
                        Dynamic_Resolution_Shadow_Set_Scaling(coll->size[0]*2.f,coll->size[1]*2.f,coll->size[2]*2.f);
                        Dynamic_Resolution_Shadow_Set_MMatrix(mMatrix);
                        Teapot_LowLevel_DrawElements(TEAPOT_MESH_CUBE);
                    }
                    else {
                        Teapot_SetHalfScaling(coll->size[0],coll->size[1],coll->size[2]);
                        Teapot_Draw(mMatrix,TEAPOT_MESH_CUBE);
                    }
                }
            }
            if (layout->num_spheres>0) {
                assert(layout->first_sphere_index>=0);
                assert((uint16_t)layout->first_sphere_index+layout->num_spheres<=c->colliders.spheres.count);
                for (uint16_t i=layout->first_sphere_index,isz=layout->first_sphere_index+layout->num_spheres;i<isz;i++)	{
                    const Transform* T2 = &c->colliders.spheres.transforms[i];
                    const Transform T = TransformMul(*T1,*T2);
                    const SphereCollider* coll = &c->colliders.spheres.data[i];
                    TransformToMat4(mMatrix,&T);
                    if (is_shadow_pass) {
                        Dynamic_Resolution_Shadow_Set_Scaling(coll->radius*2.f,coll->radius*2.f,coll->radius*2.f);
                        Dynamic_Resolution_Shadow_Set_MMatrix(mMatrix);
                        Teapot_LowLevel_DrawElements(TEAPOT_MESH_SPHERE1);
                    }
                    else {
                        Teapot_SetHalfScaling(coll->radius,coll->radius,coll->radius);
                        Teapot_Draw(mMatrix,TEAPOT_MESH_SPHERE1);
                    }
                }
            }

        }
    }

    if (is_shadow_pass) return;


    // Dbg: draw an aabb on c->active_bodies (for better understanding what this array is)
    const int dbg_draw_aabb_around_bodies_in_the_active_array = 0;
    if (dbg_draw_aabb_around_bodies_in_the_active_array
            && !globals.use_graphic_transform
            )
    {
        glLineWidth(1.f);
        Teapot_SetColor(0.8f,0.8f,0.8f,1.f);
        for (unsigned i=0;i<c->active_bodies.count;i++) {
            const unsigned body = c->active_bodies.indices[i];
            const BodyFilter* filter = &c->bodies.filters[body];
            if (filter->flags&BF_IS_FRUSTUM_CULLED) continue;
            const BodyInfo* info = &c->bodies.infos[body];
            float mMatrix[16];TransformToMat4(mMatrix,&c->bodies.transforms[body]);
            if (filter->flags&BF_HAS_COM_OFFSET)   {for (int l=0;l<3;l++) mMatrix[12+l]-=mMatrix[l]*info->com_offset[0]+mMatrix[4+l]*info->com_offset[1]+mMatrix[8+l]*info->com_offset[2];}
            Teapot_SetHalfScaling(info->aabb_half_extents[0],info->aabb_half_extents[1],info->aabb_half_extents[2]);
            Teapot_Draw(mMatrix,TEAPOT_MESHLINES_CUBE_EDGES);
        }
        //glprintf("Active_bodies:%u\n",c->active_bodies.count);
    }
    // Dbg result:
    /* It seems that all the bodies of any active island are placed in the active_bodies arrays (regardless of their sleeping state)
     * So static and kinematic bodies are in the active_bodies array (='activate' from now on) when there is at least a non-sleeping dynamic body that touches them
     * Interesting to note:
     * 1) the moving kinematic teapot 'activates' only during collision time;
     * 2) going around with the character we can see how kinamatic (and static) grounds/platforms 'activate' as the character walks on it
     */
    // End Dbg

}

void glGetThrowBodyMatrixAtMouseCoordsfv(GLfloat* mMatrixOut,int mouseX,int mouseY); // forward declaration of internal function

void ThrowBodyAtMousePos(int x, int y) {
    using namespace nudge;
    if (!can_add_box(c) && !can_add_sphere(c)) return;

    float mMatrixThrow[16];glGetThrowBodyMatrixAtMouseCoordsfv(mMatrixThrow,x,y);

    bool throwBox = ((float)rand() * (1.0f/(float)RAND_MAX)<0.5f);
    if (throwBox && !can_add_box(c)) {throwBox=false;assert(can_add_sphere(c));}
    else if (!throwBox && !can_add_sphere(c)) {throwBox=true;assert(can_add_box(c));}

    const float velocity = 15.f*((float)rand() * (1.0f/(float)RAND_MAX)+0.5f);
    const float spin = 10.f*((float)rand() * (1.0f/(float)RAND_MAX)+0.5f);
    const float friction = ((float)rand() * (2.0f/(float)RAND_MAX));    // however it works only for box-box collisions (possible reasons: [1] rolling friction not supported plus [2] micro jumps that make the friction of the single contact point very inefficient [this will never be fixed])
    const float comOffset[3] = {0,0,0};
    unsigned body = NUDGE_INVALID_BODY_ID;

    if (throwBox)   {
        float size[3];
        for (int l=0;l<3;l++)   {
            size[l] = 0.5f*(float)rand() * (1.0f/(float)RAND_MAX) + 0.5f;
            mMatrixThrow[12+l]+=mMatrixThrow[8+l]*(size[l]*1.5f+globals.proj.pMatrixNearPlane);   // advance position a bit to avoid clipping box graphic at lauch
        }
        const float mass = size[0]*size[1]*size[2];
        body = add_box(c,mass,size[0]*0.5f,size[1]*0.5f,size[2]*0.5f,mMatrixThrow,comOffset);
        bind_body(c,body,TEAPOT_MESH_CUBE,COLOR_NONE);
    }
    else {
        const float radius = 0.25f*((float)rand() * (1.0f/(float)RAND_MAX) + 0.5f);
        for (int l=0;l<3;l++)   {
            mMatrixThrow[12+l]+=mMatrixThrow[8+l]*((radius*2.f)*1.5f+globals.proj.pMatrixNearPlane);   // advance position a bit to avoid clipping sphere graphic at lauch
        }
        const float mass = 8.18879f*radius*radius*radius;
        body = add_sphere(c,mass,radius,mMatrixThrow,comOffset);
        bind_body(c,body,TEAPOT_MESH_SPHERE1,COLOR_NONE);
    }
    if (body<c->bodies.count)   {
        float* linVel = c->bodies.momentum[body].velocity;
        float* angVel = c->bodies.momentum[body].angular_velocity;
        for (int l=0;l<3;l++)   {
            linVel[l] = mMatrixThrow[8+l] * velocity;
            angVel[l] = mMatrixThrow[0+l] * spin;
        }
        c->bodies.properties[body].friction = friction;
    }
}
void DestroyGL(void);   // forward declaration
uint16_t glGetNudgeBodyAtMouseCoords(nudge::context_t* c,int mouseX,int mouseY,int16_t* pRelativeBoxColliderIndexOut=NULL,int16_t* pRelativeSphereColliderIndexOut=NULL,float* pOptionalDistanceOut=NULL); // forward declaration
void HandleKeys(void) {
    const globals_t::key_t* key = &globals.key;

    // keys to modify globals.gui.menu_idx: it must be done once per frame (devs: if you move HandleKeys(...) around, extract this block and call it in the original spot before the move)
    if ((key->pressed&(KEY_H|KEY_M|KEY_ESC)) && !(key->down&KEY_ALL_MODIFIER_KEYS))   {
        // Placing this code in the wrong place means breaking all the glprintf(...), glguihelp(...), glguimenu(...) calls
        if ((key->pressed&KEY_H)) {
            nudge::log("[H] %s\n",Dynamic_Resolution_GetInfoString());
            if (globals.use_frustum_culling) nudge::log("[H] num_frustum_culled_bodies: %u/%u\n",globals.num_frustum_culled_bodies,c->bodies.count-c->global_data.removed_bodies_count);
            nudge::log("[H] num_stars: %u/12\n",globals.bodies.num_stars);
            nudge::flush();
        }
        else if ((key->pressed&KEY_M)) {
            globals.use_shadows=!globals.use_shadows;
            nudge::log("[M] shadow mapping: %s\n",globals.use_shadows?"enabled":"disabled");nudge::flush();
        }
        else if (key->pressed&(KEY_ESC)) {
            DestroyPhysics();glutSwapBuffers();DestroyGL();exit(0);    // exit program (original glut needs exit(...))
        }
    }

    // physic-related keys
    if ((key->down&KEY_ALL_MODIFIER_KEYS)==0) {
        if (key->pressed&KEY_R) {
            // restart physics
            InitPhysics();  // this internally calls 'restart_context(c)' now (see the code)
                            // otherwise we should call 'destroy_context(c)' + 'init_context_with(c,-,-)' (losing our tweaked simulation params)
            //--------------------------------------
            c->simulation_params.num_frames=c->simulation_params.num_total_substeps=0;  // we want to restart the physic frame counters too
            //-----------------------------------------------
            // Note that here we don't reset all the fields in the 'globals' struct (e.g. camera, etc.)
            // we'll do it better when we load/save the simulation (F5/F7)
            nudge::log("[R] restart simulation\n");nudge::flush();
            return;
        }
        if (key->pressed&KEY_SPACE) {
            globals.use_graphic_transform = !globals.use_graphic_transform; /* switch draw mode */
            nudge::log("[SPACE] view mode: %s\n",globals.use_graphic_transform?"normal":"collision shapes");nudge::flush();
        }
        if (key->pressed&KEY_F) {
            globals.use_frustum_culling = !globals.use_frustum_culling; /* toggle frustum culling */
            nudge::log("[F] frustum culling: %s\n",globals.use_frustum_culling?"enabled":"disabled");nudge::flush();
        }
        if (key->pressed&(KEY_F5|KEY_F7)) {
            // quick save/load simulation
            const char* filename = "example03.sav"; // ascii only please
            const bool save = (key->pressed&KEY_F5);            
            FILE* f=fopen(filename,save?"wb":"rb");
            if (f)  {
                if (save) {
                    nudge::save_context(f,c);
                    // we append some fields in the 'globals' struct
                    fwrite(&globals.bodies,sizeof(globals_t::bodies_t),1,f);
                    fprintf(f,"\nuse_graphic_transform: %u",globals.use_graphic_transform);
                    fprintf(f,"\nuse_character_camera: %u",globals.use_character_camera);
                    fprintf(f,"\nuse_frustum_culling: %u",globals.use_frustum_culling);
                    fprintf(f,"\nuse_shadows: %u",globals.use_shadows);
                    fwrite(&globals.camera[0],sizeof(globals.camera[0]),2,f);
                    nudge::log("[F5] Saved \"%s\"\n",filename);
                }
                else {
                    nudge::load_context(f,c);
                    // we append some fields in the 'globals' struct
                    int cnt=fread(&globals.bodies,sizeof(globals_t::bodies_t),1,f);assert(cnt==1);
                    cnt=fscanf(f,"\nuse_graphic_transform: %u",&globals.use_graphic_transform);assert(cnt==1);
                    cnt=fscanf(f,"\nuse_character_camera: %u",&globals.use_character_camera);assert(cnt==1);
                    cnt=fscanf(f,"\nuse_frustum_culling: %u",&globals.use_frustum_culling);assert(cnt==1);
                    cnt=fscanf(f,"\nuse_shadows: %u",&globals.use_shadows);assert(cnt==1);
                    cnt = fread(&globals.camera[0],sizeof(globals.camera[0]),2,f);assert(cnt==2);
                    nudge::log("[F7] Loaded \"%s\"\n",filename);
                }
                fclose(f);                
            }
            else {
                if (save) nudge::log("[F5] Error: can't create file \"%s\"\n",filename);
                else nudge::log("[F7] Error: can't find file \"%s\"\n",filename);
            }
            nudge::flush(); // flushes nudge::log(...)
        }
        if (key->pressed&(KEY_MOUSE_BUTTON_RIGHT|KEY_MOUSE_BUTTON_MIDDLE|KEY_ENTER|KEY_END)) {
            globals.use_character_camera=!globals.use_character_camera;
            nudge::log("[RMB/MMB/ENTER/END] camera mode: %s\n",globals.use_character_camera?"character":"free");nudge::flush();
        }
        if (key->pressed&KEY_MOUSE_BUTTON_LEFT && !globals.use_character_camera) ThrowBodyAtMousePos(key->mouseX,key->mouseY);  /* add a body and throw it */
    }
    //else if (key->down&KEY_SHIFT) {if (key->pressed&KEY_SPACE) {const float* p=c->bodies.transforms[globals.bodies.character_body].p;nudge::log("%1.2f,%1.2f,%1.2f,\n",p[0],p[1]+0.65f,p[2]);nudge::flush();}}

    // camera movement keys
    {
        globals_t::camera_t* cam = &globals.camera[globals.use_character_camera];
        const float instantFrameTime = globals.instantFrameTime;const float speed = 0.4f;
        if (!(key->down&KEY_CTRL))	{
            if (key->down&(KEY_LEFT|KEY_RIGHT)) {
                cam->cameraYaw+= instantFrameTime*((key->down&KEY_LEFT) ? -4.0f : 4.0f)*speed;
            }
            if (key->down&(KEY_UP|KEY_DOWN)) {
                cam->cameraPitch+= instantFrameTime*((key->down&KEY_UP) ? 2.f : -2.f)*speed;
            }
            if (key->down&(KEY_PAGE_UP_OR_DOWN|KEY_MOUSE_WHEEL_UP_OR_DOWN)) {
                const float amount = (key->down&KEY_PAGE_UP_OR_DOWN)?(50.0f*speed):(60.f*speed/0.4f);
                cam->cameraDistance+= instantFrameTime*((key->down&(KEY_PAGE_DOWN|KEY_MOUSE_WHEEL_DOWN)) ? amount : -amount);
            }
            if (key->pressed&KEY_HOME) {
                // reset camera
                cam->targetPos[0]=0;cam->targetPos[1]=2;cam->targetPos[2]=0;
                cam->cameraYaw=globals.use_character_camera?M_PI:2*M_PI;cam->cameraPitch=M_PI*0.125f;
                cam->cameraDistance=globals.use_character_camera?2:20;
                cam->cameraPos[0]=cam->cameraPos[1]=cam->cameraPos[2]=0;
            }
        }
        else if (key->down&KEY_CTRL) {
            if (key->down&KEY_ALL_ARROW_KEYS && !globals.use_character_camera)   {
                // Here we move targetPos and cameraPos at the same time

                // We must find a pivot relative to the camera here (ignoring Y)
                float forward[3] = {cam->targetPos[0]-cam->cameraPos[0],0,cam->targetPos[2]-cam->cameraPos[2]};
                float up[3] = {0,1,0};
                float left[3];

                nudge::nm_Vec3Normalize(forward);
                nudge::nm_Vec3Cross(left,up,forward);

                float delta[3] = {0,0,0};
                if (key->down&(KEY_LEFT|KEY_RIGHT)) {
                    float amount = instantFrameTime*((key->down&KEY_RIGHT) ? -25.0f : 25.0f)*speed;
                    for (int i=0;i<3;i++) delta[i]+=amount*left[i];
                }
                else {
                    float amount = instantFrameTime*((key->down&KEY_DOWN) ? -25.0f : 25.0f)*speed;
                    for (int i=0;i<3;i++) delta[i]+=amount*forward[i];
                }
                for (int i=0;i<3;i++) {
                    cam->targetPos[i]+=delta[i];
                    cam->cameraPos[i]+=delta[i];
                }
            }
            if (key->down&(KEY_PAGE_DOWN|KEY_PAGE_UP))   {
                // We use world space coords here.
                cam->targetPos[1]+= instantFrameTime*((key->down&KEY_PAGE_DOWN) ? -25.0f : 25.0f)*speed;
                if (cam->targetPos[1]<-50.f) cam->targetPos[1]=-50.f;
                else if (cam->targetPos[1]>500.f) cam->targetPos[1]=500.f;
            }
        }
    }

}


void InitGL() {
    // graphics
    Dynamic_Resolution_Init(30,1,1);
    Teapot_Init();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    //const GLfloat material_specular[4] = {0.5f,0.5f,0.5f,1.0f};
    //glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,material_specular);
    //glMateriali(GL_FRONT_AND_BACK,GL_SHININESS,24); // in [0-128]


    glClearColor(0.5f, 0.7f, 1.0f, 1.0f);
    /*GLfloat light_ambient[] = { 0.1f, 0.1f, 0.1f, 1.0f };
    GLfloat light_diffuse[] = { 0.75f, 0.75f, 0.75f, 1.0f };
    GLfloat light_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };*/


#   ifdef TEAPOT_SHADER_FOG
    // We use fog to prevent ad clipping artifacts, so it just needs the near and far plane
    Teapot_SetFogDistances((globals.proj.pMatrixFarPlane-globals.proj.pMatrixNearPlane)*0.5f,globals.proj.pMatrixFarPlane); // We start the fog at the half point... but it works better nearer when farPlane is far away
    Teapot_SetFogColor(0.5f, 0.7f, 1.0f); // it should be the same as glClearColor()
#   endif

#   ifdef TEAPOT_SHADER_USE_SHADOW_MAP
    Teapot_SetShadowDarkening(90.f,0.875f);  // First number makes the shadow darker in an uniform way, the second clamps the lower intensity: (40.f,0.75f) default, (0.f,...) -> no shadows
#   endif //TEAPOT_SHADER_USE_SHADOW_MAP

    // This line can change the look of the demo considerably. Try commenting it out!
    Teapot_Enable_ColorMaterial();        
}
void DestroyGL() {
    Teapot_Destroy();
    Dynamic_Resolution_Destroy();
}
void ResizeGL(int w,int h) {
	// projection Matrix
	if (h>0) {
		globals_t::proj_t* p = &globals.proj;        
        glViewport(0,0,w,h);
        glPerspective(p->pMatrixFovDeg,(float)w/(float)h,p->pMatrixNearPlane,p->pMatrixFarPlane);
        Dynamic_Resolution_Resize(w,h);    // The dynamic resolution texture (and the shadow map) changes their size with this call
    }
}

void updateCameraPos(void);void bindShadowPass(void);void unbindShadowPass(void); // froward declarations of internal functions
void DrawGL() {
    // time calculations
	static double time_begin = double(glutGet(GLUT_ELAPSED_TIME))*0.001;
	const double time_now = double(glutGet(GLUT_ELAPSED_TIME))*0.001;
    const double elapsed_time = time_now>time_begin ? (time_now-time_begin) : 1.0/60.0; globals.instantFrameTime = elapsed_time;
	time_begin = time_now;
	
    // key state updating
    globals.key.pressed = (~globals.key.down_last_frame)&globals.key.down;
    globals.key.released = globals.key.down_last_frame&(~globals.key.down);

    // main key handling
    HandleKeys();

    // simulation
    UpdatePhysics(elapsed_time);

	// drawing

	// view Matrix
    updateCameraPos();const globals_t::camera_t* cam = &globals.camera[globals.use_character_camera];
    glLookAt(cam->cameraPos[0],cam->cameraPos[1],cam->cameraPos[2],cam->targetPos[0],cam->targetPos[1],cam->targetPos[2],0,1,0);   

#   ifdef TEAPOT_SHADER_USE_SHADOW_MAP
    const int num_rendering_passes = globals.use_shadows?2:1;
#   else
    const int num_rendering_passes = 1;
#   endif

    for (int rendering_pass=0;rendering_pass<num_rendering_passes;rendering_pass++) {
        globals.is_shadow_pass = (rendering_pass<num_rendering_passes-1);
        if (globals.is_shadow_pass) bindShadowPass();
        else {
            unbindShadowPass();

            // bind rendering pass
            Dynamic_Resolution_Bind();  // This defaults to nothing if we don't use dynamic resolution (-> it's for free: we can draw inside it as usual)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            Teapot_PreDraw();
        }

        // Draws all the physic bodies
        DrawPhysics(rendering_pass,num_rendering_passes);
    }

    // We can add a pivot at the camera target point
    if (!globals.use_character_camera)   {
        static float mMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
        mMatrix[12]=cam->targetPos[0];mMatrix[13]=cam->targetPos[1];mMatrix[14]=cam->targetPos[2];
        // we complicate things by adding half of the code to just fade the pivot in the distance... good choice doing it in a nudge example!
        if (cam->cameraDistance<40.f)   {
            float fading_alpha = cam->cameraDistance>5.f?sqrtf(sqrtf(cam->cameraDistance-4.f)):1.f;fading_alpha=1.f/(fading_alpha);
            if (fading_alpha>0.01f) {
                Teapot_LowLevel_StartDisablingLighting();
                Teapot_SetScaling(1.5f,1.5f,1.5f);
                glEnable(GL_BLEND);
                Teapot_SetColor(1,1,1,fading_alpha);
                //Teapot_SetColorAmbient(1,1,1);
                Teapot_Draw(mMatrix,TEAPOT_MESH_PIVOT3D);
                glDisable(GL_BLEND);
                Teapot_LowLevel_StopDisablingLighting();
            }
        }
    }

    // unbind rendering pass
    Teapot_PostDraw();
    Dynamic_Resolution_Unbind();

    // Draw to screen at current resolution_factor: ----------------------------------------------------------------
    glDisable(GL_DEPTH_TEST);glDisable(GL_CULL_FACE);glDepthMask(GL_FALSE);
    Dynamic_Resolution_Render(globals.instantFrameTime);    // Mandatory
    glEnable(GL_DEPTH_TEST);glEnable(GL_CULL_FACE);glDepthMask(GL_TRUE);
    //--------------------------------------------------------------------------------------------------------------

    // key state resetting
    globals.key.down&=~KEY_MOUSE_WHEEL_UP_OR_DOWN;    // we must reset the two mouse wheel flags, because otherwise it does not work
    globals.key.down_last_frame=globals.key.down;
}

float nm_QuatGetYaw(const float* q); // forward declaration of internal function
void updateCameraPos() {
    // this function is called once per frame in DrawGL()
    if (globals.use_character_camera && globals.bodies.character_body>=c->bodies.count) globals.use_character_camera=0; // no character
    globals_t::camera_t* cam = &globals.camera[globals.use_character_camera];

    if (cam->cameraYaw>M_PI) cam->cameraYaw-=2*M_PI;
    else if (cam->cameraYaw<=-M_PI) cam->cameraYaw+=2*M_PI;
    const float pitchLimits[2]={globals.use_character_camera?0.2f:(float)(-M_PI*0.05f),globals.use_character_camera?(float)(M_PI*0.25f):(float)(M_PI*0.5f-0.001f)};
    if (cam->cameraPitch>pitchLimits[1]) cam->cameraPitch=pitchLimits[1];
    else if (cam->cameraPitch<pitchLimits[0]) cam->cameraPitch=pitchLimits[0];
    if (cam->cameraDistance<1.f) cam->cameraDistance=1.f;
    if (globals.use_character_camera && cam->cameraDistance>10.f) cam->cameraDistance=10.f;

    float yaw = cam->cameraYaw;
    if (globals.use_character_camera)   {
        using namespace nudge;
        assert(globals.bodies.character_body<c->bodies.count);
        const Transform* T = &c->bodies.transforms[globals.bodies.character_body];assert(T->body==globals.bodies.character_body);
        yaw+=nm_QuatGetYaw(T->q);

        const BodyLayout* L = &c->bodies.layouts[T->body];
        assert(L->num_boxes==1);
        const float deltaY = c->colliders.boxes.data[L->first_box_index].size[1]-c->bodies.infos[T->body].com_offset[1];
        cam->targetPos[0]=T->p[0];
        cam->targetPos[1]=T->p[1]+deltaY;
        cam->targetPos[2]=T->p[2];
    }

	const float distanceY = sin(cam->cameraPitch)*cam->cameraDistance;
	cam->cameraPos[1] = cam->targetPos[1] + distanceY;
	const float distanceXZ = cos(cam->cameraPitch)*cam->cameraDistance;
    cam->cameraPos[0] = cam->targetPos[0] + sin(yaw)*distanceXZ;
    cam->cameraPos[2] = cam->targetPos[2] + cos(yaw)*distanceXZ;
    // end updateCameraPos()

    const bool smooth_camera = true;//globals.use_character_camera;    // optional: not a real slerp, just a lerp => it makes people sick...
    if (smooth_camera) {
        assert(globals.bodies.character_body<c->bodies.count);
        static float oldCameraPos[3]={},oldTargetPos[3]={};
        static unsigned last_camera_mode=100;
        const bool skip_lerp = (last_camera_mode!=globals.use_character_camera) || (globals.use_character_camera==0 && (globals.key.pressed&(KEY_HOME|KEY_F7)));
        last_camera_mode = globals.use_character_camera;
        const float lerp_constant = globals.use_character_camera ? 2.f : 30.f;  // bigger number = less lerp = mitigation of sickness
        float time = globals.instantFrameTime*lerp_constant;if (time>1) time=1;
        for (int i=0;i<3;i++) {
            if (!skip_lerp) {
                // smoothing:
                cam->cameraPos[i]=time*cam->cameraPos[i]+(1.f-time)*oldCameraPos[i];
                cam->targetPos[i]=time*cam->targetPos[i]+(1.f-time)*oldTargetPos[i];
            }
            oldCameraPos[i]=cam->cameraPos[i];oldTargetPos[i]=cam->targetPos[i];
        }
    }
}

// see KeyMask at the top
#ifdef AZERTY_KEYBOARD_LAYOUT
#   define WASD_KEYS 'z','q','s','d'
#else
#   define WASD_KEYS 'w','a','s','d'
#endif
static const unsigned char Keys[] = {WASD_KEYS,'j',' ','r',27,13,'h','m','f'};
static const int ModifierKeys[] = {GLUT_ACTIVE_SHIFT,GLUT_ACTIVE_CTRL,GLUT_ACTIVE_ALT};
static const int SpecialKeys[] = {GLUT_KEY_LEFT,GLUT_KEY_RIGHT,GLUT_KEY_UP,GLUT_KEY_DOWN,
                                     GLUT_KEY_PAGE_UP,GLUT_KEY_PAGE_DOWN,GLUT_KEY_HOME,GLUT_KEY_END,
                                     GLUT_KEY_F1,GLUT_KEY_F3,GLUT_KEY_F5,GLUT_KEY_F7
                                    };
static const int MouseKeys[] = {GLUT_LEFT_BUTTON,GLUT_RIGHT_BUTTON,GLUT_MIDDLE_BUTTON,0x0003,0x0004};

void GlutKeys(unsigned char key,int x,int y)	{
    globals_t::key_t* k = &globals.key;k->mouseX = x;k->mouseY = y;
    const int mod = glutGetModifiers();for (uint32_t i=0;i<KEY_NUM_MODIFIER_KEYS;i++) {if (mod&ModifierKeys[i]) k->down|=(1U<<(KEY_MODIFIER_KEY_START_INDEX+i)); else k->down&=~(1U<<(KEY_MODIFIER_KEY_START_INDEX+i));}
    for (uint32_t i=0;i<KEY_NUM_REGULAR_KEYS;i++) {if (key==Keys[i]) {k->down|=(1U<<(KEY_REGULAR_KEY_START_INDEX+i));}}
}
void GlutKeysUp(unsigned char key,int x,int y)	{
    globals_t::key_t* k = &globals.key;k->mouseX = x;k->mouseY = y;
    const int mod = glutGetModifiers();for (uint32_t i=0;i<KEY_NUM_MODIFIER_KEYS;i++) {if (mod&ModifierKeys[i]) k->down|=(1U<<(KEY_MODIFIER_KEY_START_INDEX+i)); else k->down&=~(1U<<(KEY_MODIFIER_KEY_START_INDEX+i));}
    for (uint32_t i=0;i<KEY_NUM_REGULAR_KEYS;i++) {if (key==Keys[i]) {k->down&=~(1U<<(KEY_REGULAR_KEY_START_INDEX+i));}}
}
void GlutSpecialKeys(int key,int x,int y)   {
    globals_t::key_t* k = &globals.key;k->mouseX = x;k->mouseY = y;
    const int mod = glutGetModifiers();for (uint32_t i=0;i<KEY_NUM_MODIFIER_KEYS;i++) {if (mod&ModifierKeys[i]) k->down|=(1U<<(KEY_MODIFIER_KEY_START_INDEX+i)); else k->down&=~(1U<<(KEY_MODIFIER_KEY_START_INDEX+i));}
    for (uint32_t i=0;i<KEY_NUM_SPECIAL_KEYS;i++) {if (key==SpecialKeys[i]) k->down|=(1U<<(KEY_SPECIAL_KEY_START_INDEX+i));}
}
void GlutSpecialKeysUp(int key,int x,int y)   {
    globals_t::key_t* k = &globals.key;k->mouseX = x;k->mouseY = y;
    const int mod = glutGetModifiers();for (uint32_t i=0;i<KEY_NUM_MODIFIER_KEYS;i++) {if (mod&ModifierKeys[i]) k->down|=(1U<<(KEY_MODIFIER_KEY_START_INDEX+i)); else k->down&=~(1U<<(KEY_MODIFIER_KEY_START_INDEX+i));}
    for (uint32_t i=0;i<KEY_NUM_SPECIAL_KEYS;i++) {if (key==SpecialKeys[i]) k->down&=~(1U<<(KEY_SPECIAL_KEY_START_INDEX+i));}
}
void GlutMouse(int button, int state, int x, int y) {
    globals_t::key_t* k = &globals.key;k->mouseX = x;k->mouseY = y;
    const int mod = glutGetModifiers();for (uint32_t i=0;i<KEY_NUM_MODIFIER_KEYS;i++) {if (mod&ModifierKeys[i]) k->down|=(1U<<(KEY_MODIFIER_KEY_START_INDEX+i)); else k->down&=~(1U<<(KEY_MODIFIER_KEY_START_INDEX+i));}
    const bool down = button<0x0003 ? (state==GLUT_DOWN) : (state==GLUT_UP);
    for (uint32_t i=0;i<KEY_NUM_MOUSE_KEYS;i++) {
        if (button==MouseKeys[i]) {
            const uint32_t mask = (1U<<(KEY_MOUSE_KEY_START_INDEX+i));
            if (down) k->down|=mask;
            else if (!(mask&KEY_MOUSE_WHEEL_UP_OR_DOWN)) k->down&=~mask; // glut AFAICS always sends a GLUT_DOWN + GLUT_UP mouse wheel event in the same frame: so they end up cancelling out if we don't handle it
        }
    }
}

static void GlutDrawGL(void)		{DrawGL();glutSwapBuffers();}
static void GlutIdle(void)			{glutPostRedisplay();}
static void GlutFakeDrawGL(void) 	{glutDisplayFunc(GlutDrawGL);}


int main(int argc, const char* argv[]) {
    NUDGE_STATIC_ASSERT(sizeof(Keys)/sizeof(Keys[0])==KEY_NUM_REGULAR_KEYS);
    NUDGE_STATIC_ASSERT(sizeof(SpecialKeys)/sizeof(SpecialKeys[0])==KEY_NUM_SPECIAL_KEYS);
    NUDGE_STATIC_ASSERT(sizeof(ModifierKeys)/sizeof(ModifierKeys[0])==KEY_NUM_MODIFIER_KEYS);
    NUDGE_STATIC_ASSERT(sizeof(MouseKeys)/sizeof(MouseKeys[0])==KEY_NUM_MOUSE_KEYS);

    NUDGE_STATIC_ASSERT(COLOR_COUNT<=255);
    NUDGE_STATIC_ASSERT(TEAPOT_MESH_COUNT<=255);

    nudge::show_info();
		
	// Start GLUT.
	glutInit(&argc, const_cast<char**>(argv));
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
#   ifndef __EMSCRIPTEN__
    glutInitWindowSize(1024, 600);
#   else
    glutInitWindowSize(960,540);
#   endif
    glutCreateWindow("nudge example03");
#	ifdef USE_GLEW
    GLenum err = glewInit();
    if( GLEW_OK != err ) {
       fprintf(stderr, "\nError initializing GLEW: %s\n", glewGetErrorString(err) );
       return 1;
    }
#	endif
    glutDisplayFunc(GlutFakeDrawGL);
    glutIdleFunc(GlutIdle);
    glutReshapeFunc(ResizeGL);
    glutMouseFunc(GlutMouse);
    glutKeyboardFunc(GlutKeys);
    glutKeyboardUpFunc(GlutKeysUp);   // GLUT Version>=4 or freeglut please
    glutSpecialFunc(GlutSpecialKeys);
    glutSpecialUpFunc(GlutSpecialKeysUp);   // GLUT Version>=4 or freeglut please
#   ifndef __EMSCRIPTEN__
    glutSetKeyRepeat(GLUT_KEY_REPEAT_OFF); // Nate Robbins' port of GLUT to win32 did not implement glutSetKeyRepeat
#   endif

    InitPhysics();
    InitGL();

    glutMainLoop();

    DestroyGL();
    DestroyPhysics();


	return 0;
}





// internal functions here
struct gl_matrices_t {
    float vMatrix[16],pMatrix[16],vpMatrix[16];  // these have been explicitely kept to calculate other quantities
    float vpMatrixInv[16];  // this is necessary for throwing
    float frustumPlanes[6][4];      // this is necessary for frustum culling
    // these are for the shadow pass:
    float lvMatrix[16],lpMatrix[16],lvpMatrix[16];
    float lvpMatrixFrustum[6][4];
} glmatrices;
void nm_Mat4Identity(float* m) {memset(m,0,16*sizeof(float));m[0]=m[5]=m[10]=m[15]=1.f;}
void glPerspective(float degfovy,float aspect, float zNear, float zFar) {
    // custom replacement of gluPerspective(...)
    Teapot_Helper_Perspective(glmatrices.pMatrix,degfovy,aspect,zNear,zFar);
    Teapot_SetProjectionMatrix(glmatrices.pMatrix);
}
void glLookAt(float eyeX, float eyeY, float eyeZ, float centerX, float centerY, float centerZ, float upX, float upY, float upZ)    {
    // custom replacement of gluLookAt(...)
    Teapot_Helper_LookAt(glmatrices.vMatrix,eyeX,eyeY,eyeZ,centerX,centerY,centerZ,upX,upY,upZ);
    nudge::nm_Vec3Normalize(globals.lightDirection);
    Teapot_SetViewMatrixAndLightDirection(glmatrices.vMatrix,globals.lightDirection);

    nudge::nm_Mat4Mul(glmatrices.vpMatrix,glmatrices.pMatrix,glmatrices.vMatrix);
    Teapot_Helper_InvertMatrix(glmatrices.vpMatrixInv,glmatrices.vpMatrix);
    Teapot_Helper_GetFrustumPlaneEquations(glmatrices.frustumPlanes,glmatrices.vpMatrix,0);

#   ifdef TEAPOT_SHADER_USE_SHADOW_MAP
    if (glmatrices.lpMatrix[0]==0)
    {
        // This changes with pMatrixFarPlane and pMatrixFovDeg
        const float y = globals.proj.pMatrixFarPlane*tanf(globals.proj.pMatrixFovDeg*3.1415f/180.0f)*0.35f;  // last coefficient is ad-hoc for this demo (in our case it should be 1.0, or maybe 0.5 for free roaming; something like 0.2 for fixed environment and MUCH better shadow quality!)
        const float x = y;
        Teapot_Helper_Ortho(glmatrices.lpMatrix,-x,x,-y,y,globals.proj.pMatrixFarPlane*0.5f,-globals.proj.pMatrixFarPlane*0.5f);
    }
    //if (glmatrices.lvMatrix[15]==0)
    {
        // This changes with lightDirection, pMatrixFarPlane, targetPos (= camera target)
        const globals_t::camera_t* cam = &globals.camera[globals.use_character_camera];
        const float distance =  globals.proj.pMatrixFarPlane*0.1f;
        const float shadowTargetPos[3] = {cam->targetPos[0],0.f,cam->targetPos[2]};   // We keep it at y=0
        const float lpos[3] = {shadowTargetPos[0]-globals.lightDirection[0]*distance,
                               shadowTargetPos[1]-globals.lightDirection[1]*distance,
                               shadowTargetPos[2]-globals.lightDirection[2]*distance};
        Teapot_Helper_LookAt(glmatrices.lvMatrix,lpos[0],lpos[1],lpos[2],shadowTargetPos[0],shadowTargetPos[1],shadowTargetPos[2],0,1,0);

        Teapot_Helper_MultMatrix(glmatrices.lvpMatrix,glmatrices.lpMatrix,glmatrices.lvMatrix);
        Teapot_Helper_GetFrustumPlaneEquations(glmatrices.lvpMatrixFrustum,glmatrices.lvpMatrix,0);
    }
#   endif
}
void glGetThrowBodyMatrixAtMouseCoordsfv(GLfloat* mMatrixOut,int mouseX,int mouseY) {
    GLint viewport[4];glGetIntegerv(GL_VIEWPORT,viewport);
    GLfloat *mat=mMatrixOut;const float yAxis[3]={0,1,0};
    Teapot_Helper_UnProjectMouseCoords(&mat[12],&mat[8],mouseX,mouseY,glmatrices.vpMatrixInv,viewport);
    nudge::nm_Vec3Cross(&mat[0],yAxis,&mat[8]);
    nudge::nm_Vec3Cross(&mat[4],&mat[8],&mat[0]);
    mat[3]=mat[7]=mat[11]=0.f;mat[15]=1.f;
}
int glIsAabbVisible(const float* __restrict mMatrix16,float aabbCenterX,float aabbCenterY,float aabbCenterZ,float aabbHalfExtentX,float aabbHalfExtentY,float aabbHalfExtentZ) {
    const float c[3] = {aabbCenterX,aabbCenterY,aabbCenterZ};
    const float he[3]= {aabbHalfExtentX,aabbHalfExtentY,aabbHalfExtentZ};
    return Teapot_Helper_IsVisible(globals.is_shadow_pass?glmatrices.lvpMatrixFrustum:glmatrices.frustumPlanes,mMatrix16,c,he);
}
int glIsAabbVisible(const float* __restrict Tpos3,float aabbEnlargedRadius) {
    // Note that we cannot simply add an aabbCenter to Tpos3 and use a simple aabbRadius (based on aabbHalfExtents):
    // But we can enlarge the aabbRadius so that: aabbEnlargedRadius = aabbRadius + distance(Tpos3,aabbCenter);
    const float aabb[6] = {Tpos3[0]-aabbEnlargedRadius,Tpos3[1]-aabbEnlargedRadius,Tpos3[2]-aabbEnlargedRadius,Tpos3[0]+aabbEnlargedRadius,Tpos3[1]+aabbEnlargedRadius,Tpos3[2]+aabbEnlargedRadius};
    return Teapot_Helper_LowLevel_IsAABBVisible(globals.is_shadow_pass?glmatrices.lvpMatrixFrustum:glmatrices.frustumPlanes,aabb);
}
void bindShadowPass(void)   {
#   ifdef TEAPOT_SHADER_USE_SHADOW_MAP
    if (globals.use_shadows) {
        Dynamic_Resolution_Bind_Shadow();   // Binds the shadow map FBO and its shader program
        glClear(GL_DEPTH_BUFFER_BIT);
        Dynamic_Resolution_Shadow_Set_VpMatrix(glmatrices.lvpMatrix);
        Teapot_LowLevel_BindVertexBufferObjectAndEnableVertexAttributes(1,1);
    }
#   endif //TEAPOT_SHADER_USE_SHADOW_MAP
}

void unbindShadowPass(void) {
#   ifdef TEAPOT_SHADER_USE_SHADOW_MAP
    if (globals.use_shadows) {
        // unbind shadow pass
        Teapot_LowLevel_UnbindVertexBufferObjectAndDisableVertexAttributes(1,1);
        Dynamic_Resolution_Unbind_Shadow();   // Unbinds the shadow map FBO and its shader program

        Teapot_SetShadowVpMatrix(glmatrices.lvpMatrix);    // Needed for the second shadowing pass (enabled with the definition: TEAPOT_SHADER_USE_SHADOW_MAP). Note that here we can't pass (lvpMatrix * cameraViewMatrixInverse), but just lvpMatrix (because the multiplication happens internally).
        Teapot_SetShadowMapFactor(Dynamic_Resolution_GetShadowMapDynResFactor());   // The shadow map has dynamic resolution too. That means that in the shader used in "teapot.h" there's an additional float uniform that must be updated from "dynamic_resolution.h"
        Teapot_SetShadowMapTexelIncrement(Dynamic_Resolution_GetShadowMapTexelIncrement(),Dynamic_Resolution_GetShadowMapTexelIncrement());
        glBindTexture(GL_TEXTURE_2D,Dynamic_Resolution_Get_Shadow_Texture_ID());
    }
    else glBindTexture(GL_TEXTURE_2D,0);
#   endif //TEAPOT_SHADER_USE_SHADOW_MAP
}

void nm_QuatToEulerYPR(const float* q,float* a0,float* a1, float* a2,int i=1,int j=0, int k=2) {
    // based on: Quaternion to Euler angles conversion:
    // A direct, general and computationally efficient method
    // Bernardes E, Viollet S. 2022
    int not_proper=1;const float eps = 0.0001f;
    if (i==k) {not_proper=0;k=3-i-j;/*because i+j+k=0+1+2=3*/}
    const float sign = (float) ((i-j)*(j-k)*(k-i)/2); // equivalent to the Levi-Civita symbol
    float a,b,c,d;  // a is the w part here
    if (not_proper) {a=q[3]-q[j];b=q[i]+q[k]*sign;c=q[j]+q[3];d=q[k]*sign-q[i];}
    else {a=q[3];b=q[i];c=q[j];d=q[k]*sign;}
    *a1 = 2*atan2f(sqrtf(c*c+d*d),sqrtf(a*a+b*b));
    const float tp=atan2f(b,a),tm=atan2f(d,c);
    if (fabsf(*a1)<eps) {*a0=0;*a2=2*tp-*a0;} // For simplicity, we are setting *a0=0
    else if (fabs(*a1-M_PI)<eps) {*a0=0;*a2=2*tm-*a0;}
    else {*a0=tp-tm;*a2=tp+tm;}
    if (not_proper) {*a2=sign*tm;*a1-=M_PI*0.5;}
}
float nm_QuatGetYaw(const float* q) {float y,p,r;nm_QuatToEulerYPR(q,&y,&p,&r);return y;}



void Teapot_SetColorEnum(ColorEnum color) {
#   ifdef COLOR_ENUM_DECODE_AS_BGRA
#       define CEU2F(RGBA) {(float)(((RGBA)&0x0000FF00)>>8)/255.f,(float)(((RGBA)&0x00FF0000)>>16)/255.f,(float)(((RGBA)&0xFF000000)>>24)/255.f/*,(float)(((RGBA)&0x000000FF)>>0)/255.f*/}
#   else
#       define CEU2F(RGBA) {(float)(((RGBA)&0xFF000000)>>24)/255.f,(float)(((RGBA)&0x00FF0000)>>16)/255.f,(float)(((RGBA)&0x0000FF00)>>8)/255.f/*,(float)(((RGBA)&0x000000FF)>>0)/255.f*/}
#   endif
    static const float colors[COLOR_COUNT+1][3] = {
        /*NONE*/CEU2F(0xFFFFFFFF),/*ALICEBLUE*/CEU2F(0xF0F8FFFF),/*ANTIQUEWHITE*/CEU2F(0xFAEBD7FF),/*AQUA*/CEU2F(0x00FFFFFF),/*AQUAMARINE*/CEU2F(0x7FFFD4FF),/*AZURE*/CEU2F(0xF0FFFFFF),/*BEIGE*/CEU2F(0xF5F5DCFF),/*BISQUE*/CEU2F(0xFFE4C4FF),/*BLACK*/CEU2F(0x000000FF),/*BLANCHEDALMOND*/CEU2F(0xFFEBCDFF),/*BLUE*/CEU2F(0x0000FFFF),/*BLUEVIOLET*/CEU2F(0x8A2BE2FF),/*BROWN*/CEU2F(0xA52A2AFF),/*BURLYWOOD*/CEU2F(0xDEB887FF),/*CADETBLUE*/CEU2F(0x5F9EA0FF),/*CHARTREUSE*/CEU2F(0x7FFF00FF),/*CHOCOLATE*/CEU2F(0xD2691EFF),/*CORAL*/CEU2F(0xFF7F50FF),/*CORNFLOWERBLUE*/CEU2F(0x6495EDFF),/*CORNSILK*/CEU2F(0xFFF8DCFF)
        ,/*CRIMSON*/CEU2F(0xDC143CFF),/*CYAN*/CEU2F(0x00FFFFFF),/*DARKBLUE*/CEU2F(0x00008BFF),/*DARKCYAN*/CEU2F(0x008B8BFF),/*DARKGOLDENROD*/CEU2F(0xB8860BFF),/*DARKGRAY*/CEU2F(0xA9A9A9FF),/*DARKGREEN*/CEU2F(0x006400FF),/*DARKKHAKI*/CEU2F(0xBDB76BFF),/*DARKMAGENTA*/CEU2F(0x8B008BFF),/*DARKOLIVEGREEN*/CEU2F(0x556B2FFF),/*DARKORANGE*/CEU2F(0xFF8C00FF),/*DARKORCHID*/CEU2F(0x9932CCFF),/*DARKRED*/CEU2F(0x8B0000FF),/*DARKSALMON*/CEU2F(0xE9967AFF),/*DARKSEAGREEN*/CEU2F(0x8FBC8FFF),/*DARKSLATEBLUE*/CEU2F(0x483D8BFF),/*DARKSLATEGRAY*/CEU2F(0x2F4F4FFF),/*DARKTURQUOISE*/CEU2F(0x00CED1FF),/*DARKVIOLET*/CEU2F(0x9400D3FF),/*DEEPPINK*/CEU2F(0xFF1493FF)
        ,/*DEEPSKYBLUE*/CEU2F(0x00BFFFFF),/*DIMGRAY*/CEU2F(0x696969FF),/*DODGERBLUE*/CEU2F(0x1E90FFFF),/*FIREBRICK*/CEU2F(0xB22222FF),/*FLORALWHITE*/CEU2F(0xFFFAF0FF),/*FORESTGREEN*/CEU2F(0x228B22FF),/*FUCHSIA*/CEU2F(0xFF00FFFF),/*GAINSBORO*/CEU2F(0xDCDCDCFF),/*GHOSTWHITE*/CEU2F(0xF8F8FFFF),/*GOLD*/CEU2F(0xFFD700FF),/*GOLDENROD*/CEU2F(0xDAA520FF),/*GRAY*/CEU2F(0x808080FF),/*GREEN*/CEU2F(0x008000FF),/*GREENYELLOW*/CEU2F(0xADFF2FFF),/*HONEYDEW*/CEU2F(0xF0FFF0FF),/*HOTPINK*/CEU2F(0xFF69B4FF),/*INDIANRED*/CEU2F(0xCD5C5CFF),/*INDIGO*/CEU2F(0x4B0082FF),/*IVORY*/CEU2F(0xFFFFF0FF),/*KHAKI*/CEU2F(0xF0E68CFF)
        ,/*LAVENDER*/CEU2F(0xE6E6FAFF),/*LAVENDERBLUSH*/CEU2F(0xFFF0F5FF),/*LAWNGREEN*/CEU2F(0x7CFC00FF),/*LEMONCHIFFON*/CEU2F(0xFFFACDFF),/*LIGHTBLUE*/CEU2F(0xADD8E6FF),/*LIGHTCORAL*/CEU2F(0xF08080FF),/*LIGHTCYAN*/CEU2F(0xE0FFFFFF),/*LIGHTGOLDENRODYELLOW*/CEU2F(0xFAFAD2FF)
        ,/*LIGHTGRAY*/CEU2F(0xD3D3D3FF),/*LIGHTGREEN*/CEU2F(0x90EE90FF),/*LIGHTPINK*/CEU2F(0xFFB6C1FF),/*LIGHTSALMON*/CEU2F(0xFFA07AFF),/*LIGHTSEAGREEN*/CEU2F(0x20B2AAFF),/*LIGHTSKYBLUE*/CEU2F(0x87CEFAFF),/*LIGHTSLATEGRAY*/CEU2F(0x778899FF),/*LIGHTSTEELBLUE*/CEU2F(0xB0C4DEFF)
        ,/*LIGHTYELLOW*/CEU2F(0xFFFFE0FF),/*LIME*/CEU2F(0x00FF00FF),/*LIMEGREEN*/CEU2F(0x32CD32FF),/*LINEN*/CEU2F(0xFAF0E6FF),/*MAGENTA*/CEU2F(0xFF00FFFF),/*MAROON*/CEU2F(0x800000FF)
        ,/*MEDIUMAQUAMARINE*/CEU2F(0x66CDAAFF),/*MEDIUMBLUE*/CEU2F(0x0000CDFF),/*MEDIUMORCHID*/CEU2F(0xBA55D3FF),/*MEDIUMPURPLE*/CEU2F(0x9370DBFF),/*MEDIUMSEAGREEN*/CEU2F(0x3CB371FF),/*MEDIUMSLATEBLUE*/CEU2F(0x7B68EEFF),/*MEDIUMSPRINGGREEN*/CEU2F(0x00FA9AFF),/*MEDIUMTURQUOISE*/CEU2F(0x48D1CCFF),/*MEDIUMVIOLETRED*/CEU2F(0xC71585FF),/*MIDNIGHTBLUE*/CEU2F(0x191970FF),/*MINTCREAM*/CEU2F(0xF5FFFAFF),/*MISTYROSE*/CEU2F(0xFFE4E1FF),/*MOCCASIN*/CEU2F(0xFFE4B5FF),/*NAVAJOWHITE*/CEU2F(0xFFDEADFF),/*NAVY*/CEU2F(0x000080FF),/*OLDLACE*/CEU2F(0xFFF5E6FF),/*OLIVE*/CEU2F(0x808000FF),/*OLIVEDRAB*/CEU2F(0x6B8E23FF),/*ORANGE*/CEU2F(0xFFA500FF),/*ORANGERED*/CEU2F(0xFF4500FF)
        ,/*ORCHID*/CEU2F(0xDA70D6FF),/*PALEGOLDENROD*/CEU2F(0xEEE8AAFF),/*PALEGREEN*/CEU2F(0x98FB98FF),/*PALETURQUOISE*/CEU2F(0xAFEEEEFF),/*PALEVIOLETRED*/CEU2F(0xDB7093FF),/*PAPAYAWHIP*/CEU2F(0xFFEFD5FF),/*PEACHPUFF*/CEU2F(0xFFDAB9FF),/*PERU*/CEU2F(0xCD853FFF),/*PINK*/CEU2F(0xFFC0CBFF),/*PLUM*/CEU2F(0xDDA0DDFF),/*POWDERBLUE*/CEU2F(0xB0E0E6FF),/*PURPLE*/CEU2F(0x800080FF),/*RED*/CEU2F(0xFF0000FF),/*ROSYBROWN*/CEU2F(0xBC8F8FFF),/*ROYALBLUE*/CEU2F(0x4169E1FF),/*SADDLEBROWN*/CEU2F(0x8B4513FF),/*SALMON*/CEU2F(0xFA8072FF),/*SANDYBROWN*/CEU2F(0xF4A460FF),/*SEAGREEN*/CEU2F(0x2E8B57FF),/*SEASHELL*/CEU2F(0xFFF5EEFF)
        ,/*SIENNA*/CEU2F(0xA0522DFF),/*SILVER*/CEU2F(0xC0C0C0FF),/*SKYBLUE*/CEU2F(0x87CEEBFF),/*SLATEBLUE*/CEU2F(0x6A5ACDFF),/*SLATEGRAY*/CEU2F(0x708090FF),/*SNOW*/CEU2F(0xFFFAFAFF),/*SPRINGGREEN*/CEU2F(0x00FF7FFF),/*STEELBLUE*/CEU2F(0x4682B4FF),/*TAN*/CEU2F(0xD2B48CFF),/*TEAL*/CEU2F(0x008080FF),/*THISTLE*/CEU2F(0xD8BFD8FF),/*TOMATO*/CEU2F(0xFF6347FF),/*TURQUOISE*/CEU2F(0x40E0D0FF),/*VIOLET*/CEU2F(0xEE82EEFF),/*WHEAT*/CEU2F(0xF5DEB3FF),/*WHITE*/CEU2F(0xFFFFFFFF),/*WHITESMOKE*/CEU2F(0xF5F5F5FF),/*YELLOW*/CEU2F(0xFFFF00FF),/*YELLOWGREEN*/CEU2F(0x9ACD32FF),/*COUNT*/CEU2F(0xFFFFFFFF)
    };
    Teapot_SetColor(colors[color][0],colors[color][1],colors[color][2],1.f);
#   undef CEU2F
}


void CharacterControllerFixSinkingEffectOnFall(unsigned body,float* mMatrix16,int16_t aux_body) {
    // attempt to mitigate the 'sinking effect' of character 'body' after a jump or a fall on static/kinematic simple bodies
    using namespace nudge;
    if (aux_body>=0 && (unsigned)aux_body<c->bodies.count && (c->bodies.filters[aux_body].flags&BF_IS_STATIC_OR_KINEMATIC)) {
        const BodyLayout* layout = &c->bodies.layouts[body];
        const BodyLayout* aux_layout = &c->bodies.layouts[aux_body];
        if (aux_layout->num_boxes==1 && aux_layout->num_spheres==0 &&
                layout->num_boxes==1 && layout->num_spheres==0
                ) {
            const BoxCollider* coll = &c->colliders.boxes.data[layout->first_box_index];
            const BoxCollider* aux_coll = &c->colliders.boxes.data[aux_layout->first_box_index];
            const Transform* T = &c->bodies.transforms[body];
            const Transform* aux_T = &c->bodies.transforms[aux_body];
            // WARNING: here we assume:
            // -> no collider offset transforms and
            // -> no center of mass offsets in the 2 bodies plus
            // -> no pitch and roll in T and aux_T
            // (but it's just a test)
            if (T->p[1]-coll->size[1]<aux_T->p[1]+aux_coll->size[1])
                mMatrix16[13]=aux_T->p[1]+aux_coll->size[1]+coll->size[1];
        }
    }
}



float* getStarPosition(unsigned number) {
    static float pos[12*3] = {11.19,4.60,-11.00,-12.03,6.43,-10.71,-11.95,6.43,5.48,-21.02,1.50,0.20,
                            1.93,3.82,20.60,20.55,4.86,0.23,-20.97,6.98,1.63,-7.04,5.78,1.38,
                            -3.69,5.53,-3.30,1.06,1.28,-11.79,-0.01,12.73,1.05,-3.55,2.43,-4.65};

    assert(number<12);
    return &pos[(number)*3];
}
