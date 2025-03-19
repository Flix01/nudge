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
// Can be compiled with the provided build files, but also with:
// c++ -march=native -fno-rtti -fno-exceptions -O3 -fno-rtti -fno-exceptions -openmp-simd -DNDEBUG -I../ ./stdafx.cpp ./example02.cpp -o example02 -lglut -lGL

// Optional definition: -DAZERTY_KEYBOARD_LAYOUT


#define EXAMPLE02_CPP_

#include "stdafx.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h> // strtol (removable, but otherwise unsafe scanf must be used)

//#define USE_FREEGLUT  // optional (it usually needs to link to -lfreeglut, except on Linux where it's still -lglut)

#ifndef USE_FREEGLUT
#   ifdef __APPLE__
#       include <GLUT/GLUT.h>
#   elif defined(_WIN32)
#       include <GLUT/glut.h>   // <GL/glut> ?
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


// custom replacements of gluPerspective(...) and gluLookAt(...), so that we don't need -lglu
void glPerspective(float degfovy, float aspect, float zNear, float zFar);
void glLookAt(float eyeX,float eyeY,float eyeZ,float centerX,float centerY,float centerZ,float upX,float upY,float upZ);

// some stuff regarding the way we render graphic bodies => not related to nudge.h
enum ShapeEnum {
	SHAPE_BOX=0,
	SHAPE_SPHERE,
    SHAPE_SPHERE_LOW_POLY,
    SHAPE_CYLINDER_Y,
    SHAPE_CYLINDER_Z,
    SHAPE_HOLLOW_CYLINDER_Y,
    SHAPE_HOLLOW_CYLINDER_Z,
    SHAPE_CAPSULE_Y,
    SHAPE_CAPSULE_Z,
    SHAPE_SKITTLE,
    SHAPE_TORUS_Y,
    SHAPE_TORUS_Z,
    SHAPE_CONE_Y,
    SHAPE_PRISM_5_Y,
    SHAPE_PRISM_6_Y,
    SHAPE_PRISM_7_Y,
    SHAPE_PRISM_8_Y,
    SHAPE_TEAPOT,
    SHAPE_ROOF,
    SHAPE_CHARACTER,
    SHAPE_STAR,
    SHAPE_PIVOT3D_CENTER,
    SHAPE_PIVOT3D_AXIS_X,
    SHAPE_PIVOT3D_AXIS_Y,
    SHAPE_PIVOT3D_AXIS_Z,
    SHAPE_PIVOT3D,
    SHAPE_AABB,
    SHAPE_STAR_2D,
    SHAPE_COUNT,
    SHAPE_NONE=SHAPE_COUNT
};
void compileDisplayLists(); // we use OpenGL display lists to speed up rendering (at the expense of flexibility)
void drawShape(ShapeEnum shape); // this internally calls glCallList(...) to display geometry

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
void glColorEnum(ColorEnum color); // internally this becomes glColor3fv(...)




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
		float pMatrixFarPlane; // = 200.0f;	
	} proj;
    struct key_t {
        uint32_t down,down_last_frame,pressed/*_this_frame*/,released/*_this_frame*/;   // bit-mask of KeyMask values below
        int mouseX,mouseY;  // mouse position (updated only when a key or mouse button is pressed)
    } key;  // read-only
    struct gui_t {
        unsigned menu_idx;
        float menu_bg_alpha;
        int blink_markers[3];
    } gui;  // internal
    struct selection_t {
        unsigned body;  // body selected by RMB
        int16_t box_relative_index,sphere_relative_index;
    } selection;
	GLuint display_lists[SHAPE_COUNT];	// display lists speed up rendering	
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

} globals = {{},1,0,{{{0,2,0},2*M_PI,M_PI*0.125f,20,{0,0,0}},{{0,2,0},M_PI,M_PI*0.125f,5,{0,0,0}}},{1,1,2},{45.f,0.5f,200.0f},{0,0,0,(uint32_t)-1,0,0},{0,0.65f,{0}},{NUDGE_INVALID_BODY_ID,-1,-1},{0},16.2,{NUDGE_INVALID_BODY_ID,NUDGE_INVALID_BODY_ID,NUDGE_INVALID_BODY_ID,NUDGE_INVALID_BODY_ID,NUDGE_INVALID_BODY_ID,0,(unsigned)-1,0},1,0};
static nudge::context_t* c = &globals.nudge_context;	// shorter... we'll use this

// some experimental stuff to ease tweaking/debugging
void glprintf(const char *format,...);  // handy user function to display text onscreen printf-style
void glprintf_blinkmarker(int idx);     // interaction with the blink markups: glprintf("My /b[1]text/b[1] blinkable");glprintf_blinkmarker(1); makes 'text' blink

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
inline void bind_body(nudge::context_t* c,unsigned body,ShapeEnum shape,ColorEnum color=COLOR_NONE) {c->bodies.infos[body].user.u16[0] = shape; c->bodies.infos[body].user.u16[1] = (color!=COLOR_NONE)?color:(ColorEnum)(1+(body%(COLOR_COUNT-1)));/*(persistent) 'random' body color*/}
inline ShapeEnum bodyinfo_get_shape_enum(const nudge::BodyInfo* info) {return (ShapeEnum) info->user.u16[0];}
inline ColorEnum bodyinfo_get_color_enum(const nudge::BodyInfo* info) {return (ColorEnum) info->user.u16[1];}
float* getStarPosition(unsigned number); // forward declaration
void InitPhysics() {
    using namespace nudge;
    if (c->MAX_NUM_BODIES==0) {
        init_context_with(c,1024,512);    // first call
        // usually here users can set some c->simulation_params if necessary
    }
    else restart_context(c); // when KEY_R is pressed

    unsigned body;
    Transform T = identity_transform;

    // Ground mesh (box)
    T.p[0]=0.f;T.p[1]=-0.3f;T.p[2]=0.0f;
    body = add_box(c,0,1.5f*6.f,0.3f,1.5f*6.f,&T);assert(body!=NUDGE_INVALID_BODY_ID);    // we should always check the return value (but we don't)
    bind_body(c,body,SHAPE_BOX,COLOR_LIGHTGREEN); // this is our GRAPHIC body link (nothing regarding physics)
    //T = identity_transform; // we 'should' do this every time (but we don't)

    // 4 other ground meshes (boxes)
    T.p[0]=-16.f;T.p[1]=-0.3f;T.p[2]=2.5f;
    body = add_box(c,0,2.95f,0.3f,6.0f,&T);
    bind_body(c,body,SHAPE_BOX,COLOR_LIGHTGREEN);
    T.p[0]=11.f;T.p[1]=2.f;T.p[2]=-11.f;
    body = add_box(c,0,2.f,0.2f,2.,&T);
    bind_body(c,body,SHAPE_BOX,COLOR_DARKSEAGREEN);
    T.p[0]=-11.f;T.p[1]=3.8f;T.p[2]=-9.f;
    body = add_box(c,0,2.f,0.2f,4.,&T);
    bind_body(c,body,SHAPE_BOX,COLOR_DARKSEAGREEN);
    T=identity_transform;T.p[0]=-12.f;T.p[1]=4.f-0.2f;T.p[2]=5.5f;
    body = add_box(c,0.f,1.f,0.2f,1.f,&T);
    bind_body(c,body,SHAPE_BOX,COLOR_DARKSEAGREEN);

    // a dynamic cube (box)
    T.p[0]=-6.f;T.p[1]=0.5f;T.p[2]=3.0f;
    body = add_box(c,4.f,0.5f,0.5f,0.5f,&T);
    bind_body(c,body,SHAPE_BOX,COLOR_AZURE);

    // house (box)
    T.p[0]=-3.75f;T.p[1]=1.5f;T.p[2]=-1.0f;
    body = add_box(c,0.f,1.5f,1.5f,2.25f,&T);
    bind_body(c,body,SHAPE_BOX,COLOR_IVORY);

    // house roof (box compound)
    T.p[0]=-3.75f;T.p[1]=3.5f-0.1f;T.p[2]=-1.0f;
    Transform boxT[2] = {{{{-0.9f,0.f,0.f}},{0},{{0.f,0.f,0.f,1.f}}},{{{0.9f,0.f,0.f}},{0},{{0.f,0.f,0.f,1.f}}}};
    const float hsizeTriplets[6] = {1.2f,0.15f,2.75f,    1.2f,0.15f,2.75f};
    nm_QuatFromAngleAxis(boxT[0].q,M_DEG2RAD(36.f),0.f,0.f,1.f);
    nm_QuatFromAngleAxis(boxT[1].q,M_DEG2RAD(-36.f),0.f,0.f,1.f);
    body = add_compound(c,0.f,NULL,2,hsizeTriplets,boxT,0,NULL,NULL,&T);
    bind_body(c,body,SHAPE_ROOF,COLOR_DARKRED);

    // static cylinderY (box)
    T.p[0]=-1.65f;T.p[1]=1.2f;T.p[2]=-1.5f;
    body = add_box(c,0.f,0.2f,1.25f,0.2f,&T);
    bind_body(c,body,SHAPE_CYLINDER_Y,COLOR_LIGHTBLUE);

    // long cylinderZ / capsuleZ (compound: 1 box + 2 or 3 spheres)
    T.p[0]=0.65f;T.p[1]=4.2f;T.p[2]=-1.5f;
    float cylradius = 0.3f, cyloffs = cylradius/3.f, cylhheight = 0.75f;
    float cylsize[3] = {cylradius-cyloffs,cylradius-cyloffs,cylhheight};   // all half dimensions
    float cylradii[3] = {cylradius,cylradius,cylradius};
    Transform cylT[4] = {
        {{{0,0,0}},{},{{0,0,0,1}}},   // box
        {{{0,0,cylhheight-cylradius}},{},{{0,0,0,1}}},   // sphere0
        {{{0,0,-cylhheight+cylradius}},{},{{0,0,0,1}}},   // sphere1
        {{{0,0,0}},{},{{0,0,0,1}}},   // sphere2
    };    
    float cylmass = 1.f;
    float cylInertia[3]; calculate_cylinder_inertia(cylInertia,cylmass,cylradius,cylhheight,AXIS_Z);
    const bool cylUseJust2Spheres = false;  // we can save the sphere at the center if we like
    body = add_compound(c,cylmass,cylInertia,1,cylsize,&cylT[0],cylUseJust2Spheres?2:3,cylradii,&cylT[1],&T,NULL);
    bind_body(c,body,SHAPE_CYLINDER_Z,COLOR_LIGHTGOLDENRODYELLOW);

    T.p[1]+=2.f;T.p[2]-=3.f;cylT[1].p[2]+=cylradius;cylT[2].p[2]-=cylradius;    // in the case of capsule cylhheight is the half height without radius
    nm_QuatFromAngleAxis(T.q,-M_DEG2RAD(90.f),0.f,1.f,0.f);
    float capsInertia[3]; calculate_capsule_inertia(capsInertia,cylmass,cylradius,cylhheight,AXIS_Z);
    body = add_compound(c,cylmass,capsInertia,1,cylsize,&cylT[0],cylUseJust2Spheres?2:3,cylradii,&cylT[1],&T,NULL); // inertia is slightly incorrect here... nevermind
    bind_body(c,body,SHAPE_CAPSULE_Z,COLOR_LIGHTSEAGREEN);


    // short cylinder (box compound)
    T=identity_transform;
    cylradius = 0.4f,cylhheight=0.1f;
    T.p[0]=0.f;T.p[1]=1.f*cylradius;T.p[2]=-6.5f;
    Transform scylT[8];const int num_boxes_for_scyl = (int) sizeof(scylT)/sizeof(scylT[0]);
    float hsizeTripletsForScyl[num_boxes_for_scyl*3];
    float cylBoxLength = cylradius * sin(M_PI/(float)num_boxes_for_scyl);
    float cylBoxWidth = cylradius * cos(M_PI/(float)num_boxes_for_scyl);
    for (int l=0;l<num_boxes_for_scyl;l++)   {
        // set hsizeTripletsForScyl[l*3] and scylT[l]
        float* triplet = &hsizeTripletsForScyl[l*3];
        triplet[0]=cylBoxWidth;
        triplet[1]=cylBoxLength;
        triplet[2]=cylhheight;

        Transform* tr = &scylT[l];
        const float angle = (float)l*M_PI/(float)num_boxes_for_scyl;
        nm_QuatFromAngleAxis(tr->q,angle,0.f,0.f,1.f);
        tr->p[0]=tr->p[1]=tr->p[2]=0.f;
    }
    body = add_compound(c,cylmass,cylInertia,num_boxes_for_scyl,hsizeTripletsForScyl,scylT,0,NULL,NULL,&T);
    bind_body(c,body,SHAPE_CYLINDER_Z,COLOR_AQUA);

    // torus (box compund)
    T=identity_transform;
    const float torusScale = 1.0f;const float torusMass = 1.f;  // in short we can only scale the torus globally, because we use display list for drawing shapes (relation between R and r is fixed)
    T.p[0]=0.f;T.p[1]=1.f*torusScale;T.p[2]=-2.5f;
    // code for experts
    float r = 0.2f*torusScale, R = 0.8f*torusScale;
    float torusInertia[3];calculate_torus_inertia(torusInertia,torusMass,R,r,AXIS_Z);
    Transform csT[8];const int num_boxes = (int) sizeof(csT)/sizeof(csT[0]);
    float hsizeTripletsForTorus[num_boxes*3];
    const float boxLength = R * sin(M_PI/(float)num_boxes);
    for (int l=0;l<num_boxes;l++)   {
        // set hsizeTripletsForTorus[l*3] and csT[l]
        float* triplet = &hsizeTripletsForTorus[l*3];
        triplet[0]= boxLength;  triplet[1]=triplet[2]= r;

        Transform* tr = &csT[l];
        const float angle = (float)l*2.f*M_PI/(float)num_boxes;
        const float sinAngle = sin(angle), cosAngle = cos(angle);
        nm_QuatFromAngleAxis(tr->q,angle,0.f,0.f,1.f);
        tr->p[2]=0.f;tr->p[0]=R*sinAngle;tr->p[1]=-R*cosAngle;
    }
    // end code for experts
    body = add_compound(c,torusMass,torusInertia,num_boxes,hsizeTripletsForTorus,csT,0,NULL,NULL,&T);
    bind_body(c,body,SHAPE_TORUS_Z,COLOR_CORAL);

    // test add_clone(...) [experimental]
    T.p[1]=0.f;T.p[1]=1.5f*torusScale;T.p[2]=2.5f;
    body = add_clone(c,body,torusMass*1.5f,&T,-1.f);    // see the docs for further info
    bind_body(c,body,SHAPE_TORUS_Z,COLOR_CADETBLUE);

    // hollow cylinder (box compound)
    T=identity_transform;
    const float hollowCylScaleXZ = 1.f;cylhheight=0.75f; // tweakable
    T.p[0]=6.2f;T.p[1]=cylhheight *3.f;T.p[2]=-4.5f;
    cylradius = hollowCylScaleXZ*0.85f;
    const float cylinnerradius = hollowCylScaleXZ*0.15f;// it's R-d, where d is the width of the border. It is not frelly settable because we use display lists
    cylBoxLength = hollowCylScaleXZ* tan(M_PI/(float)num_boxes_for_scyl);

    for (int l=0;l<num_boxes_for_scyl;l++)   {
        float* triplet = &hsizeTripletsForScyl[l*3];
        triplet[0]=cylBoxLength;
        triplet[1]=cylhheight;
        triplet[2]=cylinnerradius;

        Transform* tr = &scylT[l];
        const float angle = (float)l*2.f*M_PI/(float)num_boxes_for_scyl;
        const float sinAngle = sin(angle), cosAngle = cos(angle);
        nm_QuatFromAngleAxis(tr->q,-angle,0.f,1.f,0.f);
        tr->p[0]=(cylradius)*sinAngle;
        tr->p[1]=0.f;
        tr->p[2]=-(cylradius)*cosAngle;
    }
    cylmass = 2.f;
    calculate_hollow_cylinder_inertia(cylInertia,cylmass,cylradius+cylinnerradius,cylradius-cylinnerradius,cylhheight,AXIS_Y);
    body = add_compound(c,cylmass,cylInertia,num_boxes_for_scyl,hsizeTripletsForScyl,scylT,0,NULL,NULL,&T);
    bind_body(c,body,SHAPE_HOLLOW_CYLINDER_Y,COLOR_YELLOW);
    T=identity_transform;T.p[0]=-7.5f;T.p[2]=1.f;T.p[1]=cylhheight;
    body = add_compound(c,0,NULL,num_boxes_for_scyl,hsizeTripletsForScyl,scylT,0,NULL,NULL,&T);
    bind_body(c,body,SHAPE_HOLLOW_CYLINDER_Y,COLOR_ORANGE);

    // sensor experiment (test)
    T.p[1]=cylhheight*0.1f;
    const bool correct_way_to_make_a_sensor = false;
    if (correct_way_to_make_a_sensor) {
        body = add_box(c,1000.f,cylradius/1.9f,T.p[1],cylradius/1.9f,&T);
        float* tmp = c->bodies.properties[body].inertia_inverse;tmp[0]=tmp[1]=tmp[2]=0;
        tmp = c->bodies.properties[body].gravity;tmp[0]=tmp[1]=tmp[2]=0;
        c->bodies.filters[body].flags|=BF_IS_SENSOR;    // note that BF_IS_SENSOR ia completely unused in nudge.h (just an empty tag)
        c->bodies.filters[body].collision_mask = 0; // don't collide with any body
        c->bodies.filters[body].flags|=BF_NEVER_SLEEPING;   // this is bad! This way c->active_bodies.count is always positive!
        bind_body(c,body,SHAPE_NONE,COLOR_RED);
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
        body = add_box(c,0.f,cylradius/1.9f,T.p[1],cylradius/1.9f,&T);
        c->bodies.filters[body].flags|=BF_IS_SENSOR;    // note that BF_IS_SENSOR ia completely unused in nudge.h (just an empty tag)
        //c->bodies.filters[body].collision_mask = 0; // [NOPE!] don't collide with any body (if set => no more collisions reported here)
        bind_body(c,body,SHAPE_NONE,COLOR_RED);
    }


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
        bind_body(c,body,SHAPE_SKITTLE,COLOR_GOLDENROD);
        c->bodies.filters[body].collision_group = COLLISION_GROUP_A;   // group the body belongs to (see the flags of the kinematic teapot)
    }
    memset(comOffset,0,sizeof(float)*3);    // reset comOffset variable

    // test extra:: namespace
    T=identity_transform;T.p[0]=-3.5f;T.p[1]=0.5f;T.p[2]=6.f;
    body = extra::add_compound_cylinder(c,0.3f,0.25f,T.p[1],&T,AXIS_Y,0,0);
    bind_body(c,body,SHAPE_CYLINDER_Y,COLOR_PINK);

    // test extra:: namespace
    T=identity_transform;T.p[0]=-5.0f;T.p[1]=0.5f+0.25f;T.p[2]=6.f;
    body = extra::add_compound_capsule(c,0.5f,0.25f,0.5f,&T,AXIS_Y,0,0);
    bind_body(c,body,SHAPE_CAPSULE_Y,COLOR_AZURE);

    // test extra:: namespace
    T=identity_transform;T.p[0]=-7.0f;T.p[1]=0.5f;T.p[2]=8.f;
    body = extra::add_compound_cone(c,0.5f,0.5f,T.p[1],&T,AXIS_Y,0,0);
    bind_body(c,body,SHAPE_CONE_Y,COLOR_YELLOW);    // SHAPE_CONE_Y enum draws wrong lateral normals

    // test extra:: namespace
    T=identity_transform;T.p[0]=-6.5f;T.p[1]=0.5f;T.p[2]=6.f;
    body = extra::add_compound_hollow_cylinder(c,0.5f,0.35f,0.5f,T.p[1],&T,AXIS_Y,0,0);    // same as the comment below
    bind_body(c,body,SHAPE_HOLLOW_CYLINDER_Y,COLOR_ORANGERED);

    // test extra:: namespace
    T=identity_transform;T.p[0]=-6.5f;T.p[1]=0.225f;T.p[2]=4.f;
    body = extra::add_compound_torus(c,1.f,T.p[1]*4.f,T.p[1],&T,AXIS_Y,16); // the 'graphic' torus is hard-coded in the display lists, so we can't freely choose its two radii (and I'm not sure factor 4.f is correct)
    bind_body(c,body,SHAPE_TORUS_Y,COLOR_YELLOW);

    // test extra:: namespace
    T=identity_transform;T.p[0]=-5.0f;T.p[1]=0.5f;T.p[2]=4.f;
    body = extra::add_compound_prism(c,1.f,0.5f,T.p[1],6,&T,AXIS_Y);   // min 4 lateral faces (but in that case use SHAPE_BOX).
    bind_body(c,body,SHAPE_PRISM_6_Y,COLOR_SKYBLUE);    // SHAPE_PRISM_ enums draw wrong lateral normals (and are a bit too small to fit the correct aabb)

    // kinematic teapot (box) [manual kinematic body test] [collision mask test]
    T=identity_transform;T.p[0]=1.5f;T.p[1]=0.5f;T.p[2]=8.0f;
    globals.bodies.kinematic_body = body = add_box(c,-10000.f,0.5f,0.5f,0.75f,&T);
    bind_body(c,body,SHAPE_TEAPOT,COLOR_LIGHTSKYBLUE);
    c->bodies.filters[body].collision_group = COLLISION_GROUP_E;   // group the body belongs to
    c->bodies.filters[body].collision_mask = COLLISION_GROUP_ALL & (~COLLISION_GROUP_A);    // groups the body can collide with (all except bodies of group A)

    // (kinematic animation test: door) [automatic kinematic body test]
    {
        // -> kinematic body creation
        T=identity_transform;T.p[0]=-0.5f;T.p[1]=1.0f;T.p[2]=-8.0f;
        comOffset[0]=-0.625f;   // this is JUST to ease the open-door animation later
        body = add_box(c,-10000.f,0.625f,1.f,0.1f,&T,comOffset);
        bind_body(c,body,SHAPE_BOX,COLOR_DARKBLUE);
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
        bind_body(c,body,SHAPE_BOX,COLOR_DARKBLUE);
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
    bind_body(c,body,SHAPE_CHARACTER,globals.bodies.num_stars<12?COLOR_YELLOW:COLOR_ORANGE);
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
        bind_body(c,body,SHAPE_STAR,COLOR_YELLOW);
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
        bind_body(c,body,SHAPE_BOX,COLOR_DARKGREEN);
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
        bind_body(c,body,SHAPE_BOX,COLOR_LIGHTSEAGREEN);
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
        bind_body(c,body,SHAPE_BOX,COLOR_DARKGREEN);
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
    bind_body(c,body,SHAPE_BOX,COLOR_RED);
    c->bodies.momentum[body].velocity[0]=-1.5f; // this line does not work, because we're overriding this value at runtime in UpdatePhysics(), but it usually works
    c->bodies.properties[body].friction = 2.f;
    c->bodies.filters[body].flags|=BF_IS_PLATFORM;  // test (toggle and see the difference on character)

    T.p[0]=-12.f;T.p[1]=4.f-0.2f;T.p[2]=-0.25f;
    globals.bodies.conveyor_belt_body1 = body = add_box(c,0,1.f,0.2f,4.75f,&T);
    bind_body(c,body,SHAPE_BOX,COLOR_RED);
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
        bind_body(c,body,SHAPE_BOX,COLOR_DARKGREEN);
        c->bodies.filters[body].flags|=BF_IS_PLATFORM;     // note that this flag does nothing in nudge.h
        c->bodies.infos[body].sk_user.i8[3]=0;  // we'll use this value as a driver for platform movement direction (-1,0,1); 0 = platform stands still

        globals.bodies.rotating_platform_animation_index = c->kinematic_data.animations_count;
        KinematicData::Animation* ka = &c->kinematic_data.animations[c->kinematic_data.animations_count++];
        ka->key_frame_start = start_key_frame; ka->key_frame_count = new_frames_cnt;
        ka->loop_mode = KinematicData::Animation::LM_LOOP_NORMAL;
        ka->body = body;
        ka->use_baseT = true; ka->baseT = c->bodies.transforms[body];nm_QuatFromAngleAxis(ka->baseT.q,M_PI*1.5f,0,1,0);   // unless key frame Transforms are in the absolute space, this must always be set
        ka->speed = 0.f;    // we'll set the speed dynamically when c->bodies.infos[body].aux_bodies[1]!=0 (we'll set that when the character is on it)
        ka->play_time = ka->offset_time = 0.f;
        ka->playing = true; // we could trigger this instead of ka->speed... but it's almost the same

        c->bodies.transforms[body] = ka->baseT; // this places 'body' at frame 0 (if it has an identity T like in our case) even if ka->playing==false or ka->speed==0; (otherwise is just useless)
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
            bind_body(c,body,SHAPE_BOX,COLOR_RED);

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
        bind_body(c,body,SHAPE_BOX,COLOR_RED);

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
        bind_body(c,body,SHAPE_BOX,COLOR_NONE);
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
        bind_body(c,body,SHAPE_SPHERE,COLOR_NONE);
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
            const bool character_must_jump = aux_body>=0 && (state_key_pressed&KEY_J);
            const bool character_is_on_platform =  aux_body>=0 && (c->bodies.filters[aux_body].flags&BF_IS_PLATFORM);
            if (character_must_jump) {
                const float amount = 5.f;
                c->bodies.momentum[globals.bodies.character_body].velocity[1]+=amount;
                if (c->bodies.filters[globals.bodies.character_body].flags&BF_IS_DYNAMIC) c->bodies.idle_counters[globals.bodies.character_body]=0;  // wakes up body (dynamic body only)
            }
            else if ((state_key_down&KEY_WASD) || character_is_on_platform)   {
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
                    angle*=axis[1]; // can be 1 or -1 AFAICS
                    const float sinangle=amount*sinf(angle),cosangle=amount*cosf(angle);
                    //SDL_Log("Angle axis:%1.3f; {%1.3f,%1.3f,%1.3f}\n",angle,axis[0],axis[1],axis[2]);
                    tr.position[0] = T->position[0] + sinangle;
                    tr.position[2] = T->position[2] + cosangle;
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
                //*velocityY = c->bodies.momentum[aux_body].velocity[1];
                /*if (!(state_key_down&KEY_WASD)) {
                    velocity[0]=c->bodies.momentum[aux_body].velocity[0];
                    //velocity[1]=c->bodies.momentum[aux_body].velocity[1];
                    velocity[2]=c->bodies.momentum[aux_body].velocity[2];
                }*/

                /*if (character_is_on_platform) {
                    //glprintf(0.2,0.2,"ON PLATFORM");
                    //nudge::log("[%lu]: ON PLATFORM\n",c->simulation_params.num_frames);
                }*/
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
        //unsigned num_wrong_contacts=0;
        for (unsigned i = 0; i < cd->count; i++) {
            const unsigned a = cd->bodies[i].a, b = cd->bodies[i].b;
            const BodyFilter *a_filter = &c->bodies.filters[a], *b_filter = &c->bodies.filters[b];
            //if (a_filter->flags&BF_IS_DISABLED || b_filter->flags&BF_IS_DISABLED) ++num_wrong_contacts;
            if (a==b) continue; // Not sure if/why this happens
            /*const int a_dynamic = (a_filter->flags&BF_IS_DYNAMIC);
               const int b_dynamic = (b_filter->flags&BF_IS_DYNAMIC);
               const int sum_dynamic = a_dynamic+b_dynamic;
               if (sum_dynamic==0) continue;   // Why this case is present in the first place? */
            //const uint64_t tag = cd->tags[i];

            //assert(a_filter->flags&BF_IS_DYNAMIC || b_filter->flags&BF_IS_DYNAMIC);
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
                        ++globals.bodies.num_stars;
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
                    int16_t* character_aux_body = &c->bodies.infos[character].aux_bodies[0];
                    if (*character_aux_body==-1)    {
                        // Check if contact is below 'character', and set aux_body for 'character'
                        // cd->data->position and T->position are in world space
                        // we assume contact normal is in the world space... (it makes no sense, because it should depend on the order of the two bodies, but it works)
                        // actually we are just testing: contact->normal[1]<-0.95f AFAICS, so the normal points down not up...
                        // we should do extensive tests to understand if body order is random or if static/kinematic vs dynamic has a specific order, etc...
                        const Contact* contact = &cd->data[i];
                        const float dot_down =  contact->normal[0]*0+contact->normal[1]*-1+contact->normal[2]*0;  // TODO: Better dot between contact->normal and body gravity (and normalize)
                        const int is_on_ground = (a_is_character&&dot_down>0.95f) || (b_is_character&&dot_down<-0.95f);   // TODO: Never tested the second part (I always have a_is_character)
                        if (is_on_ground) *character_aux_body=(int16_t) other_body;
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
void DrawPhysics()  {
    using namespace nudge;
    // Note: this function is part of DrawGL, and it occasionally contains gl calls

    // we use a different color for (dynamic) bodies with a center of mass offset,
    // and another for sleeping (dynamic) bodies (but only when !use_graphic_transform)
    const ColorEnum no_active_body_color = COLOR_BLACK;
    const ColorEnum sleeping_color = COLOR_DARKSLATEGRAY;
    const ColorEnum com_offset_body_color = COLOR_DARKBLUE;

    float mMatrix[16];unsigned num_sleeping_and_dynamic=0;globals.num_frustum_culled_bodies=0;
    const int use_graphic_transform = globals.use_graphic_transform;
    for (unsigned body=0,body_count=c->bodies.count;body<body_count;body++)	{
        BodyFilter* filter = &c->bodies.filters[body];	// BodyFilter hosts per-body flags, collision masks and sleeping state
        if (filter->flags&BF_IS_DISABLED_OR_REMOVED) continue;
        filter->flags&=~BF_IS_FRUSTUM_CULLED;   // removes this flag from the body
        const BodyInfo* info = &c->bodies.infos[body];   // BodyInfo hosts our custom per-body (user-side) data (in this example the SHAPE_ enum and the COLOR_ enum), plus per-body aabb data
        const ShapeEnum shape = bodyinfo_get_shape_enum(info);

        //------------------------------------------------------------
        // we exploit this loop to remove fallen bodies so that their colliders (i.e. collision shapes)
        // are freed and the body indices are reusable by nudge::add_ (this happens automatically)
        if (c->bodies.transforms[body].p[1]<-40.f && filter->flags&BF_IS_DYNAMIC) {
            // here we just remove (graphic) boxes and spheres, and respawn other bodies
            if ((shape==SHAPE_BOX || shape==SHAPE_SPHERE || (body==globals.bodies.star_body && globals.bodies.num_stars==12)) && !(filter->flags&(BF_IS_CHARACTER|BF_IS_PLATFORM))) {
                remove_body(c,body);if (body==globals.selection.body) {globals.selection.body=NUDGE_INVALID_BODY_ID;globals.selection.box_relative_index=globals.selection.sphere_relative_index=-1;}
                if (body==globals.bodies.star_body) {
                    globals.bodies.star_body=NUDGE_INVALID_BODY_ID;
                    if (globals.bodies.character_body<c->bodies.count) {bind_body(c,globals.bodies.character_body,SHAPE_CHARACTER,COLOR_ORANGE);c->bodies.momentum[globals.bodies.character_body].velocity[1]=5.f;}
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

        const bool is_dynamic = filter->flags&BF_IS_DYNAMIC;
        const int is_sleeping_and_dynamic = c->bodies.idle_counters[body]==0xFF && is_dynamic;num_sleeping_and_dynamic+=(is_sleeping_and_dynamic?1:0);
        const int has_com_offset = filter->flags&BF_HAS_COM_OFFSET;

        assert(info->user.u16[1]<COLOR_COUNT);// we have stuffed the per-body color enum in info->userUint16[1]
        ColorEnum color = (ColorEnum) bodyinfo_get_color_enum(info);assert(color!=COLOR_NONE); // COLOR_NONE is replaced in bind_body(...)

        if (use_graphic_transform)	{
            // Use use_graphic_transform=1 in release mode, where you have a single graphic mesh
            // (simulated using 1 or more boxes/spheres). We end up having a single smoothed
            // transform per body, and we use it to draw our graphic mesh.

            calculate_graphic_transform_for_body(c,body,mMatrix);	// what this does is: 1) smoothes the graphic transform (using DeltaTimes less than timeStep); 2) embeds the center of mass offset (so that it can be stripped from the aabb later [*])

            // frustum culling on mMatrix here
            const float* aabb_hextents = info->aabb_half_extents;   // 3-floats array
            const float aabb_center[3] = {info->aabb_center[0]+info->com_offset[0],info->aabb_center[1]+info->com_offset[1],info->aabb_center[2]+info->com_offset[2]};  // here we strip the comOffset from aabb_center, because we're culling on a mMatrix that embeds it already [*]
            if (globals.use_frustum_culling && !glIsAabbVisible(mMatrix,aabb_center[0],aabb_center[1],aabb_center[2],aabb_hextents[0],aabb_hextents[1],aabb_hextents[2])) {
                filter->flags|=BF_IS_FRUSTUM_CULLED;
                ++globals.num_frustum_culled_bodies;
                continue;
            }
            // end frustum culling on mMatrix

            // experiment (to remove) ------
            if ((filter->flags&BF_IS_CHARACTER) && globals.bodies.fix_character_sinking_effect_on_fall && info->aux_bodies[0]>=0)
                CharacterControllerFixSinkingEffectOnFall(body,mMatrix,info->aux_bodies[0]);  // [experimental] attempt to mitigate the 'sinking effect' of characters after a jump or a fall
            // -----------------------------

            glPushMatrix();

            // 2) inglobes the com_offset (if present), so that we should not manually translate our graphic mesh
            glMultMatrixf(mMatrix);


            // Now we just add our custom code to draw our graphic body (= the graphic representation of the body)
            // => here we don't retrieve any physic quantity at all to determine the scaling we must apply
            // => instead we retrieve the shape axis aligned bounding box extents (inside BodyInfo), which is a geometric quantity, in a suitable format
            //    Note that that quantity is NOT a physic quantity, because it's NOT USED by nudge.h at all (it was added just for user convenience)

            // Note that here we don't use info->aabb_center at all to draw the body
            // Here in our case the sum (info->aabb_center+info->com_offset) is always nearly {0,0,0}: this sum is the aabb center we have introduced in add_compund(...) calls, stripped by (=regardless of) the center of mass offset ([*])
            // In any case, even if the sum is not zero, normally we should just skip it when drawing the geometry, if our graphic mesh has the same aabb center (it should be true when we build the colliders to cover a predefined mesh).
            // In case of graphic center mismatches I suggest to: 1) center the mesh built with add_compound(...), using last argument, or 2) Calculate (info->aabb_center+info->com_offset) and use it when redering the graphic body.

            glColorEnum(color); // internally this calls glColor3fv(...)

            switch (shape) {
            case SHAPE_BOX:
            case SHAPE_SPHERE:
            case SHAPE_ROOF:
            case SHAPE_CYLINDER_Y:
            case SHAPE_CYLINDER_Z:
            case SHAPE_CONE_Y:
            case SHAPE_PRISM_5_Y:
            case SHAPE_PRISM_6_Y:
            case SHAPE_PRISM_7_Y:
            case SHAPE_PRISM_8_Y:
            case SHAPE_SKITTLE:
            case SHAPE_TEAPOT:
            case SHAPE_STAR:
            case SHAPE_CHARACTER:
            {
                glScalef(aabb_hextents[0],aabb_hextents[1],aabb_hextents[2]);
                drawShape(shape);
            }
                break;
            case SHAPE_CAPSULE_Y:
            case SHAPE_CAPSULE_Z: {
                float scale[3]={1,1,1};
                float radius = 1,hheight = 1;
                if (shape==SHAPE_CAPSULE_Z) {
                    radius = (aabb_hextents[0]+aabb_hextents[1])*0.5f;hheight = aabb_hextents[2];assert(hheight>=radius);hheight-=radius;
                    scale[0]=radius;scale[1]=radius;scale[2]=hheight;
                }
                else if (shape==SHAPE_CAPSULE_Y) {
                    radius = (aabb_hextents[0]+aabb_hextents[2])*0.5f;hheight = aabb_hextents[1];assert(hheight>=radius);hheight-=radius;
                    scale[0]=radius;scale[1]=hheight;scale[2]=radius;
                }
                glPushMatrix();
                glScalef(scale[0],scale[1],scale[2]);
                drawShape(shape);
                glPopMatrix();

                // capsules are bad shapes to render, because (like torii) they cannot be non-uniformly scaled.
                // However, unlike torii, we can still combine display lists to draw them.
                // so basically drawShape(SHAPE_CAPSULE) just draws the cylinder lateral surface,
                // and now we must add the 2 (half) spheres
                const float deltaZ = (hheight/radius);
                glPushMatrix();
                if (shape==SHAPE_CAPSULE_Y) glRotatef(-90.f,1,0,0);
                glScalef(radius,radius,radius);
                glTranslatef(0,0,deltaZ);
                drawShape(SHAPE_SPHERE_LOW_POLY);
                glTranslatef(0,0,-2.f*deltaZ);
                drawShape(SHAPE_SPHERE_LOW_POLY);
                glPopMatrix();

            }
                break;
            case SHAPE_TORUS_Y:
            case SHAPE_TORUS_Z: {
                const float uniform_scale = (shape==SHAPE_TORUS_Y)?(aabb_hextents[0]+aabb_hextents[2])*0.5f:(aabb_hextents[0]+aabb_hextents[1])*0.5f;
                glScalef(uniform_scale,uniform_scale,uniform_scale);
                drawShape(shape);
            }
                break;
            case SHAPE_HOLLOW_CYLINDER_Y:
            case SHAPE_HOLLOW_CYLINDER_Z: {
                const float lateral_scale = (shape==SHAPE_HOLLOW_CYLINDER_Y)?(aabb_hextents[0]+aabb_hextents[2])*0.5f:(aabb_hextents[0]+aabb_hextents[1])*0.5f;
                float scale[3]={lateral_scale,lateral_scale,lateral_scale};
                if (shape==SHAPE_HOLLOW_CYLINDER_Y) scale[1]=aabb_hextents[1];
                else scale[2]=aabb_hextents[2];
                glScalef(scale[0],scale[1],scale[2]);
                drawShape(shape);
            }
                break;
            case SHAPE_NONE:
                break;
            default:
                assert(0); // SHAPE_XXX not handled
                break;
            }

            glPopMatrix();
        }
        else {
            // Use use_graphic_transform=0 in debug mode, where you don't have a graphic mesh
            // or you want to see all the boxes/spheres that make up the body.
            // The transforms here are not smoothed.
            // Here we must use the physic layout of the body (the one used by nudge.h)

            const Transform* T1 = &c->bodies.transforms[body];
            if (globals.use_frustum_culling) {
                // frustum culling on T1 here

                // (1)
                // First off, here we're culling on T1 that does not embed the com_offset, so we don't need to strip it from the aabb_center [*]
                // However here we should do a 'TransformToMat4(...)', where the resulting matrix is not needed elsewhere...
                /*float mMatrix[16];TransformToMat4(mMatrix,T1);
                if (!glIsAabbVisible(mMatrix,info->aabb_center[0],info->aabb_center[1],info->aabb_center[2],info->aabb_half_extents[0],info->aabb_half_extents[1],info->aabb_half_extents[2])) {filter->flags|=BF_IS_FRUSTUM_CULLED;++globals.num_frustum_culled_bodies;continue;}*/

                // or (2)
                // Now nudge.h provides an 'aabb_enlarged_radius' that 'covers' the aabb_center so that we can skip the rotation part of T1 completely and still do frustum culling
                // However: a) the culling is slightly broader b) it's not much faster than the other glIsAabbVisible(...) c) I don't know if it can be applied when com_offset is embedded in T1 (to test)
                // Short story: good to use it here, but when we need a mMatrix[16] for something else (e.g. drawing the mesh), we must always use the other glIsAabbVisible(...)
                if (!glIsAabbVisible(T1->p,info->aabb_enlarged_radius)) {filter->flags|=BF_IS_FRUSTUM_CULLED;++globals.num_frustum_culled_bodies;continue;}

                // end frustum culling on T1
            }

            if (is_dynamic) {
                if (c->active_bodies.count==0) color = no_active_body_color;
                else if (is_sleeping_and_dynamic) color = sleeping_color;
                else if (has_com_offset) color = com_offset_body_color;
            }
            glColorEnum(color); // internally this calls glColor3fv(...)

            const BodyLayout* layout = &c->bodies.layouts[body];	// BodyLayout hosts the indices of the body colliders (i.e. collision shapes). The global arrays of colliders are in c->colliders.boxes and c->colliders.spheres
            if (layout->num_boxes>0) {
                assert(layout->first_box_index>=0);
                assert((uint16_t)layout->first_box_index+layout->num_boxes<=c->colliders.boxes.count);
                for (uint16_t i=layout->first_box_index,isz=layout->first_box_index+layout->num_boxes;i<isz;i++)	{
                    const Transform* T2 = &c->colliders.boxes.transforms[i];
                    const Transform T = TransformMul(*T1,*T2);
                    const BoxCollider* coll = &c->colliders.boxes.data[i];
                    TransformToMat4(mMatrix,&T);
                    glPushMatrix();
                    glMultMatrixf(mMatrix);
                    glScalef(coll->size[0],coll->size[1],coll->size[2]);
                    drawShape(SHAPE_BOX);
                    glPopMatrix();
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
                    glPushMatrix();
                    glMultMatrixf(mMatrix);
                    glScalef(coll->radius,coll->radius,coll->radius);
                    drawShape(SHAPE_SPHERE);
                    glPopMatrix();
                }
            }

        }
    }


    // Draw aabb of selected body (RMB)
    const globals_t::selection_t* selected = &globals.selection;
    if (selected->body<c->bodies.count && !(c->bodies.filters[selected->body].flags&(BF_IS_DISABLED_OR_REMOVED|BF_IS_FRUSTUM_CULLED))) {
        const unsigned body = selected->body;
        const BodyInfo* info = &c->bodies.infos[body];
        const float* aabb_hextents = info->aabb_half_extents;   // 3-floats array
        const float* aabb_center = info->aabb_center;   // 3-floats array (note that here we don't have to strip info->comOffset: we should when using calculate_graphic_transform_for_body(...) to get mMatrix)
        float mMatrix[16];TransformToMat4(mMatrix,&c->bodies.transforms[body]);

        glLineWidth(4.f);
        glDisable(GL_LIGHTING);
        glColor3f(0.4f,0.0f,0.0f);
        glPushMatrix();
        glMultMatrixf(mMatrix);
        glTranslatef(aabb_center[0],aabb_center[1],aabb_center[2]); // optional
        glScalef(aabb_hextents[0],aabb_hextents[1],aabb_hextents[2]);
        drawShape(SHAPE_AABB);
        glPopMatrix();

        // Optional: draw aabb of selected collider inside selected body
        const BodyLayout* L = &c->bodies.layouts[body];
        if (L->num_boxes+L->num_spheres>1)  {
            glColor3f(0.0f,0.4f,0.0f);
            if (selected->box_relative_index>=0 && selected->box_relative_index<L->num_boxes) {
                const uint16_t idx = (uint16_t)L->first_box_index+selected->box_relative_index;
                assert(L->first_box_index>=0 && idx<c->colliders.boxes.count);
                const float* he = c->colliders.boxes.data[idx].size;
                const Transform T = TransformMul(c->bodies.transforms[body],c->colliders.boxes.transforms[idx]);
                TransformToMat4(mMatrix,&T);
                glPushMatrix();
                glMultMatrixf(mMatrix);
                glScalef(he[0]*1.05f,he[1]*1.05f,he[2]*1.05f);  // scaling to avoid z-fighting
                drawShape(SHAPE_AABB);
                glPopMatrix();
            }
            if (selected->sphere_relative_index>=0 && selected->sphere_relative_index<L->num_spheres) {
                const uint16_t idx = (uint16_t)L->first_sphere_index+selected->sphere_relative_index;
                assert(L->first_sphere_index>=0 && idx<c->colliders.spheres.count);
                const float r = c->colliders.spheres.data[idx].radius;const float he[3]={r,r,r};
                const Transform T = TransformMul(c->bodies.transforms[body],c->colliders.spheres.transforms[idx]);
                TransformToMat4(mMatrix,&T);
                glPushMatrix();
                glMultMatrixf(mMatrix);
                glScalef(he[0]*1.05f,he[1]*1.05f,he[2]*1.05f);  // scaling to avoid z-fighting
                drawShape(SHAPE_AABB);
                glPopMatrix();
            }
        }
        glEnable(GL_LIGHTING);
    }


    // Dbg: draw another aabb on c->active_bodies (for better understanding what this array is)
    const int dbg_draw_aabb_around_bodies_in_the_active_array = 0;
    if (dbg_draw_aabb_around_bodies_in_the_active_array
            && !globals.use_graphic_transform
            )
    {
        glLineWidth(1.f);
        glDisable(GL_LIGHTING);
        glColor3f(0.8f,0.8f,0.8f);
        for (unsigned i=0;i<c->active_bodies.count;i++) {
            const unsigned body = c->active_bodies.indices[i];
            if (c->bodies.filters[body].flags&BF_IS_FRUSTUM_CULLED) continue;
            const BodyInfo* info = &c->bodies.infos[body];
            const float* aabb_hextents = info->aabb_half_extents;   // 3-floats array
            const float* aabb_center = info->aabb_center;   // 3-floats array (note that here we don't have to strip info->comOffset: we should when using calculate_graphic_transform_for_body(...) to get mMatrix)
            float mMatrix[16];TransformToMat4(mMatrix,&c->bodies.transforms[body]);

            glPushMatrix();
            glMultMatrixf(mMatrix);
            glTranslatef(aabb_center[0],aabb_center[1],aabb_center[2]); // optional
            glScalef(aabb_hextents[0],aabb_hextents[1],aabb_hextents[2]);
            drawShape(SHAPE_AABB);
            glPopMatrix();
        }
        glEnable(GL_LIGHTING);
        glprintf("Active_bodies:%u\n",c->active_bodies.count);
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
        bind_body(c,body,SHAPE_BOX,COLOR_NONE);
    }
    else {
        const float radius = 0.25f*((float)rand() * (1.0f/(float)RAND_MAX) + 0.5f);
        for (int l=0;l<3;l++)   {
            mMatrixThrow[12+l]+=mMatrixThrow[8+l]*((radius*2.f)*1.5f+globals.proj.pMatrixNearPlane);   // advance position a bit to avoid clipping sphere graphic at lauch
        }
        const float mass = 8.18879f*radius*radius*radius;
        body = add_sphere(c,mass,radius,mMatrixThrow,comOffset);
        bind_body(c,body,SHAPE_SPHERE,COLOR_NONE);
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
        if ((key->pressed&KEY_H)) globals.gui.menu_idx=(globals.gui.menu_idx==1)?0:1;
        else if ((key->pressed&KEY_M)) globals.gui.menu_idx=(globals.gui.menu_idx==2)?0:2;
        else if (key->pressed&(KEY_ESC)) {
            if (globals.gui.menu_idx!=0) globals.gui.menu_idx=0;    // exit menu
            else {DestroyPhysics();glutSwapBuffers();DestroyGL();exit(0);}    // exit program (original glut needs exit(...))
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
            return;
        }
        if (key->pressed&KEY_SPACE) globals.use_graphic_transform = !globals.use_graphic_transform; /* switch draw mode */
        if (key->pressed&KEY_F) globals.use_frustum_culling = !globals.use_frustum_culling; /* toggle frustum culling */
        if (key->pressed&(KEY_F5|KEY_F7)) {
            // quick save/load simulation
            const char* filename = "example02.sav"; // ascii only please
            const bool save = (key->pressed&KEY_F5);
            FILE* f=fopen(filename,save?"wb":"rb");
            if (f)  {
                if (save) {
                    nudge::save_context(f,c);
                    // we append some fields in the 'globals' struct
                    fwrite(&globals.selection,sizeof(globals_t::selection_t),1,f);
                    fwrite(&globals.bodies,sizeof(globals_t::bodies_t),1,f);
                    fprintf(f,"\nuse_graphic_transform: %u",globals.use_graphic_transform);
                    fprintf(f,"\nuse_character_camera: %u",globals.use_character_camera);
                    fprintf(f,"\nuse_frustum_culling: %u",globals.use_frustum_culling);
                    fwrite(&globals.gui,sizeof(globals.gui),1,f);
                    fwrite(&globals.camera[0],sizeof(globals.camera[0]),2,f);
                    nudge::log("Saved \"%s\"\n",filename);
                }
                else {
                    nudge::load_context(f,c);
                    // we append some fields in the 'globals' struct
                    int cnt=fread(&globals.selection,sizeof(globals_t::selection_t),1,f);assert(cnt==1);
                    cnt=fread(&globals.bodies,sizeof(globals_t::bodies_t),1,f);assert(cnt==1);
                    cnt=fscanf(f,"\nuse_graphic_transform: %u",&globals.use_graphic_transform);assert(cnt==1);
                    cnt=fscanf(f,"\nuse_character_camera: %u",&globals.use_character_camera);assert(cnt==1);
                    cnt=fscanf(f,"\nuse_frustum_culling: %u",&globals.use_frustum_culling);assert(cnt==1);
                    cnt = fread(&globals.gui,sizeof(globals.gui),1,f);assert(cnt==1);
                    cnt = fread(&globals.camera[0],sizeof(globals.camera[0]),2,f);assert(cnt==2);
                    nudge::log("Loaded \"%s\"\n",filename);
                }
                fclose(f);
            }
            else {
                if (save) nudge::log("Error: can't create file \"%s\"\n",filename);
                else nudge::log("Error: can't find file \"%s\"\n",filename);
            }
            nudge::flush(); // flushes nudge::log(...)
        }
        if (key->pressed&KEY_MOUSE_BUTTON_LEFT) ThrowBodyAtMousePos(key->mouseX,key->mouseY);  /* add a body and throw it */
        if (key->pressed&(KEY_MOUSE_BUTTON_MIDDLE|KEY_ENTER|KEY_END)) globals.use_character_camera=!globals.use_character_camera;
        if (key->pressed&KEY_MOUSE_BUTTON_RIGHT) globals.selection.body = glGetNudgeBodyAtMouseCoords(c,key->mouseX,key->mouseY,&globals.selection.box_relative_index,&globals.selection.sphere_relative_index);
    }
    //else if (key->down&KEY_SHIFT) {if (key->pressed&KEY_SPACE) {const float* p=c->bodies.transforms[globals.bodies.character_body].p;nudge::log("%1.2f,%1.2f,%1.2f,\n",p[0],p[1]+0.65f,p[2]);nudge::flush();}}

    // camera movement keys
    if (globals.gui.menu_idx!=2)    {
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
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
#   define USE_SPECULAR_LIGHTING
#   ifdef USE_SPECULAR_LIGHTING
    const GLfloat material_specular[4] = {0.5f,0.5f,0.5f,1.0f};
    glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,material_specular);
    glMateriali(GL_FRONT_AND_BACK,GL_SHININESS,24); // in [0-128]
#   endif
    glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);    // now glColor(...) calls set material_ambient and material_diffuse
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.5f, 0.7f, 1.0f, 1.0f);
    GLfloat light_ambient[] = { 0.1f, 0.1f, 0.1f, 1.0f };
    GLfloat light_diffuse[] = { 0.75f, 0.75f, 0.75f, 1.0f };
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
#   ifdef USE_SPECULAR_LIGHTING
    GLfloat light_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
#   endif

    compileDisplayLists();
}
void DestroyGL() {for (int i=0;i<SHAPE_COUNT;i++) glDeleteLists(globals.display_lists[i],1);}
void ResizeGL(int w,int h) {
	// projection Matrix
	if (h>0) {
		globals_t::proj_t* p = &globals.proj;
        glViewport(0,0,w,h);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glPerspective(p->pMatrixFovDeg,(float)w/(float)h,p->pMatrixNearPlane,p->pMatrixFarPlane);
		glMatrixMode(GL_MODELVIEW);
	}
}

void HandleMenus(void);void glprintf_flush(void);void updateCameraPos(void); // froward declarations of internal functions
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

    // internal function
    HandleMenus();

    // simulation
    UpdatePhysics(elapsed_time);

	// drawing
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// view Matrix
    glLoadIdentity();
    updateCameraPos();const globals_t::camera_t* cam = &globals.camera[globals.use_character_camera];
    glLookAt(cam->cameraPos[0],cam->cameraPos[1],cam->cameraPos[2],cam->targetPos[0],cam->targetPos[1],cam->targetPos[2],0,1,0);

	// Setup light.
    nudge::nm_Vec3Normalize(globals.lightDirection);
    glLightf(GL_LIGHT0, GL_POSITION,globals.lightDirection[0]);

    // Draws all the physic bodies
    DrawPhysics();
	
    // We can add a pivot at the camera target point
    if (!globals.use_character_camera)   {
        static float mMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
        mMatrix[12]=cam->targetPos[0];mMatrix[13]=cam->targetPos[1];mMatrix[14]=cam->targetPos[2];
        // we complicate things by adding half of the code to just fade the pivot in the distance... good choice doing it in a nudge example!
        if (cam->cameraDistance<40.f)   {
            float fading_alpha = cam->cameraDistance>5.f?sqrtf(sqrtf(cam->cameraDistance-4.f)):1.f;fading_alpha=1.f/(fading_alpha);
            if (fading_alpha>0.01f) {
                glDisable(GL_LIGHTING);
                glPushMatrix();glMultMatrixf(mMatrix);glScalef(1.5f,1.5f,1.5f);glEnable(GL_BLEND);
                //glColor4f(1,1,1,fading_effect);drawShape(SHAPE_PIVOT3D); // nope: we can't use blending, because SHAPE_PIVOT3D in its display list has hard-coded opaque glColor3f, we must split the call
                glColor4f(1,1,1,fading_alpha);drawShape(SHAPE_PIVOT3D_CENTER);
                glColor4f(1,0,0,fading_alpha);drawShape(SHAPE_PIVOT3D_AXIS_X);
                glColor4f(0,1,0,fading_alpha);drawShape(SHAPE_PIVOT3D_AXIS_Y);
                glColor4f(0,0,1,fading_alpha);drawShape(SHAPE_PIVOT3D_AXIS_Z);
                glDisable(GL_BLEND);glPopMatrix();
                glEnable(GL_LIGHTING);
            }
        }
    }

    // key state resetting
    globals.key.down&=~KEY_MOUSE_WHEEL_UP_OR_DOWN;    // we must reset the two mouse wheel flags, because otherwise it does not work
    globals.key.down_last_frame=globals.key.down;


    // internal function (once per frame)
    glprintf_flush();
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
    assert(sizeof(Keys)/sizeof(Keys[0])==KEY_NUM_REGULAR_KEYS);
    assert(sizeof(SpecialKeys)/sizeof(SpecialKeys[0])==KEY_NUM_SPECIAL_KEYS);
    assert(sizeof(ModifierKeys)/sizeof(ModifierKeys[0])==KEY_NUM_MODIFIER_KEYS);
    assert(sizeof(MouseKeys)/sizeof(MouseKeys[0])==KEY_NUM_MOUSE_KEYS);

    nudge::show_info();
		
	// Start GLUT.
	glutInit(&argc, const_cast<char**>(argv));
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(1024, 600);
    glutCreateWindow("nudge example02");
    glutDisplayFunc(GlutFakeDrawGL);
    glutIdleFunc(GlutIdle);
    glutReshapeFunc(ResizeGL);
    glutMouseFunc(GlutMouse);
    glutKeyboardFunc(GlutKeys);
    glutKeyboardUpFunc(GlutKeysUp);   // GLUT Version>=4 or freeglut please
    glutSpecialFunc(GlutSpecialKeys);
    glutSpecialUpFunc(GlutSpecialKeysUp);   // GLUT Version>=4 or freeglut please
    glutSetKeyRepeat(GLUT_KEY_REPEAT_OFF); // Nate Robbins' port of GLUT to win32 did not implement glutSetKeyRepeat


    InitPhysics();

	InitGL();
	
	glutMainLoop();
		    
	DestroyGL();

    DestroyPhysics();
		
	
	return 0;
}


// internal functions here

void nm_Mat4Identity(float* m) {memset(m,0,16*sizeof(float));m[0]=m[5]=m[10]=m[15]=1.f;}
struct gl_matrices_t {
    float vMatrix[16],pMatrix[16],vpMatrix[16];  // these have been explicitely kept to calculate other quantities
    float vpMatrixInv[16];  // this is necessary for throwing/selecting bodies
    float frustumPlanes[6][4];      // this is necessary for frustum culling
} glmatrices = {};

void glPerspective(float degfovy,float aspect, float zNear, float zFar) {
    // custom replacement of gluPerspective(...)
    float* res=glmatrices.pMatrix;
    const float eps = 0.0001f;
    float f = 1.f/tan(degfovy*1.5707963268f/180.0); //cotg
    float Dfn = (zFar-zNear);
    if (Dfn==0) {zFar+=eps;zNear-=eps;Dfn=zFar-zNear;}
    if (aspect==0) aspect = 1.f;

    res[0]  = f/aspect;
    res[1]  = 0;
    res[2]  = 0;
    res[3]  = 0;

    res[4]  = 0;
    res[5]  = f;
    res[6]  = 0;
    res[7] = 0;

    res[8]  = 0;
    res[9]  = 0;
    res[10] = -(zFar+zNear)/Dfn;
    res[11] = -1;

    res[12]  = 0;
    res[13]  = 0;
    res[14] = -2.f*zFar*zNear/Dfn;
    res[15] = 0;

    glLoadMatrixf(res);
}
int nm_Mat4Invert(float* mOut16,const float* m16)	{
    const float* m = m16;
    float* n = mOut16;

    float m00 = m[0],  m10 = m[1],  m20 = m[2],  m30 = m[3];
    float m01 = m[4],  m11 = m[5],  m21 = m[6],  m31 = m[7];
    float m02 = m[8],  m12 = m[9],  m22 = m[10], m32 = m[11];
    float m03 = m[12], m13 = m[13], m23 = m[14], m33 = m[15];

    float v0 = m20 * m31 - m21 * m30;
    float v1 = m20 * m32 - m22 * m30;
    float v2 = m20 * m33 - m23 * m30;
    float v3 = m21 * m32 - m22 * m31;
    float v4 = m21 * m33 - m23 * m31;
    float v5 = m22 * m33 - m23 * m32;

    float t00 = + (v5 * m11 - v4 * m12 + v3 * m13);
    float t10 = - (v5 * m10 - v2 * m12 + v1 * m13);
    float t20 = + (v4 * m10 - v2 * m11 + v0 * m13);
    float t30 = - (v3 * m10 - v1 * m11 + v0 * m12);

    float det = (t00 * m00 + t10 * m01 + t20 * m02 + t30 * m03);
    if (det==(float)0) return 0;
    {
        float invDet = 1 / det;

        float d00 = t00 * invDet;
        float d10 = t10 * invDet;
        float d20 = t20 * invDet;
        float d30 = t30 * invDet;

        float d01 = - (v5 * m01 - v4 * m02 + v3 * m03) * invDet;
        float d11 = + (v5 * m00 - v2 * m02 + v1 * m03) * invDet;
        float d21 = - (v4 * m00 - v2 * m01 + v0 * m03) * invDet;
        float d31 = + (v3 * m00 - v1 * m01 + v0 * m02) * invDet;

        v0 = m10 * m31 - m11 * m30;
        v1 = m10 * m32 - m12 * m30;
        v2 = m10 * m33 - m13 * m30;
        v3 = m11 * m32 - m12 * m31;
        v4 = m11 * m33 - m13 * m31;
        v5 = m12 * m33 - m13 * m32;
        {
            float d02 = + (v5 * m01 - v4 * m02 + v3 * m03) * invDet;
            float d12 = - (v5 * m00 - v2 * m02 + v1 * m03) * invDet;
            float d22 = + (v4 * m00 - v2 * m01 + v0 * m03) * invDet;
            float d32 = - (v3 * m00 - v1 * m01 + v0 * m02) * invDet;

            v0 = m21 * m10 - m20 * m11;
            v1 = m22 * m10 - m20 * m12;
            v2 = m23 * m10 - m20 * m13;
            v3 = m22 * m11 - m21 * m12;
            v4 = m23 * m11 - m21 * m13;
            v5 = m23 * m12 - m22 * m13;
            {
                float d03 = - (v5 * m01 - v4 * m02 + v3 * m03) * invDet;
                float d13 = + (v5 * m00 - v2 * m02 + v1 * m03) * invDet;
                float d23 = - (v4 * m00 - v2 * m01 + v0 * m03) * invDet;
                float d33 = + (v3 * m00 - v1 * m01 + v0 * m02) * invDet;

                n[0] =d00; n[1] =d10; n[2] =d20; n[3] =d30;
                n[4] =d01; n[5] =d11; n[6] =d21; n[7] =d31;
                n[8] =d02; n[9] =d12; n[10]=d22; n[11]=d32;
                n[12]=d03; n[13]=d13; n[14]=d23; n[15]=d33;
            }
        }
    }
    return 1;
}
void glGetFrustumPlaneEquations(float planeEquationsOut[6][4],const float* __restrict vpMatrix16/*,int normalizePlanes*/)  {
    // ax+by+cz+d=0 [xl,xr,yb,yt,zn,zf],normalizePlanes=0 -> no normalization
    float m00 = vpMatrix16[0], m01 = vpMatrix16[4], m02 = vpMatrix16[8],  m03 = vpMatrix16[12];
    float m10 = vpMatrix16[1], m11 = vpMatrix16[5], m12 = vpMatrix16[9],  m13 = vpMatrix16[13];
    float m20 = vpMatrix16[2], m21 = vpMatrix16[6], m22 = vpMatrix16[10], m23 = vpMatrix16[14];
    float m30 = vpMatrix16[3], m31 = vpMatrix16[7], m32 = vpMatrix16[11], m33 = vpMatrix16[15];
    float* p = NULL;
    p = &planeEquationsOut[0][0];   // Left
    p[0] = m30+m00; p[1] = m31+m01; p[2] = m32+m02; p[3] = m33+m03;
    p = &planeEquationsOut[1][0];   // Right
    p[0] = m30-m00; p[1] = m31-m01; p[2] = m32-m02; p[3] = m33-m03;
    p = &planeEquationsOut[2][0];   // Bottom
    p[0] = m30+m10; p[1] = m31+m11; p[2] = m32+m12; p[3] = m33+m13;
    p = &planeEquationsOut[3][0];   // Top
    p[0] = m30-m10; p[1] = m31-m11; p[2] = m32-m12; p[3] = m33-m13;
    p = &planeEquationsOut[4][0];   // Near
    p[0] = m30+m20; p[1] = m31+m21; p[2] = m32+m22; p[3] = m33+m23;
    p = &planeEquationsOut[5][0];   // Far
    p[0] = m30-m20; p[1] = m31-m21; p[2] = m32-m22; p[3] = m33-m23;
    //if (normalizePlanes) {int i;for (i=0;i<6;i++) IMPlaneNormalize(&planeEquationsOut[i][0]);}
}
void glLookAt(float eyeX, float eyeY, float eyeZ, float centerX, float centerY, float centerZ, float upX, float upY, float upZ)    {
    // custom replacement of gluLookAt(...)
    float* m=glmatrices.vMatrix;
    const float eps = 0.0001f;

    float F[3] = {eyeX-centerX,eyeY-centerY,eyeZ-centerZ};
    float length = F[0]*F[0]+F[1]*F[1]+F[2]*F[2];	// length2 now
    float up[3] = {upX,upY,upZ};

    float S[3] = {up[1]*F[2]-up[2]*F[1],up[2]*F[0]-up[0]*F[2],up[0]*F[1]-up[1]*F[0]};
    float U[3] = {F[1]*S[2]-F[2]*S[1],F[2]*S[0]-F[0]*S[2],F[0]*S[1]-F[1]*S[0]};

    if (length==0) length = eps;
    length = sqrt(length);
    F[0]/=length;F[1]/=length;F[2]/=length;

    length = S[0]*S[0]+S[1]*S[1]+S[2]*S[2];if (length==0) length = eps;
    length = sqrt(length);
    S[0]/=length;S[1]/=length;S[2]/=length;

    length = U[0]*U[0]+U[1]*U[1]+U[2]*U[2];if (length==0) length = eps;
    length = sqrt(length);
    U[0]/=length;U[1]/=length;U[2]/=length;

    m[0] = S[0];
    m[1] = U[0];
    m[2] = F[0];
    m[3]= 0;

    m[4] = S[1];
    m[5] = U[1];
    m[6] = F[1];
    m[7]= 0;

    m[8] = S[2];
    m[9] = U[2];
    m[10]= F[2];
    m[11]= 0;

    m[12] = -S[0]*eyeX -S[1]*eyeY -S[2]*eyeZ;
    m[13] = -U[0]*eyeX -U[1]*eyeY -U[2]*eyeZ;
    m[14]= -F[0]*eyeX -F[1]*eyeY -F[2]*eyeZ;
    m[15]= 1;

    glLoadMatrixf(m);

    nudge::nm_Mat4Mul(glmatrices.vpMatrix,glmatrices.pMatrix,glmatrices.vMatrix);
    nm_Mat4Invert(glmatrices.vpMatrixInv,glmatrices.vpMatrix);
    glGetFrustumPlaneEquations(glmatrices.frustumPlanes,glmatrices.vpMatrix);
}
int nm_Mat4UnProjectMvpInv(float winX,float winY,float winZ,const float* __restrict mvpMatrixInv16,const int* viewport4,float* __restrict objX,float* __restrict objY,float* __restrict objZ)    {
    const float *invpm = mvpMatrixInv16;
    const float v[4] = {2*(winX-(float)viewport4[0])/(float)viewport4[2]-1,2*(winY-(float)viewport4[1])/(float)viewport4[3]-1,2*winZ-1,1};
    float objW = 0;
    *objX =  v[0]*invpm[0] + v[1]*invpm[4] + v[2]*invpm[8]  + v[3]*invpm[12];
    *objY =  v[0]*invpm[1] + v[1]*invpm[5] + v[2]*invpm[9]  + v[3]*invpm[13];
    *objZ =  v[0]*invpm[2] + v[1]*invpm[6] + v[2]*invpm[10] + v[3]*invpm[14];
    objW =  v[0]*invpm[3] + v[1]*invpm[7] + v[2]*invpm[11] + v[3]*invpm[15];
    if (objW!=0 && objW!=1) {(*objX)/=objW;(*objY)/=objW;(*objZ)/=objW;}
    return 1;
}
void nm_Mat4UnProjectMouseCoords(float* __restrict rayOriginOut3,float* __restrict rayDirOut3,int mouseX,int mouseY,const float* __restrict vpMatrixInv,const int* viewport4)   {
    float rayOrigin[3] = {0,0,0};
    float rayDir[3] = {0,0,-1};
    int i;
    nm_Mat4UnProjectMvpInv(mouseX,viewport4[3]-mouseY-1,0.0,vpMatrixInv,viewport4,&rayOrigin[0],&rayOrigin[1],&rayOrigin[2]);
    nm_Mat4UnProjectMvpInv(mouseX,viewport4[3]-mouseY-1,1.0,vpMatrixInv,viewport4,&rayDir[0],&rayDir[1],&rayDir[2]);
    for (i=0;i<3;i++) rayDir[i]-=rayOrigin[i];
    nudge::nm_Vec3Normalize(rayDir);
    for (i=0;i<3;i++) {
        if (rayDirOut3) rayDirOut3[i] = rayDir[i];
        if (rayOriginOut3) rayOriginOut3[i] = rayOrigin[i];
    }
}
void glGetThrowBodyMatrixAtMouseCoordsfv(GLfloat* mMatrixOut,int mouseX,int mouseY) {
    GLfloat *mat=mMatrixOut;const float yAxis[3]={0,1,0};
    GLint viewport[4];glGetIntegerv(GL_VIEWPORT,viewport);
    nm_Mat4UnProjectMouseCoords(&mat[12],&mat[8],mouseX,mouseY,glmatrices.vpMatrixInv,viewport); // gluUnProject4
    nudge::nm_Vec3Cross(&mat[0],yAxis,&mat[8]);
    nudge::nm_Vec3Cross(&mat[4],&mat[8],&mat[0]);
    mat[3]=mat[7]=mat[11]=0.f;mat[15]=1.f;
}
inline void glGetNudgeBodyAtMouseCoordsFromRayStep(const float* __restrict rayOrigin3,const float* __restrict rayDir3,const nudge::Transform* T,const float* __restrict aabbMin,const float* __restrict aabbMax,
                                            int* noCollisionDetected,float* tMin) {
    using namespace nudge;
    float tMax = (float)1000000000000;
    float obbPosDelta[3] =  {T->p[0]-rayOrigin3[0],T->p[1]-rayOrigin3[1],T->p[2]-rayOrigin3[2]};
    float obbMatrix[9];nm_Mat3FromQuat(obbMatrix,T->q);  // costly! Better change this for sure! [float 4x4 matrices are far better to keep than pos+quat (except for slerp), if you're not relying to a framework that does the dirty work for you... we should definitely store all float16 transforms somewhere...]
    *noCollisionDetected=0;*tMin = 0;
    for (int j=0;j<3;j++)   {
        if (!*noCollisionDetected)   {
            const int j3 = 3*j; const float axis[3] = {obbMatrix[j3],obbMatrix[j3+1],obbMatrix[j3+2]};
            const float e = nm_Vec3Dot(axis, obbPosDelta), f = nm_Vec3Dot(rayDir3, axis);
            float t1 = (e+aabbMin[j])/f, t2 = (e+aabbMax[j])/f;
            if (t1>t2)  {float w=t1;t1=t2;t2=w;}
            if (t2 < tMax)    tMax = t2;
            if (t1 > *tMin)    *tMin = t1;
            if (*tMin > tMax) *noCollisionDetected=1;
        }
    }
}
uint16_t glGetNudgeBodyAtMouseCoordsFromRay(nudge::context_t* c,const float* __restrict rayOrigin3,const float* __restrict rayDir3,int16_t* pRelativeBoxColliderIndexOut,int16_t* pRelativeSphereColliderIndexOut,float* pOptionalDistanceOut) {
    // rayOrigin3 and rayDirection3 are in world space
    using namespace nudge;
    uint16_t rv = NUDGE_INVALID_BODY_ID;if (pRelativeBoxColliderIndexOut) *pRelativeBoxColliderIndexOut=-1;if (pRelativeSphereColliderIndexOut) *pRelativeSphereColliderIndexOut=-1;
    float intersection_distance = 0;
    if (pOptionalDistanceOut) *pOptionalDistanceOut=intersection_distance;
    if (!c) return rv;

    // Loop all meshes and find OBB vs ray intersection
    // Code based on: http://www.opengl-tutorial.org/miscellaneous/clicking-on-objects/picking-with-custom-ray-obb-function/ (WTFPL Public Licence)
    for (unsigned body=0;body<c->bodies.count;body++)   {
        const BodyFilter* filter = &c->bodies.filters[body];
        if (filter->flags&(BF_IS_DISABLED_OR_REMOVED|BF_IS_FRUSTUM_CULLED)) continue;

        const Transform* mainT = &c->bodies.transforms[body];
        const BodyInfo* info = &c->bodies.infos[body];
        const float aabbMin[3] = {info->aabb_center[0]-info->aabb_half_extents[0],info->aabb_center[1]-info->aabb_half_extents[1],info->aabb_center[2]-info->aabb_half_extents[2]};
        const float aabbMax[3] = {info->aabb_center[0]+info->aabb_half_extents[0],info->aabb_center[1]+info->aabb_half_extents[1],info->aabb_center[2]+info->aabb_half_extents[2]};

        int noCollisionDetected = 0;float tMin = 0;

        glGetNudgeBodyAtMouseCoordsFromRayStep(rayOrigin3,rayDir3,mainT,aabbMin,aabbMax,&noCollisionDetected,&tMin);

        if (!noCollisionDetected && (intersection_distance<=0 || intersection_distance>tMin))   {
            //intersection_distance = tMin;rv = body;  // nope, we add an inner per-collider scan to avoid gaps and return inner collisions too
            // otherwise for simple per-body aabb collisions we should enable line above and comment out the rest of this scope

            const BodyLayout* L = &c->bodies.layouts[body];
            if (L->num_boxes>0) {
                assert(L->first_box_index>=0 && (uint16_t)L->first_box_index+L->num_boxes<=c->colliders.boxes.count);
                const Transform* T = &c->colliders.boxes.transforms[L->first_box_index];
                const BoxCollider* C = &c->colliders.boxes.data[L->first_box_index];
                for (uint16_t i=0;i<L->num_boxes;i++) {
                    const float* he = C[i].size;
                    const float aabbMin[3]={-he[0],-he[1],-he[2]},aabbMax[3]={he[0],he[1],he[2]};
                    Transform t = TransformMul(*mainT,T[i]);
                    int noCollisionDetected = 0;float tMin = 0;
                    glGetNudgeBodyAtMouseCoordsFromRayStep(rayOrigin3,rayDir3,&t,aabbMin,aabbMax,&noCollisionDetected,&tMin);
                    if (!noCollisionDetected && (intersection_distance<=0 || intersection_distance>tMin))   {
                        intersection_distance=tMin;rv=body;
                        if (pRelativeBoxColliderIndexOut) *pRelativeBoxColliderIndexOut=(int16_t)i;
                        if (pRelativeSphereColliderIndexOut) *pRelativeSphereColliderIndexOut=-1;
                    }
                }
            }
            if (L->num_spheres>0) {
                assert(L->first_sphere_index>=0 && (uint16_t)L->first_sphere_index+L->num_spheres<=c->colliders.spheres.count);
                const Transform* T = &c->colliders.spheres.transforms[L->first_sphere_index];
                const SphereCollider* C = &c->colliders.spheres.data[L->first_sphere_index];
                for (uint16_t i=0;i<L->num_spheres;i++) {
                    const float r = C[i].radius, aabbMin[3]={-r,-r,-r},aabbMax[3]={r,r,r};
                    Transform t = TransformMul(*mainT,T[i]);
                    int noCollisionDetected = 0;float tMin = 0;
                    glGetNudgeBodyAtMouseCoordsFromRayStep(rayOrigin3,rayDir3,&t,aabbMin,aabbMax,&noCollisionDetected,&tMin);
                    if (!noCollisionDetected && (intersection_distance<=0 || intersection_distance>tMin))   {
                        intersection_distance=tMin;rv=body;
                        if (pRelativeBoxColliderIndexOut) *pRelativeBoxColliderIndexOut=-1;
                        if (pRelativeSphereColliderIndexOut) *pRelativeSphereColliderIndexOut=(int16_t)i;
                    }
                }
            }
        }
    }

    if (pOptionalDistanceOut) *pOptionalDistanceOut=intersection_distance;
    return rv;
}
uint16_t glGetNudgeBodyAtMouseCoords(nudge::context_t* c,int mouseX,int mouseY,int16_t* pRelativeBoxColliderIndexOut,int16_t* pRelativeSphereColliderIndexOut,float* pOptionalDistanceOut) {
    assert(c);
    float rayOrigin[3] = {0,0,0},rayDir[3] = {0,0,-1};
    GLint viewport[4];glGetIntegerv(GL_VIEWPORT,viewport);
    nm_Mat4UnProjectMouseCoords(rayOrigin,rayDir,mouseX,mouseY,glmatrices.vpMatrixInv,viewport);  // gluUnProject4
    return glGetNudgeBodyAtMouseCoordsFromRay(c,rayOrigin,rayDir,pRelativeBoxColliderIndexOut,pRelativeSphereColliderIndexOut,pOptionalDistanceOut);
}

int glIsAabbVisible(const float* __restrict mMatrix16,float aabbCenterX,float aabbCenterY,float aabbCenterZ,float aabbHalfExtentX,float aabbHalfExtentY,float aabbHalfExtentZ) {
    int i;const float zero = (float)0;
    // Start OBB => AABB transformation based on: http://dev.theomader.com/transform-bounding-boxes/
    float aabb[6];const float* m = mMatrix16;
    const float c[3] = {aabbCenterX,aabbCenterY,aabbCenterZ};
    const float he[3]= {aabbHalfExtentX,aabbHalfExtentY,aabbHalfExtentZ};
    for (i=0;i<3;i++)   {
        const float hd=fabsf(m[i]*he[0])+fabsf(m[4+i]*he[1])+fabsf(m[8+i]*he[2]);
        const float ct=m[i]*c[0]+m[4+i]*c[1]+m[8+i]*c[2]+m[12+i];aabb[i]=ct-hd;aabb[3+i]=ct+hd;
    }
    // Tip: From now on 'aabb' must be constant
    // End OBB => AABB transformation based on: http://dev.theomader.com/transform-bounding-boxes/
    // FAST VERSION
    {
        for(i=0; i < 6; i++) {
            const float *pl = &glmatrices.frustumPlanes[i][0];
            const int p[3] = {3*(int)(pl[0]>zero),3*(int)(pl[1]>zero),3*(int)(pl[2]>zero)};   // p[j] = 0 or 3
            const float dp = pl[0]*aabb[p[0]] + pl[1]*aabb[p[1]+1] + pl[2]*aabb[p[2]+2] + pl[3];
            if (dp < 0) return 0;
        }
    }
    // note that we still have a lot of false positives
    return 1;
}
int glIsAabbVisible(const float* __restrict Tpos3,float aabbEnlargedRadius) {
    // Note that we cannot simply add an aabbCenter to Tpos3 and use a simple aabbRadius (based on aabbHalfExtents):
    // But we can enlarge the aabbRadius so that: aabbEnlargedRadius = aabbRadius + distance(Tpos3,aabbCenter);
    int i;const float zero = (float)0;
    const float aabb[6] = {Tpos3[0]-aabbEnlargedRadius,Tpos3[1]-aabbEnlargedRadius,Tpos3[2]-aabbEnlargedRadius,Tpos3[0]+aabbEnlargedRadius,Tpos3[1]+aabbEnlargedRadius,Tpos3[2]+aabbEnlargedRadius};
    for(i=0; i < 6; i++) {
        const float *pl = &glmatrices.frustumPlanes[i][0];
        const int p[3] = {3*(int)(pl[0]>zero),3*(int)(pl[1]>zero),3*(int)(pl[2]>zero)};   // p[j] = 0 or 3
        const float dp = pl[0]*aabb[p[0]] + pl[1]*aabb[p[1]+1] + pl[2]*aabb[p[2]+2] + pl[3];
        if (dp < 0) return 0;
    }
    return 1;
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

void drawShape(ShapeEnum shape) {
	assert(shape<=SHAPE_COUNT);
	const GLuint dl = globals.display_lists[shape];assert(dl);
	glCallList(dl);
}

void glDrawCylinder(float radiusTop,float radiusBottom,float height,int slices=8,int axis=1,bool center_in_height=true,uint lateralSurfaceMask=3,
                    float deltaRadiusTopX=0,float deltaRadiusTopY=0,float deltaRadiusTopZ=0,
                    float deltaRadiusBottomX=0,float deltaRadiusBottomY=0,float deltaRadiusBottomZ=0) {
    // glutSolidCylinder(radius,height,slices,1) [available only in freeglut.h, not in glut.h]
    // should be roughly equivalent to: glDrawCylinder(radius,height,slices,2,false); [TO CHECK]
    const float angle = M_PI*2.f;
    const float R=radiusTop,r=radiusBottom,h=height/2,delta_angle=angle/slices,a_start=delta_angle*0.5f;
    const float hlow = center_in_height ? -h : 0, hhigh = center_in_height ? h : height;
    const float DR[3] = {deltaRadiusTopX,deltaRadiusTopY,deltaRadiusTopZ};
    const float dr[3] = {deltaRadiusBottomX,deltaRadiusBottomY,deltaRadiusBottomZ};

    glBegin(GL_QUAD_STRIP);
    float a=a_start;
    for (int i=0;i<slices+1;i++)  {
        const float c = cos(a), s = sin(a), rc = r*c, rs = r*s, Rc = R*c, Rs = R*s;
        if (axis==0)      {glNormal3f(0,c,s);glVertex3f(DR[0]+hhigh,DR[1]+Rc,DR[2]+Rs);glVertex3f(dr[0]+hlow,dr[1]+rc,dr[2]+rs);a+=delta_angle;}
        else if (axis==2) {glNormal3f(c,s,0);glVertex3f(DR[0]+Rc,DR[1]+Rs,DR[2]+hhigh);glVertex3f(dr[0]+rc,dr[1]+rs,dr[2]+hlow);a+=delta_angle;}
        else /*axis==11*/ {glNormal3f(c,0,s);glVertex3f(DR[0]+Rc,DR[1]+hhigh,DR[2]+Rs);glVertex3f(dr[0]+rc,dr[1]+hlow,dr[2]+rs);a-=delta_angle;}
        if (i==slices-1) a=a_start;
    }
    glEnd();
    if (lateralSurfaceMask&(1<<1U) && R!=0.f) {
        glBegin(GL_TRIANGLE_FAN);
        if (axis==0)      {glNormal3f(1,0,0);glVertex3f(DR[0]+hhigh,DR[1],DR[2]);}
        else if (axis==2) {glNormal3f(0,0,1);glVertex3f(DR[0],DR[1],DR[2]+hhigh);}
        else /*axis==1*/  {glNormal3f(0,1,0);glVertex3f(DR[0],DR[1]+hhigh,DR[2]);}
        a=a_start;
        for (int i=0;i<slices+1;i++)  {
            if (axis==0)      {glVertex3f(DR[0]+hhigh,DR[1]+R*cos(a),DR[2]+R*sin(a));a+=delta_angle;}
            else if (axis==2) {glVertex3f(DR[0]+R*cos(a),DR[1]+R*sin(a),DR[2]+hhigh);a+=delta_angle;}
            else /*axis==1*/  {glVertex3f(DR[0]+R*cos(a),DR[1]+hhigh,DR[2]+R*sin(a));a-=delta_angle;}
            if (i==slices-1) a=a_start;
        }
        glEnd();
    }
    if (lateralSurfaceMask&(1<<0U) && r!=0.f) {
        glBegin(GL_TRIANGLE_FAN);
        if (axis==0)      {glNormal3f(-1,0,0);glVertex3f(dr[0]+hlow,dr[1],dr[2]);}
        else if (axis==2) {glNormal3f(0,0,-1);glVertex3f(dr[0],dr[1],dr[2]+hlow);}
        else /*axis==1*/  {glNormal3f(0,-1,0);glVertex3f(dr[0],dr[1]+hlow,dr[2]);}
        a=a_start;
        for (int i=0;i<slices+1;i++)  {
            if (axis==0)      {glVertex3f(dr[0]+hlow,dr[1]+r*cos(a),dr[2]+r*sin(a));a-=delta_angle;}
            else if (axis==2) {glVertex3f(dr[0]+r*cos(a),dr[1]+r*sin(a),dr[2]+hlow);a-=delta_angle;}
            else /*axis==1*/  {glVertex3f(dr[0]+r*cos(a),dr[1]+hlow,dr[2]+r*sin(a));a+=delta_angle;}
            if (i==slices-1) a=a_start;
        }
        glEnd();
    }
}

void glDrawCylinderX(float radius,float height,int slices=8, bool center_in_height=true,uint lateralSurfaceMask=3,float deltaRadiusTopX=0,float deltaRadiusTopY=0,float deltaRadiusTopZ=0,float deltaRadiusBottomX=0,float deltaRadiusBottomY=0,float deltaRadiusBottomZ=0)
    {glDrawCylinder(radius,radius,height,slices,0,center_in_height,lateralSurfaceMask,deltaRadiusTopX,deltaRadiusTopY,deltaRadiusTopZ,deltaRadiusBottomX,deltaRadiusBottomY,deltaRadiusBottomZ);}
void glDrawCylinderY(float radius,float height,int slices=8, bool center_in_height=true,uint lateralSurfaceMask=3,float deltaRadiusTopX=0,float deltaRadiusTopY=0,float deltaRadiusTopZ=0,float deltaRadiusBottomX=0,float deltaRadiusBottomY=0,float deltaRadiusBottomZ=0)
    {glDrawCylinder(radius,radius,height,slices,1,center_in_height,lateralSurfaceMask,deltaRadiusTopX,deltaRadiusTopY,deltaRadiusTopZ,deltaRadiusBottomX,deltaRadiusBottomY,deltaRadiusBottomZ);}
void glDrawCylinderZ(float radius,float height,int slices=8, bool center_in_height=true,uint lateralSurfaceMask=3,float deltaRadiusTopX=0,float deltaRadiusTopY=0,float deltaRadiusTopZ=0,float deltaRadiusBottomX=0,float deltaRadiusBottomY=0,float deltaRadiusBottomZ=0)
    {glDrawCylinder(radius,radius,height,slices,2,center_in_height,lateralSurfaceMask,deltaRadiusTopX,deltaRadiusTopY,deltaRadiusTopZ,deltaRadiusBottomX,deltaRadiusBottomY,deltaRadiusBottomZ);}


void glDrawHollowCylinder(float R,float r,float height,int slices=8,int axis=1,bool center_in_height=false) {
    // Experimental
    const float angle=M_PI*2.f;
    const float r_sum=R+r,r_dif=R-r,h=height/2,delta_angle=angle/slices,a_start=delta_angle*0.5f;
    const float hlow = center_in_height ? -h : 0, hhight = center_in_height ? h : height;
    glBegin(GL_QUAD_STRIP);
    float a=a_start;for (int i=0;i<slices+1;i++)  {
        const float c = cos(a), s = sin(a), rc = r_sum*c, rs = r_sum*s;
        if (axis==0)      {glNormal3f(0,c,s);glVertex3f(hhight,rc,rs);glVertex3f(hlow,rc,rs);a+=delta_angle;}
        else if (axis==2) {glNormal3f(c,s,0);glVertex3f(rc,rs,hhight);glVertex3f(rc,rs,hlow);a+=delta_angle;}
        else /*axis==1*/  {glNormal3f(c,0,s);glVertex3f(rc,hhight,rs);glVertex3f(rc,hlow,rs);a-=delta_angle;}
        if (i==slices-1) a=a_start;
    }
    a=a_start;for (int i=0;i<slices+1;i++)  {
        const float c = cos(a), s = sin(a), rc = r_dif*c, rs = r_dif*s;
        if (axis==0)      {glNormal3f(0,-c,-s);glVertex3f(hlow,rc,rs);glVertex3f(hhight,rc,rs);a+=delta_angle;}
        else if (axis==2) {glNormal3f(-c,-s,0);glVertex3f(rc,rs,hlow);glVertex3f(rc,rs,hhight);a+=delta_angle;}
        else /*axis==1*/  {glNormal3f(-c,0,-s);glVertex3f(rc,hlow,rs);glVertex3f(rc,hhight,rs);a-=delta_angle;}
        if (i==slices-1) a=a_start;
    }
    glEnd();
    glBegin(GL_QUAD_STRIP);
    glNormal3f(axis==0?1:0,axis==1?1:0,axis==2?1:0);a=a_start;for (int i=0;i<slices+1;i++)  {
        if (axis==0)        {glVertex3f(hhight,r_dif*cos(a),r_dif*sin(a));glVertex3f(hhight,r_sum*cos(a),r_sum*sin(a));a+=delta_angle;}
        else if (axis==2)   {glVertex3f(r_dif*cos(a),r_dif*sin(a),hhight);glVertex3f(r_sum*cos(a),r_sum*sin(a),hhight);a+=delta_angle;}
        else /*axis==1*/    {glVertex3f(r_dif*cos(a),hhight,r_dif*sin(a));glVertex3f(r_sum*cos(a),hhight,r_sum*sin(a));a-=delta_angle;}
        if (i==slices-1) a=a_start;
    }
    glEnd();
    glBegin(GL_QUAD_STRIP);
    glNormal3f(axis==0?-1:0,axis==1?-1:0,axis==2?-1:0);a=a_start;for (int i=0;i<slices+1;i++)  {
        if (axis==0)        {glVertex3f(hlow,r_sum*cos(a),r_sum*sin(a));glVertex3f(hlow,r_dif*cos(a),r_dif*sin(a));a+=delta_angle;}
        else if (axis==2)   {glVertex3f(r_sum*cos(a),r_sum*sin(a),hlow);glVertex3f(r_dif*cos(a),r_dif*sin(a),hlow);a+=delta_angle;}
        else /*axis==1*/    {glVertex3f(r_sum*cos(a),hlow,r_sum*sin(a));glVertex3f(r_dif*cos(a),hlow,r_dif*sin(a));a-=delta_angle;}
        if (i==slices-1) a=a_start;
    }
    glEnd();
}
void glDrawHollowCylinderX(float R,float r,float height,int slices=8,bool center_in_height=false) {glDrawHollowCylinder(R,r,height,slices,0,center_in_height);}
void glDrawHollowCylinderY(float R,float r,float height,int slices=8,bool center_in_height=false) {glDrawHollowCylinder(R,r,height,slices,1,center_in_height);}
void glDrawHollowCylinderZ(float R,float r,float height,int slices=8,bool center_in_height=false) {glDrawHollowCylinder(R,r,height,slices,2,center_in_height);}

void compileDisplayLists() {
	for (int shape=0;shape<SHAPE_COUNT;shape++) {
	GLuint* pdl = &globals.display_lists[shape];
	if (*pdl) continue;
	*pdl = glGenLists(1);assert(*pdl);
	glNewList(*pdl,GL_COMPILE_AND_EXECUTE);		
	glPushMatrix();
    switch (shape) {
    case SHAPE_BOX: glutSolidCube(2.0f);
        break;
    case SHAPE_SPHERE: glutSolidSphere(1.0f, 16, 8);
        break;
    case SHAPE_SPHERE_LOW_POLY: glutSolidSphere(1.0f, 8, 4);
        break;
    case SHAPE_CYLINDER_Y:
        glDrawCylinderY(1.f, 2.f, 16, true);
        break;
    case SHAPE_CYLINDER_Z: glDrawCylinderZ(1.f, 2.f, 16, true);
        break;
    case SHAPE_HOLLOW_CYLINDER_Y:
        glDrawHollowCylinderY(0.85f,0.15f,2.f,16, true);
        break;
    case SHAPE_HOLLOW_CYLINDER_Z:
        glDrawHollowCylinderZ(0.85f,0.15f,2.f,16, true);
        break;
    case SHAPE_CAPSULE_Y:
        //glRotatef(-90.f,1,0,0);
        glDrawCylinderY(1.f, 2.f,8,true,0);
        break;
    case SHAPE_CAPSULE_Z:
        glDrawCylinderZ(1.f, 2.f,8,true,0);
        break;
    case SHAPE_SKITTLE:
        glScalef(2.f,1.f,2.f);
        glPushMatrix();
        glTranslatef(0,-0.85f,0);
        glDrawCylinder(0.5f,0.25f,0.35f,8,/*axis*/ 1, true,1);
        glPopMatrix();
        glPushMatrix();
        glTranslatef(0,-0.175f,0);
        glDrawCylinderY(0.5f,1.f,8,true,0);
        glPopMatrix();
        glPushMatrix();
        glTranslatef(0,0.5f,0);
        glDrawCylinder(0.1f,0.5f,0.35f,8,/*axis*/ 1,true,2);
        glPopMatrix();
        glPushMatrix();
        glTranslatef(0,0.85f,0);
        glScalef(1.5f,1.f,1.5f);
        glutSolidSphere(0.25f, 8, 4);
        glPopMatrix();
        break;
    case SHAPE_TORUS_Y:
        glRotatef(-90.f,1,0,0);
        glutSolidTorus(
                    0.2,	// GLdouble innerRadius
                    0.8,	// GLdouble outerRadius
                    16,	// GLint nsides
                    16); // GLint rings)
        break;
    case SHAPE_TORUS_Z: glutSolidTorus(
                    0.2,	// GLdouble innerRadius
                    0.8,	// GLdouble outerRadius
                    16,	// GLint nsides
                    16); // GLint rings)
        break;
    case SHAPE_CONE_Y:
        glDrawCylinder(0.f,1.f, 2.f, 16, 1, true, 1);
        break;
    case SHAPE_PRISM_5_Y:
        glRotatef(-(float)(5-2)*180.f*0.5f,0,1,0);glDrawCylinderY(1.f,2.f,5);
        break;
    case SHAPE_PRISM_6_Y:
        glRotatef((float)(6-2)*180.f*0.5f,0,1,0);glDrawCylinderY(1.f,2.f,6);
        break;
    case SHAPE_PRISM_7_Y:
        glRotatef((float)(7-2)*180.f*0.5f,0,1,0);glDrawCylinderY(1.f,2.f,7);
        break;
    case SHAPE_PRISM_8_Y:
        glRotatef((float)(8-2)*180.f*0.5f,0,1,0);glDrawCylinderY(1.f,2.f,8);
        break;
    case SHAPE_TEAPOT:
        glRotatef(-90,0,1,0);
        glScalef(0.8,1.5,1.2);
        glFrontFace(GL_CW);glutSolidTeapot(1.0f);glFrontFace(GL_CCW);    // extremely slow!
        break;
    case SHAPE_CHARACTER: {
        // {0.4f,0.625f,0.1f}
        glScalef(1.f/0.4f,1.f/0.625f,1.f/0.1f);
        glPushMatrix();
        {
            // torso
            float hl =0.35f;
            glPushMatrix();
            glTranslatef(0,0.175f,0);
            glScalef(1,1,0.3f);
            glDrawCylinder(0.245f,0.225f,hl,8,/*axis*/1,true,3);
            glPopMatrix();


            // arms
            glTranslatef(0,0.1f,0);
            hl =0.5f;
            float dx= 0.325f;
            glPushMatrix();
            glTranslatef(dx,0.025f,0.f);
            glDrawCylinder(0.06f,0.075f,hl-0.05f,8,/*axis*/1,true,3,-0.05f);
            glPopMatrix();

            glPushMatrix();
            glTranslatef(-dx,0.025f,0.f);
            glDrawCylinder(0.06f,0.075f,hl-0.05f,8,/*axis*/1,true,3,0.05f);
            glPopMatrix();

            // belt
            glColorEnum(COLOR_BROWN);
            glPushMatrix();
            glTranslatef(0,-0.125f,0);
            glScalef(1,0.725,0.3f);
            glDrawCylinder(0.225f,0.225f,0.075f,8,/*axis*/1,true,3);
            glPopMatrix();

            // pelvis
            glColorEnum(COLOR_DARKBLUE);
            glPushMatrix();
            glTranslatef(0,-0.2075f,0);
            glScalef(1,1.46,0.3f);
            glDrawCylinder(0.225f,0.225f,0.075f,8,/*axis*/1,true,3);
            glPopMatrix();
        }
        glPopMatrix();
        glPushMatrix();
        {
            // legs        
            glTranslatef(0,-0.36f,0);
            float dx = 0.15f, hl =0.4f;
            glPushMatrix();
            glTranslatef(-dx,0.f,0.f);
            glScalef(1.f,1.01f,0.65);
            glDrawCylinder(0.1f,0.1f,hl,8,/*axis*/ 1,true,2,dx*0.25f);
            glPopMatrix();

            glPushMatrix();
            glTranslatef(dx,0.f,0.f);
            glScalef(1.f,1.01f,0.65);
            glDrawCylinder(0.1f,0.1f,hl,8,/*axis*/ 1,true,2,-dx*0.25);
            glPopMatrix();

            // shoes
            glColorEnum(COLOR_BROWN);
            glTranslatef(0,-0.235f,0);
            dx = 0.15f;
            glPushMatrix();
            glTranslatef(-dx,0.f,0.045f);
            glScalef(1.75,0.65,2.0);
            glutSolidCube(0.1f);
            glPopMatrix();

            glPushMatrix();
            glTranslatef(dx,0.f,0.045f);
            glScalef(1.75,0.65,2.0);
            glutSolidCube(0.1f);
            glPopMatrix();
        }
        glPopMatrix();
        glPushMatrix();
        {
            glColorEnum(COLOR_LIGHTPINK);
            // neck
            glPushMatrix();
            glTranslatef(0,0.375f,0);
            glScalef(0.5,1,0.3f);
            glDrawCylinder(0.05f,0.2f,0.1f,8,/*axis*/ 1,true,2);
            glPopMatrix();
            // head
            glPushMatrix();
            glTranslatef(0,0.5f,0);
            glScalef(1.1,1,0.75f);
            glutSolidSphere(0.15f,8,4);
            glPopMatrix();
            // nose
            glPushMatrix();
            glTranslatef(0,0.46f,0.11f);
            glRotatef(-10,1,0,0);
            glScalef(0.5325*0.3f,0.5*0.3f,0.5*0.3f);
            glDrawCylinder(0.075,0.2f,0.5f,4,/*axis*/1,false);
            glPopMatrix();
            // hair
            glColorEnum(COLOR_BROWN);
            glPushMatrix();
            glTranslatef(0,0.525f,-0.025f);
            glScalef(1.1,1,0.75f);
            glutSolidSphere(0.15f,8,4);
            glPopMatrix();
            // eyes
            glColorEnum(COLOR_WHITE);
            float dx=0.06f;
            glTranslatef(0.f,0.53f,0.088f);
            glPushMatrix();
            glTranslatef(dx,0,0);
            //glScalef(1,0.75f,1);glDrawCylinderZ(0.035,0.001f,8,false,2);
            glScalef(1,0.75f,0.2f);glutSolidSphere(0.05,8,4);
            glPopMatrix();
            glPushMatrix();
            glTranslatef(-dx,0,0);
            //glScalef(1,0.75f,1);glDrawCylinderZ(0.035,0.001f,8,false,2);
            glScalef(1,0.75f,0.2f);glutSolidSphere(0.05,8,4);
            glPopMatrix();
            glColorEnum(COLOR_BLUE);
            dx=0.07f;
            glTranslatef(0,-0.003f,0.0075f);
            glPushMatrix();
            glTranslatef(dx,0,0);
            //glDrawCylinderZ(0.02,0.001f,8,false,2);
            glScalef(1,0.85,0.2f);glutSolidSphere(0.02,8,4);
            glPopMatrix();
            glPushMatrix();
            glTranslatef(-dx,0,0);
            //glDrawCylinderZ(0.02,0.001f,8,false,2);
            glScalef(1,0.85,0.2f);glutSolidSphere(0.02,8,4);
            glPopMatrix();
        }
        glPopMatrix();
        glPushMatrix();
        {
            // hands
            glColorEnum(COLOR_LIGHTPINK);
            float hr = 0.1f;
            glTranslatef(0,-0.05-hr,0);
            float dx= 0.325f;
            glPushMatrix();
            glTranslatef(dx,0.f,0.f);
            glScalef(0.55f,1,0.75f);
            glutSolidSphere(hr,8,4);
            glPopMatrix();
            glPushMatrix();
            glTranslatef(-dx,0.f,0.f);
            glScalef(0.55f,1,0.75f);
            glutSolidSphere(hr,8,4);
            glPopMatrix();
        }
        glPopMatrix();
    }
        break;
    case SHAPE_ROOF:
        glScalef(0.65f/1.2f,0.2f/0.15f,1.0f/2.75f);
        glTranslatef(0.f,-0.15f,0.0f);
        glScalef(0.54f,0.225f,1.0f);
        glRotatef(30.f,0,0,1);
        glDrawCylinderZ(4.f,5.5f,3,true);
        break;
    case SHAPE_PIVOT3D_CENTER: glutSolidSphere(0.06,8,8);
        break;
    case SHAPE_PIVOT3D_AXIS_X:
        // X Axis
        glPushMatrix();
        glRotatef(90,0,1,0);
        glDrawCylinderZ(0.04,0.25,8,false);
        glTranslatef(0,0,0.25);
        glutSolidCone(0.06,0.1,8,8);
        glPopMatrix();
        break;
    case SHAPE_PIVOT3D_AXIS_Y:
        // Y Axis
        glPushMatrix();
        glRotatef(-90,1,0,0);
        glDrawCylinderZ(0.04,0.25,8,false);
        glTranslatef(0,0,0.25);
        glutSolidCone(0.06,0.1,8,8);
        glPopMatrix();
        break;
    case SHAPE_PIVOT3D_AXIS_Z:
        // Z Axis
        glPushMatrix();
        glDrawCylinderZ(0.04,0.25,8,false);
        glTranslatef(0,0,0.25);
        glutSolidCone(0.06,0.1,8,8);
        glPopMatrix();
        break;
    case SHAPE_PIVOT3D:
        glCallList(globals.display_lists[SHAPE_PIVOT3D_CENTER]); // Center
        glColor3f(1,0,0);glCallList(globals.display_lists[SHAPE_PIVOT3D_AXIS_X]); // X Axis
        glColor3f(0,1,0);glCallList(globals.display_lists[SHAPE_PIVOT3D_AXIS_Y]); // Y Axis
        glColor3f(0,0,1);glCallList(globals.display_lists[SHAPE_PIVOT3D_AXIS_Z]); // Z Axis
        glColor3f(1,1,1);
        break;
    case SHAPE_AABB:
        glBegin(GL_LINE_STRIP);
        glVertex3f(-1.f,-1.f,-1.f);glVertex3f(-1.f,-1.f,1.f);glVertex3f(1.f,-1.f,1.f);glVertex3f(1.f,-1.f,-1.f);glVertex3f(-1.f,-1.f,-1.f);
        glVertex3f(-1.f, 1.f,-1.f);glVertex3f(-1.f, 1.f,1.f);glVertex3f(1.f, 1.f,1.f);glVertex3f(1.f, 1.f,-1.f);glVertex3f(-1.f, 1.f,-1.f);
        glEnd();
        glBegin(GL_LINES);
        glVertex3f( 1.f,-1.f,-1.f);glVertex3f( 1.f,1.f,-1.f);glVertex3f( 1.f,-1.f, 1.f);glVertex3f( 1.f,1.f, 1.f);glVertex3f(-1.f,-1.f, 1.f);glVertex3f(-1.f,1.f, 1.f);
        glEnd();
        break;
    case SHAPE_STAR_2D: {
        const float r=1.f,r_in=0.382f*r; float P[2][10];
        const float angles[10]={M_DEG2RAD(90.f),M_DEG2RAD(126.f),M_DEG2RAD(162.f),M_DEG2RAD(198.f),M_DEG2RAD(234.f),M_DEG2RAD(270.f),M_DEG2RAD(306.f),M_DEG2RAD(342.f),M_DEG2RAD(18.f), M_DEG2RAD(54.f)};
        for (int i=0;i<10;i++) {
            const float radius=(i%2==0)?r:r_in;
            P[0][i]=radius*cosf(angles[i]);
            P[1][i]=radius*sinf(angles[i]);
        }
        glBegin(GL_TRIANGLES);
        glNormal3f(0,0,1);
        for (int i=0;i<10;i+= 2) {
            const int ip1=(i+1)%10, im1=(i+9)%10;
            glVertex2f(P[0][i],   P[1][i]);
            glVertex2f(P[0][ip1], P[1][ip1]);
            glVertex2f(P[0][im1], P[1][im1]);
        }
        glVertex2f(P[0][1], P[1][1]);
        glVertex2f(P[0][3], P[1][3]);
        glVertex2f(P[0][5], P[1][5]);

        glVertex2f(P[0][1], P[1][1]);
        glVertex2f(P[0][5], P[1][5]);
        glVertex2f(P[0][7], P[1][7]);

        glVertex2f(P[0][1], P[1][1]);
        glVertex2f(P[0][7], P[1][7]);
        glVertex2f(P[0][9], P[1][9]);
        glEnd();
    }
        break;
    case SHAPE_STAR: {
        const float R=1.0f,r=R*0.5f,hd=1.0f; // hd: half depth
        const float offy = 0.5f*R*cosf(M_DEG2RAD(72.f)), scaleY=2.f*R/(2.f*R-offy), scaleXZ=1.f-((1.f-scaleY)*0.25f);
        glTranslatef(0,-offy*0.5f,0);glScalef(scaleXZ,scaleY,scaleXZ); // the latter 2 lines are for centering/rescaling (=optional)
        struct Vertex {float x, y, z;} front[10]={},back[10]={};
        for (int i=0;i<10;i++) {
            Vertex *v0=&front[i],*v1=&back[i];
            const float a=M_PI*(0.5f+i*0.2f),rad=(i%2==0)?R:r;
            const float x=rad*cosf(a),y=rad*sinf(a);
            v0->x=x;v0->y=y;v0->z=hd;v1->x=x;v1->y=y;v1->z=-hd;
        }
        glBegin(GL_TRIANGLES);
        for (int j=0;j<2;j++)  {
            Vertex* vertices=(j==0?front:back), center={0.f,0.f,vertices[0].z};
            glNormal3f(0.f,0.f,j==0?1.f:-1.f);
            for (int i=0;i<10;i++) {
                const int next=(i+1)%10;
                const Vertex *v0=&center,*v1=&vertices[i],*v2=&vertices[next];
                if (j==0) {glVertex3fv(&v0->x);glVertex3fv(&v1->x);glVertex3fv(&v2->x);}
                else {glVertex3fv(&v0->x);glVertex3fv(&v2->x);glVertex3fv(&v1->x);}
            }
        }
        for (int i=0;i<10;i++) {
            const int next=(i+1)%10;
            const Vertex *v0=&front[i],*v1=&back[i],*v2=&back[next],*v3=&front[next];
            const Vertex d1={v1->x-v0->x,v1->y-v0->y,v1->z-v0->z};
            const Vertex d2={v2->x-v0->x,v2->y-v0->y,v2->z-v0->z};
            Vertex n;nudge::nm_Vec3Cross(&n.x,&d1.x,&d2.x);glNormal3fv(&n.x);
            glVertex3fv(&v0->x);glVertex3fv(&v1->x);glVertex3fv(&v2->x);
            glVertex3fv(&v0->x);glVertex3fv(&v2->x);glVertex3fv(&v3->x);
        }
        glEnd();
    }
        break;
    case SHAPE_COUNT:
        break;
    }
    glPopMatrix();
    glEndList();
    }
}


void glColorEnum(ColorEnum color) {
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
    glColor3fv(&colors[color][0]);
#   undef CEU2F
}

#define GLPRINTF_BUFFER_SIZE (4096) // for a page of text
static char glprintf_buffer[GLPRINTF_BUFFER_SIZE];
static int glprintf_buffer_size=0;

void glprintf_flush(void) {
    int line_height=24,text_size = 18,base_x=0;
    void* font = GLUT_BITMAP_HELVETICA_18;
    bool blinking = false;
    static int timeBegin = glutGet(GLUT_ELAPSED_TIME);
    int elapsedTime = glutGet(GLUT_ELAPSED_TIME) - timeBegin;
    if (elapsedTime<0) {timeBegin = glutGet(GLUT_ELAPSED_TIME);elapsedTime=0;}
    const bool can_blink = ((elapsedTime/100)%10)<5;
    if (glprintf_buffer_size==0 && c->simulation_params.num_frames==1) {glprintf_buffer_size=2;glprintf_buffer[0]=' ';glprintf_buffer[1]='\0';}

    if (glprintf_buffer_size>0) {
        GLint viewport[4];glGetIntegerv(GL_VIEWPORT,viewport);
        glDisable(GL_CULL_FACE);glDisable(GL_LIGHTING);glDisable(GL_DEPTH_TEST);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(viewport[0],viewport[2],viewport[1],viewport[3],0,1); // x [0], y [1], width [2], height [3]
        glScalef(1, -1, 1);glTranslatef(0, -viewport[3]/**factor*/, 0);
        const int tab = viewport[2]/20;      // "/t", not "\t"
        const int tab_w = viewport[2]/10;   // "/h[4]"  with index in [0,9]
        const int tab_h = viewport[3]/10;   // "/v[4]"  with index in [0,9]

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();

        // Fading background effect ---------------------        
        const float target_alpha = (globals.gui.menu_idx!=0)?globals.gui.menu_bg_alpha:0.f;
        static float alpha = 0.f;
        if (alpha!=target_alpha) {
            const float speed = (float)elapsedTime*0.001f;
            if (globals.gui.menu_idx!=0) {alpha+=speed;if (alpha>target_alpha) alpha=target_alpha;}
            else {alpha-=speed;if (alpha<target_alpha) alpha=target_alpha;}
        }
        if (alpha>0) {
            if (alpha<1) glEnable(GL_BLEND);
            glBegin(GL_QUADS);
            const float c0=199.f/255.f,c1=180.f/255.f,c2=166.f/255,c3=150.f/255.f;
            glColor4f(c0,c0,c0,alpha);glVertex2d(viewport[0],viewport[1]);
            glColor4f(c1,c1,c1,alpha);glVertex2d(viewport[2],viewport[1]);
            glColor4f(c2,c2,c2,alpha);glVertex2d(viewport[2],viewport[3]);
            glColor4f(c3,c3,c3,alpha);glVertex2d(viewport[0],viewport[3]);
            glEnd();
            if (globals.gui.menu_idx==1) {
                const float screen_center[2] = {(float)(viewport[2]+viewport[0])*0.5f,(float)(viewport[3]+viewport[1])*0.5f};
                const float screen_halfsize[2] = {(float)(viewport[2]-viewport[0])*0.5f,(float)(viewport[3]-viewport[1])*0.5f};
                const float c[2] = {screen_center[0]+screen_halfsize[0]*0.075f,screen_center[1]+screen_halfsize[1]*0.775f};
                const float h = screen_halfsize[1]*0.2f;const float s[2] = {1.5f*h,h};
                glColor4f(0.f,51.f/255.f,153.f/255.f,1.f-(1.f-alpha)*0.75f);
                glPushMatrix();
                glBegin(GL_QUADS);
                glVertex2f(c[0]-s[0],c[1]-s[1]);glVertex2f(c[0]+s[0],c[1]-s[1]);
                glVertex2f(c[0]+s[0],c[1]+s[1]);glVertex2f(c[0]-s[0],c[1]+s[1]);
                glEnd();
                glColor4f(1.f,204.f/255.f,0.f,alpha);
                const float scale = h/8.f;const float scale2 = 5.5f;
                glTranslatef(c[0],c[1]*1.0025f,0.f);glScalef(scale,-scale,1.f);
                for (int j=0;j<12;j++)   {
                    const float angle = (float)j*(M_PI/6.f),cosa=cosf(angle),sina=sinf(angle);
                    glPushMatrix();glTranslatef(scale2*cosa,scale2*sina,0.f);drawShape(SHAPE_STAR_2D);glPopMatrix();
                }
                glPopMatrix();
            }
            if (alpha<1) glDisable(GL_BLEND);
        }
        // -----------------------------------------------
        if (alpha==target_alpha) {
            //int base_y = 0;
#           ifndef GLPRINTF_FLUSH_NO_TWEAKING
            static float initial_h = viewport[3];
            glViewport(viewport[0],viewport[1]-(viewport[3]-initial_h),viewport[2],viewport[3]);
#           endif //GLPRINTF_FLUSH_NO_TWEAKING
            glColor3f(0.65f,0.f,0.f); // initial text color
            const globals_t::gui_t* gui = &globals.gui;assert(gui->menu_idx<3);
            const int blink_marker = gui->blink_markers[gui->menu_idx];
            const char* end_char = glprintf_buffer+glprintf_buffer_size,*c = glprintf_buffer;
            const int base_y = 0;
            int x=0,y=line_height;

            glRasterPos2d(x,y);
            while (c!=end_char) {
                if (*c=='\n') {
                    y+=line_height;line_height=text_size;if (y>viewport[3]) break;
                    x=base_x;glRasterPos2d(x,y);++c;continue;
                }
                else if (*c=='\t') {/*tab*/x+=2*24;glRasterPos2d(x,y);++c;continue;}
                else if (*c=='/' && c+1!=end_char) {
                    const char type_char = *(c+1);
                    if (type_char=='t') {/*absolute column tab proportional to screen width*/x=(x%tab)*tab+tab;glRasterPos2d(x,y);c+=2;continue;}
                    else if (c+4<end_char && *(c+2)=='[') {
                        // Format: "/t[a]" where 't' is a specific single char names 'type_char', and 'a' is its argument (that can span multiple chars)
                        const char* beg = c+3;
                        //beg = strchr(beg,']');  // strchr is not safe!
                        beg = (const char*) memchr((void*)beg,']',(end_char-beg>16?16:end_char-beg));    // safer
                        if (beg && beg>c+3) {
                            const char* txt = c+3;
                            if (type_char=='c' || type_char=='b')    {
                                // arg_char is int
                                static char tmp[16]={};
                                int res = 0, rv = sscanf(txt,"%15s",tmp);tmp[15]='\0';assert(rv==1);   // safer than sscanf with integers
                                char* beg2=NULL;res = (int) strtol(tmp,&beg2,10); // strtol stops when '\0' is found
                                assert(*beg==*beg2);  // ']' char
                                if (type_char=='c')   {
                                    // COLOR: glprintf("HI /c[%d]EVERYBODY! /c[%d]HELLO!",COLOR_RED,COLOR_BLUE); // displays: "HI EVERYBODY! HELLO!" with "EVERYBODY!" in red and "HELLO!" in blue
                                    assert(res<COLOR_COUNT);
                                    glColorEnum((ColorEnum)res);
                                    glRasterPos2f(x,y);
                                }
                                else if (type_char=='b') {
                                    // BLINK MARKER: glprintf("HI /b[2]EVERYBODY!/b[2] HELLO!"); // "EVERYBODY!" blinks when user sets: glprintf_blink_marker=2;
                                    if (res==blink_marker) {
                                        blinking=!blinking;
                                    }
                                }
                            }
                            else if (type_char=='f' || type_char=='h' || type_char=='v')    {
                                // arg_char is single char
                                const char arg_char = *(c+3);
                                assert(beg==c+4);   // one single char allowed inside [ ]
                                if (type_char=='f') {
                                    // FONT: glprintf("HI /f[b]EVERYBODY! /f[r]HELLO!"); // "EVERYBODY!" is writtem using the big font
                                    if (arg_char=='r' || arg_char=='R' || arg_char=='n' || arg_char=='N')   {
                                        font = GLUT_BITMAP_HELVETICA_18;text_size=18;
                                    }
                                    else if (arg_char=='s' || arg_char=='S')   {
                                        font = GLUT_BITMAP_HELVETICA_12;text_size=12;
                                    }
                                    else if (arg_char=='b' || arg_char=='B')    {
                                        font = GLUT_BITMAP_TIMES_ROMAN_24;text_size=24;
                                    }
                                    else {assert(0);}
                                }
                                else if (type_char=='h') {
                                    // HORIZONTAL WARP in [0,9]: glprintf("HI /h[1]EVERYBODY! /h[8]HELLO!"); // displays EVERYBODY! on absolute column-block 1 and HELLO! on absolute column-block 4
                                    assert(arg_char>='0' && arg_char<='9');
                                    x=base_x=tab_w*(int)(arg_char-'0');glRasterPos2d(x,y);
                                }
                                else if (type_char=='v') {
                                    // VERTICAL WARP in [0,9]: glprintf("/v[1]EVERYBODY!/v[8]HELLO!"); // displays EVERYBODY! on absolute row-block 1 and HELLO! on absolute row-block 4
                                    assert(arg_char>='0' && arg_char<='9');
                                    y=line_height+tab_h*(int)(arg_char-'0');x=base_x;glRasterPos2d(x,y);
                                }
                            }
                            c=beg+1;assert(c<=end_char);continue;
                        }
                    }
                }
                x+=glutBitmapWidth(font, *c);
                if (!blinking || can_blink) glutBitmapCharacter(font, *c);
                else glRasterPos2d(x,y);
                ++c;
            }
            glViewport(viewport[0],viewport[1]-base_y,viewport[2],viewport[3]);
        }
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glColor3f(1.f,1.f,1.f);
        glEnable(GL_DEPTH_TEST);glEnable(GL_LIGHTING);glEnable(GL_CULL_FACE);

        glprintf_buffer_size=0;
    }
    else globals.gui.menu_idx=0;
}



void glvprintf(const char* format, va_list vlist) {
    if (glprintf_buffer_size<GLPRINTF_BUFFER_SIZE-1) {
        const int rv = ::vsnprintf(&glprintf_buffer[glprintf_buffer_size],GLPRINTF_BUFFER_SIZE-glprintf_buffer_size,format,vlist);
        glprintf_buffer_size+=rv;
    }
}
void glprintf(const char *format,...) {
    if (globals.gui.menu_idx!=0) return;
    va_list vlist;
    va_start(vlist, format);
    glvprintf(format, vlist);
    va_end(vlist);
}
void glguihelp(const char *format,...) {
    if (globals.gui.menu_idx!=1) return;
    va_list vlist;
    va_start(vlist, format);
    glvprintf(format, vlist);
    va_end(vlist);
}
void glguimenu(const char *format,...) {
    if (globals.gui.menu_idx!=2) return;
    va_list vlist;
    va_start(vlist, format);
    glvprintf(format, vlist);
    va_end(vlist);
}
void glprintf_blinkmarker(int idx) {
    assert(globals.gui.menu_idx<3);
    globals.gui.blink_markers[globals.gui.menu_idx]=idx;
}
void HandleMenus(void)  {
    globals_t::gui_t* gui = &globals.gui;
    const ColorEnum H1 = COLOR_DARKRED,H2 = COLOR_BROWN,N = COLOR_DARKBLUE,Y = COLOR_DARKRED, E = COLOR_BLACK, D = COLOR_DARKBLUE, S = COLOR_DARKRED;
    switch (gui->menu_idx) {
    case 1: {
        // HELP PAGE
        glguihelp("\n/h[5]/c[%d]HELP PAGE\n\n",H1);
        glguihelp("/c[%d]KEYS:\n\n",H2);

        glguihelp("/c[%d]/h[2]/v[2]",N);
        glguihelp("/c[%d][H]    /c[%d]Show/Hide help page\n",Y,N);
        glguihelp("/c[%d][M]    /c[%d]Show/Hide menu page\n",Y,N);
        glguihelp("/c[%d][SPACE]    /c[%d]Toggle display mode\n",Y,N);
        glguihelp("/c[%d][F]    /c[%d]Toggle frustum culling (%s)\n",Y,N,globals.use_frustum_culling?"on":"off");
        glguihelp("/c[%d][ESC]  /c[%d]Quit program\n",Y,N);

        glguihelp("/c[%d]/h[7]/v[2]",N);
        glguihelp("/c[%d][R]    /c[%d]Restart simulation\n",Y,N);
        glguihelp("/c[%d][F5]   /c[%d]Quick save simulation\n",Y,N);
        glguihelp("/c[%d][F7]   /c[%d]Quick reload simulation\n",Y,N);
        glguihelp("/c[%d][LMB]  /c[%d]Throw body\n",Y,N);
        glguihelp("/c[%d][RMB]  /c[%d]Select body\n",Y,N);

        glguihelp("\n/c[%d]/h[2]",N);
        glguihelp("/c[%d][ENTER/END/MMB]    /c[%d]Toggle camera mode\n",Y,N);
        glguihelp("/c[%d][ARROW_KEYS]   /c[%d]Rotate camera\n",Y,N);
        glguihelp("/c[%d][PAGE_UP/PAGE_DOWN]    /c[%d]Zoom camera\n",Y,N);
        glguihelp("/c[%d][CTRL][ARROW_KEYS/PAGE_UP/PAGE_DOWN]   /c[%d]Move camera target (in free mode)\n",Y,N);
        glguihelp("/c[%d][HOME]   /c[%d]Reset camera\n",Y,N);
        glguihelp("/c[%d][WASD][J]    /c[%d]Move character and jump\n",Y,N);

        glguihelp("\n/h[5]/c[%d]CREDITS\n\n/h[2]/c[%d]",H2,N);
        glguihelp("-    To Mark J. Kilgard for releasing the OpenGL Utility Toolkit in 1994.\n");
        glguihelp("-    To Rasmus Barringer for releasing Nudge Physics in 2017.\n");

        glguihelp("/h[5]/v[7]\n/c[%d]Made in EU",N);

        static float FPS=60.f,sum=0.f;static int cnt=0;
        sum+=globals.instantFrameTime;if (cnt==60) {FPS=sum>0?60.f/sum:60.f;sum=0;cnt=0;}
        glguihelp("/h[9]/v[9]\n\n/c[%d]FPS: %1.f",N,FPS);

        if (globals.bodies.num_stars>0) {
            if (globals.use_frustum_culling) glguihelp("/f[s]/h[0]/v[9]\n/c[%d]   Num stars: %u/12\n   Num frustum culled bodies: %u/%u/f[n]",N,globals.bodies.num_stars,globals.num_frustum_culled_bodies,c->bodies.count-c->global_data.removed_bodies_count);
            else glguihelp("/f[s]/h[0]/v[9]\n\n/c[%d]   Num stars: %u/12/f[n]",N,globals.bodies.num_stars);
        }
        else if (globals.use_frustum_culling) glguihelp("/f[s]/h[0]/v[9]\n\n/c[%d]   Num frustum culled bodies: %u/%u/f[n]",N,globals.num_frustum_culled_bodies,c->bodies.count-c->global_data.removed_bodies_count);

    }
        break;
    case 2: {
        // MENU PAGE
        nudge::SimulationParams* sp = &c->simulation_params;
        int blink = 0;
        glguimenu("\n/h[5]/c[%d]MENU PAGE\n\n\n",H1);
        glguimenu("/h[2]/c[%d]SIMULATION PARAMS:\n\n",H2,COLOR_TEAL);
        glguimenu("\n/c[%d]",N);

        // processing input and drawing menu (making this a static struct works in our case, but it's less robust)
        static struct edit_t {
            struct var_uint32_t {const char* name;uint32_t* p;uint32_t def,vmin,vmax,vstep;} u[5];
            struct var_float_t {const char* name;float* p;float def,vmin,vmax,vstep;} f[7];
        } edit = {
        {{"inverse_time_step",NULL,(uint32_t)floor(0.5 + 1.0/(double)NUDGE_DEFAULT_SIMULATION_TIMESTEP),20,240,1},
        {"max_num_substeps",&sp->max_num_substeps,NUDGE_DEFAULT_MAX_NUM_SIMULATION_SUBSTEPS,1,10,1},
        {"numsubsteps_overflow_warning_mode",&sp->numsubsteps_overflow_warning_mode,0,0,2,1},
        {"num_iterations_per_substep",&sp->num_iterations_per_substep,NUDGE_DEFAULT_NUM_SIMULATION_ITERATIONS,3,1000,1},
        {"fix_character_sinking_effect_on_fall",&globals.bodies.fix_character_sinking_effect_on_fall,0,0,1,1}
        },
        {
        {"sleeping_threshold_linear_velocity_squared",&sp->sleeping_threshold_linear_velocity_squared,NUDGE_DEFAULT_SLEEPING_THRESHOLD_LINEAR_VELOCITY_SQUARED,0.001f,2.f,0.005f},
        {"sleeping_threshold_angular_velocity_squared",&sp->sleeping_threshold_angular_velocity_squared,NUDGE_DEFAULT_SLEEPING_THRESHOLD_ANGULAR_VELOCITY_SQUARED,0.01f,10.f,0.05f},
        {"linear_damping",&sp->linear_damping,NUDGE_DEFAULT_DAMPING_LINEAR,0.01f,2.f,0.01f},
        {"angular_damping",&sp->angular_damping,NUDGE_DEFAULT_DAMPING_ANGULAR,0.01f,2.f,0.01f},
        {"penetration_allowed_amount",&sp->penetration_allowed_amount,NUDGE_DEFAULT_PENETRATION_ALLOWED_AMOUNT,0.0001f,0.01f,0.0001f},
        {"penetration_bias_factor",&sp->penetration_bias_factor,NUDGE_DEFAULT_PENETRATION_BIAS_FACTOR,0.5f,4.f,0.1f},
        {"Menu background opacity",&gui->menu_bg_alpha,0.65f,0.0f,1.f,0.1f}
        }
        };
        const int usz = sizeof(edit.u)/sizeof(edit.u[0]), fsz = sizeof(edit.f)/sizeof(edit.f[0]), sz = usz+fsz;
        int* blinker = &globals.gui.blink_markers[globals.gui.menu_idx];

        // processing input
        uint32_t inv_time_step = (uint32_t)floor(0.5+1.0/sp->time_step);edit.u[0].p = &inv_time_step;
        const uint32_t old_inv_time_step = inv_time_step;
        const globals_t::key_t* key = &globals.key;
        if (key->down&(KEY_ALL_ARROW_KEYS|KEY_HOME)) {
            if (key->down&(KEY_UP|KEY_DOWN))  {
                if (key->pressed&KEY_DOWN) {if (++(*blinker)>=sz) *blinker=0;}
                else if (key->pressed&KEY_UP) {if (--(*blinker)<0) *blinker=sz-1;}
            }
            if (key->down&(KEY_LEFT|KEY_RIGHT|KEY_HOME)) {
                if (*blinker<usz)   {
                    assert(*blinker>=0 && *blinker<usz);
                    edit_t::var_uint32_t* v = &edit.u[*blinker];
                    const uint32_t step = v->vstep*((key->down&KEY_SHIFT)?8:((key->down&KEY_CTRL)?2:1));
                    if (key->pressed&KEY_RIGHT) {*v->p+=step;if (*v->p>v->vmax) *v->p=v->vmax;}
                    else if (key->pressed&KEY_LEFT) {if ((*v->p)>=v->vmin+step) *v->p-=step;else *v->p=v->vmin;}
                    else if (key->pressed&KEY_HOME) *v->p=v->def;
                    if (*blinker==0 && inv_time_step!=old_inv_time_step) sp->time_step = 1.0/(double)inv_time_step;
                }
                else {
                    assert(*blinker>=usz && *blinker<sz);
                    edit_t::var_float_t* v = &edit.f[*blinker-usz];
                    float step = globals.instantFrameTime;if (step<=0) step = 1.f;
                    if (key->down&KEY_SHIFT) step*=8.f;
                    else if (key->down&KEY_CTRL) step*=2.f;
                    if (key->down&KEY_RIGHT) {*v->p+=v->vstep*step;if (*v->p>v->vmax) *v->p=v->vmax;}
                    else if (key->down&KEY_LEFT) {*v->p-=v->vstep*step;if (*v->p<v->vmin) *v->p=v->vmin;}
                    else if (key->pressed&KEY_HOME) *v->p=v->def;
                }
            }
        }

        // drawing menu
        for (unsigned i=0;i<usz;i++) {
            edit_t::var_uint32_t* u = &edit.u[i];
            glguimenu("/c[%d]/h[2]%s/h[6]/c[%d][/b[%d]%u/b[%d]]/h[7]/c[%d][%u]\n",*blinker==blink?S:N,u->name,E,blink,*u->p,blink,*blinker==blink?S:D,u->def);++blink;
            if (i==usz-2) glguimenu("\n");
        }
        glguimenu("\n");
        for (unsigned i=0;i<fsz;i++) {
            edit_t::var_float_t* f = &edit.f[i];
            glguimenu("/c[%d]/h[2]%s/h[6]/c[%d][/b[%d]%1.4f/b[%d]]/h[7]/c[%d][%1.4f]\n",*blinker==blink?S:N,f->name,E,blink,*f->p,blink,*blinker==blink?S:D,f->def);++blink;
            if (i==fsz-2) glguimenu("\n");
        }

        glguimenu("\n\n/h[2]/c[%d][UP/DOWN]/c[%d]: navigate   /c[%d][LEFT/RIGHT]/c[%d]: edit (with CTRL/SHIFT)    /c[%d][HOME]/c[%d]: reset\n",E,N,E,N,E,N);


        static const char* desc[] = {
            /*inv_time_step*/ "the number of physic substeps per second\nby increasing it, the simulation improves a lot, but performance drops",
            /*max_num_substeps*/ "the maximum number of physic substeps per frame;\npast this limit the simulation 'drops' frames",
            /*numsubsteps_overflow_warning_mode*/ "when the 'max_num_substeps' per frame is not enough,\na nudge::log can warn the user\n0: warn only from the second successive frame\n1: warn always\n2: never warn",
            /*num_iterations_per_substep*/ "the number of iterations that are used in each physic substep\nto recover from contact constraints:\nincreasing the number of iterations will improve stability",
            /*fix_character_sinking_effect_on_fall*/ "this is just a quick test hack in this demo (not really a nudge::SimulationParams at all)\nI've exploited this tweaking page for personal usage here...",
            /*sleeping_threshold_linear_velocity_squared*/ "each physic body enters sleeping mode when its velocities are below a certain threshold",
            /*sleeping_threshold_angular_velocity_squared*/ "each physic body enters sleeping mode when its velocities are below a certain threshold",
            /*linear_damping*/ "a measure of the decrease of the velocities of each physic body with time",
            /*angular_damping*/ "a measure of the decrease of the velocities of each physic body with time",
            /*penetration_allowed_amount*/ "a constant that was hard-coded in the original nudge library",
            /*penetration_bias_factor*/ "a constant that was hard-coded in the original nudge library",
            /*menu opacity*/ "the alpha value of this background",

        };
        assert(sz==sizeof(desc)/sizeof(desc[0]));
        if (*blinker>=0 && *blinker<sz) {
            glguimenu("\n/c[%d]%s/c[%d]:\n%s\n",Y,(*blinker)<usz?edit.u[*blinker].name:edit.f[*blinker-usz].name,N,desc[*blinker]);
        }
    }
        break;
    case 0:break;
    default:assert(0); break;
    }
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
