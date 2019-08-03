//
// Copyright (c) 2017 Rasmus Barringer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

// Ported code to no fixed function pipeline by Flix.
// Also added camera support (arrows keys and page up/down, optionally with CTRL down)

// PLEASE COMPILE FROM THE nudge/example folder

// Linux:
//--------
// clang++ -O2 main_no_ffp.cpp -o example_no_ffp -I"./" -I"../" ../nudge.cpp -lglut -lGL
// (Optionally with -march=haswell)
// Or:
// g++ -no-pie main_no_ffp.cpp -o example_no_ffp -I"./" -I"../" ../nudge.cpp -lglut -lGL
// (Optionally with -march=haswell)
// [AFAIR g++ version 5.4.0 20160609 (Ubuntu 5.4.0-6ubuntu1~16.04.4) does not work correctly with -O2 or -O3]

// Emscripten
//------------
// SIMD version (it does not work for me):
// em++ -O2 -D"NUM_SIMULATION_STEPS=1" -D"NUM_SIMULATION_ITERATIONS=5" -D"NUM_BOXES=16" -D"NUM_SPHERES=8" -D"MAX_BODY_COUNT=32" -D"ARENA_SIZE=(128*1024)"  -D"WINDOW_WIDTH=960" -D"WINDOW_HEIGHT=540" -fno-rtti -fno-exceptions -msse2 -o html/nudge.html main_no_ffp.cpp ../nudge.cpp -I"./" -I"../" -s LEGACY_GL_EMULATION=0 -lglut --closure 1
// Then run html/nudge.html by using a web server.
// This demo is not ready for SIMDE (simd emulation) support. Please see "main_no_ffp_with_shadows.cpp" if interested.

// Windows
//---------
// Never tried, but it needs GLEW to import all the required GL functions

// Mac
//----
// Never tried


#include <nudge.h>
#include <immintrin.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef GL_GLEXT_PROTOTYPES
#	define GL_GLEXT_PROTOTYPES	// This includes all gl functions on all systems except Windows 
#endif

#ifdef _WIN32
#undef USE_GLEW
#define USE_GLEW		// Mandatory to include all gl functions
#endif

#ifdef __APPLE__
#	ifdef USE_GLEW
#		include <GLEW/glew.h>
#	endif
#	include <GLUT/GLUT.h>
#	include <OpenGL/gl.h>
#elif _MSC_VER
#	ifdef USE_GLEW
#		include <GLEW/glew.h>
#	endif
#	include <GLUT/glut.h>
#	include <GL/gl.h>
#else // linux
#	ifdef USE_GLEW
#		include <GL/glew.h>
#	endif
#	include <GL/glut.h>
#	include <GL/gl.h>
#endif

#ifdef __EMSCRIPTEN__
// No <mm_malloc.h> header in emscripten so far
static inline void* _mm_malloc (size_t size, size_t alignment)	{
  void *ptr;
  if (alignment == 1) return malloc (size);
  if (alignment == 2 || (sizeof (void *) == 8 && alignment == 4)) alignment = sizeof (void *);
  if (posix_memalign (&ptr, alignment, size) == 0) return ptr;
  else return NULL;
}
static inline void _mm_free (void * ptr) {free (ptr);}
#endif


#ifndef M_PI
#define M_PI 3.1415
#endif

// camera data:
float targetPos[3] = {0,-8,0};     // This demo places the ground plane below zero
float cameraYaw = 2*M_PI;
float cameraPitch = M_PI*0.125f;
float cameraDistance = 60;
float cameraPos[3] = {0,0,0};       // Derived value (do not edit)
float lightDirection[3] = {1,1,2};	// Will be normalized
float vMatrix[16];                  // view matrix

// pMatrix data:
float pMatrix[16];                  // projection matrix
const float pMatrixFovDeg = 45.f;
const float pMatrixNearPlane = 0.5f;
const float pMatrixFarPlane = 200.0f;

float instantFrameTime = 16.2f;


// teapot.h--------------------------------
//#define TEAPOT_SHADER_USE_NORMAL_MATRIX       // Not needed in this demo
#define TEAPOT_SHADER_SPECULAR
#define TEAPOT_SHADER_FOG
//#define TEAPOT_SHADER_FOG_HINT_FRAMENT_SHADER   // Better fog on ground plane (but a bit expensive)
#define TEAPOT_IMPLEMENTATION
#include "teapot.h"
// ----------------------------------------


#ifndef MAX_BODY_COUNT
#define MAX_BODY_COUNT 2048
#endif

#ifndef NUM_BOXES
#define NUM_BOXES 1024
#endif

#ifndef NUM_SPHERES
#define NUM_SPHERES 512
#endif

static const unsigned max_body_count = MAX_BODY_COUNT;
static const unsigned max_box_count = MAX_BODY_COUNT;
static const unsigned max_sphere_count = MAX_BODY_COUNT;


static const nudge::Transform identity_transform = { {}, 0, { 0.0f, 0.0f, 0.0f, 1.0f } };

static nudge::Arena arena;
static nudge::BodyData bodies;
static nudge::ColliderData colliders;
static nudge::ContactData contact_data;
static nudge::ContactCache contact_cache;
static nudge::ActiveBodies active_bodies;

static inline void quaternion_concat(float r[4], const float a[4], const float b[4]) {
	r[0] = b[0]*a[3] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1];
	r[1] = b[1]*a[3] + a[1]*b[3] + a[2]*b[0] - a[0]*b[2];
	r[2] = b[2]*a[3] + a[2]*b[3] + a[0]*b[1] - a[1]*b[0];
	r[3] = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];
}

static inline void quaternion_transform(float r[3], const float a[4], const float b[3]) {
	float t[3];
	t[0] = a[1]*b[2] - a[2]*b[1];
	t[1] = a[2]*b[0] - a[0]*b[2];
	t[2] = a[0]*b[1] - a[1]*b[0];
	
	t[0] += t[0];
	t[1] += t[1];
	t[2] += t[2];
	
	r[0] = b[0] + a[3]*t[0] + a[1]*t[2] - a[2]*t[1];
	r[1] = b[1] + a[3]*t[1] + a[2]*t[0] - a[0]*t[2];
	r[2] = b[2] + a[3]*t[2] + a[0]*t[1] - a[1]*t[0];
}

static inline void matrix(float r[16], const float s[3], const float q[4], const float t[3]) {
	float kx = q[0] + q[0];
	float ky = q[1] + q[1];
	float kz = q[2] + q[2];
	
	float xx = kx*q[0];
	float yy = ky*q[1];
	float zz = kz*q[2];
	float xy = kx*q[1];
	float xz = kx*q[2];
	float yz = ky*q[2];
	float sx = kx*q[3];
	float sy = ky*q[3];
	float sz = kz*q[3];
	
	r[0] = (1.0f - yy - zz) * s[0];
	r[1] = (xy + sz) * s[0];
	r[2] = (xz - sy) * s[0];
	r[3] = 0.0f;
	
	r[4] = (xy - sz) * s[1];
	r[5] = (1.0f - xx - zz) * s[1];
	r[6] = (yz + sx) * s[1];
	r[7] = 0.0f;
	
	r[8] = (xz + sy) * s[2];
	r[9] = (yz - sx) * s[2];
	r[10] = (1.0f - xx - yy) * s[2];
	r[11] = 0.0f;
	
	r[12] = t[0];
	r[13] = t[1];
	r[14] = t[2];
	r[15] = 1.0f;
}

// custom replacement of gluPerspective(...)
static void nuPerspective(float res[16],float degfovy,float aspect, float zNear, float zFar) {
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
}
// custom replacement of gluLookAt(...)
static void nuLookAt(float m[16],float eyeX,float eyeY,float eyeZ,float centerX,float centerY,float centerZ,float upX,float upY,float upZ)    {
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
}

static inline float dot(const float v0[3],const float v1[3]) {
    return v0[0]*v1[0]+v0[1]*v1[1]+v0[2]*v1[2];
}
static inline void normalize(float v[3]) {
    float len = dot(v,v);
    if (len!=0) {
        len = sqrt(len);
        for (int i=0;i<3;i++) v[i]/=len;
    }
}
static inline void cross(float rv[3],const float a[3],const float b[3]) {
    rv[0] =	a[1] * b[2] - a[2] * b[1];
    rv[1] =	a[2] * b[0] - a[0] * b[2];
    rv[2] =	a[0] * b[1] - a[1] * b[0];
}


static inline unsigned add_box(float mass, float cx, float cy, float cz) {
	if (bodies.count == max_body_count || colliders.boxes.count == max_box_count)
		return 0;
	
	unsigned body = bodies.count++;
	unsigned collider = colliders.boxes.count++;
	
	float k = mass * (1.0f/3.0f);
	
	float kcx2 = k*cx*cx;
	float kcy2 = k*cy*cy;
	float kcz2 = k*cz*cz;
	
	nudge::BodyProperties properties = {};
	properties.mass_inverse = 1.0f / mass;
	properties.inertia_inverse[0] = 1.0f / (kcy2+kcz2);
	properties.inertia_inverse[1] = 1.0f / (kcx2+kcz2);
	properties.inertia_inverse[2] = 1.0f / (kcx2+kcy2);
	
	memset(&bodies.momentum[body], 0, sizeof(bodies.momentum[body]));
	bodies.idle_counters[body] = 0;
	bodies.properties[body] = properties;
	bodies.transforms[body] = identity_transform;
	
	colliders.boxes.transforms[collider] = identity_transform;
	colliders.boxes.transforms[collider].body = body;
	
	colliders.boxes.data[collider].size[0] = cx;
	colliders.boxes.data[collider].size[1] = cy;
	colliders.boxes.data[collider].size[2] = cz;
	colliders.boxes.tags[collider] = collider;
	
	return body;
}

static inline unsigned add_sphere(float mass, float radius) {
	if (bodies.count == max_body_count || colliders.spheres.count == max_sphere_count)
		return 0;
	
	unsigned body = bodies.count++;
	unsigned collider = colliders.spheres.count++;
	
	float k = 2.5f / (mass*radius*radius);
	
	nudge::BodyProperties properties = {};
	properties.mass_inverse = 1.0f / mass;
	properties.inertia_inverse[0] = k;
	properties.inertia_inverse[1] = k;
	properties.inertia_inverse[2] = k;
	
	memset(&bodies.momentum[body], 0, sizeof(bodies.momentum[body]));
	bodies.idle_counters[body] = 0;
	bodies.properties[body] = properties;
	bodies.transforms[body] = identity_transform;
	
	colliders.spheres.transforms[collider] = identity_transform;
	colliders.spheres.transforms[collider].body = body;
	
	colliders.spheres.data[collider].radius = radius;
	colliders.spheres.tags[collider] = collider + max_box_count;
	
	return body;
}


static void resize(int w,int h) {
	// projection Matrix
	if (h>0) {
        glViewport(0,0,w,h);
        nuPerspective(pMatrix,pMatrixFovDeg,(float)w/(float)h,pMatrixNearPlane,pMatrixFarPlane);
        Teapot_SetProjectionMatrix(pMatrix);
#       ifdef TEAPOT_SHADER_FOG
        Teapot_SetFogColor(0.3f, 0.6f, 1.0f);
        Teapot_SetFogDistances((pMatrixFarPlane-pMatrixNearPlane)*0.5f,pMatrixFarPlane);
#       endif
	}	
}

#define NUM_COLORS 10
static float Colors[NUM_COLORS][3]={
{0.4f,0.4f,0.f},
{0.2f,0.7f,0.4f},
{0.8f,0.2f,0.2f},
{0.4f,1.0f,0.2f},
{0.2f,0.4f,1.f},
{1.0f,0.4f,0.4f},
{1.0f,1.f,0.4f},
{0.8f,0.2f,0.8f},
{0.5f,0.7f,1.0f},
{0.65f,0.65f,0.65f}
};

static void simulate();

static void render() {
	static const unsigned simFrameTime = 1000/60;
	static unsigned begin = glutGet(GLUT_ELAPSED_TIME);
	unsigned timeNow = glutGet(GLUT_ELAPSED_TIME);
	unsigned passed = timeNow - begin;
	instantFrameTime = passed*0.001f;
	while (passed>=simFrameTime)	{
		passed-=simFrameTime;
		simulate();		
		begin = timeNow;
		break;
	}


	glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

	glClearColor(0.3f, 0.6f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// view Matrix
    nuLookAt(vMatrix,cameraPos[0],cameraPos[1],cameraPos[2],targetPos[0],targetPos[1],targetPos[2],0,1,0);
	Teapot_SetViewMatrixAndLightDirection(vMatrix,lightDirection);

    Teapot_Enable_ColorMaterial();
    //Teapot_Enable_MeshOutline();

	Teapot_PreDraw();
	
	//Teapot_SetColorSpecular(float R, float G, float B, float SHI);
	unsigned objs = 0;
    float m[16];    // model matrix


	// Render boxes.
	for (unsigned i = 0; i < colliders.boxes.count; ++i) {
		unsigned body = colliders.boxes.transforms[i].body;
		
        float scale[3] = {1,1,1};   // we leave scale outside of the model matrix (but the original code still works...)
		float rotation[4];
		float position[3];
		
        //memcpy(scale, colliders.boxes.data[i].size, sizeof(scale));   // moved down


		quaternion_concat(rotation, bodies.transforms[body].rotation, colliders.boxes.transforms[i].rotation);
		quaternion_transform(position, bodies.transforms[body].rotation, colliders.boxes.transforms[i].position);
		
		position[0] += bodies.transforms[body].position[0];
		position[1] += bodies.transforms[body].position[1];
		position[2] += bodies.transforms[body].position[2];
		
        matrix(m, scale, rotation, position);
	
        memcpy(scale, colliders.boxes.data[i].size, sizeof(scale)); // Now scale is set


		const int ci = (++objs)%NUM_COLORS;
		Teapot_SetColor(Colors[ci][0],Colors[ci][1],Colors[ci][2],1.f);
        Teapot_SetScaling(2*scale[0],2*scale[1],2*scale[2]);    // factor 2 is because all the Teapot_Meshes' height is one (with a few notable exceptions)
        if (i==0)   {
            // ground object
#           ifdef TEAPOT_SHADER_SPECULAR
            Teapot_SetColorSpecular(0.12,0.05,0.12,0.05);
#           endif
            Teapot_Draw(m,TEAPOT_MESH_CUBIC_GROUND);    // we an use TEAPOT_MESH_CUBE, but TEAPOT_MESH_CUBIC_GROUND has a more tesselated top face (5x5 verts, better for vertex-lighting/fog)
#           ifdef TEAPOT_SHADER_SPECULAR
            Teapot_SetColorSpecular(0.05,0.35,0.25,20);        // For all the other objects
#           endif
        }
        else Teapot_Draw(m,TEAPOT_MESH_CUBE);


	}
	
	// Render spheres.
	for (unsigned i = 0; i < colliders.spheres.count; ++i) {
		unsigned body = colliders.spheres.transforms[i].body;
		
        float scale[3]={1,1,1};
		float rotation[4];
		float position[3];
		
        //scale[0] = scale[1] = scale[2] = colliders.spheres.data[i].radius;    // moved down
		
		quaternion_concat(rotation, bodies.transforms[body].rotation, colliders.spheres.transforms[i].rotation);
		quaternion_transform(position, bodies.transforms[body].rotation, colliders.spheres.transforms[i].position);
		
		position[0] += bodies.transforms[body].position[0];
		position[1] += bodies.transforms[body].position[1];
		position[2] += bodies.transforms[body].position[2];
		
        matrix(m, scale, rotation, position);

        scale[0] = scale[1] = scale[2] = colliders.spheres.data[i].radius;    // Now scale is set

		const int ci = (++objs)%NUM_COLORS;
		Teapot_SetColor(Colors[ci][0],Colors[ci][1],Colors[ci][2],1.f);		
		Teapot_SetScaling(2,2,2);
        Teapot_SetScaling(2*scale[0],2*scale[1],2*scale[2]);    // factor 2 is because all the Teapot_Meshes' height is one (with a few notable exceptions)
        Teapot_Draw(m,TEAPOT_MESH_SPHERE2); // factor 2 is because all the Teapot_Meshes' height is one (with a few notable exceptions)
	}
	
	Teapot_PostDraw();

}

#ifndef NUM_SIMULATION_STEPS
#   define NUM_SIMULATION_STEPS 2
#endif
#ifndef NUM_SIMULATION_ITERATIONS
#   define NUM_SIMULATION_ITERATIONS 20
#endif

void simulate() {
    static const unsigned steps = NUM_SIMULATION_STEPS;
    static const unsigned iterations = NUM_SIMULATION_ITERATIONS;
	
	float time_step = 1.0f / (60.0f * (float)steps);
	
	for (unsigned n = 0; n < steps; ++n) {
		// Setup a temporary memory arena. The same temporary memory is reused each iteration.
		nudge::Arena temporary = arena;
		
		// Find contacts.
		nudge::BodyConnections connections = {}; // NOTE: Custom constraints should be added as body connections.
		nudge::collide(&active_bodies, &contact_data, bodies, colliders, connections, temporary);
		
		// NOTE: Custom contacts can be added here, e.g., against the static environment.
		
		// Apply gravity and damping.
		float damping = 1.0f - time_step*0.25f;
		
		for (unsigned i = 0; i < active_bodies.count; ++i) {
			unsigned index = active_bodies.indices[i];
			
			bodies.momentum[index].velocity[1] -= 9.82f * time_step;
			
			bodies.momentum[index].velocity[0] *= damping;
			bodies.momentum[index].velocity[1] *= damping;
			bodies.momentum[index].velocity[2] *= damping;
			
			bodies.momentum[index].angular_velocity[0] *= damping;
			bodies.momentum[index].angular_velocity[1] *= damping;
			bodies.momentum[index].angular_velocity[2] *= damping;
		}
		
		// Read previous impulses from contact cache.
		nudge::ContactImpulseData* contact_impulses = nudge::read_cached_impulses(contact_cache, contact_data, &temporary);
		
		// Setup contact constraints and apply the initial impulses.
		nudge::ContactConstraintData* contact_constraints = nudge::setup_contact_constraints(active_bodies, contact_data, bodies, contact_impulses, &temporary);
		
		// Apply contact impulses. Increasing the number of iterations will improve stability.
		for (unsigned i = 0; i < iterations; ++i) {
			nudge::apply_impulses(contact_constraints, bodies);
			// NOTE: Custom constraint impulses should be applied here.
		}
		
		// Update contact impulses.
		nudge::update_cached_impulses(contact_constraints, contact_impulses);
		
		// Write the updated contact impulses to the cache.
		nudge::write_cached_impulses(&contact_cache, contact_data, contact_impulses);
		
		// Move active bodies.
		nudge::advance(active_bodies, bodies, time_step);
	}
}

/*static void timer(int) {
	glutPostRedisplay();
	glutTimerFunc(16, timer, 0);
	simulate();
}*/

#ifndef ARENA_SIZE
#define ARENA_SIZE (64*1024*1024)
#endif
#ifndef ARENA_SIZE_ALIGNMENT
#define ARENA_SIZE_ALIGNMENT 4096
#endif

#ifndef WINDOW_WIDTH
#define WINDOW_WIDTH 1024
#endif
#ifndef WINDOW_HEIGHT
#define WINDOW_HEIGHT 600
#endif

static void updateCameraPos() {
	const float distanceY = sin(cameraPitch)*cameraDistance;
	cameraPos[1] = targetPos[1] + distanceY;
	const float distanceXZ = cos(cameraPitch)*cameraDistance;
	cameraPos[0] = targetPos[0] + sin(cameraYaw)*distanceXZ;
	cameraPos[2] = targetPos[2] + cos(cameraYaw)*distanceXZ;
}

void GlutSpecialKeys(int key,int /*x*/,int /*y*/)
{
    const int mod = glutGetModifiers();
    if (!(mod&GLUT_ACTIVE_CTRL))	{
        switch (key) {
        case GLUT_KEY_LEFT:
        case GLUT_KEY_RIGHT:
            cameraYaw+= instantFrameTime*(key==GLUT_KEY_LEFT ? -4.0f : 4.0f);
			if (cameraYaw>M_PI) cameraYaw-=2*M_PI;
			else if (cameraYaw<=-M_PI) cameraYaw+=2*M_PI;
            updateCameraPos();		break;
        case GLUT_KEY_UP:
        case GLUT_KEY_DOWN:
            cameraPitch+= instantFrameTime*(key==GLUT_KEY_UP ? 2.f : -2.f);
			if (cameraPitch>M_PI-0.001f) cameraPitch=M_PI-0.001f;
            else if (cameraPitch<-M_PI*0.05f) cameraPitch=-M_PI*0.05f;
            updateCameraPos();
            break;
        case GLUT_KEY_PAGE_UP:
        case GLUT_KEY_PAGE_DOWN:
            cameraDistance+= instantFrameTime*(key==GLUT_KEY_PAGE_DOWN ? 50.0f : -50.0f);
			if (cameraDistance<1.f) cameraDistance=1.f;
            updateCameraPos();
            break;
        }
    }
    else if (mod&GLUT_ACTIVE_CTRL) {
        switch (key) {
        case GLUT_KEY_LEFT:
        case GLUT_KEY_RIGHT:            
        case GLUT_KEY_UP:
        case GLUT_KEY_DOWN:
        {
            // Here we move targetPos and cameraPos at the same time

            // We must find a pivot relative to the camera here (ignoring Y)
            float forward[3] = {targetPos[0]-cameraPos[0],0,targetPos[2]-cameraPos[2]};
            float up[3] = {0,1,0};
            float left[3];

            normalize(forward);
            cross(left,up,forward);

            float delta[3] = {0,0,0};
            if (key==GLUT_KEY_LEFT || key==GLUT_KEY_RIGHT) {
                float amount = instantFrameTime*(key==GLUT_KEY_RIGHT ? -25.0f : 25.0f);
                for (int i=0;i<3;i++) delta[i]+=amount*left[i];
            }
            else {
                float amount = instantFrameTime*(key==GLUT_KEY_DOWN ? -25.0f : 25.0f);
                for (int i=0;i<3;i++) delta[i]+=amount*forward[i];
            }
            for (int i=0;i<3;i++) {
                targetPos[i]+=delta[i];
                cameraPos[i]+=delta[i];
            }
        }
        break;
        case GLUT_KEY_PAGE_UP:
        case GLUT_KEY_PAGE_DOWN:
            // We use world space coords here.
            targetPos[1]+= instantFrameTime*(key==GLUT_KEY_PAGE_DOWN ? -25.0f : 25.0f);
            if (targetPos[1]<-50.f) targetPos[1]=-50.f;
            else if (targetPos[1]>500.f) targetPos[1]=500.f;
            updateCameraPos();
		break;
        }
    }
}




static void GlutDrawGL(void)		{render();glutSwapBuffers();}
static void GlutIdle(void)			{glutPostRedisplay();}
static void GlutFakeDrawGL(void) 	{glutDisplayFunc(GlutDrawGL);}

int main(int argc, const char* argv[]) {
#ifndef __EMSCRIPTEN__
	// Disable denormals for performance.
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif //__EMSCRIPTEN__	

	// Print information about instruction set.
#ifdef __AVX2__
	printf("Using 8-wide AVX\n");
#else
	printf("Using 4-wide SSE\n");
#if defined(__SSE4_1__) || defined(__AVX__)
	printf("BLENDVPS: Enabled\n");
#else
	printf("BLENDVPS: Disabled\n");
#endif
#endif
	
#ifdef __FMA__
	printf("FMA: Enabled\n");
#else
	printf("FMA: Disabled\n");
#endif
	
	// Allocate memory for simulation arena.
	arena.size = ARENA_SIZE;
	arena.data = _mm_malloc(arena.size, ARENA_SIZE_ALIGNMENT);


	// Allocate memory for bodies, colliders, and contacts.
	active_bodies.capacity = max_box_count;
	active_bodies.indices = static_cast<uint16_t*>(_mm_malloc(sizeof(uint16_t)*max_body_count, 64));
	
	bodies.idle_counters = static_cast<uint8_t*>(_mm_malloc(sizeof(uint8_t)*max_body_count, 64));
	bodies.transforms = static_cast<nudge::Transform*>(_mm_malloc(sizeof(nudge::Transform)*max_body_count, 64));
	bodies.momentum = static_cast<nudge::BodyMomentum*>(_mm_malloc(sizeof(nudge::BodyMomentum)*max_body_count, 64));
	bodies.properties = static_cast<nudge::BodyProperties*>(_mm_malloc(sizeof(nudge::BodyProperties)*max_body_count, 64));
	
	colliders.boxes.data = static_cast<nudge::BoxCollider*>(_mm_malloc(sizeof(nudge::BoxCollider)*max_box_count, 64));
	colliders.boxes.tags = static_cast<uint16_t*>(_mm_malloc(sizeof(uint16_t)*max_box_count, 64));
	colliders.boxes.transforms = static_cast<nudge::Transform*>(_mm_malloc(sizeof(nudge::Transform)*max_box_count, 64));
	
	colliders.spheres.data = static_cast<nudge::SphereCollider*>(_mm_malloc(sizeof(nudge::SphereCollider)*max_sphere_count, 64));
	colliders.spheres.tags = static_cast<uint16_t*>(_mm_malloc(sizeof(uint16_t)*max_sphere_count, 64));
	colliders.spheres.transforms = static_cast<nudge::Transform*>(_mm_malloc(sizeof(nudge::Transform)*max_sphere_count, 64));
	
	contact_data.capacity = max_body_count*64;
	contact_data.bodies = static_cast<nudge::BodyPair*>(_mm_malloc(sizeof(nudge::BodyPair)*contact_data.capacity, 64));
	contact_data.data = static_cast<nudge::Contact*>(_mm_malloc(sizeof(nudge::Contact)*contact_data.capacity, 64));
	contact_data.tags = static_cast<uint64_t*>(_mm_malloc(sizeof(uint64_t)*contact_data.capacity, 64));
	contact_data.sleeping_pairs = static_cast<uint32_t*>(_mm_malloc(sizeof(uint32_t)*contact_data.capacity, 64));
	
	contact_cache.capacity = max_body_count*64;
	contact_cache.data = static_cast<nudge::CachedContactImpulse*>(_mm_malloc(sizeof(nudge::CachedContactImpulse)*contact_cache.capacity, 64));
	contact_cache.tags = static_cast<uint64_t*>(_mm_malloc(sizeof(uint64_t)*contact_cache.capacity, 64));
	
	// The first body is the static world.
	bodies.count = 1;
	bodies.idle_counters[0] = 0;
	bodies.transforms[0] = identity_transform;
	memset(bodies.momentum, 0, sizeof(bodies.momentum[0]));
	memset(bodies.properties, 0, sizeof(bodies.properties[0]));
	
	// Add ground.
	{
		unsigned collider = colliders.boxes.count++;
		
		colliders.boxes.transforms[collider] = identity_transform;
		colliders.boxes.transforms[collider].position[1] -= 20.0f;
		
		colliders.boxes.data[collider].size[0] = 400.0f;
		colliders.boxes.data[collider].size[1] = 10.0f;
		colliders.boxes.data[collider].size[2] = 400.0f;
		colliders.boxes.tags[collider] = collider;
	}
	
	// Add boxes.
	for (unsigned i = 0; i < NUM_BOXES; ++i) {
		float sx = (float)rand() * (1.0f/(float)RAND_MAX) + 0.5f;
		float sy = (float)rand() * (1.0f/(float)RAND_MAX) + 0.5f;
		float sz = (float)rand() * (1.0f/(float)RAND_MAX) + 0.5f;
		
		unsigned body = add_box(8.0f*sx*sy*sz, sx, sy, sz);
		
		bodies.transforms[body].position[0] += (float)rand() * (1.0f/(float)RAND_MAX) * 10.0f - 5.0f;
		bodies.transforms[body].position[1] += (float)rand() * (1.0f/(float)RAND_MAX) * 300.0f;
		bodies.transforms[body].position[2] += (float)rand() * (1.0f/(float)RAND_MAX) * 10.0f - 5.0f;
	}
	
	// Add spheres.
	for (unsigned i = 0; i < NUM_SPHERES; ++i) {
		float s = (float)rand() * (1.0f/(float)RAND_MAX) + 0.5f;
		
		unsigned body = add_sphere(4.18879f*s*s*s, s);
		
		bodies.transforms[body].position[0] += (float)rand() * (1.0f/(float)RAND_MAX) * 10.0f - 5.0f;
		bodies.transforms[body].position[1] += (float)rand() * (1.0f/(float)RAND_MAX) * 300.0f;
		bodies.transforms[body].position[2] += (float)rand() * (1.0f/(float)RAND_MAX) * 10.0f - 5.0f;
	}
	
	// Start GLUT.
	glutInit(&argc, const_cast<char**>(argv));
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutCreateWindow("nudge_no_ffp");
		
#	ifdef USE_GLEW
    GLenum err = glewInit();
    if( GLEW_OK != err ) {
       fprintf(stderr, "\nError initializing GLEW: %s\n", glewGetErrorString(err) );
       return 1;
    }
#	endif

    glutSpecialFunc(GlutSpecialKeys);
    glutDisplayFunc(GlutFakeDrawGL);
	glutIdleFunc(GlutIdle);
    glutReshapeFunc(resize);
 
    printf("\nKEYS:\n");
    printf("AROW KEYS + PAGE_UP/PAGE_DOWN:\tmove camera (optionally with CTRL down)\n");
 	
	updateCameraPos();
	
	Teapot_Init();
	
	glutMainLoop();
	
	Teapot_Destroy();

	return 0;
}
