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
// clang++ -O2 main_no_ffp_with_shadows.cpp -o example_no_ffp_with_shadows -I"./" -I"../" ../nudge.cpp -lglut -lGL
// (Optionally with -march=haswell)
// Or:
// g++ main_no_ffp_with_shadows.cpp -o example_no_ffp_with_shadows -I"./" -I"../" ../nudge.cpp -lglut -lGL
// (Optionally with -march=haswell)
// [AFAIR g++ version 5.4.0 20160609 (Ubuntu 5.4.0-6ubuntu1~16.04.4) does not work correctly with -O2 or -O3]

// Emscripten
//------------
// em++ -O2 -D"NUM_SIMULATION_STEPS=1" -D"NUM_SIMULATION_ITERATIONS=5" -D"NUM_BOXES=16" -D"NUM_SPHERES=8" -D"MAX_BODY_COUNT=32" -D"ARENA_SIZE=(128*1024)"  -D"WINDOW_WIDTH=960" -D"WINDOW_HEIGHT=540" -fno-rtti -fno-exceptions -msse2 -o html/nudge_with_shadows.html main_no_ffp_with_shadows.cpp ../nudge.cpp -I"./" -I"../" -s LEGACY_GL_EMULATION=0 -lglut --closure 1
// Then run html/nudge_with_shadows.html locally with Firefox or by using a web server for other browsers.

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
float targetPos[3] = {0,-3,0};     // This demo places the ground plane below zero
float cameraYaw = 2*M_PI;
float cameraPitch = M_PI*0.125f;
float cameraDistance = 70;
float cameraPos[3] = {0,0,0};       // Derived value (do not edit)
float lightDirection[3] = {1,1,2};	// Will be normalized
float vMatrix[16];                  // view matrix

// pMatrix data:
float pMatrix[16];                  // projection matrix
const float pMatrixFovDeg = 45.f;
const float pMatrixNearPlane = 0.5f;
const float pMatrixFarPlane = 200.0f;

float instantFrameTime = 16.2f;

// "dynamic_resolution.h" implements the first shadow mapping step and optionally dynamic resolution (that by default should keep frame rate > config.dynamic_resolution_target_fps)
//#define DYNAMIC_RESOLUTION_USE_GLSL_VERSION_330   // (Optional) Not sure it's faster...
#define DYNAMIC_RESOLUTION_IMPLEMENTATION           // Mandatory in 1 source file (.c or .cpp)
#include "dynamic_resolution.h"

// teapot.h--------------------------------
//#define TEAPOT_CENTER_MESHES_ON_FLOOR           // (Optional) Otherwise meshes are centered in their local aabb center
//#define TEAPOT_INVERT_MESHES_Z_AXIS           // (Optional) Otherwise meshes look in the opposite Z direction
//#define TEAPOT_SHADER_USE_NORMAL_MATRIX       // Not needed in this demo
#define TEAPOT_SHADER_SPECULAR
#define TEAPOT_SHADER_FOG
#define TEAPOT_SHADER_FOG_HINT_FRAMENT_SHADER   // Better fog on ground plane (but a bit expensive)
#define TEAPOT_SHADER_USE_SHADOW_MAP
#define TEAPOT_ENABLE_FRUSTUM_CULLING           // (Optional) a bit expensive, and does not cull 100% hidden objects. You'd better test if it works and if it's faster...
#define TEAPOT_IMPLEMENTATION
#include "teapot.h"
// ----------------------------------------


#ifndef NUM_BOXES
#define NUM_BOXES 1024*4
#endif

#ifndef NUM_SPHERES
#define NUM_SPHERES 1024*4-1 //512*6
#endif

#ifndef MAX_BODY_COUNT
#define MAX_BODY_COUNT (NUM_BOXES+NUM_SPHERES+1)//+512)//2048
#endif

static const unsigned max_body_count = MAX_BODY_COUNT;
static const unsigned max_box_count = NUM_BOXES+1;      // +1 = ground box
static const unsigned max_sphere_count = NUM_SPHERES;

static Teapot_MeshData* allocated_memory = NULL;    // This is allocated in InitGL() and freed in DestroyGL()
static Teapot_MeshData* pMeshData[MAX_BODY_COUNT];
static const int maxNumMeshData = MAX_BODY_COUNT;
static int numMeshData = NUM_BOXES+NUM_SPHERES+1;


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
// custom replacement of glOrtho(...)
static void nuOrtho(float res[16],float left,float right, float bottom, float top,float nearVal,float farVal) {
    const float eps = 0.0001f;
    float Drl = (right-left);
    float Dtb = (top-bottom);
    float Dfn = (farVal-nearVal);
    if (Drl==0) {right+=eps;left-=eps;Drl=right-left;}
    if (Dtb==0) {top+=eps;bottom-=eps;Dtb=top-bottom;}
    if (Dfn==0) {farVal+=eps;nearVal-=eps;Dfn=farVal-nearVal;}

    res[0]  = 2.f/Drl;
    res[1]  = 0;
    res[2]  = 0;
    res[3] = 0;

    res[4]  = 0;
    res[5]  = 2.f/Dtb;
    res[6]  = 0;
    res[7] = 0;

    res[8]  = 0;
    res[9]  = 0;
    res[10] = -2.f/Dfn;
    res[11] = 0;

    res[12]  = -(right+left)/Drl;
    res[13]  = -(top+bottom)/Dtb;
    res[14] = (farVal+nearVal)/Dfn;
    res[15] = 1;
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

        Dynamic_Resolution_Resize(w,h);    // The dynamic resolution texture (and the shadow map) change their size with this call
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

#ifndef NUM_SIMULATION_STEPS
#   define NUM_SIMULATION_STEPS 1
#endif
#ifndef NUM_SIMULATION_ITERATIONS
#   define NUM_SIMULATION_ITERATIONS 5
#endif
static void simulate(float timeStep=1.f/60.f, unsigned numSubSteps=NUM_SIMULATION_STEPS, unsigned numIterations=NUM_SIMULATION_ITERATIONS);


static void render() {
	static const unsigned simFrameTime = 1000/60;
	static unsigned begin = glutGet(GLUT_ELAPSED_TIME);
	unsigned timeNow = glutGet(GLUT_ELAPSED_TIME);
	unsigned passed = timeNow - begin;
	instantFrameTime = passed*0.001f;
    int numSubsteps = 0;
    while (passed>=simFrameTime)	{
        passed-=simFrameTime;
        begin = timeNow;
        if (++numSubsteps>=NUM_SIMULATION_STEPS) break;
	}
    // Simulate and Update render object transforms
    if (numSubsteps>0)   {
        simulate(1.f/60.f,numSubsteps);
        static float mMatrix[16];const float one[3]={1.f,1.f,1.f};
        for (int i=0;i<numMeshData;i++) {
            Teapot_MeshData* md = pMeshData[i];
            const nudge::Transform& T = *((nudge::Transform*) md->userPtr);
            matrix(mMatrix,one,T.rotation,T.position);
            Teapot_MeshData_SetMMatrix(md,mMatrix);
        }
    }


	// view Matrix
    nuLookAt(vMatrix,cameraPos[0],cameraPos[1],cameraPos[2],targetPos[0],targetPos[1],targetPos[2],0,1,0);
	Teapot_SetViewMatrixAndLightDirection(vMatrix,lightDirection);


#   ifdef TEAPOT_SHADER_USE_SHADOW_MAP
    // Draw to Shadow Map------------------------------------------------------------------------------------------
    {
    // We're currently calculating all these matrices every frame. This is obviously wrong.
    // Also: there's no fixed rule I know to calculate these matrices. Feel free to change them!
    static float lpMatrix[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    static float lvMatrix[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    static float lvpMatrix[16]; // = light_pMatrix*light_vMatrix
    static float lvpMatrixFrustum[6][4];
    if (lpMatrix[0]==0)
    {
        // This changes with pMatrixFarPlane and pMatrixFovDeg
        const float y = pMatrixFarPlane*tan(pMatrixFovDeg*3.1415/180.0)*0.35f;  // last coefficient is ad-hoc for this demo (in our case it should be 1.0, or maybe 0.5 for free roaming; something like 0.2 for fixed environment and MUCH better shadow quality!)
        const float x = y;
        nuOrtho(lpMatrix,-x,x,-y,y,pMatrixFarPlane*0.5f,-pMatrixFarPlane*0.5f);
/* POSSIBLE IMPROVEMENTS (https://msdn.microsoft.com/en-us/library/windows/desktop/ee416324(v=vs.85).aspx):
 To calculate the projection, the eight points that make up the view
 frustum are transformed into light space. Next, the minimum and maximum
 values in X and Z are found. These values make up the bounds for an
 orthographic projection [-x,x,-y,y].

 For directional lights, the solution to the moving shadow edges problem is to round the minimum/maximum value in X and Y (that make up the orthographic projection bounds) to pixel size increments. This can be done with a divide operation, a floor operation, and a multiply.
    vLightCameraOrthographicMin /= vWorldUnitsPerTexel;
    vLightCameraOrthographicMin = XMVectorFloor( vLightCameraOrthographicMin );
    vLightCameraOrthographicMin *= vWorldUnitsPerTexel;
    vLightCameraOrthographicMax /= vWorldUnitsPerTexel;
    vLightCameraOrthographicMax = XMVectorFloor( vLightCameraOrthographicMax );
    vLightCameraOrthographicMax *= vWorldUnitsPerTexel;

 The vWorldUnitsPerTexel value is calculated by taking a bound of
 the view frustum, and dividing by the buffer size.

        FLOAT fWorldUnitsPerTexel = fCascadeBound /
        (float)m_CopyOfCascadeConfig.m_iBufferSize;
        vWorldUnitsPerTexel = XMVectorSet( fWorldUnitsPerTexel, fWorldUnitsPerTexel, 						   	0.0f, 0.0f );

Bounding the maximum size of the view frustum results in a looser fit for the orthographic projection.
It is important to note that the texture is 1 pixel larger in width and height when using this technique. This keeps shadow coordinates from indexing outside of the shadow map.

*/
/*
Building the matrix:

How I'd go about the very simple case you have:
You have an orthographic projection for your light-camera. So your frustum is just an Oriented Bounding Box (OBB). That means you can simply feed its world space coordinates (like the width and height) to glm::ortho().

But how do you construct the OBB around the frustum?
Good question. ;D

Here's a simple approach:
First determine the direction of your light (as a normalized vector). Now simply project every vertex of your main-camera's frustum onto that vector and find the nearest and furthest one (simply store the distances along the vector). Now subtract the two distances.
Congratulations! You already have your OBB's depth (Z).
Now repeat that process for the other two vectors. One pointing upwards or downwards (Y) and the other to the right or left (X) relative to your light-camera. Now you have your OBB's orientation (the three vectors) and dimensions. Now simply pass the OBB's dimensions to glm::ortho() and then transform the orthographic matrix so it has the same orientation as your OBB.
You're done. :D

Projecting a point onto a vector:
This step is actually very easy. Just take the dot product between your vector and your point (both stored as vec3).
Example code:

float distance_on_vector = dot(p, vector);

Vector should be normalized, because you need the world-space distance. You don't need the actual position of p in world space (you just need the projected length) to calculate the dimensions of the OBB. That's why the above code is enough.
*/
// https://gamedev.stackexchange.com/questions/73851/how-do-i-fit-the-camera-frustum-inside-directional-light-space
/*
    Calculate the 8 corners of the view frustum in world space. This can be done by using the inverse view-projection matrix to transform the 8 corners of the NDC cube (which in OpenGL is [‒1, 1] along each axis).

    Transform the frustum corners to a space aligned with the shadow map axes. This would commonly be the directional light object's local space. (In fact, steps 1 and 2 can be done in one step by combining the inverse view-projection matrix of the camera with the inverse world matrix of the light.)

    Calculate the bounding box of the transformed frustum corners. This will be the view frustum for the shadow map.

    Pass the bounding box's extents to glOrtho or similar to set up the orthographic projection matrix for the shadow map.

There are a couple caveats with this basic approach. First, the Z bounds for the shadow map will be tightly fit around the view frustum, which means that objects outside the view frustum, but between the view frustum and the light, may fall outside the shadow frustum. This could lead to missing shadows. To fix this, depth clamping can be enabled so that objects in front of the shadow frustum will be rendered with clamped Z instead of clipped. Alternatively, the Z-near of the shadow frustum can be pushed out to ensure any possible shadowers are included.

The bigger issue is that this produces a shadow frustum that continuously changes size and position as the camera moves around. This leads to shadows "swimming", which is a very distracting artifact. In order to fix this, it's common to do the following additional two steps:

    Fix the overall size of the frustum based on the longest diagonal of the camera frustum. This ensures that the camera frustum can fit into the shadow frustum in any orientation. Don't allow the shadow frustum to change size as the camera rotates.

    Discretize the position of the frustum, based on the size of texels in the shadow map. In other words, if the shadow map is 1024×1024, then you only allow the frustum to move around in discrete steps of 1/1024th of the frustum size. (You also need to increase the size of the frustum by a factor of 1024/1023, to give room for the shadow frustum and view frustum to slip against each other.)

If you do these, the shadow will remain rock solid in world space as the camera moves around. (It won't remain solid if the camera's FOV, near or far planes are changed, though.)

As a bonus, if you do all the above, you're well on your way to implementing cascaded shadow maps, which are "just" a set of shadow maps calculated from the view frustum as above, but using different view frustum near and far plane values to place each shadow map.
*/
    }
    if (lvMatrix[15]==0)
    {
        // This changes with lightDirection, pMatrixFarPlane, targetPos (= camera target)
        const float distance =  pMatrixFarPlane*0.1f;
        const float shadowTargetPos[3] = {targetPos[0],0.f,targetPos[2]};   // We keep it at y=0
        const float lpos[3] = {shadowTargetPos[0]-lightDirection[0]*distance,
                               shadowTargetPos[1]-lightDirection[1]*distance,
                               shadowTargetPos[2]-lightDirection[2]*distance};
        nuLookAt(lvMatrix,lpos[0],lpos[1],lpos[2],shadowTargetPos[0],shadowTargetPos[1],shadowTargetPos[2],0,1,0);

        Teapot_Helper_MultMatrix(lvpMatrix,lpMatrix,lvMatrix);
        Teapot_Helper_GetFrustumPlaneEquations(lvpMatrixFrustum,lvpMatrix,0);
    }

    Teapot_HiLevel_DrawMulti_ShadowMap_Vp(pMeshData,numMeshData,lvpMatrix,0.5f);
    //Teapot_HiLevel_DrawMulti_ShadowMap_Vp_WithFrustumCulling(pMeshData,numMeshData,lvpMatrix,lvpMatrixFrustum,0.5f);


    }
#   endif //TEAPOT_SHADER_USE_SHADOW_MAP


    // Render to framebuffer---------------------------------------------------------------------------------------
    Dynamic_Resolution_Bind();  // This defaults to nothing if we don't use dynamic resolution (-> it's for free: we can draw inside it as usual)

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	Teapot_PreDraw();


    // We can add a pivot at the camera target point
    {
        static float mMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
        mMatrix[12]=targetPos[0];mMatrix[13]=targetPos[1];mMatrix[14]=targetPos[2];
        Teapot_SetColor(0.5,0.5,0,1); // the color of the center of the pivot (and for the pivot mesh only there's a hack that transfers the brightness of this color to the whole mesh)
        Teapot_SetScaling(0.4,0.4,0.4);
        // Tip: We can change the outline color/params with Teapot_Set_MeshOutline_XXX(...) methods
        Teapot_Enable_MeshOutline();  // This does not work together with glDisable(GL_DEPTH_TEST);
        //glDisable(GL_DEPTH_TEST);
        Teapot_Draw(mMatrix,TEAPOT_MESH_PIVOT3D);
        //glEnable(GL_DEPTH_TEST);
        Teapot_Disable_MeshOutline();
    }

    // Here we draw all our pMeshData
    Teapot_DrawMulti(pMeshData,numMeshData,0);    // Please note that to handle transparent objects correctly, it can change the object order (see last argument). So to detect an object, just store pointers and don't realloc the initial buffer (see that we have maxNumMeshData>=numMeshData)

	Teapot_PostDraw();

    Dynamic_Resolution_Unbind();

    // Draw to screen at current resolution_factor: ----------------------------------------------------------------
    glDisable(GL_DEPTH_TEST);glDisable(GL_CULL_FACE);glDepthMask(GL_FALSE);
    Dynamic_Resolution_Render(instantFrameTime);    // Mandatory
    glEnable(GL_DEPTH_TEST);glEnable(GL_CULL_FACE);glDepthMask(GL_TRUE);
    //--------------------------------------------------------------------------------------------------------------


}



void simulate(float timeStep,unsigned numSubSteps,unsigned numIterations) {
    const float time_step = timeStep * (float)numSubSteps;

    for (unsigned n = 0; n < numSubSteps; ++n) {
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
        for (unsigned i = 0; i < numIterations; ++i) {
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
#define ARENA_SIZE (24*MAX_BODY_COUNT*MAX_BODY_COUNT/4)//(64*1024*1024)
#endif
#ifndef ARENA_SIZE_ALIGNMENT
#define ARENA_SIZE_ALIGNMENT 4096
#endif

#ifndef WINDOW_WIDTH
#define WINDOW_WIDTH 1024//16*3 // 192
#endif
#ifndef WINDOW_HEIGHT
#define WINDOW_HEIGHT 600//16*3
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
	

	// Start GLUT.
	glutInit(&argc, const_cast<char**>(argv));
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    glutCreateWindow("nudge_no_ffp_with_shadows");
		
#	ifdef USE_GLEW
    GLenum err = glewInit();
    if( GLEW_OK != err ) {
       fprintf(stderr, "\nError initializing GLEW: %s\n", glewGetErrorString(err) );
       return 1;
    }
#	endif

	
    Dynamic_Resolution_Init(30,1);
    Teapot_Init();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  // Otherwise transparent objects are not displayed correctly
    glClearColor(0.3f, 0.6f, 1.0f, 1.0f);

#   ifdef TEAPOT_SHADER_USE_SHADOW_MAP
    Teapot_SetShadowDarkening(40.f,0.75f);  // First number makes the shadow darker in an uniform way, the second clamps the lower intensity: (40.f,0.75f) default, (0.f,...) -> no shadows
#   endif //TEAPOT_SHADER_USE_SHADOW_MAP
	
    // Allocate teapot_meshes
    allocated_memory = (Teapot_MeshData*) malloc(maxNumMeshData*sizeof(Teapot_MeshData));
    for (int i=0;i<maxNumMeshData;i++) {
        pMeshData[i] = &allocated_memory[i];   // Mandatory Assignment (to split our allocated memory)
        Teapot_MeshData_Clear(pMeshData[i]);
    }

    // The first body is the static world.
    bodies.count = 1;
    bodies.idle_counters[0] = 0;
    bodies.transforms[0] = identity_transform;
    memset(bodies.momentum, 0, sizeof(bodies.momentum[0]));
    memset(bodies.properties, 0, sizeof(bodies.properties[0]));

    Teapot_MeshData* md = NULL;
    float mMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    const float one[3] = {1.f,1.f,1.f};

    // Add ground.
    {
        unsigned collider = colliders.boxes.count++;

        colliders.boxes.transforms[collider] = identity_transform;
        colliders.boxes.transforms[collider].position[1] -= 10.0f;
        colliders.boxes.transforms[collider].position[0] =
        colliders.boxes.transforms[collider].position[2] = 0.0f;

        colliders.boxes.data[collider].size[0] = 50.0f;
        colliders.boxes.data[collider].size[1] = 5.0f;
        colliders.boxes.data[collider].size[2] = 50.0f;
        colliders.boxes.tags[collider] = collider;

        // Teapot_h stuff
        md = pMeshData[bodies.count-1];
        Teapot_MeshData_SetMeshId(md,TEAPOT_MESH_CUBIC_GROUND);
        const int ci = (bodies.count-1)%NUM_COLORS;
        Teapot_MeshData_SetColor(md,Colors[ci][0],Colors[ci][1],Colors[ci][2],1.f);
        Teapot_MeshData_SetScaling(md,2.f*colliders.boxes.data[collider].size[0],2.f*colliders.boxes.data[collider].size[1],2.f*colliders.boxes.data[collider].size[2]);
        Teapot_MeshData_SetOutlineEnabled(md,0);
#       ifdef TEAPOT_SHADER_SPECULAR
        Teapot_MeshData_SetColorSpecular(md,0.12,0.05,0.12,0.05);
#       endif
        md->userPtr = &colliders.boxes.transforms[collider];    // Important: will simplify things at runtime

        const nudge::Transform& T = *((nudge::Transform*) md->userPtr);
        matrix(mMatrix,one,T.rotation,T.position);
        Teapot_MeshData_SetMMatrix(md,mMatrix);
    }

    // Add boxes.
    for (unsigned i = 0; i < NUM_BOXES; ++i) {
        float sx = (float)rand() * (1.0f/(float)RAND_MAX) + 0.5f;
        float sy = (float)rand() * (1.0f/(float)RAND_MAX) + 0.5f;
        float sz = (float)rand() * (1.0f/(float)RAND_MAX) + 0.5f;

        unsigned body = add_box(12.0f*sx*sy*sz, sx, sy, sz);
        unsigned collider = colliders.boxes.count-1;

        bodies.transforms[body].position[0] += (float)rand() * (1.0f/(float)RAND_MAX) * 10.0f - 5.0f;
        bodies.transforms[body].position[1] += (float)rand() * (1.0f/(float)RAND_MAX) * 300.0f;
        bodies.transforms[body].position[2] += (float)rand() * (1.0f/(float)RAND_MAX) * 10.0f - 5.0f;

        // Teapot_h stuff
        md = pMeshData[bodies.count-1];
        Teapot_MeshData_SetMeshId(md,TEAPOT_MESH_CUBE);
        const int ci = (bodies.count-1)%NUM_COLORS;
        Teapot_MeshData_SetColor(md,Colors[ci][0],Colors[ci][1],Colors[ci][2],1.f);
        Teapot_MeshData_SetScaling(md,2.f*colliders.boxes.data[collider].size[0],2.f*colliders.boxes.data[collider].size[1],2.f*colliders.boxes.data[collider].size[2]);
        Teapot_MeshData_SetOutlineEnabled(md,0);
#       ifdef TEAPOT_SHADER_SPECULAR
        Teapot_MeshData_SetColorSpecular(md,0.05,0.35,0.25,20);        // For all the other objects
#       endif
        md->userPtr = &bodies.transforms[body];    // Important: will simplify things at runtime

        const nudge::Transform& T = *((nudge::Transform*) md->userPtr);
        matrix(mMatrix,one,T.rotation,T.position);
        Teapot_MeshData_SetMMatrix(md,mMatrix);
    }

    // Add spheres.
    for (unsigned i = 0; i < NUM_SPHERES; ++i) {
        float s = (float)rand() * (1.0f/(float)RAND_MAX) + 0.5f;

        unsigned body = add_sphere(8.18879f*s*s*s, s);
        unsigned collider = colliders.spheres.count-1;

        bodies.transforms[body].position[0] += (float)rand() * (1.0f/(float)RAND_MAX) * 10.0f - 5.0f;
        bodies.transforms[body].position[1] += (float)rand() * (1.0f/(float)RAND_MAX) * 300.0f;
        bodies.transforms[body].position[2] += (float)rand() * (1.0f/(float)RAND_MAX) * 10.0f - 5.0f;

        // Teapot_h stuff
        md = pMeshData[bodies.count-1];
        Teapot_MeshData_SetMeshId(md,TEAPOT_MESH_SPHERE2);
        const int ci = (bodies.count-1)%NUM_COLORS;
        Teapot_MeshData_SetColor(md,Colors[ci][0],Colors[ci][1],Colors[ci][2],1.f);
        Teapot_MeshData_SetScaling(md,2.f*colliders.spheres.data[collider].radius,2.f*colliders.spheres.data[collider].radius,2.f*colliders.spheres.data[collider].radius);
        Teapot_MeshData_SetOutlineEnabled(md,0);
#       ifdef TEAPOT_SHADER_SPECULAR
        Teapot_MeshData_SetColorSpecular(md,0.05,0.35,0.25,20);        // For all the other objects
#       endif
        md->userPtr = &bodies.transforms[body];    // Important: will simplify things at runtime

        matrix(mMatrix,one,colliders.spheres.transforms[body].rotation,colliders.spheres.transforms[body].position);
        Teapot_MeshData_SetMMatrix(md,mMatrix);
    }


    glutSpecialFunc(GlutSpecialKeys);
    glutDisplayFunc(GlutFakeDrawGL);
    glutIdleFunc(GlutIdle);
    glutReshapeFunc(resize);

    printf("\nKEYS:\n");
    printf("AROW KEYS + PAGE_UP/PAGE_DOWN:\tmove camera (optionally with CTRL down)\n");

    updateCameraPos();

	glutMainLoop();
	
    // Deallocate memory
    if (allocated_memory) {free(allocated_memory);allocated_memory=NULL;}
    for (int i=0;i<maxNumMeshData;i++) pMeshData[i]=NULL;

	Teapot_Destroy();
    Dynamic_Resolution_Destroy();

	return 0;
}
