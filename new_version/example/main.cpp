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

// Modified by Flix01 in 2024 to use the new version of nudge
// The code just shows the new API (no new feature has been used)

// Can be compiled with the provided build files, but also with:
// c++ -march=native -O3 -Wall -fno-rtti -fno-exceptions -I../ ./stdafx.cpp ./main.cpp -o example -lglut -lGL

#include "stdafx.h"
#include <assert.h>
/*#include <nudge.h>
#include <immintrin.h>*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __APPLE__
#include <GLUT/GLUT.h>
#include <OpenGL/gl.h>
#elif defined(_WIN32)
#include <GLUT/glut.h>
#include <gl/gl.h>
#else // Linux.
#include <GL/glut.h>
#include <GL/gl.h>
#endif


static const nudge::Transform identity_transform = { {}, 0, { 0.0f, 0.0f, 0.0f, 1.0f } };

static nudge::context_t nudge_context = {};
static nudge::context_t* c = &nudge_context;	// shorter... we'll use this


struct {
	int use_graphic_transform;
	
	double inner_torus_radius,outer_torus_radius;
	unsigned num_torii,num_boxes_per_torus;
} globals = {1,0.925,2.75,8,8};


static void render() {
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	
	glClearColor(0.6f, 0.8f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// Setup projection.
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	glMatrixMode(GL_PROJECTION);
	
	float fov = 0.25f;
	float aspect = (float)viewport[2]/(float)viewport[3];
	float near_z = 1.0f;
	float far_z = 1000.0f;
	{
		float dy = 2.0f * near_z * tanf(fov);
		float zn = 2.0f * near_z;
		float dz = far_z - near_z;
		
		float m[16] = {
			zn / (dy * aspect), 0.0f, 0.0f, 0.0f,
			0.0f, zn / dy, 0.0f, 0.0f,
			0.0f, 0.0f, (-far_z - near_z) / dz, -1.0f,
			0.0f, 0.0f, (-zn * far_z) / dz, 0.0f,
		};
		
		glLoadMatrixf(m);
	}
	
	glTranslatef(0.0f, 0.0f, -75.0f);
	
	// Switch to model matrix.
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	// Setup light.
	GLfloat light_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat light_diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	GLfloat light_direction[] = { 1.0f, 1.0f, 1.0f, 0.0f };
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, light_direction);
	
	
	static const float colors[][3]={{0.5,0.7,0.5},{0.7,0.5,0.5},{0.5,0.5,0.7},{0.7,0.7,0.5},{0.7,0.6,0.5},
									{0.5,1.0,0.5},{1.0,0.5,0.5},{0.5,0.5,1.0},{1.0,1.0,0.5},{1.0,0.75,0.5}};
	static const int num_colors = sizeof(colors)/sizeof(colors[0]);
	static const float sleeping_color[3] = {0.25,0.25,0.25};
	static const float com_offset_body_color[3] = {0.0,0.0,0.7}; // we use a different color for bodies with a center of mass offset
	
	glPushMatrix();
	float mMatrix[16];unsigned num_sleeping_and_dynamic=0;
	for (unsigned body=0,body_count=c->bodies.count;body<body_count;body++)	{
		const nudge::BodyFilter* filter = &c->bodies.filters[body];	// BodyFilter hosts per-body flags, collision masks and sleeping state
        const int is_sleeping_and_dynamic = c->bodies.idle_counters[body]==0xFF && filter->flags&nudge::BF_IS_DYNAMIC;num_sleeping_and_dynamic+=is_sleeping_and_dynamic;
		const int has_com_offset = filter->flags&nudge::BF_HAS_COM_OFFSET;
	
		const float* color =  !is_sleeping_and_dynamic?(has_com_offset?&com_offset_body_color[0]:&colors[body%num_colors][0]):&sleeping_color[0];
		glColor3f(color[0],color[1],color[2]);

		const int use_graphic_transform = globals.use_graphic_transform;	

        const nudge::BodyLayout* layout = &c->bodies.layouts[body];	// BodyLayout hosts the indices of the body colliders (i.e. collision shapes). The global arrays of colliders are in c->colliders.boxes and c->colliders.spheres
		//glPushMatrix();
		if (use_graphic_transform)	{
			// Use use_graphic_transform=1 in release mode, where you have a single graphic mesh 
			// (simulated using 1 or more boxes/spheres). We end out having a single smoothed
			// transform per body, and we use it to draw our graphic mesh.
			
			// Here we just draw one box or one sphere
			nudge::calculate_graphic_transform_for_body(c,body,mMatrix);	// what this does is: 1) smoothes the graphic transform (using DeltaTimes less than timeStep)
																			// 2) inglobes the com_offset (if present), so that we should not manually translate our graphic mesh
			glLoadMatrixf(mMatrix);
            if (layout->num_boxes>0) {
                if (layout->num_boxes==1) {
                    assert(layout->first_box_index>=0 && layout->num_boxes==1 && layout->num_spheres==0 && layout->first_sphere_index==-1);
                    assert((uint16_t)layout->first_box_index<c->colliders.boxes.count);
                    const nudge::BoxCollider* coll = &c->colliders.boxes.data[layout->first_box_index];
					glScalef(coll->size[0],coll->size[1],coll->size[2]);
					glutSolidCube(2.0f);
				}
                else if (layout->num_boxes==globals.num_boxes_per_torus)	{
					// it's a torus				
                    assert(layout->first_box_index>=0 && layout->num_spheres==0 && layout->first_sphere_index==-1);
                    assert((uint16_t)layout->first_box_index+layout->num_boxes<=c->colliders.boxes.count);
					//const nudge::BoxCollider* coll = &c->colliders.boxes.data[info->first_box_index];	// first box shape (of globals.num_boxes_per_torus)
					//const nudge::Transform* T = &c->colliders.boxes.transforms[info->first_box_index];	// first relative transform (of globals.num_boxes_per_torus)
                    // the idea was to use 'coll->size[]' and/or 'T' to extract a scaling that works...
					// ... but in the end I stored the dims in 'globals' at the top of the file
					glScalef(1.f,1.f,1.f);
					glutSolidTorus(globals.inner_torus_radius,	// GLdouble innerRadius
					               globals.outer_torus_radius,	// GLdouble outerRadius
                    			   16,	// GLint nsides
                    			   16); // GLint rings)
				}					
			}
            else if (layout->num_spheres>0) {
                assert(layout->first_sphere_index>=0 && layout->num_spheres==1 && layout->num_boxes==0 && layout->first_box_index==-1);
                assert((uint16_t)layout->first_sphere_index<c->colliders.spheres.count);
                const nudge::SphereCollider* coll = &c->colliders.spheres.data[layout->first_sphere_index];
				glutSolidSphere(coll->radius, 16, 8); 
			}
		}
		else {
			// Use use_graphic_transform=0 in debug mode, where you don't have a graphic mesh 
            // or you want to see all the boxes/spheres that make up the body.
			// The transforms here are not smoothed.						
			const nudge::Transform* T1 = &c->bodies.transforms[body];
            if (layout->num_boxes>0) {
                assert(layout->first_box_index>=0);
                assert((uint16_t)layout->first_box_index+layout->num_boxes<=c->colliders.boxes.count);
                for (uint16_t i=layout->first_box_index,isz=layout->first_box_index+layout->num_boxes;i<isz;i++)	{
					const nudge::Transform* T2 = &c->colliders.boxes.transforms[i];
					const nudge::Transform T = nudge::TransformMul(*T1,*T2);
					const nudge::BoxCollider* coll = &c->colliders.boxes.data[i];			
					nudge::TransformToMat4(mMatrix,&T);					
					glLoadMatrixf(mMatrix);
					glScalef(coll->size[0],coll->size[1],coll->size[2]);
					glutSolidCube(2.0f);
				}				
			}
            if (layout->num_spheres>0) {
                assert(layout->first_sphere_index>=0);
                assert((uint16_t)layout->first_sphere_index+layout->num_spheres<=c->colliders.spheres.count);
                for (uint16_t i=layout->first_sphere_index,isz=layout->first_sphere_index+layout->num_spheres;i<isz;i++)	{
					const nudge::Transform* T2 = &c->colliders.spheres.transforms[i];
					const nudge::Transform T = nudge::TransformMul(*T1,*T2);
					const nudge::SphereCollider* coll = &c->colliders.spheres.data[i];			
					nudge::TransformToMat4(mMatrix,&T);
					glLoadMatrixf(mMatrix);
					glutSolidSphere(coll->radius, 16, 8); 
				}				
			}			
		
		}
		//glPopMatrix();
	}
	glPopMatrix();
	
	glutSwapBuffers();
	
	// this is the dbg line I use to monitor the weird (wrong?) behavior of the sleeping bodies (probably introduced by my mod, and not present in the original nudge library [TO CHECK]).
	//if (num_sleeping_and_dynamic>0) {printf("[Frame:%llu] num_sleeping_bodies:%u (num_active_bodies:%u)\n",c->simulation_params.num_frames,num_sleeping_and_dynamic,c->active_bodies.count);fflush(stdout);}
	// wrong! [2814] 1526 (1532) -> sleeping objects are not removed from the active list. Why?

    // [CHECK] Checked the original nudge library: given that the original library had all dynamic bodies except body=0 that was the static ground (no kinematic bodies):
    // -> sleeping dynamic bodies ARE indeed present in the active_bodies list (=> are not removed from the active list)
    // -> The static ground (body=0) is not present in the active_bodies list
    // So the original behavior is similar to this one, although here static/kinematic bodies are present in the active_bodies list as well
    // And I've tried to keep static/kinematic bodies out of the active_bodies list, but collision detection is not reliable anymore (bodies can sink through the ground, expecially the character in example02.cpp)
    // So this is probably the best we can get so far
}

static void simulate() {
	static double time_begin = double(glutGet(GLUT_ELAPSED_TIME))*0.001;
	const double time_now = double(glutGet(GLUT_ELAPSED_TIME))*0.001;
	const double elapsed_time = time_now-time_begin;
	time_begin = time_now;
	
	// simulation
	const unsigned numSubSteps = nudge::pre_simulation_step(c,elapsed_time);
	if (numSubSteps) {
		nudge::simulation_step(c);
	}
}

static void timer(int) {
	glutPostRedisplay();
	glutTimerFunc(16, timer, 0);
	simulate();
}


void onKeyPressed(unsigned char key,int x, int y)	{
	globals.use_graphic_transform = !globals.use_graphic_transform;
}

int main(int argc, const char* argv[]) {
	nudge::show_info();	
	nudge::init_context_with(c,1024,512);
	// these 2 must be tuned depending on the masses and the sizes of the bodies in the world
	c->simulation_params.sleeping_threshold_linear_velocity_squared = 10.f;// default value: (1e-2f)
	c->simulation_params.sleeping_threshold_angular_velocity_squared = 200.f;// default value: (1e-1f)
	// P.S. it seems that in nudge sleeping bodies can still move a bit until their whole island go to sleep.
	// Not sure if it's a bug, or it's caused by my mods or not... the 'num_active_bodies' don't decrease 
	// with the increase of 'num_sleeping_bodies', but at a certain point 'num_active_bodies' becomes zero and 
	// everything is properly sleeping (= still).

	unsigned body;
	nudge::Transform T = identity_transform;

	// Add ground.
	T.position[1]=-20.f;
	body = nudge::add_box(c,0,400.f,10.f,400.f,&T);assert(body!=NUDGE_INVALID_BODY_ID);
	T = identity_transform;
	
	// Add boxes, but first reserve some box colliders for the torii we'll insert later.
	unsigned num_boxes_reserved_for_torii = globals.num_torii*globals.num_boxes_per_torus;
	if (num_boxes_reserved_for_torii>c->MAX_NUM_BOXES-1) num_boxes_reserved_for_torii=c->MAX_NUM_BOXES-1;
	num_boxes_reserved_for_torii = num_boxes_reserved_for_torii - num_boxes_reserved_for_torii%globals.num_boxes_per_torus;
	for (unsigned i = 0; i < c->MAX_NUM_BOXES-1-num_boxes_reserved_for_torii; ++i) {
		float sx = (float)rand() * (1.0f/(float)RAND_MAX) + 0.5f;
		float sy = (float)rand() * (1.0f/(float)RAND_MAX) + 0.5f;
		float sz = (float)rand() * (1.0f/(float)RAND_MAX) + 0.5f;
		T = identity_transform;
		T.position[0] += (float)rand() * (1.0f/(float)RAND_MAX) * 10.0f - 5.0f;
		T.position[1] += (float)rand() * (1.0f/(float)RAND_MAX) * 300.0f;
		T.position[2] += (float)rand() * (1.0f/(float)RAND_MAX) * 10.0f - 5.0f;
		if (i<c->MAX_NUM_BOXES*3/4)
			body = nudge::add_box(c,8.0f*sx*sy*sz, sx, sy, sz, &T);
		else {
			const float com_offset[3]={0.f,-sy*0.75f,0.f};
			body = nudge::add_box(c,8.0f*sx*sy*sz, sx, sy, sz, &T, com_offset);
		}
		assert(body!=NUDGE_INVALID_BODY_ID);
	}
	
	// Add spheres.
	for (unsigned i = 0; i < c->MAX_NUM_SPHERES; ++i) {
		float s = (float)rand() * (1.0f/(float)RAND_MAX) + 0.5f;
		T = identity_transform;
		T.position[0] += (float)rand() * (1.0f/(float)RAND_MAX) * 10.0f - 5.0f;
		T.position[1] += (float)rand() * (1.0f/(float)RAND_MAX) * 300.0f;
		T.position[2] += (float)rand() * (1.0f/(float)RAND_MAX) * 10.0f - 5.0f;
		if (i<c->MAX_NUM_SPHERES*3/4)
			body = nudge::add_sphere(c,4.18879f*s*s*s, s, &T);
		else {
			const float com_offset[3]={0.f,-s*0.5f,0.f};
			body = nudge::add_sphere(c,4.18879f*s*s*s, s, &T, com_offset);
		}
		assert(body!=NUDGE_INVALID_BODY_ID);
	}
	
	// Add torii using compound shapes (we use z as axis, to match glutSolidTorus(...))
	globals.num_torii = nudge::colliders_get_num_remaining_boxes(c)/globals.num_boxes_per_torus;    
	T = identity_transform;const float base_pos_y = (float)rand() * (1.0f/(float)RAND_MAX) * 375.0f;		
    for (unsigned i=0;i<globals.num_torii;i++)    {
    	const float r = globals.inner_torus_radius;
        const float R = globals.outer_torus_radius;      // torus major radius
        float torusMass = 10.5f*R*r;
        float torusInertia[3];nudge::calculate_torus_inertia(torusInertia,torusMass,R,r,nudge::AXIS_Z);
                
		T.position[0] = (float)rand() * (1.0f/(float)RAND_MAX) * 10.0f - 5.0f;
		T.position[1] = (float)rand() * (1.0f/(float)RAND_MAX) * 50.0f-25.f + base_pos_y;
		T.position[2] += (float)rand() * (1.0f/(float)RAND_MAX) * 10.0f - 5.0f;                
                
        // We use nudge::Transforms here (but we could have used normal float16 matrices instead)
        const int num_boxes = globals.num_boxes_per_torus;    // bigger R, needs bigger num_boxes
        nudge::Transform csT[num_boxes];
        float hsizeTriplets[num_boxes*3];
        const float boxLength = R * sin(M_PI/(float)num_boxes);
        for (int l=0;l<num_boxes;l++)   {
            // set hsizeTriplets[l*3] and csT[l]
            float* triplet = &hsizeTriplets[l*3];
            triplet[0]= boxLength;  triplet[1]=triplet[2]= r;

            nudge::Transform* tr = &csT[l];
            const float angle = (float)l*2.f*M_PI/(float)num_boxes;
            const float sinAngle = sin(angle), cosAngle = cos(angle);
            nudge::nm_QuatFromAngleAxis(tr->rotation,angle,0.f,0.f,1.f);	// I've added some functions from my minimath.h library (hence the nm_ prefix)
            tr->position[2]=0.f;tr->position[0]=R*sinAngle;tr->position[1]=-R*cosAngle;
        }
        body = nudge::add_compound(c,torusMass,torusMass>0.f?torusInertia:NULL,num_boxes,hsizeTriplets,csT,0,NULL,NULL,&T);
    }	
		
	
	// Start GLUT.
	glutInit(&argc, const_cast<char**>(argv));
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(1024, 600);
	glutCreateWindow("nudge");
	glutKeyboardFunc(onKeyPressed);
	glutDisplayFunc(render);
	
	timer(0);
	
	glutMainLoop();
		
	nudge::destroy_context(c);
	
	return 0;
}
