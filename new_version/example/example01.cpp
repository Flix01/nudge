// file: example01.cpp [note: this is the Getting Started example present in the documentation]
// g++ example01.cpp -I../ -I./ -march=native -O3 -Wall -o example01
#define NUDGE_IMPLEMENTATION // [TODO 0] better do this in another cpp file to speed up recompilations
#include "nudge.h"

int main() {
    using namespace nudge;

    // Display helpful info
    show_info();

    // Initialize the context
    context_t c = {};	// reset it
    init_context(&c);

    // Add bodies
    Transform T = identity_transform;T.position[1]=40.f;
    unsigned body = add_sphere(&c,1.f,0.5f,&T); // returns the 'permanent' index to the physic body

    // Program main loop
    while (1) {
        double elapsed_time_from_previous_frame_in_seconds = 1.0f/60.f; // [TODO 1] get the seconds elapsed from the previous (graphic) frame here somehow

        // Update simulation
        const unsigned substeps = pre_simulation_step(&c,elapsed_time_from_previous_frame_in_seconds);   // mandatory call (substeps are the number of physic frames that are going to be performed in simulation_step(...))
        if (substeps>0) {
              // here you can move manually kinematic bodies for example, using TransformAssignToBody(...)
        }
        simulation_step(&c);  // mandatory call (main function of the library)

        // Read back bodies
        for (unsigned body=0;body<c.bodies.count;body++) {
            const Transform* T = &c.bodies.transforms[body];
            printf("[physic frame: %llu] [body:%u] pos: {%1.3f,%1.3f,%1.3f}\n",c.simulation_params.num_frames,body,T->position[0],T->position[1],T->position[2]);

            // or just draw the body using a smoothed 16-float column-major matrix:
            // float mMatrix[16];calculate_graphic_transform_for_body(&c,body,mMatrix);
            // [TODO 2] place the code to draw 'body' at model matrix 'mMatrix' here
        }

        if (c.simulation_params.num_frames>120) break; // [TODO 3] break the loop when user presses ESC somehow
    }

    // Free the context
    destroy_context(&c);

    printf("Exiting...\n");fflush(stdout);

    return 0;
 }
 
