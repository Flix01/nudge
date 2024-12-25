# Nudge Physics Library

## Introduction

Nudge is a small data-oriented and SIMD-optimized 3D rigid body physics library created by Rasmus Barringer in 2017 ([GitHub](https://github.com/rasmusbarr/nudge)).

This 2024 version ([GitHub](https://github.com/Flix01/nudge/tree/master/new_version)) is just an **attempt** to ease user experience, by embedding part of the original demo code into the library itself, and extending some of its functionalities.

> **Note:** Most of the physics-related stuff has been kept intact, so the new version does not improve stuff in this area.

## Features
- Single file `nudge.h`
- Only box and sphere colliders (i.e. collision shapes) supported (but every body can be a compound of colliders)
- Max number of colliders: 8192
- Static, kinematic and dynamic bodies
- Collision groups and collision masks
- Kinematic animation support
- Doxygen documentation

## Usage

### Getting Started
<details>
<summary>Here is a code snippet to get started with the library (users can easily extend it using the TODO sections)</summary>

```cpp
// file: example01.cpp
// g++ example01.cpp -I../ -I./ -march=native -O3 -Wall -o example01
#define NUDGE_IMPLEMENTATION // [TODO 0] better do this in another cpp file to speed up recompilations
#include "nudge.h"

int main() {
    using namespace nudge;

    // Display helpful info
    show_info();

    // Initialize the context
    context_t c = {}; // reset it
    init_context(&c);

    // Add bodies
    Transform T = {}; T.position[1] = 40.f;
    unsigned body = add_sphere(&c, 1.f, 0.5f, &T); // returns the 'permanent' index to the physic body

    // Program main loop
    while (1) {
        double elapsed_time_from_previous_frame_in_seconds = 1.0f / 60.f; // [TODO 1] get the seconds elapsed from the previous (graphic) frame here somehow

        // Update simulation
        const unsigned substeps = pre_simulation_step(&c, elapsed_time_from_previous_frame_in_seconds);   // mandatory call (substeps are the number of physic frames that are going to be performed in simulation_step(...))
        if (substeps > 0) {
            // here you can move manually kinematic bodies for example, using nudge::TransformAssignToBody(...)
        }
        simulation_step(&c);  // mandatory call (main function of the library)

        // Read back bodies
        for (unsigned body = 0; body < c.bodies.count; body++) {
            const Transform* T = &c.bodies.transforms[body];
            printf("[physic frame: %llu] [body:%u] pos: {%1.3f,%1.3f,%1.3f}\n", c.simulation_params.num_frames, body, T->position[0], T->position[1], T->position[2]);

            // or just draw the body using a smoothed 16-float column-major matrix:
            // float mMatrix[16]; calculate_graphic_transform_for_body(&c, body, mMatrix);
            // [TODO 2] place the code to draw 'body' at model matrix 'mMatrix' here
        }

        if (c.simulation_params.num_frames > 120) break; // [TODO 3] break the loop when user presses ESC somehow
    }

    // Free the context
    destroy_context(&c);

    printf("Exiting...\n"); fflush(stdout);

    return 0;
}
```
</details>

## FAQ
<details>
<summary><B>The sample application is crashing. Why?</B></summary>

&nbsp;
Most likely, your CPU doesn't support AVX2 and/or FMA. The project files are set to compile with AVX2 and FMA support and you need to disable it in build settings.
- **Xcode:** Set "Enable Additional Vector Extensions" to your supported level. Remove `-mfma` and `-mno-fma4` from "Other C Flags".
- **Visual Studio:** Set "Enable Enhanced Instruction Set" under code generation to your supported level. Remove `__FMA__` from the preprocessor definitions.

</details>

<details>
<summary><B>How can I add other colliders (i.e. collision shapes), so that I can use height maps, convex and concave meshes, cylinders, capsules, etc.? And how can I add constraints between bodies?</B></summary>

&nbsp;
...I suggest you use another physics library!  
In any case, the new (2024) version was made mainly to ease the nudge API, and to expose properties (like friction) that were hard-coded before.  
Extending the physics-related stuff is not a purpose of this work.

If you have some experience in physics-engine programming, maybe you could try extending the original version: it allows some extension possibility even in the example code (without touching `nudge.h` at all)!  
Also, it might be helpful to read this (old) [link](https://rasmusbarr.github.io/blog/dod-physics.html) from the original author.

</details>

