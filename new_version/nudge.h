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

// Modified, refactored and documented by Flix01 (https://github.com/Flix01/nudge/tree/master/new_version) in 2024
// [I'm not a physics engine expert at all, the mods I've made are just to ease my usage scenario]
// [I've also probably decreased performance and broken something, so if you're a
// physics engine expert, I suggest you base your mods on the original version]



/**
 * \mainpage Nudge Physics Library Documentation
 *
 * \section intro_sec Introduction
 *
 * Nudge is a small data-oriented and SIMD-optimized 3D rigid body physics library created by Rasmus Barringer in 2017 (https://github.com/rasmusbarr/nudge).
 *
 * This 2024 version (https://github.com/Flix01/nudge/tree/master/new_version subfolder) is just an attempt
 * to ease user experience, by embedding part of the original demo code in the library itself,
 * and extending some of its functionalities.
 *
 * \note Most of the physics-related stuff has been kept intact, so the new version does not improve stuff in this area.
 *
 *
 * \section features_sec Features
 * - Single file nudge.h
 * - Only box and sphere colliders (i.e. collision shapes) supported (but every body can be a compound of colliders)
 * - Max number of colliders: 8192
 * - Static, kinematic and dynamic bodies
 * - Collision groups and collision masks
 * - Kinematic animation support
 * - Doxygen documentation
 *
 * \section building_sec Building
 * - Being a header-only library, all that is required is to define NUDGE_IMPLEMENTATION in a .cpp file before including "nudge.h"
 * - The library works only with SIMD enabled: the recommended requirements are AVX2 and FMA; the minimum requirement is just SSE2 (AFAIR). In many cases adding something like -march=native in the compiler options is enough (in g++/clang++ syntax): -march=haswell is probably better for independent builds.
 * - \note Running a SIMD-compiled program on hardware where SIMD is not supported is likely to cause RANDOM CRASHES.
 * - \note When using emscripten, -msimd128 must be added to the other command-line (simd) options (please read \link https://emscripten.org/docs/porting/simd.html [HERE] \endlink). However not all browsers support SIMD by default (without activating it someway): that's one of the main reasons of random crashes inside browsers.
 * - \note The \link https://github.com/simd-everywhere/simde [SIMDE] \endlink library can be used to compile and run nudge replacing or removing SIMD support. One way to remove SIMD (through SIMDE) is to replace the SIMD compilation options with (g++/clang syntax): -DUSE_SIMDE -DSIMDE_NO_NATIVE (optionally with: -DSIMDE_ENABLE_OPENMP -fopenmp-simd) [TODO: check if this still works] (of course there's a performance penalty without SIMD).
 * - The library documentation (recommended) can be generated with the doxygen command launched from the same folder as the Doxyfile file.
 *
 * \section usage_sec Usage
 * \subsection example_code Example Code
 * Here is a code snippet to get started with the library (users can easily extend it using the TODO sections):
 *
 * \code
 * // file: example01.cpp
 * // g++ example01.cpp -I../ -I./ -march=native -O3 -Wall -o example01
 * #define NUDGE_IMPLEMENTATION // [TODO 0] better do this in another cpp file to speed up recompilations
 * #include "nudge.h"
 *
 * int main() {
 *     using namespace nudge;
 *
 *     // Display helpful info
 *     show_info();
 *
 *     // Initialize the context
 *     context_t c = {}; // reset it
 *     init_context(&c);
 *
 *     // Add bodies
 *     Transform T = identity_transform;T.position[1]=40.f;
 *     unsigned body = add_sphere(&c,1.f,0.5f,&T); // returns the 'permanent' index to the physic body
 *
 *     // Program main loop
 *     while (1) {
 *        double elapsed_time_from_previous_frame_in_seconds = 1.0f/60.f; // [TODO 1] get the seconds elapsed from the previous (graphic) frame here somehow
 *
 *        // Update simulation
 *        const unsigned substeps = pre_simulation_step(&c,elapsed_time_from_previous_frame_in_seconds);   // mandatory call (substeps are the number of physic frames that are going to be performed in simulation_step(...))
 *        if (substeps>0) {
 *              // here you can move manually kinematic bodies for example, using nudge::TransformAssignToBody(...)
 *        }
 *        simulation_step(&c);  // mandatory call (main function of the library)
 *
 *        // Read back bodies
 *        for (unsigned body=0;body<c.bodies.count;body++) {
 *          const Transform* T = &c.bodies.transforms[body];
 *          printf("[physic frame: %llu] [body:%u] pos: {%1.3f,%1.3f,%1.3f}\n",c.simulation_params.num_frames,body,T->position[0],T->position[1],T->position[2]);
 *
 *          // or just draw the body using a smoothed 16-float column-major matrix:
 *          // float mMatrix[16];calculate_graphic_transform_for_body(&c,body,mMatrix);
 *          // [TODO 2] place the code to draw 'body' at model matrix 'mMatrix' here
 *        }
 *
 *        if (c.simulation_params.num_frames>120) break; // [TODO 3] break the loop when user presses ESC somehow
 *     }
 *
 *     // Free the context
 *     destroy_context(&c);
 *
 *     printf("Exiting...\n");fflush(stdout);
 *
 *     return 0;
 * }
 * \endcode
 *
 *
 * \section faq_sec FAQ
 * - The sample application is crashing. Why?
 *
 *   Most likely, your CPU doesn't support AVX2 and/or FMA. The project files are set to compile with AVX2 and FMA support and you need to disable it in build settings.
 *   Xcode: Set "Enable Additional Vector Extensions" to your supported level. Remove -mfma and -mno-fma4 from "Other C Flags".
 *   Visual Studio: Set "Enable Enhanced Instruction Set" under code generation to your supported level. Remove __FMA__ from the preprocessor definitions.
 *
 * - How can I add other colliders (i.e. collision shapes), so that I can use height maps, convex and concave meshes, cylinders, capsules, etc.? And how can I add constraints between bodies?
 *
 *   ...I suggest you use another physics library!
 *   In any case, the new (2024) version was made mainly to ease the nudge API, and to expose properties (like friction) that were hard-coded before.
 *   Extending the physics-related stuff is not a purpose of this work.
 *
 *   If you have some experience in physics-engine programming, maybe you could try extending the original version: it allows some extension possibility even in the example code (without touching nudge.h at all)!
 *   Also it might be helpful to read this (old) \link https://rasmusbarr.github.io/blog/dod-physics.html [link] \endlink from the original author.
 *
 * - How do collision masks work?
 *
 *   Well, the way collision groups and masks are implemented is very efficient, but a bit difficult to understand, because it allows incoherent conditions.
 *   Every body belongs to a single collision group, and owns a collision mask of all the groups the body should collide with. An example of incoherent condition if the following:
 *   \code
 *   // c nudge context ptr
 *   unsigned a,b;  // set these to 2 body indices
 *   // we could have used 'body_set_collision_group_and_mask(...)' here:
 *   c->bodies.filters[a].collision_group = nudge::COLLISION_GROUP_A;c->bodies.filters[a].collision_mask = nudge::COLLISION_GROUP_ALL&(~nudge::COLLISION_GROUP_B);
 *   c->bodies.filters[b].collision_group = nudge::COLLISION_GROUP_B;c->bodies.filters[b].collision_mask = nudge::COLLISION_GROUP_ALL;
 *   // => a doesn't want to collide with b, but b wants to collide with a
 *   \endcode
 *   How incoherent conditions are handled depends on the optional definition NUDGE_COLLISION_MASKS_CONSISTENT (undefined by default, can be defined before the NUDGE_IMPLEMENTATION definition).
 *   By default, in the code above, a and b do not collide. Please note that by always using coherent conditions, collision behavior should not depend on the NUDGE_COLLISION_MASKS_CONSISTENT definition at all.
 */



#ifndef NUDGE_H
#define NUDGE_H

#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>	// log function declaration
#ifdef NUDGE_USER_CFG_FILE_NAME
#   include NUDGE_USER_CFG_FILE_NAME // optional definition for advanced users (to be placed in the project options or in the compiler commandline, not inside code files). Remember to escape quotes (e.g. -DNUDGE_USER_CFG_FILE_NAME="\"nudge_user_cfg.h\"" on the commandline)
#endif
#ifndef NUDGE_NO_STDIO
#	include <stdio.h>
#endif
#ifndef __cplusplus
#   error nudge.h is a c++ file and should be compiled as c++
#elif __cplusplus < 201103L
#   define NUDGE_NO_CPP11_DETECTED
#   define NUDGE_CONSTEXPR const
#   define NUDGE_STATIC_ASSERT(X,MESSAGE) assert(X) /*this needs <assert.h>, but I don't want to include it here*/
#   undef NUDGE_USE_INT32_ENUMS
#   define NUDGE_USE_INT32_ENUMS    // if set: bigger enums (more space for body flags and collision groups: the BodyFilter struct is 3x bigger), but worse cache performance (note that bit operations are generally not faster with smaller types)
//  Also note that there are still a few anonymous structs (that by standard require C++11 compilation),
//  but in my tests both g++ and clang++ work just fine with -std=c++98
//  In case of problems please define NUDGE_NO_ANONYMOUS_STRUCTS
#else // c++11 detected
#   define NUDGE_CONSTEXPR constexpr /*never used in nudge.h*/
#   define NUDGE_STATIC_ASSERT(X,MESSAGE) static_assert((X), MESSAGE) /*no <assert.h> required; never used in nudge.h*/
#endif  // __cplusplus

#ifdef NUDGE_USE_INT32_ENUMS
#   undef NUDGE_COLLISION_MASK_TYPE
#   define NUDGE_COLLISION_MASK_TYPE uint32_t
#   undef NUDGE_FLAG_MASK_TYPE
#   define NUDGE_FLAG_MASK_TYPE uint32_t
#else //NUDGE_USE_INT32_ENUMS
#   ifndef NUDGE_COLLISION_MASK_TYPE
#       define NUDGE_COLLISION_MASK_TYPE uint8_t
#   endif
#   ifndef NUDGE_FLAG_MASK_TYPE
#       define NUDGE_FLAG_MASK_TYPE uint16_t
#   endif
#endif // NUDGE_USE_INT32_ENUMS

namespace nudge {

    /**
     * @brief The Arena struct used internally
     * @note Devs: technically we could allow multiple \ref context_t "nudge contexts" running on the same thread to share the same arena to minimize memory usage, but this is not a priority (and in most cases multiple contexts are used in parallel)
     */
    struct Arena {
        void* data;
        uintptr_t size;
    };

    /**
     * @brief Storage struct for user data (by default used inside \ref context_t "context_t"): a per-context 64-bit user space in 11 different variable names that share the same space, so that ONLY one of them must be chosen and used
     */
    union UserData64Bit{
        void* ptr;int64_t i64;uint64_t u64;double f64;int32_t i32[2];uint32_t u32[2];float f32[2];
        int16_t i16[4];uint16_t u16[4];int8_t i8[8];uint8_t u8[8];
    };
    /**
     * @brief Storage struct for user data (by default used inside \ref BodyInfo "BodyInfo"): a per-body 32-bit user space in 7 different variable names that share the same space, so that ONLY one of them must be chosen and used
     */
    union UserData32Bit{int32_t i32;uint32_t u32;float f32;int16_t i16[2];uint16_t u16[2];int8_t i8[4];uint8_t u8[4];};

    /**
     * @brief The Transform struct
     * @note This struct has been rearranged to use anomymous unions for improved flexibility, but now initializations must be done with and additional pair of curly parenthesis, otherwise some compiler can issue a warning; for example: Transform T = { {{0,0,0}} , {0} , {{0,0,0,1}} };
     * @note The 'vector' and 'quaternion' fields have been added only to allow partial assignments like: T2.quaternion = T1.quaternion;
     * @note By default this struct contains nested anonymous structs that allow to use {.px,.px,.pz} and {.rx,.rx,.rz,.rw}/{.qx,.qx,.qz,.qw} fields: this requires the C++11 standard (but in my tests g++ and clang++ with -std=c++98 is enough)
     * @note There is a definition named NUDGE_NO_ANONYMOUS_STRUCTS that can be used to remove anonymous structs, that usually require -std=c++11 (even if g++ and clang++ seem to work just fine with -std=c++98 in my tests)
     */
    struct Transform {
        union {
            float position[3];  /**< position applied before rotation */
            float p[3];
            struct {float x,y,z;} vector;
#           ifndef NUDGE_NO_ANONYMOUS_STRUCTS
            struct {float px,py,pz;};
#           endif
        };
        union {
            uint32_t body;  /**< the body index (used mainly inside colliders to retrieve the body) */
            float time;     /**< only used inside KinematicData key frames */
        };
        union {
            float rotation[4];  /**< the orientation in quaternion form */
            float r[4];
            float q[4];
            struct {float x,y,z,w;} quaternion;
#           ifndef NUDGE_NO_ANONYMOUS_STRUCTS
            struct {float rx,ry,rz,rw;};
            struct {float qx,qy,qz,qw;};
#           endif
        };
    };
    /**
     * @brief The BodyProperties struct
     */
    struct BodyProperties {
        float inertia_inverse[3]; /**< the inertia tensor inverse (@see inertia) (each component is always positive or null, even if the body is kinematic) */
        float mass_inverse; /**< the inverse of the mass of the body (it's always positive or null, even if the body is kinematic) */
        float gravity[3];   /**< the body gravity; default value: {0,-9.82,0} */
        float friction;     /**< the body friction; default value: 1.0 */
    };
    /**
     * @brief The BodyMomentum struct
     */
	struct BodyMomentum {
        float velocity[3];  /**< the body linear velocity in world space */
        float unused0;      /**< padding value used internally (better not touch) */
        float angular_velocity[3];  /**< the body angular velocity in world space */
        float unused1;      /**< padding value (this is not used AFAIK) */
	};

    /**
     * @brief The SphereCollider struct
     */
	struct SphereCollider {
        float radius;   /**< the radius of the sphere collider */
	};
    /**
     * @brief The BoxCollider struct
     */
	struct BoxCollider {
        float size[3];  /**< the half dimensions of the box collider */
        float unused;   /**< padding value used internally (better not touch) */
	};
    /**
     * @brief The Contact struct
     */
	struct Contact {
        float position[3];  /**< position of the contact in world space */
        float penetration;  /**< amount of penetration */
        float normal[3];    /**< contact normal in world space (not sure if this makes sense: it should be positive or negative according to the order of the two bodies, shouldn't it?) */
        float friction;     /**< contact friction */
	};
    /**
     * @brief The BodyPair struct
     */
	struct BodyPair {
        uint16_t a; /**< index of body a */
        uint16_t b; /**< index of body b */
	};
    /**
     * @brief The ContactData class
     */
	struct ContactData {
        Contact* data;  /**< array of Contact structs of size count */
        BodyPair* bodies;   /**< array of BodyPair structs of size count */
        uint64_t* tags; /**< array of uint64_t tags of size count. Each tag contains the two uint16_t tags of the colliders involved in the contact */
        uint32_t capacity;  /**< the capacity of the arrays */
        uint32_t count; /**< the number of contacts */
		
        uint32_t* sleeping_pairs;   /**< [something related to sleeping] */
        uint32_t sleeping_count;    /**< the number of sleeping pairs */
	};
    /**
     * @brief This struct is used to access all the colliders in the physic world
     * @note Colliders are shuffled in their two arrays, so it's not safe to store indices (but colliders relative to a single body are kept contiguos in their two arrays). Every collider has an (automatic) unique tag to reference it.
     */
	struct ColliderData {
        struct {
            uint16_t* tags; /**< array of the collider unique identifiers of size count (do not touch) */
            BoxCollider* data; /**< array of BoxCollider structs of size count */
            Transform* transforms;  /**< array of Transform structs of size count, also used to get the body this collider belongs to */
            uint32_t count; /**< the number of box colliders */
        } boxes;    /**< Anonymous struct instance used to access all the box colliders */
		
		struct {
            uint16_t* tags; /**< array of the collider unique identifiers of size count (do not touch) */
            SphereCollider* data; /**< array of SphereCollider structs of size count */
            Transform* transforms;    /**< array of Transform structs of size count, also used to get the body this collider belongs to */ // probably a simple position is enough here... possible optimization?
            uint32_t count; /**< the number of sphere colliders */
        } spheres;    /**< Anonymous struct instance used to access all the shpere colliders */
	};

    /**
     * @brief The unsigned type used for the \ref CollisionMaskEnum "COLLISION_GROUP_ flags"; it defaults to uint8_t (i.e. 8 groups available) if C++11 is supported and NUDGE_USE_INT32_ENUMS is not defined, and to uint32_t otherwise (i.e. 32 groups available)
     * @note Always using this typedef for collision group flags is recommended
     * @note When C++11 is available the type can be further tuned with the NUDGE_COLLISION_MASK_TYPE definition (having smaller types has benefits on the cache size, not much on the bit operation speed)
     */
    typedef NUDGE_COLLISION_MASK_TYPE CollisionMask;

    /**
     * @brief The CollisionMaskEnum enum
     * @note If more groups are required, please see the \ref CollisionMask "CollisionMask doc" for the available space and add the additional entries in your code: namespace nudge {const CollisionMask COLLISION_GROUP_H=1<<8,COLLISION_GROUP_I=1<<9,...;}
     */
    enum CollisionMaskEnum
#   ifndef NUDGE_USE_INT32_ENUMS
    : CollisionMask
#   endif
    {
        COLLISION_GROUP_DEFAULT   =     1<<0,
        COLLISION_GROUP_A   = 1<<1,
        COLLISION_GROUP_B   = 1<<2,
        COLLISION_GROUP_C   = 1<<3,
        COLLISION_GROUP_D   = 1<<4,
        COLLISION_GROUP_E   = 1<<5,
        COLLISION_GROUP_F   = 1<<6,
        COLLISION_GROUP_G   = 1<<7,
        COLLISION_GROUP_ALL = (CollisionMask)(-1)
    };

    /**
     * @brief The unsigned type used for the \ref BodyFlagEnum "BF_ flags"; it defaults to uint16_t (i.e. 16 flags available) if C++11 is supported and NUDGE_USE_INT32_ENUMS is not defined, and to uint32_t otherwise (i.e. 32 flags available)
     * @note Always using this typedef for body flags is recommended
     * @note When C++11 is available the type can be further tuned with the NUDGE_FLAG_MASK_TYPE definition (having smaller types has benefits on the cache size, not much on the bit operation speed)
     */
    typedef NUDGE_FLAG_MASK_TYPE FlagMask;

    /**
     * @brief The BodyFlagEnum enum
     * @note User can easily add custom flags this way: namespace nudge {const FlagMask BF_IS_MY_TYPE_1=1<<12,BF_IS_MY_TYPE_2=1<<13,BF_IS_MY_TYPE_3=1<<14,BF_IS_MY_TYPE_4=1<<15;}
     * @note Flags marked [unused, not implemented] will never be implemented inside nudge.h
     * @note To understand how many entries are available, please see the \ref FlagMask "FlagMask doc" (by default they are at least 16)
     */
    enum BodyFlagEnum
#   ifndef NUDGE_USE_INT32_ENUMS
    : FlagMask
#   endif
    {
        BF_HAS_COM_OFFSET = 1<<0, /**< read-only [internal usage: automatically set when bodies are created] */
        BF_IS_DISABLED = 1<<1,  /**< [experimental] if used, it's better to set the body to static, reset its velocities and set its collision group and mask both to zero (TODO: test if this is really necessary) */
        BF_IS_REMOVED = 1<<2,   /**< read-only [internal flag added when bodies are removed] */
        BF_IS_DISABLED_OR_REMOVED = BF_IS_DISABLED|BF_IS_REMOVED,   /**< read-only flag useful to filter out bodies excluded from simulation */
        BF_IS_STATIC = 1<<3,    /**< read-only [internal flag added when bodies are added] */
        BF_IS_KINEMATIC = 1<<4, /**< read-only [internal flag added when bodies are added] */
        BF_IS_DYNAMIC = 1<<5,    /**< read-only [internal flag added when bodies are added] */
        BF_NEVER_SLEEPING = 1<<6, /**< [experimental] affects only dynamic bodies */
        BF_HAS_DIFFERENT_GRAVITY_MODE = 1<<7, /**< [experimental] inverts the \ref GlobalDataMaskEnum "GF_USE_GLOBAL_GRAVITY" mode in \ref GlobalData "c->global_data.flags" on a per-body base */
#       ifndef NUDGE_BODYFLAG_ENUM_NO_UNUSED_FLAGS
        BF_IS_CHARACTER = 1<<8, /**< [unused, not implemented, removable] flag added for user convenience */
        BF_IS_PLATFORM = 1<<9,  /**< [unused, not implemented, removable] flag added for user convenience */
        BF_IS_SENSOR = 1<<10,    /**< [unused, not implemented, removable] flag added for user convenience */
        BF_IS_FRUSTUM_CULLED = 1<<11,    /**< [unused, not implemented, removable] flag added for user convenience */
        BF_IS_DISABLED_OR_REMOVED_OR_FRUSTUM_CULLED = BF_IS_DISABLED_OR_REMOVED|BF_IS_FRUSTUM_CULLED, /**< [unused, not implemented, removable] flag added for user convenience */
#       endif
        BF_IS_STATIC_OR_KINEMATIC = BF_IS_STATIC|BF_IS_KINEMATIC,   /**< read-only [internal flag] */
        BF_IS_STATIC_OR_DYNAMIC = BF_IS_STATIC|BF_IS_DYNAMIC,   /**< read-only [internal flag] */
        BF_IS_KINEMATIC_OR_DYNAMIC = BF_IS_KINEMATIC|BF_IS_DYNAMIC,   /**< read-only [internal flag] */
        BF_IS_STATIC_OR_KINEMATIC_OR_DYNAMIC = BF_IS_STATIC|BF_IS_KINEMATIC|BF_IS_DYNAMIC,   /**< read-only [internal flag] */
        BF_IS_STATIC_OR_KINEMATIC_OR_DISABLED_OR_REMOVED = BF_IS_STATIC_OR_KINEMATIC|BF_IS_DISABLED_OR_REMOVED  /**< read-only [internal flag] */
#       ifdef NUDGE_BODYFLAG_ENUM_EXTRA_FIELDS
#           define NUDGE_BODYFLAG_ENUM_EXTRA_FIELDS
#       endif
    };
    /**
     * @brief The BodyFilter struct
     */
    struct BodyFilter {
        FlagMask flags; /**< a bit masks of \ref BodyFlagEnum "BF_ enums"; most values are read-only, so don't (re)set it, but just add/remove tweakable flags to it */
        CollisionMask collision_group; /**< a SINGLE \ref CollisionMaskEnum "COLLISION_GROUP_ value". Default value: COLLISION_GROUP_DEFAULT (means that the body belongs to this group. Better not use more than one group per body) */
        CollisionMask collision_mask;   /**< a bit mask of \ref CollisionMaskEnum "COLLISION_GROUP_ values". Default value: COLLISION_GROUP_ALL (means that the body can collide with all the groups) */
    };

    /**
     * @brief Per-body struct that contains the indices of the body colliders inside \ref ColliderData "ColliderData::boxes and ColliderData::spheres"
     * @note It's also possible to retrieve the body index from \ref ColliderData "c->colliders" using the bodyId field inside each collider's \ref Transform "Transform"
    */
    struct BodyLayout {
        uint16_t num_boxes; /**< the number of box colliders this body owns */
        int16_t first_box_index; /**< the index of the first box collider in \ref ColliderData "ColliderData::boxes" (box colliders are assumed to be contiguous, with no fragmentation), or -1 */

        uint16_t num_spheres; /**< the number of sphere colliders this body owns */
        int16_t first_sphere_index; /**< the index of the first sphere collider in \ref ColliderData "ColliderData::spheres" (sphere colliders are assumed to be contiguous, with no fragmentation), or -1 */
    };

    /**
     * @brief The BodyInfo struct contains some read-only graphic properties of the body (e.g. axis aligned bounding box and center of mass offset), and plenty of per-body user available space, handy to bind user-side structs to a nudge physic body
     * @note User custom field injection is allowed by the two definitions NUDGE_BODYINFO_STRUCT_EXTRA_FIELDS and NUDGE_BODYINFO_STRUCT_EXTRA_PADDING; the best place to do so is in a nudge config header file (setting its name in the project-scope definition: NUDGE_USER_CFG_FILE_NAME)
     * @note It's possible to remove the default (per-body) UserData32Bit user field of this struct using the definition NUDGE_BODYINFO_STRUCT_NO_USER_DATA, and the 'aux_bodies[...]' array by setting the NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES definition to zero
     */
    struct BodyInfo {
#       ifndef NUDGE_BODYINFO_STRUCT_NO_USER_DATA
        UserData32Bit user; /**< user data: a per-body user space in 7 different variable names that share the same space, so that ONLY one of them must be chosen and used */
#       endif // NUDGE_BODYINFO_STRUCT_NO_USER_DATA
#       ifdef NUDGE_BODYINFO_STRUCT_EXTRA_FIELDS
        NUDGE_BODYINFO_STRUCT_EXTRA_FIELDS /**< user stuff injected at the beginning of the BodyInfo struct */
#       endif // NUDGE_BODYINFO_STRUCT_EXTRA_FIELDS
        float aabb_center[3];  /**< [read-only; unused by nudge.h] the axis aligned bounding box center of the body; note that the com_offset has not been stripped from it (it must be summed to strip it from aabb_center) */
        float aabb_half_extents[3];  /**< [read-only; unused by nudge.h] the axis aligned bounding box half extents of the body (it does not depend on com_offset) */
        float com_offset[3]; /**< [read-only] the center of mass offset of the body (it's only used in the \ref calculate_graphic_transform_for_body "calculate_graphic_transform_xxx(...)" functions)  */
        float aabb_enlarged_radius;  /**< [read-only] this radius is the sum of |com_offset|+|aabb_half_extents|, so that it can be used only with the body's transform position (no transform orientation required) */
#       ifndef NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES
#           define NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES (2)  // sizeof(BodyInfo): (2) => 48 bytes; (4) => 52 bytes; (6) => 56 bytes; (8) => 60 bytes; (10) => 64 bytes
#       endif // NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES
#       if NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES>0
        union {
            int16_t aux_bodies[NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES]; /**< user data that can be used for dynamic bodies ONLY, and that are reset to -1 every frame (but only for dynamic bodies in the \ref ActiveBodies "c->active_bodies" list); they can be used to store body indices, since they are currently in the [0,8192) range */
            union {
                int16_t i16[NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES]; /**< user data that can be used for static and kinematic bodies only (and only one of its array variants) */
                uint16_t u16[NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES]; /**< user data that can be used for static and kinematic bodies only (and only one of its array variants) */
                int8_t i8[NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES*2];  /**< user data that can be used for static and kinematic bodies only (and only one of its array variants) */
                uint8_t u8[NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES*2]; /**< user data that can be used for static and kinematic bodies only (and only one of its array variants) */
            } sk_user; /**< user data that can be used for static and kinematic bodies ONLY (and only one of the available array variants) */
        };
#       endif // NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES
#       ifdef NUDGE_BODYINFO_STRUCT_EXTRA_PADDING
        NUDGE_BODYINFO_STRUCT_EXTRA_PADDING /**< user stuff injected at the end of the BodyInfo struct */
#       endif // NUDGE_BODYINFO_STRUCT_EXTRA_PADDING
    };
    /**
     * @brief The main struct contained in \ref context_t "context_t": it exposes every per-body data in the simulation, except collisions
     * @note This structure contains arrays of size: count. Every array index is persistent (it's never shuffled or moved)
     */
	struct BodyData {
        Transform* transforms;  /**< array of size count, containing the position and rotation (in quaternion form) of each body */
        BodyProperties* properties;  /**< array of size count, containing the mass, inertia, gravity and friction of each body */
        BodyMomentum* momentum;  /**< array of size count, containing the linear and angular velocity of each body */
        BodyFilter* filters;    /**< array of size count, containing the collision group, the collision mask, and flags for each body */
        BodyLayout* layouts;     /**< array of size count, containing the colliders (i.e. collision shapes) indices of each body (see also the \ref ColliderData "c->colliders" arrays) */
        BodyInfo* infos;    /**< array of size count, , and customizable per-body user data */
        uint8_t* idle_counters; /**< array of size count, containing a counter per body, where a value of 0xFF indicates that the body is sleeping (it currently affectes only dynamic bodies); each value can be set to 0x00 to wake up a dynamic body */
        uint32_t count; /**< the number of bodies in the world. Bodies are never shuffled in the array, so indices are preserved. When bodies are removed count does not change (tip: filter the bodies with their flags to exclude removed and disabled bodies) */
	};
    /**
     * @brief [unused] The BodyConnections struct is actually just sketched in nudge (it was intended to add custom constraints)
     * @note The original version of the nudge library is better suited for user-side physic-related extensions (see the original nudge example code)
     */
	struct BodyConnections {
		BodyPair* data;
		uint32_t count;
	};
    /**
     * @brief The CachedContactImpulse struct
     * @note Cached impulses persist across frames to implement warmstarting
     */
	struct CachedContactImpulse {
		float impulse[3];
		float unused;
	};
    /**
     * @brief The ContactCache struct contains the CachedContactImpulse and persists across frames
     */
	struct ContactCache {
		uint64_t* tags;
		CachedContactImpulse* data;
		uint32_t capacity;
		uint32_t count;
	};
    /**
     * @brief The ActiveBodies struct
     * @note AFAICU it has something to do with bodies that have contacts between them. However for some strange reason even sleeping bodies can be in the list.
     */
	struct ActiveBodies {
        uint16_t* indices;  /**< array of size count of body indices */
		uint32_t capacity;
		uint32_t count;
	};
	
	struct ContactImpulseData;
	struct ContactConstraintData;
	        
    /**
     * \def NUDGE_INVALID_BODY_ID
     * Value of a body index in an invalid state
     */
#   define NUDGE_INVALID_BODY_ID (32767)

    // TODO: currently int32_t or uint32_t is used as body index: but at most MAX_NUM_BODIES is 8192.
    // So we can use int16_t and uint16_t (where padding/alignment is not required).
    // [Well, there are some parts in nudge internal where the upper
    // 16-bit of a body index is used, so I'm not sure if this can be done everywhere]

    /**
     * @brief The KinematicData is composed by two arrays: an array of global key frames and an array of animations.
     * Each animation owns a (kinematic) body index and a range of key frames.
     * @note Kinematic animations are just used to automatically move kinematic bodies. Each range of key frames can be used by more than one Animation (i.e. body), because each Animation can have an offset transform (baseT) and/or an offset time (offset_time).
     * @note All the times are intended in seconds, and they represent the (relative) time to get to the current key frame.
     */
    struct KinematicData  {
        // key_frame_transforms and key_frame_modes are a single memory block used by all animations
        Transform* key_frame_transforms;   /**< array of size key_frame_count of Transform structs, where each element has the Transform::time field set (it represents the seconds to get from the previous frame to that frame when the animation speed is 1.0f) */
        /**
         * @brief TimeMode enum is an optional experimental flag
         */
        enum TimeMode {
            TM_NORMAL=0,    /**< uniform speed across the transforms */
            TM_ACCELERATE,  /**< accelerated speed across the transforms */
            TM_DECELERATE   /**< decelerated speed across the transforms */
        }* key_frame_modes; /**< array of size key_frame_count of TimeMode enums (experimental, it can probably be completely ignored in most cases) [TODO: remove?] */
        uint32_t key_frame_capacity;    /**< the number of key frames the arrays can contain (use \ref kinematic_data_reserve_key_frames "kinematic_data_reserve_key_frames(...)" to increase it) */
        uint32_t key_frame_count;      /**< the number of inserted key frames */
        /**
         * @brief The Animation class. Each animation owns a (kinematic) body index and a range of key frames.
         * @note Each animation use (or reuse) a chunk of the key_frame array. The same chunk (i.e. range of key frames) can be used by more than one Animation (i.e. body), because each Animation can have an offset transform (baseT) and/or an offset time (offset_time).
         * @note Animations referencing removed bodies (by default) are assigned to \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" (and skipped) next time \ref simulation_step "simulation_step(...)" is called (this behavior can be changed using the definition NUDGE_DELETE_KINEMATIC_ANIMATIONS_REFERENCING_REMOVED_BODIES).
         * @note So by default it's safe to store animation indices (unless we manually delete animations, or define NUDGE_DELETE_KINEMATIC_ANIMATIONS_REFERENCING_REMOVED_BODIES).
         * @note Using static/dynamic bodies in kinematic animations is something never tested (undefined behavior).
         */
        struct Animation    {
            float play_time;    /**< [read] animation current time of play */
            float offset_time;  /**< if set, animation starts after offset_time */
            float speed;        /**< tweakable animation speed (can be negative) */
            float total_time;   /**< total duration of the animation (if negative, value is refreshed before playing it) */
            Transform baseT;    /**< offset transform used if use_baseT is set */
            uint32_t key_frame_start; /**< start animation key_frame index */
            uint32_t key_frame_count; /**< num key_frame indices (from key_frame_start) */
            uint32_t body;      /**< index of the kinematic body to be animated */
            bool playing;       /**< true if animation is playing */
            bool use_baseT;     /**< activates the offset transform baseT */
            enum LoopMode {
                LM_NO_LOOP,     /**< normal mode */
                LM_LOOP_NORMAL, /**< at the end, animation restarts */
                LM_LOOP_PING_PONG   /**< at the end, animation goes back and forth */
            } loop_mode;        /**< animation loop mode */
        }* animations;          /**< array of size animations_count of Animation structs; by default entries in this array are persistent, i.e. not deleted or reordered by nudge (but this behavior can be changed using the definition NUDGE_DELETE_KINEMATIC_ANIMATIONS_REFERENCING_REMOVED_BODIES) */
        uint32_t animations_capacity;    /**< the number of animations the array can contain (use \ref kinematic_data_reserve_animations "kinematic_data_reserve_animations(...)" to increase it) */
        uint32_t animations_count;     /**< the number of inserted animations */
    };
    /**
     * @brief The SimulationParams struct
     * @note It's used to tweak the simulation
     */
    struct SimulationParams {
        // read/write
        double time_step;    /**< [tweakable] [default value 1.0/60.0] */
        unsigned max_num_substeps; /**< [tweakable] [default value 2] when too much time passes, only max_num_substeps are performed, and the simulation 'burns' the remaining substeps */
        unsigned num_iterations_per_substep;   /**< [tweakable] [default value 5] it improves some stability at the expense of performance */
        float sleeping_threshold_linear_velocity_squared;  /**< [tweakable] [default value 1e-2f]; by increasing it the bodies goes to sleep faster */
        float sleeping_threshold_angular_velocity_squared; /**< [tweakable] [default value 1e-1f]; by increasing it the bodies goes to sleep faster */
        float linear_damping;   /**< [tweakable] [default value 0.25]; by increasing it, the bodies slow down their movement faster */
        float angular_damping;  /**< [tweakable] [default value 0.25]; by increasing it, the bodies slow down their rotation faster  */
        float penetration_allowed_amount; /**< [tweakable(?)] [default value 1e-3f]; */
        float penetration_bias_factor; /**< [tweakable(?)] [default value 2.0f]; */
        unsigned numsubsteps_overflow_warning_mode; /**< [tweakable] nudge::log warns on every frame with a numsubstep overflown. Modes: 0:warn on second successive frame;1:warn always;2:never warn. Default is 0 */

        // read only
        unsigned long long num_frames;  /**< [read-only] number of physic frames, i.e. the total number of calls to \ref simulation_step "simulation_step(...)" with at least one substep to perform. Usually in each graphic frame there is a single call to \ref simulation_step "simulation_step" and the number of physic frames can increase by one unit or remain constant (TODO: consider removing this, and renaming num_frames the num_total_substeps) */
        unsigned long long num_total_substeps;  /**< [read-only] the total number of physic substeps so far. In one physic frame there can be a number of substeps in the interval [0,max_num_substeps]; good value for benchmark measures */
        double remaining_time_in_seconds;    /**< [used internally] */
        float time_step_minus_remaining_time;   /**< [used intenally] */
        unsigned num_substeps_in_last_frame;    /**< [read-only] returned by \ref pre_simulation_step "pre_simulation_step(...)"; it's <= max_num_substeps, and it equals the number of substeps that next call to \ref simulation_step "simulation_step" will perform */
        unsigned numsubsteps_overflow_in_last_frame; /**< [read-only] set in \ref simulation_step "simulation_step" */
    };
    /**
     * @brief The GlobalDataMaskEnum enum
     */
    enum GlobalDataMaskEnum {
        GF_USE_GLOBAL_GRAVITY = 1<<0    /**< this flag disables per-body gravity and uses \ref GlobalData "GlobalData::gravity" instead (disabled by default) */
#       ifdef NUDGE_GLOBALDATAMASK_ENUM_EXTRA_FIELDS
#           define NUDGE_GLOBALDATAMASK_ENUM_EXTRA_FIELDS
#       endif
    };
    /**
     * @brief The GlobalData struct inglobes global fields that could not fit in the SimulationParams struct
     */
    struct GlobalData   {
        float gravity[3];    /**< by default each body uses its own gravity (in its 'properties'), but by using the GF_USE_GLOBAL_GRAVITY flags, all the bodies share this gravity value (defaults to {0,-9.82,0}) */
        uint32_t flags;     /**< bit mask of \ref GlobalDataMaskEnum "GF_ flags"; please do not reset it, just set or reset flags inside it (because some flags could be set by default, and other flags could be added in the future) */
        FlagMask exclude_smoothing_graphic_transform_flags;   /**< bit mask of \ref BodyFlagEnum "BF_ flags"; default value is 0; this flag just affects body transform smoothing in two functions (\ref calculate_graphic_transform_for_body "calculate_graphic_transform_for_body(...)" and \ref calculate_graphic_transforms "calculate_graphic_transforms(...)"); note that in any case static bodies and dynamic sleeping bodies are always not smoothed, because they are still; possible values are usually: BF_IS_DYNAMIC and/or BF_IS_KINEMATIC (both together disable transform smoothing completely) */
        uint32_t* removed_bodies;  /**< [internal usage, \ref remove_body "remove_body(...)"] since we can't remove bodies properly, instead we put bodies in stand-by ready to be reused (when new bodies are added) and we free their colliders; removed bodies end in this list and can be filtered out using their BF_IS_REMOVED flag */
        uint32_t removed_bodies_count;  /**< [internal usage] */
        uint32_t finalized_removed_bodies_count;  /**< [internal usage] finalized_removed_bodies_count<=removed_bodies_count; finalization happens at the start of the \ref simulation_step "simulation_step(...)" function */
        const uint32_t removed_bodies_capacity; /**< [internal usage] should be == c->MAX_NUM_BODIES */
    };
    /**
     * @brief Main struct of the library.
     * @note It should be initialized at program startup (see \ref init_context_with "init_context_with(...)" or \ref init_context "init_context(...)")
     * @note It should be destroyed at program exit (see \ref destroy_context "destroy_context(...)")
     */
    struct context_t {
        // original nudge fields
        Arena arena;        /**< [internal usage] */

        BodyData bodies;    /**< accessor to the main physics data of the library */
        ColliderData colliders; /**< accessor to the (global) collider (i.e. collision shape) (box and sphere) arrays. These arrays can be reordered by nudge, but their 'tags' are unique and are automatically assigned and preserved (please NEVER touch them); it's NOT safe to store array indices in colliders.boxes or collider.spheres! Always use c->bodies.infos[body].first_box_index and c->bodies.infos[body].num_boxes instead (same for spheres) */
        ContactData contact_data;   /**< accessor to the array of contacts, so that users can understand if a body is in contact with another. Please note that only dynamic non-sleeping bodies are guaranteed to report all their contact every frame AFAIK */

        ContactCache contact_cache; /**< accessor to the persistent (across frames) CachedContactImpulses that are used to implement warmstarting (used to slightly improved stacking AFAIK); users should simply ignore it */
        ActiveBodies active_bodies; /**< accessor to an array of active bodies. AFAICU this array has something to do with bodies that have contacts between them (however for some strange reason even sleeping bodies can be in the list). I personally prefer using the official c->bodies array (more robust and thrustable) */

        // extended stuff
        KinematicData kinematic_data;   /**< accessor to the KinematicData::key_frames array and the KinematicData::animations array that can be used to move kinamatic bodies in an automatic way */
        GlobalData global_data; /**< accessor to some global fields that don't fit the SimulationParams category */
        SimulationParams simulation_params; /**< accessor to some global fields that deal with the physic simulation, such as the simulation time_step, the max_num_substeps, sleeping thresholds, damping factors, etc. */

        const unsigned MAX_NUM_BOXES;   /**< fixed value of the max number of box colliders that can be used in the simulation; @see init_context_with(...); it must be: c->MAX_NUM_BOXES+c->MAX_NUM_SPHERES<8192 (the limitation seems to arise from the broadphase management) */
        const unsigned MAX_NUM_SPHERES; /**< fixed value of the max number of sphere colliders that can be used in the simulation; @see init_context_with(...); it must be: c->MAX_NUM_BOXES+c->MAX_NUM_SPHERES<8192 (the limitation seems to arise from the broadphase management) */
        const unsigned MAX_NUM_BODIES;  /**< fixed value that should always be equal to: MAX_NUM_BOXES+MAX_NUM_SPHERES; this value is only reachable when a single collider per body is used */

#       ifdef NUDGE_CONTEXT_STRUCT_EXTRA_FIELDS
        NUDGE_CONTEXT_STRUCT_EXTRA_FIELDS /**< user stuff injected into the context_t struct */
#       endif // NUDGE_CONTEXT_STRUCT_EXTRA_FIELDS
#       ifndef NUDGE_CONTEXT_STRUCT_NO_USER_DATA
        UserData64Bit user;     /**< user data: a per-context user space in 11 different variable names that share the same space, so that ONLY one of them must be chosen and used */
#       endif // NUDGE_CONTEXT_STRUCT_NO_USER_DATA
    };

    /**
     * @brief The AxisEnum enum
     */
    enum AxisEnum {AXIS_X=0,AXIS_Y=1,AXIS_Z=2};


#   ifdef NUDGE_USE_TIME_CONTEXT
    /**
     * @brief Optional struct (not used at all in nudge.h)
     */
    struct time_context_t   {
        // (How bad is proper cpp style with private variables and getters/setters... plain C rules!)
    public:
        // Usage: just call this ALWAYS once per frame
        inline void update(double globalTimeInSeconds)    {
            double elapsedTime,elapsedNetTime,deltaTime;
            double currentTime = totalTime;

            // paused time code
            if (wasPausedLastFrame!=paused) {
                wasPausedLastFrame = paused;
                if (paused) beginPausedTime=globalTimeInSeconds;
                else {
                    beginNetTime+=globalTimeInSeconds-beginPausedTime;beginPausedTime = 0;
                }
            }
            // time calculations
            if (beginTime==0) beginTime = globalTimeInSeconds;
            if (beginNetTime==0) beginNetTime = globalTimeInSeconds;

            elapsedTime = globalTimeInSeconds;if (elapsedTime<beginTime) beginTime=elapsedTime;
            elapsedTime-=beginTime;
            totalTime = elapsedTime;

            if (!paused)    {
                elapsedNetTime = globalTimeInSeconds;if (elapsedNetTime<beginNetTime) beginNetTime=elapsedNetTime;
                elapsedNetTime-=beginNetTime;
                totalTimeWithoutPause = elapsedNetTime;
            }

            deltaTime = elapsedTime;if (deltaTime<currentTime) currentTime=deltaTime;
            deltaTime-=currentTime;
            currentTime = elapsedTime;
            instantFrameTime = deltaTime;

            timeNow = globalTimeInSeconds;
            ++num_frames;
            //assert(totalTime==currentTime);
        }
        inline double getInstantFrameTime() const {return instantFrameTime;}
        inline double getTotalTime() const {return totalTime;}
        inline double getTotalTimeWithoutPause() const {return totalTimeWithoutPause;}
        inline double getBeginTime() const {return beginTime;}
        inline double getTimeNow() const {return timeNow;}
        inline double getInstantFPS() const {return instantFrameTime!=0?1.0/instantFrameTime:0;}
        inline unsigned long getNumFrames() const {return num_frames;}
        inline bool getPaused() const {return paused;}
        inline void setPaused(bool flag) {paused = flag;}
        inline void togglePaused() {paused = !paused;}
        time_context_t() : instantFrameTime(16.2),totalTime(0),totalTimeWithoutPause(0),paused(false),
            beginTime(0),beginNetTime(0),beginPausedTime(0),timeNow(0),num_frames(0),wasPausedLastFrame(false) {}
        inline void restoreFrom(time_context_t* o)  {
            //const double deltaTime = totalTime - o->totalTime;
            //beginTime += deltaTime;
            beginNetTime += totalTimeWithoutPause - o->totalTimeWithoutPause;
            totalTimeWithoutPause = o->totalTimeWithoutPause;
            num_frames = o->num_frames;
            //return deltaTime;
        }
    private:
        double instantFrameTime;        // get; seconds elapsed from last frame
        double totalTime;               // get; second elapsed from the start (including 'paused' time)
        double totalTimeWithoutPause;   // get; second elapsed from the start (excluding 'paused' time)
        bool paused;                    // get/set

        double beginTime,beginNetTime,beginPausedTime,timeNow;
        unsigned long num_frames;
        bool wasPausedLastFrame;
    };
#   endif //NUDGE_USE_TIME_CONTEXT


    /**
     * @defgroup context_group Context Functions
     * @brief Set of functions regarding the nudge \ref context_t "context", and in general the program startup and shutdown functions
     * @{
     */
    /**
     * @brief Displays basic info at program startup; very important call to detect the SIMD configuration of the program
     */
    void show_info();
    /**
     * @brief Mandatory function to be called at program startup
     * @param c the nudge context; best practice is to clear its memory before calling this function
     * @param MAX_NUM_BOXES the max number of box colliders (i.e. box collision shapes) that can be used in the library (each physic body can contain one or more colliders and each collider is owned by a single physic body)
     * @param MAX_NUM_SPHERES the max number of sphere colliders (i.e. sphere collision shapes) that can be used in the library (each physic body can contain one or more colliders and each collider is owned by a single physic body)
     * @note It must be MAX_NUM_BOXES+MAX_NUM_SPHERES<=8192
     */
    void init_context_with(context_t *c, unsigned MAX_NUM_BOXES,unsigned MAX_NUM_SPHERES);
    /**
     * @brief Mandatory function to be called at program startup
     * @param c the nudge context; best practice is to clear its memory before calling this function
     * @note It sets MAX_NUM_BOXES and MAX_NUM_SPHERES to some default value (see \ref init_context_with "init_context_with(...)")
     */
    void init_context(context_t* c);
    /**
     * @brief Mandatory function to be called at program exit
     * @param c the nudge context
     * @note After this call \ref init_context "init_context(...)" and \ref init_context_with "init_context_with(...)" can be called again, but \ref restart_context "restart_context(...)" can't
     */
    void destroy_context(context_t* c);
    /**
     * @brief Optional function that restarts a valid context, preserving the simulation settings and the allocated memory
     * @param c a valid nudge context that must be inited with \ref init_context "init_context(...)" or \ref init_context_with "init_context_with(...)"
     * @note This function is faster than calling in sequence \ref destroy_context "destroy_context(...)" and \ref init_context "init_context(...)" (no deallocations/allocations)
     * @note If you want to restart the simulation frame/substep counters, please manually set: c->simulation_params.num_frames=0;c->simulation_params.num_total_substeps=0;
    */
    void restart_context(context_t* c);

#   ifndef NUDGE_NO_STDIO
    /**
     * @brief Saves the nudge context
     * @param f the output file
     * @param c the input context
     * @note Experimental feature
     * @note Available only when NUDGE_NO_STDIO is not defined
     */
    void save_context(FILE* f,const context_t* c);
    /**
     * @brief Loads a saved nudge context
     * @param f the input file
     * @param c the output (inited) context
     * @note Experimental feature, currently c must have a compatible c->MAX_NUM_BOXES and c->MAX_NUM_SPHERES to work (and user pointers must of course be handled by the user)
     * @note Currently it just asserts on failing
     * @note Available only when NUDGE_NO_STDIO is not defined
     */
    void load_context(FILE* f,context_t* c);
#   endif //NUDGE_NO_STDIO
    /** @} */ // end of context_group

    /**
    * @defgroup main_group Main Functions
    * @brief Set of functions regarding stepping the simulation and getting the body transforms back
    * @{
    */
    /**
     * @brief Mandatory function that must be called once per frame
     * @param c the nudge context
     * @param elapsedSecondsFromLastCall time tn seconds elapsed from last call
     * @return the number of simulation substeps (i.e. physic frame substeps) that will be executed in the next \ref simulation_step "simulation_step(...)" call
     */
    unsigned pre_simulation_step(context_t* c,double elapsedSecondsFromLastCall);

    /**
     * @brief Mandatory function that must be called once per frame
     * @param c the nudge context
     * @note It's the main function of the whole library
     */
    void simulation_step(context_t* c);

    /**
     * @brief Function that can be used to calculate the smoothed 16-float column-major model matrix of a single body
     * @param c the nudge context
     * @param body the input body index
     * @param pModelMatrix16Out the output smoothed 16-float column-major model matrix
     * @return the same as pModelMatrix16Out (for chaining the call only)
     * @note This function must be used after calling \ref simulation_step "simulation_step(...)", and the returned matrix inglobes the center of mass offset if present (so that no offset operation is required on the user-side in most cases)
     */
    float* calculate_graphic_transform_for_body(context_t* c,unsigned body,float* pModelMatrix16Out);
    /**
     * @brief Function that can be used to calculate the smoothed 16-float column-major model matrices of all the bodies together
     * @param c the nudge context
     * @param pModelMatricesOut a pointer to the output c->bodies.count*modelMatrixStrideInFloatUnits floats that represent the returned smoothed 16-float column-major model matrices of this function
     * @param modelMatrixStrideInFloatUnits stride (in number of floats) between two 16-float matrices inside the pModelMatricesOut array (it must be at least 16)
     * @param loopActiveBodiesOnly (experimental) if not zero, it only updates bodies present in the c->active_bodies list, i.e. not all the output matrices are updated (not recommended)
     * @note This function must be used after calling \ref simulation_step "simulation_step(...)", and the returned matrices inglobe the center of mass offsets if present (so that no offset operation is required on the user-side in most cases)
     */
    void calculate_graphic_transforms(context_t* c,float* pModelMatricesOut,unsigned modelMatrixStrideInFloatUnits,int loopActiveBodiesOnly=0);
    /** @} */ // end of main_group

    /**
     * @defgroup add_group Add-Bodies Functions
     * @brief Set of functions regarding creation and removal of physic bodies
     * @{
     */
    /**
     * @brief Adds a new body to the simulation with a single box collider
     * @param c the nudge context
     * @param mass positive => dynamic; 0 => static; negative => kinematic (where the absolute value will be used as mass internally)
     * @param hsizex half box size in the x direction
     * @param hsizey half box size in the y direction
     * @param hsizez half box size in the z direction
     * @param T a pointer to a Transform
     * @param comOffset an optional array of 3 floats that determines the center of mass offset of the body
     * @return the body index, or \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" if no more boxes can be added
     * @note Internally \ref BodyProperties "mass_inverse and inertia_inverse" are always stored as positive values (except for static bodies): this makes kinematic to dynamic body conversions a bit easier
     * @note Every time an \ref add_group "add_xxx(...)" function is called, if c->global_data.finalized_removed_bodies_count>0, the body c->global_data.removed_bodies[0] is always reused and returned
     */
    unsigned add_box(context_t* c,float mass, float hsizex, float hsizey, float hsizez, const Transform* T=NULL,const float comOffset[3]=NULL);
    /**
     * @overload
     * @param mMatrix16WithoutScaling a pointer to a 4x4 column-major matrix with only translation and rotation
     */
    unsigned add_box(context_t* c,float mass, float hsizex, float hsizey, float hsizez, const float* mMatrix16WithoutScaling,const float comOffset[3]=NULL);
    /**
     * @brief Adds a new body to the simulation with a single sphere collider
     * @param c the nudge context
     * @param mass positive => dynamic; 0 => static; negative => kinematic (where the absolute value will be used as mass internally)
     * @param radius the sphere radius
     * @param T a pointer to a Transform
     * @param comOffset an optional array of 3 floats that determines the center of mass offset of the body
     * @return the body index, or \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" if no more spheres can be added
     * @note Internally \ref BodyProperties "mass_inverse and inertia_inverse" are always stored as positive values (except for static bodies): this makes kinematic to dynamic body conversions a bit easier
     * @note Every time an \ref add_group "add_xxx(...)" function is called, if c->global_data.finalized_removed_bodies_count>0, the body c->global_data.removed_bodies[0] is always reused and returned
     */
    unsigned add_sphere(context_t* c,float mass, float radius, const Transform* T=NULL,const float comOffset[3]=NULL);
    /**
     * @overload
     * @param mMatrix16WithoutScaling a pointer to a 4x4 column-major matrix with only translation and rotation
     */
    unsigned add_sphere(context_t* c,float mass, float radius, const float* mMatrix16WithoutScaling,const float comOffset[3]=NULL);
    /**
     * @brief Adds a new body to the simulation with a compound collider made up of num_boxes box colliders and num_spheres sphere colliders
     * @param c the nudge context
     * @param mass positive => dynamic; 0 => static; negative => kinematic (where the absolute value will be used as mass internally)
     * @param inertia an inertia tensor in a 3-float array form that is used only if mass is not zero (see also the @ref inertia_group "inertia helper functions"); it can be NULL (in that case a box inertia on the body axis-aligned bounding box extents is used)
     * @param num_boxes
     * @param hsizeTriplets pointer to an array of size 3*num_boxes floats
     * @param boxOffsetTransforms pointer to an array of num_boxes Transforms
     * @param num_spheres
     * @param radii pointer to an array of num_sphere floats
     * @param sphereOffsetTransforms pointer to an array of num_sphere Transforms
     * @param T a pointer to a Transform
     * @param comOffset an optional input array of 3 floats that determines the center of mass offset of the body
     * @param centerMeshAndRetrieveOldCenter3Out [experimental] an optional output array of 3 floats: if set, the input mesh is recentered (before applying the comOffset) and the axis-aligned bounding box center that has been subtracted from the mesh is returned
     * @return the body index, or \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" if no more boxes can be added
     * @note Internally \ref BodyProperties "mass_inverse and inertia_inverse" are always stored as positive values (except for static bodies): this makes kinematic to dynamic body conversions a bit easier
     * @note Every time an \ref add_group "add_xxx(...)" function is called, if c->global_data.finalized_removed_bodies_count>0, the body c->global_data.removed_bodies[0] is always reused and returned
     */
    unsigned add_compound(context_t* c, float mass, float inertia[3], unsigned num_boxes, const float* hsizeTriplets, const Transform* boxOffsetTransforms, unsigned num_spheres, const float* radii, const Transform* sphereOffsetTransforms, const Transform* T=NULL, const float comOffset[3]=NULL, float *centerMeshAndRetrieveOldCenter3Out = NULL);
    /**
     * @overload
     * @param mMatrix16WithoutScaling a pointer to a 4x4 column-major matrix with only translation and rotation
     */
    unsigned add_compound(context_t* c,float mass, float inertia[3],unsigned num_boxes,const float* hsizeTriplets,const float* boxOffsetMatrices16WithoutScaling,unsigned num_spheres,const float* radii,const float* sphereOffsetMatrices16WithoutScaling,const float* mMatrix16WithoutScaling=NULL, const float comOffset[3]=NULL, float *centerMeshAndRetrieveOldCenter3Out = NULL);
    /**
     * @brief [Experimental] Adds a new body to the simulation cloning an existing body
     * @param c the nudge context
     * @param body_to_clone the body to clone
     * @param mass positive => dynamic; 0 => static; negative => kinematic (where the absolute value will be used as mass internally)
     * @param T a pointer to a Transform
     * @param scale_factor  positive => the uniform scaling factor to apply (to each single axis); negative => scale so that the half bounding box y-component of the body becomes exactly -scaling_factor; 0 => invalid value (asserts)
     * @param newComOffsetInPreScaledUnits if set, an absolute new center of mass offset will be added (replacing the old one if present) using the pre-scaled coordinates; otherwise the old center of mass offset (if present) is kept (and possibly scaled)
     * @return the body index, or \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" if no more boxes can be added
     * @note Internally \ref BodyProperties "mass_inverse and inertia_inverse" are always stored as positive values (except for static bodies): this makes kinematic to dynamic body conversions a bit easier
     * @note Every time an \ref add_group "add_xxx(...)" function is called, if c->global_data.finalized_removed_bodies_count>0, the body c->global_data.removed_bodies[0] is always reused and returned
     */
    unsigned add_clone(context_t* c,unsigned body_to_clone,float mass,const Transform* T=NULL,float scale_factor=1.f,const float newComOffsetInPreScaledUnits[3]=NULL);
    /**
     * @overload
     * @param mMatrix16WithoutScaling a pointer to a 4x4 column-major matrix with only translation and rotation
     */
    unsigned add_clone(context_t* c,unsigned body_to_clone,float mass,const float* mMatrix16WithoutScaling,float scale_factor=1.f,const float newComOffsetInPreScaledUnits[3]=NULL);

    /**
     * @brief Removes a body from the simulation
     * @note The body is actually removed next time \ref simulation_step "simulation_step(...)" is called (the call 'finalizes' the removal of all pending bodies).
     * The body can't be reused after this call and could still be present in ContactData for some (1?) frames.
     * In any case user can detect it with: ((*body_get_flags(...))&BF_IS_REMOVED).
     * Also removed bodies are NOT subtracted from c->bodies.count, but are reused when new bodies are added with: \ref add_group "add_xxx(...)".
     * @note Internally, removed bodies are kept in the c->global_data.removed_bodies array.
     * @note If you just want to reuse the body WITH THE SAME (optionally rescaled) collider(s) soon,
     * you should not remove the body, but just change its properties (more efficient + no delay).
     * @note Sometimes it's better to just disable a body, instead of removing it, using: (*body_get_flags(...))|=BF_IS_DISABLED: this way the body can be optionally put into a user-side custom list for later reusage/reactivation. This way the body colliders are preserved.
     * @note Every time an \ref add_group "add_xxx(...)" function is called, if c->global_data.finalized_removed_bodies_count>0, the body: c->global_data.removed_bodies[0] is always returned.
     * @note User data in the \ref BodyInfo "BodyInfo" struct are NOT reset when a body is removed, but most of the other data are reset when removed bodies are finalized at the beginning of next \ref simulation_step "simulation_step(...)" call.
     * @note That means that when bodies are reused in \ref add_group "add_xxx(...)" functions, their old user data are preserved.
     * @note Kinematic animations referencing removed bodies are assigned to \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" when removed bodies are finalized (there's an optional definition NUDGE_DELETE_KINEMATIC_ANIMATIONS_REFERENCING_REMOVED_BODIES to delete the kinematic animations instead).
     */
    void remove_body(context_t* c,unsigned body);
    /**
     * @brief Return the number of box colliders that can still be added to the physic world
     * @note The maximum number can be set in \ref init_context_with "init_context_with(...)" and can't be changed at runtime
     */
    uint32_t colliders_get_num_remaining_boxes(context_t* c);
    /**
     * @brief Return the number of sphere colliders that can still be added to the physic world
     * @note The maximum number can be set in \ref init_context_with "init_context_with(...)" and can't be changed at runtime
     */
    uint32_t colliders_get_num_remaining_spheres(context_t* c);
    /**
     * @return 1 if \ref add_box "add_box(...)" can be called successfully
     */
    inline int can_add_box(context_t* c) {return colliders_get_num_remaining_boxes(c)>=1;}
    /**
     * @return 1 if \ref add_sphere "add_sphere(...)" can be called successfully
     */
    inline int can_add_sphere(context_t* c) {return colliders_get_num_remaining_spheres(c)>=1;}
    /**
     * @return 1 if \ref add_compound "add_compound(...)" can be called successfully with num_boxes and num_spheres
     */
    inline int can_add_compound(context_t* c,unsigned num_boxes,unsigned num_spheres) {return (colliders_get_num_remaining_boxes(c)>=num_boxes && colliders_get_num_remaining_spheres(c)>=num_spheres);}
    /**
     * @return 1 if \ref add_clone "add_clone(...)" can be called successfully
     */
    inline int can_add_clone(context_t* c,unsigned body_to_clone) {return (colliders_get_num_remaining_boxes(c)>=c->bodies.layouts[body_to_clone].num_boxes && colliders_get_num_remaining_spheres(c)>=c->bodies.layouts[body_to_clone].num_spheres);}
    /**
     * @brief Allows to peek the body index that is going to be returned in next \ref add_group "add_xxx(...)" call
     * @param c the nudge context
     * @return one of the following values: c->bodies.count, c->global_data.removed_bodies[0] or \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID"
     * @note Next call to any \ref add_group "add_xxx(...)" function can still return \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID", if the number of available colliders runs out (that condition can be queried before adding the body using \ref colliders_get_num_remaining_boxes "colliders_get_num_remaining_boxes(...)", \ref colliders_get_num_remaining_spheres "colliders_get_num_remaining_spheres(...)" or \ref can_add_box "can_add_box(...)", \ref can_add_sphere "can_add_sphere(...)", \ref can_add_compound "can_add_compound(...)")
     * @note The returned body is valid until next \ref simulation_step "simulation_step(...)" call
     * @note A returned value of c->bodies.count can still be used, because the arrays are allocated at context init time, with a size of c->MAX_NUM_BODIES
     */
    inline unsigned get_next_add_body_index(context_t* c) {return c->global_data.finalized_removed_bodies_count>0?c->global_data.removed_bodies[0]:(c->bodies.count>=c->MAX_NUM_BODIES?NUDGE_INVALID_BODY_ID:c->bodies.count);}


    /**
     * @brief Recalculates the bounding box of the body (\ref BodyInfo "BodyInfo::aabb_center and BodyInfo::aabb_extents")
     * @param c the nudge context
     * @param body the body index
     * @note This function is already called when bodies are added (see \ref add_group "add_xxx" functions); so it's almost pointless to call it again (except maybe when manually changing the size of some collider)
     * @note nudge.h does not use the body bounding box at all: it has been added just to ease user experience
    */
    void body_recalculate_bounding_box(context_t* c,uint32_t body);

    /**
     * @brief [Experimental] Changes the body motion state (i.e. the BF_IS_STATIC_OR_KINEMATIC_OR_DYNAMIC group of body flags)
     * @param c the nudge context
     * @param body the index of the body
     * @param new_motion_state the desired new motion state; it must be: BF_IS_STATIC, BF_IS_KINEMATIC or BF_IS_DYNAMIC
     * @param mass_fallback a mass fallback used only when 'new_motion_state' is BF_IS_DYNAMIC and the body had zero mass (the body was static)
     * @note Experimental feature: use it at your own risk and for a limited and selected amount of bodies
     * @note This function resets the body velocities
     * @note In case of BF_IS_DYNAMIC 'new_motion_state' this function can replace the components of inertia of the body, if it was not set before (the body was static) with a box inertia on the body axis aligned bounding box
     * @note It's better to leave static bodies alone, and just make kinematic bodies dynamic or viceversa
     */
    void body_change_motion_state(nudge::context_t* c,unsigned body,nudge::FlagMask new_motion_state,float mass_fallback=1.f);

    /**
     * @brief [Experimental] Uniformly scales the specified body incrementally
     * @param c the nudge context
     * @param body the target body
     * @param scale_factor positive => the uniform scaling factor to apply (to each single axis); negative => scale so that the half bounding box y-component of the body becomes exactly -scaling_factor; 0 => invalid value (asserts)
     * @param mass_scale_factor positive => the scaling amount to apply to the mass; 0 => sets 'mass_scale_factor' to the cube of the 'scaling_factor' argument; negative => sets the new mass exactly to -mass_scale_factor
     * @note Experimental feature: use it at your own risk and for a limited and selected amount of bodies
     * @note Since the scaling is incremental, floating point errors are introduced when this function is called multiple times (e.g. calling it with 2.f and then with 0.5f is not perfectly equivalent to leaving the body unscaled), and it's not possible to retrieve the scaling factor that has been applied
     * @note In case 'mass_scale_factor' is negative and the body has no inertia set, a box inertia for the body is calculated on the body axis aligned bounding box
     */
    void body_scale(nudge::context_t* c,unsigned body,float scale_factor,float mass_scale_factor=0.f);

    /** @} */ // end of add_group


    /**
     * @brief Gets the pointer to the body linear velocity (3-floats)
     * @param c the nudge context
     * @param body the index of the body
     * @return a reference to the body linear velocity (a float3 array)
     */
    inline float* body_get_velocity(context_t* c,uint32_t body) {return c->bodies.momentum[body].velocity;}

    /**
     * @brief Gets the pointer to the body angular velocity (3-floats)
     * @param c the nudge context
     * @param body the index of the body
     * @return a reference to the body angular velocity (a float3 array)
     */
    inline float* body_get_angular_velocity(context_t* c,uint32_t body) {return c->bodies.momentum[body].angular_velocity;}


    /**
     * @brief Gets the pointer to the body position (3-floats vector)
     * @param c the nudge context
     * @param body the index of the body
     * @return a reference to the body position (a float3 array)
     */
    inline float* body_get_position(context_t* c,uint32_t body) {return c->bodies.transforms[body].p;}
    /**
     * @brief Gets the pointer to the body orientation (4-floats quaternion in {x,y,z,w} format)
     * @param c the nudge context
     * @param body the index of the body
     * @return a reference to the body orientation (floats4 quaternion in {x,y,z,w} format)
     */
    inline float* body_get_orientation(context_t* c,uint32_t body) {return c->bodies.transforms[body].q;}


    namespace extra {
    /**
     * @defgroup extra_group Optional Extra Functions
     * @brief Set of helper functions that just wrap existing functions
     * @{
     */

    /**
     * @brief Adds a new body to the simulation with a compound collider that represent a prism of 4 or more lateral faces
     * @param c the nudge context
     * @param mass positive => dynamic; 0 => static; negative => kinematic (where the absolute value will be used as mass internally)
     * @param radius the radius of the cylinder inscribed into the prism (the prism covers all the inscribed cylinder); please use (radius*cosf(M_PI/num_lateral_faces)) as radius for a prism that is entirely covered by the cylinder of radius: radius
     * @param hheight the half height of the prism
     * @param num_lateral_faces the number of lateral faces of the prism (must be >=4)
     * @param T a pointer to a Transform
     * @param axis one of the \ref AxisEnum "AXIS_X,AXIS_Y,AXIS_Z" values
     * @param comOffset an optional array of 3 floats that determines the center of mass offset of the body
     * @return the body index, or \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" if not enough colliders can be added, or if the input arguments are not supported
     * @note This function is just a wrapper around the \ref add_compound "add_compound(...)" function
     * @note This function consumes at most 'num_lateral_faces' boxes
     * @note This function can return \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" even if there are enough colliders (unlike \ref add_compound "add_compound(...)")
     */
    unsigned add_compound_prism(context_t* c,float mass,float radius,float hheight, unsigned num_lateral_faces=0,const Transform* T=NULL,AxisEnum axis=AXIS_Y,const float comOffset[3]=NULL);
    /**
     * @overload
     * @param mMatrix16WithoutScaling a pointer to a 4x4 column-major matrix with only translation and rotation
     */
    unsigned add_compound_prism(context_t* c, float mass, float radius, float hheight, unsigned num_lateral_faces, const float* mMatrix16WithoutScaling, AxisEnum axis=AXIS_Y, const float comOffset[3]=NULL);
    /**
     * @brief Adds a new body to the simulation with a compound collider that represent a cylinder
     * @param c the nudge context
     * @param mass positive => dynamic; 0 => static; negative => kinematic (where the absolute value will be used as mass internally)
     * @param radius the radius of the cylinder
     * @param hheight the half height of the cylinder
     * @param T a pointer to a Transform
     * @param axis one of the \ref AxisEnum "AXIS_X,AXIS_Y,AXIS_Z" values
     * @param num_boxes the number of boxes to consume (0 is replaced by some predefined value)
     * @param num_spheres the number of spheres to consume (0 is replaced by some predefined value)
     * @param comOffset an optional array of 3 floats that determines the center of mass offset of the body
     * @param box_lateral_side_shrinking constant to tune how much the lateral half size of the box(es) is smaller than radius of the spheres (default value of -1 is replaced by some predefined value)
     * @return the body index, or \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" if not enough colliders can be added, or if the input arguments are not supported
     * @note This function is just a wrapper around the \ref add_compound "add_compound(...)" function
     * @note This function can return \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" even if there are enough colliders (unlike \ref add_compound "add_compound(...)")
     */
    unsigned add_compound_cylinder(context_t* c, float mass, float radius, float hheight, const Transform* T=NULL, AxisEnum axis=AXIS_Y, unsigned num_boxes=0, unsigned num_spheres=0, const float comOffset[3]=NULL, float box_lateral_side_shrinking=-1.f);
    /**
     * @overload
     * @param mMatrix16WithoutScaling a pointer to a 4x4 column-major matrix with only translation and rotation
     */
    unsigned add_compound_cylinder(context_t* c,float mass,float radius,float hheight, const float* mMatrix16WithoutScaling,AxisEnum axis=AXIS_Y,unsigned num_boxes=0,unsigned num_spheres=0,const float comOffset[3]=NULL, float box_lateral_side_shrinking=-1.f);
    /**
     * @brief Adds a new body to the simulation with a compound collider that represent a capsule
     * @param c the nudge context
     * @param mass positive => dynamic; 0 => static; negative => kinematic (where the absolute value will be used as mass internally)
     * @param radius the radius of the capsule
     * @param hheight the half height of the cylinder that compose the capsule (if it's zero, then the capsule becomes a sphere)
     * @param T a pointer to a Transform
     * @param axis one of the \ref AxisEnum "AXIS_X,AXIS_Y,AXIS_Z" values
     * @param num_boxes the number of boxes to consume (0 is replaced by some predefined value)
     * @param num_spheres the number of spheres to consume (0 is replaced by some predefined value)
     * @param comOffset an optional array of 3 floats that determines the center of mass offset of the body
     * @param box_lateral_side_shrinking constant to tune how much the lateral half size of the box(es) is smaller than radius of the spheres (default value of -1 is replaced by some predefined value)
     * @return the body index, or \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" if not enough colliders can be added, or if the input arguments are not supported
     * @note This function is just a wrapper around the \ref add_compound "add_compound(...)" function
     * @note This function can return \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" even if there are enough colliders (unlike \ref add_compound "add_compound(...)")
     */
    unsigned add_compound_capsule(context_t* c,float mass,float radius,float hheight, const Transform* T=NULL,AxisEnum axis=AXIS_Y,unsigned num_boxes=1,unsigned num_spheres=3,const float comOffset[3]=NULL, float box_lateral_side_shrinking=-1.f);
    /** @overload
     * @param mMatrix16WithoutScaling  a pointer to a 4x4 column-major matrix with only translation and rotation
     */
    unsigned add_compound_capsule(context_t* c,float mass,float radius,float hheight, const float* mMatrix16WithoutScaling,AxisEnum axis=AXIS_Y,unsigned num_boxes=1,unsigned num_spheres=3,const float comOffset[3]=NULL, float box_lateral_side_shrinking=-1.f);
    /**
     * @brief Adds a new body to the simulation with a compound collider that represent the hollow lateral surface of a cylinder
     * @param c the nudge context
     * @param mass positive => dynamic; 0 => static; negative => kinematic (where the absolute value will be used as mass internally)
     * @param min_radius the minimum radius that starts at the center of the cylinder
     * @param max_radius the maximum radius that starts at the center of the cylinder (the whole depth is: max_radius - min_radius)
     * @param hheight the half height of the cylinder
     * @param T a pointer to a Transform
     * @param axis one of the \ref AxisEnum "AXIS_X,AXIS_Y,AXIS_Z" values
     * @param num_boxes the number of boxes to consume (0 is replaced by some predefined value)
     * @param comOffset an optional array of 3 floats that determines the center of mass offset of the body
     * @return the body index, or \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" if not enough colliders can be added, or if the input arguments are not supported
     * @note This function is just a wrapper around the \ref add_compound "add_compound(...)" function
     * @note This function can return \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" even if there are enough colliders (unlike \ref add_compound "add_compound(...)")
     */
    unsigned add_compound_hollow_cylinder(context_t* c,float mass,float min_radius,float max_radius,float hheight, const Transform* T=NULL,AxisEnum axis=AXIS_Y,unsigned num_boxes=8,const float comOffset[3]=NULL);
    /**
     * @overload
     * @param mMatrix16WithoutScaling a pointer to a 4x4 column-major matrix with only translation and rotation
     */
    unsigned add_compound_hollow_cylinder(context_t* c,float mass,float min_radius,float max_radius,float hheight, const float* mMatrix16WithoutScaling, AxisEnum axis=AXIS_Y, unsigned num_boxes=8, const float comOffset[3]=NULL);
    /**
     * @brief Adds a new body to the simulation with a compound collider that represent a torus
     * @param c the nudge context
     * @param mass positive => dynamic; 0 => static; negative => kinematic (where the absolute value will be used as mass internally)
     * @param radius the radius that starts at the origin of the shape and ends in the middle of the rotational donut-like solid
     * @param inner_radius the internal radius of the donut-like solid (so that maximum_radius = radius + inner_radius represent the maximum radius of the shape, starting at its origin)
     * @param T a pointer to a Transform
     * @param axis one of the \ref AxisEnum "AXIS_X,AXIS_Y,AXIS_Z" values
     * @param num_boxes the number of boxes to consume (0 is replaced by some predefined value)
     * @param comOffset an optional array of 3 floats that determines the center of mass offset of the body
     * @return the body index, or \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" if not enough colliders can be added, or if the input arguments are not supported
     * @note This function is just a wrapper around the \ref add_compound "add_compound(...)" function
     * @note This function can return \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" even if there are enough colliders (unlike \ref add_compound "add_compound(...)")
     */
    unsigned add_compound_torus(context_t* c,float mass,float radius,float inner_radius, const Transform* T=NULL,AxisEnum axis=AXIS_Y,unsigned num_boxes=8,const float comOffset[3]=NULL);
    /**
     * @overload
     * @param mMatrix16WithoutScaling a pointer to a 4x4 column-major matrix with only translation and rotation
     */
    unsigned add_compound_torus(context_t* c,float mass,float radius,float inner_radius, const float* mMatrix16WithoutScaling,AxisEnum axis=AXIS_Y,unsigned num_boxes=8,const float comOffset[3]=NULL);
    /**
     * @brief Adds a new body to the simulation with a compound collider that represent an approximated cone
     * @param c the nudge context
     * @param mass positive => dynamic; 0 => static; negative => kinematic (where the absolute value will be used as mass internally)
     * @param radius the radius of the cone
     * @param hheight the half height of the cone
     * @param T a pointer to a Transform
     * @param axis one of the \ref AxisEnum "AXIS_X,AXIS_Y,AXIS_Z" values
     * @param num_boxes the number of boxes to consume (0 is replaced by some predefined value)
     * @param num_spheres the number of spheres to consume (0 is replaced by some predefined value)
     * @param comOffset an optional array of 3 floats that determines the center of mass offset of the body
     * @return the body index, or \ref NUDGE_INVALID_BODY_ID "NUDGE_INVALID_BODY_ID" if not enough colliders can be added
     * @note This function is just a wrapper around the \ref add_compound "add_compound(...)" function
     * @note The constructed shape is a raw approximatation, and the comOffset itself should not be at zero by default in a cone
     */
    unsigned add_compound_cone(context_t* c, float mass, float radius, float hheight, const Transform* T=NULL, AxisEnum axis=AXIS_Y, unsigned num_boxes=0, unsigned num_spheres=0, const float comOffset[3]=NULL);
    /**
     * @overload
     * @param mMatrix16WithoutScaling a pointer to a 4x4 column-major matrix with only translation and rotation
     */
    unsigned add_compound_cone(context_t* c, float mass, float radius, float hheight, const float* mMatrix16WithoutScaling, AxisEnum axis=AXIS_Y, unsigned num_boxes=0, unsigned num_spheres=0, const float comOffset[3]=NULL);


    /** @} */ // end of extra_group
    } // namespace extra

    /**
     * @defgroup filter_group Filter-Bodies Functions
     * @brief Set of functions regarding body filtering (i.e. body collision group and mask and body flags)
     * @{
     */
    /**
     * @brief Sets the body collision group (a single value of \ref CollisionMaskEnum "COLLISION_GROUP_") and mask (a combination of \ref CollisionMaskEnum "COLLISION_GROUP_ values")
     * @note Please use a single group per body
     */
    inline void body_set_collision_group_and_mask(context_t* c,uint32_t body,CollisionMask single_collision_group_body_belongs_to,CollisionMask collision_group_mask_body_can_collide_with=COLLISION_GROUP_ALL) {
       nudge::BodyFilter* filter = &c->bodies.filters[body];
       filter->collision_group=single_collision_group_body_belongs_to;
       filter->collision_mask=collision_group_mask_body_can_collide_with;
    }
    /**
     * @brief Gets the body collision group (a single value of \ref CollisionMaskEnum "COLLISION_GROUP_")
     * @return a pointer to the body collision group
     * @note This mask indicates the group this body belongs. Please use a single group per body
     */
    inline CollisionMask* body_get_collision_group(context_t* c,uint32_t body) {return &c->bodies.filters[body].collision_group;}
    /**
     * @brief Gets the body collision mask (a combination of \ref CollisionMaskEnum "COLLISION_GROUP_ values")
     * @return a pointer to the body collision mask
     * @note This mask indicates the groups this body wants to collide with.
     */
    inline CollisionMask* body_get_collision_mask(context_t* c,uint32_t body) {return &c->bodies.filters[body].collision_mask;}
    /**
     * @brief Shortcut that returns a pointer to the body flags (a combination of \ref BodyFlagEnum "BF_ enums")
     * @return a pointer to the body flags
     */
    inline FlagMask* body_get_flags(context_t* c,uint32_t body) {return &c->bodies.filters[body].flags;}
    /** @} */ // end of filter_group

    /**
     * @defgroup kinematic_animation_group Kinematic Animation Functions
     * @brief Kinematic animations can be used to move kinematic bodies around automatically; they are designed to be set directly in the \ref KinematicData "KinematicData" struct, so here are just some handy functions to enlarge the animation capacity.
     * @{
     */
    /**
     * @brief Reserves additional space for \ref KinematicData "KinematicData" key frames
     * @param kd \ref KinematicData "KinematicData" instance (typically c->kinematic_data)
     * @param new_size the new size required for the whole key frames (after the call the new capacity could be bigger)
     * @note The effect of this function is only the modification of the key frames capacity
     */
    void kinematic_data_reserve_key_frames(KinematicData* kd, size_t new_size);
    /**
     * @brief Reserves additional space for \ref KinematicData "KinematicData" animations
     * @param kd \ref KinematicData "KinematicData" instance (typically c->kinematic_data)
     * @param new_size the new size required for the whole animation array (after the call the new capacity could be bigger)
     * @note The effect of this function is only the modification of the animation capacity
     */
    void kinematic_data_reserve_animations(KinematicData* kd, size_t new_size);
    /** @} */ // end of kinematic_animation_group  

    /**
     * @defgroup log_group Log Functions
     * @brief Log (e.g. printf) function used by the library and exposed to the user
     * @{
     */
    /**
     * @brief Logging function used by the library. It defaults to printf.
     * @note User can define NUDGE_LOG_FILE_PTR (default: stdout) to redirect output
     * @note User can define NUDGE_VLOG_FUNC(CONST_CHAR_PTR_ARG_PTR , VA_LIST_ARG) (default: vfprintf(NUDGE_LOG_FILE_PTR,CONST_CHAR_PTR_ARG_PTR,VA_LIST_ARG)) to have better control
     */
    int log(const char* format, ...);
    /**
     * @brief Flushes the log. It defaults to fflush(NUDGE_LOG_FILE_PTR)
     * @note When user defines NUDGE_VLOG_FUNC(CONST_CHAR_PTR_ARG_PTR , VA_LIST_ARG), it must define NUDGE_LOG_FLUSH() too
     */
    int flush(void);
    /** @} */ // end of log_group

    /**
     * @defgroup contact_group Contact Processing Functions
     * @brief Contact processing is designed to be done directly in the \ref ContactData "contact_data" field of the \ref context_t "context_t" struct, so here is just a handy function to detect which collider (i.e. collision shape) of each body is involved in a specific collision.
     * @{
     */
    /**
     * @brief Find out which collider belonging to body_a and body_b is involved in the collision determined by a \ref ContactData "ContactData" index
     * @note Only relevant if body_a (or body_b) has more than one collider assigned to it (i.e. it's a compound body)
     */
    void contact_data_find_colliders(const context_t* c,
                                     unsigned contact_data_index,   /**< in [0,c->contact_data.count) */
                                     int16_t* box_collider_index_for_body_a, /**< out param, index in c->colliders.boxes, -1 means that sphere_collider_index_for_body_a>=0 */
                                     int16_t* sphere_collider_index_for_body_a, /**< out param, index in c->colliders.spheres, -1 means that box_collider_index_for_body_a>=0 */
                                     int16_t* box_collider_index_for_body_b, /**< out param, index in c->colliders.boxes, -1 means that sphere_collider_index_for_body_b>=0 */
                                     int16_t* sphere_collider_index_for_body_b,   /**< out param, index in c->colliders.spheres, -1 means that box_collider_index_for_body_b>=0 */
                                     int use_relative_values_for_output_indices=0   /**< 0 or 1. Absolute values can't be stored. Relative values are relative to the first collider index of the body */
                                     );
    /** @} */ // end of contact_group

    /**
     * @defgroup math_group Math Functions
     * @brief General-Math Functions. Functions without the nm_ prefix take as arguments nudge structures (e.g. nudge::Transform), and some of them were present in the original nudge release. The other functions, the ones with the nm_ prefix, have been added to ease user experience, or to support some extended feature, and are more generic.
     * @{
     */
    static const Transform identity_transform = { {}, {0}, { {0.0f, 0.0f, 0.0f, 1.0f} } };
    /**
     * @brief Converts a column-major 16-floats matrix without any scaling applied to a nudge::Transform
     * @param Tout the output nudge::Transform
     * @param matrix16WithourScaling the input column-major 16-floats matrix without any scaling applied
     */
    void Mat4WithoutScalingToTransform(Transform* Tout,const float* matrix16WithourScaling);
    /**
     * @brief Converts a nudge::Transform to a column-major 16-floats matrix
     * @param matrix16Out the output column-major 16-floats matrix
     * @param T the input nudge::Transform
     */
    void TransformToMat4(float* matrix16Out,const Transform* T);
    /**
     * @brief Assigns a new Transform to a body, and sets its linear and angular velocities based on the differences between the new transform and the old one: this is essential when manually moving bodies by changing their transform.
     * @param c the nudge context
     * @param body the target body index
     * @param newT the new transform
     * @param deltaTime the (small) time difference in seconds (used to calculate linear and angular velocities: v = space/time)
     * @param aux_body [experimental] if the aux_body index is >=0, then additional linear and angular velocities are calculated and summed, so that the aux_body 'should' generate a 'dragging' effect on 'body' (the idea was to experiment with a kind of platform-convey effect that does not depend on friction only)
     */
    void TransformAssignToBody(context_t* c,unsigned body,Transform newT,float deltaTime,int16_t aux_body=-1);
    /**
     * @brief Advances the body's transform based on its linear and angular velocities
     * @param c the nudge context
     * @param body the target body index
     * @param deltaTime the (small) time difference in seconds
     */
    void TransformAdvanceBodyFromVelocities(context_t* c,unsigned body,float deltaTime);    // advance body Transform based on its lin and ang velocities
    /**
     * @brief Applies (spherical) lerp between T0 and T1
     * @param T0
     * @param T1
     * @param time must be in the [0,1] interval
     * @return the resulting transform
     */
    Transform TransformSlerp(Transform T0,Transform T1,float time);
    /**
     * @brief Multiplies two transforms
     * @param T0
     * @param T1
     * @return the result of the multiplication
     */
    Transform TransformMul(Transform T0,Transform T1);

    /**
     * @brief Advances a quaternion given an angular velocity and a (small) time step
     * @param qOut4 the resulting quaternion in 4-floats format
     * @param q4 the input quaternion in 4-floats format
     * @param angVel3 the angular velocity in 3-floats format
     * @param halfTimeStep the half of the delta time: must be small for this function to work
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    void nm_QuatAdvance(float* __restrict qOut4,const float* __restrict q4,const float* __restrict angVel3,float halfTimeStep);
    /**
     * @brief Turns the 3x3 submatrix of a 16-floats column-major matrix (without scaling applied) into a quaternion
     * @param result4 the resulting quaternion in 4-floats format
     * @param m16 the input 16-floats column-major matrix (without scaling applied)
     * @return same as result4 (for chaining the call only)
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    float* nm_QuatFromMat4(float* __restrict result4,const float* __restrict m16);  
    /**
     * @brief Replaces the 3x3 submatrix of a 16-floats column-major matrix with the 3x3 matrix representing the given quaternion
     * @param result16 the input/output 16-floats column-major matrix
     * @param q4 the input quaternion in 4-floats format
     * @return same as result16 (for chaining the call only)
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    float* nm_Mat4SetRotationFromQuat(float* __restrict result16,const float* __restrict q4);
    /**
     * @brief Turns the 3x3 9-floats column-major rotation matrix (without scaling applied) into a quaternion
     * @param result4 the resulting quaternion in 4-floats format
     * @param m9 the input 9-floats column-major rotation matrix (without scaling applied)
     * @return same as result4 (for chaining the call only)
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
    */
    float* nm_QuatFromMat3(float* __restrict result4,const float* __restrict m9);
    /**
     * @brief Converts the given quaternion the a 3x3 9-floats column-major rotation matrix
     * @param result9 the input/output 9-floats column-major rotation matrix
     * @param q4 the input quaternion in 4-floats format
     * @return same as result9 (for chaining the call only)
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
    */
    float* nm_Mat3FromQuat(float* __restrict result9,const float* __restrict q4);
    /**
     * @brief Given an old and a new quaternion and a small time step, it calculates the angular velocities
     * @param angVel3 the resulting angular velocity between the two quaternions
     * @param newQuat4 the new quaternion
     * @param oldQuat4 the old quaternion
     * @param halfTimeStep half time step: must be small for this function to work
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    void nm_QuatGetAngularVelocity(float* __restrict angVel3,const float* newQuat4,const float* oldQuat4,float halfTimeStep);
    /**
     * @brief Performs a spherical lerp between two quaternions (in 4-floats format)
     * @param result4 the resulting quaternion
     * @param a4 the first quaternion
     * @param b4 the second quaternion
     * @param slerpTime_In_0_1 time in [0,1] interval
     * @param normalizeResult4AfterLerp if not zero, when the function performs a lerp (because a4 and b4 are close enough) the resulting quaternion is normalized (default: 1)
     * @return same as result4 (for chaining the call only)
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    float* nm_QuatSlerp(float* __restrict result4,const float* __restrict a4,const float* __restrict b4,float slerpTime_In_0_1,int normalizeResult4AfterLerp/*=1*/);
    /**
     * @brief Multiplies two 4-floats quaternions
     * @param qOut4 the result
     * @param a4 the first input 4-floats quaternion
     * @param b4 the second input 4-floats quaternion
     * @return same as qOut4 (for chaining the call only)
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    float* nm_QuatMul(float* /*__restrict*/ qOut4,const float* /*__restrict*/ a4,const float* /*__restrict*/ b4);
    /**
     * @brief normalizes a 4-floats quaternion in place
     * @param q4 the input/output 4-floats quaternion
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    void nm_QuatNormalize(float* __restrict q4);
    /**
     * @brief Generates a 4-floats quaternion based on an orientation arouns an axis
     * @param qOut4 the result
     * @param rfAngle the input angle in radians
     * @param rkAxisX the first component of the (normalized) axis
     * @param rkAxisY the second component of the (normalized) axis
     * @param rkAxisZ the third component of the (normalized) axis
     * @return same as qOut4 (for chaining the call only)
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    float* nm_QuatFromAngleAxis(float* __restrict qOut4,float rfAngle,float rkAxisX,float rkAxisY,float rkAxisZ);
    /**
     * @brief Calculates the angle-axis representation of the given 4-float quaternion
     * @param q4 input quaternion
     * @param rfAngleOut1 output angle in radians
     * @param rkAxisOut3 output 3-floats vector
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    void nm_QuatToAngleAxis(const float* __restrict q4,float* __restrict rfAngleOut1,float* __restrict rkAxisOut3);
    /**
     * @brief Normalizes a 3-floats vector in place
     * @param v3 the input/output 3-floats vector
     * @return the length of the input 3-floats vector
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    float nm_Vec3Normalize(float* __restrict v3);
    /**
     * @brief Get a normalizes copy of an input 3-floats vector
     * @param v3Out the output 3-floats vector
     * @param v3 the input 3-floats vector
     * @return the length of the input 3-floats vector (v3)
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    float nm_Vec3Normalized(float* __restrict v3Out,const float* __restrict v3);
    /**
     * @brief Computes the dot product between two 3-floats vectors
     * @param a3 the first input 3-floats vector
     * @param b3 the second input 3-floats vector
     * @return the resulting dot product
    */
    float nm_Vec3Dot(const float* __restrict a3,const float* __restrict b3);
    /**
     * @brief Computes the cross product between two 3-floats vectors
     * @param vOut3 the output 3-floats vector
     * @param a3 the first input 3-floats vector
     * @param b3 the second input 3-floats vector
     * @return same as vOut3 (for chaining the call only)
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */	
	float* nm_Vec3Cross(float* __restrict vOut3,const float* __restrict a3,const float* __restrict b3);
    
    /**
     * @brief Transforms a 3-floats vector by a 4-floats quaternion
     * @param vOut3 the output 3-floats vector
     * @param q4 the input 4-floats unit quaternion
     * @param vIn3 the input 3-floats vector
     * @return same as vOut3 (for chaining the call only)
     * @note it should be equivalent to: nm_QuatMul(vOut4,q4,vOut4);nm_QuatMul(vOut4,vOut4,qc4); where qc is q conjugate and vOut4 is {vOut3,0}
     * @note This function is never used in nudge.h
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    float* nm_QuatMulVec3(float* __restrict vOut3,const float* __restrict q4,const float* __restrict vIn3);

    /**
     * @brief Transforms a particular axis from the input quaternion space to word space
     * @param vOut3 the output 3-floats vector
     * @param q4 the first input 4-floats unit quaternion
     * @param axisX the first component of the input axis
     * @param axisY the second component of the input axis
     * @param axisZ the third component of the input axis
     * @return same as vOut3 (for chaining the call only)
     * @note if the input axis is not normalized, then user should call nm_Vec3Normalize(vOut3);
     * @note it should be equivalent to: nm_QuatMul(vOut4,q4,vOut4);nm_QuatMul(vOut4,vOut4,qc4); where qc is q conjugate and vOut4 is {vOut3,0}
     * @note This function is never used in nudge.h
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    float* nm_QuatGetAxis(float* __restrict vOut3,const float* __restrict q4,float axisX,float axisY,float axisZ);
    inline float* nm_QuatGetAxisX(float* __restrict axisOut3,const float* __restrict q4) {return nm_QuatGetAxis(axisOut3,q4,1,0,0);}
    inline float* nm_QuatGetAxisY(float* __restrict axisOut3,const float* __restrict q4) {return nm_QuatGetAxis(axisOut3,q4,0,1,0);}
    inline float* nm_QuatGetAxisZ(float* __restrict axisOut3,const float* __restrict q4) {return nm_QuatGetAxis(axisOut3,q4,0,0,1);}

    /**
     * @brief Rotates an input 4-floats unit quaternion by an angle in radians around a specified axis
     * @param qInOut4 the input/output 4-floats unit quaternion
     * @param angle the input angle in radians
     * @param axisX the first component of the input axis
     * @param axisY the second component of the input axis
     * @param axisZ the third component of the input axis
     * @return same as qInOut4 (for chaining the call only)
     * @note This function is never used in nudge.h
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    float* nm_QuatRotate(float* __restrict qInOut4,float angle,float axisX,float axisY,float axisZ);

    /**
     * @brief Multiplies two column-major 16-floats 4x4 matrices
     * @param result16 the resulting column-major 16-floats 4x4 matrix (can be the same as one of the other arguments)
     * @param ml16 the first column-major 16-floats 4x4 matrix (can be the same as one of the other arguments)
     * @param mr16 the second column-major 16-floats 4x4 matrix (can be the same as one of the other arguments)
     * @return the same as result16 (for chaining effect only)
     * @note This function is never used in nudge.h
     * @note Functions with the nm_ prefix have been imported by the <a href="https://github.com/Flix01/Header-Only-GL-Helpers/blob/master/minimath.h">minimath library</a>  (MIT license). These functions might be renamed, removed or hidden in the future
     */
    float* nm_Mat4Mul(float* result16,const float* ml16,const float* mr16);
    /** @} */ // end of math_group

    /**
     * @defgroup inertia_group Inertia Calculation Functions
     * @brief Helper functions to calculate the inertia vector of a body (to feed \ref add_compound "add_compound(...)"), or its inverse (to feed c->bodies.properties[body].inertia_inverse directly)
     * @{
     */
    void calculate_box_inertia(float result[3],float mass,float hsizex,float hsizey,float hsizez,const float comOffset[3]=NULL);
    void calculate_sphere_inertia(float result[3],float mass,float radius,const float comOffset[3]=NULL,bool hollow=false);
    void calculate_cylinder_inertia(float result[3],float mass,float radius,float halfHeight,AxisEnum upAxis=AXIS_Y,const float comOffset[3]=NULL);
    void calculate_capsule_inertia(float result[3],float mass,float radius,float halfCylinderHeight,AxisEnum upAxis=AXIS_Y,const float comOffset[3]=NULL);
    void calculate_torus_inertia(float result[3],float mass,float majorRadius,float minorRadius,AxisEnum upAxis=AXIS_Y,const float comOffset[3]=NULL);
    void calculate_hollow_cylinder_inertia(float result[3], float mass, float R, float r, float halfHeight, AxisEnum upAxis=AXIS_Y, const float comOffset[3]=NULL);
    void calculate_cone_inertia(float result[3],float mass,float radius,float halfHeight,AxisEnum upAxis=AXIS_Y,const float comOffset[3]=NULL);

    void calculate_box_inertia_inverse(float result[3],float mass,float hsizex,float hsizey,float hsizez,const float comOffset[3]=NULL);
    void calculate_sphere_inertia_inverse(float result[3],float mass,float radius,const float comOffset[3]=NULL,bool hollow=false);
    void calculate_cylinder_inertia_inverse(float result[3], float mass, float radius, float halfHeight, AxisEnum upAxis=AXIS_Y, const float comOffset[3]=NULL);
    void calculate_capsule_inertia_inverse(float result[3], float mass, float radius, float halfCylinderHeight, AxisEnum upAxis=AXIS_Y, const float comOffset[3]=NULL);
    void calculate_torus_inertia_inverse(float result[3],float mass,float majorRadius,float minorRadius,AxisEnum upAxis=AXIS_Y,const float comOffset[3]=NULL);
    void calculate_hollow_cylinder_inertia_inverse(float result[3], float mass, float R, float r, float halfHeight, AxisEnum upAxis=AXIS_Y, const float comOffset[3]=NULL);
    void calculate_cone_inertia_inverse(float result[3], float mass, float radius, float halfHeight, AxisEnum upAxis=AXIS_Y, const float comOffset[3]=NULL);
    /** @} */ // end of inertia



#   ifndef M_PIOVER180
#       define M_PIOVER180 ((float)(3.14159265358979323846/180.0))
#   endif
#   ifndef M_180OVERPI
#       define M_180OVERPI ((float)(180.0/3.14159265358979323846))
#   endif
#   ifndef M_DEG2RAD
#       define M_DEG2RAD(X)   ((X)*(float)M_PIOVER180)
#   endif
#   ifndef M_RAD2DEG
#       define M_RAD2DEG(X)   ((X)*(float)M_180OVERPI)
#   endif

#   ifndef NUDGE_NO_STDIO
#   ifdef NUDGE_USE_TIME_CONTEXT
    void save_time_context(FILE* f,const time_context_t* c);
    void load_time_context(FILE* f,time_context_t* c);
#   endif //NUDGE_USE_TIME_CONTEXT
#   endif //NUDGE_NO_STDIO

#ifndef NUDGE_DEFAULT_SIMULATION_TIMESTEP
#   define NUDGE_DEFAULT_SIMULATION_TIMESTEP (1.0/60.0)
#endif
#ifndef NUDGE_DEFAULT_MAX_NUM_SIMULATION_SUBSTEPS
#   define NUDGE_DEFAULT_MAX_NUM_SIMULATION_SUBSTEPS (2) //2-10
#endif
#ifndef NUDGE_DEFAULT_NUM_SIMULATION_ITERATIONS
#   define NUDGE_DEFAULT_NUM_SIMULATION_ITERATIONS (5)  //5-350
#endif
#ifndef NUDGE_DEFAULT_DAMPING_LINEAR
#   define NUDGE_DEFAULT_DAMPING_LINEAR (0.25f)
#endif
#ifndef NUDGE_DEFAULT_DAMPING_ANGULAR
#   define NUDGE_DEFAULT_DAMPING_ANGULAR (0.25f)
#endif
#ifndef NUDGE_DEFAULT_SLEEPING_THRESHOLD_LINEAR_VELOCITY_SQUARED
#   define NUDGE_DEFAULT_SLEEPING_THRESHOLD_LINEAR_VELOCITY_SQUARED (1e-2f)
#endif
#ifndef NUDGE_DEFAULT_SLEEPING_THRESHOLD_ANGULAR_VELOCITY_SQUARED
#   define NUDGE_DEFAULT_SLEEPING_THRESHOLD_ANGULAR_VELOCITY_SQUARED (1e-1f)
#endif
#ifndef NUDGE_DEFAULT_PENETRATION_ALLOWED_AMOUNT
#   define NUDGE_DEFAULT_PENETRATION_ALLOWED_AMOUNT (1e-3f)
#endif
#ifndef NUDGE_DEFAULT_PENETRATION_BIAS_FACTOR
#   define NUDGE_DEFAULT_PENETRATION_BIAS_FACTOR (2.0f)
#endif

    //--------------------------------------------------------------------------------------------------------------------------       

} // namespace nudge

#endif // NUDGE_H






//--- Hack for better code completion on QtCreator (to remove? no)-------------
#if (!defined(HELLO_WORLD_CPP_) && !defined(EXAMPLE02_CPP_) && !defined(NUDGE_IMPLEMENTATION) && defined(NUDGE_DEVELOPMENT))
    #define NUDGE_IMPLEMENTATION
#endif
//-----------------------------------------------------------------------------

#ifdef NUDGE_IMPLEMENTATION
#ifndef NUDGE_IMPLEMENTATION_GUARD
#define NUDGE_IMPLEMENTATION_GUARD

#include <assert.h>

#ifdef NUDGE_USE_SIMDE
#   ifndef SIMDE_ENABLE_NATIVE_ALIASES
//#       error Please define SIMDE_ENABLE_NATIVE_ALIASES globally and recompile
#       define SIMDE_ENABLE_NATIVE_ALIASES
#   endif
//#   include "./simde/x86/avx2.h"
#   include "./simde/x86/sse2.h"
#ifndef NUDGE_SIMDE_USE_CUSTOM_MM_MALLOC
#   include <mm_malloc.h>  // _mm_malloc and _mm_free
#endif
#else
#   include <immintrin.h>
#endif

#include <math.h>
#include <string.h>

#ifdef __MSC_VER//_WIN32
#include <intrin.h>
#define NUDGE_ALIGNED(n) __declspec(align(n))
#define NUDGE_FORCEINLINE __forceinline
#else
#define NUDGE_ALIGNED(n) __attribute__((aligned(n)))
#define NUDGE_FORCEINLINE inline __attribute__((always_inline))
#endif

#ifdef __AVX2__
#define NUDGE_SIMDV_WIDTH 256
#else
#define NUDGE_SIMDV_WIDTH 128
#endif

#define NUDGE_ARENA_SCOPE(A) Arena& scope_arena_##A = A; Arena A = scope_arena_##A


//---- LOGGING IMPLEMENTATION -----------------------------------
namespace nudge {
    int dummy_vprintf(const char* /*format*/, va_list /*vlist*/ ) {return 0;}
    int dummy_flush(void) {return 0;}
}
#ifdef NUDGE_NO_STDIO
#   ifndef NUDGE_VLOG_FUNC
//#		error Please define the two macros NUDGE_VLOG_FUNC(FORMAT,VLIST) and NUDGE_LOG_FLUSH() for custom logging without stdio.h
#       define NUDGE_VLOG_FUNC(A,B) nudge::dummy_vprintf((A),(B))
#       undef NUDGE_LOG_FLUSH
#   endif //NUDGE_VLOG_FUNC
#endif // NUDGE_NO_STDIO
#ifndef NUDGE_VLOG_FUNC
#	include <stdio.h>
#	ifndef NUDGE_LOG_FILE_PTR
        /**
         * \def NUDGE_LOG_FILE_PTR
         * Definition that specifies the FILE pointer used for logging when <stdio.h> in included (i.e. NUDGE_NOSTDIO is not defined) (it defaults to stdout)
         */
#		define NUDGE_LOG_FILE_PTR (stdout)
#	endif // NUDGE_LOG_FILE_PTR
    // int vprintf(const char* format, va_list vlist);
    /**
     * \macro NUDGE_VLOG_FUNC
     * Macro that specifies the vlog(const char* format, va_list vlist) function used for logging (it defaults to vfprintf(\ref NUDGE_LOG_FILE_PTR "NUDGE_LOG_FILE_PTR", format, vlist))
     */
#	define NUDGE_VLOG_FUNC(CONST_CHAR_PTR_ARG_PTR,VA_LIST_ARG) vfprintf(NUDGE_LOG_FILE_PTR,CONST_CHAR_PTR_ARG_PTR,VA_LIST_ARG)
    /**
     * \def NUDGE_LOG_FLUSH
     * Macro that specifies the flush(void) fuction used for logging (it defaults to fflush(\ref NUDGE_LOG_FILE_PTR "NUDGE_LOG_FILE_PTR"))
     */
#	define NUDGE_LOG_FLUSH() fflush(NUDGE_LOG_FILE_PTR) // int fflush(FILE*)
#endif // NUDGE_VLOG_FUNC
#ifndef NUDGE_LOG_FLUSH
#   define NUDGE_LOG_FLUSH() nudge::dummy_flush()
#endif //NUDGE_LOG_FLUSH
//--------------------------------------------------------------


namespace nudge {

int log(const char* format, ...) {va_list ap;va_start(ap, format);int rv=NUDGE_VLOG_FUNC(format, ap);va_end(ap);return rv;}
int flush(void) {return NUDGE_LOG_FLUSH();}


#if NUDGE_SIMDV_WIDTH == 128
#define NUDGE_SIMDV_ALIGNED NUDGE_ALIGNED(16)
static const unsigned simdv_width32 = 4;
static const unsigned simdv_width32_log2 = 2;
#elif NUDGE_SIMDV_WIDTH == 256
#define NUDGE_SIMDV_ALIGNED NUDGE_ALIGNED(32)
static const unsigned simdv_width32 = 8;
static const unsigned simdv_width32_log2 = 3;
#endif

#ifdef __MSC_VER//_WIN32
NUDGE_FORCEINLINE __m128 operator - (__m128 a) {
    return _mm_xor_ps(a, _mm_set1_ps(-0.0f));
}

NUDGE_FORCEINLINE __m128 operator + (__m128 a, __m128 b) {
    return _mm_add_ps(a, b);
}

NUDGE_FORCEINLINE __m128 operator - (__m128 a, __m128 b) {
    return _mm_sub_ps(a, b);
}

NUDGE_FORCEINLINE __m128 operator * (__m128 a, __m128 b) {
    return _mm_mul_ps(a, b);
}

NUDGE_FORCEINLINE __m128 operator / (__m128 a, __m128 b) {
    return _mm_div_ps(a, b);
}

NUDGE_FORCEINLINE __m128& operator += (__m128& a, __m128 b) {
    return a = _mm_add_ps(a, b);
}

NUDGE_FORCEINLINE __m128& operator -= (__m128& a, __m128 b) {
    return a = _mm_sub_ps(a, b);
}

NUDGE_FORCEINLINE __m128& operator *= (__m128& a, __m128 b) {
    return a = _mm_mul_ps(a, b);
}

NUDGE_FORCEINLINE __m128& operator /= (__m128& a, __m128 b) {
    return a = _mm_div_ps(a, b);
}
#ifdef __AVX2__
NUDGE_FORCEINLINE __m256 operator - (__m256 a) {
    return _mm256_xor_ps(a, _mm256_set1_ps(-0.0f));
}

NUDGE_FORCEINLINE __m256 operator + (__m256 a, __m256 b) {
    return _mm256_add_ps(a, b);
}

NUDGE_FORCEINLINE __m256 operator - (__m256 a, __m256 b) {
    return _mm256_sub_ps(a, b);
}

NUDGE_FORCEINLINE __m256 operator * (__m256 a, __m256 b) {
    return _mm256_mul_ps(a, b);
}

NUDGE_FORCEINLINE __m256 operator / (__m256 a, __m256 b) {
    return _mm256_div_ps(a, b);
}

NUDGE_FORCEINLINE __m256& operator += (__m256& a, __m256 b) {
    return a = _mm256_add_ps(a, b);
}

NUDGE_FORCEINLINE __m256& operator -= (__m256& a, __m256 b) {
    return a = _mm256_sub_ps(a, b);
}

NUDGE_FORCEINLINE __m256& operator *= (__m256& a, __m256 b) {
    return a = _mm256_mul_ps(a, b);
}

NUDGE_FORCEINLINE __m256& operator /= (__m256& a, __m256 b) {
    return a = _mm256_div_ps(a, b);
}
#endif
#endif

typedef __m128 simd4_float;
typedef __m128i simd4_int32;

namespace simd128 {
    NUDGE_FORCEINLINE __m128 unpacklo32(__m128 x, __m128 y) {
        return _mm_unpacklo_ps(x, y);
    }

    NUDGE_FORCEINLINE __m128 unpackhi32(__m128 x, __m128 y) {
        return _mm_unpackhi_ps(x, y);
    }

    NUDGE_FORCEINLINE __m128i unpacklo32(__m128i x, __m128i y) {
        return _mm_unpacklo_epi32(x, y);
    }

    NUDGE_FORCEINLINE __m128i unpackhi32(__m128i x, __m128i y) {
        return _mm_unpackhi_epi32(x, y);
    }

    template<unsigned x0, unsigned x1, unsigned y0, unsigned y1>
    NUDGE_FORCEINLINE __m128 concat2x32(__m128 x, __m128 y) {
        return _mm_shuffle_ps(x, y, _MM_SHUFFLE(y1, y0, x1, x0));
    }

    template<unsigned i0, unsigned i1, unsigned i2, unsigned i3>
    NUDGE_FORCEINLINE __m128 shuffle32(__m128 x) {
        return _mm_shuffle_ps(x, x, _MM_SHUFFLE(i3, i2, i1, i0));
    }

    template<unsigned i0, unsigned i1, unsigned i2, unsigned i3>
    NUDGE_FORCEINLINE __m128i shuffle32(__m128i x) {
        return _mm_shuffle_epi32(x, _MM_SHUFFLE(i3, i2, i1, i0));
    }

    NUDGE_FORCEINLINE void transpose32(simd4_float& x, simd4_float& y, simd4_float& z, simd4_float& w) {
        _MM_TRANSPOSE4_PS(x, y, z, w);
    }
}

namespace simd {
    NUDGE_FORCEINLINE unsigned signmask32(__m128 x) {
        return _mm_movemask_ps(x);
    }

    NUDGE_FORCEINLINE unsigned signmask32(__m128i x) {
        return _mm_movemask_ps(_mm_castsi128_ps(x));
    }

    NUDGE_FORCEINLINE __m128 bitwise_xor(__m128 x, __m128 y) {
        return _mm_xor_ps(x, y);
    }

    NUDGE_FORCEINLINE __m128 bitwise_or(__m128 x, __m128 y) {
        return _mm_or_ps(x, y);
    }

    NUDGE_FORCEINLINE __m128 bitwise_and(__m128 x, __m128 y) {
        return _mm_and_ps(x, y);
    }

    NUDGE_FORCEINLINE __m128 bitwise_notand(__m128 x, __m128 y) {
        return _mm_andnot_ps(x, y);
    }

    NUDGE_FORCEINLINE __m128i bitwise_xor(__m128i x, __m128i y) {
        return _mm_xor_si128(x, y);
    }

    NUDGE_FORCEINLINE __m128i bitwise_or(__m128i x, __m128i y) {
        return _mm_or_si128(x, y);
    }

    NUDGE_FORCEINLINE __m128i bitwise_and(__m128i x, __m128i y) {
        return _mm_and_si128(x, y);
    }

    NUDGE_FORCEINLINE __m128i bitwise_notand(__m128i x, __m128i y) {
        return _mm_andnot_si128(x, y);
    }

    NUDGE_FORCEINLINE __m128 blendv32(__m128 x, __m128 y, __m128 s) {
#if defined(__SSE4_1__) || defined(__AVX__)
#define NUDGE_NATIVE_BLENDV32
        return _mm_blendv_ps(x, y, s);
#else
        s = _mm_castsi128_ps(_mm_srai_epi32(_mm_castps_si128(s), 31));
        return _mm_or_ps(_mm_andnot_ps(s, x), _mm_and_ps(s, y));
#endif
    }

    NUDGE_FORCEINLINE __m128i blendv32(__m128i x, __m128i y, __m128i s) {
        return _mm_castps_si128(blendv32(_mm_castsi128_ps(x), _mm_castsi128_ps(y), _mm_castsi128_ps(s)));
    }
}

namespace simd_float {
    NUDGE_FORCEINLINE float extract_first_float(simd4_float x) {
        return _mm_cvtss_f32(x);
    }

    NUDGE_FORCEINLINE simd4_float zero4() {
        return _mm_setzero_ps();
    }

    NUDGE_FORCEINLINE simd4_float make4(float x) {
        return _mm_set1_ps(x);
    }

    NUDGE_FORCEINLINE simd4_float make4(float x, float y, float z, float w) {
        return _mm_setr_ps(x, y, z, w);
    }

    NUDGE_FORCEINLINE simd4_float broadcast_load4(const float* p) {
        return _mm_set1_ps(*p);
    }

    NUDGE_FORCEINLINE simd4_float load4(const float* p) {
        return _mm_load_ps(p);
    }

    NUDGE_FORCEINLINE simd4_float loadu4(const float* p) {
        return _mm_loadu_ps(p);
    }

    NUDGE_FORCEINLINE void store4(float* p, simd4_float x) {
        _mm_store_ps(p, x);
    }

    NUDGE_FORCEINLINE void storeu4(float* p, simd4_float x) {
        _mm_storeu_ps(p, x);
    }

    NUDGE_FORCEINLINE simd4_float madd(simd4_float x, simd4_float y, simd4_float z) {
#ifdef __FMA__
        return _mm_fmadd_ps(x, y, z);
#else
        return _mm_add_ps(_mm_mul_ps(x, y), z);
#endif
    }

    NUDGE_FORCEINLINE simd4_float msub(simd4_float x, simd4_float y, simd4_float z) {
#ifdef __FMA__
        return _mm_fmsub_ps(x, y, z);
#else
        return _mm_sub_ps(_mm_mul_ps(x, y), z);
#endif
    }

    // Note: First operand is returned on NaN.
    NUDGE_FORCEINLINE simd4_float min(simd4_float x, simd4_float y) {
        return _mm_min_ps(y, x); // Note: For SSE, second operand is returned on NaN.
    }

    // Note: First operand is returned on NaN.
    NUDGE_FORCEINLINE simd4_float max(simd4_float x, simd4_float y) {
        return _mm_max_ps(y, x); // Note: For SSE, second operand is returned on NaN.
    }

    NUDGE_FORCEINLINE simd4_float rsqrt(simd4_float x) {
        return _mm_rsqrt_ps(x);
    }

    NUDGE_FORCEINLINE simd4_float recip(simd4_float x) {
        return _mm_rcp_ps(x);
    }

    NUDGE_FORCEINLINE simd4_float sqrt(simd4_float x) {
        return _mm_sqrt_ps(x);
    }

    NUDGE_FORCEINLINE simd4_float abs(simd4_float x) {
        return _mm_andnot_ps(_mm_set1_ps(-0.0f), x);
    }

    NUDGE_FORCEINLINE simd4_float cmp_gt(simd4_float x, simd4_float y) {
        return _mm_cmpgt_ps(x, y);
    }

    NUDGE_FORCEINLINE simd4_float cmp_ge(simd4_float x, simd4_float y) {
        return _mm_cmpge_ps(x, y);
    }

    NUDGE_FORCEINLINE simd4_float cmp_le(simd4_float x, simd4_float y) {
        return _mm_cmple_ps(x, y);
    }

    NUDGE_FORCEINLINE simd4_float cmp_eq(simd4_float x, simd4_float y) {
        return _mm_cmpeq_ps(x, y);
    }

    NUDGE_FORCEINLINE simd4_float cmp_neq(simd4_float x, simd4_float y) {
        return _mm_cmpneq_ps(x, y);
    }

    NUDGE_FORCEINLINE simd4_int32 asint(simd4_float x) {
        return _mm_castps_si128(x);
    }

    NUDGE_FORCEINLINE simd4_int32 toint(simd4_float x) {
        return _mm_cvttps_epi32(x);
    }
}

namespace simd_int32 {
    NUDGE_FORCEINLINE simd4_int32 zero4() {
        return _mm_setzero_si128();
    }

    NUDGE_FORCEINLINE simd4_int32 make4(int32_t x) {
        return _mm_set1_epi32(x);
    }

    NUDGE_FORCEINLINE simd4_int32 make4(int32_t x, int32_t y, int32_t z, int32_t w) {
        return _mm_setr_epi32(x, y, z, w);
    }

    NUDGE_FORCEINLINE simd4_int32 load4(const int32_t* p) {
        return _mm_load_si128((const __m128i*)p);
    }

    NUDGE_FORCEINLINE simd4_int32 loadu4(const int32_t* p) {
        return _mm_loadu_si128((const __m128i*)p);
    }

    NUDGE_FORCEINLINE void store4(int32_t* p, simd4_int32 x) {
        _mm_store_si128((__m128i*)p, x);
    }

    NUDGE_FORCEINLINE void storeu4(int32_t* p, simd4_int32 x) {
        _mm_storeu_si128((__m128i*)p, x);
    }

    template<unsigned bits>
    NUDGE_FORCEINLINE simd4_int32 shift_left(simd4_int32 x) {
        return _mm_slli_epi32(x, bits);
    }

    template<unsigned bits>
    NUDGE_FORCEINLINE simd4_int32 shift_right(simd4_int32 x) {
        return _mm_srli_epi32(x, bits);
    }

    NUDGE_FORCEINLINE simd4_int32 add(simd4_int32 x, simd4_int32 y) {
        return _mm_add_epi32(x, y);
    }

    NUDGE_FORCEINLINE simd4_int32 cmp_eq(simd4_int32 x, simd4_int32 y) {
        return _mm_cmpeq_epi32(x, y);
    }

    NUDGE_FORCEINLINE simd4_float asfloat(simd4_int32 x) {
        return _mm_castsi128_ps(x);
    }
}

#ifdef __AVX2__
typedef __m256 simd8_float;
typedef __m256i simd8_int32;

namespace simd128 {
    NUDGE_FORCEINLINE __m256 unpacklo32(__m256 x, __m256 y) {
        return _mm256_unpacklo_ps(x, y);
    }

    NUDGE_FORCEINLINE __m256 unpackhi32(__m256 x, __m256 y) {
        return _mm256_unpackhi_ps(x, y);
    }

    NUDGE_FORCEINLINE __m256i unpacklo32(__m256i x, __m256i y) {
        return _mm256_unpacklo_epi32(x, y);
    }

    NUDGE_FORCEINLINE __m256i unpackhi32(__m256i x, __m256i y) {
        return _mm256_unpackhi_epi32(x, y);
    }

    template<unsigned x0, unsigned x1, unsigned y0, unsigned y1>
    NUDGE_FORCEINLINE __m256 concat2x32(__m256 x, __m256 y) {
        return _mm256_shuffle_ps(x, y, _MM_SHUFFLE(y1, y0, x1, x0));
    }

    template<unsigned i0, unsigned i1, unsigned i2, unsigned i3>
    NUDGE_FORCEINLINE __m256 shuffle32(__m256 x) {
        return _mm256_shuffle_ps(x, x, _MM_SHUFFLE(i3, i2, i1, i0));
    }

    template<unsigned i0, unsigned i1, unsigned i2, unsigned i3>
    NUDGE_FORCEINLINE __m256i shuffle32(__m256i x) {
        return _mm256_shuffle_epi32(x, _MM_SHUFFLE(i3, i2, i1, i0));
    }

    NUDGE_FORCEINLINE void transpose32(simd8_float& x, simd8_float& y, simd8_float& z, simd8_float& w) {
        __m256 t0 = _mm256_unpacklo_ps(x, y);
        __m256 t1 = _mm256_unpacklo_ps(z, w);
        __m256 t2 = _mm256_unpackhi_ps(x, y);
        __m256 t3 = _mm256_unpackhi_ps(z, w);
        x = _mm256_shuffle_ps(t0, t1, _MM_SHUFFLE(1,0,1,0));
        y = _mm256_shuffle_ps(t0, t1, _MM_SHUFFLE(3,2,3,2));
        z = _mm256_shuffle_ps(t2, t3, _MM_SHUFFLE(1,0,1,0));
        w = _mm256_shuffle_ps(t2, t3, _MM_SHUFFLE(3,2,3,2));
    }
}

namespace simd256 {
    template<unsigned i0, unsigned i1>
    NUDGE_FORCEINLINE simd8_float permute128(simd8_float x, simd8_float y) {
        return _mm256_castsi256_ps(_mm256_permute2x128_si256(_mm256_castps_si256(x), _mm256_castps_si256(y), i0 | (i1 << 4)));
    }

    template<unsigned i0, unsigned i1>
    NUDGE_FORCEINLINE simd8_int32 permute128(simd8_int32 x, simd8_int32 y) {
        return _mm256_permute2x128_si256(x, y, i0 | (i1 << 4));
    }

    template<unsigned i0, unsigned i1>
    NUDGE_FORCEINLINE simd8_float shuffle128(simd8_float x) {
        return _mm256_castsi256_ps(_mm256_permute2x128_si256(_mm256_castps_si256(x), _mm256_castps_si256(x), i0 | (i1 << 4)));
    }

    template<unsigned i0, unsigned i1>
    NUDGE_FORCEINLINE simd8_int32 shuffle128(simd8_int32 x) {
        return _mm256_permute2x128_si256(x, x, i0 | (i1 << 4));
    }

    NUDGE_FORCEINLINE simd8_float broadcast(simd4_float x) {
        return _mm256_insertf128_ps(_mm256_castps128_ps256(x), x, 1);
    }

    NUDGE_FORCEINLINE simd8_int32 broadcast(simd4_int32 x) {
        return _mm256_insertf128_si256(_mm256_castsi128_si256(x), x, 1);
    }
}

namespace simd {
    NUDGE_FORCEINLINE simd8_float concat(simd4_float x, simd4_float y) {
        return _mm256_insertf128_ps(_mm256_castps128_ps256(x), y, 1);
    }

    NUDGE_FORCEINLINE simd4_float extract_low(simd8_float x) {
        return _mm256_castps256_ps128(x);
    }

    NUDGE_FORCEINLINE simd4_float extract_high(simd8_float x) {
        return _mm256_extractf128_ps(x, 1);
    }

    NUDGE_FORCEINLINE simd4_int32 extract_low(simd8_int32 x) {
        return _mm256_castsi256_si128(x);
    }

    NUDGE_FORCEINLINE simd4_int32 extract_high(simd8_int32 x) {
        return _mm256_extractf128_si256(x, 1);
    }

    NUDGE_FORCEINLINE unsigned signmask32(__m256 x) {
        return _mm256_movemask_ps(x);
    }

    NUDGE_FORCEINLINE unsigned signmask32(__m256i x) {
        return _mm256_movemask_ps(_mm256_castsi256_ps(x));
    }

    NUDGE_FORCEINLINE __m256 bitwise_xor(__m256 x, __m256 y) {
        return _mm256_xor_ps(x, y);
    }

    NUDGE_FORCEINLINE __m256 bitwise_or(__m256 x, __m256 y) {
        return _mm256_or_ps(x, y);
    }

    NUDGE_FORCEINLINE __m256 bitwise_and(__m256 x, __m256 y) {
        return _mm256_and_ps(x, y);
    }

    NUDGE_FORCEINLINE __m256 bitwise_notand(__m256 x, __m256 y) {
        return _mm256_andnot_ps(x, y);
    }

    NUDGE_FORCEINLINE __m256i bitwise_xor(__m256i x, __m256i y) {
        return _mm256_xor_si256(x, y);
    }

    NUDGE_FORCEINLINE __m256i bitwise_or(__m256i x, __m256i y) {
        return _mm256_or_si256(x, y);
    }

    NUDGE_FORCEINLINE __m256i bitwise_and(__m256i x, __m256i y) {
        return _mm256_and_si256(x, y);
    }

    NUDGE_FORCEINLINE __m256i bitwise_notand(__m256i x, __m256i y) {
        return _mm256_andnot_si256(x, y);
    }

    NUDGE_FORCEINLINE __m256 blendv32(__m256 x, __m256 y, __m256 s) {
        return _mm256_blendv_ps(x, y, s);
    }

    NUDGE_FORCEINLINE __m256i blendv32(__m256i x, __m256i y, __m256i s) {
        return _mm256_castps_si256(_mm256_blendv_ps(_mm256_castsi256_ps(x), _mm256_castsi256_ps(y), _mm256_castsi256_ps(s)));
    }
}

namespace simd_float {
    NUDGE_FORCEINLINE float extract_first_float(simd8_float x) {
        return _mm_cvtss_f32(_mm256_castps256_ps128(x));
    }

    NUDGE_FORCEINLINE simd8_float zero8() {
        return _mm256_setzero_ps();
    }

    NUDGE_FORCEINLINE simd8_float make8(float x) {
        return _mm256_set1_ps(x);
    }

    NUDGE_FORCEINLINE simd8_float make8(float x0, float y0, float z0, float w0, float x1, float y1, float z1, float w1) {
        return _mm256_setr_ps(x0, y0, z0, w0, x1, y1, z1, w1);
    }

    NUDGE_FORCEINLINE simd8_float broadcast_load8(const float* p) {
        return _mm256_broadcast_ss(p);
    }

    NUDGE_FORCEINLINE simd8_float load8(const float* p) {
        return _mm256_load_ps(p);
    }

    NUDGE_FORCEINLINE simd8_float loadu8(const float* p) {
        return _mm256_loadu_ps(p);
    }

    NUDGE_FORCEINLINE void store8(float* p, simd8_float x) {
        _mm256_store_ps(p, x);
    }

    NUDGE_FORCEINLINE void storeu8(float* p, simd8_float x) {
        _mm256_storeu_ps(p, x);
    }

    NUDGE_FORCEINLINE simd8_float madd(simd8_float x, simd8_float y, simd8_float z) {
#ifdef __FMA__
        return _mm256_fmadd_ps(x, y, z);
#else
        return _mm256_add_ps(_mm256_mul_ps(x, y), z);
#endif
    }

    NUDGE_FORCEINLINE simd8_float msub(simd8_float x, simd8_float y, simd8_float z) {
#ifdef __FMA__
        return _mm256_fmsub_ps(x, y, z);
#else
        return _mm256_sub_ps(_mm256_mul_ps(x, y), z);
#endif
    }

    // Note: First operand is returned on NaN.
    NUDGE_FORCEINLINE simd8_float min(simd8_float x, simd8_float y) {
        return _mm256_min_ps(y, x); // Note: For SSE, second operand is returned on NaN.
    }

    // Note: First operand is returned on NaN.
    NUDGE_FORCEINLINE simd8_float max(simd8_float x, simd8_float y) {
        return _mm256_max_ps(y, x); // Note: For SSE, second operand is returned on NaN.
    }

    NUDGE_FORCEINLINE simd8_float rsqrt(simd8_float x) {
        return _mm256_rsqrt_ps(x);
    }

    NUDGE_FORCEINLINE simd8_float recip(simd8_float x) {
        return _mm256_rcp_ps(x);
    }

    NUDGE_FORCEINLINE simd8_float sqrt(simd8_float x) {
        return _mm256_sqrt_ps(x);
    }

    NUDGE_FORCEINLINE simd8_float abs(simd8_float x) {
        return _mm256_andnot_ps(_mm256_set1_ps(-0.0f), x);
    }

    NUDGE_FORCEINLINE simd8_float cmp_gt(simd8_float x, simd8_float y) {
        return _mm256_cmp_ps(x, y, _CMP_GT_OQ);
    }

    NUDGE_FORCEINLINE simd8_float cmp_ge(simd8_float x, simd8_float y) {
        return _mm256_cmp_ps(x, y, _CMP_GE_OQ);
    }

    NUDGE_FORCEINLINE simd8_float cmp_le(simd8_float x, simd8_float y) {
        return _mm256_cmp_ps(x, y, _CMP_LE_OQ);
    }

    NUDGE_FORCEINLINE simd8_float cmp_eq(simd8_float x, simd8_float y) {
        return _mm256_cmp_ps(x, y, _CMP_EQ_OQ);
    }

    NUDGE_FORCEINLINE simd8_float cmp_neq(simd8_float x, simd8_float y) {
        return _mm256_cmp_ps(x, y, _CMP_NEQ_OQ);
    }

    NUDGE_FORCEINLINE simd8_int32 asint(simd8_float x) {
        return _mm256_castps_si256(x);
    }

    NUDGE_FORCEINLINE simd8_int32 toint(simd8_float x) {
        return _mm256_cvttps_epi32(x);
    }
}

namespace simd_int32 {
    NUDGE_FORCEINLINE simd8_int32 zero8() {
        return _mm256_setzero_si256();
    }

    NUDGE_FORCEINLINE simd8_int32 make8(int32_t x) {
        return _mm256_set1_epi32(x);
    }

    NUDGE_FORCEINLINE simd8_int32 make8(int32_t x0, int32_t y0, int32_t z0, int32_t w0, int32_t x1, int32_t y1, int32_t z1, int32_t w1) {
        return _mm256_setr_epi32(x0, y0, z0, w0, x1, y1, z1, w1);
    }

    NUDGE_FORCEINLINE simd8_int32 load8(const int32_t* p) {
        return _mm256_load_si256((const __m256i*)p);
    }

    NUDGE_FORCEINLINE simd8_int32 loadu8(const int32_t* p) {
        return _mm256_loadu_si256((const __m256i*)p);
    }

    NUDGE_FORCEINLINE void store8(int32_t* p, simd8_int32 x) {
        _mm256_store_si256((__m256i*)p, x);
    }

    NUDGE_FORCEINLINE void storeu8(int32_t* p, simd8_int32 x) {
        _mm256_storeu_si256((__m256i*)p, x);
    }

    template<unsigned bits>
    NUDGE_FORCEINLINE simd8_int32 shift_left(simd8_int32 x) {
        return _mm256_slli_epi32(x, bits);
    }

    template<unsigned bits>
    NUDGE_FORCEINLINE simd8_int32 shift_right(simd8_int32 x) {
        return _mm256_srli_epi32(x, bits);
    }

    NUDGE_FORCEINLINE simd8_int32 add(simd8_int32 x, simd8_int32 y) {
        return _mm256_add_epi32(x, y);
    }

    NUDGE_FORCEINLINE simd8_int32 cmp_eq(simd8_int32 x, simd8_int32 y) {
        return _mm256_cmpeq_epi32(x, y);
    }

    NUDGE_FORCEINLINE simd8_float asfloat(simd8_int32 x) {
        return _mm256_castsi256_ps(x);
    }
}
#endif

#if NUDGE_SIMDV_WIDTH == 128
typedef simd4_float simdv_float;
typedef simd4_int32 simdv_int32;

namespace simd_float {
    NUDGE_FORCEINLINE simdv_float zerov() {
        return zero4();
    }

    NUDGE_FORCEINLINE simdv_float makev(float x) {
        return make4(x);
    }

    NUDGE_FORCEINLINE simdv_float broadcast_loadv(const float* p) {
        return broadcast_load4(p);
    }

    NUDGE_FORCEINLINE simdv_float loadv(const float* p) {
        return load4(p);
    }

    NUDGE_FORCEINLINE simdv_float loaduv(const float* p) {
        return loadu4(p);
    }

    NUDGE_FORCEINLINE void storev(float* p, simdv_float x) {
        store4(p, x);
    }

    NUDGE_FORCEINLINE void storeuv(float* p, simdv_float x) {
        storeu4(p, x);
    }
}

namespace simd_int32 {
    NUDGE_FORCEINLINE simdv_int32 zerov() {
        return zero4();
    }

    NUDGE_FORCEINLINE simdv_int32 makev(int32_t x) {
        return make4(x);
    }

    NUDGE_FORCEINLINE simdv_int32 loadv(const int32_t* p) {
        return load4(p);
    }

    NUDGE_FORCEINLINE simdv_int32 loaduv(const int32_t* p) {
        return loadu4(p);
    }

    NUDGE_FORCEINLINE void storev(int32_t* p, simdv_int32 x) {
        store4(p, x);
    }

    NUDGE_FORCEINLINE void storeuv(int32_t* p, simdv_int32 x) {
        storeu4(p, x);
    }
}
#elif NUDGE_SIMDV_WIDTH == 256
typedef simd8_float simdv_float;
typedef simd8_int32 simdv_int32;

namespace simd_float {
    NUDGE_FORCEINLINE simdv_float zerov() {
        return zero8();
    }

    NUDGE_FORCEINLINE simdv_float makev(float x) {
        return make8(x);
    }

    NUDGE_FORCEINLINE simdv_float broadcast_loadv(const float* p) {
        return broadcast_load8(p);
    }

    NUDGE_FORCEINLINE simdv_float loadv(const float* p) {
        return load8(p);
    }

    NUDGE_FORCEINLINE simdv_float loaduv(const float* p) {
        return loadu8(p);
    }

    NUDGE_FORCEINLINE void storev(float* p, simdv_float x) {
        store8(p, x);
    }

    NUDGE_FORCEINLINE void storeuv(float* p, simdv_float x) {
        storeu8(p, x);
    }
}

namespace simd_int32 {
    NUDGE_FORCEINLINE simdv_int32 zerov() {
        return zero8();
    }

    NUDGE_FORCEINLINE simdv_int32 makev(int32_t x) {
        return make8(x);
    }

    NUDGE_FORCEINLINE simdv_int32 loadv(const int32_t* p) {
        return load8(p);
    }

    NUDGE_FORCEINLINE simdv_int32 loaduv(const int32_t* p) {
        return loadu8(p);
    }

    NUDGE_FORCEINLINE void storev(int32_t* p, simdv_int32 x) {
        store8(p, x);
    }

    NUDGE_FORCEINLINE void storeuv(int32_t* p, simdv_int32 x) {
        storeu8(p, x);
    }
}
#endif

namespace simd_aos {
    NUDGE_FORCEINLINE simd4_float dot(simd4_float a, simd4_float b) {
        simd4_float c = a*b;
        return simd128::shuffle32<0,0,0,0>(c) + simd128::shuffle32<1,1,1,1>(c) + simd128::shuffle32<2,2,2,2>(c);
    }

    NUDGE_FORCEINLINE simd4_float cross(simd4_float a, simd4_float b) {
        simd4_float c = simd128::shuffle32<1,2,0,0>(a) * simd128::shuffle32<2,0,1,0>(b);
        simd4_float d = simd128::shuffle32<2,0,1,0>(a) * simd128::shuffle32<1,2,0,0>(b);
        return c - d;
    }
}

namespace simd_soa {
    NUDGE_FORCEINLINE void cross(simd4_float ax, simd4_float ay, simd4_float az, simd4_float bx, simd4_float by, simd4_float bz, simd4_float& rx, simd4_float& ry, simd4_float& rz) {
        rx = ay*bz - az*by;
        ry = az*bx - ax*bz;
        rz = ax*by - ay*bx;
    }

    NUDGE_FORCEINLINE void normalize(simd4_float& x, simd4_float& y, simd4_float& z) {
        simd4_float f = simd_float::rsqrt(x*x + y*y + z*z);
        x *= f;
        y *= f;
        z *= f;
    }

#if NUDGE_SIMDV_WIDTH >= 256
    NUDGE_FORCEINLINE void cross(simd8_float ax, simd8_float ay, simd8_float az, simd8_float bx, simd8_float by, simd8_float bz, simd8_float& rx, simd8_float& ry, simd8_float& rz) {
        rx = ay*bz - az*by;
        ry = az*bx - ax*bz;
        rz = ax*by - ay*bx;
    }

    NUDGE_FORCEINLINE void normalize(simd8_float& x, simd8_float& y, simd8_float& z) {
        simd8_float f = simd_float::rsqrt(x*x + y*y + z*z);
        x *= f;
        y *= f;
        z *= f;
    }
#endif
}

#ifdef NUDGE_USE_ANONYMOUS_NAMESPACE
namespace {
#endif
    struct float3 {
        float x, y, z;
    };

    struct float3x3 {
        float3 c0, c1, c2;
    };

    struct Rotation {
        float3 v;
        float s;
    };

    struct AABB {
        float3 min;
        float unused0;
        float3 max;
        float unused1;
    };

    struct AABBV {
        float min_x[simdv_width32];
        float max_x[simdv_width32];
        float min_y[simdv_width32];
        float max_y[simdv_width32];
        float min_z[simdv_width32];
        float max_z[simdv_width32];
    };

    struct ContactSlotV {
        uint32_t indices[simdv_width32];
    };

    struct ContactPairV {
        uint32_t ab[simdv_width32];
    };

    struct ContactConstraintV {
        uint16_t a[simdv_width32];
        uint16_t b[simdv_width32];

        float pa_z[simdv_width32];
        float pa_x[simdv_width32];
        float pa_y[simdv_width32];

        float pb_z[simdv_width32];
        float pb_x[simdv_width32];
        float pb_y[simdv_width32];

        float n_x[simdv_width32];
        float u_x[simdv_width32];
        float v_x[simdv_width32];

        float n_y[simdv_width32];
        float u_y[simdv_width32];
        float v_y[simdv_width32];

        float n_z[simdv_width32];
        float u_z[simdv_width32];
        float v_z[simdv_width32];

        float bias[simdv_width32];
        float friction[simdv_width32];
        float normal_velocity_to_normal_impulse[simdv_width32];

        float friction_coefficient_x[simdv_width32];
        float friction_coefficient_y[simdv_width32];
        float friction_coefficient_z[simdv_width32];

        float na_x[simdv_width32];
        float na_y[simdv_width32];
        float na_z[simdv_width32];

        float nb_x[simdv_width32];
        float nb_y[simdv_width32];
        float nb_z[simdv_width32];

        float ua_x[simdv_width32];
        float ua_y[simdv_width32];
        float ua_z[simdv_width32];

        float va_x[simdv_width32];
        float va_y[simdv_width32];
        float va_z[simdv_width32];

        float ub_x[simdv_width32];
        float ub_y[simdv_width32];
        float ub_z[simdv_width32];

        float vb_x[simdv_width32];
        float vb_y[simdv_width32];
        float vb_z[simdv_width32];
    };

    struct ContactConstraintStateV {
        float applied_normal_impulse[simdv_width32];
        float applied_friction_impulse_x[simdv_width32];
        float applied_friction_impulse_y[simdv_width32];
    };

    struct InertiaTransform {
        float xx;
        float yy;
        float zz;
        float unused0;
        float xy;
        float xz;
        float yz;
        float unused1;
    };
#ifdef NUDGE_USE_ANONYMOUS_NAMESPACE
}
#endif

#ifdef __MSC_VER//_WIN32
static inline unsigned first_set_bit(unsigned x) {
    unsigned long r = 0;
    _BitScanForward(&r, x);
    return r;
}
#else
static inline unsigned first_set_bit(unsigned x) {
    return __builtin_ctz(x);
}
#endif


static inline void* align(Arena* arena, uintptr_t alignment) {
    uintptr_t data = (uintptr_t)arena->data;
    uintptr_t end = data + arena->size;
    uintptr_t mask = alignment-1;

    data = (data + mask) & ~mask;

    arena->data = (void*)data;
    arena->size = end - data;

    assert((intptr_t)arena->size >= 0); // Out of memory.

    return arena->data;
}

static inline void* allocate(Arena* arena, uintptr_t size) {
    void* data = arena->data;
    arena->data = (void*)((uintptr_t)data + size);
    arena->size -= size;

    assert((intptr_t)arena->size >= 0); // Out of memory.

    return data;
}

static inline void* allocate(Arena* arena, uintptr_t size, uintptr_t alignment) {
    align(arena, alignment);

    void* data = arena->data;
    arena->data = (void*)((uintptr_t)data + size);
    arena->size -= size;

    assert((intptr_t)arena->size >= 0); // Out of memory.   [this probably happens when initial arena size is too small (to many live contacts per frame). aligned_realloc does not exist on most systems, and probably would not work]

    return data;
}

template<class T>
static inline T* allocate_struct(Arena* arena, uintptr_t alignment) {
    return static_cast<T*>(allocate(arena, sizeof(T), alignment));
}

template<class T>
static inline T* allocate_array(Arena* arena, uintptr_t count, uintptr_t alignment) {
    return static_cast<T*>(allocate(arena, sizeof(T)*count, alignment));
}

static inline void* reserve(Arena* arena, uintptr_t size, uintptr_t alignment) {
    align(arena, alignment);
    assert(size <= arena->size); // Cannot reserve this amount.
    return arena->data;
}

static inline void commit(Arena* arena, uintptr_t size) {
    allocate(arena, size);
}

template<class T>
static inline T* reserve_array(Arena* arena, uintptr_t count, uintptr_t alignment) {
    return static_cast<T*>(reserve(arena, sizeof(T)*count, alignment));
}

template<class T>
static inline void commit_array(Arena* arena, uintptr_t count) {
    commit(arena, sizeof(T)*count);
}

static inline Rotation make_rotation(const float q[4]) {
    Rotation r = { { q[0], q[1], q[2] }, q[3] };
    return r;
}

static inline float3 make_float3(const float x[3]) {
    float3 r = { x[0], x[1], x[2] };
    return r;
}

static inline float3 make_float3(float x, float y, float z) {
    float3 r = { x, y, z };
    return r;
}

static inline float3 make_float3(float x) {
    float3 r = { x, x, x };
    return r;
}

static inline float3 operator + (float3 a, float3 b) {
    float3 r = { a.x + b.x, a.y + b.y, a.z + b.z };
    return r;
}

static inline float3 operator - (float3 a, float3 b) {
    float3 r = { a.x - b.x, a.y - b.y, a.z - b.z };
    return r;
}

static inline float3 operator * (float a, float3 b) {
    float3 r = { a * b.x, a * b.y, a * b.z };
    return r;
}

static inline float3 operator * (float3 a, float b) {
    float3 r = { a.x * b, a.y * b, a.z * b };
    return r;
}

static inline float3& operator *= (float3& a, float b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
    return a;
}

static inline float dot(float3 a, float3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

static inline float length2(float3 a) {
    return dot(a, a);
}

static inline float3 cross(float3 a, float3 b) {
    float3 v = { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
    return v;
}

static inline float3 operator * (Rotation lhs, float3 rhs) {
    float3 t = 2.0f * cross(lhs.v, rhs);
    return rhs + lhs.s * t + cross(lhs.v, t);
}

static inline Rotation operator * (Rotation lhs, Rotation rhs) {
    float3 v = rhs.v*lhs.s + lhs.v*rhs.s + cross(lhs.v, rhs.v);
    Rotation r = { v, lhs.s*rhs.s - dot(lhs.v, rhs.v) };
    return r;
}

static inline Rotation normalize(Rotation r) {
    float f = 1.0f / sqrtf(r.s*r.s + r.v.x*r.v.x + r.v.y*r.v.y + r.v.z*r.v.z);
    r.v *= f;
    r.s *= f;
    return r;
}

static inline Rotation inverse(Rotation r) {
    r.v.x = -r.v.x;
    r.v.y = -r.v.y;
    r.v.z = -r.v.z;
    return r;
}

static inline float3x3 matrix(Rotation q) {
    float kx = q.v.x + q.v.x;
    float ky = q.v.y + q.v.y;
    float kz = q.v.z + q.v.z;

    float xx = kx*q.v.x;
    float yy = ky*q.v.y;
    float zz = kz*q.v.z;
    float xy = kx*q.v.y;
    float xz = kx*q.v.z;
    float yz = ky*q.v.z;
    float sx = kx*q.s;
    float sy = ky*q.s;
    float sz = kz*q.s;

    float3x3 m = {
        { 1.0f - yy - zz, xy + sz, xz - sy },
        { xy - sz, 1.0f - xx - zz, yz + sx },
        { xz + sy, yz - sx, 1.0f - xx - yy },
    };
    return m;
}

static inline Transform operator * (Transform lhs, Transform rhs) {
    float3 p = make_rotation(lhs.rotation) * make_float3(rhs.position) + make_float3(lhs.position);
    Rotation q = make_rotation(lhs.rotation) * make_rotation(rhs.rotation);

    Transform r = {
        {{ p.x, p.y, p.z }},
        {rhs.body},
        {{ q.v.x, q.v.y, q.v.z, q.s }},
    };
    return r;
}

// old function declarations now hidden in the implementation
void simulate(context_t* c,float timeStep, unsigned numSubSteps, unsigned numIterations);
void collide(context_t* c, BodyConnections body_connections);
ContactImpulseData* read_cached_impulses(ContactCache contact_cache, ContactData contacts, Arena* memory);
void write_cached_impulses(ContactCache* contact_cache, ContactData contacts, ContactImpulseData* contact_impulses);
ContactConstraintData* setup_contact_constraints(context_t* c,/*ActiveBodies active_bodies, ContactData contacts, BodyData bodies,*/ ContactImpulseData* contact_impulses, Arena* memory);
void apply_impulses(ContactConstraintData* data, BodyData bodies);
void update_cached_impulses(ContactConstraintData* data, ContactImpulseData* contact_impulses);
void advance(context_t* c, float time_step);

// new stuff -------------------------------------------------------------------------
Transform TransformMul(Transform T0,Transform T1)   {return T0*T1;}

#ifdef NUDGE_SIMDE_USE_CUSTOM_MM_MALLOC
#if (defined(__EMSCRIPTEN__) || (defined(NUDGE_USE_SIMDE) && defined(SIMDE_NO_NATIVE)))
static inline void* _mm_malloc (size_t size, size_t alignment)	{
#	ifdef _WIN32
    return ::_aligned_malloc(size, alignment);
#	else
    void *ptr;
    if (alignment == 1) return ::malloc (size);
    if (alignment == 2 || (sizeof (void *) == 8 && alignment == 4)) alignment = sizeof (void *);
    if (::posix_memalign (&ptr, alignment, size) == 0) return ptr;
    else return NULL;
#	endif
}
static inline void _mm_free (void * ptr) {
#	if defined(WIN32)
    ::_aligned_free(ptr);
#	else
    ::free (ptr);
#	endif
}
#endif
#endif // NUDGE_SIMDE_USE_CUSTOM_MM_MALLOC

void* malloc(size_t size)   {return _mm_malloc(size,64);}
void free(void* ptr)    {_mm_free(ptr);}

inline float* nm_QuatFromMat3Or4(float* __restrict result4,const float* __restrict m16,int num_m16_cols=4)  {
    // this code is glm based
    float* q=result4;const float* m=m16;
    float *qx=&q[0],*qy=&q[1],*qz=&q[2],*qw=&q[3];const int bc2=num_m16_cols,bc3=num_m16_cols*2;
    const float c00=m[0],c01=m[1],c02=m[2], c10=m[bc2],c11=m[bc2+1],c12=m[bc2+2], c20=m[bc3],c21=m[bc3+1],c22=m[bc3+2];

    float fourXSquaredMinus1 = c00 - c11 - c22, fourYSquaredMinus1 = c11 - c00 - c22;
    float fourZSquaredMinus1 = c22 - c00 - c11, fourWSquaredMinus1 = c00 + c11 + c22;
    float biggestVal,mult,fourBiggestSquaredMinus1 = fourWSquaredMinus1;
    int biggestIndex = 0;

    if(fourXSquaredMinus1 > fourBiggestSquaredMinus1)   {fourBiggestSquaredMinus1 = fourXSquaredMinus1;biggestIndex = 1;}
    if(fourYSquaredMinus1 > fourBiggestSquaredMinus1)   {fourBiggestSquaredMinus1 = fourYSquaredMinus1;biggestIndex = 2;}
    if(fourZSquaredMinus1 > fourBiggestSquaredMinus1)   {fourBiggestSquaredMinus1 = fourZSquaredMinus1;biggestIndex = 3;}

    biggestVal = sqrtf(fourBiggestSquaredMinus1 + (float)1) * (float)0.5;
    mult = (float)0.25 / biggestVal;

    switch  (biggestIndex)    {
    case 0:
        *qw = biggestVal; *qx = (c12 - c21) * mult; *qy = (c20 - c02) * mult; *qz = (c01 - c10) * mult;
        break;
    case 1:
        *qw = (c12 - c21) * mult; *qx = biggestVal; *qy = (c01 + c10) * mult; *qz = (c20 + c02) * mult;
        break;
    case 2:
        *qw = (c20 - c02) * mult; *qx = (c01 + c10) * mult; *qy = biggestVal; *qz = (c12 + c21) * mult;
        break;
    case 3:
        *qw = (c01 - c10) * mult; *qx = (c20 + c02) * mult; *qy = (c12 + c21) * mult; *qz = biggestVal;
        break;

    default:					// Silence a -Wswitch-default warning in GCC. Should never actually get here. Assert is just for sanity.
        //NM_ASSER(1);
        *qx=*qy=*qz=(float)0;*qw=(float)1;
        break;
    }
    return result4;
}
float* nm_QuatFromMat4(float* __restrict result4,const float* __restrict m16)  {return nm_QuatFromMat3Or4(result4,m16,4);}
float* nm_QuatFromMat3(float* __restrict result4,const float* __restrict m9)  {return nm_QuatFromMat3Or4(result4,m9,3);}

inline float* nm_Mat3Or4SetRotationFromQuat(float* __restrict result16,const float* __restrict q4,int num_res_cols=4)    {
    // this code is glm based
    const float one =(float)1,two=(float)2;
    float* m=result16;const float* q=q4;
    const float qx=q[0],qy=q[1],qz=q[2],qw=q[3];const int bc2=num_res_cols,bc3=num_res_cols*2;
    float *c00=&m[0],*c01=&m[1],*c02=&m[2], *c10=&m[bc2],*c11=&m[bc2+1],*c12=&m[bc2+2], *c20=&m[bc3],*c21=&m[bc3+1],*c22=&m[bc3+2];

    float qxx = (qx * qx), qyy = (qy * qy), qzz = (qz * qz);
    float qxz = (qx * qz), qxy = (qx * qy), qyz = (qy * qz);
    float qwx = (qw * qx), qwy = (qw * qy), qwz = (qw * qz);

    *c00 = one - two * (qyy +  qzz); *c01 = two * (qxy + qwz); *c02 = two * (qxz - qwy);
    *c10 = two * (qxy - qwz); *c11 = one - two * (qxx +  qzz); *c12 = two * (qyz + qwx);
    *c20 = two * (qxz + qwy); *c21 = two * (qyz - qwx); *c22 = one - two * (qxx +  qyy);

    return result16;
}
float* nm_Mat4SetRotationFromQuat(float* __restrict result16,const float* __restrict q4)    {return nm_Mat3Or4SetRotationFromQuat(result16,q4,4);}
float* nm_Mat3FromQuat(float* __restrict result9,const float* __restrict q4)    {return nm_Mat3Or4SetRotationFromQuat(result9,q4,3);}

void nm_QuatGetAngularVelocity(float* __restrict angVel3,const float* newQuat4,const float* oldQuat4,float halfTimeStep)   {
    // assert: this works for unit length quaternions only
    // oldQuat4 and newQuat4 must be 'close' for this to work (small halfTimeStep)
    const float a[4] = {newQuat4[0]-oldQuat4[0],newQuat4[1]-oldQuat4[1],newQuat4[2]-oldQuat4[2],newQuat4[3]-oldQuat4[3]}; // deltaQ
    const float b[4] = {-oldQuat4[0],-oldQuat4[1],-oldQuat4[2],oldQuat4[3]};  // invOldQ
    const float invHalfTimeStep = halfTimeStep!=(float)0 ? (float)1/halfTimeStep : (float)0;
    //assert(halfTimeStep!=0);
    angVel3[0] = (a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1])*invHalfTimeStep;  // x
    angVel3[1] = (a[3] * b[1] + a[1] * b[3] + a[2] * b[0] - a[0] * b[2])*invHalfTimeStep;  // y
    angVel3[2] = (a[3] * b[2] + a[2] * b[3] + a[0] * b[1] - a[1] * b[0])*invHalfTimeStep;  // z

    //nm_QuatDifferentiateAngularVelocityApprox(angVel3,newQuat4,oldQuat4,halfTimeStep*2.0);   // is this better?
}
float* nm_QuatMul(float* /*__restrict*/ qOut4,const float* /*__restrict*/ a4,const float* /*__restrict*/ b4)  {
// we should activate simd, but maybe this same function is already present somewhere in the nudge code. TODO: fetch it!
#	if (defined(NM_USE_SIMD) && defined(__SSE__))
/*#	ifndef NM_ALIGN_STRUCTS // hope all the calls are used with aligned data... (not sure!)
#		define NM_MM_LOAD_PS(X) _mm_loadu_ps(X)
#		define NM_MM256_LOAD_PD(X) _mm256_loadu_pd(X)
#		define NM_MM_STORE_PS(X,Y) _mm_storeu_ps(X,Y)
#		define NM_MM256_STORE_PD(X,Y) _mm256_storeu_pd(X,Y)
#	else //NM_ALIGN_STRUCTS*/
#		define NM_MM_LOAD_PS(X) _mm_load_ps(X)
#		define NM_MM256_LOAD_PD(X) _mm256_load_pd(X)
#		define NM_MM_STORE_PS(X,Y) _mm_store_ps(X,Y)
#		define NM_MM256_STORE_PD(X,Y) _mm256_store_pd(X,Y)
//#	endif //NM_ALIGN_STRUCTS
    __m128 xyzw = NM_MM_LOAD_PS(a4);
    __m128 abcd = NM_MM_LOAD_PS(b4);

    __m128 wzyx = _mm_shuffle_ps(xyzw, xyzw, _MM_SHUFFLE(0,1,2,3));
    __m128 baba = _mm_shuffle_ps(abcd, abcd, _MM_SHUFFLE(0,1,0,1));
    __m128 dcdc = _mm_shuffle_ps(abcd, abcd, _MM_SHUFFLE(2,3,2,3));

    /* variable names below are for parts of componens of result (X,Y,Z,W) */
    /* nX stands for -X and similarly for the other components             */

    /* znxwy  = (xb - ya, zb - wa, wd - zc, yd - xc) */
    __m128 ZnXWY = _mm_hsub_ps(_mm_mul_ps(xyzw, baba), _mm_mul_ps(wzyx, dcdc));

    /* xzynw  = (xd + yc, zd + wc, wb + za, yb + xa) */
    __m128 XZYnW = _mm_hadd_ps(_mm_mul_ps(xyzw, dcdc), _mm_mul_ps(wzyx, baba));

    /* _mm_shuffle_ps(XZYnW, ZnXWY, _MM_SHUFFLE(3,2,1,0)) */
    /*      = (xd + yc, zd + wc, wd - zc, yd - xc)        */
    /* _mm_shuffle_ps(ZnXWY, XZYnW, _MM_SHUFFLE(2,3,0,1)) */
    /*      = (zb - wa, xb - ya, yb + xa, wb + za)        */

    /* _mm_addsub_ps adds elements 1 and 3 and subtracts elements 0 and 2, so we get: */
    /* _mm_addsub_ps(*, *) = (xd+yc-zb+wa, xb-ya+zd+wc, wd-zc+yb+xa, yd-xc+wb+za)     */

    __m128 XZWY = _mm_addsub_ps(_mm_shuffle_ps(XZYnW, ZnXWY, _MM_SHUFFLE(3,2,1,0)),
                                _mm_shuffle_ps(ZnXWY, XZYnW, _MM_SHUFFLE(2,3,0,1)));

    /* now we only need to shuffle the components in place and return the result      */
    NM_MM_STORE_PS(qOut4,_mm_shuffle_ps(XZWY, XZWY, _MM_SHUFFLE(2,1,3,0)));
#   else //NM_USE_SIMD
    /* reference implementation */
    const float x = a4[0],y = a4[1],z = a4[2], w = a4[3];
    const float a = b4[0],b = b4[1],c = b4[2], d = b4[3];
    qOut4[0] =  x*d + y*c - z*b + w*a;
    qOut4[1] = -x*c + y*d + z*a + w*b;
    qOut4[2] =  x*b - y*a + z*d + w*c;
    qOut4[3] = -x*a - y*b - z*c + w*d;
#   endif //NM_USE_SIMD
    return qOut4;
    /*float* q=qOut4;const float *a=a4,*b=b4;
    q[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    q[1] = a[3] * b[1] + a[1] * b[3] + a[2] * b[0] - a[0] * b[2];
    q[2] = a[3] * b[2] + a[2] * b[3] + a[0] * b[1] - a[1] * b[0];
    q[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    return qOut4;*/
}
void nm_QuatAdvance(float* __restrict qOut4,const float* __restrict q4,const float* __restrict angVel3,float halfTimeStep)    {
    // assert: this works for unit length quaternions only
    // advancement must be small for this to work (small halfTimeStep)
    float deltaQ[4] = {angVel3[0],angVel3[1],angVel3[2],(float)0};int i;
    nm_QuatMul(deltaQ,deltaQ,q4);
    for (i=0;i<4;i++) qOut4[i] = q4[i]+deltaQ[i]*halfTimeStep;
    nm_QuatNormalize(qOut4);

    //nm_QuatIntegrateAngularVelocityApprox(qOut4,q4,angVel3,halfTimeStep*2.0);   // is this better?
}
float nm_Vec3Dot(const float* __restrict a3,const float* __restrict b3) {return a3[0]*b3[0]+a3[1]*b3[1]+a3[2]*b3[2];}
float* nm_Vec3Cross(float* __restrict vOut3,const float* __restrict a3,const float* __restrict b3) {
    vOut3[0] =	a3[1] * b3[2] - a3[2] * b3[1];
    vOut3[1] =	a3[2] * b3[0] - a3[0] * b3[2];
    vOut3[2] =	a3[0] * b3[1] - a3[1] * b3[0];
    return vOut3;
}
#ifndef NM_EPSILON
#   define NM_EPSILON (0.00000000001f)
#endif
float nm_Vec3Normalize(float* __restrict v3) {
    float len = v3[0]*v3[0]+v3[1]*v3[1]+v3[2]*v3[2];int i;
    if (len>NM_EPSILON) {len = sqrtf(len);for (i=0;i<3;i++) v3[i]/=len;}
    else {len=v3[0]=v3[2]=(float)0;v3[1]=(float)1;}
    return len;
}
float nm_Vec3Normalized(float* __restrict v3Out,const float* __restrict v3) {
    float len = v3[0]*v3[0]+v3[1]*v3[1]+v3[2]*v3[2];int i;
    if (len>NM_EPSILON) {len = sqrtf(len);for (i=0;i<3;i++) v3Out[i]=v3[i]/len;}
    else {len=v3Out[0]=v3Out[2]=(float)0;v3Out[1]=(float)1;}
    return len;
}
void nm_QuatNormalize(float* __restrict q4) {const float len=sqrtf(q4[0]*q4[0]+q4[1]*q4[1]+q4[2]*q4[2]+q4[3]*q4[3]);if (len>0) {q4[0]/=len;q4[1]/=len;q4[2]/=len;q4[3]/=len;} else {q4[0]=q4[1]=q4[2]=q4[3]=(float)0;}}
float* nm_QuatSlerpEps(float* __restrict result4,const float* __restrict a4,const float* __restrict b4,float slerpTime_In_0_1,int normalizeResult4AfterLerp/*=1*/,float eps/*= NM_SLERP_EPSILON*/)  {
    // Adapted from OgraMath (www.ogre3d.org AFAIR)

    //const int normalizeQOutAfterLerp = 1;            // When using Lerp instead of Slerp qOut should be normalized. However some users prefer setting eps small enough so that they can leave the Lerp as it is.
    //const float eps = NM_SLERP_EPSILON;              // In [0 = 100% Slerp,1 = 100% Lerp] Faster but less precise with bigger epsilon (Lerp is used instead of Slerp more often). Users should tune it to achieve a performance boost.
    const float one = (float)1;
    const float *qStart=a4;
    float qEnd[4]={b4[0],b4[1],b4[2],b4[3]};
    float* qOut=result4;

    float fCos = qStart[0] * qEnd[0] + qStart[1] * qEnd[1] + qStart[2] * qEnd[2] + qStart[3] * qEnd[3];

    // Do we need to invert rotation?
    if(fCos < 0)	//Originally it was if(fCos < (float)0 && shortestPath)
        {fCos = -fCos;qEnd[0] = -qEnd[0];qEnd[1] = -qEnd[1];qEnd[2] = -qEnd[2];qEnd[3] = -qEnd[3];}

    if( fCos < one - eps)	// Originally if was "Ogre::Math::Abs(fCos)" instead of "fCos", but we know fCos>0, because we have hard coded shortestPath=true
    {
        // Standard case (slerp)
#       ifndef NM_QUAT_SLERP_USE_ACOS_AND_SIN_INSTEAD_OF_ATAN2_AND_SQRT
        // Ogre::Quaternion uses this branch by default
        float fSin = sqrtf(one - fCos*fCos);
        float fAngle = atan2f(fSin, fCos);
#       else //NM_QUAT_SLERP_USE_ACOS_AND_SIN_INSTEAD_OF_ATAN2_AND_SQRT
        // Possible replacement of the two lines above
        // (it's hard to tell if they're faster, but my instinct tells me I should trust atan2 better than acos (geometry geeks needed here...)):
        // But probably sin(...) is faster than (sqrt + 1 subtraction and mult)
        float fAngle = acosf(fCos);
        float fSin = sinf(fAngle);
#       endif //NM_QUAT_SLERP_USE_ACOS_AND_SIN_INSTEAD_OF_ATAN2_AND_SQRT

        const float fInvSin = one / fSin;
        const float fCoeff0 = sinf((one - slerpTime_In_0_1) * fAngle) * fInvSin;
        const float fCoeff1 = sinf(slerpTime_In_0_1 * fAngle) * fInvSin;

        //qOut =  fCoeff0 * qStart + fCoeff1 * qEnd; //Avoided for maximum portability and conversion of the code
        qOut[0] = (fCoeff0 * qStart[0] + fCoeff1 * qEnd[0]);
        qOut[1] = (fCoeff0 * qStart[1] + fCoeff1 * qEnd[1]);
        qOut[2] = (fCoeff0 * qStart[2] + fCoeff1 * qEnd[2]);
        qOut[3] = (fCoeff0 * qStart[3] + fCoeff1 * qEnd[3]);
    } else
    {
        // There are two situations:
        // 1. "qStart" and "qEnd" are very close (fCos ~= +1), so we can do a linear
        //    interpolation safely.
        // 2. "qStart" and "qEnd" are almost inverse of each other (fCos ~= -1), there
        //    are an infinite number of possibilities interpolation. but we haven't
        //    have method to fix this case, so just use linear interpolation here.
        // IMPORTANT: CASE 2 can't happen anymore because we have hardcoded "shortestPath = true" and now fCos > 0

        const float fCoeff0 = one - slerpTime_In_0_1;
        const float fCoeff1 = slerpTime_In_0_1;

        //qOut =  fCoeff0 * qStart + fCoeff1 * qEnd; //Avoided for maximum portability and conversion of the code
        qOut[0] = (fCoeff0 * qStart[0] + fCoeff1 * qEnd[0]);
        qOut[1] = (fCoeff0 * qStart[1] + fCoeff1 * qEnd[1]);
        qOut[2] = (fCoeff0 * qStart[2] + fCoeff1 * qEnd[2]);
        qOut[3] = (fCoeff0 * qStart[3] + fCoeff1 * qEnd[3]);
        if (normalizeResult4AfterLerp)  nm_QuatNormalize(qOut);
    }

    return qOut;
}
#ifndef NM_SLERP_EPSILON
#   define NM_SLERP_EPSILON (0.0001f)
#endif //NM_SLERP_EPSILON
float* nm_QuatSlerp(float* __restrict result4,const float* __restrict a4,const float* __restrict b4,float slerpTime_In_0_1,int normalizeResult4AfterLerp/*=1*/)  {return nm_QuatSlerpEps(result4,a4,b4,slerpTime_In_0_1,normalizeResult4AfterLerp,NM_SLERP_EPSILON);}
float* nm_QuatFromAngleAxis(float* __restrict qOut4,float rfAngle,float rkAxisX,float rkAxisY,float rkAxisZ)   {
    // assert:  axis[] is unit length
    //
    // The quaternion representing the rotation is
    //   q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k)
    float fSin,fCos;//sincosf((float)(0.5)*rfAngle,&fSin,&fCos);
    const float hangle=(float)(0.5)*rfAngle;fSin=sinf(hangle),fCos=cosf(hangle);
    qOut4[3]=fCos; qOut4[0]=fSin*rkAxisX; qOut4[1]=fSin*rkAxisY; qOut4[2]=fSin*rkAxisZ;
    return qOut4;
}
void nm_QuatToAngleAxis(const float* __restrict q4,float* __restrict rfAngleOut1,float* __restrict rkAxisOut3)  {
    const float* q=q4;
    // These both seem to work.
    // Implementation 1
            // The quaternion representing the rotation is
            //   q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k)

            float fSqrLength = q[0]*q[0]+q[1]*q[1]+q[2]*q[2];
            if (fSqrLength > (float)0)  {
                float fInvLength;*rfAngleOut1 = (float)2*acosf(q[3]);fInvLength = (float)1/sqrtf(fSqrLength);
                rkAxisOut3[0] = q[0]*fInvLength;rkAxisOut3[1] = q[1]*fInvLength;rkAxisOut3[2] = q[2]*fInvLength;
            }
            else    {
                // angle is 0 (mod 2*pi), so any axis will do
                *rfAngleOut1 = rkAxisOut3[0] = rkAxisOut3[2] = (float)0;
                rkAxisOut3[1] = (float)1;
            }
    /*// Implementation 2
        // more based on the glm library code:
        float tmp1 = (float)1 - q[3]*q[3];
        *rfAngleOut1 = acosf(q[3]) * (float)2;
        if (tmp1 <= (float)0)   {rkAxisOut3[0]=rkAxisOut3[1]=(float)0;rkAxisOut3[2]=(float)1;}
        else    {
            float tmp2 = (float)1 / sqrtf(tmp1);
            rkAxisOut[0]=q[0]*tmp2; rkAxisOut[1]=q[1]*tmp2; rkAxisOut[2]=q[2]*tmp2;
        }*/
}
void Mat4WithoutScalingToTransform(Transform* Tout,const float* matrix16WithourScaling)  {
    if (matrix16WithourScaling)    {
        nm_QuatFromMat4(Tout->rotation,matrix16WithourScaling);
        memcpy(Tout->position,&matrix16WithourScaling[12],3*sizeof(float));
    }
    else *Tout = identity_transform;
}
void TransformToMat4(float* matrix16Out,const Transform* T)    {
    int i;
    nm_Mat4SetRotationFromQuat(matrix16Out,T->rotation);
    for (i=0;i<3;i++) matrix16Out[12+i] = T->position[i];
    matrix16Out[3]=matrix16Out[7]=matrix16Out[11]=0.f;matrix16Out[15]=1.f;
}
float* nm_QuatMulVec3(float* __restrict vOut3,const float* __restrict q4,const float* __restrict vIn3)   {
    float uv[3],uuv[3];int i;
    nudge::nm_Vec3Cross(uuv,q4,nm_Vec3Cross(uv,q4,vIn3));
    for (i=0;i<3;i++) vOut3[i] = vIn3[i] + ((uv[i] * q4[3]) + uuv[i]) * (float)2;
    return vOut3;
}
float* nm_QuatGetAxis(float* __restrict vOut3,const float* __restrict q4,float axisX,float axisY,float axisZ) {
    const float vIn[3]={axisX,axisY,axisZ};
    return nm_QuatMulVec3(vOut3,q4,vIn);
    /* Other stuff that we can do if input axis is {0,1,0}:
        // direct calculation
        //vOut3[0] = 2.f*(q4[0]*q4[2]+q4[3]*q4[1]);
        //vOut3[1] = 2.f*(q4[1]*q4[2]-q4[3]*q4[0]);
        //vOut3[2] = 1.f-2.f*(q4[0]*q4[0]+q4[1]*q4[1]);

        // or this
        float angle,axis[3];nm_QuatToAngleAxis(q4,&angle,axis);    // if we know that 'axis' is (0,1,0) in advance, we can ignore it
        //angle*=axis[1]; // axis[1] can be 1 or -1 AFAICS [useless if quat axis y is always axis[1]==1]
        vOut3[0]=sinf(angle),vOut3[1]=0.f,vOut3[2]=cosf(angle); // however I'm not too sure this is faster than the other methods...
    */
}
float* nm_QuatRotate(float* __restrict qInOut4,float angle,float axisX,float axisY,float axisZ) {
    float qa[4];nm_QuatFromAngleAxis(qa,angle,axisX,axisY,axisZ);
    return nm_QuatMul(qInOut4,qInOut4,qa);
}
inline float* nm_Mat4Mul_NoCheck(float* __restrict result16,const float* __restrict ml16,const float* __restrict mr16)   {
   int i,i4;float mri4plus0,mri4plus1,mri4plus2,mri4plus3;
   for(i = 0; i < 4; i++) {
       i4=4*i;mri4plus0=mr16[i4];mri4plus1=mr16[i4+1];mri4plus2=mr16[i4+2];mri4plus3=mr16[i4+3];
       result16[  i4] = ml16[0]*mri4plus0 + ml16[4]*mri4plus1 + ml16[ 8]*mri4plus2 + ml16[12]*mri4plus3;
       result16[1+i4] = ml16[1]*mri4plus0 + ml16[5]*mri4plus1 + ml16[ 9]*mri4plus2 + ml16[13]*mri4plus3;
       result16[2+i4] = ml16[2]*mri4plus0 + ml16[6]*mri4plus1 + ml16[10]*mri4plus2 + ml16[14]*mri4plus3;
       result16[3+i4] = ml16[3]*mri4plus0 + ml16[7]*mri4plus1 + ml16[11]*mri4plus2 + ml16[15]*mri4plus3;
   }
   return result16;
}
float* nm_Mat4Mul(float* result16,const float* ml16,const float* mr16) {
   if (result16==ml16) {float ML16[16];memcpy(ML16,ml16,16*sizeof(float));return nm_Mat4Mul_NoCheck(result16,ML16,mr16);}
   else if (result16==mr16) {float MR16[16];memcpy(MR16,mr16,16*sizeof(float));return nm_Mat4Mul_NoCheck(result16,ml16,MR16);}
   return nm_Mat4Mul_NoCheck(result16,ml16,mr16);
}

void TransformAssignToBody(context_t* c,unsigned body,Transform newT,float deltaTime,int16_t aux_body)   {
    assert(c && body<c->bodies.count);
    BodyFilter* filter = &c->bodies.filters[body];
    const FlagMask flags = filter->flags;
    Transform* T = &c->bodies.transforms[body];
    float* P = newT.position;float* Q = newT.rotation;
    // calculate velocities
    float* linvel = c->bodies.momentum[body].velocity;
    float* angvel = c->bodies.momentum[body].angular_velocity;
    if (deltaTime!=0.f) {
        nm_QuatGetAngularVelocity(angvel,Q,T->rotation,deltaTime*0.5f);
        if (aux_body>=0)    {
            assert((unsigned)aux_body!=T->body);
            assert((unsigned)aux_body<c->bodies.count);
            const float* auxLinVel = c->bodies.momentum[aux_body].velocity;
            const float* auxAngVel = c->bodies.momentum[aux_body].angular_velocity;
            const Transform* auxT = &c->bodies.transforms[aux_body];
            // add auxLinVel and auxAngVel to linvel and angvel
            const float delta_position[3] = {T->position[0] - auxT->position[0],T->position[1] - auxT->position[1],T->position[2] - auxT->position[2]};
            float deltaLinVel[3];nm_Vec3Cross(deltaLinVel,auxAngVel,delta_position);
            for (int l=0;l<3;l++) {
                linvel[l]=(P[l]-T->position[l])/deltaTime + auxLinVel[l] + deltaLinVel[l];
                angvel[l]+=auxAngVel[l];
            }
            if (flags&BF_IS_KINEMATIC) {
                // recalculate P and Q based on T, linvel and angvel
                for (int l=0;l<3;l++)   {P[l]=T->position[l]+linvel[l]*deltaTime;}
                nm_QuatAdvance(Q,T->rotation,angvel,deltaTime*0.5f);
                //nm_QuatNormalize(T->rotation);
            }
        }
        else {for (int l=0;l<3;l++) linvel[l]=(P[l]-T->position[l])/deltaTime;}
        if (flags&BF_IS_KINEMATIC) {
            // assign the new position and orientation
            memcpy(T->position,P,3*sizeof(float));
            memcpy(T->rotation,Q,4*sizeof(float));
        }
    }
    else    {
        // if aux_body is present this is not correct (but we'd need delta_aux_T to calculate it)
        memset(linvel,0,3*sizeof(float));
        memset(angvel,0,3*sizeof(float));
        // assign the new position and orientation
        memcpy(T->position,P,3*sizeof(float));
        memcpy(T->rotation,Q,4*sizeof(float));
    }
    if (flags&BF_IS_DYNAMIC)
        c->bodies.idle_counters[body]=0;   // this prevents sleeping (but does not improve things if body has a negative mass
}
void TransformAdvanceBodyFromVelocities(context_t* c,unsigned body,float deltaTime)    {
    // advance Transform based on lin and ang velocities
    assert(c && body<c->bodies.count);
    Transform* newT = &c->bodies.transforms[body];
    Transform oldT = *newT;
    float* linvel = c->bodies.momentum[body].velocity;
    float* angvel = c->bodies.momentum[body].angular_velocity;
    // advance newT based on oldT, linvel and angvel directly only in kinematic objects
    //const float mass_inverse = c->bodies.properties[body].mass_inverse;
    const uint32_t flags = c->bodies.filters[body].flags;
    if (flags&BF_IS_KINEMATIC)    {
        // advance newT based on oldT, linvel and angvel
        for (int l=0;l<3;l++)   {newT->position[l]=oldT.position[l]+linvel[l]*deltaTime;}
        nm_QuatAdvance(newT->rotation,oldT.rotation,angvel,deltaTime*0.5f);
    }
    else if (flags&BF_IS_DYNAMIC) c->bodies.idle_counters[body]=0;  // wake up dynamic object
}

Transform TransformSlerp(Transform T0,Transform T1,float time)  {
    Transform R;const float c0 = 1.f - time, c1 = time;R.body=T0.body;
    for (int l=0;l<3;l++) R.position[l]=c0*T0.position[l]+c1*T1.position[l];
    nm_QuatSlerp(R.rotation,T0.rotation,T1.rotation,time,1);
    return R;
}

void calculate_box_inertia(float result[3],float mass,float hsizex,float hsizey,float hsizez,const float comOffset[3])  {
    if (mass!=0.f)   {
        float k = mass/3.f;
        float kcx2 = k*hsizex*hsizex,   kcy2 = k*hsizey*hsizey,   kcz2 = k*hsizez*hsizez;
        result[0] = (kcy2+kcz2);    result[1] = (kcx2+kcz2);    result[2] = (kcx2+kcy2);
        if (comOffset) {
            result[0] += mass * (comOffset[1] * comOffset[1] + comOffset[2] * comOffset[2]);
            result[1] += mass * (comOffset[0] * comOffset[0] + comOffset[2] * comOffset[2]);
            result[2] += mass * (comOffset[0] * comOffset[0] + comOffset[1] * comOffset[1]);
        }
    }
    else memset(result, 0, 3*sizeof(float));
}
void calculate_box_inertia_inverse(float result[3],float mass,float hsizex,float hsizey,float hsizez,const float comOffset[3])  {
    if (mass!=0.f)   {
        calculate_box_inertia(result,mass,hsizex,hsizey,hsizez,comOffset);
        for (int i=0;i<3;i++) result[i] = 1.f/result[i];
    }
    else memset(result, 0, 3*sizeof(float));
}
void calculate_sphere_inertia(float result[3], float mass, float radius, const float comOffset[3], bool hollow)  {
    if (mass!=0.f)   {
        result[0] = result[1] = result[2] = (mass*radius*radius)/(hollow?1.5f:2.5f);
        if (comOffset) {
            result[0] += mass * (comOffset[1] * comOffset[1] + comOffset[2] * comOffset[2]);
            result[1] += mass * (comOffset[0] * comOffset[0] + comOffset[2] * comOffset[2]);
            result[2] += mass * (comOffset[0] * comOffset[0] + comOffset[1] * comOffset[1]);
        }
    }
    else memset(result, 0, 3*sizeof(float));
}
void calculate_sphere_inertia_inverse(float result[3], float mass, float radius, const float comOffset[3], bool hollow)  {
    if (mass!=0.f)   {
        calculate_sphere_inertia(result,mass,radius,comOffset,hollow);
        for (int i=0;i<3;i++) result[i] = 1.f/result[i];
    }
    else memset(result, 0, 3*sizeof(float));
}
void calculate_cylinder_inertia(float result[3], float mass, float radius, float halfHeight, AxisEnum upAxis, const float comOffset[3])  {
    if (mass!=0.f)   {
        float radius2 = radius*radius,  h2 = halfHeight*halfHeight*4.f;
        result[0] = result[1] = result[2] = mass*(3.f*radius2+h2)/12.f;
        result[upAxis] = mass*radius2/2.f;
        if (comOffset) {
            result[0] += mass * (comOffset[1] * comOffset[1] + comOffset[2] * comOffset[2]);
            result[1] += mass * (comOffset[0] * comOffset[0] + comOffset[2] * comOffset[2]);
            result[2] += mass * (comOffset[0] * comOffset[0] + comOffset[1] * comOffset[1]);
        }
    }
    else memset(result, 0, 3*sizeof(float));
}
void calculate_cylinder_inertia_inverse(float result[3],float mass,float radius,float halfHeight,AxisEnum upAxis,const float comOffset[3])  {
    if (mass!=0.f)   {
        calculate_cylinder_inertia(result,mass,radius,halfHeight,upAxis,comOffset);
        for (int i=0;i<3;i++) result[i] = 1.f/result[i];
    }
    else memset(result, 0, 3*sizeof(float));
}
void calculate_capsule_inertia(float result[3], float mass, float radius, float halfCylinderHeight, AxisEnum upAxis, const float comOffset[3])  {
    // based on https://xissburg.github.io/2022-10-01-calculating-moment-of-inertia-capsule/
    if (mass!=0.f)   {
        const float L = 2.f*halfCylinderHeight;
        float radius2 = radius*radius;
        const float Vcyl = M_PI*radius2*L, Vhem = 2.0f*M_PI*radius2*radius;const float Vtot = Vcyl + 2.f*Vhem;
        const float Mcyl = mass*Vcyl/Vtot, Mhem = mass*Vhem/Vtot;
        const float Icyl = Mcyl*(L*L+3.f*radius2)/12.f;
        const float Ihem = Mhem*radius2/2.5f;
        result[0] = result[1] = result[2] = Icyl + 2.f*Ihem + Mhem*(4.f*L+3*radius)*(4.f*L+3*radius)/32.f;
        result[upAxis] = (5.f*Mcyl+8.f*Mhem)*radius2*0.1f;
        if (comOffset) {
            result[0] += mass * (comOffset[1] * comOffset[1] + comOffset[2] * comOffset[2]);
            result[1] += mass * (comOffset[0] * comOffset[0] + comOffset[2] * comOffset[2]);
            result[2] += mass * (comOffset[0] * comOffset[0] + comOffset[1] * comOffset[1]);
        }
    }
    else memset(result, 0, 3*sizeof(float));
}
void calculate_capsule_inertia_inverse(float result[3],float mass,float radius,float halfCylinderHeight,AxisEnum upAxis,const float comOffset[3])  {
    if (mass!=0.f)   {
        calculate_capsule_inertia(result,mass,radius,halfCylinderHeight,upAxis,comOffset);
        for (int i=0;i<3;i++) result[i] = 1.f/result[i];
    }
    else memset(result, 0, 3*sizeof(float));
}
void calculate_hollow_cylinder_inertia(float result[3],float mass,float R,float r,float halfHeight,AxisEnum upAxis,const float comOffset[3])  {
    // R the total radius of the cylinder (including the border depth)
    // r must be: R-border_depth
    // => same as calculate_cylinder_inertia, but with: radius = sqrtf(R*R+r*r)
    if (mass!=0.f)   {
        float radius2 = R*R+r*r,  h2 = halfHeight*halfHeight*4.f;
        result[0] = result[1] = result[2] = mass*(3.f*radius2+h2)/12.f;
        result[upAxis] = mass*radius2/2.f;
        if (comOffset) {
            result[0] += mass * (comOffset[1] * comOffset[1] + comOffset[2] * comOffset[2]);
            result[1] += mass * (comOffset[0] * comOffset[0] + comOffset[2] * comOffset[2]);
            result[2] += mass * (comOffset[0] * comOffset[0] + comOffset[1] * comOffset[1]);
        }
    }
    else memset(result, 0, 3*sizeof(float));
}
void calculate_hollow_cylinder_inertia_inverse(float result[3],float mass,float R,float r,float halfHeight,AxisEnum upAxis,const float comOffset[3])  {
    if (mass!=0.f)   {
        calculate_hollow_cylinder_inertia(result,mass,R,r,halfHeight,upAxis,comOffset);
        for (int i=0;i<3;i++) result[i] = 1.f/result[i];
    }
    else memset(result, 0, 3*sizeof(float));
}
void calculate_torus_inertia(float result[3], float mass, float majorRadius, float minorRadius, AxisEnum upAxis, const float comOffset[3])  {
    // majorRadius radius of the torus  ( 0|-----------|--->R    | )
    // minorRadius radius of the circle ( 0|           |<---R    | )
    if (mass!=0.f)   {
        float a2 = minorRadius*minorRadius,  b2 = majorRadius*majorRadius;
        result[0] = result[1] = result[2] = 0.25f*mass*(4.f*b2+3*a2);
        result[upAxis] = 0.125f*mass*(5.f*a2+4.f*b2);
        if (comOffset) {
            result[0] += mass * (comOffset[1] * comOffset[1] + comOffset[2] * comOffset[2]);
            result[1] += mass * (comOffset[0] * comOffset[0] + comOffset[2] * comOffset[2]);
            result[2] += mass * (comOffset[0] * comOffset[0] + comOffset[1] * comOffset[1]);
        }
    }
    else memset(result, 0, 3*sizeof(float));
}
void calculate_torus_inertia_inverse(float result[3], float mass, float majorRadius, float minorRadius, AxisEnum upAxis, const float comOffset[3])  {
    if (mass!=0.f)   {
        calculate_torus_inertia(result,mass,majorRadius,minorRadius,upAxis,comOffset);
        for (int i=0;i<3;i++) result[i] = 1.f/result[i];
    }
    else memset(result, 0, 3*sizeof(float));
}
void calculate_cone_inertia(float result[3], float mass, float radius, float halfHeight, AxisEnum upAxis, const float comOffset[3])  {
    if (mass!=0.f)   {
        float radius2 = radius*radius,  h2 = halfHeight*halfHeight;
        result[0] = result[1] = result[2] = mass*(3.f*radius2+2.f*h2)/20.f;
        result[upAxis] = mass*3.f*radius2/10.f;
        if (comOffset) {
            result[0] += mass * (comOffset[1] * comOffset[1] + comOffset[2] * comOffset[2]);
            result[1] += mass * (comOffset[0] * comOffset[0] + comOffset[2] * comOffset[2]);
            result[2] += mass * (comOffset[0] * comOffset[0] + comOffset[1] * comOffset[1]);
        }
    }
    else memset(result, 0, 3*sizeof(float));
}
void calculate_cone_inertia_inverse(float result[3], float mass, float radius, float halfHeight, AxisEnum upAxis, const float comOffset[3])  {
    if (mass!=0.f)   {
        calculate_cone_inertia(result,mass,radius,halfHeight,upAxis,comOffset);
        for (int i=0;i<3;i++) result[i] = 1.f/result[i];
    }
    else memset(result, 0, 3*sizeof(float));
}


// main functions
void show_info()  {
    // Print information about instruction set.
#ifdef __AVX2__
    log("Using 8-wide AVX\n");
#else
    log("Using 4-wide SSE\n");
#if defined(__SSE4_1__) || defined(__AVX__)
    log("BLENDVPS: Enabled\n");
#else
    log("BLENDVPS: Disabled\n");
#endif
#endif

#ifdef __FMA__
    log("FMA: Enabled\n");
#else
    log("FMA: Disabled\n");
#endif

#	ifdef NUDGE_USE_SIMDE
    log("\nUSING SIMDE (simd everywhere).\n");
#		ifdef SIMDE_AVX2_NATIVE
        log("SIMDE: SIMDE_AVX2_NATIVE is defined.\n");
//#		error SIMDE_AVX2_NATIVE is defined.
#		endif
#		ifdef SIMDE_AVX_NATIVE
        log("SIMDE: SIMDE_AVX_NATIVE is defined.\n");
//#		error SIMDE_AVX_NATIVE is defined.
#		endif
#		ifdef SIMDE_SSE2_NATIVE
        log("SIMDE: SIMDE_SSE2_NATIVE is defined.\n");
//#		error SIMDE_SSE2_NATIVE is defined.
#		endif
#		ifdef SIMDE_SSE_NATIVE
        log("SIMDE: SIMDE_SSE_NATIVE is defined.\n");
//#		error SIMDE_SSE_NATIVE is defined.
#		endif
#		ifdef SIMDE_MMX_NATIVE
        log("SIMDE: SIMDE_MMX_NATIVE is defined.\n");
//#		error SIMDE_MMX_NATIVE is defined.
#		endif
#		ifdef SIMDE_NO_NATIVE
        log("SIMDE: SIMDE_NO_NATIVE is defined.\n");
#		endif
#	endif

    flush();
}

#ifndef NUDGE_DEFAULT_MAX_NUM_BOXES
#  define NUDGE_DEFAULT_MAX_NUM_BOXES 256 /*1024*4*/
#endif
#ifndef NUDGE_DEFAULT_MAX_NUM_SPHERES
#  define NUDGE_DEFAULT_MAX_NUM_SPHERES 256  /*1024*4 //512*6*/
#endif
#define NUDGE_START_SPHERE_TAG (16384)

#if ((NUDGE_DEFAULT_MAX_NUM_BOXES+NUDGE_DEFAULT_MAX_NUM_SPHERES)>8192)
#	error. It must be (NUDGE_DEFAULT_MAX_NUM_BOXES+NUDGE_DEFAULT_MAX_NUM_SPHERES)<=8192
#endif

#ifndef NUDGE_FRICTION_MODEL
#   define NUDGE_FRICTION_MODEL(F1,F2)  ((F1)*(F2)*0.5f)
#endif

#ifndef NUDGE_TOTAL_NUM_KINEMATIC_ANIMATION_KEY_FRAMES
#   define NUDGE_TOTAL_NUM_KINEMATIC_ANIMATION_KEY_FRAMES 40
#endif
#ifndef NUDGE_MAX_NUM_KINEMATIC_ANIMATIONS
#   define NUDGE_MAX_NUM_KINEMATIC_ANIMATIONS           10
#endif

void* _my_mm_realloc(void** pp,size_t new_capacity,size_t capacity,size_t item_size,size_t alignment) {
    assert(pp);
    unsigned char* p_old = (unsigned char*) *pp;assert(p_old);
    unsigned char* p = (unsigned char*) _mm_malloc(new_capacity*item_size, alignment);assert(p);
    memcpy(p,p_old,capacity*item_size);
    _mm_free(p_old);
    *pp = p;
    return p;
}
size_t _my_mm_realloc_grow(void** pp,size_t new_size,size_t capacity,size_t item_size,size_t alignment) {
    //returns the new_capacity
    if (new_size>=capacity) return capacity;
    const size_t new_capacity = capacity==0 ?  new_size : (new_size + capacity/2);
    void* p = _my_mm_realloc(pp,new_capacity,capacity,item_size,alignment);
    assert(*pp=p);
    return new_capacity;
}
void kinematic_data_reserve_key_frames(KinematicData* kd, size_t new_size) {
    const uint32_t capacity = kd->key_frame_capacity;
    if (capacity>=new_size) return;
    const size_t new_capacity = _my_mm_realloc_grow((void**) &kd->key_frame_transforms,new_size,capacity,sizeof(kd->key_frame_transforms[0]),64);
    const size_t tmp = _my_mm_realloc_grow((void**) &kd->key_frame_modes,new_size,capacity,sizeof(kd->key_frame_modes[0]),64);assert(tmp==new_capacity);
    kd->key_frame_capacity = new_capacity;
    for (uint32_t i=capacity;i<new_capacity;i++)    {
        kd->key_frame_transforms[i] = identity_transform;
        kd->key_frame_modes[i]=KinematicData::TM_NORMAL;
    }
    assert(new_capacity>new_size);
}
void kinematic_data_reserve_animations(KinematicData* kd, size_t new_size) {
    const uint32_t capacity = kd->animations_capacity;
    if (capacity>=new_size) return;
    const size_t new_capacity = _my_mm_realloc_grow((void**) &kd->animations,new_size,capacity,sizeof(kd->animations[0]),64);
    kd->animations_capacity = new_capacity;
    memset(&kd->animations[capacity],0,sizeof(KinematicData::Animation)*(new_capacity-capacity));
    for (uint32_t i=capacity;i<new_capacity;i++)   {
        KinematicData::Animation* m = &kd->animations[i];
        m->baseT = identity_transform;
        m->loop_mode = KinematicData::Animation::LM_LOOP_NORMAL;
        m->total_time = m->play_time = -1.f;
        m->speed = 1.f;
    }
    assert(new_capacity>new_size);
}



void restart_context(context_t* c) {
#   ifndef NUDGE_DEFAULT_GRAVITY
#      define NUDGE_DEFAULT_GRAVITY (-9.82f)
#   endif
#   ifndef NUDGE_DEFAULT_FRICTION
#      define NUDGE_DEFAULT_FRICTION (1.f)
#   endif
    assert(c && c->MAX_NUM_BODIES==c->MAX_NUM_SPHERES+c->MAX_NUM_BOXES && c->MAX_NUM_SPHERES>0 && c->MAX_NUM_BOXES>0);
    memset(c->bodies.idle_counters,0,sizeof(uint8_t)*c->MAX_NUM_BODIES);
    memset(c->colliders.boxes.data,0,sizeof(SphereCollider)*c->MAX_NUM_BOXES);
    memset(c->colliders.spheres.data,0,sizeof(SphereCollider)*c->MAX_NUM_SPHERES);
    memset(c->kinematic_data.animations,0,sizeof(KinematicData::Animation)*c->kinematic_data.animations_capacity);
    memset(c->kinematic_data.key_frame_transforms,0,sizeof(Transform)*c->kinematic_data.key_frame_capacity);
    memset(c->kinematic_data.key_frame_modes,0,sizeof(KinematicData::TimeMode)*c->kinematic_data.key_frame_capacity);
    c->active_bodies.count=0;
    c->bodies.count=0;
    c->colliders.boxes.count = 0;
    for (uint16_t i=0;i<c->MAX_NUM_BOXES;i++)    {c->colliders.boxes.tags[i] = i;}
    c->colliders.spheres.count = 0;
    for (uint16_t i=0;i<c->MAX_NUM_SPHERES;i++)    {c->colliders.spheres.tags[i] = NUDGE_START_SPHERE_TAG+i;}
    c->contact_cache.count=0;assert(c->contact_cache.capacity>0);
    c->contact_data.count=0;assert(c->contact_data.capacity>0);
    c->global_data.removed_bodies_count = c->global_data.finalized_removed_bodies_count = 0;assert(c->global_data.removed_bodies_capacity>0);
    c->kinematic_data.key_frame_count=0;
    c->kinematic_data.animations_count=0;
    for (uint32_t i=0;i<c->kinematic_data.key_frame_capacity;i++)    {
        c->kinematic_data.key_frame_transforms[i] = identity_transform;
        c->kinematic_data.key_frame_modes[i]=KinematicData::TM_NORMAL;
    }
    for (uint32_t i=0;i<c->kinematic_data.animations_capacity;i++)   {
        KinematicData::Animation* m = &c->kinematic_data.animations[i];
        m->baseT = identity_transform;
        m->loop_mode = KinematicData::Animation::LM_LOOP_NORMAL;
        m->total_time = m->play_time = -1.f;
        m->speed = 1.f;
    }
    for (unsigned i=0;i<c->MAX_NUM_BODIES;i++)    {
        BodyInfo* info = &c->bodies.infos[i];
        BodyProperties* property = &c->bodies.properties[i];
        BodyFilter* filter = &c->bodies.filters[i];
        BodyLayout* layout = &c->bodies.layouts[i];
        memset(info,0,sizeof(BodyInfo));
        memset(property,0,sizeof(BodyProperties));
        memset(filter,0,sizeof(BodyFilter));
        memset(layout,0,sizeof(BodyLayout));
        layout->first_box_index=layout->first_sphere_index=-1;
        property->gravity[1] = NUDGE_DEFAULT_GRAVITY;
        property->friction = NUDGE_DEFAULT_FRICTION;
        filter->flags=0;
        filter->collision_group=COLLISION_GROUP_DEFAULT;
        filter->collision_mask=COLLISION_GROUP_ALL;
#       if NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES>0
        for (int ab=0;ab<NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES;ab++) info->aux_bodies[ab] = -1;
#       endif // NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES
    }

    SimulationParams* sp = &c->simulation_params;
    sp->numsubsteps_overflow_in_last_frame=0;
    sp->num_substeps_in_last_frame=0;
    sp->remaining_time_in_seconds=0;
    sp->time_step_minus_remaining_time=0;

    //sp->num_total_substeps=sp->num_frames = 0;
}
void init_context_with(context_t* c,unsigned MAX_NUM_BOXES,unsigned MAX_NUM_SPHERES)   {
#if (!defined(__EMSCRIPTEN__) && !defined(NUDGE_USE_SIMDE))	// TODO: #if (defined(__EMSCRIPTEN__) || (defined(NUDGE_USE_SIMDE) && defined(SIMDE_SSE_NO_NATIVE)))
    // Disable denormals for performance.
#ifndef NUDGE_USE_SIMDE
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#else //NUDGE_USE_SIMDE
    SIMDE_MM_SET_FLUSH_ZERO_MODE(SIMDE_MM_FLUSH_ZERO_ON);
    SIMDE_MM_SET_DENORMALS_ZERO_MODE(SIMDE_MM_DENORMALS_ZERO_ON);
#endif //NUDGE_USE_SIMDE
#endif //__EMSCRIPTEN__

#ifndef NUDGE_ARENA_SIZE_MACRO
#   define NUDGE_ARENA_SIZE_MACRO(MAX_NUM_BODIES)  (512000+50*(MAX_NUM_BODIES)*(MAX_NUM_BODIES))//(64*1024*1024) //(48*NUDGE_MAX_NUM_BODIES*NUDGE_MAX_NUM_BODIES/4)//(64*1024*1024)
#endif

    assert(c);
    assert(c->MAX_NUM_BODIES==0);
    assert((MAX_NUM_BOXES+MAX_NUM_SPHERES<=8192) && "nudge has a upper limit on the number of colliders: (MAX_NUM_BOXES+MAX_NUM_SPHERES<=8192).");    // nudge has a upper limit on the number of colliders: (MAX_NUM_BOXES+MAX_NUM_SPHERES<=8192)
    //memset(&c,0,sizeof(context_t));

    *((unsigned*)&c->MAX_NUM_BOXES) = MAX_NUM_BOXES;
    *((unsigned*)&c->MAX_NUM_SPHERES) = MAX_NUM_SPHERES;
    *((unsigned*)&c->MAX_NUM_BODIES) = c->MAX_NUM_BOXES+c->MAX_NUM_SPHERES;

    const unsigned NUDGE_MAX_NUM_BODIES = c->MAX_NUM_BODIES;
    const unsigned NUDGE_MAX_NUM_BOXES = c->MAX_NUM_BOXES;
    const unsigned NUDGE_MAX_NUM_SPHERES = c->MAX_NUM_SPHERES;

    // Set valid simulation data
    struct SimulationParams* sp = &c->simulation_params;
    if (sp->time_step<=0) sp->time_step = NUDGE_DEFAULT_SIMULATION_TIMESTEP;
    assert(sp->time_step>0);
    if (sp->max_num_substeps==0) sp->max_num_substeps = NUDGE_DEFAULT_MAX_NUM_SIMULATION_SUBSTEPS;
    assert(sp->max_num_substeps>0);
    if (sp->num_iterations_per_substep==0) sp->num_iterations_per_substep = NUDGE_DEFAULT_NUM_SIMULATION_ITERATIONS;
    assert(sp->num_iterations_per_substep>0);
    if (sp->sleeping_threshold_linear_velocity_squared<=0.f) sp->sleeping_threshold_linear_velocity_squared = NUDGE_DEFAULT_SLEEPING_THRESHOLD_LINEAR_VELOCITY_SQUARED;
    if (sp->sleeping_threshold_angular_velocity_squared<=0.f) sp->sleeping_threshold_angular_velocity_squared = NUDGE_DEFAULT_SLEEPING_THRESHOLD_ANGULAR_VELOCITY_SQUARED;
    if (sp->linear_damping<=0.f) sp->linear_damping = NUDGE_DEFAULT_DAMPING_LINEAR;
    if (sp->angular_damping<=0.f) sp->angular_damping = NUDGE_DEFAULT_DAMPING_ANGULAR;
    if (sp->penetration_allowed_amount<=0) sp->penetration_allowed_amount = NUDGE_DEFAULT_PENETRATION_ALLOWED_AMOUNT;
    if (sp->penetration_bias_factor<=0) sp->penetration_bias_factor = NUDGE_DEFAULT_PENETRATION_BIAS_FACTOR;
    if (sp->numsubsteps_overflow_warning_mode>2) sp->numsubsteps_overflow_warning_mode=0;
    sp->num_total_substeps=sp->num_frames=0;

    // Allocate memory for simulation arena.
#   ifndef NUDGE_ARENA_SIZE_ALIGNMENT
#       define NUDGE_ARENA_SIZE_ALIGNMENT (4096)      // is this correct? Isn't 4096 too much?
#   endif
    assert(c->arena.size==0);assert(c->arena.data==NULL);  // sharing it could be useful, we should allow it and add an 'owned' flag
    c->arena.size = NUDGE_ARENA_SIZE_MACRO(NUDGE_MAX_NUM_BODIES);
    c->arena.data = _mm_malloc(c->arena.size, NUDGE_ARENA_SIZE_ALIGNMENT);//memset(c->arena.data,0,c->arena.size);

    // Allocate memory for bodies, colliders, and contacts.
    c->active_bodies.capacity = NUDGE_MAX_NUM_BODIES;
    c->active_bodies.indices = static_cast<uint16_t*>(_mm_malloc(sizeof(uint16_t)*NUDGE_MAX_NUM_BODIES, 64));

    c->bodies.transforms = static_cast<Transform*>(_mm_malloc(sizeof(Transform)*NUDGE_MAX_NUM_BODIES, 64));
    c->bodies.momentum = static_cast<BodyMomentum*>(_mm_malloc(sizeof(BodyMomentum)*NUDGE_MAX_NUM_BODIES, 64));
    c->bodies.properties = static_cast<BodyProperties*>(_mm_malloc(sizeof(BodyProperties)*NUDGE_MAX_NUM_BODIES, 64));
    c->bodies.filters = static_cast<BodyFilter*>(_mm_malloc(sizeof(BodyFilter)*NUDGE_MAX_NUM_BODIES, 64));
    c->bodies.layouts = static_cast<BodyLayout*>(_mm_malloc(sizeof(BodyLayout)*NUDGE_MAX_NUM_BODIES, 64));
    c->bodies.idle_counters = static_cast<uint8_t*>(_mm_malloc(sizeof(uint8_t)*NUDGE_MAX_NUM_BODIES, 64));
    c->bodies.infos = static_cast<BodyInfo*>(_mm_malloc(sizeof(BodyInfo)*NUDGE_MAX_NUM_BODIES,64));    

    c->colliders.boxes.data = static_cast<BoxCollider*>(_mm_malloc(sizeof(BoxCollider)*NUDGE_MAX_NUM_BOXES, 64));
    c->colliders.boxes.tags = static_cast<uint16_t*>(_mm_malloc(sizeof(uint16_t)*NUDGE_MAX_NUM_BOXES, 64));
    c->colliders.boxes.transforms = static_cast<Transform*>(_mm_malloc(sizeof(Transform)*NUDGE_MAX_NUM_BOXES, 64));        

    c->colliders.spheres.data = static_cast<SphereCollider*>(_mm_malloc(sizeof(SphereCollider)*NUDGE_MAX_NUM_SPHERES, 64));
    c->colliders.spheres.tags = static_cast<uint16_t*>(_mm_malloc(sizeof(uint16_t)*NUDGE_MAX_NUM_SPHERES, 64));
    c->colliders.spheres.transforms = static_cast<Transform*>(_mm_malloc(sizeof(Transform)*NUDGE_MAX_NUM_SPHERES, 64));

    c->contact_data.capacity = NUDGE_MAX_NUM_BODIES*64;
    c->contact_data.bodies = static_cast<BodyPair*>(_mm_malloc(sizeof(BodyPair)*c->contact_data.capacity, 64));
    c->contact_data.data = static_cast<Contact*>(_mm_malloc(sizeof(Contact)*c->contact_data.capacity, 64));
    c->contact_data.tags = static_cast<uint64_t*>(_mm_malloc(sizeof(uint64_t)*c->contact_data.capacity, 64));
    c->contact_data.sleeping_pairs = static_cast<uint32_t*>(_mm_malloc(sizeof(uint32_t)*c->contact_data.capacity, 64));

    c->contact_cache.capacity = NUDGE_MAX_NUM_BODIES*64;
    c->contact_cache.data = static_cast<CachedContactImpulse*>(_mm_malloc(sizeof(CachedContactImpulse)*c->contact_cache.capacity, 64));
    c->contact_cache.tags = static_cast<uint64_t*>(_mm_malloc(sizeof(uint64_t)*c->contact_cache.capacity, 64));

    c->kinematic_data.key_frame_capacity = NUDGE_TOTAL_NUM_KINEMATIC_ANIMATION_KEY_FRAMES;
    c->kinematic_data.key_frame_transforms = static_cast<Transform*>(_mm_malloc(sizeof(Transform)*c->kinematic_data.key_frame_capacity,64));
    c->kinematic_data.key_frame_modes = static_cast<KinematicData::TimeMode*>(_mm_malloc(sizeof(KinematicData::TimeMode)*c->kinematic_data.key_frame_capacity,64));
    c->kinematic_data.animations_capacity = NUDGE_MAX_NUM_KINEMATIC_ANIMATIONS;
    c->kinematic_data.animations = static_cast<KinematicData::Animation*>(_mm_malloc(sizeof(KinematicData::Animation)*c->kinematic_data.animations_capacity,64));

    *((uint32_t*)&c->global_data.removed_bodies_capacity) = NUDGE_MAX_NUM_BODIES;assert(c->global_data.removed_bodies_capacity==NUDGE_MAX_NUM_BODIES);
    c->global_data.removed_bodies = static_cast<uint32_t*>(_mm_malloc(sizeof(uint32_t)*c->global_data.removed_bodies_capacity, 64));
    c->global_data.flags = 0;
    c->global_data.exclude_smoothing_graphic_transform_flags=0;
    c->global_data.gravity[0]=c->global_data.gravity[2]=0.f;
    c->global_data.gravity[1]=NUDGE_DEFAULT_GRAVITY;

    restart_context(c);
}
void init_context(context_t* c) {init_context_with(c,NUDGE_DEFAULT_MAX_NUM_BOXES,NUDGE_DEFAULT_MAX_NUM_SPHERES);}
void destroy_context(context_t* c)   {
    assert(c->MAX_NUM_BODIES>0);
    assert(c->global_data.removed_bodies_capacity>0 && c->global_data.removed_bodies);
    _mm_free(c->global_data.removed_bodies);c->global_data.removed_bodies_count = c->global_data.finalized_removed_bodies_count = 0;
    *((uint32_t*)&c->global_data.removed_bodies_capacity) = 0;

    _mm_free(c->kinematic_data.animations);c->kinematic_data.animations_capacity = c->kinematic_data.animations_count = 0;
    _mm_free(c->kinematic_data.key_frame_modes);
    _mm_free(c->kinematic_data.key_frame_transforms);
    c->kinematic_data.key_frame_capacity = c->kinematic_data.key_frame_count = 0;

    _mm_free(c->contact_cache.data);c->contact_cache.data = 0;
    _mm_free(c->contact_cache.tags);c->contact_cache.tags = 0;
    c->contact_cache.capacity = c->contact_cache.count = 0;

    _mm_free(c->contact_data.bodies);c->contact_data.bodies = 0;
    _mm_free(c->contact_data.data);c->contact_data.data = 0;
    _mm_free(c->contact_data.tags);c->contact_data.tags = 0;
    _mm_free(c->contact_data.sleeping_pairs);c->contact_data.sleeping_pairs = 0;
    c->contact_data.count = c->contact_data.capacity = 0;

    _mm_free(c->colliders.spheres.data);c->colliders.spheres.data = 0;
    _mm_free(c->colliders.spheres.tags);c->colliders.spheres.tags = 0;
    _mm_free(c->colliders.spheres.transforms);c->colliders.spheres.transforms = 0;
    c->colliders.spheres.count = 0;

    _mm_free(c->colliders.boxes.data);c->colliders.boxes.data = 0;
    _mm_free(c->colliders.boxes.tags);c->colliders.boxes.tags = 0;
    _mm_free(c->colliders.boxes.transforms);c->colliders.boxes.transforms = 0;
    c->colliders.boxes.count = 0;

    _mm_free(c->bodies.infos);c->bodies.infos = 0;
    _mm_free(c->bodies.idle_counters);c->bodies.idle_counters = 0;
    _mm_free(c->bodies.filters);c->bodies.filters = 0;
    _mm_free(c->bodies.layouts);c->bodies.layouts = 0;
    _mm_free(c->bodies.transforms);c->bodies.transforms = 0;
    _mm_free(c->bodies.momentum);c->bodies.momentum = 0;
    _mm_free(c->bodies.properties);c->bodies.properties = 0;
    c->bodies.count = 0;

    _mm_free(c->active_bodies.indices);c->active_bodies.indices = 0;
    c->active_bodies.capacity = c->active_bodies.count = 0;

    _mm_free(c->arena.data);c->arena.data = 0;
    c->arena.size = 0;

    //memset(c,0,sizeof(*c));
    *((unsigned*)&c->MAX_NUM_BODIES) = *((unsigned*)&c->MAX_NUM_BOXES) = *((unsigned*)&c->MAX_NUM_SPHERES) =0;
}


typedef unsigned body_type;
static context_t* _tmpc = NULL;
static inline int _compare_bodies_by_box_collider(const void* av,const void*bv) {
    const body_type a = *((body_type*)av), b = *((body_type*)bv);
    assert(_tmpc);
    assert(a<_tmpc->bodies.count);
    assert(b<_tmpc->bodies.count);
    const int aa=_tmpc->bodies.layouts[a].first_box_index,bb=_tmpc->bodies.layouts[b].first_box_index;
    return (aa<bb)?-1:(aa>bb)?1:0;
}
static inline int _compare_bodies_by_sphere_collider(const void* av,const void*bv) {
    const body_type a = *((body_type*)av), b = *((body_type*)bv);
    assert(_tmpc);
    assert(a<_tmpc->bodies.count);
    assert(b<_tmpc->bodies.count);
    const int aa=_tmpc->bodies.layouts[a].first_sphere_index,bb=_tmpc->bodies.layouts[b].first_sphere_index;
    return (aa<bb)?-1:(aa>bb)?1:0;
}
void finalize_removed_bodies(context_t* c) {
    // I honestly don't know what alse we should do other then removing the collision shapes here
    // Here are some optional attempts to perform other tasks:
    const int clean_active_bodies = 0;   // removes the removed bodies from the c->active_body list
    const int clean_contact_data = 0;    // slow... removes the c->contact_data that refer to at least a removed body
    const int clean_cached_impulses = 0; // slowest... (same for c->contact_cache) + implementation is probably wrong.
    // End optional attempts

    const int32_t removed_bodies_count = (int32_t) c->global_data.removed_bodies_count;
    const int32_t finalized_removed_bodies_count = (int32_t) c->global_data.finalized_removed_bodies_count;
    assert(finalized_removed_bodies_count<=removed_bodies_count);
    if (finalized_removed_bodies_count==removed_bodies_count) return;

    //log("[nudge_frame:%llu] finalize_removed_bodies(...): finalized_removed_bodies_count=%d removed_bodies_count=%d\n",c->simulation_params.num_frames,finalized_removed_bodies_count,removed_bodies_count);
    assert(sizeof(body_type)==sizeof(c->global_data.removed_bodies[0]));
    body_type* removed_bodies = &c->global_data.removed_bodies[0];
    int16_t start;uint16_t count;
    BodyLayout* layouts = c->bodies.layouts;
    uint32_t num_boxes_to_remove=0,num_spheres_to_remove=0;int i;
    uint32_t max_num_allocated_tags = 0;
    Arena arena = c->arena;
    uint16_t* tags = NULL;

    // [pre-processing for arena allocation]
    for (i=removed_bodies_count-1;i>=finalized_removed_bodies_count;--i) {
        const body_type body = removed_bodies[i];assert(body<c->bodies.count);
        const uint16_t num_boxes=layouts[body].num_boxes;
        const uint16_t num_spheres=layouts[body].num_spheres;
        num_boxes_to_remove+=num_boxes;
        num_spheres_to_remove+=num_spheres;
        // dbg:
        if (num_boxes>0) {assert(layouts[body].first_box_index>=0);}
        if (num_spheres>0) {assert(layouts[body].first_sphere_index>=0);}

        if (clean_active_bodies && c->active_bodies.count) {
            for (int j=(int)(c->active_bodies.count-1);j>=0;--j) {
                assert(j<(int)c->active_bodies.count);
                if (body==c->active_bodies.indices[j]) {
                    //  |-------------|--|-----------|
                    //  0             j j+1        count
                    memmove(&c->active_bodies.indices[j],&c->active_bodies.indices[j+1],sizeof(c->active_bodies.indices[0])*(c->active_bodies.count-(j+1)));
                    --c->active_bodies.count;
                }
            }
        }
        if (clean_contact_data && c->contact_data.count) {
            for (int j=(int)(c->contact_data.count-1);j>=0;--j) {
                // struct ContactData {Contact* data;BodyPair* bodies;uint64_t* tags;uint32_t capacity;uint32_t count;   uint32_t* sleeping_pairs;uint32_t sleeping_count;};
                const BodyPair* bp = &c->contact_data.bodies[j];
                if (body==bp->a || body==bp->b) {
                    memmove(&c->contact_data.data[j],&c->contact_data.data[j+1],sizeof(c->contact_data.data[0])*(c->contact_data.count-(j+1)));
                    memmove(&c->contact_data.bodies[j],&c->contact_data.bodies[j+1],sizeof(c->contact_data.bodies[0])*(c->contact_data.count-(j+1)));
                    memmove(&c->contact_data.tags[j],&c->contact_data.tags[j+1],sizeof(c->contact_data.tags[0])*(c->contact_data.count-(j+1)));
                    --c->active_bodies.count;
                }
                // What are 'sleeping_pairs' and 'sleeping_pairs_count' inside 'ContactData'?
            }
        }
    }
    max_num_allocated_tags = num_boxes_to_remove>num_spheres_to_remove?num_boxes_to_remove:num_spheres_to_remove;
    tags = allocate_array<uint16_t>(&arena, max_num_allocated_tags, 32);
    assert(sizeof(tags[0])==sizeof(c->colliders.boxes.tags[0]));
    assert(sizeof(tags[0])==sizeof(c->colliders.spheres.tags[0]));

    // [boxes]
    if (num_boxes_to_remove>0)
    {
        _tmpc=c;qsort(&removed_bodies[finalized_removed_bodies_count],removed_bodies_count-finalized_removed_bodies_count,sizeof(body_type),&_compare_bodies_by_box_collider);_tmpc=NULL;
        // process
        uint32_t num_finalized_boxes=0;unsigned moveGap,amount,lastBodyId;
        const body_type last_body = removed_bodies[removed_bodies_count-1];
        start=layouts[last_body].first_box_index;count=layouts[last_body].num_boxes;
        for (i=removed_bodies_count-1;i>=finalized_removed_bodies_count;--i) {
            if (i>finalized_removed_bodies_count)   {
                const body_type body = removed_bodies[i-1]; // prev_body actually (next in the for loop)
                const int16_t body_start = layouts[body].first_box_index;
                const uint16_t body_count = layouts[body].num_boxes;
                if (body_start+body_count==start) {start=body_start;count+=body_count;continue;}
            }
            if (count>0)    {
                assert(start>=0);
                //log("[nudge_frame:%llu] [Finalize %u/%u Boxes in [%d,%u): c->colliders.boxes.count=%u]\n",c->simulation_params.num_frames,count,num_boxes_to_remove,start,start+count,c->colliders.boxes.count);
                num_finalized_boxes+=count;
                // process interval: move [start,start+count) after c->colliders.boxes.count
                // remove [start,start+count)
                /*
                |-----------|----------------------|------------------------|
                            start               start+count           c->colliders.boxes.count
                            |                      |                        |
                            |-------moveGap--------|---------amount---------|

                After the move we want:

                |-----------|----------------------|------------------------|
                            start         c->colliders.boxes.count            |
                            |                      |                        |
                            |--------amount--------|---------moveGap--------|

                We also need to keep the original tags that were present in [start,start+count)
                and place them in the unused space past (the new) c->colliders.boxes.count
                */
                assert(start+count<=(int)c->colliders.boxes.count);
                moveGap = count;amount = c->colliders.boxes.count-(start+count);
                memmove(&c->colliders.boxes.data[start],&c->colliders.boxes.data[start+count],amount*sizeof(c->colliders.boxes.data[0]));
                memmove(&c->colliders.boxes.transforms[start],&c->colliders.boxes.transforms[start+count],amount*sizeof(c->colliders.boxes.transforms[0]));
                // handle tags the correct way! -------------
                assert(count<=max_num_allocated_tags);
                memcpy(tags,&c->colliders.boxes.tags[start],moveGap*sizeof(c->colliders.boxes.tags[0]));    // pre-move
                memmove(&c->colliders.boxes.tags[start],&c->colliders.boxes.tags[start+count],amount*sizeof(c->colliders.boxes.tags[0]));
                //memcpy(&c->colliders.boxes.tags[count-moveGap],tags,moveGap*sizeof(c->colliders.boxes.tags[0])); // post-move
                memcpy(&c->colliders.boxes.tags[start+amount],tags,moveGap*sizeof(c->colliders.boxes.tags[0])); // post-move
                // -------------------------------------------
                c->colliders.boxes.count-=count;

                // we must re-assign c->bodies.infos[bodyId].first_box_index for
                // all the moved box-colliders in [start,c->colliders.boxes.count)
                lastBodyId = c->MAX_NUM_BODIES;
                for (unsigned i=start,isz=c->colliders.boxes.count;i<isz;i++)   {
                    const unsigned bodyId = c->colliders.boxes.transforms[i].body;
                    assert(bodyId<c->bodies.count);                    
                    assert(!(c->bodies.filters[bodyId].flags&BF_IS_REMOVED));   // not sure about this
                    if (lastBodyId!=bodyId)   {
                        lastBodyId=bodyId;
                        BodyLayout* bl = &c->bodies.layouts[bodyId];
                        assert(bl->first_box_index>=0 && bl->num_boxes>0);
                        assert(bl->first_box_index>=start+count);
                        assert((int16_t)i==bl->first_box_index-(int16_t)count);
                        bl->first_box_index = (int16_t) i;
                        assert((uint16_t)bl->first_box_index+bl->num_boxes<=c->colliders.boxes.count);
                    }
                }
            }
            //---------------------------------------------
            if (i>finalized_removed_bodies_count)   {
                const body_type body = removed_bodies[i-1]; // prev_body actually (next in the for loop)
                start = layouts[body].first_box_index;
                count = layouts[body].num_boxes;
            }
        }
        if (num_finalized_boxes!=num_boxes_to_remove) {
            log("[nudge_frame:%llu] finalize_removed_bodies(...) has NOT handled %u box colliders and %u sphere colliders\n",c->simulation_params.num_frames,num_boxes_to_remove,num_spheres_to_remove);
            flush();
            assert(num_finalized_boxes==num_boxes_to_remove);
        }
    }

    // [spheres]
    if (num_spheres_to_remove>0)
    {
        _tmpc=c;qsort(&removed_bodies[finalized_removed_bodies_count],removed_bodies_count-finalized_removed_bodies_count,sizeof(body_type),&_compare_bodies_by_sphere_collider);_tmpc=NULL;
        // process
        uint32_t num_finalized_spheres=0;unsigned moveGap,amount,lastBodyId;
        const body_type last_body = removed_bodies[removed_bodies_count-1];
        start=layouts[last_body].first_sphere_index;count=layouts[last_body].num_spheres;
        for (i=removed_bodies_count-1;i>=finalized_removed_bodies_count;--i) {
            if (i>finalized_removed_bodies_count)   {
                const body_type body = removed_bodies[i-1]; // prev_body actually (next in the for loop)
                const int16_t body_start = layouts[body].first_sphere_index;
                const uint16_t body_count = layouts[body].num_spheres;
                if (body_start+body_count==start) {start=body_start;count+=body_count;continue;}
            }
            if (count>0)    {
                assert(start>=0);
                //log("[nudge_frame:%llu] [Finalize %u/%u Spheres in [%d,%u): c->colliders.spheres.count=%u]\n",c->simulation_params.num_frames,count,num_spheres_to_remove,start,start+count,c->colliders.spheres.count);
                num_finalized_spheres+=count;
                // process interval: move [start,start+count) after c->colliders.spheres.count
                // remove [start,start+count)
                /*
                        |-----------|----------------------|------------------------|
                                  start               start+count           c->colliders.spheres.count
                                    |                      |                        |
                                    |-------moveGap--------|---------amount---------|

                        After the move we want:

                        |-----------|----------------------|------------------------|
                                  start         c->colliders.spheres.count          |
                                    |                      |                        |
                                    |--------amount--------|---------moveGap--------|

                        We also need to keep the original tags that were present in [start,start+count)
                        and place them in the unused space past (the new) c->colliders.spheres.count
                */
                assert(start+count<=(int)c->colliders.spheres.count);
                moveGap = count;amount = c->colliders.spheres.count-(start+count);    // 46-(46+1) = -1 [TODO: ERROR!]
                memmove(&c->colliders.spheres.data[start],&c->colliders.spheres.data[start+count],amount*sizeof(c->colliders.spheres.data[0]));
                memmove(&c->colliders.spheres.transforms[start],&c->colliders.spheres.transforms[start+count],amount*sizeof(c->colliders.spheres.transforms[0]));
                // handle tags the correct way! -------------
                assert(count<=max_num_allocated_tags);
                memcpy(tags,&c->colliders.spheres.tags[start],moveGap*sizeof(c->colliders.spheres.tags[0]));    // pre-move
                memmove(&c->colliders.spheres.tags[start],&c->colliders.spheres.tags[start+count],amount*sizeof(c->colliders.spheres.tags[0]));
                //memcpy(&c->colliders.spheres.tags[count-moveGap],tags,moveGap*sizeof(c->colliders.spheres.tags[0])); // post-move
                memcpy(&c->colliders.spheres.tags[start+amount],tags,moveGap*sizeof(c->colliders.spheres.tags[0])); // post-move
                // -------------------------------------------
                c->colliders.spheres.count-=count;

                // we must re-assign c->bodies.infos[bodyId].first_sphere_index for
                // all the moved sphere-colliders in [start,c->colliders.spheres.count)
                lastBodyId = c->MAX_NUM_BOXES+c->MAX_NUM_SPHERES;assert(c->MAX_NUM_BODIES==c->MAX_NUM_BOXES+c->MAX_NUM_SPHERES);
                for (unsigned i=start,isz=c->colliders.spheres.count;i<isz;i++)   {
                    const unsigned bodyId = c->colliders.spheres.transforms[i].body;
                    assert(bodyId<c->bodies.count);                    
                    assert(!(c->bodies.filters[bodyId].flags&BF_IS_REMOVED));
                    if (lastBodyId!=bodyId)   {
                        lastBodyId=bodyId;
                        BodyLayout* bl = &c->bodies.layouts[bodyId];
                        assert(bl->first_sphere_index>=0 && bl->num_spheres>0);
                        assert(bl->first_sphere_index>=start+count);
                        assert((int16_t)i==bl->first_sphere_index-(int16_t)count);
                        bl->first_sphere_index = (int16_t) i;
                        assert((uint16_t)bl->first_sphere_index+bl->num_spheres<=c->colliders.spheres.count);
                    }
                }
            }
            //---------------------------------------------
            if (i>finalized_removed_bodies_count)   {
                const body_type body = removed_bodies[i-1]; // prev_body actually (next in the for loop)
                start = layouts[body].first_sphere_index;
                count = layouts[body].num_spheres;
            }
        }
        if (num_finalized_spheres!=num_spheres_to_remove) {
            log("[nudge_frame:%llu] finalize_removed_bodies(...) has NOT handled %u box colliders and %u sphere colliders\n",c->simulation_params.num_frames,num_boxes_to_remove,num_spheres_to_remove);
            flush();
            assert(num_finalized_spheres==num_spheres_to_remove);
        }
    }

    // remove colliders from removed objects
    for (int i=finalized_removed_bodies_count;i<removed_bodies_count;i++) {
        const body_type body = removed_bodies[i];
        BodyLayout* bl = &c->bodies.layouts[body];
        bl->first_box_index=-1;bl->first_sphere_index=-1;
        bl->num_boxes=0;bl->num_spheres=0;
        BodyInfo* info = &c->bodies.infos[body];    // reset some info data too
        memset(&info->aabb_center[0],0,3*sizeof(float));
        memset(&info->aabb_half_extents[0],0,3*sizeof(float));
        memset(&info->com_offset[0],0,3*sizeof(float));
        info->aabb_enlarged_radius=0.f;
    }
    c->global_data.finalized_removed_bodies_count=c->global_data.removed_bodies_count;

    //log("[nudge_frame:%llu] finalize_removed_bodies(...) has handled %u box colliders and %u sphere colliders\n",c->simulation_params.num_frames,num_boxes_to_remove,num_spheres_to_remove);
    //flush();
    assert(num_boxes_to_remove || num_spheres_to_remove);


    if (clean_cached_impulses && c->contact_cache.count) {
        // not robust enough...
        for (int i=(int)c->contact_cache.count-1;i>=0;--i) {
            //struct ContactCache {uint64_t* tags;CachedContactImpulse* data;uint32_t capacity;uint32_t count;};
            const uint64_t tag = c->contact_cache.tags[i];
            const uint16_t a_tag = (uint16_t) ((tag&0x0000FFFF00000000ULL)>>(2ULL*16ULL));
            const uint16_t b_tag = (uint16_t) ((tag&0xFFFF000000000000ULL)>>(3ULL*16ULL));
            //const uint16_t a_tag = (uint16_t) ((tag&0x000000000000FFFFULL)>>(0ULL*16ULL));
            //const uint16_t b_tag = (uint16_t) ((tag&0x00000000FFFF0000ULL)>>(1ULL*16ULL));
            int found=0;
            // here I'm assuming that the tags of the colliders we've just removed
            // are in the range: [j,jsz]. Hope it's correct...
            for (unsigned j=c->colliders.boxes.count,jsz=c->colliders.boxes.count+num_boxes_to_remove;j<jsz;j++) {
                const uint16_t tg = c->colliders.boxes.tags[j];
                if (tg==a_tag || tg==b_tag) {found = 1;break;}
            }
            if (!found) {
                for (unsigned j=c->colliders.spheres.count,jsz=c->colliders.spheres.count+num_spheres_to_remove;j<jsz;j++) {
                    const uint16_t tg = c->colliders.spheres.tags[j];
                    if (tg==a_tag || tg==b_tag) {found = 1;break;}
                }
            }
            if (found) {
                // remove c->contact_cache[i]
                // |---------------|--|----------------|
                // 0               i i+1              count
                memmove(&c->contact_cache.tags[i],&c->contact_cache.tags[i+1],sizeof(c->contact_cache.tags[0])*(c->contact_cache.count-(i+1)));
                memmove(&c->contact_cache.data[i],&c->contact_cache.data[i+1],sizeof(CachedContactImpulse)*(c->contact_cache.count-(i+1)));
                --c->contact_cache.count;
            }
        }
    }

#   define TEST_NUDGE_COLLIDER_TAGS_INTEGRITY   // TO REMOVE
#   ifdef TEST_NUDGE_COLLIDER_TAGS_INTEGRITY
    {
        Arena arena = c->arena;
        if (arena.size>=sizeof(uint8_t)*(c->MAX_NUM_BOXES+c->MAX_NUM_SPHERES))  {
            uint8_t* tagsMap = allocate_array<uint8_t>(&arena, c->MAX_NUM_BOXES+c->MAX_NUM_SPHERES, 32);assert(tagsMap);
            memset(tagsMap,0,(c->MAX_NUM_BOXES+c->MAX_NUM_SPHERES)*sizeof(uint8_t));
            for (unsigned i=0;i<c->MAX_NUM_BOXES;i++) {
                const uint16_t tag = c->colliders.boxes.tags[i];
                assert(tag<c->MAX_NUM_BOXES);
                assert(tagsMap[tag]==0);
                tagsMap[tag]=1;
            }
            for (unsigned i=0;i<c->MAX_NUM_SPHERES;i++) {
                uint16_t tag = c->colliders.spheres.tags[i];
                assert(tag>=NUDGE_START_SPHERE_TAG && tag<NUDGE_START_SPHERE_TAG+c->MAX_NUM_SPHERES);
                tag=tag-NUDGE_START_SPHERE_TAG+c->MAX_NUM_BOXES;
                assert(tagsMap[tag]==0);
                tagsMap[tag]=1;
            }
            unsigned unset_tags=0;
            for (unsigned i=0;i<c->MAX_NUM_BOXES+c->MAX_NUM_SPHERES;i++) {
                if (!tagsMap[i]) ++unset_tags;
            }
            assert(unset_tags==0);
        }
    }
#   endif //TEST_NUDGE_COLLIDER_TAGS_INTEGRITY

#   define TEST_COLLIDER_COHERENCY  // TO REMOVE
#   ifdef  TEST_COLLIDER_COHERENCY
    {
        unsigned body_last = NUDGE_INVALID_BODY_ID,delta_shape_count=0;
        for (unsigned i=0;i<c->colliders.boxes.count;i++) {
            //const uint16_t tag = &c->colliders.boxes.tags[i];
            const unsigned body = c->colliders.boxes.transforms[i].body;
            assert(body<c->bodies.count);
            BodyLayout* bl = &c->bodies.layouts[body];
            if (body_last!=body)   {
                body_last=body;
                assert(bl->first_box_index==(int)i);
                delta_shape_count=0;
            }
            else {
                ++delta_shape_count;
                assert(i>=(uint16_t)bl->first_box_index);
                assert(i<(uint16_t)bl->first_box_index+bl->num_boxes);
                assert(i==(uint16_t)bl->first_box_index+delta_shape_count);
            }
        }
        body_last = NUDGE_INVALID_BODY_ID;delta_shape_count=0;
        for (unsigned i=0;i<c->colliders.spheres.count;i++) {
            //const uint16_t tag = &c->colliders.spheres.tags[i];
            const unsigned body = c->colliders.spheres.transforms[i].body;
            assert(body<c->bodies.count);
            assert(!(c->bodies.filters[body].flags&BF_IS_REMOVED));   // not sure about this
            BodyLayout* bl = &c->bodies.layouts[body];assert(bl->first_sphere_index>=0);
            if (body_last!=body)    {
                body_last=body;
                assert(bl->first_sphere_index==(int)i);
                delta_shape_count=0;
            }
            else {
                ++delta_shape_count;
                assert(bl->first_sphere_index>=0);
                assert(i>=(uint16_t)bl->first_sphere_index);
                assert(i<(uint16_t)bl->first_sphere_index+bl->num_spheres);
                assert(i==(uint16_t)bl->first_sphere_index+delta_shape_count);
            }
        }
    }
#   endif

}

void remove_body(context_t* c,unsigned body)    {
    assert(body<c->bodies.count);
    for (unsigned i=0;i<c->global_data.removed_bodies_count;i++) {if (c->global_data.removed_bodies[i]==body) return;}
    assert(c->global_data.removed_bodies_count<c->MAX_NUM_BODIES);
    BodyFilter* f = &c->bodies.filters[body];
    f->flags|=BF_IS_DISABLED_OR_REMOVED;
    f->collision_group=f->collision_mask=0;
    c->bodies.idle_counters[body]=0xff;              // set it to sleep
    c->bodies.properties[body].mass_inverse = 0.f;   // turn it to static so it stops falling
    f->flags&=~(BF_IS_DYNAMIC|BF_IS_KINEMATIC);f->flags|=BF_IS_STATIC;    // not sure about this
    float* lvel = &c->bodies.momentum[body].velocity[0];
    float* avel = &c->bodies.momentum[body].angular_velocity[0];
    lvel[0]=lvel[1]=lvel[2]=avel[0]=avel[1]=avel[2]=0.f;
    float* pos = c->bodies.transforms[body].position;pos[1]-=100000.f;  // probably useless
    c->global_data.removed_bodies[c->global_data.removed_bodies_count++] = body;
}

uint32_t colliders_get_num_remaining_boxes(context_t* c) {assert(c->colliders.boxes.count<=c->MAX_NUM_BOXES);return c->MAX_NUM_BOXES-c->colliders.boxes.count;}
uint32_t colliders_get_num_remaining_spheres(context_t* c) {assert(c->colliders.spheres.count<=c->MAX_NUM_SPHERES);return c->MAX_NUM_SPHERES-c->colliders.spheres.count;}
    

unsigned add_box(context_t* c,float mass, float hsizex, float hsizey, float hsizez, const Transform* T, const float comOffset[3])    {
    unsigned body,collider;
    if (c->global_data.finalized_removed_bodies_count>0) {
        assert(c->global_data.finalized_removed_bodies_count<=c->global_data.removed_bodies_count);
        assert(sizeof(c->global_data.removed_bodies[0])==sizeof(body_type));
        body=c->global_data.removed_bodies[0];--c->global_data.finalized_removed_bodies_count;--c->global_data.removed_bodies_count;
        memmove(&c->global_data.removed_bodies[0],&c->global_data.removed_bodies[1],c->global_data.removed_bodies_count*sizeof(body_type));
        assert(body<c->bodies.count);
        const BodyLayout* bl = &c->bodies.layouts[body];
        assert(bl->first_box_index==-1);
        assert(bl->num_boxes==0);
        assert(bl->first_sphere_index==-1);
        assert(bl->num_spheres==0);
    }
    else {
        assert(c->bodies.count<c->MAX_NUM_BODIES && c->colliders.boxes.count<c->MAX_NUM_BOXES);   // Further boxes can't be added
        if (c->bodies.count == c->MAX_NUM_BODIES || c->colliders.boxes.count == c->MAX_NUM_BOXES) return NUDGE_INVALID_BODY_ID;
        body = c->bodies.count++;
    }

    BodyProperties* prop = &c->bodies.properties[body];
    Transform* xform = &c->bodies.transforms[body], *xform_collider = NULL;
    assert(xform);
    BodyInfo* info = &c->bodies.infos[body];BodyFilter* filter = &c->bodies.filters[body];BodyLayout* layout = &c->bodies.layouts[body];
    if (comOffset && comOffset[0]==0.f && comOffset[1]==0.f && comOffset[2]==0.f) comOffset = NULL;
    collider = c->colliders.boxes.count++;
    BoxCollider* boxCollider = &c->colliders.boxes.data[collider];
    filter->flags = mass>0?BF_IS_DYNAMIC:(mass<0?BF_IS_KINEMATIC:BF_IS_STATIC);

    *xform = T ? (*T) : identity_transform; // transform
    xform->body = body; // body id
    memset(&c->bodies.momentum[body], 0, sizeof(c->bodies.momentum[body])); // no velocity/angular velocity
    memset(prop,0,sizeof(*prop));prop->friction = NUDGE_DEFAULT_FRICTION;prop->gravity[1]=NUDGE_DEFAULT_GRAVITY;   // reset mass/inertia/friction
    if (mass<0) mass=-mass;
    if (mass>0)   {prop->mass_inverse = 1.0f/mass;calculate_box_inertia_inverse(prop->inertia_inverse,mass,hsizex,hsizey,hsizez,comOffset);}
    c->bodies.idle_counters[body] = (filter->flags&BF_IS_DYNAMIC)?0:0xff;

    //memset(info,0,sizeof(BodyInfo));
    info->com_offset[0]=info->com_offset[1]=info->com_offset[2]=0.f;
    layout->num_boxes = 1;layout->num_spheres = 0;
    layout->first_box_index = (int16_t) collider;
    layout->first_sphere_index = -1;
    boxCollider->size[0] = hsizex;
    boxCollider->size[1] = hsizey;
    boxCollider->size[2] = hsizez;
    xform_collider = &c->colliders.boxes.transforms[collider];
    *xform_collider = identity_transform;
    xform_collider->body = body;
    if (comOffset) {filter->flags|=BF_HAS_COM_OFFSET;for (int l=0;l<3;l++) {xform_collider->position[l]-=comOffset[l];info->com_offset[l]=comOffset[l];}}

    filter->collision_group = COLLISION_GROUP_DEFAULT;filter->collision_mask = COLLISION_GROUP_ALL;
    //log("%d) Added box [mass:%1.3f;hsize{%1.3f,%1.3f,%1.3f};pos{%1.3f,%1.3f,%1.3f}]\n",body,mass,hsizex,hsizey,hsizez,T->position[0],T->position[1],T->position[2]);
    //assert(getBoxColliderId(c->bodies.count-1)==collider);

    body_recalculate_bounding_box(c,body);  // new

    return body;
}
unsigned add_box(context_t* c,float mass, float hsizex, float hsizey, float hsizez, const float* mMatrix16WithoutScaling, const float comOffset[3]) {
    if (!mMatrix16WithoutScaling)   return add_box(c,mass,hsizex,hsizey,hsizez,(const Transform*)NULL);
    else {Transform T;Mat4WithoutScalingToTransform(&T,mMatrix16WithoutScaling);return add_box(c,mass,hsizex,hsizey,hsizez,&T,comOffset);}
}

unsigned add_sphere(context_t* c, float mass, float radius, const Transform* T, const float comOffset[3])    {
    unsigned body,collider;
    if (c->global_data.finalized_removed_bodies_count>0) {
        assert(c->global_data.finalized_removed_bodies_count<=c->global_data.removed_bodies_count);
        assert(sizeof(c->global_data.removed_bodies[0])==sizeof(body_type));
        body=c->global_data.removed_bodies[0];--c->global_data.finalized_removed_bodies_count;--c->global_data.removed_bodies_count;
        memmove(&c->global_data.removed_bodies[0],&c->global_data.removed_bodies[1],c->global_data.removed_bodies_count*sizeof(body_type));
        assert(body<c->bodies.count);
        const BodyLayout* bl = &c->bodies.layouts[body];
        assert(bl->first_box_index==-1);
        assert(bl->num_boxes==0);
        assert(bl->first_sphere_index==-1);
        assert(bl->num_spheres==0);
    }
    else {
        assert(c->bodies.count<c->MAX_NUM_BODIES && c->colliders.spheres.count<c->MAX_NUM_SPHERES);   // Further spheres can't be added
        if (c->bodies.count == c->MAX_NUM_BODIES || c->colliders.spheres.count == c->MAX_NUM_SPHERES) return NUDGE_INVALID_BODY_ID;
        body = c->bodies.count++;
    }

    BodyProperties* prop = &c->bodies.properties[body];
    Transform *xform = &c->bodies.transforms[body], *xform_collider = NULL;
    BodyInfo* info = &c->bodies.infos[body];BodyFilter* filter = &c->bodies.filters[body];BodyLayout* layout = &c->bodies.layouts[body];
    if (comOffset && comOffset[0]==0.f && comOffset[1]==0.f && comOffset[2]==0.f) comOffset = NULL;
    collider = c->colliders.spheres.count++;
    filter->flags = mass>0?BF_IS_DYNAMIC:(mass<0?BF_IS_KINEMATIC:BF_IS_STATIC);

    *xform = T ? (*T) : identity_transform; // transform
    xform->body = body; // body id
    memset(&c->bodies.momentum[body], 0, sizeof(c->bodies.momentum[body])); // no velocity/angular velocity
    memset(prop,0,sizeof(*prop));prop->friction = NUDGE_DEFAULT_FRICTION;prop->gravity[1]=NUDGE_DEFAULT_GRAVITY;   // reset mass/inertia/friction
    if (mass<0) mass=-mass;
    if (mass>0)   {prop->mass_inverse = 1.0f/mass;calculate_sphere_inertia_inverse(prop->inertia_inverse,mass,radius,comOffset);}
    c->bodies.idle_counters[body] = (filter->flags&BF_IS_DYNAMIC)?0:0xff;

    //memset(info,0,sizeof(BodyInfo));
    info->com_offset[0]=info->com_offset[1]=info->com_offset[2]=0.f;
    layout->num_boxes = 0;layout->num_spheres = 1;
    layout->first_box_index = -1;
    layout->first_sphere_index =  (int16_t) collider;    
    c->colliders.spheres.data[collider].radius = radius;
    xform_collider = &c->colliders.spheres.transforms[collider];
    *xform_collider = identity_transform;
    xform_collider->body = body;
    if (comOffset) {filter->flags|=BF_HAS_COM_OFFSET;for (int l=0;l<3;l++) {xform_collider->position[l]-=comOffset[l];info->com_offset[l]=comOffset[l];}}

    filter->collision_group = COLLISION_GROUP_DEFAULT;filter->collision_mask = COLLISION_GROUP_ALL;

    body_recalculate_bounding_box(c,body);  // new

    //log("%d) Added sphere [mass:%1.3f;radius=%1.3f;pos{%1.3f,%1.3f,%1.3f}]\n",body,mass,radius,T->position[0],T->position[1],T->position[2]);
    //assert(getSphereColliderId(c->bodies.count-1)==collider);
    return body;
}
unsigned add_sphere(context_t* c, float mass, float radius, const float* mMatrix16WithoutScaling, const float comOffset[3]) {
    if (!mMatrix16WithoutScaling)   return add_sphere(c,mass,radius,(const Transform*)NULL);
    else {Transform T;Mat4WithoutScalingToTransform(&T,mMatrix16WithoutScaling);return add_sphere(c,mass,radius,&T,comOffset);}
}

unsigned add_compound(context_t* c, float mass, float inertia[3], unsigned num_boxes, const float* hsizeTriplets, const Transform* boxOffsetTransforms, unsigned num_spheres, const float* radii, const Transform* sphereOffsetTransforms, const Transform* T, const float comOffset[3],float* centerMeshAndRetrieveOldCenter3Out)   {
    unsigned body = NUDGE_INVALID_BODY_ID;
    assert(num_boxes+num_spheres>0);
    assert(c->colliders.boxes.count+num_boxes<=c->MAX_NUM_BOXES);
    assert(c->colliders.spheres.count+num_spheres<=c->MAX_NUM_SPHERES);
    if (c->colliders.boxes.count+num_boxes>c->MAX_NUM_BOXES || c->colliders.spheres.count+num_spheres>c->MAX_NUM_SPHERES) return NUDGE_INVALID_BODY_ID;
    if (c->global_data.finalized_removed_bodies_count>0) {
        assert(c->global_data.finalized_removed_bodies_count<=c->global_data.removed_bodies_count);
        assert(sizeof(c->global_data.removed_bodies[0])==sizeof(body_type));
        body=c->global_data.removed_bodies[0];--c->global_data.finalized_removed_bodies_count;--c->global_data.removed_bodies_count;
        memmove(&c->global_data.removed_bodies[0],&c->global_data.removed_bodies[1],c->global_data.removed_bodies_count*sizeof(body_type));
        assert(body<c->bodies.count);
        const BodyLayout* bl = &c->bodies.layouts[body];
        assert(bl->first_box_index==-1);
        assert(bl->num_boxes==0);
        assert(bl->first_sphere_index==-1);
        assert(bl->num_spheres==0);
    }
    else {
        assert(c->bodies.count<c->MAX_NUM_BODIES);   // Further bodies can't be added (it should never happen, since: c->MAX_NUM_BODIES=c->MAX_NUM_BOXES+c->MAX_NUM_SPHERES)
        if (c->bodies.count == c->MAX_NUM_BODIES) return NUDGE_INVALID_BODY_ID;
        body = c->bodies.count++;
    }
    BodyProperties* prop = &c->bodies.properties[body];
    Transform *xform = &c->bodies.transforms[body];
    BodyInfo* info = &c->bodies.infos[body];BodyFilter* filter = &c->bodies.filters[body];BodyLayout* layout = &c->bodies.layouts[body];
    if (comOffset && comOffset[0]==0.f && comOffset[1]==0.f && comOffset[2]==0.f) comOffset = NULL;
    //memset(info,0,sizeof(BodyInfo));
    info->com_offset[0]=info->com_offset[1]=info->com_offset[2]=0.f;
    filter->flags = mass>0?BF_IS_DYNAMIC:(mass<0?BF_IS_KINEMATIC:BF_IS_STATIC);
    if (comOffset) filter->flags|=BF_HAS_COM_OFFSET;

    *xform = T ? (*T) : identity_transform; // transform
    xform->body = body; // body id
    memset(&c->bodies.momentum[body], 0, sizeof(c->bodies.momentum[body])); // no velocity/angular velocity
    memset(prop,0,sizeof(*prop));prop->friction = NUDGE_DEFAULT_FRICTION;prop->gravity[1]=NUDGE_DEFAULT_GRAVITY;   // reset mass/inertia/friction
    if (mass<0) mass=-mass;
    if (mass>0) prop->mass_inverse = 1.0f/mass;
    c->bodies.idle_counters[body] = (filter->flags&BF_IS_DYNAMIC)?0:0xff;

    for (unsigned i=0;i<num_boxes;i++)   {
        unsigned collider = c->colliders.boxes.count++;
        BoxCollider* boxCollider = &c->colliders.boxes.data[collider];
        Transform* xf = &c->colliders.boxes.transforms[collider];
        for (int j=0;j<3;j++) boxCollider->size[j] = hsizeTriplets[3*i+j];
        *xf = boxOffsetTransforms ? boxOffsetTransforms[i] : identity_transform;
        if (comOffset && !centerMeshAndRetrieveOldCenter3Out) {for (int l=0;l<3;l++) {xf->position[l]-=comOffset[l];info->com_offset[l]=comOffset[l];}}
        xf->body = body;
        if (i==0) layout->first_box_index = collider;
    }
    layout->num_boxes = num_boxes;

    for (unsigned i=0;i<num_spheres;i++)   {
        unsigned collider = c->colliders.spheres.count++;
        assert(collider<c->MAX_NUM_SPHERES && collider<c->colliders.spheres.count);
        Transform* xf = &c->colliders.spheres.transforms[collider];
        c->colliders.spheres.data[collider].radius = radii[i];
        *xf = sphereOffsetTransforms ? sphereOffsetTransforms[i] : identity_transform;
        if (comOffset && !centerMeshAndRetrieveOldCenter3Out) {for (int l=0;l<3;l++) {xf->position[l]-=comOffset[l];info->com_offset[l]=comOffset[l];}}
        xf->body = body;
        if (i==0) layout->first_sphere_index = collider;
    }
    layout->num_spheres = num_spheres;

    filter->collision_group = COLLISION_GROUP_DEFAULT;filter->collision_mask = COLLISION_GROUP_ALL;

    body_recalculate_bounding_box(c,body);  // new get aabb

    float aabb_he[3] = {info->aabb_half_extents[0],info->aabb_half_extents[1],info->aabb_half_extents[2]};
    if (centerMeshAndRetrieveOldCenter3Out) {
        centerMeshAndRetrieveOldCenter3Out[0]=info->aabb_center[0];
        centerMeshAndRetrieveOldCenter3Out[1]=info->aabb_center[1];
        centerMeshAndRetrieveOldCenter3Out[2]=info->aabb_center[2];
        float offset[3]; // aabb_center + com_offset
        // calculate total offset and correct info->aabb_min_max
        for (int i=0;i<3;i++) {
            offset[i]=centerMeshAndRetrieveOldCenter3Out[i]+(comOffset?comOffset[i]:0.f);
            info->aabb_center[i]-=centerMeshAndRetrieveOldCenter3Out[i];
        }
        // remove offset from each collider transform and set info->com_offset to comOffset
        for (unsigned i=0;i<layout->num_boxes;i++)   {
            Transform* xf = &c->colliders.boxes.transforms[layout->first_box_index+i];
            {for (int l=0;l<3;l++) {xf->position[l]-=offset[l];if (comOffset) info->com_offset[l]=comOffset[l];}}
        }
        for (unsigned i=0;i<layout->num_spheres;i++)   {
            Transform* xf = &c->colliders.spheres.transforms[layout->first_sphere_index+i];
            {for (int l=0;l<3;l++) {xf->position[l]-=offset[l];if (comOffset) info->com_offset[l]=comOffset[l];}}
        }
        // recalculate info->aabb_enlarged_radius
        info->aabb_enlarged_radius = 0;
        const float* t = info->aabb_half_extents;float s = t[0]*t[0] + t[1]*t[1] + t[2]*t[2];if (s>NM_EPSILON) info->aabb_enlarged_radius+=sqrtf(s);
        t = info->aabb_center;s = t[0]*t[0] + t[1]*t[1] + t[2]*t[2];if (s>NM_EPSILON) info->aabb_enlarged_radius+=sqrtf(s);
    }

    if (mass>0)    {
        float tmp[3];
        if (!inertia) {
            // now that we can calculate a per-body aabb, we can assign a box inertia as default
            calculate_box_inertia(tmp,mass,aabb_he[0],aabb_he[1],aabb_he[2],comOffset); // here we simply use comOffset (not the additional offset to recenter the mesh): is this the default option? Yes: comOffset should not depend on the input mesh aabb center
            inertia=tmp;
        }
        assert(inertia);
        if (inertia) {for (int i=0;i<3;i++)   prop->inertia_inverse[i] = inertia[i]!=0.f ? (1.0f / inertia[i]) : 0.f;}
    }

    //log("%d) Added compound [mass:%1.3f;pos{%1.3f,%1.3f,%1.3f};num_boxes=%u;num_spheres=%u]\n",body,mass,T->position[0],T->position[1],T->position[2],num_boxes,num_spheres);

    return body;
}
unsigned add_compound(context_t* c, float mass, float inertia[3], unsigned num_boxes, const float* hsizeTriplets, const float* boxOffsetMatrices16WithoutScaling, unsigned num_spheres, const float* radii, const float* sphereOffsetMatrices16WithoutScaling, const float* mMatrix16WithoutScaling, const float comOffset[3], float *centerMeshAndRetrieveOldCenter3Out)   {
    Arena arena = c->arena;
    Transform* boxTransforms = allocate_array<Transform>(&arena, num_boxes+num_spheres, 32);
    Transform* sphereTransforms = &boxTransforms[num_boxes];
    for (unsigned i=0;i<num_boxes;i++)      Mat4WithoutScalingToTransform(&boxTransforms[i],&boxOffsetMatrices16WithoutScaling[16*i]);
    for (unsigned i=0;i<num_spheres;i++)    Mat4WithoutScalingToTransform(&sphereTransforms[i],&sphereOffsetMatrices16WithoutScaling[16*i]);
    Transform T = identity_transform;
    if (mMatrix16WithoutScaling) Mat4WithoutScalingToTransform(&T,mMatrix16WithoutScaling);
    return add_compound(c,mass,inertia,num_boxes,hsizeTriplets,boxTransforms,num_spheres,radii,sphereTransforms,&T,comOffset,centerMeshAndRetrieveOldCenter3Out);
}

unsigned add_clone(context_t* c, unsigned body_to_clone, float mass, const Transform* T, float scale_factor, const float newComOffsetInPreScaledUnits[3]) {
    unsigned body = NUDGE_INVALID_BODY_ID;
    const unsigned srcbody = body_to_clone;
    assert(srcbody<c->bodies.count);
    assert(scale_factor!=0.f);
    const BodyLayout* srclayout = &c->bodies.layouts[srcbody];const uint16_t num_boxes = srclayout->num_boxes, num_spheres = srclayout->num_spheres;
    if (num_boxes>colliders_get_num_remaining_boxes(c) || num_spheres>colliders_get_num_remaining_spheres(c)) return NUDGE_INVALID_BODY_ID;
    assert(num_boxes+num_spheres>0);
    assert(c->colliders.boxes.count+num_boxes<=c->MAX_NUM_BOXES);
        assert(c->colliders.spheres.count+num_spheres<=c->MAX_NUM_SPHERES);
        if (c->colliders.boxes.count+num_boxes>c->MAX_NUM_BOXES || c->colliders.spheres.count+num_spheres>c->MAX_NUM_SPHERES) return NUDGE_INVALID_BODY_ID;
        if (c->global_data.finalized_removed_bodies_count>0) {
            assert(c->global_data.finalized_removed_bodies_count<=c->global_data.removed_bodies_count);
            assert(sizeof(c->global_data.removed_bodies[0])==sizeof(body_type));
            body=c->global_data.removed_bodies[0];--c->global_data.finalized_removed_bodies_count;--c->global_data.removed_bodies_count;
            memmove(&c->global_data.removed_bodies[0],&c->global_data.removed_bodies[1],c->global_data.removed_bodies_count*sizeof(body_type));
            assert(body<c->bodies.count);
            const BodyLayout* bl = &c->bodies.layouts[body];
            assert(bl->first_box_index==-1);
            assert(bl->num_boxes==0);
            assert(bl->first_sphere_index==-1);
            assert(bl->num_spheres==0);
        }
        else {
            assert(c->bodies.count<c->MAX_NUM_BODIES);   // Further bodies can't be added (it should never happen, since: c->MAX_NUM_BODIES=c->MAX_NUM_BOXES+c->MAX_NUM_SPHERES)
            if (c->bodies.count == c->MAX_NUM_BODIES) return NUDGE_INVALID_BODY_ID;
            body = c->bodies.count++;
        }
        const BodyProperties* srcprop = &c->bodies.properties[srcbody];BodyProperties* prop = &c->bodies.properties[body];
        const BodyInfo* srcinfo = &c->bodies.infos[srcbody];BodyInfo* info = &c->bodies.infos[body];
        const BodyFilter* srcfilter = &c->bodies.filters[srcbody];BodyFilter* filter = &c->bodies.filters[body];
        BodyLayout* layout = &c->bodies.layouts[body];
        Transform *xform = &c->bodies.transforms[body];
        *xform = T ? (*T) : identity_transform; // transform
        xform->body = body; // body id
        memset(&c->bodies.momentum[body], 0, sizeof(c->bodies.momentum[body])); // no velocity/angular velocity
        float com_delta[3] = {0.f,0.f,0.f};if (newComOffsetInPreScaledUnits) {for (int k=0;k<3;k++) com_delta[k] = newComOffsetInPreScaledUnits[k]-srcinfo->com_offset[k];}
        if (scale_factor<0.f) scale_factor=(srcinfo->aabb_half_extents[1]>=0.f)?(-scale_factor/srcinfo->aabb_half_extents[1]):-scale_factor;
        assert(scale_factor>0.f);

        // clone almost everything (with some flags/scaling/com_offset adjustments)
        *prop=*srcprop;
        *filter = *srcfilter;filter->flags&=~(BF_IS_STATIC_OR_KINEMATIC_OR_DYNAMIC|BF_IS_DISABLED_OR_REMOVED|BF_HAS_COM_OFFSET);
        filter->flags|=(mass>0.f)?BF_IS_DYNAMIC:((mass<0.f)?BF_IS_KINEMATIC:BF_IS_STATIC);if (mass<0.f) mass=-mass;
        c->bodies.idle_counters[body]=(filter->flags&BF_IS_DYNAMIC)?0:0xFF;
        if (num_boxes) {
            assert(srclayout->first_box_index>=0 && (uint16_t)srclayout->first_box_index+num_boxes<=c->colliders.boxes.count);
            const Transform* srcT = &c->colliders.boxes.transforms[srclayout->first_box_index];
            const BoxCollider* srcC = &c->colliders.boxes.data[srclayout->first_box_index];
            layout->first_box_index = c->colliders.boxes.count;layout->num_boxes = num_boxes;c->colliders.boxes.count+=num_boxes;assert(c->colliders.boxes.count<=c->MAX_NUM_BOXES);
            Transform* T = &c->colliders.boxes.transforms[layout->first_box_index];
            BoxCollider* C = &c->colliders.boxes.data[layout->first_box_index];
            for (uint16_t i=0;i<num_boxes;i++) {
                T[i]=srcT[i];T[i].body=body;C[i]=srcC[i];
                for (int k=0;k<3;k++) {
                    T[i].p[k]=scale_factor*(T[i].p[k]-com_delta[k]);
                    C[i].size[k]*=scale_factor;
                }
            }
        }
        if (num_spheres) {
            assert(srclayout->first_sphere_index>=0 && (uint16_t)srclayout->first_sphere_index+num_spheres<=c->colliders.spheres.count);
            const Transform* srcT = &c->colliders.spheres.transforms[srclayout->first_sphere_index];
            const SphereCollider* srcC = &c->colliders.spheres.data[srclayout->first_sphere_index];
            layout->first_sphere_index = c->colliders.spheres.count;layout->num_spheres = num_spheres;c->colliders.spheres.count+=num_spheres;assert(c->colliders.spheres.count<=c->MAX_NUM_SPHERES);
            Transform* T = &c->colliders.spheres.transforms[layout->first_sphere_index];
            SphereCollider* C = &c->colliders.spheres.data[layout->first_sphere_index];
            for (uint16_t i=0;i<num_spheres;i++) {
                T[i]=srcT[i];T[i].body=body;C[i]=srcC[i];C[i].radius*=scale_factor;
                for (int k=0;k<3;k++) T[i].p[k]=scale_factor*(T[i].p[k]-com_delta[k]);
            }
        }
        // com offset adjustments
        if (newComOffsetInPreScaledUnits) {
            if (newComOffsetInPreScaledUnits[0]==0.f && newComOffsetInPreScaledUnits[1]==0.f && newComOffsetInPreScaledUnits[2]==0.f) memset(info->com_offset,0,3*sizeof(float));
            else {
                for (int k=0;k<3;k++) info->com_offset[k]=scale_factor*newComOffsetInPreScaledUnits[k];
                filter->flags|=BF_HAS_COM_OFFSET;
            }
        }
        else {
            // we must leave (and scale) srcinfo->com_offset
            if (srcinfo->com_offset[0]==0.f && srcinfo->com_offset[1]==0.f && srcinfo->com_offset[2]==0.f) {
                assert(!(srcfilter->flags&BF_HAS_COM_OFFSET));
                memset(info->com_offset,0,3*sizeof(float));
            }
            else {
                assert(srcfilter->flags&BF_HAS_COM_OFFSET);
                for (int k=0;k<3;k++) info->com_offset[k]=scale_factor*srcinfo->com_offset[k];
                filter->flags|=BF_HAS_COM_OFFSET;
            }
        }
        // recalculate aabb
        body_recalculate_bounding_box(c,body);
        // scale mass and local_inertia
        assert(mass>=0);assert(srcprop->mass_inverse>=0.f);
        if (mass>0.f)   {
            if (srcprop->mass_inverse>0.f) {
                // const float sc = scale_factor*scale_factor*mass*srcprop->mass_inverse;
                // I'xx = Ixx*sc => (1/I'xx) = (1/Ixx)*(1/sc) => (1/I'xx) = (1/Ixx)/sc => (1/I'xx)=(1/Ixx);(1/I'xx)/=sc; // valid for all the 3 components
                for (int k=0;k<3;k++) prop->inertia_inverse[k]/=scale_factor*scale_factor*mass*srcprop->mass_inverse;
                prop->mass_inverse=1.f/mass;
            }
            else {
                prop->mass_inverse=0.f;
                calculate_box_inertia_inverse(prop->inertia_inverse,mass,info->aabb_half_extents[0],info->aabb_half_extents[1],info->aabb_half_extents[2],info->com_offset);
            }
        }

        return body;
}
unsigned add_clone(context_t* c, unsigned body_to_clone, float mass, const float* mMatrix16WithoutScaling, float scale_factor, const float newComOffsetInPreScaledUnits[3]) {
    Transform T;Mat4WithoutScalingToTransform(&T,mMatrix16WithoutScaling);return add_clone(c,body_to_clone,mass,&T,scale_factor,newComOffsetInPreScaledUnits);
}


namespace extra {

#ifndef NUDGE_EXTRA_RADIUS_ON_BOX_SHRINK
#   define  NUDGE_EXTRA_RADIUS_ON_BOX_SHRINK (1.f/3.5f)
#endif

unsigned add_compound_prism(context_t* c, float mass, float radius, float hsize, unsigned num_lateral_faces, const Transform* T, AxisEnum axis, const float comOffset[3])  {
    if (num_lateral_faces==0) num_lateral_faces=8;
    if (num_lateral_faces<4) return NUDGE_INVALID_BODY_ID;
    if (num_lateral_faces==4) return add_box(c,mass,axis==AXIS_X?hsize:radius,axis==AXIS_Y?hsize:radius,axis==AXIS_Z?hsize:radius,T,comOffset);
    const int use_half_number_of_boxes = ((num_lateral_faces%2)==0);
    if (!use_half_number_of_boxes)  return add_compound_hollow_cylinder(c,mass,0.f,radius,hsize,T,axis,num_lateral_faces,comOffset);
    const unsigned num_boxes = num_lateral_faces/2;
    Arena arena = c->arena;assert(arena.size>num_boxes*(3*sizeof(float)+sizeof(Transform))+64);
    const float axisv[3] = {(axis==AXIS_X)?1.f:0.f,(axis==AXIS_Y)?1.f:0.f,(axis==AXIS_Z)?1.f:0.f};
    float* box_hsizes = NULL;Transform* boxT=NULL;
    const float hsz = radius*tanf(M_PI/(float)num_lateral_faces);
    const float hln = radius;
    int axisi[3] = {0,1,2};
    if (axis==AXIS_X) {axisi[0]=2;axisi[1]=0;axisi[2]=1;}
    else if (axis==AXIS_Z) {axisi[0]=1;axisi[1]=2;axisi[2]=0;}
    box_hsizes = allocate_array<float>(&arena, num_boxes*3, 32);
    boxT = allocate_array<Transform>(&arena, num_boxes, 32);
    for (unsigned i=0;i<num_boxes;i++) {
        float* hs = &box_hsizes[3*i];Transform* t = &boxT[i];*t=identity_transform;
        const float angle = (float)i*M_PI/(float)num_boxes;
        nm_QuatFromAngleAxis(t->q,-angle,axisv[0],axisv[1],axisv[2]);
        hs[axisi[0]]=hln;hs[axisi[1]]=hsize;hs[axisi[2]]=hsz;
    }
    float inertia[3]; calculate_cylinder_inertia(inertia,mass,radius,hsize,axis,comOffset);float stripped_center[3];
    return add_compound(c,mass,inertia,num_boxes,box_hsizes,boxT,0,NULL,NULL,T,comOffset,stripped_center);
}

unsigned add_compound_cylinder(context_t* c,float mass,float radius,float hsize, const Transform* T,AxisEnum axis,unsigned num_boxes,unsigned num_spheres,const float comOffset[3],float box_lateral_side_shrinking)  {
    const bool is_short_cylinder = (radius>=hsize);
    if (num_boxes==0 && num_spheres==0) {
        if (is_short_cylinder) {num_boxes=8;num_spheres=0;}
        else {num_boxes=1;num_spheres=3;}
    }
    if (is_short_cylinder) num_spheres = 0;
    if (box_lateral_side_shrinking<0.f) {
        if (num_spheres==0) box_lateral_side_shrinking=(num_boxes<=1)?0.f:(1.f-1.f/1.41f);
        else box_lateral_side_shrinking=NUDGE_EXTRA_RADIUS_ON_BOX_SHRINK;
    }
    Arena arena = c->arena;assert(arena.size>num_boxes*(3*sizeof(float)+sizeof(Transform))+num_spheres*(1*sizeof(float)+sizeof(Transform))+128);
    const float axisv[3] = {(axis==AXIS_X)?1.f:0.f,(axis==AXIS_Y)?1.f:0.f,(axis==AXIS_Z)?1.f:0.f};
    float* box_hsizes = NULL;Transform* boxT=NULL;
    if (num_boxes>0) {
        const float offset = radius*box_lateral_side_shrinking;    // box sides is radius-offset
        const float box_size = radius-offset;
        float angle = M_PI*0.5f/num_boxes;
        box_hsizes = allocate_array<float>(&arena, num_boxes*3, 32);
        boxT = allocate_array<Transform>(&arena, num_boxes, 32);
        for (unsigned i=0;i<num_boxes;i++) {
            float* hs = &box_hsizes[3*i];Transform* t = &boxT[i];*t=identity_transform;
            hs[0]=hs[1]=hs[2]=box_size;hs[axis]=hsize;
            nm_QuatFromAngleAxis(t->q,angle*i,axisv[0],axisv[1],axisv[2]);
        }
    }
    float* sphere_radii = NULL;Transform* sphereT = NULL;
    if (num_spheres>0)  {
        sphere_radii = allocate_array<float>(&arena, num_boxes*1, 32);
        sphereT = allocate_array<Transform>(&arena, num_boxes, 32);
        for (unsigned i=0;i<num_spheres;i++) {sphere_radii[i]=radius;sphereT[i]=identity_transform;}
        if (num_spheres>=2) {
            Transform* t = &sphereT[0];t->p[axis]=-hsize+radius;
            t = &sphereT[1];t->p[axis]=hsize-radius;
            if (num_spheres>2) {
                // |---------------|----------------| // 1 => 1/2
                // |---------|           |----------| // 2 => 1/3
                // |-------|-------|--------|-------| // 3 => 1/4
                const float dist = (2.f*hsize)/(float)(num_spheres+1);
                for (unsigned i=2;i<num_spheres;i++) {sphereT[i].p[axis]=-hsize+dist*i;}
            }
        }
        /*else if (mass==0) {
            assert(num_spheres==1);
            sphereT[i].p[axis]=-hsize+radius; // if it's static maybe we prefer a single sphere at the bottom?
        }*/
    }
    float inertia[3]; calculate_cylinder_inertia(inertia,mass,radius,hsize,axis,comOffset);float stripped_center[3];
    return add_compound(c,mass,inertia,num_boxes,box_hsizes,boxT,num_spheres,sphere_radii,sphereT,T,comOffset,stripped_center);
}
unsigned add_compound_capsule(context_t* c, float mass, float radius, float hsize, const Transform* T, AxisEnum axis, unsigned num_boxes, unsigned num_spheres, const float comOffset[3], float box_lateral_side_shrinking)    {
    if (num_boxes==0 && num_spheres==0) {num_boxes=1;num_spheres=3;}
    assert(num_spheres>=2);
    if (box_lateral_side_shrinking<0.f) box_lateral_side_shrinking=NUDGE_EXTRA_RADIUS_ON_BOX_SHRINK;
    Arena arena = c->arena;assert(arena.size>num_boxes*(3*sizeof(float)+sizeof(Transform))+num_spheres*(1*sizeof(float)+sizeof(Transform))+128);
    const float axisv[3] = {(axis==AXIS_X)?1.f:0.f,(axis==AXIS_Y)?1.f:0.f,(axis==AXIS_Z)?1.f:0.f};
    float* box_hsizes = NULL;Transform* boxT=NULL;
    if (num_boxes>0) {
        const float offset = radius*box_lateral_side_shrinking;    // box sides is radius-offset
        float angle = M_PI*0.5f/num_boxes;
        box_hsizes = allocate_array<float>(&arena, num_boxes*3, 32);
        boxT = allocate_array<Transform>(&arena, num_boxes, 32);
        for (unsigned i=0;i<num_boxes;i++) {
            float* hs = &box_hsizes[3*i];Transform* t = &boxT[i];*t=identity_transform;
            hs[0]=hs[1]=hs[2]=radius-offset;hs[axis]=hsize;
            nm_QuatFromAngleAxis(t->q,angle*i,axisv[0],axisv[1],axisv[2]);
        }
    }
    float* sphere_radii = NULL;Transform* sphereT = NULL;
    if (num_spheres>0)  {
        sphere_radii = allocate_array<float>(&arena, num_boxes*1, 32);
        sphereT = allocate_array<Transform>(&arena, num_boxes, 32);
        for (unsigned i=0;i<num_spheres;i++) {sphere_radii[i]=radius;sphereT[i]=identity_transform;}
        Transform* t = &sphereT[0];t->p[axis]=-hsize;
        t = &sphereT[1];t->p[axis]=hsize;
        if (num_spheres>2) {
            // |---------------|----------------| // 1 => 1/2
            // |---------|           |----------| // 2 => 1/3
            // |-------|-------|--------|-------| // 3 => 1/4
            const float dist = (2.f*(hsize+radius))/(float)(num_spheres+1);
            for (unsigned i=2;i<num_spheres;i++) {sphereT[i].p[axis]=-hsize-radius+dist*i;}
        }
    }
    float inertia[3]; calculate_capsule_inertia(inertia,mass,radius,hsize,axis,comOffset);float stripped_center[3];
    return add_compound(c,mass,inertia,num_boxes,box_hsizes,boxT,num_spheres,sphere_radii,sphereT,T,comOffset,stripped_center);
}
unsigned add_compound_hollow_cylinder(context_t* c,float mass,float min_radius,float max_radius,float hsize, const Transform* T,AxisEnum axis,unsigned num_boxes,const float comOffset[3])  {
    const unsigned num_spheres = 0;assert(min_radius<max_radius);
    const float radius = (max_radius+min_radius)*0.5f,inner_radius=(max_radius-min_radius)*0.5f;
    Arena arena = c->arena;assert(arena.size>num_boxes*(3*sizeof(float)+sizeof(Transform))+num_spheres*(1*sizeof(float)+sizeof(Transform))+128);
    const float axisv[3] = {(axis==AXIS_X)?1.f:0.f,(axis==AXIS_Y)?1.f:0.f,(axis==AXIS_Z)?1.f:0.f};
    int axisi[3] = {0,1,2};
    if (axis==AXIS_X) {axisi[0]=2;axisi[1]=0;axisi[2]=1;}
    else if (axis==AXIS_Z) {axisi[0]=1;axisi[1]=2;axisi[2]=0;}
    float* box_hsizes = NULL;Transform* boxT=NULL;
    if (num_boxes==0) num_boxes=8;
    if (num_boxes>0) {
        const float box_length = max_radius*tanf(M_PI/(float)num_boxes);
        box_hsizes = allocate_array<float>(&arena, num_boxes*3, 32);
        boxT = allocate_array<Transform>(&arena, num_boxes, 32);
        for (unsigned i=0;i<num_boxes;i++) {
            float* hs = &box_hsizes[3*i];Transform* t = &boxT[i];*t=identity_transform;
            hs[axisi[0]]=box_length;hs[axisi[1]]=hsize;hs[axisi[2]]=inner_radius;
            const float angle = (float)i*2.f*M_PI/(float)num_boxes;
            const float sinAngle = sinf(angle), cosAngle = cosf(angle);
            nm_QuatFromAngleAxis(t->q,-angle,axisv[0],axisv[1],axisv[2]);
            t->p[axisi[0]]=(radius)*sinAngle;t->p[axisi[1]]=0.f;t->p[axisi[2]]=-(radius)*cosAngle;
        }
    }
    float* sphere_radii = NULL;Transform* sphereT = NULL;
    /*if (num_spheres>0)  {
        sphere_radii = allocate_array<float>(&arena, num_boxes*1, 32);
        sphereT = allocate_array<Transform>(&arena, num_boxes, 32);
        for (unsigned i=0;i<num_spheres;i++) {sphere_radii[i]=radius;sphereT[i]=identity_transform;}
        if (num_spheres>=2) {
            Transform* t = &sphereT[0];t->p[axis]=-hsize+radius;
            t = &sphereT[1];t->p[axis]=hsize-radius;
            if (num_spheres>2) {
                // |---------------|----------------| // 1 => 1/2
                // |---------|           |----------| // 2 => 1/3
                // |-------|-------|--------|-------| // 3 => 1/4
                const float dist = (2.f*hsize)/(float)(num_spheres+1);
                for (unsigned i=2;i<num_spheres;i++) {sphereT[i].p[axis]=-hsize+dist*i;}
            }
        }
    }*/
    float inertia[3]; calculate_hollow_cylinder_inertia(inertia,mass,max_radius,min_radius,hsize,axis,comOffset);float stripped_center[3];
    return add_compound(c,mass,inertia,num_boxes,box_hsizes,boxT,num_spheres,sphere_radii,sphereT,T,comOffset,stripped_center);
}
unsigned add_compound_torus(context_t* c,float mass,float radius,float inner_radius, const Transform* T,AxisEnum axis,unsigned num_boxes,const float comOffset[3])   {
    const unsigned num_spheres = 0;assert(inner_radius<=radius);
    Arena arena = c->arena;assert(arena.size>num_boxes*(3*sizeof(float)+sizeof(Transform))+num_spheres*(1*sizeof(float)+sizeof(Transform))+128);
    const float axisv[3] = {(axis==AXIS_X)?1.f:0.f,(axis==AXIS_Y)?1.f:0.f,(axis==AXIS_Z)?1.f:0.f};
    int axisi[3] = {0,1,2};
    if (axis==AXIS_X) {axisi[0]=2;axisi[1]=0;axisi[2]=1;}
    else if (axis==AXIS_Z) {axisi[0]=1;axisi[1]=2;axisi[2]=0;}
    float* box_hsizes = NULL;Transform* boxT=NULL;
    if (num_boxes==0) num_boxes=8;
    if (num_boxes>0) {
        const float box_length = (radius+inner_radius)*tanf(M_PI/(float)num_boxes);
        box_hsizes = allocate_array<float>(&arena, num_boxes*3, 32);
        boxT = allocate_array<Transform>(&arena, num_boxes, 32);
        for (unsigned i=0;i<num_boxes;i++) {
            float* hs = &box_hsizes[3*i];Transform* t = &boxT[i];*t=identity_transform;
            hs[axisi[0]]=box_length;hs[axisi[1]]=inner_radius;hs[axisi[2]]=inner_radius;
            const float angle = (float)i*2.f*M_PI/(float)num_boxes;
            const float sinAngle = sinf(angle), cosAngle = cosf(angle);
            nm_QuatFromAngleAxis(t->q,-angle,axisv[0],axisv[1],axisv[2]);
            t->p[axisi[0]]=(radius)*sinAngle;t->p[axisi[1]]=0.f;t->p[axisi[2]]=-(radius)*cosAngle;
        }
    }
    float* sphere_radii = NULL;Transform* sphereT = NULL;
    /*if (num_spheres>0)  {
        sphere_radii = allocate_array<float>(&arena, num_boxes*1, 32);
        sphereT = allocate_array<Transform>(&arena, num_boxes, 32);
        for (unsigned i=0;i<num_spheres;i++) {sphere_radii[i]=radius;sphereT[i]=identity_transform;}
        if (num_spheres>=2) {
            Transform* t = &sphereT[0];t->p[axis]=-hsize+radius;
            t = &sphereT[1];t->p[axis]=hsize-radius;
            if (num_spheres>2) {
                // |---------------|----------------| // 1 => 1/2
                // |---------|           |----------| // 2 => 1/3
                // |-------|-------|--------|-------| // 3 => 1/4
                const float dist = (2.f*hsize)/(float)(num_spheres+1);
                for (unsigned i=2;i<num_spheres;i++) {sphereT[i].p[axis]=-hsize+dist*i;}
            }
        }
    }*/
    float inertia[3]; calculate_torus_inertia(inertia,mass,radius,inner_radius,axis,comOffset);float stripped_center[3];
    return add_compound(c,mass,inertia,num_boxes,box_hsizes,boxT,num_spheres,sphere_radii,sphereT,T,comOffset,stripped_center);
}
unsigned add_compound_cone(context_t* c, float mass, float radius, float hheight, const Transform* T, AxisEnum axis, unsigned num_boxes, unsigned num_spheres, const float comOffset[3])    {
    if (num_boxes==0) num_boxes=4;
    if (num_spheres==0) num_spheres=3;
    Arena arena = c->arena;assert(arena.size>num_boxes*(3*sizeof(float)+sizeof(Transform))+num_spheres*(1*sizeof(float)+sizeof(Transform))+128);
    const float axisv[3] = {(axis==AXIS_X)?1.f:0.f,(axis==AXIS_Y)?1.f:0.f,(axis==AXIS_Z)?1.f:0.f};
    const float R=radius,HH=hheight,H=hheight*2.f,theta=atanf(R/H);//,phi=M_PI*0.5f-theta;
    float* box_hsizes = NULL;Transform* boxT=NULL;
    if (num_boxes>0) {
        box_hsizes = allocate_array<float>(&arena, num_boxes*3, 32);
        boxT = allocate_array<Transform>(&arena, num_boxes, 32);for (unsigned i=0;i<num_boxes;i++) boxT[i]=identity_transform;
        if (num_boxes>0) {
            unsigned num_group_boxes[4] = {};
            if (num_boxes>3) num_group_boxes[3]=1;
            if (num_boxes>5) {num_group_boxes[1]=2;}
            else if (num_boxes>4) {num_group_boxes[1]=1;}
            if (num_spheres>=3 && num_group_boxes[1]==1) {num_group_boxes[3]=0;num_group_boxes[1]=2;}
            num_group_boxes[0]=num_boxes-num_group_boxes[1]-num_group_boxes[2]-num_group_boxes[3];
            //log("%u+%u+%u+%u=%u\n",num_group_boxes[0],num_group_boxes[1],num_group_boxes[2],num_group_boxes[3],num_boxes);
            const float h_fracs[4]={0.1f,0.2f,0.7f,0.85f};
            unsigned num_box_offset=0;
            for (unsigned j=0;j<4;j++) {
                const unsigned group_boxes = num_group_boxes[j];if (group_boxes==0) continue;;
                const float dh = h_fracs[j]*HH,dr = R*(HH-dh)/(HH*1.41f);
                float angle = M_PI*0.5f/group_boxes, angle_offset=M_PI*0.25f*j;
                for (unsigned i=0;i<group_boxes;i++) {
                    float* hs = &box_hsizes[3*(i+num_box_offset)];Transform* t = &boxT[i+num_box_offset];*t=identity_transform;
                    t->p[axis]=-HH+dh;
                    hs[0]=hs[1]=hs[2]=dr;hs[axis]=dh;
                    nm_QuatFromAngleAxis(t->q,angle_offset+angle*i,axisv[0],axisv[1],axisv[2]);
                }
                num_box_offset+=group_boxes;
            }
        }
    }
    float* sphere_radii = NULL;Transform* sphereT = NULL;
    if (num_spheres>0)  {
        sphere_radii = allocate_array<float>(&arena, num_spheres*1, 32);
        sphereT = allocate_array<Transform>(&arena, num_spheres, 32);
        for (unsigned i=0;i<num_spheres;i++) sphereT[i]=identity_transform;
        const float sin_theta = sinf(theta);
        const float r = H*sin_theta/(sin_theta+1);sphere_radii[0]=r;sphereT[0].p[axis]=-HH+r;    // bigger sphere
        if (num_spheres>1)  {
            const unsigned remaining_spheres = num_spheres-1;
            const float min_rad=0.2f*r;
            const float max_rad=num_spheres<=3?0.45f*r:(num_spheres>=6?0.85f*r:(0.45f*r+((0.85f*r-0.45f*r)*(num_spheres-3))/2));
            for (unsigned i=0;i<remaining_spheres;i++) {
                const float rtop = remaining_spheres==1?min_rad:(min_rad+((max_rad-min_rad)*i)/(remaining_spheres-1));//0.65f*r-0.2f*r*((float)(i+1)/(float)remaining_spheres);
                sphere_radii[i+1]=rtop;
                sphereT[i+1].p[axis]=HH-rtop/sinf(theta);
            }
        }
    }
    float inertia[3]; calculate_cone_inertia(inertia,mass,radius,hheight,axis,comOffset);float stripped_center[3];
    return add_compound(c,mass,inertia,num_boxes,box_hsizes,boxT,num_spheres,sphere_radii,sphereT,T,comOffset,stripped_center);
}
unsigned add_compound_prism(context_t* c, float mass, float radius, float hsize, unsigned num_lateral_faces, const float* mMatrix16WithoutScaling, AxisEnum axis, const float comOffset[3]) {
    if (!mMatrix16WithoutScaling)   return add_compound_prism(c,mass,radius,hsize,num_lateral_faces,(const Transform*)NULL);
    else {Transform T;Mat4WithoutScalingToTransform(&T,mMatrix16WithoutScaling);return add_compound_prism(c,mass,radius,hsize,num_lateral_faces,&T,axis,comOffset);}
}
unsigned add_compound_cylinder(context_t* c, float mass, float radius, float hsize, const float* mMatrix16WithoutScaling, AxisEnum axis, unsigned num_boxes, unsigned num_spheres, const float comOffset[3], float box_lateral_side_shrinking) {
    if (!mMatrix16WithoutScaling)   return add_compound_cylinder(c,mass,radius,hsize,(const Transform*)NULL);
    else {Transform T;Mat4WithoutScalingToTransform(&T,mMatrix16WithoutScaling);return add_compound_cylinder(c,mass,radius,hsize,&T,axis,num_boxes,num_spheres,comOffset,box_lateral_side_shrinking);}
}
unsigned add_compound_capsule(context_t* c, float mass, float radius, float hsize, const float* mMatrix16WithoutScaling, AxisEnum axis, unsigned num_boxes, unsigned num_spheres, const float comOffset[3], float box_lateral_side_shrinking)    {
    if (!mMatrix16WithoutScaling)   return add_compound_capsule(c,mass,radius,hsize,(const Transform*)NULL);
    else {Transform T;Mat4WithoutScalingToTransform(&T,mMatrix16WithoutScaling);return add_compound_capsule(c,mass,radius,hsize,&T,axis,num_boxes,num_spheres,comOffset,box_lateral_side_shrinking);}
}
unsigned add_compound_hollow_cylinder(context_t* c,float mass,float min_radius,float max_radius,float hsize,const float* mMatrix16WithoutScaling,AxisEnum axis,unsigned num_boxes,const float comOffset[3])    {
    if (!mMatrix16WithoutScaling)   return add_compound_hollow_cylinder(c,mass,min_radius,max_radius,hsize,(const Transform*)NULL);
    else {Transform T;Mat4WithoutScalingToTransform(&T,mMatrix16WithoutScaling);return add_compound_hollow_cylinder(c,mass,min_radius,max_radius,hsize,&T,axis,num_boxes,comOffset);}
}
unsigned add_compound_torus(context_t* c,float mass,float radius,float inner_radius, const float* mMatrix16WithoutScaling,AxisEnum axis,unsigned num_boxes,const float comOffset[3]) {
    if (!mMatrix16WithoutScaling)   return add_compound_torus(c,mass,radius,inner_radius,(const Transform*)NULL);
    else {Transform T;Mat4WithoutScalingToTransform(&T,mMatrix16WithoutScaling);return add_compound_torus(c,mass,radius,inner_radius,&T,axis,num_boxes,comOffset);}
}
unsigned add_compound_cone(context_t* c, float mass, float radius, float hheight, const float* mMatrix16WithoutScaling, AxisEnum axis, unsigned num_boxes, unsigned num_spheres, const float comOffset[3])    {
    if (!mMatrix16WithoutScaling)   return add_compound_cone(c,mass,radius,hheight,(const Transform*)NULL);
    else {Transform T;Mat4WithoutScalingToTransform(&T,mMatrix16WithoutScaling);return add_compound_cone(c,mass,radius,hheight,&T,axis,num_boxes,num_spheres,comOffset);}
}



} // namespace extra
// ------------
void body_recalculate_bounding_box(context_t* c,uint32_t body) {
    assert(c && body<c->bodies.count);
    float aabb_min[3]={0,0,0},aabb_max[3]={0,0,0};
    const BodyLayout* L = &c->bodies.layouts[body];
    if (L->num_spheres>0) {
        assert(L->first_sphere_index>=0 && (uint16_t)L->first_sphere_index+L->num_spheres<=c->colliders.spheres.count);
        const SphereCollider* S = &c->colliders.spheres.data[L->first_sphere_index];
        const Transform* T = &c->colliders.spheres.transforms[L->first_sphere_index];
        for (int i=0;i<3;i++) {aabb_min[i]=T[0].p[i]-S[0].radius;aabb_max[i]=T[0].p[i]+S[0].radius;} // assign first aabb_min_max
        // process the remaining spheres and enlarge aabb_min_max
        for (int j=1;j<L->num_spheres;j++) {
            const Transform* t = &T[j];const float r = S[j].radius;
            for (int i=0;i<3;i++) {
                if (aabb_min[i]>t->p[i]-r) aabb_min[i]=t->p[i]-r;
                if (aabb_max[i]<t->p[i]+r) aabb_max[i]=t->p[i]+r;   // 'else' at the beginning is wrong!
            }
        }
    }
    if (L->num_boxes>0) {
        assert(L->first_box_index>=0 && (uint16_t)L->first_box_index+L->num_boxes<=c->colliders.boxes.count);
        const BoxCollider* B = &c->colliders.boxes.data[L->first_box_index];
        const Transform* T = &c->colliders.boxes.transforms[L->first_box_index];
        for (int j=0;j<L->num_boxes;j++) {
            const float* hs = &B[j].size[0];const Transform* t = &T[j];
            if (t->q[0]==0.f && t->q[1]==0.f && t->q[2]==0.f && t->q[3]==1.f) {
                // fast code path
                if (j==0 && L->num_spheres==0) {
                    for (int i=0;i<3;i++) {aabb_min[i]=t->p[i]-hs[i];aabb_max[i]=t->p[i]+hs[i];} // assign first aabb_min_max
                    continue;
                }
                // enlarge aabb_min_max with t and hs
                for (int i=0;i<3;i++) {
                    if (aabb_min[i]>t->p[i]-hs[i]) aabb_min[i]=t->p[i]-hs[i];
                    if (aabb_max[i]<t->p[i]+hs[i]) aabb_max[i]=t->p[i]+hs[i];   // 'else' at the beginning is wrong!
                }
                continue;
            }
            else {
                // slow code path
                float m[9];nm_Mat3FromQuat(m,t->q); // rotation matrix of t->q
                // calculate result based on m[9], t->p[] and hs[]
                for (int i=0;i<3;i++)   {
                    float vmin,vmax;    // min and max of the (i) component of the aabb of the j-th box
                    const float hd=fabsf(m[i]*hs[0])+fabsf(m[3+i]*hs[1])+fabsf(m[6+i]*hs[2]);
                    vmin = vmax = t->p[i]; vmin-= hd;vmax+= hd;
                    if (j==0 && L->num_spheres==0) {
                        aabb_min[i]=vmin;aabb_max[i]=vmax; // assign first aabb_min_max
                        continue;
                    }
                    // enlarge (aabb_min_max) with vmin and vmax
                    if (aabb_min[i]>vmin) aabb_min[i]=vmin;
                    if (aabb_max[i]<vmax) aabb_max[i]=vmax;    // 'else' at the beginning is wrong!
                }
            }
        }
    }
    // convert  aabb_min/aabb_max to aabb_center/aabb_extents
    const int stripComOffset=0;
    BodyInfo* info = &c->bodies.infos[body];
    for (int i=0;i<3;i++) {
        info->aabb_center[i]= (aabb_max[i]+aabb_min[i])*0.5f;
        info->aabb_half_extents[i]=(aabb_max[i]-aabb_min[i])*0.5f;
        if (stripComOffset)  info->aabb_center[i]+=info->com_offset[i];
    }
    // calculate info->aabb_enlarged_radius
    info->aabb_enlarged_radius = 0;
    const float* t = info->aabb_half_extents;float s = t[0]*t[0] + t[1]*t[1] + t[2]*t[2];if (s>NM_EPSILON) info->aabb_enlarged_radius+=sqrtf(s);
    t = info->aabb_center;s = t[0]*t[0] + t[1]*t[1] + t[2]*t[2];if (s>NM_EPSILON) info->aabb_enlarged_radius+=sqrtf(s);
}

void body_change_motion_state(nudge::context_t* c,unsigned body,nudge::FlagMask new_motion_state,float mass_fallback)  {
    using namespace nudge;assert(c && body<c->bodies.count);
    new_motion_state&=BF_IS_STATIC_OR_KINEMATIC_OR_DYNAMIC;assert(new_motion_state);    // wrong input argument
    BodyFilter* bf = &c->bodies.filters[body];if (bf->flags&new_motion_state) return;
    BodyProperties* bp = &c->bodies.properties[body];BodyMomentum* bm = &c->bodies.momentum[body];
    if (new_motion_state&BF_IS_DYNAMIC) {
        if (bp->mass_inverse!=0) {assert(bp->mass_inverse>0.f);/*if (bp->mass_inverse<0) {bp->mass_inverse=-bp->mass_inverse;mass_fallback = 1.f/bp->mass_inverse;}*/}
        else {if (mass_fallback<0.f) {mass_fallback=-mass_fallback;} bp->mass_inverse=1.f/mass_fallback;}
        if (bp->inertia_inverse[0]==0.f && bp->inertia_inverse[1]==0.f && bp->inertia_inverse[2]==0.f) {BodyInfo* bi = &c->bodies.infos[body];float* he=bi->aabb_half_extents;calculate_box_inertia_inverse(bp->inertia_inverse,mass_fallback,he[0],he[1],he[2],bi->com_offset);}
        else {assert(!(bp->inertia_inverse[0]<0.f) && !(bp->inertia_inverse[1]<0.f) && !(bp->inertia_inverse[2]<0.f));
            /*for (int k=0;k<3;k++) {if (bp->inertia_inverse[k]<0.f) bp->inertia_inverse[k]=-bp->inertia_inverse[k];}*/
        }
    }
    memset(bm->velocity,0,3*sizeof(bm->velocity));memset(bm->angular_velocity,0,3*sizeof(bm->angular_velocity));
    bf->flags&=~BF_IS_STATIC_OR_KINEMATIC_OR_DYNAMIC;bf->flags|=new_motion_state;
    c->bodies.idle_counters[body]=(new_motion_state&BF_IS_DYNAMIC)?0:0xFF; // wake up or put to sleep
}

void body_scale(nudge::context_t* c,unsigned body,float scale_factor,float mass_scale_factor) {
    assert(c && body<c->bodies.count);
    assert(scale_factor!=0.f);
    if (scale_factor<0.f) {
        const float hey = c->bodies.infos[body].aabb_half_extents[1];assert(hey>0.f);
        scale_factor = -scale_factor/hey;
    }
    // scale mass and local_inertia
    BodyProperties* bp = &c->bodies.properties[body];
    if (mass_scale_factor==0.f) mass_scale_factor = scale_factor*scale_factor*scale_factor;
    else if (mass_scale_factor<0.f) {
        mass_scale_factor=-mass_scale_factor;   // this is our new mass
        for (int k=0;k<3;k++) bp->inertia_inverse[k]/=scale_factor*scale_factor*mass_scale_factor*bp->mass_inverse;
        bp->mass_inverse=1.f/mass_scale_factor;
    }
    else {
        bp->mass_inverse/=mass_scale_factor;
        for (int k=0;k<3;k++) bp->inertia_inverse[k]/=(scale_factor*scale_factor*mass_scale_factor);
    }
    // scale colliders
    BodyLayout* bl = &c->bodies.layouts[body];
    if (bl->num_boxes>0) {
        assert(bl->first_box_index>=0 && (uint16_t)bl->first_box_index+bl->num_boxes<=c->colliders.boxes.count);
        Transform* T = &c->colliders.boxes.transforms[bl->first_box_index];
        BoxCollider* C = &c->colliders.boxes.data[bl->first_box_index];
        for (uint16_t i=0;i<bl->num_boxes;i++) {for (int k=0;k<3;k++) {T[i].p[k]*=scale_factor;C[i].size[k]*=scale_factor;}}
    }
    if (bl->num_spheres>0) {
        assert(bl->first_sphere_index>=0 && (uint16_t)bl->first_sphere_index+bl->num_spheres<=c->colliders.spheres.count);
        Transform* T = &c->colliders.spheres.transforms[bl->first_sphere_index];
        SphereCollider* C = &c->colliders.spheres.data[bl->first_sphere_index];
        for (uint16_t i=0;i<bl->num_spheres;i++) {Transform* t = &T[i];C[i].radius*=scale_factor;for (int k=0;k<3;k++) t->p[k]*=scale_factor;}
    }
    // scale aabb and com_offset
    body_recalculate_bounding_box(c,body);

    if (bp->mass_inverse && (bp->inertia_inverse[0]==0 && bp->inertia_inverse[1]==0 && bp->inertia_inverse[2]==0)) {
        // this case might indeed happen, when a static body with zero mass and inertia is scaled with a negative 'mass_scaling_factor'
        // the body remains static (flag+sleeping state), but it's better to set a valid inertia to it (or to just reset its mass to zero)
        const BodyInfo* bi = &c->bodies.infos[body];
        calculate_box_inertia_inverse(bp->inertia_inverse,1.f/bp->mass_inverse,bi->aabb_half_extents[0],bi->aabb_half_extents[1],bi->aabb_half_extents[2],bi->com_offset);
    }
}

static void simulate_kinematic_animations(context_t* c,float timeStep)  {
    // This code is too long... can we compress it better?
    for (int j=0,j_sz=(int)c->kinematic_data.animations_count;j<j_sz;j++) {
        KinematicData::Animation* ka = &c->kinematic_data.animations[j];
        if (ka->body>=c->bodies.count) continue;
        const uint32_t flags = c->bodies.filters[ka->body].flags;        
        if (flags&BF_IS_DISABLED_OR_REMOVED) {
            if (flags&BF_IS_REMOVED) {
#               ifdef NUDGE_DELETE_KINEMATIC_ANIMATIONS_REFERENCING_REMOVED_BODIES
                // Delete KinematicData::Animation
                // |------|-----|-----|-----|
                // 0      1     2     3     4
                //        j                count
                memmove(&c->kinematic_data.animations[j],&c->kinematic_data.animations[j+1],(c->kinematic_data.animations_count-(j+1))*sizeof(KinematicData::Animation));
                --j;--j_sz;--c->kinematic_data.animations_count;
#               else // NUDGE_DELETE_KINEMATIC_ANIMATIONS_REFERENCING_REMOVED_BODIES
                ka->body = NUDGE_INVALID_BODY_ID;assert(ka->body>=c->bodies.count);
#               endif // NUDGE_DELETE_KINEMATIC_ANIMATIONS_REFERENCING_REMOVED_BODIES
            }
            continue;
        }
        const float absSpeed = fabsf(ka->speed); //ka->speed<0 ? -ka->speed : ka->speed;
        if (ka->playing && absSpeed!=0 && ka->body<c->bodies.count
                && flags&BF_IS_KINEMATIC
                ) {
            const float deltaTime = timeStep;
            Transform T = identity_transform;
            //assert(ka->body<c->bodies.count);
            //assert(c->bodies.properties[ka->body].mass_inverse<0.f);
            assert(ka->key_frame_start+ka->key_frame_count<=c->kinematic_data.key_frame_capacity);
            const Transform* pkfT = &c->kinematic_data.key_frame_transforms[ka->key_frame_start];
            const KinematicData::TimeMode* pkfMode = &c->kinematic_data.key_frame_modes[ka->key_frame_start];
            if (ka->total_time<0.f)  {ka->total_time = 0;for (uint32_t l=0;l<ka->key_frame_count;l++)  ka->total_time+=pkfT[l] .time;}
            //if (ka->play_time<0.f)   play_time = ka->offset_time; // Nope: this must be set together with ka->playing to start playback
            ka->play_time+=deltaTime*ka->speed;
            bool mustReverse = ka->play_time < 0;
            float absPlayTime =  mustReverse ? -ka->play_time : ka->play_time;
            float curTime(0);

            if (ka->loop_mode!=KinematicData::Animation::LM_NO_LOOP) {
                const float totalTime = ka->total_time;
                if (totalTime<=0) continue;
                const float fractionTime = absPlayTime/totalTime;
                const unsigned long fractionTimeUL = (unsigned long)fractionTime;
                if (ka->loop_mode==KinematicData::Animation::LM_LOOP_PING_PONG) {
                    if (mustReverse) mustReverse = fractionTimeUL%2==0;
                    else mustReverse = fractionTimeUL%2==1;
                }
                absPlayTime-=(totalTime*fractionTimeUL);
            }
            const int ksz=(int)ka->key_frame_count;
            if (!mustReverse || ksz<=1) {
                for (int keyFrameIndex=0;keyFrameIndex<ksz;keyFrameIndex++)  {
                    const Transform* kfT = &pkfT[keyFrameIndex];
                    const KinematicData::TimeMode* kfMode = &pkfMode[keyFrameIndex];
                    if (kfT->time<=0) continue;
                    curTime+=kfT->time;
                    if (absPlayTime <= curTime) {
                        float factor = float(1)-(curTime-absPlayTime)/kfT->time;
                        if (*kfMode == KinematicData::TM_ACCELERATE) factor*=factor;
                        else if (*kfMode == KinematicData::TM_DECELERATE && factor>0) factor=sqrtf(factor);

                        if (keyFrameIndex>0) {
                            const Transform* kfTp = &pkfT[keyFrameIndex-1];
                            T = TransformSlerp(*kfTp,*kfT,factor);
                            if (ka->use_baseT) T = ka->baseT*T;
                        }
                        else {
                            if (ka->loop_mode!=KinematicData::Animation::LM_LOOP_NORMAL || ksz<=1) {
                                T = TransformSlerp(c->bodies.transforms[ka->body],ka->use_baseT ? (ka->baseT*(*kfT)) : (*kfT),factor);
                            }
                            else {
                                const Transform* startT = &pkfT[ksz-1];
                                T = TransformSlerp(*startT,*kfT,factor);
                                if (ka->use_baseT) T = ka->baseT*T;
                            }
                        }
                        TransformAssignToBody(c,ka->body,T,deltaTime);
                        break;
                    }
                    else if (keyFrameIndex == ksz-1) {
                        if (ka->loop_mode==KinematicData::Animation::LM_NO_LOOP) {
                            TransformAssignToBody(c,ka->body,ka->use_baseT ? (ka->baseT*(*kfT)) : (*kfT),deltaTime);
                            ka->playing = false;ka->play_time=ka->offset_time;    //TODO: fire end event here?
                        }
                    }
                }
            }
            else {
                // Play animations backwards
                // ksz > 1 here
                for (int keyFrameIndex=ksz-1;keyFrameIndex>=0;keyFrameIndex--)  {
                    const Transform* kfT = &pkfT[keyFrameIndex];
                    const bool isFirstKeyFrame = (keyFrameIndex==ksz-1);
                    const float& kfTime =  isFirstKeyFrame ? pkfT[0].time : pkfT[keyFrameIndex+1].time;
                    const KinematicData::TimeMode timeMode = isFirstKeyFrame ? pkfMode[0] : pkfMode[keyFrameIndex+1];
                    if (kfTime<=0) continue;   //MMMhhh, empty frame array will break both autoFlags and endEvents here....
                    curTime+=kfTime;
                    if (absPlayTime <= curTime) {
                        float factor = float(1)-(curTime-absPlayTime)/kfTime;
                        if (timeMode == KinematicData::TM_DECELERATE ) factor*=factor;
                        else if (timeMode == KinematicData::TM_ACCELERATE   && factor>0) factor=sqrtf(factor);

                        if (!isFirstKeyFrame) {
                            const Transform* kfTp = &pkfT[keyFrameIndex+1];
                            T = TransformSlerp(*kfTp,*kfT,factor);
                            if (ka->use_baseT) T = ka->baseT*T;
                        }
                        else {
                            if (ka->loop_mode!=KinematicData::Animation::LM_LOOP_NORMAL || ksz<=1) {
                                T = TransformSlerp(c->bodies.transforms[ka->body],ka->use_baseT ? (ka->baseT*(*kfT)) : (*kfT),factor);
                            }
                            else {
                                const Transform* startT = &pkfT[0];
                                T = TransformSlerp(*startT,*kfT,factor);
                                if (ka->use_baseT) T = ka->baseT*T;
                            }
                        }
                        TransformAssignToBody(c,ka->body,T,deltaTime);
                        break;
                    }
                    else if (keyFrameIndex == 0) {
                        if (ka->loop_mode==KinematicData::Animation::LM_NO_LOOP) {
                            TransformAssignToBody(c,ka->body,ka->use_baseT ? (ka->baseT*(*kfT)) : (*kfT),deltaTime);
                            ka->playing = false;ka->play_time=ka->offset_time;    //TODO: fire end event here?
                        }
                    }
                }
            }
        }
    }
}


float* calculate_graphic_transform_for_body(context_t* c,unsigned body,float* pModelMatrix16Out)   {
    assert(body<c->bodies.count);
    assert(pModelMatrix16Out);
    const Transform* T = &c->bodies.transforms[body];assert(T->body==body);
    //const float mass_inverse = c->bodies.properties[T->body].mass_inverse<0.f;
    const uint32_t flags = c->bodies.filters[T->body].flags;
    const FlagMask exclude_flags = c->global_data.exclude_smoothing_graphic_transform_flags;
    const int isSleeping = c->bodies.idle_counters[T->body]==0xff;
    const float timeStepMinusRemainingTime=c->simulation_params.time_step_minus_remaining_time;
    Transform Tn;memcpy(&Tn,T,sizeof(Transform));

    int mustSmoothTransform = timeStepMinusRemainingTime>0 && !(exclude_flags&flags);
    if (mustSmoothTransform && (flags&BF_IS_STATIC || (flags&BF_IS_DYNAMIC && isSleeping))) mustSmoothTransform=0;

    if (mustSmoothTransform)   {
        // same work done in advance(...) here AFAIK
        assert(T==&c->bodies.transforms[T->body]);
        /*
            // Actually this is the correct code to advance graphic transforms...
            // ...but we DON'T want to do it
            const float* linvel = c->bodies.momentum[T->body].velocity;
            const float* angvel = c->bodies.momentum[T->body].angular_velocity;
            // advance Tn based on T, linvel and angvel
            for (int l=0;l<3;l++)   {Tn.position[l]+=linvel[l]*remainingTime;}
            nm_QuatAdvance(Tn.rotation,T->rotation,angvel,remainingTime*0.5f);
            */
        // Instead we want to move graphic transforms backwards!
        // This is what we really want (even is we add a 'timeStep' delay)!
        // In fact simulate(...) calculates linvel and angvel per body and then uses them to advance bodies in advance(...).
        // That's why advancing bodies further is NOT correct, but going back is CORRECT (note that we couldn't just advance a fraction in advance(...): that's the PHYSIC transform and it would affect simulation).

        const float* linvel = c->bodies.momentum[T->body].velocity;
        const float* angvel = c->bodies.momentum[T->body].angular_velocity;
        // retrace Tn based on T, linvel and angvel
        for (int l=0;l<3;l++)   {
            Tn.position[l]-=linvel[l]*timeStepMinusRemainingTime;
            //Tn.rotation[l]-=angvel[l]*timeStepMinusRemainingTime; // approximate but faster (mmmh, spheres "pulse" with this)
        }
        const float angvelinv[3] = {-angvel[0],-angvel[1],-angvel[2]};  // good but slower
        nm_QuatAdvance(Tn.rotation,T->rotation,angvelinv,timeStepMinusRemainingTime*0.5f);
    }

    // we must convert the physic Transform Tn to the graphic mMatrix (16 floats) used for rendering
    if (pModelMatrix16Out) TransformToMat4(pModelMatrix16Out,&Tn);
    // graphic transform (pModelMatrix16Out) must be updated if the body has a COM offset
    if ((flags&BF_HAS_COM_OFFSET))   {
        // comOffset must be subtracted
        const float* comOffset = c->bodies.infos[T->body].com_offset;
        //for (int l=0;l<3;l++) Tn.position[l]-= ???;
        for (int l=0;l<3;l++) pModelMatrix16Out[12+l] -= pModelMatrix16Out[l]*comOffset[0]+pModelMatrix16Out[4+l]*comOffset[1]+pModelMatrix16Out[8+l]*comOffset[2];
    }
    return pModelMatrix16Out;
}

void calculate_graphic_transforms(context_t* c,float* pModelMatricesOut,unsigned modelMatrixStrideInFloatUnits,int loopActiveBodiesOnly)   {
    if (modelMatrixStrideInFloatUnits<16) modelMatrixStrideInFloatUnits=16;
    const unsigned bodies_count = loopActiveBodiesOnly ? : c->bodies.count;

    for (uint32_t i=0;i<bodies_count;i++)    {
        const uint32_t body = loopActiveBodiesOnly ? c->active_bodies.indices[i] : i;
        calculate_graphic_transform_for_body(c,body,&pModelMatricesOut[body*modelMatrixStrideInFloatUnits]);
    }
}

void contact_data_find_colliders(const context_t* c,
                                 unsigned contact_data_index,
                                 int16_t* box_collider_index_for_body_a,
                                 int16_t* sphere_collider_index_for_body_a,
                                 int16_t* box_collider_index_for_body_b,
                                 int16_t* sphere_collider_index_for_body_b,
                                 int use_relative_values_for_output_indices
                                 )  {
    assert(c && contact_data_index<c->contact_data.count);
    const ContactData* cc = &c->contact_data;
    const uint64_t tag = cc->tags[contact_data_index];   // each 64-bit tag is a combination of the 2 16-bit tags of the colliders inside the two bodies and a 32-bit(?) tag of the contact feature (e.g. EDGE-EDGE, etc.)
    const BodyPair* bp = &cc->bodies[contact_data_index];
    const unsigned a = bp->a;assert(a<c->bodies.count);
    const unsigned b = bp->b;assert(b<c->bodies.count);
    const uint64_t a_tag =  (tag&0x0000FFFF00000000ULL)>>(2ULL*16ULL);
    const uint64_t b_tag =  (tag&0xFFFF000000000000ULL)>>(3ULL*16ULL);
    struct coll_t {unsigned body;uint64_t tag;int16_t first_box_index;int16_t* box_colliding_index;uint16_t num_boxes;int16_t first_sphere_index;int16_t* sphere_colliding_index;uint16_t num_spheres;};
    struct coll_t coll[2]=
        {{a,a_tag,c->bodies.layouts[a].first_box_index,box_collider_index_for_body_a,c->bodies.layouts[a].num_boxes,c->bodies.layouts[a].first_sphere_index,sphere_collider_index_for_body_a,c->bodies.layouts[a].num_spheres},
        {b,b_tag,c->bodies.layouts[b].first_box_index,box_collider_index_for_body_b,c->bodies.layouts[b].num_boxes,c->bodies.layouts[b].first_sphere_index,sphere_collider_index_for_body_b,c->bodies.layouts[b].num_spheres}};
    for (int t=0;t<2;t++) {
        struct coll_t* cl = &coll[t];
        assert(cl->num_boxes || cl->num_spheres);
        assert(cl->tag<NUDGE_START_SPHERE_TAG+c->MAX_NUM_SPHERES);
        if (cl->box_colliding_index) {
            *cl->box_colliding_index=-1;
            if (cl->num_boxes>0) {
                assert(cl->first_box_index>=0);
                assert((unsigned)cl->first_box_index+cl->num_boxes<=c->colliders.boxes.count);
                for (uint16_t ci=cl->first_box_index;ci<cl->first_box_index+cl->num_boxes;ci++) {
                    assert(c->colliders.boxes.transforms[ci].body==cl->body);
                    if (c->colliders.boxes.tags[ci]==cl->tag) {*cl->box_colliding_index=use_relative_values_for_output_indices?(ci-cl->first_box_index):ci;break;}
                }
            assert(*cl->box_colliding_index>=0);
            }
        }
        if (cl->sphere_colliding_index) {
            *cl->sphere_colliding_index=-1;
            if (cl->num_spheres>0) {
                assert(cl->first_sphere_index>=0);
                assert((unsigned)cl->first_sphere_index+cl->num_spheres<=c->colliders.spheres.count);
                for (uint16_t ci=cl->first_sphere_index;ci<cl->first_sphere_index+cl->num_spheres;ci++) {
                    assert(c->colliders.spheres.transforms[ci].body==cl->body);
                    if (c->colliders.spheres.tags[ci]==cl->tag) {*cl->sphere_colliding_index=use_relative_values_for_output_indices?(ci-cl->first_sphere_index):ci;break;}
                }
                assert(*cl->sphere_colliding_index>=0);
            }
            if (cl->box_colliding_index && cl->sphere_colliding_index) {
                assert(*cl->box_colliding_index>=0 || *cl->sphere_colliding_index>=0);
                assert(*cl->box_colliding_index==-1 || *cl->sphere_colliding_index==-1);
                // we've correctly found the subshape involved in the collision!
            }
        }
    }
}


unsigned pre_simulation_step(context_t* c,double elapsedSecondsFromLastCall)    {
    struct SimulationParams* sp = &c->simulation_params;
    unsigned sim_is_burning_time = 0;
    sp->num_substeps_in_last_frame=0;
    if (elapsedSecondsFromLastCall<0) elapsedSecondsFromLastCall=0;
    sp->remaining_time_in_seconds+=elapsedSecondsFromLastCall;
    while (sp->remaining_time_in_seconds>=sp->time_step)	{
        if (sp->num_substeps_in_last_frame>=sp->max_num_substeps) {
            sp->remaining_time_in_seconds = 0;
//#           ifndef NDEBUG
            if (sp->numsubsteps_overflow_warning_mode!=2) {
                const int must_warn = sp->numsubsteps_overflow_warning_mode!=0 || sp->numsubsteps_overflow_in_last_frame>0;
                if (must_warn) log("[PhysicFrame: %llu] max_num_substeps=%u reached:\tBurnt remaining_time=%1.3f (on time_step=%1.3f)\n",sp->num_frames,sp->max_num_substeps,elapsedSecondsFromLastCall,sp->time_step);
            }
//#           endif
            // setting a bigger 'sp->max_num_substeps' can help if you see this message every frame, but it you only see
            // it every now and then it's absolutely normal.            
            sim_is_burning_time = 1;
            break;
        }
        sp->remaining_time_in_seconds-=sp->time_step;
        ++sp->num_substeps_in_last_frame;
    }
    //assert(sp->remaining_time_in_seconds<sp->time_step);
    //if (sp->remaining_time_in_seconds>sp->time_step) sp->remaining_time_in_seconds=0;   // It might happen when we pause the game
    sp->time_step_minus_remaining_time=(float)(sp->time_step-sp->remaining_time_in_seconds);

    sp->numsubsteps_overflow_in_last_frame = sim_is_burning_time;
    // Good to debug 'sp->num_substeps_in_last_frame':
    //log("[PhysicFrame: %llu] num_substeps_in_last_frame=%d time_step=%f remaining_time=%f\n",sp->num_frames,sp->num_substeps_in_last_frame,sp->time_step,sp->remaining_time_in_seconds);
    return sp->num_substeps_in_last_frame;
}


extern uintptr_t get_required_arena_size_for_setup_contact_constraints(context_t* c);
void simulate(context_t* c,float timeStep, unsigned numSubSteps, unsigned numIterations)   {

    finalize_removed_bodies(c);

#   define NUDGE_KINEMATIC_ANIMATION_QUALITY_LOW
#   ifdef NUDGE_KINEMATIC_ANIMATION_QUALITY_LOW
    if (numSubSteps>0) simulate_kinematic_animations(c,timeStep*numSubSteps);
#   endif

    for (unsigned n = 0; n < numSubSteps; ++n) {
        // Kinematic objects (or outside this loop?)
#       ifndef NUDGE_KINEMATIC_ANIMATION_QUALITY_LOW
        simulate_kinematic_animations(c,timeStep);
#       endif

        uintptr_t required_arena_size = get_required_arena_size_for_setup_contact_constraints(c);
        // TODO: we should estimate all the other memory contributions, since 'setup_contact_constraints' is not the only called function that allocates... so we do:
        required_arena_size = required_arena_size+required_arena_size;

        if (c->arena.size<required_arena_size) {
            _mm_free(c->arena.data);c->arena.data=0;
            const uintptr_t new_size = required_arena_size+c->arena.size/2;
            c->arena.data = _mm_malloc(new_size,NUDGE_ARENA_SIZE_ALIGNMENT);memset(c->arena.data,0,new_size);
            log("[nudge_frame: %llu] Resized Arena from %lu to %lu [consider redefining NUDGE_ARENA_SIZE]\n",c->simulation_params.num_frames,c->arena.size,new_size);flush();
            c->arena.size = new_size;
        }


        // Find contacts.
        BodyConnections connections = {}; // NOTE: Custom constraints should be added as body connections.
        collide(c, connections);

        Arena temporary = c->arena;

        // NOTE: Custom contacts can be added here, e.g., against the static environment.

        // Apply gravity and damping.
        float damping_linear  = 1.0f - timeStep*c->simulation_params.linear_damping;
        float damping_angular = 1.0f - timeStep*c->simulation_params.angular_damping;
        const float* pGravity[2] = {c->global_data.gravity,NULL};
        const int gravityIdx = (c->global_data.flags&GF_USE_GLOBAL_GRAVITY) ? 0 : 1;
        for (unsigned i = 0; i < c->active_bodies.count; ++i) {
            const unsigned index = c->active_bodies.indices[i];
            //const BodyFilter* filter = &c->bodies.filters[index];
            const FlagMask flags = c->bodies.filters[index].flags;
            if (flags&BF_IS_DYNAMIC)    {
                // Apply gravity and damping
                BodyMomentum* momentum = &c->bodies.momentum[index];
                pGravity[1] = c->bodies.properties[index].gravity;
                const float* gravity = pGravity[(flags&BF_HAS_DIFFERENT_GRAVITY_MODE)?(!gravityIdx):gravityIdx];
                for (int l=0;l<3;l++)   {
                    momentum->velocity[l] += gravity[l] * timeStep;
                    momentum->velocity[l] *= damping_linear;
                    momentum->angular_velocity[l] *= damping_angular;
                }
                if (flags&BF_NEVER_SLEEPING) c->bodies.idle_counters[index]=0;   // prevents sleeping

#               if NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES>0
                memset(&c->bodies.infos[index].aux_bodies,0xFF,NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES*sizeof(int16_t));    // sets all components to -1
#               endif // NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES
            }
        }

        // Read previous impulses from contact cache.
        ContactImpulseData* contact_impulses = read_cached_impulses(c->contact_cache, c->contact_data, &temporary);

        // Setup contact constraints and apply the initial impulses.
        ContactConstraintData* contact_constraints = setup_contact_constraints(c,/*c->active_bodies, c->contact_data, c->bodies,*/ contact_impulses, &temporary);

        // Apply contact impulses. Increasing the number of iterations will improve stability.
        for (unsigned i = 0; i < numIterations; ++i) {
            apply_impulses(contact_constraints, c->bodies);
            // NOTE: Custom constraint impulses should be applied here.
        }

        // Update contact impulses.
        update_cached_impulses(contact_constraints, contact_impulses);

        // Write the updated contact impulses to the cache.
        write_cached_impulses(&c->contact_cache, c->contact_data, contact_impulses);

        // Move active bodies.
        advance(c, timeStep);

        ++c->simulation_params.num_total_substeps;
    }

}

void simulation_step(context_t* c)  {
    struct SimulationParams* sp = &c->simulation_params;
    if (sp->num_substeps_in_last_frame>0)   {
        assert(sp->time_step>0 && sp->num_iterations_per_substep>0);
        simulate(c,(float)sp->time_step,sp->num_substeps_in_last_frame,sp->num_iterations_per_substep);
        ++sp->num_frames;
    }
}

#ifndef NUDGE_NO_STDIO
void save_context(FILE* f,const context_t* c)   {
    size_t rv = 0;
    assert(f && c);
    // max values
    fprintf(f,"MAX_NUM_BOXES:\n%u\n",c->MAX_NUM_BOXES);
    fprintf(f,"MAX_NUM_SPHERES:\n%u\n",c->MAX_NUM_SPHERES);
    // c->bodies
    const BodyData* bd = &c->bodies;
    uint32_t size_of_BodyInfo_user = 0;
#   ifndef NUDGE_BODYINFO_STRUCT_NO_USER_DATA
    size_of_BodyInfo_user = (uint32_t) sizeof(BodyInfo::user);
#   endif
    fprintf(f,"BodyData:\ncount: %u\nsizeof(BodyData): %u\nsizeof(Transform): %u\nsizeof(BodyProperties): %u\nsizeof(BodyMomentum): %u\n"
              "sizeof(BodyFilter): %u\nsizeof(BodyInfo): %u\nsizeof(BodyInfo::user): %u\nnum_aux_bodies: %u\n",bd->count,(uint32_t)sizeof(BodyData),(uint32_t)sizeof(Transform),(uint32_t)sizeof(BodyProperties),(uint32_t)sizeof(BodyMomentum),(uint32_t)sizeof(BodyFilter),(uint32_t)sizeof(BodyInfo),size_of_BodyInfo_user,NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES);
    rv=fwrite(bd->transforms,sizeof(Transform),bd->count,f);assert(rv==bd->count);
    rv=fwrite(bd->properties,sizeof(BodyProperties),bd->count,f);assert(rv==bd->count);
    rv=fwrite(bd->momentum,sizeof(BodyMomentum),bd->count,f);assert(rv==bd->count);
    rv=fwrite(bd->filters,sizeof(BodyFilter),bd->count,f);assert(rv==bd->count);
    rv=fwrite(bd->layouts,sizeof(BodyLayout),bd->count,f);assert(rv==bd->count);
    rv=fwrite(bd->idle_counters,sizeof(uint8_t),bd->count,f);assert(rv==bd->count);
    rv=fwrite(bd->infos,sizeof(BodyInfo),bd->count,f);assert(rv==bd->count);
    // c->colliders
    const ColliderData* cd = &c->colliders;
    fprintf(f,"\nColliderData::boxes:\n%u\n",cd->boxes.count);
    rv=fwrite(cd->boxes.tags,sizeof(uint16_t),cd->boxes.count,f);assert(rv==cd->boxes.count);
    rv=fwrite(cd->boxes.data,sizeof(BoxCollider),cd->boxes.count,f);assert(rv==cd->boxes.count);
    rv=fwrite(cd->boxes.transforms,sizeof(Transform),cd->boxes.count,f);assert(rv==cd->boxes.count);
    fprintf(f,"\nColliderData::spheres:\n%u\n",cd->spheres.count);
    rv=fwrite(cd->spheres.tags,sizeof(uint16_t),cd->spheres.count,f);assert(rv==cd->spheres.count);
    rv=fwrite(cd->spheres.data,sizeof(SphereCollider),cd->spheres.count,f);assert(rv==cd->spheres.count);
    rv=fwrite(cd->spheres.transforms,sizeof(Transform),cd->spheres.count,f);assert(rv==cd->spheres.count);
    // c->contact_data
    const ContactData* td = &c->contact_data;
    fprintf(f,"\nContactData::count\n%u\n",td->count); // we skip td->capacity
    rv=fwrite(td->data,sizeof(Contact),td->count,f);assert(rv==td->count);
    rv=fwrite(td->bodies,sizeof(BodyPair),td->count,f);assert(rv==td->count);
    rv=fwrite(td->tags,sizeof(uint64_t),td->count,f);assert(rv==td->count);
    fprintf(f,"\nContactData::sleeping_count\n%u\n",td->sleeping_count);
    rv=fwrite(td->sleeping_pairs,sizeof(uint32_t),td->sleeping_count,f);assert(rv==td->sleeping_count);
    // c->contact_cache
    const ContactCache* tc = &c->contact_cache;
    fprintf(f,"\nContactCache::count\n%u\n",tc->count); // we skip tc->capacity
    rv=fwrite(tc->tags,sizeof(uint64_t),tc->count,f);assert(rv==tc->count);
    rv=fwrite(tc->data,sizeof(CachedContactImpulse),tc->count,f);assert(rv==tc->count);
    // c->active_bodies
    const ActiveBodies* ab = &c->active_bodies;
    fprintf(f,"\nActiveBodies::count\n%u\n",ab->count); // we skip ab->capacity
    rv=fwrite(ab->indices,sizeof(uint16_t),ab->count,f);assert(rv==ab->count);
    // c->kinematic_data
    const KinematicData* kd = &c->kinematic_data;
    fprintf(f,"\nKinematicData::key_frame_count\n%u\n",kd->key_frame_count);
    rv=fwrite(kd->key_frame_transforms,sizeof(Transform),kd->key_frame_count,f);assert(rv==kd->key_frame_count);
    rv=fwrite(kd->key_frame_modes,sizeof(KinematicData::TimeMode),kd->key_frame_count,f);assert(rv==kd->key_frame_count);
    fprintf(f,"\nKinematicData::animations_count\n%u\n",kd->animations_count);
    rv=fwrite(kd->animations,sizeof(KinematicData::Animation),kd->animations_count,f);assert(rv==kd->animations_count);
    // c->simulation_params
    fprintf(f,"\nsizeof(SimulationParams)\n%u\n",(uint32_t)sizeof(SimulationParams));
    rv=fwrite(&c->simulation_params,sizeof(SimulationParams),1,f);assert(rv==1);
    // c->global_data
    fprintf(f,"\nsizeof(GlobalData)\n%u\n",(uint32_t)sizeof(GlobalData));
    rv=fwrite(&c->global_data,sizeof(GlobalData),1,f);assert(rv==1);
    rv=fwrite(c->global_data.removed_bodies,sizeof(c->global_data.removed_bodies[0]),c->global_data.removed_bodies_count,f);assert(rv==c->global_data.removed_bodies_count);    
    // c->userUint64
#   ifndef NUDGE_CONTEXT_STRUCT_NO_USER_DATA
    fprintf(f,"\nsizeof(c->user)\n%u\n",(uint32_t)sizeof(c->user));
    rv=fwrite(&c->user,sizeof(c->user),1,f);assert(rv==1);
#   endif // NUDGE_CONTEXT_STRUCT_NO_USER_DATA
}
void load_context(FILE* f,context_t* c) {
    size_t rv = 0;uint32_t tmp[8]={};
    assert(f && c);
    // max values
    unsigned num_saved_boxes=0,num_saved_spheres=0;
    rv=fscanf(f,"MAX_NUM_BOXES:\n%u\n",&num_saved_boxes);assert(rv==1);
    rv=fscanf(f,"MAX_NUM_SPHERES:\n%u\n",&num_saved_spheres);assert(rv==1);
    assert(c->MAX_NUM_BOXES>=num_saved_boxes); // TODO?: free and reallocate 'c' before continuing?
    assert(c->MAX_NUM_SPHERES>=num_saved_spheres); // TODO?: free and reallocate 'c' before continuing?
    // Must be limit current context? Yes...
    *((unsigned*)&c->MAX_NUM_BOXES) = num_saved_boxes;*((unsigned*)&c->MAX_NUM_SPHERES) = num_saved_spheres;*((unsigned*)&c->MAX_NUM_BODIES) = num_saved_boxes+num_saved_spheres;
    // c->bodies
    BodyData* bd = &c->bodies;assert(bd->count<=c->MAX_NUM_BODIES);
    rv=fscanf(f,"BodyData:\ncount: %u\nsizeof(BodyData): %u\nsizeof(Transform): %u\nsizeof(BodyProperties): %u\nsizeof(BodyMomentum): %u\n"
                "sizeof(BodyFilter): %u\nsizeof(BodyInfo): %u\nsizeof(BodyInfo::user): %u\nnum_aux_bodies: %u\n",&bd->count,&tmp[0],&tmp[1],&tmp[2],&tmp[3],&tmp[4],&tmp[5],&tmp[6],&tmp[7]);assert(rv==9);
    assert(tmp[0]==sizeof(BodyData));
    assert(tmp[1]==sizeof(Transform));
    assert(tmp[2]==sizeof(BodyProperties));
    assert(tmp[3]==sizeof(BodyMomentum));
    assert(tmp[4]==sizeof(BodyFilter));
    assert(tmp[5]==sizeof(BodyInfo));
#   ifndef NUDGE_BODYINFO_STRUCT_NO_USER_DATA
    assert(tmp[6]==sizeof(BodyInfo::user));
#   else
    assert(tmp[6]==0);
#   endif
    assert(tmp[7]==NUDGE_BODYINFO_STRUCT_NUM_AUX_BODIES);
    rv=fread(bd->transforms,sizeof(Transform),bd->count,f);assert(rv==bd->count);
    rv=fread(bd->properties,sizeof(BodyProperties),bd->count,f);assert(rv==bd->count);
    rv=fread(bd->momentum,sizeof(BodyMomentum),bd->count,f);assert(rv==bd->count);
    rv=fread(bd->filters,sizeof(BodyFilter),bd->count,f);assert(rv==bd->count);
    rv=fread(bd->layouts,sizeof(BodyLayout),bd->count,f);assert(rv==bd->count);
    rv=fread(bd->idle_counters,sizeof(uint8_t),bd->count,f);assert(rv==bd->count);
    rv=fread(bd->infos,sizeof(BodyInfo),bd->count,f);assert(rv==bd->count);
    // c->colliders
    ColliderData* cd = &c->colliders;
    rv=fscanf(f,"\nColliderData::boxes:\n%u\n",&cd->boxes.count);assert(rv==1);assert(cd->boxes.count<=c->MAX_NUM_BOXES);
    rv=fread(cd->boxes.tags,sizeof(uint16_t),cd->boxes.count,f);assert(rv==cd->boxes.count);
    rv=fread(cd->boxes.data,sizeof(BoxCollider),cd->boxes.count,f);assert(rv==cd->boxes.count);
    rv=fread(cd->boxes.transforms,sizeof(Transform),cd->boxes.count,f);assert(rv==cd->boxes.count);
    rv=fscanf(f,"\nColliderData::spheres:\n%u\n",&cd->spheres.count);assert(rv==1);assert(cd->spheres.count<=c->MAX_NUM_BOXES);
    rv=fread(cd->spheres.tags,sizeof(uint16_t),cd->spheres.count,f);assert(rv==cd->spheres.count);
    rv=fread(cd->spheres.data,sizeof(SphereCollider),cd->spheres.count,f);assert(rv==cd->spheres.count);
    rv=fread(cd->spheres.transforms,sizeof(Transform),cd->spheres.count,f);assert(rv==cd->spheres.count);
    // c->contact_data
    ContactData* td = &c->contact_data;
    rv=fscanf(f,"\nContactData::count\n%u\n",&td->count);assert(rv==1);assert(td->count<=td->capacity);
    rv=fread(td->data,sizeof(Contact),td->count,f);assert(rv==td->count);
    rv=fread(td->bodies,sizeof(BodyPair),td->count,f);assert(rv==td->count);
    rv=fread(td->tags,sizeof(uint64_t),td->count,f);assert(rv==td->count);
    rv=fscanf(f,"\nContactData::sleeping_count\n%u\n",&td->sleeping_count);assert(rv==1);
    rv=fread(td->sleeping_pairs,sizeof(uint32_t),td->sleeping_count,f);assert(rv==td->sleeping_count);
    // c->contact_cache
    ContactCache* tc = &c->contact_cache;
    rv=fscanf(f,"\nContactCache::count\n%u\n",&tc->count);assert(tc->count<=tc->capacity);assert(rv==1);
    rv=fread(tc->tags,sizeof(uint64_t),tc->count,f);assert(rv==tc->count);
    rv=fread(tc->data,sizeof(CachedContactImpulse),tc->count,f);assert(rv==tc->count);
    // c->active_bodies
    ActiveBodies* ab = &c->active_bodies;
    rv=fscanf(f,"\nActiveBodies::count\n%u\n",&ab->count);assert(rv==1);assert(ab->count<=ab->capacity);
    rv=fread(ab->indices,sizeof(uint16_t),ab->count,f);assert(rv==ab->count);
    // c->kinematic_data
    KinematicData* kd = &c->kinematic_data;
    rv=fscanf(f,"\nKinematicData::key_frame_count\n%u\n",&kd->key_frame_count);assert(rv==1);
    if (kd->key_frame_count>kd->key_frame_capacity) kinematic_data_reserve_key_frames(kd,kd->key_frame_count); // TODO: we should reset unuset space between count and capacity
    rv=fread(kd->key_frame_transforms,sizeof(Transform),kd->key_frame_count,f);assert(rv==kd->key_frame_count);
    rv=fread(kd->key_frame_modes,sizeof(KinematicData::TimeMode),kd->key_frame_count,f);assert(rv==kd->key_frame_count);
    rv=fscanf(f,"\nKinematicData::animations_count\n%u\n",&kd->animations_count);assert(kd->animations_count<=kd->animations_capacity);assert(rv==1);
    if (kd->animations_count>kd->animations_capacity) kinematic_data_reserve_animations(kd,kd->animations_count); // TODO: we should reset unuset space between count and capacity
    rv=fread(kd->animations,sizeof(KinematicData::Animation),kd->animations_count,f);assert(rv==kd->animations_count);
    // c->simulation_params
    const SimulationParams old_simulation_params = c->simulation_params;
    uint32_t simulation_params_size=0;
    rv=fscanf(f,"\nsizeof(SimulationParams)\n%u\n",&simulation_params_size);assert(simulation_params_size==(uint32_t)sizeof(SimulationParams));assert(rv==1);
    rv=fread(&c->simulation_params,sizeof(SimulationParams),1,f);assert(rv==1);
    // c->global_data
    uint32_t* host_ptr = c->global_data.removed_bodies; // we can't overwrite this!
    uint32_t global_data_size=0;    
    rv=fscanf(f,"\nsizeof(GlobalData)\n%u\n",&global_data_size);assert(global_data_size==(uint32_t)sizeof(GlobalData));assert(rv==1);
    rv=fread(&c->global_data,sizeof(GlobalData),1,f);assert(rv==1);
    c->global_data.removed_bodies = host_ptr;
    rv=fread(c->global_data.removed_bodies,sizeof(c->global_data.removed_bodies[0]),c->global_data.removed_bodies_count,f);assert(rv==c->global_data.removed_bodies_count);    
    // c->userUint64
#   ifndef NUDGE_CONTEXT_STRUCT_NO_USER_DATA
    uint32_t user_size=0;
    rv=fscanf(f,"\nsizeof(c->user)\n%u\n",&user_size);assert(user_size==(uint32_t)sizeof(c->user));assert(rv==1);
    rv=fread(&c->user,sizeof(c->user),1,f);assert(rv==1);
#   endif // NUDGE_CONTEXT_STRUCT_NO_USER_DATA
    // Which fields of 'old_simulation_params' must we keep? I'd say nothing, all or 'num_frames' and 'num_total_substeps', but what to choose?
    c->simulation_params.num_frames = old_simulation_params.num_frames;
    c->simulation_params.num_total_substeps = old_simulation_params.num_total_substeps;
    // -----------------------------------------------------

    // But now we must reassign the remaining tags
    {
        //for (uint16_t i=0;i<c->MAX_NUM_BOXES;i++)    {c->colliders.boxes.tags[i] = i;}
        //for (uint16_t i=0;i<c->MAX_NUM_SPHERES;i++)    {c->colliders.spheres.tags[i] = NUDGE_START_SPHERE_TAG+i;}
        Arena arena = c->arena;
        const unsigned required_size = c->MAX_NUM_BOXES>c->MAX_NUM_SPHERES?c->MAX_NUM_BOXES:c->MAX_NUM_SPHERES;assert(arena.size>=required_size*sizeof(bool));
        bool* check_array = allocate_array<bool>(&arena, required_size, 32);assert(check_array);
        {
            bool* box_checks = check_array;
            for (unsigned i=0;i<c->MAX_NUM_BOXES;i++) box_checks[i]=false;
            for (uint32_t i=0;i<c->colliders.boxes.count;i++) {
                const uint16_t tag = c->colliders.boxes.tags[i];
                assert(tag<c->MAX_NUM_BOXES);
                assert(box_checks[tag]==false);
                box_checks[tag]=true;
            }
            uint32_t starti = c->colliders.boxes.count;
            for (uint32_t i=0;i<c->MAX_NUM_BOXES;i++) {
                if (!box_checks[i]) {
                    assert(starti<c->MAX_NUM_BOXES);
                    c->colliders.boxes.tags[starti++]=(uint16_t)i;
                }
            }
            assert(starti==c->MAX_NUM_BOXES);
        }
        {
            bool* sphere_checks = check_array;
            for (unsigned i=0;i<c->MAX_NUM_SPHERES;i++) sphere_checks[i]=false;
            for (uint32_t i=0;i<c->colliders.spheres.count;i++) {
                const uint16_t tag = c->colliders.spheres.tags[i];
                assert(tag>=NUDGE_START_SPHERE_TAG && tag<NUDGE_START_SPHERE_TAG+c->MAX_NUM_SPHERES);
                assert(sphere_checks[tag-NUDGE_START_SPHERE_TAG]==false);
                sphere_checks[tag-NUDGE_START_SPHERE_TAG]=true;
            }
            uint32_t starti = c->colliders.spheres.count;
            for (uint32_t i=0;i<c->MAX_NUM_SPHERES;i++) {
                if (!sphere_checks[i]) {
                    assert(starti<c->MAX_NUM_SPHERES);
                    c->colliders.spheres.tags[starti++]=(uint16_t)(NUDGE_START_SPHERE_TAG+i);
                }
            }
            assert(starti==c->MAX_NUM_SPHERES);
        }
    }

    // And we should reset
}

#   ifdef NUDGE_USE_TIME_CONTEXT
void save_time_context(FILE* f,const time_context_t* c) {
    assert(f && c);
    fprintf(f,"\nsizeof(time_context_t)\n%u\n",(uint32_t)sizeof(time_context_t));
    fwrite(c,sizeof(time_context_t),1,f);
}
void load_time_context(FILE* f,time_context_t* c)   {
    assert(f && c);
    uint32_t time_context_size=0;
    fscanf(f,"\nsizeof(time_context_t)\n%u\n",&time_context_size);assert(time_context_size==(uint32_t)sizeof(time_context_t));
    fread(c,sizeof(time_context_t),1,f);
}
#   endif //NUDGE_USE_TIME_CONTEXT

#endif //NUDGE_NO_STDIO

// -----------------------------------------------------------------------------------

static unsigned box_box_collide(uint32_t* pairs, unsigned pair_count, BoxCollider* colliders, Transform* transforms, Contact* contacts, BodyPair* bodies, uint64_t* tags, const BodyProperties* properties, Arena temporary) {
    // TODO: We may want to batch/chunk this for better cache behavior for repeatedly accessed data.
    // TODO: We should make use of 8-wide SIMD here as well.

    float* feature_penetrations = allocate_array<float>(&temporary, pair_count + 7, 32); // Padding is required.
    uint32_t* features = allocate_array<uint32_t>(&temporary, pair_count + 7, 32);

    unsigned count = 0;

    // Determine most separating face and reject pairs separated by a face.
    {
        pairs[pair_count+0] = 0; // Padding.
        pairs[pair_count+1] = 0;
        pairs[pair_count+2] = 0;

        unsigned added = 0;

        // Transform each box into the local space of the other in order to quickly determine per-face penetration.
        for (unsigned i = 0; i < pair_count; i += 4) {
            // Load pairs.
            unsigned pair0 = pairs[i+0];
            unsigned pair1 = pairs[i+1];
            unsigned pair2 = pairs[i+2];
            unsigned pair3 = pairs[i+3];

            unsigned a0_index = pair0 & 0xffff;
            unsigned b0_index = pair0 >> 16;

            unsigned a1_index = pair1 & 0xffff;
            unsigned b1_index = pair1 >> 16;

            unsigned a2_index = pair2 & 0xffff;
            unsigned b2_index = pair2 >> 16;

            unsigned a3_index = pair3 & 0xffff;
            unsigned b3_index = pair3 >> 16;

            // Load rotations.
            simd4_float a_rotation_x = simd_float::load4(transforms[a0_index].rotation);
            simd4_float a_rotation_y = simd_float::load4(transforms[a1_index].rotation);
            simd4_float a_rotation_z = simd_float::load4(transforms[a2_index].rotation);
            simd4_float a_rotation_s = simd_float::load4(transforms[a3_index].rotation);

            simd4_float b_rotation_x = simd_float::load4(transforms[b0_index].rotation);
            simd4_float b_rotation_y = simd_float::load4(transforms[b1_index].rotation);
            simd4_float b_rotation_z = simd_float::load4(transforms[b2_index].rotation);
            simd4_float b_rotation_s = simd_float::load4(transforms[b3_index].rotation);

            simd128::transpose32(a_rotation_x, a_rotation_y, a_rotation_z, a_rotation_s);
            simd128::transpose32(b_rotation_x, b_rotation_y, b_rotation_z, b_rotation_s);

            // Determine quaternion for rotation from a to b.
            simd4_float t_x, t_y, t_z;
            simd_soa::cross(b_rotation_x, b_rotation_y, b_rotation_z, a_rotation_x, a_rotation_y, a_rotation_z, t_x, t_y, t_z);

            simd4_float relative_rotation_x = a_rotation_x * b_rotation_s - b_rotation_x * a_rotation_s - t_x;
            simd4_float relative_rotation_y = a_rotation_y * b_rotation_s - b_rotation_y * a_rotation_s - t_y;
            simd4_float relative_rotation_z = a_rotation_z * b_rotation_s - b_rotation_z * a_rotation_s - t_z;
            simd4_float relative_rotation_s = (a_rotation_x * b_rotation_x +
                                               a_rotation_y * b_rotation_y +
                                               a_rotation_z * b_rotation_z +
                                               a_rotation_s * b_rotation_s);

            // Compute the corresponding matrix.
            // Note that the b to a matrix is simply the transpose of a to b.
            simd4_float kx = relative_rotation_x + relative_rotation_x;
            simd4_float ky = relative_rotation_y + relative_rotation_y;
            simd4_float kz = relative_rotation_z + relative_rotation_z;

            simd4_float xx = kx * relative_rotation_x;
            simd4_float yy = ky * relative_rotation_y;
            simd4_float zz = kz * relative_rotation_z;
            simd4_float xy = kx * relative_rotation_y;
            simd4_float xz = kx * relative_rotation_z;
            simd4_float yz = ky * relative_rotation_z;
            simd4_float sx = kx * relative_rotation_s;
            simd4_float sy = ky * relative_rotation_s;
            simd4_float sz = kz * relative_rotation_s;

            simd4_float one = simd_float::make4(1.0f);

            simd4_float vx_x = one - yy - zz;
            simd4_float vx_y = xy + sz;
            simd4_float vx_z = xz - sy;

            simd4_float vy_x = xy - sz;
            simd4_float vy_y = one - xx - zz;
            simd4_float vy_z = yz + sx;

            simd4_float vz_x = xz + sy;
            simd4_float vz_y = yz - sx;
            simd4_float vz_z = one - xx - yy;

            // Load sizes.
            simd4_float a_size_x = simd_float::load4(colliders[a0_index].size);
            simd4_float a_size_y = simd_float::load4(colliders[a1_index].size);
            simd4_float a_size_z = simd_float::load4(colliders[a2_index].size);
            simd4_float a_size_w = simd_float::load4(colliders[a3_index].size);

            simd4_float b_size_x = simd_float::load4(colliders[b0_index].size);
            simd4_float b_size_y = simd_float::load4(colliders[b1_index].size);
            simd4_float b_size_z = simd_float::load4(colliders[b2_index].size);
            simd4_float b_size_w = simd_float::load4(colliders[b3_index].size);

            simd128::transpose32(a_size_x, a_size_y, a_size_z, a_size_w);
            simd128::transpose32(b_size_x, b_size_y, b_size_z, b_size_w);

            // Compute the penetration.
            vx_x = simd_float::abs(vx_x);
            vx_y = simd_float::abs(vx_y);
            vx_z = simd_float::abs(vx_z);

            vy_x = simd_float::abs(vy_x);
            vy_y = simd_float::abs(vy_y);
            vy_z = simd_float::abs(vy_z);

            vz_x = simd_float::abs(vz_x);
            vz_y = simd_float::abs(vz_y);
            vz_z = simd_float::abs(vz_z);

            simd4_float pax = b_size_x + vx_x*a_size_x + vy_x*a_size_y + vz_x*a_size_z;
            simd4_float pay = b_size_y + vx_y*a_size_x + vy_y*a_size_y + vz_y*a_size_z;
            simd4_float paz = b_size_z + vx_z*a_size_x + vy_z*a_size_y + vz_z*a_size_z;

            simd4_float pbx = a_size_x + vx_x*b_size_x + vx_y*b_size_y + vx_z*b_size_z;
            simd4_float pby = a_size_y + vy_x*b_size_x + vy_y*b_size_y + vy_z*b_size_z;
            simd4_float pbz = a_size_z + vz_x*b_size_x + vz_y*b_size_y + vz_z*b_size_z;

            // Load positions.
            simd4_float a_position_x = simd_float::load4(transforms[a0_index].position);
            simd4_float a_position_y = simd_float::load4(transforms[a1_index].position);
            simd4_float a_position_z = simd_float::load4(transforms[a2_index].position);
            simd4_float a_position_w = simd_float::load4(transforms[a3_index].position);

            simd4_float b_position_x = simd_float::load4(transforms[b0_index].position);
            simd4_float b_position_y = simd_float::load4(transforms[b1_index].position);
            simd4_float b_position_z = simd_float::load4(transforms[b2_index].position);
            simd4_float b_position_w = simd_float::load4(transforms[b3_index].position);

            // Compute relative positions and offset the penetrations.
            simd4_float delta_x = a_position_x - b_position_x;
            simd4_float delta_y = a_position_y - b_position_y;
            simd4_float delta_z = a_position_z - b_position_z;
            simd4_float delta_w = a_position_w - b_position_w;

            simd128::transpose32(delta_x, delta_y, delta_z, delta_w);

            simd_soa::cross(b_rotation_x, b_rotation_y, b_rotation_z, delta_x, delta_y, delta_z, t_x, t_y, t_z);
            t_x += t_x;
            t_y += t_y;
            t_z += t_z;

            simd4_float u_x, u_y, u_z;
            simd_soa::cross(b_rotation_x, b_rotation_y, b_rotation_z, t_x, t_y, t_z, u_x, u_y, u_z);

            simd4_float a_offset_x = u_x + delta_x - b_rotation_s * t_x;
            simd4_float a_offset_y = u_y + delta_y - b_rotation_s * t_y;
            simd4_float a_offset_z = u_z + delta_z - b_rotation_s * t_z;

            pax -= simd_float::abs(a_offset_x);
            pay -= simd_float::abs(a_offset_y);
            paz -= simd_float::abs(a_offset_z);

            simd_soa::cross(delta_x, delta_y, delta_z, a_rotation_x, a_rotation_y, a_rotation_z, t_x, t_y, t_z);
            t_x += t_x;
            t_y += t_y;
            t_z += t_z;

            simd_soa::cross(a_rotation_x, a_rotation_y, a_rotation_z, t_x, t_y, t_z, u_x, u_y, u_z);

            simd4_float b_offset_x = u_x - delta_x - a_rotation_s * t_x;
            simd4_float b_offset_y = u_y - delta_y - a_rotation_s * t_y;
            simd4_float b_offset_z = u_z - delta_z - a_rotation_s * t_z;

            pbx -= simd_float::abs(b_offset_x);
            pby -= simd_float::abs(b_offset_y);
            pbz -= simd_float::abs(b_offset_z);

            // Reduce face penetrations.
            simd4_float payz = simd_float::min(pay, paz);
            simd4_float pbyz = simd_float::min(pby, pbz);

            simd4_float pa = simd_float::min(pax, payz);
            simd4_float pb = simd_float::min(pbx, pbyz);

            simd4_float p = simd_float::min(pa, pb);

            // Determine the best aligned face for each collider.
            simd4_float aymf = simd_float::cmp_eq(payz, pa);
            simd4_float azmf = simd_float::cmp_eq(paz, pa);

            simd4_float bymf = simd_float::cmp_eq(pbyz, pb);
            simd4_float bzmf = simd_float::cmp_eq(pbz, pb);

            simd4_int32 aymi = simd::bitwise_and(simd_float::asint(aymf), simd_int32::make4(1));
            simd4_int32 azmi = simd::bitwise_and(simd_float::asint(azmf), simd_int32::make4(1));

            simd4_int32 bymi = simd::bitwise_and(simd_float::asint(bymf), simd_int32::make4(1));
            simd4_int32 bzmi = simd::bitwise_and(simd_float::asint(bzmf), simd_int32::make4(1));

            simd4_int32 aface = simd_int32::add(aymi, azmi);
            simd4_int32 bface = simd_int32::add(bymi, bzmi);

            // Swap so that collider a has the most separating face.
            simd4_float swap = simd_float::cmp_eq(pa, p);

            simd4_float pair_a_b = simd_int32::asfloat(simd_int32::load4((const int32_t*)(pairs + i)));
            simd4_float pair_b_a = simd_int32::asfloat(simd::bitwise_or(simd_int32::shift_left<16>(simd_float::asint(pair_a_b)), simd_int32::shift_right<16>(simd_float::asint(pair_a_b))));

            simd4_float face = simd::blendv32(simd_int32::asfloat(bface), simd_int32::asfloat(aface), swap);
            simd4_float pair = simd::blendv32(pair_a_b, pair_b_a, swap);

            // Store data for pairs with positive penetration.
            unsigned mask = simd::signmask32(simd_float::cmp_gt(p, simd_float::zero4()));

            NUDGE_ALIGNED(16) float face_penetration_array[4];
            NUDGE_ALIGNED(16) uint32_t face_array[4];
            NUDGE_ALIGNED(16) uint32_t pair_array[4];

            simd_float::store4(face_penetration_array, p);
            simd_float::store4((float*)face_array, face);
            simd_float::store4((float*)pair_array, pair);

            while (mask) {
                unsigned index = first_set_bit(mask);
                mask &= mask-1;

                feature_penetrations[added] = face_penetration_array[index];
                features[added] = face_array[index];
                pairs[added] = pair_array[index];

                ++added;
            }
        }

        // Erase padding.
        while (added && !pairs[added-1])
            --added;

        pair_count = added;
    }

    // Check if edge pairs are more separating.
    // Do face-face test if not.
    {
        pairs[pair_count+0] = 0; // Padding.
        pairs[pair_count+1] = 0;
        pairs[pair_count+2] = 0;

        feature_penetrations[pair_count+0] = 0.0f;
        feature_penetrations[pair_count+1] = 0.0f;
        feature_penetrations[pair_count+2] = 0.0f;

        unsigned added = 0;

        for (unsigned pair_offset = 0; pair_offset < pair_count; pair_offset += 4) {
            // Load pairs.
            unsigned pair0 = pairs[pair_offset+0];
            unsigned pair1 = pairs[pair_offset+1];
            unsigned pair2 = pairs[pair_offset+2];
            unsigned pair3 = pairs[pair_offset+3];

            unsigned a0_index = pair0 & 0xffff;
            unsigned b0_index = pair0 >> 16;

            unsigned a1_index = pair1 & 0xffff;
            unsigned b1_index = pair1 >> 16;

            unsigned a2_index = pair2 & 0xffff;
            unsigned b2_index = pair2 >> 16;

            unsigned a3_index = pair3 & 0xffff;
            unsigned b3_index = pair3 >> 16;

            // Load rotations.
            simd4_float a_rotation_x = simd_float::load4(transforms[a0_index].rotation);
            simd4_float a_rotation_y = simd_float::load4(transforms[a1_index].rotation);
            simd4_float a_rotation_z = simd_float::load4(transforms[a2_index].rotation);
            simd4_float a_rotation_s = simd_float::load4(transforms[a3_index].rotation);

            simd4_float b_rotation_x = simd_float::load4(transforms[b0_index].rotation);
            simd4_float b_rotation_y = simd_float::load4(transforms[b1_index].rotation);
            simd4_float b_rotation_z = simd_float::load4(transforms[b2_index].rotation);
            simd4_float b_rotation_s = simd_float::load4(transforms[b3_index].rotation);

            simd128::transpose32(a_rotation_x, a_rotation_y, a_rotation_z, a_rotation_s);
            simd128::transpose32(b_rotation_x, b_rotation_y, b_rotation_z, b_rotation_s);

            // Determine quaternion for rotation from a to b.
            simd4_float t_x, t_y, t_z;
            simd_soa::cross(b_rotation_x, b_rotation_y, b_rotation_z, a_rotation_x, a_rotation_y, a_rotation_z, t_x, t_y, t_z);

            simd4_float relative_rotation_x = a_rotation_x * b_rotation_s - b_rotation_x * a_rotation_s - t_x;
            simd4_float relative_rotation_y = a_rotation_y * b_rotation_s - b_rotation_y * a_rotation_s - t_y;
            simd4_float relative_rotation_z = a_rotation_z * b_rotation_s - b_rotation_z * a_rotation_s - t_z;
            simd4_float relative_rotation_s = (a_rotation_x * b_rotation_x +
                                               a_rotation_y * b_rotation_y +
                                               a_rotation_z * b_rotation_z +
                                               a_rotation_s * b_rotation_s);

            // Compute the corresponding matrix.
            // Note that the b to a matrix is simply the transpose of a to b.
            simd4_float kx = relative_rotation_x + relative_rotation_x;
            simd4_float ky = relative_rotation_y + relative_rotation_y;
            simd4_float kz = relative_rotation_z + relative_rotation_z;

            simd4_float xx = kx * relative_rotation_x;
            simd4_float yy = ky * relative_rotation_y;
            simd4_float zz = kz * relative_rotation_z;
            simd4_float xy = kx * relative_rotation_y;
            simd4_float xz = kx * relative_rotation_z;
            simd4_float yz = ky * relative_rotation_z;
            simd4_float sx = kx * relative_rotation_s;
            simd4_float sy = ky * relative_rotation_s;
            simd4_float sz = kz * relative_rotation_s;

            simd4_float one = simd_float::make4(1.0f);

            simd4_float vx_x = one - yy - zz;
            simd4_float vx_y = xy + sz;
            simd4_float vx_z = xz - sy;

            simd4_float vy_x = xy - sz;
            simd4_float vy_y = one - xx - zz;
            simd4_float vy_z = yz + sx;

            simd4_float vz_x = xz + sy;
            simd4_float vz_y = yz - sx;
            simd4_float vz_z = one - xx - yy;

            NUDGE_ALIGNED(16) float a_to_b[4*9];

            simd_float::store4(a_to_b + 0, vx_x);
            simd_float::store4(a_to_b + 4, vx_y);
            simd_float::store4(a_to_b + 8, vx_z);

            simd_float::store4(a_to_b + 12, vy_x);
            simd_float::store4(a_to_b + 16, vy_y);
            simd_float::store4(a_to_b + 20, vy_z);

            simd_float::store4(a_to_b + 24, vz_x);
            simd_float::store4(a_to_b + 28, vz_y);
            simd_float::store4(a_to_b + 32, vz_z);

            // Load sizes.
            simd4_float a_size_x = simd_float::load4(colliders[a0_index].size);
            simd4_float a_size_y = simd_float::load4(colliders[a1_index].size);
            simd4_float a_size_z = simd_float::load4(colliders[a2_index].size);
            simd4_float a_size_w = simd_float::load4(colliders[a3_index].size);

            simd4_float b_size_x = simd_float::load4(colliders[b0_index].size);
            simd4_float b_size_y = simd_float::load4(colliders[b1_index].size);
            simd4_float b_size_z = simd_float::load4(colliders[b2_index].size);
            simd4_float b_size_w = simd_float::load4(colliders[b3_index].size);

            simd128::transpose32(a_size_x, a_size_y, a_size_z, a_size_w);
            simd128::transpose32(b_size_x, b_size_y, b_size_z, b_size_w);

            // Load positions.
            simd4_float a_position_x = simd_float::load4(transforms[a0_index].position);
            simd4_float a_position_y = simd_float::load4(transforms[a1_index].position);
            simd4_float a_position_z = simd_float::load4(transforms[a2_index].position);
            simd4_float a_position_w = simd_float::load4(transforms[a3_index].position);

            simd4_float b_position_x = simd_float::load4(transforms[b0_index].position);
            simd4_float b_position_y = simd_float::load4(transforms[b1_index].position);
            simd4_float b_position_z = simd_float::load4(transforms[b2_index].position);
            simd4_float b_position_w = simd_float::load4(transforms[b3_index].position);

            // Compute relative positions and offset the penetrations.
            simd4_float delta_x = a_position_x - b_position_x;
            simd4_float delta_y = a_position_y - b_position_y;
            simd4_float delta_z = a_position_z - b_position_z;
            simd4_float delta_w = a_position_w - b_position_w;

            simd128::transpose32(delta_x, delta_y, delta_z, delta_w);

            simd_soa::cross(delta_x, delta_y, delta_z, a_rotation_x, a_rotation_y, a_rotation_z, t_x, t_y, t_z);
            t_x += t_x;
            t_y += t_y;
            t_z += t_z;

            simd4_float u_x, u_y, u_z;
            simd_soa::cross(a_rotation_x, a_rotation_y, a_rotation_z, t_x, t_y, t_z, u_x, u_y, u_z);

            simd4_float b_offset_x = u_x - delta_x - a_rotation_s * t_x;
            simd4_float b_offset_y = u_y - delta_y - a_rotation_s * t_y;
            simd4_float b_offset_z = u_z - delta_z - a_rotation_s * t_z;

            NUDGE_ALIGNED(16) float b_offset_array[3*4];

            simd_float::store4(b_offset_array + 0, b_offset_x);
            simd_float::store4(b_offset_array + 4, b_offset_y);
            simd_float::store4(b_offset_array + 8, b_offset_z);

            simd4_float face_penetration = simd_float::load4(feature_penetrations + pair_offset);

            // Is an edge pair more separating?
            NUDGE_ALIGNED(16) float edge_penetration_a[4*9];
            NUDGE_ALIGNED(16) float edge_penetration_b[4*9];

            for (unsigned i = 0; i < 3; ++i) {
                simd4_float acx = simd_float::load4(a_to_b + (0*3 + i)*4);
                simd4_float acy = simd_float::load4(a_to_b + (1*3 + i)*4);
                simd4_float acz = simd_float::load4(a_to_b + (2*3 + i)*4);

                simd4_float bcx = simd_float::load4(a_to_b + (i*3 + 0)*4);
                simd4_float bcy = simd_float::load4(a_to_b + (i*3 + 1)*4);
                simd4_float bcz = simd_float::load4(a_to_b + (i*3 + 2)*4);

                simd4_float ac2x = acx*acx;
                simd4_float ac2y = acy*acy;
                simd4_float ac2z = acz*acz;

                simd4_float bc2x = bcx*bcx;
                simd4_float bc2y = bcy*bcy;
                simd4_float bc2z = bcz*bcz;

                simd4_float aacx = simd_float::abs(acx);
                simd4_float aacy = simd_float::abs(acy);
                simd4_float aacz = simd_float::abs(acz);

                simd4_float abcx = simd_float::abs(bcx);
                simd4_float abcy = simd_float::abs(bcy);
                simd4_float abcz = simd_float::abs(bcz);

                simd4_float r_a0 = ac2y + ac2z;
                simd4_float r_a1 = ac2z + ac2x;
                simd4_float r_a2 = ac2x + ac2y;

                simd4_float r_b0 = bc2y + bc2z;
                simd4_float r_b1 = bc2z + bc2x;
                simd4_float r_b2 = bc2x + bc2y;

                simd4_float nan_threshold = simd_float::make4(1e-3f);

                r_a0 = simd::bitwise_or(simd_float::rsqrt(r_a0), simd_float::cmp_le(r_a0, nan_threshold));
                r_a1 = simd::bitwise_or(simd_float::rsqrt(r_a1), simd_float::cmp_le(r_a1, nan_threshold));
                r_a2 = simd::bitwise_or(simd_float::rsqrt(r_a2), simd_float::cmp_le(r_a2, nan_threshold));

                r_b0 = simd::bitwise_or(simd_float::rsqrt(r_b0), simd_float::cmp_le(r_b0, nan_threshold));
                r_b1 = simd::bitwise_or(simd_float::rsqrt(r_b1), simd_float::cmp_le(r_b1, nan_threshold));
                r_b2 = simd::bitwise_or(simd_float::rsqrt(r_b2), simd_float::cmp_le(r_b2, nan_threshold));

                simd4_float pa0 = aacy*a_size_z + aacz*a_size_y;
                simd4_float pa1 = aacz*a_size_x + aacx*a_size_z;
                simd4_float pa2 = aacx*a_size_y + aacy*a_size_x;

                simd4_float pb0 = abcy*b_size_z + abcz*b_size_y;
                simd4_float pb1 = abcz*b_size_x + abcx*b_size_z;
                simd4_float pb2 = abcx*b_size_y + abcy*b_size_x;

                simd4_float o0 = simd_float::abs(acy*b_offset_z - acz*b_offset_y);
                simd4_float o1 = simd_float::abs(acz*b_offset_x - acx*b_offset_z);
                simd4_float o2 = simd_float::abs(acx*b_offset_y - acy*b_offset_x);

                simd_float::store4(edge_penetration_a + (i*3 + 0)*4, (pa0 - o0) * r_a0);
                simd_float::store4(edge_penetration_a + (i*3 + 1)*4, (pa1 - o1) * r_a1);
                simd_float::store4(edge_penetration_a + (i*3 + 2)*4, (pa2 - o2) * r_a2);

                simd_float::store4(edge_penetration_b + (i*3 + 0)*4, pb0 * r_b0);
                simd_float::store4(edge_penetration_b + (i*3 + 1)*4, pb1 * r_b1);
                simd_float::store4(edge_penetration_b + (i*3 + 2)*4, pb2 * r_b2);
            }

            simd4_int32 a_edge = simd_int32::make4(0);
            simd4_int32 b_edge = simd_int32::make4(0);

            simd4_float penetration = face_penetration;

            for (unsigned i = 0; i < 3; ++i) {
                for (unsigned j = 0; j < 3; ++j) {
                    simd4_float p = simd_float::load4(edge_penetration_a + (i*3 + j)*4) + simd_float::load4(edge_penetration_b + (j*3 + i)*4);

                    simd4_float mask = simd_float::cmp_gt(penetration, p);

                    penetration = simd_float::min(penetration, p); // Note: First operand is returned on NaN.
                    a_edge = simd::blendv32(a_edge, simd_int32::make4(j), simd_float::asint(mask));
                    b_edge = simd::blendv32(b_edge, simd_int32::make4(i), simd_float::asint(mask));
                }
            }

            simd4_float face_bias = simd_float::make4(1e-3f);

            unsigned edge = simd::signmask32(simd_float::cmp_gt(face_penetration, penetration + face_bias));
            unsigned overlapping = simd::signmask32(simd_float::cmp_gt(penetration, simd_float::zero4()));

            unsigned face = ~edge;

            edge &= overlapping;
            face &= overlapping;

            NUDGE_ALIGNED(16) float penetration_array[4];
            NUDGE_ALIGNED(16) int32_t a_edge_array[4];
            NUDGE_ALIGNED(16) int32_t b_edge_array[4];

            simd_float::store4(penetration_array, penetration);
            simd_int32::store4(a_edge_array, a_edge);
            simd_int32::store4(b_edge_array, b_edge);

            // Do face-face tests.
            while (face) {
                unsigned index = first_set_bit(face);
                face &= face-1;

                unsigned pair = pairs[pair_offset + index];
                unsigned a_face = features[pair_offset + index];

                unsigned a_index = pair & 0xffff;
                unsigned b_index = pair >> 16;

                // Gather.
                simd4_float dirs = simd_float::make4(a_to_b[(a_face*3 + 0)*4 + index],
                                                     a_to_b[(a_face*3 + 1)*4 + index],
                                                     a_to_b[(a_face*3 + 2)*4 + index],
                                                     0.0f);

                simd4_float c0 = simd_float::make4(a_to_b[(0*3 + 0)*4 + index],
                                                   a_to_b[(1*3 + 0)*4 + index],
                                                   a_to_b[(2*3 + 0)*4 + index],
                                                   0.0f);

                simd4_float c1 = simd_float::make4(a_to_b[(0*3 + 1)*4 + index],
                                                   a_to_b[(1*3 + 1)*4 + index],
                                                   a_to_b[(2*3 + 1)*4 + index],
                                                   0.0f);

                simd4_float c2 = simd_float::make4(a_to_b[(0*3 + 2)*4 + index],
                                                   a_to_b[(1*3 + 2)*4 + index],
                                                   a_to_b[(2*3 + 2)*4 + index],
                                                   0.0f);

                simd4_float b_offset = simd_float::make4(b_offset_array[0*4 + index],
                                                         b_offset_array[1*4 + index],
                                                         b_offset_array[2*4 + index],
                                                         0.0f);

                // Load sizes.
                simd4_float a_size = simd_float::load4(colliders[a_index].size);
                simd4_float b_size = simd_float::load4(colliders[b_index].size);

                // Find most aligned face of b.
                dirs = simd_float::abs(dirs);

                simd4_float max_dir = simd_float::max(simd128::shuffle32<0,2,1,3>(dirs), simd128::shuffle32<0,0,0,0>(dirs));

                unsigned dir_mask = simd::signmask32(simd_float::cmp_ge(dirs, max_dir));

                // Compute the coordinates of the two quad faces.
                c0 *= simd128::shuffle32<0,0,0,0>(b_size);
                c1 *= simd128::shuffle32<1,1,1,1>(b_size);
                c2 *= simd128::shuffle32<2,2,2,2>(b_size);

                unsigned b_face = 0;

                if (dir_mask & 4) {
                    simd4_float t = c0;
                    c0 = c2;
                    c2 = c1;
                    c1 = t;
                    b_face = 2;
                }
                else if (dir_mask & 2) {
                    simd4_float t = c0;
                    c0 = c1;
                    c1 = c2;
                    c2 = t;
                    b_face = 1;
                }

                simd4_float c = c0;
                simd4_float dx = c1;
                simd4_float dy = c2;

                unsigned b_positive_face_bit = simd::signmask32(simd::bitwise_xor(b_offset, c)) & (1 << a_face);
                unsigned b_offset_neg = simd::signmask32(b_offset) & (1 << a_face);

                if (!b_positive_face_bit)
                    c = -c;

                c += b_offset;

                // Quad coordinate packing:
                // Size of quad a, center of quad b, x-axis of quad b, y-axis of quad b.
                // a.size.x, c.x, dx.x, dy.x
                // a.size.y, c.y, dx.y, dy.y
                // a.size.z, c.z, dx.z, dy.z
                NUDGE_ALIGNED(16) float quads[4*3];

                simd4_float q0 = simd128::unpacklo32(a_size, c);
                simd4_float q1 = simd128::unpackhi32(a_size, c);
                simd4_float q2 = simd128::unpacklo32(dx, dy);
                simd4_float q3 = simd128::unpackhi32(dx, dy);

                simd_float::store4(quads + 0, simd128::concat2x32<0,1,0,1>(q0, q2));
                simd_float::store4(quads + 4, simd128::concat2x32<2,3,2,3>(q0, q2));
                simd_float::store4(quads + 8, simd128::concat2x32<0,1,0,1>(q1, q3));

                // Transform so that overlap testing can be done in two dimensions.
                const float* transformed_x = quads + 4*((a_face+1) % 3);
                const float* transformed_y = quads + 4*((a_face+2) % 3);
                const float* transformed_z = quads + 4*a_face;

                // Find support points for the overlap between the quad faces in two dimensions.
                NUDGE_ALIGNED(32) float support[16*3];
                NUDGE_ALIGNED(32) uint32_t support_tags[16];
                unsigned mask; // Indicates valid points.
                {
                    float* support_x = support + 0;
                    float* support_y = support + 16;

                    simd4_float tx = simd_float::load4(transformed_x);
                    simd4_float ty = simd_float::load4(transformed_y);

                    simd4_float sxycxy = simd128::unpacklo32(tx, ty);
                    simd4_float dxy = simd128::unpackhi32(tx, ty);

                    simd4_float sx = simd128::shuffle32<0,0,0,0>(sxycxy);
                    simd4_float sy = simd128::shuffle32<1,1,1,1>(sxycxy);
                    simd4_float cx = simd128::shuffle32<2,2,2,2>(sxycxy);
                    simd4_float cy = simd128::shuffle32<3,3,3,3>(sxycxy);

                    simd4_float sign_npnp = simd_float::make4(-0.0f, 0.0f, -0.0f, 0.0f);

                    // Add corner points to the support if they are part of the intersection.
                    __m128i corner_mask;
                    __m128i edge_mask;
                    {
                        simd4_float sign_pnpn = simd_float::make4(0.0f, -0.0f, 0.0f, -0.0f);
                        simd4_float sign_nnpp = simd_float::make4(-0.0f, -0.0f, 0.0f, 0.0f);

                        simd4_float corner0x = simd::bitwise_xor(sx, sign_pnpn);
                        simd4_float corner0y = simd::bitwise_xor(sy, sign_nnpp);

                        simd4_float corner1x = cx + simd::bitwise_xor(simd128::shuffle32<0,0,0,0>(dxy), sign_npnp) + simd::bitwise_xor(simd128::shuffle32<2,2,2,2>(dxy), sign_nnpp);
                        simd4_float corner1y = cy + simd::bitwise_xor(simd128::shuffle32<1,1,1,1>(dxy), sign_npnp) + simd::bitwise_xor(simd128::shuffle32<3,3,3,3>(dxy), sign_nnpp);

                        simd4_float k = (simd128::concat2x32<2,2,0,0>(sxycxy, dxy) * simd128::shuffle32<3,1,3,1>(dxy) -
                                         simd128::concat2x32<3,3,1,1>(sxycxy, dxy) * simd128::shuffle32<2,0,2,0>(dxy));

                        simd4_float ox = simd128::shuffle32<0,0,0,0>(k);
                        simd4_float oy = simd128::shuffle32<1,1,1,1>(k);
                        simd4_float delta_max = simd_float::abs(simd128::shuffle32<2,2,2,2>(k));

                        simd4_float sdxy = dxy * simd128::shuffle32<1,0,1,0>(sxycxy);

                        simd4_float delta_x = ox + simd::bitwise_xor(simd128::shuffle32<2,2,2,2>(sdxy), sign_nnpp) + simd::bitwise_xor(simd128::shuffle32<3,3,3,3>(sdxy), sign_npnp);
                        simd4_float delta_y = oy + simd::bitwise_xor(simd128::shuffle32<0,0,0,0>(sdxy), sign_nnpp) + simd::bitwise_xor(simd128::shuffle32<1,1,1,1>(sdxy), sign_npnp);

                        simd4_float inside_x = simd_float::cmp_le(simd_float::abs(corner1x), sx);
                        simd4_float inside_y = simd_float::cmp_le(simd_float::abs(corner1y), sy);

                        simd4_float mask0 = simd_float::cmp_le(simd_float::max(simd_float::abs(delta_x), simd_float::abs(delta_y)), delta_max);
                        simd4_float mask1 = simd::bitwise_and(inside_x, inside_y);

                        corner_mask = _mm_packs_epi32(simd_float::asint(mask0), simd_float::asint(mask1));

                        // Don't allow edge intersections if both vertices are inside.
                        edge_mask = _mm_packs_epi32(simd_float::asint(simd::bitwise_and(simd128::shuffle32<3,2,0,2>(mask0), simd128::shuffle32<1,0,1,3>(mask0))),
                                                    simd_float::asint(simd::bitwise_and(simd128::shuffle32<1,3,2,3>(mask1), simd128::shuffle32<0,2,0,1>(mask1))));

                        simd_float::store4(support_x + 0, corner0x);
                        simd_float::store4(support_y + 0, corner0y);
                        simd_float::store4(support_x + 4, corner1x);
                        simd_float::store4(support_y + 4, corner1y);
                    }

                    // Find additional support points by intersecting the edges of the second quad against the bounds of the first.
                    unsigned edge_axis_near;
                    unsigned edge_axis_far;
                    {
                        simd4_float one = simd_float::make4(1.0f);
                        simd4_float rdxy = one/dxy;

                        simd4_float offset_x = simd128::shuffle32<0,0,2,2>(dxy);
                        simd4_float offset_y = simd128::shuffle32<1,1,3,3>(dxy);

                        simd4_float pivot_x = cx + simd::bitwise_xor(simd128::shuffle32<2,2,0,0>(dxy), sign_npnp);
                        simd4_float pivot_y = cy + simd::bitwise_xor(simd128::shuffle32<3,3,1,1>(dxy), sign_npnp);

                        simd4_float sign_mask = simd_float::make4(-0.0f);
                        simd4_float pos_x = simd::bitwise_or(simd::bitwise_and(offset_x, sign_mask), sx); // Copy sign.
                        simd4_float pos_y = simd::bitwise_or(simd::bitwise_and(offset_y, sign_mask), sy);

                        simd4_float rx = simd128::shuffle32<0,0,2,2>(rdxy);
                        simd4_float ry = simd128::shuffle32<1,1,3,3>(rdxy);

                        simd4_float near_x = (pos_x + pivot_x) * rx;
                        simd4_float far_x = (pos_x - pivot_x) * rx;

                        simd4_float near_y = (pos_y + pivot_y) * ry;
                        simd4_float far_y = (pos_y - pivot_y) * ry;

                        simd4_float a = simd_float::min(one, near_x); // First operand is returned on NaN.
                        simd4_float b = simd_float::min(one, far_x);

                        edge_axis_near = simd::signmask32(simd_float::cmp_gt(a, near_y));
                        edge_axis_far = simd::signmask32(simd_float::cmp_gt(b, far_y));

                        a = simd_float::min(a, near_y);
                        b = simd_float::min(b, far_y);

                        simd4_float ax = pivot_x - offset_x * a;
                        simd4_float ay = pivot_y - offset_y * a;
                        simd4_float bx = pivot_x + offset_x * b;
                        simd4_float by = pivot_y + offset_y * b;

                        simd4_float mask = simd_float::cmp_gt(a + b, simd_float::zero4()); // Make sure -a < b.

                        simd4_float mask_a = simd_float::cmp_neq(a, one);
                        simd4_float mask_b = simd_float::cmp_neq(b, one);

                        mask_a = simd::bitwise_and(mask_a, mask);
                        mask_b = simd::bitwise_and(mask_b, mask);

                        edge_mask = simd::bitwise_notand(edge_mask, _mm_packs_epi32(simd_float::asint(mask_a), simd_float::asint(mask_b)));

                        simd_float::store4(support_x + 8, ax);
                        simd_float::store4(support_y + 8, ay);
                        simd_float::store4(support_x + 12, bx);
                        simd_float::store4(support_y + 12, by);
                    }

                    mask = _mm_movemask_epi8(_mm_packs_epi16(corner_mask, edge_mask));

                    // Calculate and store vertex labels.
                    // The 8 vertices are tagged using the sign bit of each axis.
                    // Bit rotation is used to "transform" the coordinates.
                    unsigned a_sign_face_bit = b_offset_neg ? (1 << a_face) : 0;
                    unsigned b_sign_face_bit = b_positive_face_bit ? 0 : (1 << b_face);

                    unsigned a_vertices = 0x12003624 >> (3 - a_face); // Rotates all vertices in parallel.
                    unsigned b_vertices = 0x00122436 >> (3 - b_face);

                    unsigned a_face_bits = 0xffff0000 | a_sign_face_bit;
                    unsigned b_face_bits = 0x0000ffff | (b_sign_face_bit << 16);

                    support_tags[0] = ((a_vertices >>  0) & 0x7) | a_face_bits;
                    support_tags[1] = ((a_vertices >>  8) & 0x7) | a_face_bits;
                    support_tags[2] = ((a_vertices >> 16) & 0x7) | a_face_bits;
                    support_tags[3] = ((a_vertices >> 24) & 0x7) | a_face_bits;

                    support_tags[4] = ((b_vertices << 16) & 0x70000) | b_face_bits;
                    support_tags[5] = ((b_vertices <<  8) & 0x70000) | b_face_bits;
                    support_tags[6] = ((b_vertices >>  0) & 0x70000) | b_face_bits;
                    support_tags[7] = ((b_vertices >>  8) & 0x70000) | b_face_bits;

                    // Calculate edge numbers in the local coordinate frame.
                    unsigned edge_axis_winding = simd::signmask32(dxy);

                    unsigned y_near0 = (edge_axis_near >> 0) & 1;
                    unsigned y_near1 = (edge_axis_near >> 1) & 1;
                    unsigned y_near2 = (edge_axis_near >> 2) & 1;
                    unsigned y_near3 = (edge_axis_near >> 3) & 1;

                    unsigned y_far0 = (edge_axis_far >> 0) & 1;
                    unsigned y_far1 = (edge_axis_far >> 1) & 1;
                    unsigned y_far2 = (edge_axis_far >> 2) & 1;
                    unsigned y_far3 = (edge_axis_far >> 3) & 1;

                    unsigned a_near_edge0 = y_near0*2 + ((edge_axis_winding >> (0 + y_near0)) & 1);
                    unsigned a_near_edge1 = y_near1*2 + ((edge_axis_winding >> (0 + y_near1)) & 1);
                    unsigned a_near_edge2 = y_near2*2 + ((edge_axis_winding >> (2 + y_near2)) & 1);
                    unsigned a_near_edge3 = y_near3*2 + ((edge_axis_winding >> (2 + y_near3)) & 1);

                    edge_axis_winding ^= 0xf;

                    unsigned a_far_edge0 = y_far0*2 + ((edge_axis_winding >> (0 + y_far0)) & 1);
                    unsigned a_far_edge1 = y_far1*2 + ((edge_axis_winding >> (0 + y_far1)) & 1);
                    unsigned a_far_edge2 = y_far2*2 + ((edge_axis_winding >> (2 + y_far2)) & 1);
                    unsigned a_far_edge3 = y_far3*2 + ((edge_axis_winding >> (2 + y_far3)) & 1);

                    // Map local edges to labels (so that faces can share an edge).
                    // The 12 edges are tagged using two ordered points.
                    // We use the same trick as the vertex transform but do it for pairs of vertices (in correct order).
                    uint64_t a_edge_map = 0x1200362424003612llu >> (3 - a_face);
                    uint64_t b_edge_map = 0x2400361212003624llu >> (3 - b_face);

                    unsigned face_bits = a_sign_face_bit | (a_sign_face_bit << 8) | (b_sign_face_bit << 16) | (b_sign_face_bit << 24);

                    unsigned b_edge0 = ((unsigned)((b_edge_map >> (0<<4)) & 0x0707) << 16) | face_bits;
                    unsigned b_edge1 = ((unsigned)((b_edge_map >> (1<<4)) & 0x0707) << 16) | face_bits;
                    unsigned b_edge2 = ((unsigned)((b_edge_map >> (2<<4)) & 0x0707) << 16) | face_bits;
                    unsigned b_edge3 = ((unsigned)((b_edge_map >> (3<<4)) & 0x0707) << 16) | face_bits;

                    support_tags[ 8] = (unsigned)((a_edge_map >> (a_near_edge0<<4)) & 0x0707) | b_edge0;
                    support_tags[ 9] = (unsigned)((a_edge_map >> (a_near_edge1<<4)) & 0x0707) | b_edge1;
                    support_tags[10] = (unsigned)((a_edge_map >> (a_near_edge2<<4)) & 0x0707) | b_edge2;
                    support_tags[11] = (unsigned)((a_edge_map >> (a_near_edge3<<4)) & 0x0707) | b_edge3;

                    support_tags[12] = (unsigned)((a_edge_map >> (a_far_edge0<<4)) & 0x0707) | b_edge0;
                    support_tags[13] = (unsigned)((a_edge_map >> (a_far_edge1<<4)) & 0x0707) | b_edge1;
                    support_tags[14] = (unsigned)((a_edge_map >> (a_far_edge2<<4)) & 0x0707) | b_edge2;
                    support_tags[15] = (unsigned)((a_edge_map >> (a_far_edge3<<4)) & 0x0707) | b_edge3;
                }

                // Compute z-plane through face b and calculate z for the support points.
                simd4_float a_size_transformed = simd_float::load4(transformed_x);
                simd4_float c_transformed = simd_float::load4(transformed_y);
                simd4_float dx_transformed = simd_float::load4(transformed_z);
                simd4_float dy_transformed = simd_float::zero4();

                simd128::transpose32(a_size_transformed, c_transformed, dx_transformed, dy_transformed);

                simd4_float zn = simd_aos::cross(dx_transformed, dy_transformed);
                simd4_float plane = simd128::concat2x32<0,1,0,1>(simd::bitwise_xor(zn, simd_float::make4(-0.0f)), simd_aos::dot(c_transformed, zn));
                plane *= simd_float::make4(1.0f)/simd128::shuffle32<2,2,2,2>(zn);

                NUDGE_ALIGNED(32) float penetrations[16];

                simdv_float z_sign = simd_float::zerov();

                if (b_offset_neg)
                    z_sign = simd_float::makev(-0.0f);

#if NUDGE_SIMDV_WIDTH == 256
                simdv_float penetration_offset = simd256::broadcast(simd128::shuffle32<2,2,2,2>(a_size_transformed));
                simdv_float plane256 = simd256::broadcast(plane);
#else
                simdv_float penetration_offset = simd128::shuffle32<2,2,2,2>(a_size_transformed);
#endif
                unsigned penetration_mask = 0;

                for (unsigned i = 0; i < 16; i += simdv_width32) {
#if NUDGE_SIMDV_WIDTH == 256
                    simdv_float plane = plane256;
#endif

                    simdv_float x = simd_float::loadv(support + 0 + i);
                    simdv_float y = simd_float::loadv(support + 16 + i);
                    simdv_float z = x*simd128::shuffle32<0,0,0,0>(plane) + y*simd128::shuffle32<1,1,1,1>(plane) + simd128::shuffle32<2,2,2,2>(plane);

                    simdv_float penetration = penetration_offset - simd::bitwise_xor(z, z_sign);

                    z += penetration * simd::bitwise_xor(simd_float::makev(0.5f), z_sign);

                    penetration_mask |= simd::signmask32(simd_float::cmp_gt(penetration, simd_float::zerov())) << i;

                    simd_float::storev(penetrations + i, penetration);
                    simd_float::storev(support + 32 + i, z);
                }

                mask &= penetration_mask;

                // Inverse transform.
                unsigned a_face_inverse = (a_face ^ 1) ^ (a_face >> 1);

                const float* support_x = support + 16*((a_face_inverse+1) % 3);
                const float* support_y = support + 16*((a_face_inverse+2) % 3);
                const float* support_z = support + 16*a_face_inverse;

                // Setup rotation matrix from a to world.
                simd4_float a_to_world0, a_to_world1, a_to_world2;
                {
                    simd4_float qx_qy_qz_qs = simd_float::load4(transforms[a_index].rotation);
                    simd4_float kx_ky_kz_ks = qx_qy_qz_qs + qx_qy_qz_qs;

                    // Make ks negative so that we can create +sx from kx*qs and -sx from ks*qx.
                    kx_ky_kz_ks = simd::bitwise_xor(kx_ky_kz_ks, simd_float::make4(0.0f, 0.0f, 0.0f, -0.0f));

                    //  1.0f - yy - zz, xy + sz, xz - sy
                    a_to_world0 = (simd128::shuffle32<1,0,0,3>(kx_ky_kz_ks) * simd128::shuffle32<1,1,2,3>(qx_qy_qz_qs) +
                                   simd128::shuffle32<2,2,3,3>(kx_ky_kz_ks) * simd128::shuffle32<2,3,1,3>(qx_qy_qz_qs));

                    // xy - sz, 1.0f - zz - xx, yz + sx
                    a_to_world1 = (simd128::shuffle32<0,2,1,3>(kx_ky_kz_ks) * simd128::shuffle32<1,2,2,3>(qx_qy_qz_qs) +
                                   simd128::shuffle32<3,0,0,3>(kx_ky_kz_ks) * simd128::shuffle32<2,0,3,3>(qx_qy_qz_qs));

                    // xz + sy, yz - sx, 1.0f - xx - yy
                    a_to_world2 = (simd128::shuffle32<0,1,0,3>(kx_ky_kz_ks) * simd128::shuffle32<2,2,0,3>(qx_qy_qz_qs) +
                                   simd128::shuffle32<1,3,1,3>(kx_ky_kz_ks) * simd128::shuffle32<3,0,1,3>(qx_qy_qz_qs));

                    a_to_world0 = a_to_world0 - simd_float::make4(1.0f, 0.0f, 0.0f, 0.0f);
                    a_to_world1 = a_to_world1 - simd_float::make4(0.0f, 1.0f, 0.0f, 0.0f);
                    a_to_world2 = a_to_world2 - simd_float::make4(0.0f, 0.0f, 1.0f, 0.0f);

                    a_to_world0 = simd::bitwise_xor(a_to_world0, simd_float::make4(-0.0f, 0.0f, 0.0f, 0.0f));
                    a_to_world1 = simd::bitwise_xor(a_to_world1, simd_float::make4(0.0f, -0.0f, 0.0f, 0.0f));
                    a_to_world2 = simd::bitwise_xor(a_to_world2, simd_float::make4(0.0f, 0.0f, -0.0f, 0.0f));
                }

                // Add valid support points as contacts.
                simd4_float wn = a_face == 0 ? a_to_world0 : (a_face == 1 ? a_to_world1 : a_to_world2);

                if (b_offset_neg)
                    wn = simd::bitwise_xor(wn, simd_float::make4(-0.0f));

                simd4_float a_position = simd_float::load4(transforms[a_index].position);

                uint16_t a_body = (uint16_t)transforms[a_index].body;
                uint16_t b_body = (uint16_t)transforms[b_index].body;

                a_index = transforms[a_index].body >> 16;
                b_index = transforms[b_index].body >> 16;

                unsigned tag_swap = 0;

                if (b_index > a_index) {
                    unsigned tc = a_index;
                    uint16_t tb = a_body;

                    a_index = b_index;
                    b_index = tc;

                    a_body = b_body;
                    b_body = tb;

                    tag_swap = 16;

                    wn = simd::bitwise_xor(wn, simd_float::make4(-0.0f));;
                }

                uint64_t high_tag = ((uint64_t)a_index << 32) | ((uint64_t)b_index << 48);

                while (mask) {
                    unsigned index = first_set_bit(mask);
                    mask &= mask-1;

                    simd4_float wp = (a_to_world0 * simd_float::broadcast_load4(support_x + index) +
                                      a_to_world1 * simd_float::broadcast_load4(support_y + index) +
                                      a_to_world2 * simd_float::broadcast_load4(support_z + index) + a_position);

                    float penetration = penetrations[index];

                    simd_float::store4(contacts[count].position, wp);
                    simd_float::store4(contacts[count].normal, wn);

                    contacts[count].penetration = penetration;
                    contacts[count].friction = NUDGE_FRICTION_MODEL(properties[a_body].friction,properties[b_body].friction);   // this works!
                    bodies[count].a = a_body;
                    bodies[count].b = b_body;
                    tags[count] = (uint32_t)(support_tags[index] >> tag_swap) | (uint32_t)(support_tags[index] << tag_swap) | high_tag;

                    ++count;
                }
            }

            // Batch edge pairs.
            // Note: We need to output the edge pairs after handling the faces since we read from the pairs array during face processing.
            while (edge) {
                unsigned index = first_set_bit(edge);
                edge &= edge-1;

                unsigned pair = pairs[pair_offset + index];
                unsigned edge_a = a_edge_array[index];
                unsigned edge_b = b_edge_array[index];

                unsigned a = pair & 0xffff;
                unsigned b = pair >> 16;

                a = transforms[a].body >> 16;
                b = transforms[b].body >> 16;

                feature_penetrations[added] = penetration_array[index];
                features[added] = a > b ? edge_a | (edge_b << 16) : edge_b | (edge_a << 16);
                pairs[added] = a > b ? pair : (pair >> 16) | (pair << 16);

                ++added;
            }
        }

        assert(!added || pairs[added-1]); // There should be no padding.

        pair_count = added;
    }

    // Do edge-edge tests.
    {
        pairs[pair_count+0] = 0; // Padding.
        pairs[pair_count+1] = 0;
        pairs[pair_count+2] = 0;

        features[pair_count+0] = 0;
        features[pair_count+1] = 0;
        features[pair_count+2] = 0;

        feature_penetrations[pair_count+0] = 0.0f;
        feature_penetrations[pair_count+1] = 0.0f;
        feature_penetrations[pair_count+2] = 0.0f;

        for (unsigned i = 0; i < pair_count; i += 4) {
            // Load pairs.
            unsigned pair0 = pairs[i + 0];
            unsigned pair1 = pairs[i + 1];
            unsigned pair2 = pairs[i + 2];
            unsigned pair3 = pairs[i + 3];

            unsigned a0_index = pair0 & 0xffff;
            unsigned b0_index = pair0 >> 16;

            unsigned a1_index = pair1 & 0xffff;
            unsigned b1_index = pair1 >> 16;

            unsigned a2_index = pair2 & 0xffff;
            unsigned b2_index = pair2 >> 16;

            unsigned a3_index = pair3 & 0xffff;
            unsigned b3_index = pair3 >> 16;

            // Load rotations.
            simd4_float a_rotation_x = simd_float::load4(transforms[a0_index].rotation);
            simd4_float a_rotation_y = simd_float::load4(transforms[a1_index].rotation);
            simd4_float a_rotation_z = simd_float::load4(transforms[a2_index].rotation);
            simd4_float a_rotation_s = simd_float::load4(transforms[a3_index].rotation);

            simd4_float b_rotation_x = simd_float::load4(transforms[b0_index].rotation);
            simd4_float b_rotation_y = simd_float::load4(transforms[b1_index].rotation);
            simd4_float b_rotation_z = simd_float::load4(transforms[b2_index].rotation);
            simd4_float b_rotation_s = simd_float::load4(transforms[b3_index].rotation);

            simd128::transpose32(a_rotation_x, a_rotation_y, a_rotation_z, a_rotation_s);
            simd128::transpose32(b_rotation_x, b_rotation_y, b_rotation_z, b_rotation_s);

            // Compute rotation matrices.
            simd4_float a_basis_xx, a_basis_xy, a_basis_xz;
            simd4_float a_basis_yx, a_basis_yy, a_basis_yz;
            simd4_float a_basis_zx, a_basis_zy, a_basis_zz;
            {
                simd4_float kx = a_rotation_x + a_rotation_x;
                simd4_float ky = a_rotation_y + a_rotation_y;
                simd4_float kz = a_rotation_z + a_rotation_z;

                simd4_float xx = kx*a_rotation_x;
                simd4_float yy = ky*a_rotation_y;
                simd4_float zz = kz*a_rotation_z;
                simd4_float xy = kx*a_rotation_y;
                simd4_float xz = kx*a_rotation_z;
                simd4_float yz = ky*a_rotation_z;
                simd4_float sx = kx*a_rotation_s;
                simd4_float sy = ky*a_rotation_s;
                simd4_float sz = kz*a_rotation_s;

                a_basis_xx = simd_float::make4(1.0f) - yy - zz;
                a_basis_xy = xy + sz;
                a_basis_xz = xz - sy;

                a_basis_yx = xy - sz;
                a_basis_yy = simd_float::make4(1.0f) - xx - zz;
                a_basis_yz = yz + sx;

                a_basis_zx = xz + sy;
                a_basis_zy = yz - sx;
                a_basis_zz = simd_float::make4(1.0f) - xx - yy;
            }

            simd4_float b_basis_xx, b_basis_xy, b_basis_xz;
            simd4_float b_basis_yx, b_basis_yy, b_basis_yz;
            simd4_float b_basis_zx, b_basis_zy, b_basis_zz;
            {
                simd4_float kx = b_rotation_x + b_rotation_x;
                simd4_float ky = b_rotation_y + b_rotation_y;
                simd4_float kz = b_rotation_z + b_rotation_z;

                simd4_float xx = kx*b_rotation_x;
                simd4_float yy = ky*b_rotation_y;
                simd4_float zz = kz*b_rotation_z;
                simd4_float xy = kx*b_rotation_y;
                simd4_float xz = kx*b_rotation_z;
                simd4_float yz = ky*b_rotation_z;
                simd4_float sx = kx*b_rotation_s;
                simd4_float sy = ky*b_rotation_s;
                simd4_float sz = kz*b_rotation_s;

                b_basis_xx = simd_float::make4(1.0f) - yy - zz;
                b_basis_xy = xy + sz;
                b_basis_xz = xz - sy;

                b_basis_yx = xy - sz;
                b_basis_yy = simd_float::make4(1.0f) - xx - zz;
                b_basis_yz = yz + sx;

                b_basis_zx = xz + sy;
                b_basis_zy = yz - sx;
                b_basis_zz = simd_float::make4(1.0f) - xx - yy;
            }

            // Load edges.
            simd4_int32 edge = simd_int32::load4((const int32_t*)(features + i));

            // Select edge directions.
#ifdef NUDGE_NATIVE_BLENDV32
            simd4_int32 a_select_y = simd_int32::shift_left<32-1>(edge); // Shifts the relevant bit to the top.
            simd4_int32 a_select_z = simd_int32::shift_left<32-2>(edge);

            simd4_int32 b_select_y = simd_int32::shift_left<16-1>(edge);
            simd4_int32 b_select_z = simd_int32::shift_left<16-2>(edge);

            simd4_float u_x = simd::blendv32(a_basis_xx, a_basis_yx, simd_int32::asfloat(a_select_y));
            simd4_float u_y = simd::blendv32(a_basis_xy, a_basis_yy, simd_int32::asfloat(a_select_y));
            simd4_float u_z = simd::blendv32(a_basis_xz, a_basis_yz, simd_int32::asfloat(a_select_y));

            simd4_float v_x = simd::blendv32(b_basis_xx, b_basis_yx, simd_int32::asfloat(b_select_y));
            simd4_float v_y = simd::blendv32(b_basis_xy, b_basis_yy, simd_int32::asfloat(b_select_y));
            simd4_float v_z = simd::blendv32(b_basis_xz, b_basis_yz, simd_int32::asfloat(b_select_y));

            u_x = simd::blendv32(u_x, a_basis_zx, simd_int32::asfloat(a_select_z));
            u_y = simd::blendv32(u_y, a_basis_zy, simd_int32::asfloat(a_select_z));
            u_z = simd::blendv32(u_z, a_basis_zz, simd_int32::asfloat(a_select_z));

            v_x = simd::blendv32(v_x, b_basis_zx, simd_int32::asfloat(b_select_z));
            v_y = simd::blendv32(v_y, b_basis_zy, simd_int32::asfloat(b_select_z));
            v_z = simd::blendv32(v_z, b_basis_zz, simd_int32::asfloat(b_select_z));
#else
            simd4_int32 a_edge = simd::bitwise_and(edge, simd_int32::make4(0xffff));
            simd4_int32 b_edge = simd_int32::shift_right<16>(edge);

            simd4_float a_select_x = simd_int32::asfloat(simd_int32::cmp_eq(a_edge, simd_int32::zero4()));
            simd4_float a_select_y = simd_int32::asfloat(simd_int32::cmp_eq(a_edge, simd_int32::make4(1)));
            simd4_float a_select_z = simd_int32::asfloat(simd_int32::cmp_eq(a_edge, simd_int32::make4(2)));

            simd4_float b_select_x = simd_int32::asfloat(simd_int32::cmp_eq(b_edge, simd_int32::zero4()));
            simd4_float b_select_y = simd_int32::asfloat(simd_int32::cmp_eq(b_edge, simd_int32::make4(1)));
            simd4_float b_select_z = simd_int32::asfloat(simd_int32::cmp_eq(b_edge, simd_int32::make4(2)));

            simd4_float u_x = simd::bitwise_and(a_basis_xx, a_select_x);
            simd4_float u_y = simd::bitwise_and(a_basis_xy, a_select_x);
            simd4_float u_z = simd::bitwise_and(a_basis_xz, a_select_x);

            simd4_float v_x = simd::bitwise_and(b_basis_xx, b_select_x);
            simd4_float v_y = simd::bitwise_and(b_basis_xy, b_select_x);
            simd4_float v_z = simd::bitwise_and(b_basis_xz, b_select_x);

            u_x = simd::bitwise_or(u_x, simd::bitwise_and(a_basis_yx, a_select_y));
            u_y = simd::bitwise_or(u_y, simd::bitwise_and(a_basis_yy, a_select_y));
            u_z = simd::bitwise_or(u_z, simd::bitwise_and(a_basis_yz, a_select_y));

            v_x = simd::bitwise_or(v_x, simd::bitwise_and(b_basis_yx, b_select_y));
            v_y = simd::bitwise_or(v_y, simd::bitwise_and(b_basis_yy, b_select_y));
            v_z = simd::bitwise_or(v_z, simd::bitwise_and(b_basis_yz, b_select_y));

            u_x = simd::bitwise_or(u_x, simd::bitwise_and(a_basis_zx, a_select_z));
            u_y = simd::bitwise_or(u_y, simd::bitwise_and(a_basis_zy, a_select_z));
            u_z = simd::bitwise_or(u_z, simd::bitwise_and(a_basis_zz, a_select_z));

            v_x = simd::bitwise_or(v_x, simd::bitwise_and(b_basis_zx, b_select_z));
            v_y = simd::bitwise_or(v_y, simd::bitwise_and(b_basis_zy, b_select_z));
            v_z = simd::bitwise_or(v_z, simd::bitwise_and(b_basis_zz, b_select_z));
#endif

            // Compute axis.
            simd4_float n_x, n_y, n_z;
            simd_soa::cross(u_x, u_y, u_z, v_x, v_y, v_z, n_x, n_y, n_z);

            // Load positions.
            simd4_float a_position_x = simd_float::load4(transforms[a0_index].position);
            simd4_float a_position_y = simd_float::load4(transforms[a1_index].position);
            simd4_float a_position_z = simd_float::load4(transforms[a2_index].position);
            simd4_float a_position_w = simd_float::load4(transforms[a3_index].position);

            simd4_float b_position_x = simd_float::load4(transforms[b0_index].position);
            simd4_float b_position_y = simd_float::load4(transforms[b1_index].position);
            simd4_float b_position_z = simd_float::load4(transforms[b2_index].position);
            simd4_float b_position_w = simd_float::load4(transforms[b3_index].position);

            simd128::transpose32(a_position_x, a_position_y, a_position_z, a_position_w);
            simd128::transpose32(b_position_x, b_position_y, b_position_z, b_position_w);

            // Compute relative position.
            simd4_float delta_x = b_position_x - a_position_x;
            simd4_float delta_y = b_position_y - a_position_y;
            simd4_float delta_z = b_position_z - a_position_z;

            // Flip normal?
            simd4_float sign_mask = simd_float::make4(-0.0f);
            simd4_float flip_sign = simd::bitwise_and(n_x*delta_x + n_y*delta_y + n_z*delta_z, sign_mask);

            n_x = simd::bitwise_xor(n_x, flip_sign);
            n_y = simd::bitwise_xor(n_y, flip_sign);
            n_z = simd::bitwise_xor(n_z, flip_sign);

            // Load sizes.
            simd4_float a_size_x = simd_float::load4(colliders[a0_index].size);
            simd4_float a_size_y = simd_float::load4(colliders[a1_index].size);
            simd4_float a_size_z = simd_float::load4(colliders[a2_index].size);
            simd4_float a_size_w = simd_float::load4(colliders[a3_index].size);

            simd4_float b_size_x = simd_float::load4(colliders[b0_index].size);
            simd4_float b_size_y = simd_float::load4(colliders[b1_index].size);
            simd4_float b_size_z = simd_float::load4(colliders[b2_index].size);
            simd4_float b_size_w = simd_float::load4(colliders[b3_index].size);

            simd128::transpose32(a_size_x, a_size_y, a_size_z, a_size_w);
            simd128::transpose32(b_size_x, b_size_y, b_size_z, b_size_w);

            // Compute direction to the edge.
            simd4_float a_sign_x = a_basis_xx*n_x + a_basis_xy*n_y + a_basis_xz*n_z;
            simd4_float a_sign_y = a_basis_yx*n_x + a_basis_yy*n_y + a_basis_yz*n_z;
            simd4_float a_sign_z = a_basis_zx*n_x + a_basis_zy*n_y + a_basis_zz*n_z;

            simd4_float b_sign_x = b_basis_xx*n_x + b_basis_xy*n_y + b_basis_xz*n_z;
            simd4_float b_sign_y = b_basis_yx*n_x + b_basis_yy*n_y + b_basis_yz*n_z;
            simd4_float b_sign_z = b_basis_zx*n_x + b_basis_zy*n_y + b_basis_zz*n_z;

            a_sign_x = simd::bitwise_and(a_sign_x, sign_mask);
            a_sign_y = simd::bitwise_and(a_sign_y, sign_mask);
            a_sign_z = simd::bitwise_and(a_sign_z, sign_mask);

            b_sign_x = simd::bitwise_and(b_sign_x, sign_mask);
            b_sign_y = simd::bitwise_and(b_sign_y, sign_mask);
            b_sign_z = simd::bitwise_and(b_sign_z, sign_mask);

            simd4_int32 edge_x = simd::bitwise_or(simd_int32::shift_right<31-0>(simd_float::asint(a_sign_x)), simd_int32::shift_right<31-16>(simd_float::asint(simd::bitwise_xor(b_sign_x, simd_float::make4(-0.0f)))));
            simd4_int32 edge_y = simd::bitwise_or(simd_int32::shift_right<31-1>(simd_float::asint(a_sign_y)), simd_int32::shift_right<31-17>(simd_float::asint(simd::bitwise_xor(b_sign_y, simd_float::make4(-0.0f)))));
            simd4_int32 edge_z = simd::bitwise_or(simd_int32::shift_right<31-2>(simd_float::asint(a_sign_z)), simd_int32::shift_right<31-18>(simd_float::asint(simd::bitwise_xor(b_sign_z, simd_float::make4(-0.0f)))));
            simd4_int32 edge_w = _mm_add_epi16(_mm_add_epi16(edge, _mm_set1_epi16(1)), _mm_srli_epi16(edge, 1)); // Calculates 1 << edge (valid for 0-2).

            simd4_int32 edge_xy = simd::bitwise_or(edge_x, edge_y);
            simd4_int32 edge_zw = simd::bitwise_or(edge_z, edge_w);

            simd4_int32 tag_hi = simd::bitwise_or(edge_xy, edge_zw);
            simd4_int32 tag_lo = simd::bitwise_notand(edge_w, tag_hi);
            tag_hi = simd_int32::shift_left<8>(tag_hi);

            simd4_int32 tag = simd::bitwise_or(tag_lo, tag_hi);

            a_size_x = simd::bitwise_xor(a_size_x, a_sign_x);
            a_size_y = simd::bitwise_xor(a_size_y, a_sign_y);
            a_size_z = simd::bitwise_xor(a_size_z, a_sign_z);

            b_size_x = simd::bitwise_xor(b_size_x, b_sign_x);
            b_size_y = simd::bitwise_xor(b_size_y, b_sign_y);
            b_size_z = simd::bitwise_xor(b_size_z, b_sign_z);

            a_basis_xx *= a_size_x;
            a_basis_xy *= a_size_x;
            a_basis_xz *= a_size_x;

            a_basis_yx *= a_size_y;
            a_basis_yy *= a_size_y;
            a_basis_yz *= a_size_y;

            a_basis_zx *= a_size_z;
            a_basis_zy *= a_size_z;
            a_basis_zz *= a_size_z;

            b_basis_xx *= b_size_x;
            b_basis_xy *= b_size_x;
            b_basis_xz *= b_size_x;

            b_basis_yx *= b_size_y;
            b_basis_yy *= b_size_y;
            b_basis_yz *= b_size_y;

            b_basis_zx *= b_size_z;
            b_basis_zy *= b_size_z;
            b_basis_zz *= b_size_z;

            simd4_float ca_x = a_basis_xx + a_basis_yx + a_basis_zx + a_position_x;
            simd4_float ca_y = a_basis_xy + a_basis_yy + a_basis_zy + a_position_y;
            simd4_float ca_z = a_basis_xz + a_basis_yz + a_basis_zz + a_position_z;

            simd4_float cb_x = b_basis_xx + b_basis_yx + b_basis_zx - b_position_x; // Note that cb really is negated to save some operations.
            simd4_float cb_y = b_basis_xy + b_basis_yy + b_basis_zy - b_position_y;
            simd4_float cb_z = b_basis_xz + b_basis_yz + b_basis_zz - b_position_z;

            // Calculate closest point between the two lines.
            simd4_float o_x = ca_x + cb_x;
            simd4_float o_y = ca_y + cb_y;
            simd4_float o_z = ca_z + cb_z;

            simd4_float ia = u_x*u_x + u_y*u_y + u_z*u_z;
            simd4_float ib = u_x*v_x + u_y*v_y + u_z*v_z;
            simd4_float ic = v_x*v_x + v_y*v_y + v_z*v_z;
            simd4_float id = o_x*u_x + o_y*u_y + o_z*u_z;
            simd4_float ie = o_x*v_x + o_y*v_y + o_z*v_z;

            simd4_float half = simd_float::make4(0.5f);
            simd4_float ir = half / (ia*ic - ib*ib);

            simd4_float sa = (ib*ie - ic*id) * ir;
            simd4_float sb = (ia*ie - ib*id) * ir;

            simd4_float p_x = (ca_x - cb_x)*half + u_x*sa + v_x*sb;
            simd4_float p_y = (ca_y - cb_y)*half + u_y*sa + v_y*sb;
            simd4_float p_z = (ca_z - cb_z)*half + u_z*sa + v_z*sb;

            simd_soa::normalize(n_x, n_y, n_z);

            simd4_float p_w = simd_float::load4(feature_penetrations + i);
            simd4_float n_w = simd_float::make4(0.5f);

            simd128::transpose32(p_x, p_y, p_z, p_w);
            simd128::transpose32(n_x, n_y, n_z, n_w);

            simd_float::store4(contacts[count + 0].position, p_x);
            simd_float::store4(contacts[count + 0].normal, n_x);
            simd_float::store4(contacts[count + 1].position, p_y);
            simd_float::store4(contacts[count + 1].normal, n_y);
            simd_float::store4(contacts[count + 2].position, p_z);
            simd_float::store4(contacts[count + 2].normal, n_z);
            simd_float::store4(contacts[count + 3].position, p_w);
            simd_float::store4(contacts[count + 3].normal, n_w);

            simd4_float body_pair = simd::bitwise_or(simd::bitwise_and(a_position_w, simd_int32::asfloat(simd_int32::make4(0xffff))), simd_int32::asfloat(simd_int32::shift_left<16>(simd_float::asint(b_position_w))));
            simd_float::storeu4((float*)(bodies + count), body_pair);

            simd4_int32 pair = simd_float::asint(simd::bitwise_or(simd::bitwise_and(b_position_w, simd_int32::asfloat(simd_int32::make4(0xffff0000))), simd_int32::asfloat(simd_int32::shift_right<16>(simd_float::asint(a_position_w)))));

            simd_int32::storeu4((int32_t*)tags + count*2 + 0, simd128::unpacklo32(tag, pair));
            simd_int32::storeu4((int32_t*)tags + count*2 + 4, simd128::unpackhi32(tag, pair));

            count += 4;
        }

        // Get rid of padding.
        while (count && bodies[count-1].a == bodies[count-1].b)
            --count;
    }

    return count;
}

static inline unsigned sphere_sphere_collide(SphereCollider a, SphereCollider b, Transform a_transform, Transform b_transform, Contact* contacts, BodyPair* bodies,float friction) {
    float r = a.radius + b.radius;

    float3 dp = make_float3(b_transform.position) - make_float3(a_transform.position);
    float l2 = length2(dp);

    if (l2 > r*r)
        return 0;

    float3 n;
    float l = sqrtf(l2);

    if (l2 > 1e-4f)
        n = dp * (1.0f / l);
    else
        n = make_float3(1.0f, 0.0f, 0.0f);

    float3 p = make_float3(a_transform.position) + n * (l - b.radius);

    contacts[0].position[0] = p.x;
    contacts[0].position[1] = p.y;
    contacts[0].position[2] = p.z;
    contacts[0].penetration = r - l;
    contacts[0].normal[0] = n.x;
    contacts[0].normal[1] = n.y;
    contacts[0].normal[2] = n.z;
    contacts[0].friction = friction;    // doesn't work (try setting it to 100000.f: nothing changes!)

    bodies[0].a = (uint16_t)a_transform.body;
    bodies[0].b = (uint16_t)b_transform.body;

    return 1;
}

static inline unsigned box_sphere_collide(BoxCollider a, SphereCollider b, Transform a_transform, Transform b_transform, Contact* contacts, BodyPair* bodies, float friction) {
    Rotation a_to_world = make_rotation(a_transform.rotation);
    Rotation world_to_a = inverse(a_to_world);
    float3 offset_b = world_to_a * (make_float3(b_transform.position) - make_float3(a_transform.position));

    float dx = fabsf(offset_b.x);
    float dy = fabsf(offset_b.y);
    float dz = fabsf(offset_b.z);

    float w = a.size[0] + b.radius;
    float h = a.size[1] + b.radius;
    float d = a.size[2] + b.radius;

    if (dx >= w || dy >= h || dz >= d)
        return 0;

    float3 n;
    float penetration;

    float r = b.radius;

    unsigned outside_x = dx > a.size[0];
    unsigned outside_y = dy > a.size[1];
    unsigned outside_z = dz > a.size[2];

    if (outside_x + outside_y + outside_z >= 2) {
        float3 corner = {
            outside_x ? (offset_b.x > 0.0f ? a.size[0] : -a.size[0]) : offset_b.x,
            outside_y ? (offset_b.y > 0.0f ? a.size[1] : -a.size[1]) : offset_b.y,
            outside_z ? (offset_b.z > 0.0f ? a.size[2] : -a.size[2]) : offset_b.z,
        };

        float3 dp = offset_b - corner;
        float l2 = length2(dp);

        if (l2 > r*r)
            return 0;

        float l = sqrtf(l2);
        float m = 1.0f / l;

        n = dp * m;
        penetration = r - l;
    }
    else if (w - dx < h - dy && w - dx < d - dz) {
        n.x = offset_b.x > 0.0f ? 1.0f : -1.0f;
        n.y = 0.0f;
        n.z = 0.0f;
        penetration = w - dx;
    }
    else if (h - dy < d - dz) {
        n.x = 0.0f;
        n.y = offset_b.y > 0.0f ? 1.0f : -1.0f;
        n.z = 0.0f;
        penetration = h - dy;
    }
    else {
        n.x = 0.0f;
        n.y = 0.0f;
        n.z = offset_b.z > 0.0f ? 1.0f : -1.0f;
        penetration = d - dz;
    }

    float3 p = offset_b - n*r;

    p = a_to_world * p + make_float3(a_transform.position);
    n = a_to_world * n;

    contacts[0].position[0] = p.x;
    contacts[0].position[1] = p.y;
    contacts[0].position[2] = p.z;
    contacts[0].penetration = penetration;
    contacts[0].normal[0] = n.x;
    contacts[0].normal[1] = n.y;
    contacts[0].normal[2] = n.z;
    contacts[0].friction = friction;    // affects only the box shape (try setting it to 100000.f: nothing changes for the sphere!)

    bodies[0].a = (uint16_t)a_transform.body;
    bodies[0].b = (uint16_t)b_transform.body;

    return 1;
}

template<unsigned offset>
static inline void dilate_3(simdv_int32 x, simdv_int32& lo32, simdv_int32& hi32) {
    simdv_int32 mask0 = simd_int32::makev(0xff);
    simdv_int32 mask1 = simd_int32::makev(0x0f00f00f);
    simdv_int32 mask2 = simd_int32::makev(0xc30c30c3);
    simdv_int32 mask3 = simd_int32::makev(0x49249249);

    simdv_int32 lo24 = x;
    simdv_int32 hi24 = simd_int32::shift_right<8>(x);
    lo24 = simd::bitwise_and(lo24, mask0);
    hi24 = simd::bitwise_and(hi24, mask0);

    lo24 = simd::bitwise_or(lo24, simd_int32::shift_left<8>(lo24));
    hi24 = simd::bitwise_or(hi24, simd_int32::shift_left<8>(hi24));
    lo24 = simd::bitwise_and(lo24, mask1);
    hi24 = simd::bitwise_and(hi24, mask1);

    lo24 = simd::bitwise_or(lo24, simd_int32::shift_left<4>(lo24));
    hi24 = simd::bitwise_or(hi24, simd_int32::shift_left<4>(hi24));
    lo24 = simd::bitwise_and(lo24, mask2);
    hi24 = simd::bitwise_and(hi24, mask2);

    lo24 = simd::bitwise_or(lo24, simd_int32::shift_left<2>(lo24));
    hi24 = simd::bitwise_or(hi24, simd_int32::shift_left<2>(hi24));
    lo24 = simd::bitwise_and(lo24, mask3);
    hi24 = simd::bitwise_and(hi24, mask3);

    lo32 = simd::bitwise_or(simd_int32::shift_left<offset>(lo24), simd_int32::shift_left<24+offset>(hi24));
    hi32 = simd_int32::shift_right<8-offset>(hi24);
}

static inline void morton(simdv_int32 x, simdv_int32 y, simdv_int32 z, simdv_int32& lo32, simdv_int32& hi32) {
    simdv_int32 lx, hx, ly, hy, lz, hz;
    dilate_3<2>(x, lx, hx);
    dilate_3<1>(y, ly, hy);
    dilate_3<0>(z, lz, hz);

    lo32 = simd::bitwise_or(simd::bitwise_or(lx, ly), lz);
    hi32 = simd::bitwise_or(simd::bitwise_or(hx, hy), hz);
}

static inline void radix_sort_uint64_low48(uint64_t* data, unsigned count, Arena temporary) {
    uint64_t* temp = allocate_array<uint64_t>(&temporary, count, 16);

    unsigned buckets0[257] = {};
    unsigned buckets1[257] = {};
    unsigned buckets2[257] = {};
    unsigned buckets3[257] = {};
    unsigned buckets4[257] = {};
    unsigned buckets5[257] = {};

    unsigned* histogram0 = buckets0+1;
    unsigned* histogram1 = buckets1+1;
    unsigned* histogram2 = buckets2+1;
    unsigned* histogram3 = buckets3+1;
    unsigned* histogram4 = buckets4+1;
    unsigned* histogram5 = buckets5+1;

    for (unsigned i = 0; i < count; ++i) {
        uint64_t d = data[i];

        ++histogram0[(d >> (0 << 3)) & 0xff];
        ++histogram1[(d >> (1 << 3)) & 0xff];
        ++histogram2[(d >> (2 << 3)) & 0xff];
        ++histogram3[(d >> (3 << 3)) & 0xff];
        ++histogram4[(d >> (4 << 3)) & 0xff];
        ++histogram5[(d >> (5 << 3)) & 0xff];
    }

    for (unsigned i = 1; i < 256; ++i) {
        buckets0[i] += buckets0[i-1];
        buckets1[i] += buckets1[i-1];
        buckets2[i] += buckets2[i-1];
        buckets3[i] += buckets3[i-1];
        buckets4[i] += buckets4[i-1];
        buckets5[i] += buckets5[i-1];
    }

    for (unsigned i = 0; i < count; ++i) {
        uint64_t d = data[i];
        unsigned index = buckets0[(d >> (0 << 3)) & 0xff]++;
        temp[index] = d;
    }

    for (unsigned i = 0; i < count; ++i) {
        uint64_t d = temp[i];
        unsigned index = buckets1[(d >> (1 << 3)) & 0xff]++;
        data[index] = d;
    }

    for (unsigned i = 0; i < count; ++i) {
        uint64_t d = data[i];
        unsigned index = buckets2[(d >> (2 << 3)) & 0xff]++;
        temp[index] = d;
    }

    for (unsigned i = 0; i < count; ++i) {
        uint64_t d = temp[i];
        unsigned index = buckets3[(d >> (3 << 3)) & 0xff]++;
        data[index] = d;
    }

    for (unsigned i = 0; i < count; ++i) {
        uint64_t d = data[i];
        unsigned index = buckets4[(d >> (4 << 3)) & 0xff]++;
        temp[index] = d;
    }

    for (unsigned i = 0; i < count; ++i) {
        uint64_t d = temp[i];
        unsigned index = buckets5[(d >> (5 << 3)) & 0xff]++;
        data[index] = d;
    }
}

static inline void radix_sort_uint32_x2(uint32_t* data, uint32_t* data2, unsigned count, Arena temporary) {
    uint32_t* temp = allocate_array<uint32_t>(&temporary, count, 16);
    uint32_t* temp2 = allocate_array<uint32_t>(&temporary, count, 16);

    unsigned buckets0[257] = {};
    unsigned buckets1[257] = {};
    unsigned buckets2[257] = {};
    unsigned buckets3[257] = {};

    unsigned* histogram0 = buckets0+1;
    unsigned* histogram1 = buckets1+1;
    unsigned* histogram2 = buckets2+1;
    unsigned* histogram3 = buckets3+1;

    for (unsigned i = 0; i < count; ++i) {
        uint32_t d = data[i];

        ++histogram0[(d >> (0 << 3)) & 0xff];
        ++histogram1[(d >> (1 << 3)) & 0xff];
        ++histogram2[(d >> (2 << 3)) & 0xff];
        ++histogram3[(d >> (3 << 3)) & 0xff];
    }

    for (unsigned i = 1; i < 256; ++i) {
        buckets0[i] += buckets0[i-1];
        buckets1[i] += buckets1[i-1];
        buckets2[i] += buckets2[i-1];
        buckets3[i] += buckets3[i-1];
    }

    for (unsigned i = 0; i < count; ++i) {
        uint32_t d = data[i];
        uint32_t d2 = data2[i];
        unsigned index = buckets0[(d >> (0 << 3)) & 0xff]++;
        temp[index] = d;
        temp2[index] = d2;
    }

    for (unsigned i = 0; i < count; ++i) {
        uint32_t d = temp[i];
        uint32_t d2 = temp2[i];
        unsigned index = buckets1[(d >> (1 << 3)) & 0xff]++;
        data[index] = d;
        data2[index] = d2;
    }

    for (unsigned i = 0; i < count; ++i) {
        uint32_t d = data[i];
        uint32_t d2 = data2[i];
        unsigned index = buckets2[(d >> (2 << 3)) & 0xff]++;
        temp[index] = d;
        temp2[index] = d2;
    }

    for (unsigned i = 0; i < count; ++i) {
        uint32_t d = temp[i];
        uint32_t d2 = temp2[i];
        unsigned index = buckets3[(d >> (3 << 3)) & 0xff]++;
        data[index] = d;
        data2[index] = d2;
    }
}

static inline void radix_sort_uint32(uint32_t* data, unsigned count, Arena temporary) {
    uint32_t* temp = allocate_array<uint32_t>(&temporary, count, 16);

    unsigned buckets0[257] = {};
    unsigned buckets1[257] = {};
    unsigned buckets2[257] = {};
    unsigned buckets3[257] = {};

    unsigned* histogram0 = buckets0+1;
    unsigned* histogram1 = buckets1+1;
    unsigned* histogram2 = buckets2+1;
    unsigned* histogram3 = buckets3+1;

    for (unsigned i = 0; i < count; ++i) {
        uint32_t d = data[i];

        ++histogram0[(d >> (0 << 3)) & 0xff];
        ++histogram1[(d >> (1 << 3)) & 0xff];
        ++histogram2[(d >> (2 << 3)) & 0xff];
        ++histogram3[(d >> (3 << 3)) & 0xff];
    }

    for (unsigned i = 1; i < 256; ++i) {
        buckets0[i] += buckets0[i-1];
        buckets1[i] += buckets1[i-1];
        buckets2[i] += buckets2[i-1];
        buckets3[i] += buckets3[i-1];
    }

    for (unsigned i = 0; i < count; ++i) {
        uint32_t d = data[i];
        unsigned index = buckets0[(d >> (0 << 3)) & 0xff]++;
        temp[index] = d;
    }

    for (unsigned i = 0; i < count; ++i) {
        uint32_t d = temp[i];
        unsigned index = buckets1[(d >> (1 << 3)) & 0xff]++;
        data[index] = d;
    }

    for (unsigned i = 0; i < count; ++i) {
        uint32_t d = data[i];
        unsigned index = buckets2[(d >> (2 << 3)) & 0xff]++;
        temp[index] = d;
    }

    for (unsigned i = 0; i < count; ++i) {
        uint32_t d = temp[i];
        unsigned index = buckets3[(d >> (3 << 3)) & 0xff]++;
        data[index] = d;
    }
}

template<unsigned data_stride, unsigned index_stride, class T>
NUDGE_FORCEINLINE static void load4(const float* data, const T* indices,
                                    simdv_float& d0, simdv_float& d1, simdv_float& d2, simdv_float& d3) {
    static const unsigned stride_in_floats = data_stride/sizeof(float);

#if NUDGE_SIMDV_WIDTH == 256
    unsigned i0 = indices[0*index_stride];
    unsigned i1 = indices[1*index_stride];
    unsigned i2 = indices[2*index_stride];
    unsigned i3 = indices[3*index_stride];

    simd4_float t0 = simd_float::load4(data + i0*stride_in_floats);
    simd4_float t1 = simd_float::load4(data + i1*stride_in_floats);
    simd4_float t2 = simd_float::load4(data + i2*stride_in_floats);
    simd4_float t3 = simd_float::load4(data + i3*stride_in_floats);

    unsigned i4 = indices[4*index_stride];
    unsigned i5 = indices[5*index_stride];
    unsigned i6 = indices[6*index_stride];
    unsigned i7 = indices[7*index_stride];

    simd4_float t4 = simd_float::load4(data + i4*stride_in_floats);
    simd4_float t5 = simd_float::load4(data + i5*stride_in_floats);
    simd4_float t6 = simd_float::load4(data + i6*stride_in_floats);
    simd4_float t7 = simd_float::load4(data + i7*stride_in_floats);

    d0 = simd::concat(t0, t4);
    d1 = simd::concat(t1, t5);
    d2 = simd::concat(t2, t6);
    d3 = simd::concat(t3, t7);
#else
    unsigned i0 = indices[0*index_stride];
    unsigned i1 = indices[1*index_stride];
    unsigned i2 = indices[2*index_stride];
    unsigned i3 = indices[3*index_stride];

    d0 = simd_float::load4(data + i0*stride_in_floats);
    d1 = simd_float::load4(data + i1*stride_in_floats);
    d2 = simd_float::load4(data + i2*stride_in_floats);
    d3 = simd_float::load4(data + i3*stride_in_floats);
#endif

    simd128::transpose32(d0, d1, d2, d3);
}

template<unsigned data_stride, unsigned index_stride, class T>
NUDGE_FORCEINLINE static void load8(const float* data, const T* indices,
                                    simdv_float& d0, simdv_float& d1, simdv_float& d2, simdv_float& d3,
                                    simdv_float& d4, simdv_float& d5, simdv_float& d6, simdv_float& d7) {
    static const unsigned stride_in_floats = data_stride/sizeof(float);

#if NUDGE_SIMDV_WIDTH == 256
    unsigned i0 = indices[0*index_stride];
    unsigned i1 = indices[1*index_stride];
    unsigned i2 = indices[2*index_stride];
    unsigned i3 = indices[3*index_stride];

    simdv_float t0 = simd_float::load8(data + i0*stride_in_floats);
    simdv_float t1 = simd_float::load8(data + i1*stride_in_floats);
    simdv_float t2 = simd_float::load8(data + i2*stride_in_floats);
    simdv_float t3 = simd_float::load8(data + i3*stride_in_floats);

    unsigned i4 = indices[4*index_stride];
    unsigned i5 = indices[5*index_stride];
    unsigned i6 = indices[6*index_stride];
    unsigned i7 = indices[7*index_stride];

    simdv_float t4 = simd_float::load8(data + i4*stride_in_floats);
    simdv_float t5 = simd_float::load8(data + i5*stride_in_floats);
    simdv_float t6 = simd_float::load8(data + i6*stride_in_floats);
    simdv_float t7 = simd_float::load8(data + i7*stride_in_floats);

    d0 = simd256::permute128<0,2>(t0, t4);
    d1 = simd256::permute128<0,2>(t1, t5);
    d2 = simd256::permute128<0,2>(t2, t6);
    d3 = simd256::permute128<0,2>(t3, t7);

    d4 = simd256::permute128<1,3>(t0, t4);
    d5 = simd256::permute128<1,3>(t1, t5);
    d6 = simd256::permute128<1,3>(t2, t6);
    d7 = simd256::permute128<1,3>(t3, t7);
#else
    unsigned i0 = indices[0*index_stride];
    unsigned i1 = indices[1*index_stride];
    unsigned i2 = indices[2*index_stride];
    unsigned i3 = indices[3*index_stride];

    d0 = simd_float::load4(data + i0*stride_in_floats);
    d1 = simd_float::load4(data + i1*stride_in_floats);
    d2 = simd_float::load4(data + i2*stride_in_floats);
    d3 = simd_float::load4(data + i3*stride_in_floats);

    d4 = simd_float::load4(data + i0*stride_in_floats + 4);
    d5 = simd_float::load4(data + i1*stride_in_floats + 4);
    d6 = simd_float::load4(data + i2*stride_in_floats + 4);
    d7 = simd_float::load4(data + i3*stride_in_floats + 4);
#endif

    simd128::transpose32(d0, d1, d2, d3);
    simd128::transpose32(d4, d5, d6, d7);
}

template<unsigned data_stride, unsigned index_stride, class T>
NUDGE_FORCEINLINE static void store8(float* data, const T* indices,
                                     simdv_float d0, simdv_float d1, simdv_float d2, simdv_float d3,
                                     simdv_float d4, simdv_float d5, simdv_float d6, simdv_float d7) {
    static const unsigned stride_in_floats = data_stride/sizeof(float);

#if NUDGE_SIMDV_WIDTH == 256
    simdv_float t0 = simd256::permute128<0,2>(d0, d4);
    simdv_float t1 = simd256::permute128<0,2>(d1, d5);
    simdv_float t2 = simd256::permute128<0,2>(d2, d6);
    simdv_float t3 = simd256::permute128<0,2>(d3, d7);

    simdv_float t4 = simd256::permute128<1,3>(d0, d4);
    simdv_float t5 = simd256::permute128<1,3>(d1, d5);
    simdv_float t6 = simd256::permute128<1,3>(d2, d6);
    simdv_float t7 = simd256::permute128<1,3>(d3, d7);

    simd128::transpose32(t0, t1, t2, t3);
    simd128::transpose32(t4, t5, t6, t7);

    unsigned i0 = indices[0*index_stride];
    unsigned i1 = indices[1*index_stride];
    unsigned i2 = indices[2*index_stride];
    unsigned i3 = indices[3*index_stride];

    simd_float::store8(data + i0*stride_in_floats, t0);
    simd_float::store8(data + i1*stride_in_floats, t1);
    simd_float::store8(data + i2*stride_in_floats, t2);
    simd_float::store8(data + i3*stride_in_floats, t3);

    unsigned i4 = indices[4*index_stride];
    unsigned i5 = indices[5*index_stride];
    unsigned i6 = indices[6*index_stride];
    unsigned i7 = indices[7*index_stride];

    simd_float::store8(data + i4*stride_in_floats, t4);
    simd_float::store8(data + i5*stride_in_floats, t5);
    simd_float::store8(data + i6*stride_in_floats, t6);
    simd_float::store8(data + i7*stride_in_floats, t7);
#else
    simd128::transpose32(d0, d1, d2, d3);
    simd128::transpose32(d4, d5, d6, d7);

    unsigned i0 = indices[0*index_stride];
    unsigned i1 = indices[1*index_stride];
    unsigned i2 = indices[2*index_stride];
    unsigned i3 = indices[3*index_stride];

    simd_float::store4(data + i0*stride_in_floats, d0);
    simd_float::store4(data + i1*stride_in_floats, d1);
    simd_float::store4(data + i2*stride_in_floats, d2);
    simd_float::store4(data + i3*stride_in_floats, d3);

    simd_float::store4(data + i0*stride_in_floats + 4, d4);
    simd_float::store4(data + i1*stride_in_floats + 4, d5);
    simd_float::store4(data + i2*stride_in_floats + 4, d6);
    simd_float::store4(data + i3*stride_in_floats + 4, d7);
#endif
}

#ifndef NUDGE_COLLISION_MASKS_CONSISTENT
#   define NUDGE_INTERNAL_CSBM &&   /* inconsistent mode (default): no collision if A don't want to collide with B or B don't want to collide with A */
#else
#   define NUDGE_INTERNAL_CSBM ||   /* consistent mode (like in Bullet): no collision only if A don't want to collide with B and B don't want to collide with A */
#endif

#ifndef NUDGE_COLLIDE_SKIP_BODYFILTERS_MACRO
// original code (used to works when macro was NUDGE_COLLIDE_SKIP_BODIES_MACRO)
//#define NUDGE_COLLIDE_SKIP_BODIES_MACRO(A,B)    (!(A) || !(B)) // Body 0 is the static world and is ignored (original code).
// no-op (works)
//#define NUDGE_COLLIDE_SKIP_BODIES_MACRO(A,B)    (0) /* no-op */
// This is the original code plus a second condition (but with &&). Seems OK, but maybe the new check is redundant... or not?
//#define NUDGE_COLLIDE_SKIP_BODIES_MACRO(A,B)  ((!(A) || !(B)) || (c->bodies.properties[(A)].mass_inverse<=0 && c->bodies.properties[(B)].mass_inverse<=0))
/* De Morgan's laws:
    not (A or B) = (not A) and (not B)
    not (A and B) = (not A) or (not B)
*/
// Q) can we merge the first 2 lines together by applying De Morgan's laws in chain?
// A) not sure, but I don't think so. The condition states that: at least one body must be dynamic and both must be active
//    Maybe we could just write the first line including the second-line flag (without removing any line) to maximise early exiting if both bodies are disabled or removed (DONE).
#define NUDGE_COLLIDE_SKIP_BODYFILTERS_MACRO(a,b)  \
    ( \
        ((a)->flags&BF_IS_STATIC_OR_KINEMATIC_OR_DISABLED_OR_REMOVED && (b)->flags&BF_IS_STATIC_OR_KINEMATIC_OR_DISABLED_OR_REMOVED) /* if no body is dynamic we can skip. */ \
        || (((a)->flags&BF_IS_DISABLED_OR_REMOVED) || ((b)->flags&BF_IS_DISABLED_OR_REMOVED)) /* if one body is disabled or has been removed we can skip */ \
        || (!(((a)->collision_group&(b)->collision_mask) NUDGE_INTERNAL_CSBM ((b)->collision_group&(a)->collision_mask))) \
    )
#endif //NUDGE_COLLIDE_SKIP_BODYFILTERS_MACRO

void collide(context_t* c,BodyConnections body_connections)  {
    assert(c);

    ActiveBodies* active_bodies = &c->active_bodies;
    ContactData* contacts = &c->contact_data;
    BodyData bodies = c->bodies;
    ColliderData colliders = c->colliders;
    const BodyProperties* properties = c->bodies.properties;
    Arena temporary = c->arena;

    // Dbg test [OK]:
    //for (unsigned i=0;i<colliders.boxes.count;i++)   {assert(colliders.boxes.transforms[i].body<bodies.count);}
    //for (unsigned i=0;i<colliders.spheres.count;i++)   {assert(colliders.spheres.transforms[i].body<bodies.count);}

    contacts->count = 0;
    contacts->sleeping_count = 0;
    active_bodies->count = 0;

    const Transform* body_transforms = bodies.transforms;

    unsigned count = colliders.spheres.count + colliders.boxes.count;
    unsigned aligned_count = (count + 7) & (~7);

    assert(count <= (1 << 13)); // Too many colliders. 2^13 (=8192) is currently the maximum.

    AABB* aos_bounds = allocate_array<AABB>(&temporary, aligned_count, 32);

    unsigned box_bounds_offset = 0;
    unsigned sphere_bounds_offset = colliders.boxes.count;

    Transform* transforms = allocate_array<Transform>(&temporary, count, 32);
    uint16_t* collider_tags = allocate_array<uint16_t>(&temporary, count, 32);
    uint16_t* collider_bodies = allocate_array<uint16_t>(&temporary, count, 32);

    if (colliders.boxes.count) {
        for (unsigned i = 0; i < colliders.boxes.count; ++i) {
            Transform transform = colliders.boxes.transforms[i];
            transform = body_transforms[transform.body] * transform;
            transform.body |= (uint32_t)colliders.boxes.tags[i] << 16;  // <--- transform.body loses its bodyId info here!

            float3x3 m = matrix(make_rotation(transform.rotation));

            m.c0 *= colliders.boxes.data[i].size[0];
            m.c1 *= colliders.boxes.data[i].size[1];
            m.c2 *= colliders.boxes.data[i].size[2];

            float3 size = {
                fabsf(m.c0.x) + fabsf(m.c1.x) + fabsf(m.c2.x),
                fabsf(m.c0.y) + fabsf(m.c1.y) + fabsf(m.c2.y),
                fabsf(m.c0.z) + fabsf(m.c1.z) + fabsf(m.c2.z),
            };

            float3 min = make_float3(transform.position) - size;
            float3 max = make_float3(transform.position) + size;

            AABB aabb = {
                min, 0.0f,
                max, 0.0f,
            };

            transforms[i + box_bounds_offset] = transform;
            aos_bounds[i + box_bounds_offset] = aabb;
            collider_tags[i + box_bounds_offset] = colliders.boxes.tags[i];
            collider_bodies[i + box_bounds_offset] = colliders.boxes.transforms[i].body;
        }

        colliders.boxes.transforms = transforms + box_bounds_offset;   // <---
    }

    if (colliders.spheres.count) {
        for (unsigned i = 0; i < colliders.spheres.count; ++i) {
            Transform transform = colliders.spheres.transforms[i];
            transform = body_transforms[transform.body] * transform;
            transform.body |= (uint32_t)colliders.spheres.tags[i] << 16;  // <--- transform.body loses its bodyId info here!

            float radius = colliders.spheres.data[i].radius;

            float3 min = make_float3(transform.position) - make_float3(radius);
            float3 max = make_float3(transform.position) + make_float3(radius);

            AABB aabb = {
                min, 0.0f,
                max, 0.0f,
            };

            transforms[i + sphere_bounds_offset] = transform;
            aos_bounds[i + sphere_bounds_offset] = aabb;
            collider_tags[i + sphere_bounds_offset] = colliders.spheres.tags[i];
            collider_bodies[i + sphere_bounds_offset] = colliders.spheres.transforms[i].body;
        }

        colliders.spheres.transforms = transforms + sphere_bounds_offset;   // <---
    }

    for (unsigned i = count; i < aligned_count; ++i) {
        AABB zero = {};
        aos_bounds[i] = zero;
    }

    // Morton order using the min corner should improve coherence: After some point, all BBs' min points will be outside a BB's max.
    simd4_float scene_min128 = simd_float::load4(&aos_bounds[0].min.x);
    simd4_float scene_max128 = scene_min128;

    for (unsigned i = 1; i < count; ++i) {
        simd4_float p = simd_float::load4(&aos_bounds[i].min.x);
        scene_min128 = simd_float::min(scene_min128, p);
        scene_max128 = simd_float::max(scene_max128, p);
    }

    simd4_float scene_scale128 = simd_float::make4((1<<16)-1) * simd_float::recip(scene_max128 - scene_min128);

    scene_scale128 = simd_float::min(simd128::shuffle32<0,1,2,2>(scene_scale128), simd128::shuffle32<2,2,0,1>(scene_scale128));
    scene_scale128 = simd_float::min(scene_scale128, simd128::shuffle32<1,0,3,2>(scene_scale128));
    scene_min128 = scene_min128 * scene_scale128;

#ifdef DEBUG
    if (simd_float::extract_first_float(scene_scale128) < 2.0f)
        log("Warning: World bounds are very large, which may decrease performance. Perhaps there's a body in free fall?\n");
#endif

#if NUDGE_SIMDV_WIDTH == 256
    simdv_float scene_min = simd256::broadcast(scene_min128);
    simdv_float scene_scale = simd256::broadcast(scene_scale128);
    simdv_int32 index = simd_int32::make8(0 << 16, 1 << 16, 2 << 16, 3 << 16, 4 << 16, 5 << 16, 6 << 16, 7 << 16);
#else
    simdv_float scene_min = scene_min128;
    simdv_float scene_scale = scene_scale128;
    simdv_int32 index = simd_int32::make4(0 << 16, 1 << 16, 2 << 16, 3 << 16);
#endif

    simdv_float scene_min_x = simd128::shuffle32<0,0,0,0>(scene_min);
    simdv_float scene_min_y = simd128::shuffle32<1,1,1,1>(scene_min);
    simdv_float scene_min_z = simd128::shuffle32<2,2,2,2>(scene_min);

    uint64_t* morton_codes = allocate_array<uint64_t>(&temporary, aligned_count, 32);

    for (unsigned i = 0; i < count; i += simdv_width32) {
#if NUDGE_SIMDV_WIDTH == 256
        simd4_float pos_xl = simd_float::load4(&aos_bounds[i+0].min.x);
        simd4_float pos_yl = simd_float::load4(&aos_bounds[i+1].min.x);
        simd4_float pos_zl = simd_float::load4(&aos_bounds[i+2].min.x);
        simd4_float pos_wl = simd_float::load4(&aos_bounds[i+3].min.x);

        simdv_float pos_x = simd::concat(pos_xl, simd_float::load4(&aos_bounds[i+4].min.x));
        simdv_float pos_y = simd::concat(pos_yl, simd_float::load4(&aos_bounds[i+5].min.x));
        simdv_float pos_z = simd::concat(pos_zl, simd_float::load4(&aos_bounds[i+6].min.x));
        simdv_float pos_w = simd::concat(pos_wl, simd_float::load4(&aos_bounds[i+7].min.x));
#else
        simd4_float pos_x = simd_float::load4(&aos_bounds[i+0].min.x);
        simd4_float pos_y = simd_float::load4(&aos_bounds[i+1].min.x);
        simd4_float pos_z = simd_float::load4(&aos_bounds[i+2].min.x);
        simd4_float pos_w = simd_float::load4(&aos_bounds[i+3].min.x);
#endif

        simd128::transpose32(pos_x, pos_y, pos_z, pos_w);

        pos_x = simd_float::msub(pos_x, scene_scale, scene_min_x);
        pos_y = simd_float::msub(pos_y, scene_scale, scene_min_y);
        pos_z = simd_float::msub(pos_z, scene_scale, scene_min_z);

        simdv_int32 lm, hm;
        morton(simd_float::toint(pos_x), simd_float::toint(pos_y), simd_float::toint(pos_z), lm, hm);
        hm = simd::bitwise_or(hm, index);

        simdv_int32 mi0 = simd128::unpacklo32(lm, hm);
        simdv_int32 mi1 = simd128::unpackhi32(lm, hm);

#if NUDGE_SIMDV_WIDTH == 256
        simd_int32::store8((int32_t*)(morton_codes + i) + 0, simd256::permute128<0,2>(mi0, mi1));
        simd_int32::store8((int32_t*)(morton_codes + i) + 8, simd256::permute128<1,3>(mi0, mi1));
#else
        simd_int32::store4((int32_t*)(morton_codes + i) + 0, mi0);
        simd_int32::store4((int32_t*)(morton_codes + i) + 4, mi1);
#endif

        index = simd_int32::add(index, simd_int32::makev(simdv_width32 << 16));
    }

    radix_sort_uint64_low48(morton_codes, count, temporary);
    uint16_t* sorted_indices = allocate_array<uint16_t>(&temporary, aligned_count, 32);

    for (unsigned i = 0; i < count; ++i)
        sorted_indices[i] = (uint16_t)(morton_codes[i] >> 48);

    for (unsigned i = count; i < aligned_count; ++i)
        sorted_indices[i] = 0;

    unsigned bounds_count = aligned_count >> simdv_width32_log2;
    AABBV* bounds = allocate_array<AABBV>(&temporary, bounds_count, 32);

    for (unsigned i = 0; i < count; i += simdv_width32) {
        simdv_float min_x, min_y, min_z, min_w;
        simdv_float max_x, max_y, max_z, max_w;
        load8<sizeof(aos_bounds[0]), 1>(&aos_bounds[0].min.x, sorted_indices + i,
                                        min_x, min_y, min_z, min_w,
                                        max_x, max_y, max_z, max_w);

        simd_float::storev(bounds[i >> simdv_width32_log2].min_x, min_x);
        simd_float::storev(bounds[i >> simdv_width32_log2].max_x, max_x);
        simd_float::storev(bounds[i >> simdv_width32_log2].min_y, min_y);
        simd_float::storev(bounds[i >> simdv_width32_log2].max_y, max_y);
        simd_float::storev(bounds[i >> simdv_width32_log2].min_z, min_z);
        simd_float::storev(bounds[i >> simdv_width32_log2].max_z, max_z);
    }

    for (unsigned i = count; i < aligned_count; ++i) {
        unsigned bounds_group = i >> simdv_width32_log2;
        unsigned bounds_lane = i & (simdv_width32-1);

        bounds[bounds_group].min_x[bounds_lane] = NAN;
        bounds[bounds_group].max_x[bounds_lane] = NAN;
        bounds[bounds_group].min_y[bounds_lane] = NAN;
        bounds[bounds_group].max_y[bounds_lane] = NAN;
        bounds[bounds_group].min_z[bounds_lane] = NAN;
        bounds[bounds_group].max_z[bounds_lane] = NAN;
    }

    // Pack each set of 8 consecutive AABBs into coarse AABBs.
    unsigned coarse_count = aligned_count >> 3;
    unsigned aligned_coarse_count = (coarse_count + (simdv_width32-1)) & (~(simdv_width32-1));

    unsigned coarse_bounds_count = aligned_coarse_count >> simdv_width32_log2;
    AABBV* coarse_bounds = allocate_array<AABBV>(&temporary, coarse_bounds_count, 32);

    for (unsigned i = 0; i < coarse_count; ++i) {
        unsigned start = i << (3 - simdv_width32_log2);

        simd4_float coarse_min_x = simd_float::load4(bounds[start].min_x);
        simd4_float coarse_max_x = simd_float::load4(bounds[start].max_x);
        simd4_float coarse_min_y = simd_float::load4(bounds[start].min_y);
        simd4_float coarse_max_y = simd_float::load4(bounds[start].max_y);
        simd4_float coarse_min_z = simd_float::load4(bounds[start].min_z);
        simd4_float coarse_max_z = simd_float::load4(bounds[start].max_z);

        // Note that the first operand is returned on NaN. The last padded bounds are NaN, so the earlier bounds should be in the first operand.
#if NUDGE_SIMDV_WIDTH == 256
        coarse_min_x = simd_float::min(coarse_min_x, simd_float::load4(bounds[start].min_x + 4));
        coarse_max_x = simd_float::max(coarse_max_x, simd_float::load4(bounds[start].max_x + 4));
        coarse_min_y = simd_float::min(coarse_min_y, simd_float::load4(bounds[start].min_y + 4));
        coarse_max_y = simd_float::max(coarse_max_y, simd_float::load4(bounds[start].max_y + 4));
        coarse_min_z = simd_float::min(coarse_min_z, simd_float::load4(bounds[start].min_z + 4));
        coarse_max_z = simd_float::max(coarse_max_z, simd_float::load4(bounds[start].max_z + 4));
#else
        coarse_min_x = simd_float::min(coarse_min_x, simd_float::load4(bounds[start+1].min_x));
        coarse_max_x = simd_float::max(coarse_max_x, simd_float::load4(bounds[start+1].max_x));
        coarse_min_y = simd_float::min(coarse_min_y, simd_float::load4(bounds[start+1].min_y));
        coarse_max_y = simd_float::max(coarse_max_y, simd_float::load4(bounds[start+1].max_y));
        coarse_min_z = simd_float::min(coarse_min_z, simd_float::load4(bounds[start+1].min_z));
        coarse_max_z = simd_float::max(coarse_max_z, simd_float::load4(bounds[start+1].max_z));
#endif

        coarse_min_x = simd_float::min(coarse_min_x, simd128::shuffle32<2,3,0,1>(coarse_min_x));
        coarse_max_x = simd_float::max(coarse_max_x, simd128::shuffle32<2,3,0,1>(coarse_max_x));
        coarse_min_y = simd_float::min(coarse_min_y, simd128::shuffle32<2,3,0,1>(coarse_min_y));
        coarse_max_y = simd_float::max(coarse_max_y, simd128::shuffle32<2,3,0,1>(coarse_max_y));
        coarse_min_z = simd_float::min(coarse_min_z, simd128::shuffle32<2,3,0,1>(coarse_min_z));
        coarse_max_z = simd_float::max(coarse_max_z, simd128::shuffle32<2,3,0,1>(coarse_max_z));

        coarse_min_x = simd_float::min(coarse_min_x, simd128::shuffle32<1,0,3,2>(coarse_min_x));
        coarse_max_x = simd_float::max(coarse_max_x, simd128::shuffle32<1,0,3,2>(coarse_max_x));
        coarse_min_y = simd_float::min(coarse_min_y, simd128::shuffle32<1,0,3,2>(coarse_min_y));
        coarse_max_y = simd_float::max(coarse_max_y, simd128::shuffle32<1,0,3,2>(coarse_max_y));
        coarse_min_z = simd_float::min(coarse_min_z, simd128::shuffle32<1,0,3,2>(coarse_min_z));
        coarse_max_z = simd_float::max(coarse_max_z, simd128::shuffle32<1,0,3,2>(coarse_max_z));

        unsigned bounds_group = i >> simdv_width32_log2;
        unsigned bounds_lane = i & (simdv_width32-1);

        coarse_bounds[bounds_group].min_x[bounds_lane] = simd_float::extract_first_float(coarse_min_x);
        coarse_bounds[bounds_group].max_x[bounds_lane] = simd_float::extract_first_float(coarse_max_x);
        coarse_bounds[bounds_group].min_y[bounds_lane] = simd_float::extract_first_float(coarse_min_y);
        coarse_bounds[bounds_group].max_y[bounds_lane] = simd_float::extract_first_float(coarse_max_y);
        coarse_bounds[bounds_group].min_z[bounds_lane] = simd_float::extract_first_float(coarse_min_z);
        coarse_bounds[bounds_group].max_z[bounds_lane] = simd_float::extract_first_float(coarse_max_z);
    }

    for (unsigned i = coarse_count; i < aligned_coarse_count; ++i) {
        unsigned bounds_group = i >> simdv_width32_log2;
        unsigned bounds_lane = i & (simdv_width32-1);

        coarse_bounds[bounds_group].min_x[bounds_lane] = NAN;
        coarse_bounds[bounds_group].max_x[bounds_lane] = NAN;
        coarse_bounds[bounds_group].min_y[bounds_lane] = NAN;
        coarse_bounds[bounds_group].max_y[bounds_lane] = NAN;
        coarse_bounds[bounds_group].min_z[bounds_lane] = NAN;
        coarse_bounds[bounds_group].max_z[bounds_lane] = NAN;
    }

    // Test all coarse groups against each other and generate pairs with potential overlap.
    uint32_t* coarse_groups = reserve_array<uint32_t>(&temporary, coarse_count*coarse_count, 32);
    unsigned coarse_group_count = 0;

    for (unsigned i = 0; i < coarse_count; ++i) {
        unsigned bounds_group = i >> simdv_width32_log2;
        unsigned bounds_lane = i & (simdv_width32-1);

        simdv_float min_a_x = simd_float::broadcast_loadv(coarse_bounds[bounds_group].min_x + bounds_lane);
        simdv_float max_a_x = simd_float::broadcast_loadv(coarse_bounds[bounds_group].max_x + bounds_lane);
        simdv_float min_a_y = simd_float::broadcast_loadv(coarse_bounds[bounds_group].min_y + bounds_lane);
        simdv_float max_a_y = simd_float::broadcast_loadv(coarse_bounds[bounds_group].max_y + bounds_lane);
        simdv_float min_a_z = simd_float::broadcast_loadv(coarse_bounds[bounds_group].min_z + bounds_lane);
        simdv_float max_a_z = simd_float::broadcast_loadv(coarse_bounds[bounds_group].max_z + bounds_lane);

        unsigned first = coarse_group_count;

        // Maximum number of colliders is 2^13, i.e., 13 bit indices.
        // i needs 10 bits.
        // j needs 7 or 8 bits.
        // mask needs 4 or 8 bits.
        unsigned ij_bits = (bounds_group << 8) | (i << 16);

        for (unsigned j = bounds_group; j < coarse_bounds_count; ++j) {
            simdv_float min_b_x = simd_float::loadv(coarse_bounds[j].min_x);
            simdv_float max_b_x = simd_float::loadv(coarse_bounds[j].max_x);
            simdv_float min_b_y = simd_float::loadv(coarse_bounds[j].min_y);
            simdv_float max_b_y = simd_float::loadv(coarse_bounds[j].max_y);
            simdv_float min_b_z = simd_float::loadv(coarse_bounds[j].min_z);
            simdv_float max_b_z = simd_float::loadv(coarse_bounds[j].max_z);

            simdv_float inside_x = simd::bitwise_and(simd_float::cmp_gt(max_b_x, min_a_x), simd_float::cmp_gt(max_a_x, min_b_x));
            simdv_float inside_y = simd::bitwise_and(simd_float::cmp_gt(max_b_y, min_a_y), simd_float::cmp_gt(max_a_y, min_b_y));
            simdv_float inside_z = simd::bitwise_and(simd_float::cmp_gt(max_b_z, min_a_z), simd_float::cmp_gt(max_a_z, min_b_z));

            unsigned mask = simd::signmask32(simd::bitwise_and(simd::bitwise_and(inside_x, inside_y), inside_z));

            coarse_groups[coarse_group_count] = mask | ij_bits;
            coarse_group_count += mask != 0;

            ij_bits += 1 << 8;
        }

        // Mask out collisions already handled.
        coarse_groups[first] &= ~((1 << bounds_lane) - 1);
    }

    commit_array<uint32_t>(&temporary, coarse_group_count);

    uint32_t* coarse_pairs = reserve_array<uint32_t>(&temporary, coarse_group_count*simdv_width32, 32);
    unsigned coarse_pair_count = 0;

    for (unsigned i = 0; i < coarse_group_count; ++i) {
        unsigned group = coarse_groups[i];
        unsigned mask = group & 0xff;

        unsigned batch = (group & 0xff00) >> (8 - simdv_width32_log2);
        unsigned other = group & 0xffff0000;

        while (mask) {
            unsigned index = first_set_bit(mask);
            mask &= mask-1;

            coarse_pairs[coarse_pair_count++] = other | (batch + index);
        }
    }

    commit_array<uint32_t>(&temporary, coarse_pair_count);

    // Test AABBs within the coarse pairs.
    uint32_t* groups = reserve_array<uint32_t>(&temporary, coarse_pair_count*16, 32);
    unsigned group_count = 0;

#if NUDGE_SIMDV_WIDTH == 256
    for (unsigned n = 0; n < coarse_pair_count; ++n) {
        unsigned pair = coarse_pairs[n];

        unsigned a = pair >> 16;
        unsigned b = pair & 0xffff;

        unsigned lane_count = 8;

        if (a == b)
            --lane_count;

        if (lane_count + (a << 3) > count)
            lane_count = count - (a << 3);

        // Maximum number of colliders is 2^13, i.e., 13 bit indices.
        // i needs 13 bits.
        // j needs 10 or 11 bits.
        // mask needs 4 or 8 bits.
        unsigned ij_bits = (b << 8) | (a << 22);

        unsigned lower_lane_mask = a == b ? 0xfe00 : 0xffff;

        simdv_float min_b_x = simd_float::loadv(bounds[b].min_x);
        simdv_float max_b_x = simd_float::loadv(bounds[b].max_x);
        simdv_float min_b_y = simd_float::loadv(bounds[b].min_y);
        simdv_float max_b_y = simd_float::loadv(bounds[b].max_y);
        simdv_float min_b_z = simd_float::loadv(bounds[b].min_z);
        simdv_float max_b_z = simd_float::loadv(bounds[b].max_z);

        for (unsigned i = 0; i < lane_count; ++i, ij_bits += (1 << 19)) {
            simdv_float min_a_x = simd_float::broadcast_loadv(bounds[a].min_x + i);
            simdv_float max_a_x = simd_float::broadcast_loadv(bounds[a].max_x + i);
            simdv_float min_a_y = simd_float::broadcast_loadv(bounds[a].min_y + i);
            simdv_float max_a_y = simd_float::broadcast_loadv(bounds[a].max_y + i);
            simdv_float min_a_z = simd_float::broadcast_loadv(bounds[a].min_z + i);
            simdv_float max_a_z = simd_float::broadcast_loadv(bounds[a].max_z + i);

            simdv_float inside_x = simd::bitwise_and(simd_float::cmp_gt(max_b_x, min_a_x), simd_float::cmp_gt(max_a_x, min_b_x));
            simdv_float inside_y = simd::bitwise_and(simd_float::cmp_gt(max_b_y, min_a_y), simd_float::cmp_gt(max_a_y, min_b_y));
            simdv_float inside_z = simd::bitwise_and(simd_float::cmp_gt(max_b_z, min_a_z), simd_float::cmp_gt(max_a_z, min_b_z));

            unsigned mask = simd::signmask32(simd::bitwise_and(simd::bitwise_and(inside_x, inside_y), inside_z));

            // Mask out collisions already handled.
            mask &= lower_lane_mask >> 8;
            lower_lane_mask <<= 1;

            groups[group_count] = mask | ij_bits;
            group_count += mask != 0;
        }
    }
#else
    // TODO: This version is currently much worse than the 256-bit version. We should fix it.
    for (unsigned n = 0; n < coarse_pair_count; ++n) {
        unsigned pair = coarse_pairs[n];

        unsigned a = pair >> 16;
        unsigned b = pair & 0xffff;

        unsigned a_start = a << 3;
        unsigned a_end = a_start + (1 << 3);

        if (a_end > count)
            a_end = count;

        unsigned b_start = b << (3 - simdv_width32_log2);
        unsigned b_end = b_start + (1 << (3 - simdv_width32_log2));

        if (b_end > bounds_count)
            b_end = bounds_count;

        for (unsigned i = a_start; i < a_end; ++i) {
            unsigned bounds_group = i >> simdv_width32_log2;
            unsigned bounds_lane = i & (simdv_width32-1);

            simdv_float min_a_x = simd_float::broadcast_loadv(bounds[bounds_group].min_x + bounds_lane);
            simdv_float max_a_x = simd_float::broadcast_loadv(bounds[bounds_group].max_x + bounds_lane);
            simdv_float min_a_y = simd_float::broadcast_loadv(bounds[bounds_group].min_y + bounds_lane);
            simdv_float max_a_y = simd_float::broadcast_loadv(bounds[bounds_group].max_y + bounds_lane);
            simdv_float min_a_z = simd_float::broadcast_loadv(bounds[bounds_group].min_z + bounds_lane);
            simdv_float max_a_z = simd_float::broadcast_loadv(bounds[bounds_group].max_z + bounds_lane);

            unsigned first = group_count;

            unsigned start = (i+1) >> simdv_width32_log2;

            if (start < b_start)
                start = b_start;

            // Maximum number of colliders is 2^13, i.e., 13 bit indices.
            // i needs 13 bits.
            // j needs 10 or 11 bits.
            // mask needs 4 or 8 bits.
            unsigned ij_bits = (start << 8) | (i << 19);

            for (unsigned j = start; j < b_end; ++j) {
                simdv_float min_b_x = simd_float::loadv(bounds[j].min_x);
                simdv_float max_b_x = simd_float::loadv(bounds[j].max_x);
                simdv_float min_b_y = simd_float::loadv(bounds[j].min_y);
                simdv_float max_b_y = simd_float::loadv(bounds[j].max_y);
                simdv_float min_b_z = simd_float::loadv(bounds[j].min_z);
                simdv_float max_b_z = simd_float::loadv(bounds[j].max_z);

                simdv_float inside_x = simd::bitwise_and(simd_float::cmp_gt(max_b_x, min_a_x), simd_float::cmp_gt(max_a_x, min_b_x));
                simdv_float inside_y = simd::bitwise_and(simd_float::cmp_gt(max_b_y, min_a_y), simd_float::cmp_gt(max_a_y, min_b_y));
                simdv_float inside_z = simd::bitwise_and(simd_float::cmp_gt(max_b_z, min_a_z), simd_float::cmp_gt(max_a_z, min_b_z));

                unsigned mask = simd::signmask32(simd::bitwise_and(simd::bitwise_and(inside_x, inside_y), inside_z));

                groups[group_count] = mask | ij_bits;
                group_count += mask != 0;

                ij_bits += 1 << 8;
            }

            // Mask out collisions already handled.
            if (first < group_count && (groups[first] & 0x7ff00) == (bounds_group << 8))
                groups[first] &= ~((2 << bounds_lane) - 1);
        }
    }
#endif

    commit_array<uint32_t>(&temporary, group_count);

    uint32_t* pairs = reserve_array<uint32_t>(&temporary, group_count*simdv_width32, 32);
    unsigned pair_count = 0;

    for (unsigned i = 0; i < group_count; ++i) {
        unsigned group = groups[i];
        unsigned mask = group & 0xff;

        unsigned batch = (group & 0x7ff00) >> (8 - simdv_width32_log2);
        unsigned base = ((uint32_t)(group >> 19) << 16) | batch;

        while (mask) {
            unsigned index = first_set_bit(mask);
            mask &= mask-1;

            pairs[pair_count++] = base + index;
        }
    }

    commit_array<uint32_t>(&temporary, pair_count);

    for (unsigned i = 0; i < pair_count; ++i) {
        unsigned pair = pairs[i];
        pairs[i] = sorted_indices[pair & 0xffff] | ((uint32_t)sorted_indices[pair >> 16] << 16);
    }

    radix_sort_uint32(pairs, pair_count, temporary);

    // Discard islands of inactive objects at a coarse level, before detailed collisions.
    {
        NUDGE_ARENA_SCOPE(temporary);

        // Find connected sets.
        uint16_t* heights = allocate_array<uint16_t>(&temporary, bodies.count, 16);
        uint16_t* parents = allocate_array<uint16_t>(&temporary, bodies.count, 16);

        memset(heights, 0, sizeof(heights[0])*bodies.count);
        memset(parents, 0xff, sizeof(parents[0])*bodies.count);

        for (unsigned i = 0; i < body_connections.count; ++i) {
            BodyPair pair = body_connections.data[i];

            unsigned a = pair.a;
            unsigned b = pair.b;
            BodyFilter *a_filter=&c->bodies.filters[a], *b_filter=&c->bodies.filters[b];


            if (NUDGE_COLLIDE_SKIP_BODYFILTERS_MACRO(a_filter,b_filter)) continue;
            // Body 0 is the static world and is ignored.
//            if (!a || !b) continue;

            // Determine the root of a and b.
            unsigned a_root = a;
            unsigned a_parent = parents[a];

            for (unsigned parent = a_parent; parent != 0xffff; parent = parents[a_root])
                a_root = parent;

            unsigned b_root = b;
            unsigned b_parent = parents[b];

            for (unsigned parent = b_parent; parent != 0xffff; parent = parents[b_root])
                b_root = parent;

            if (a_root == b_root)
                continue;

            // Put a and b under the same root.
            unsigned a_height = heights[a_root];
            unsigned b_height = heights[b_root];

            unsigned root;

            if (a_height < b_height) {
                parents[a_root] = b_root;
                root = b_root;
            }
            else {
                parents[b_root] = a_root;
                root = a_root;
            }

            if (a_height == b_height) // Height of subtree increased.
                heights[a_root] = a_height+1;

            // Propagate the root to make subsequent iterations faster.
            if (a_root != a) {
                while (a_parent != a_root) {
                    unsigned next = parents[a_parent];
                    parents[a] = root;

                    a = a_parent;
                    a_parent = next;
                }
            }

            if (b_root != b) {
                while (b_parent != b_root) {
                    unsigned next = parents[b_parent];
                    parents[b] = root;

                    b = b_parent;
                    b_parent = next;
                }
            }
        }

        for (unsigned i = 0; i < pair_count; ++i) {
            unsigned pair = pairs[i];

            unsigned a = collider_bodies[pair & 0xffff];
            unsigned b = collider_bodies[pair >> 16];

            BodyFilter *a_filter=&c->bodies.filters[a], *b_filter=&c->bodies.filters[b];

            if (NUDGE_COLLIDE_SKIP_BODYFILTERS_MACRO(a_filter,b_filter)) continue;
            // Body 0 is the static world and is ignored.
//            if (!a || !b) continue;

            // Determine the root of a and b.
            unsigned a_root = a;
            unsigned a_parent = parents[a];

            for (unsigned parent = a_parent; parent != 0xffff; parent = parents[a_root])
                a_root = parent;

            unsigned b_root = b;
            unsigned b_parent = parents[b];

            for (unsigned parent = b_parent; parent != 0xffff; parent = parents[b_root])
                b_root = parent;

            if (a_root == b_root)
                continue;

            // Put a and b under the same root.
            unsigned a_height = heights[a_root];
            unsigned b_height = heights[b_root];

            unsigned root;

            if (a_height < b_height) {
                parents[a_root] = b_root;
                root = b_root;
            }
            else {
                parents[b_root] = a_root;
                root = a_root;
            }

            if (a_height == b_height) // Height of subtree increased.
                heights[a_root] = a_height+1;

            // Propagate the root to make subsequent iterations faster.
            if (a_root != a) {
                while (a_parent != a_root) {
                    unsigned next = parents[a_parent];
                    parents[a] = root;

                    a = a_parent;
                    a_parent = next;
                }
            }

            if (b_root != b) {
                while (b_parent != b_root) {
                    unsigned next = parents[b_parent];
                    parents[b] = root;

                    b = b_parent;
                    b_parent = next;
                }
            }
        }

        // Identify a numbered set for each body.
        unsigned set_count = 0;
        uint16_t* sets = heights;
        memset(sets, 0xff, sizeof(sets[0])*bodies.count);

        for (unsigned i = 0 /* was 1 */; i < bodies.count; ++i) {
            //if (bodies.filters[i].flags&BF_IS_STATIC_OR_KINEMATIC_OR_DISABLED_OR_REMOVED) continue; // new
            if (bodies.filters[i].flags&BF_IS_DISABLED_OR_REMOVED) continue;
            unsigned root = parents[i];

            for (unsigned parent = root; parent != 0xffff; parent = parents[root])
                root = parent;

            if (root == 0xffff)
                root = i;

            if (sets[root] == 0xffff)
                sets[root] = set_count++;

            sets[i] = sets[root];
        }

        sets[0] = 0;

        // Determine active sets.
        uint8_t* active = allocate_array<uint8_t>(&temporary, set_count, 16);
        memset(active, 0, sizeof(active[0])*set_count);

        for (unsigned i = 0 /* was 1 */; i < bodies.count; ++i) {
            if (bodies.idle_counters[i] != 0xff
                    //&& !(c->bodies.filters[i].flags&BF_IS_STATIC_OR_KINEMATIC_OR_DISABLED_OR_REMOVED)    // new
                    && !(c->bodies.filters[i].flags&BF_IS_DISABLED_OR_REMOVED)
                    )
                active[sets[i]] = 1;
        }

        // Remove inactive pairs.
        unsigned removed = 0;

        for (unsigned i = 0; i < pair_count; ++i) {
            unsigned pair = pairs[i];

            unsigned a = collider_bodies[pair & 0xffff];
            unsigned b = collider_bodies[pair >> 16];

            if (a == b) {
                ++removed;
                continue;
            }

            unsigned set = sets[a] | sets[b];

            if (active[set]) {
                pairs[i-removed] = pair;
            }
            else {
                unsigned a = collider_tags[pair & 0xffff];
                unsigned b = collider_tags[pair >> 16];

                contacts->sleeping_pairs[contacts->sleeping_count++] = a > b ? a | (b << 16): b | (a << 16);
                ++removed;
            }
        }

        pair_count -= removed;
    }

    uint32_t bucket_sizes[4] = {};

    for (unsigned i = 0; i < pair_count; ++i) {
        unsigned pair = pairs[i];

        unsigned a = pair & 0xffff;
        unsigned b = pair >> 16;

        a = a >= colliders.boxes.count ? 1 : 0;
        b = b >= colliders.boxes.count ? 2 : 0;

        unsigned ab = a | b;

        ++bucket_sizes[ab];
    }

    uint32_t bucket_offsets[4] = {
        0,
        ((bucket_sizes[0] + 7) & ~3),
        ((bucket_sizes[0] + 7) & ~3) + bucket_sizes[1],
        ((bucket_sizes[0] + 7) & ~3) + bucket_sizes[1] + bucket_sizes[2],
    };

    uint32_t written_per_bucket[4] = { bucket_offsets[0], bucket_offsets[1], bucket_offsets[2], bucket_offsets[3] };

    uint32_t* partitioned_pairs = allocate_array<uint32_t>(&temporary, pair_count + 7, 16); // Padding is required.

    for (unsigned i = 0; i < pair_count; ++i) {
        unsigned pair = pairs[i];

        unsigned a = pair & 0xffff;
        unsigned b = pair >> 16;

        a = a >= colliders.boxes.count ? 1 : 0;
        b = b >= colliders.boxes.count ? 2 : 0;

        unsigned ab = a | b;

        partitioned_pairs[written_per_bucket[ab]++] = pair;
    }

    for (unsigned i = 0; i < bucket_sizes[2]; ++i) {
        unsigned index = bucket_offsets[2] + i;
        unsigned pair = partitioned_pairs[index];

        partitioned_pairs[index] = (pair >> 16) | (pair << 16);
    }

    contacts->count += box_box_collide(partitioned_pairs, bucket_sizes[0], colliders.boxes.data, colliders.boxes.transforms, contacts->data + contacts->count, contacts->bodies + contacts->count, contacts->tags + contacts->count, properties, temporary);

    // TODO: SIMD-optimize this loop.
    for (unsigned i = 0; i < bucket_sizes[1] + bucket_sizes[2]; ++i) {
        unsigned pair = partitioned_pairs[bucket_offsets[1] + i];

        unsigned a = pair >> 16;
        unsigned b = pair & 0xffff;

        b -= colliders.boxes.count;

        BoxCollider box = colliders.boxes.data[a];
        SphereCollider sphere = colliders.spheres.data[b];

        // make dbg asserts optional ---------------------------
        unsigned bodyA = c->colliders.boxes.transforms[a].body;
        unsigned bodyB = c->colliders.spheres.transforms[b].body;
        /*assert(bodyA<c->bodies.count);
        assert(bodyB<c->bodies.count);
        assert(c->bodies.infos[bodyA].num_boxes>0);
        assert(c->bodies.infos[bodyB].num_spheres>0);*/
        const float friction = NUDGE_FRICTION_MODEL(properties[bodyA].friction,properties[bodyB].friction);
        //if (friction!=0.5f) log("box_sphere_collide: %u (friction:%1.f);  %u (friction:%1.f);  contact_friction = %1.3f\n",bodyA,properties[bodyA].friction,bodyB,properties[bodyB].friction,friction);
        // ------------------------------------------------------

        contacts->tags[contacts->count] = (uint64_t)((colliders.boxes.transforms[a].body >> 16) | (colliders.spheres.transforms[b].body & 0xffff0000)) << 32;
        contacts->count += box_sphere_collide(box, sphere, colliders.boxes.transforms[a], colliders.spheres.transforms[b], contacts->data + contacts->count, contacts->bodies + contacts->count, friction);
    }

    // TODO: SIMD-optimize this loop.
    for (unsigned i = 0; i < bucket_sizes[3]; ++i) {
        unsigned pair = partitioned_pairs[bucket_offsets[3] + i];

        unsigned a = pair >> 16;
        unsigned b = pair & 0xffff;

        a -= colliders.boxes.count;
        b -= colliders.boxes.count;

        SphereCollider sphere_a = colliders.spheres.data[a];
        SphereCollider sphere_b = colliders.spheres.data[b];

        // make dbg asserts optional ----------------------------
        unsigned bodyA = c->colliders.spheres.transforms[a].body;
        unsigned bodyB = c->colliders.spheres.transforms[b].body;
        /*assert(bodyA<c->bodies.count);
        assert(bodyB<c->bodies.count);
        assert(c->bodies.infos[bodyA].num_spheres>0);
        assert(c->bodies.infos[bodyB].num_spheres>0);*/
        const float friction = NUDGE_FRICTION_MODEL(properties[bodyA].friction,properties[bodyB].friction);
        //if (friction!=0.5f) log("sphere_sphere_collide: %u (friction:%1.f);  %u (friction:%1.f);  contact_friction = %1.3f\n",bodyA,properties[bodyA].friction,bodyB,properties[bodyB].friction,friction);
        // ------------------------------------------------------

        contacts->tags[contacts->count] = (uint64_t)((colliders.spheres.transforms[a].body >> 16) | (colliders.spheres.transforms[b].body & 0xffff0000)) << 32;
        contacts->count += sphere_sphere_collide(sphere_a, sphere_b, colliders.spheres.transforms[a], colliders.spheres.transforms[b], contacts->data + contacts->count, contacts->bodies + contacts->count, friction);
    }

    // Discard islands of inactive objects at a fine level.
    {
        NUDGE_ARENA_SCOPE(temporary);

        // Find connected sets.
        uint16_t* heights = allocate_array<uint16_t>(&temporary, bodies.count, 16);
        uint16_t* parents = allocate_array<uint16_t>(&temporary, bodies.count, 16);

        memset(heights, 0, sizeof(heights[0])*bodies.count);
        memset(parents, 0xff, sizeof(parents[0])*bodies.count);

        for (unsigned i = 0; i < body_connections.count; ++i) {
            BodyPair pair = body_connections.data[i];

            unsigned a = pair.a;
            unsigned b = pair.b;

            BodyFilter *a_filter=&c->bodies.filters[a], *b_filter=&c->bodies.filters[b];

            if (NUDGE_COLLIDE_SKIP_BODYFILTERS_MACRO(a_filter,b_filter)) continue;
            // Body 0 is the static world and is ignored.
//            if (!a || !b) continue;

            // Determine the root of a and b.
            unsigned a_root = a;
            unsigned a_parent = parents[a];

            for (unsigned parent = a_parent; parent != 0xffff; parent = parents[a_root])
                a_root = parent;

            unsigned b_root = b;
            unsigned b_parent = parents[b];

            for (unsigned parent = b_parent; parent != 0xffff; parent = parents[b_root])
                b_root = parent;

            if (a_root == b_root)
                continue;

            // Put a and b under the same root.
            unsigned a_height = heights[a_root];
            unsigned b_height = heights[b_root];

            unsigned root;

            if (a_height < b_height) {
                parents[a_root] = b_root;
                root = b_root;
            }
            else {
                parents[b_root] = a_root;
                root = a_root;
            }

            if (a_height == b_height) // Height of subtree increased.
                heights[a_root] = a_height+1;

            // Propagate the root to make subsequent iterations faster.
            if (a_root != a) {
                while (a_parent != a_root) {
                    unsigned next = parents[a_parent];
                    parents[a] = root;

                    a = a_parent;
                    a_parent = next;
                }
            }

            if (b_root != b) {
                while (b_parent != b_root) {
                    unsigned next = parents[b_parent];
                    parents[b] = root;

                    b = b_parent;
                    b_parent = next;
                }
            }
        }

        for (unsigned i = 0; i < contacts->count; ) {
            unsigned a = contacts->bodies[i].a;
            unsigned b = contacts->bodies[i].b;

            do {
                ++i;
            }
            while (i < contacts->count && contacts->bodies[i].a == a && contacts->bodies[i].b == b);

            BodyFilter *a_filter=&c->bodies.filters[a], *b_filter=&c->bodies.filters[b];

            if (NUDGE_COLLIDE_SKIP_BODYFILTERS_MACRO(a_filter,b_filter)) continue;
            // Body 0 is the static world and is ignored.
 //           if (!a || !b) continue;

            // Determine the root of a and b.
            unsigned a_root = a;
            unsigned a_parent = parents[a];

            for (unsigned parent = a_parent; parent != 0xffff; parent = parents[a_root])
                a_root = parent;

            unsigned b_root = b;
            unsigned b_parent = parents[b];

            for (unsigned parent = b_parent; parent != 0xffff; parent = parents[b_root])
                b_root = parent;

            if (a_root == b_root)
                continue;

            // Put a and b under the same root.
            unsigned a_height = heights[a_root];
            unsigned b_height = heights[b_root];

            unsigned root;

            if (a_height < b_height) {
                parents[a_root] = b_root;
                root = b_root;
            }
            else {
                parents[b_root] = a_root;
                root = a_root;
            }

            if (a_height == b_height) // Height of subtree increased.
                heights[a_root] = a_height+1;

            // Propagate the root to make subsequent iterations faster.
            if (a_root != a) {
                while (a_parent != a_root) {
                    unsigned next = parents[a_parent];
                    parents[a] = root;

                    a = a_parent;
                    a_parent = next;
                }
            }

            if (b_root != b) {
                while (b_parent != b_root) {
                    unsigned next = parents[b_parent];
                    parents[b] = root;

                    b = b_parent;
                    b_parent = next;
                }
            }
        }

        // Identify a numbered set for each body.
        unsigned set_count = 0;
        uint16_t* sets = heights;
        memset(sets, 0xff, sizeof(sets[0])*bodies.count);

        for (unsigned i = 0 /* was 1 */; i < bodies.count; ++i) {
            //if (bodies.filters[i].flags&BF_IS_STATIC_OR_KINEMATIC_OR_DISABLED_OR_REMOVED) continue; // new
            if (bodies.filters[i].flags&BF_IS_DISABLED_OR_REMOVED) continue;
            unsigned root = parents[i];

            for (unsigned parent = root; parent != 0xffff; parent = parents[root])
                root = parent;

            if (root == 0xffff)
                root = i;

            if (sets[root] == 0xffff)
                sets[root] = set_count++;

            sets[i] = sets[root];
        }

        sets[0] = 0;

        // Determine active sets.
        uint8_t* active = allocate_array<uint8_t>(&temporary, set_count, 16);
        memset(active, 0, sizeof(active[0])*set_count);

        for (unsigned i = 0 /* was 1 */; i < bodies.count; ++i) {
            if (bodies.idle_counters[i] != 0xff
                    //&& !(bodies.filters[i].flags&BF_IS_STATIC_OR_KINEMATIC_OR_DISABLED_OR_REMOVED)    // new
                    && !(bodies.filters[i].flags&BF_IS_DISABLED_OR_REMOVED)    // new
                    )
                active[sets[i]] = 1;
        }

        // Determine active bodies.
        for (unsigned i = 0 /* was 1 */; i < bodies.count; ++i) {
            //if (bodies.filters[i].flags&BF_IS_STATIC_OR_KINEMATIC_OR_DISABLED_OR_REMOVED)  continue;  // new
            if (bodies.filters[i].flags&BF_IS_DISABLED_OR_REMOVED)  continue;  // new
            unsigned set = sets[i];

            if (active[set])
                active_bodies->indices[active_bodies->count++] = i;
        }

        // Remove inactive contacts.
        unsigned removed = 0;

        for (unsigned i = 0; i < contacts->count; ) {
            unsigned a = contacts->bodies[i].a;
            unsigned b = contacts->bodies[i].b;
            unsigned tag = contacts->tags[i] >> 32;

            unsigned span = 0;

            do {
                ++span;
            }
            while (i+span < contacts->count && (contacts->tags[i+span] >> 32) == tag);

            unsigned set = sets[a] | sets[b];

            if (active[set]) {
                for (unsigned j = 0; j < span; ++j) {
                    contacts->tags[i+j-removed] = contacts->tags[i+j];
                    contacts->data[i+j-removed] = contacts->data[i+j];
                    contacts->bodies[i+j-removed] = contacts->bodies[i+j];
                }
            }
            else {
                contacts->sleeping_pairs[contacts->sleeping_count++] = tag;
                removed += span;
            }

            i += span;
        }

        contacts->count -= removed;
    }

    radix_sort_uint32(contacts->sleeping_pairs, contacts->sleeping_count, temporary);
}

struct ContactImpulseData {
    uint32_t* sorted_contacts;

    CachedContactImpulse* culled_data;
    uint64_t* culled_tags;
    unsigned culled_count;

    CachedContactImpulse* data;
};

ContactImpulseData* read_cached_impulses(ContactCache contact_cache, ContactData contacts, Arena* memory) {
    ContactImpulseData* data = allocate_struct<ContactImpulseData>(memory, 64);

    // Sort contacts based on tag so that they can be quickly matched against the contact cache.
    uint32_t* sorted_contacts = allocate_array<uint32_t>(memory, contacts.count, 16);
    data->sorted_contacts = sorted_contacts;
    {
        Arena temporary = *memory;
        uint32_t* contact_keys = allocate_array<uint32_t>(&temporary, contacts.count, 16);

        for (unsigned i = 0; i < contacts.count; ++i) {
            sorted_contacts[i] = i;
            contact_keys[i] = (uint32_t)contacts.tags[i];
        }

        radix_sort_uint32_x2(contact_keys, sorted_contacts, contacts.count, temporary);

        for (unsigned i = 0; i < contacts.count; ++i) {
            unsigned index = sorted_contacts[i];
            contact_keys[i] = (uint32_t)(contacts.tags[index] >> 32);
        }

        radix_sort_uint32_x2(contact_keys, sorted_contacts, contacts.count, temporary);
    }

    // Gather warm start impulses and store away culled impulses for sleeping pairs.
    CachedContactImpulse* culled_data = allocate_array<CachedContactImpulse>(memory, contact_cache.count, 16);
    uint64_t* culled_tags = allocate_array<uint64_t>(memory, contact_cache.count, 16);
    unsigned culled_count = 0;

    CachedContactImpulse* contact_impulses = allocate_array<CachedContactImpulse>(memory, contacts.count, 32);
    data->data = contact_impulses;

    unsigned cached_contact_offset = 0;
    unsigned sleeping_pair_offset = 0;

    for (unsigned i = 0; i < contacts.count; ++i) {
        unsigned index = sorted_contacts[i];
        uint64_t tag = contacts.tags[index];

        CachedContactImpulse cached_impulse = {};

        uint64_t cached_tag;
        while (cached_contact_offset < contact_cache.count && (cached_tag = contact_cache.tags[cached_contact_offset]) < tag) {
            unsigned cached_pair = cached_tag >> 32;

            while (sleeping_pair_offset < contacts.sleeping_count && contacts.sleeping_pairs[sleeping_pair_offset] < cached_pair)
                ++sleeping_pair_offset;

            if (sleeping_pair_offset < contacts.sleeping_count && contacts.sleeping_pairs[sleeping_pair_offset] == cached_pair) {
                culled_data[culled_count] = contact_cache.data[cached_contact_offset];
                culled_tags[culled_count] = contact_cache.tags[cached_contact_offset];
                ++culled_count;
            }

            ++cached_contact_offset;
        }

        if (cached_contact_offset < contact_cache.count && contact_cache.tags[cached_contact_offset] == tag)
            cached_impulse = contact_cache.data[cached_contact_offset];

        contact_impulses[index] = cached_impulse;
    }

    for (; cached_contact_offset < contact_cache.count && sleeping_pair_offset < contacts.sleeping_count; ) {
        unsigned a = contact_cache.tags[cached_contact_offset] >> 32;
        unsigned b = contacts.sleeping_pairs[sleeping_pair_offset];

        if (a < b) {
            ++cached_contact_offset;
        }
        else if (a == b) {
            culled_data[culled_count] = contact_cache.data[cached_contact_offset];
            culled_tags[culled_count] = contact_cache.tags[cached_contact_offset];
            ++culled_count;
            ++cached_contact_offset;
        }
        else {
            ++sleeping_pair_offset;
        }
    }

    data->culled_data = culled_data;
    data->culled_tags = culled_tags;
    data->culled_count = culled_count;

    return data;
}

void write_cached_impulses(ContactCache* contact_cache, ContactData contacts, ContactImpulseData* contact_impulses) {
    uint32_t* sorted_contacts = contact_impulses->sorted_contacts;

    CachedContactImpulse* culled_data = contact_impulses->culled_data;
    uint64_t* culled_tags = contact_impulses->culled_tags;
    unsigned culled_count = contact_impulses->culled_count;

    // Cache impulses.
    assert(contact_cache->capacity >= contacts.count + culled_count); // Out of space in contact cache.
    contact_cache->count = contacts.count + culled_count;
    {
        // Pick sort from contacts and culled impulses.
        unsigned i = 0, j = 0, k = 0;

        while (i < contacts.count && j < culled_count) {
            unsigned index = sorted_contacts[i];

            uint64_t a = contacts.tags[index];
            uint64_t b = culled_tags[j];

            if (a < b) {
                contact_cache->tags[k] = contacts.tags[index];
                contact_cache->data[k] = contact_impulses->data[index];
                ++i;
            }
            else {
                contact_cache->tags[k] = culled_tags[j];
                contact_cache->data[k] = culled_data[j];
                ++j;
            }

            ++k;
        }

        for (; i < contacts.count; ++i) {
            unsigned index = sorted_contacts[i];

            contact_cache->tags[k] = contacts.tags[index];
            contact_cache->data[k] = contact_impulses->data[index];
            ++k;
        }

        for (; j < culled_count; ++j) {
            contact_cache->tags[k] = culled_tags[j];
            contact_cache->data[k] = culled_data[j];
            ++k;
        }
    }
}

struct ContactConstraintData {
    unsigned contact_count;
    InertiaTransform* momentum_to_velocity;
    uint32_t* constraint_to_contact;

    ContactConstraintV* constraints;
    ContactConstraintStateV* constraint_states;
    unsigned constraint_batches;
};

ContactConstraintData* setup_contact_constraints(context_t* c, ContactImpulseData* contact_impulses, Arena* memory) {
    // TODO: We should investigate better evaluation order for contacts.
    //ActiveBodies& active_bodies = c->active_bodies;
    const float allowed_penetration = c->simulation_params.penetration_allowed_amount;
    const float bias_factor = c->simulation_params.penetration_bias_factor;

    ContactData& contacts = c->contact_data;
    BodyData& bodies = c->bodies;

    uint32_t* contact_order = contact_impulses->sorted_contacts;

    ContactConstraintData* data = allocate_struct<ContactConstraintData>(memory, 64);
    data->contact_count = contacts.count;

    InertiaTransform* momentum_to_velocity = allocate_array<InertiaTransform>(memory, bodies.count, 32);
    data->momentum_to_velocity = momentum_to_velocity;

    // TODO: Consider SIMD-optimizing this loop.
    // TODO: Don't compute anything for inactive bodies.    // original nudge comment
    for (unsigned i = 0; i < bodies.count; ++i)     {       // original nudge code
    //for (unsigned j = 0,i=0; j < active_bodies.count; ++j)    {i=active_bodies.indices[j];   // naive attempt 1 [FAILED]
        //if (bodies.filters[i].idle_counter!=0xff || bodies.filters[i].flags&BF_IS_STATIC_OR_KINEMATIC)    {   // attempt 2 [FAILED]
        if (bodies.filters[i].flags&BF_IS_DYNAMIC)  {                           // attempt 3 [WORKS?!?]
        //if (bodies.filters[i].idle_counter!=0xff)  {// attempt 4 [Good, but artifacts when objects wake up from sleeping: they seem to start sinking again... we probably must reset their momentum somewhere (?)]
            Rotation rotation = make_rotation(bodies.transforms[i].rotation);
            float3 inertia_inverse = make_float3(bodies.properties[i].inertia_inverse);

            float3x3 m = matrix(rotation);

            InertiaTransform transform = {};

            transform.xx = inertia_inverse.x*m.c0.x*m.c0.x + inertia_inverse.y*m.c1.x*m.c1.x + inertia_inverse.z*m.c2.x*m.c2.x;
            transform.yy = inertia_inverse.x*m.c0.y*m.c0.y + inertia_inverse.y*m.c1.y*m.c1.y + inertia_inverse.z*m.c2.y*m.c2.y;
            transform.zz = inertia_inverse.x*m.c0.z*m.c0.z + inertia_inverse.y*m.c1.z*m.c1.z + inertia_inverse.z*m.c2.z*m.c2.z;
            transform.xy = inertia_inverse.x*m.c0.x*m.c0.y + inertia_inverse.y*m.c1.x*m.c1.y + inertia_inverse.z*m.c2.x*m.c2.y;
            transform.xz = inertia_inverse.x*m.c0.x*m.c0.z + inertia_inverse.y*m.c1.x*m.c1.z + inertia_inverse.z*m.c2.x*m.c2.z;
            transform.yz = inertia_inverse.x*m.c0.y*m.c0.z + inertia_inverse.y*m.c1.y*m.c1.z + inertia_inverse.z*m.c2.y*m.c2.z;

            momentum_to_velocity[i] = transform;
            bodies.momentum[i].unused0 = bodies.properties[i].mass_inverse;
        }
        else {memset(&momentum_to_velocity[i],0,sizeof(InertiaTransform));bodies.momentum[i].unused0 = 0;} // attempt 3 [WORKS?!?]
        //}   // attempt 2
    }

    CachedContactImpulse* impulses = contact_impulses->data;

    uint32_t* constraint_to_contact = allocate_array<uint32_t>(memory, contacts.count*simdv_width32, 32);
    data->constraint_to_contact = constraint_to_contact;

    // Schedule contacts so there are no conflicts within a SIMD width.
    ContactSlotV* contact_slots = reserve_array<ContactSlotV>(memory, contacts.count, 32);
    unsigned contact_slot_count = 0;
    {
        Arena temporary = *memory;
        commit_array<ContactSlotV>(&temporary, contacts.count);
#       ifndef NUDGE_SETUP_CONTACT_CONSTRAINTS_BUCKET_COUNT
#           define NUDGE_SETUP_CONTACT_CONSTRAINTS_BUCKET_COUNT (16)
#       endif
        static const unsigned bucket_count = NUDGE_SETUP_CONTACT_CONSTRAINTS_BUCKET_COUNT;  // (16)

        ContactPairV* vacant_pair_buckets[bucket_count];
        ContactSlotV* vacant_slot_buckets[bucket_count];
        unsigned bucket_vacancy_count[bucket_count] = {};

        simdv_int32 invalid_index = simd_int32::makev(~0u);

        assert(temporary.size>bucket_count*((2*contacts.count+1)+31)*sizeof(ContactPairV));
        /*if (contacts.count>600) {
            log("[%llu] contacts.count:%u temporary.size:%lu min_size=%lu\n",
                   c->simulation_params.num_frames,contacts.count,temporary.size,c->simulation_params.min_remaining_arena_size);
            flush();
        }*/
#       ifdef ORIGINAL_CODE
        for (unsigned i = 0; i < bucket_count; ++i) {
            vacant_pair_buckets[i] = allocate_array<ContactPairV>(&temporary, contacts.count+1, 32);
            vacant_slot_buckets[i] = allocate_array<ContactSlotV>(&temporary, contacts.count, 32);

            // Add padding with invalid data so we don't have to range check.
            simd_int32::storev((int32_t*)vacant_pair_buckets[i]->ab, invalid_index);
        }
#       else
        assert(sizeof(ContactPairV)==sizeof(ContactSlotV));
        const unsigned stride = (2*contacts.count+1);
        ContactPairV* unified_alloc = allocate_array<ContactPairV>(&temporary, bucket_count*stride, 32);
        for (unsigned i = 0; i < bucket_count; ++i) {
            vacant_pair_buckets[i] = unified_alloc;
            vacant_slot_buckets[i] = (ContactSlotV*) &unified_alloc[contacts.count+1];
            unified_alloc+=stride;
            // Add padding with invalid data so we don't have to range check.
            simd_int32::storev((int32_t*)vacant_pair_buckets[i]->ab, invalid_index);
        }
#       endif

        for (unsigned i = 0; i < contacts.count; ++i) {
            unsigned index = contact_order[i];
            BodyPair active_bodies = contacts.bodies[index];

            unsigned bucket = i % bucket_count;
            ContactPairV* vacant_pairs = vacant_pair_buckets[bucket];
            ContactSlotV* vacant_slots = vacant_slot_buckets[bucket];
            unsigned vacancy_count = bucket_vacancy_count[bucket];

            BodyFilter *a_filter=&c->bodies.filters[active_bodies.a], *b_filter=&c->bodies.filters[active_bodies.b];
            // new: [Optimizable by unwrapping]
            if (NUDGE_COLLIDE_SKIP_BODYFILTERS_MACRO(a_filter,b_filter))
                continue; // but I'm not sure if this early exit is harmless...

            // Ignore dependencies on body 0. // original note: originally body 0 was reserved for static background (and I don't know if now this relation can be completely broken...)
            //unsigned ca = active_bodies.a ? active_bodies.a : active_bodies.b;
            //unsigned cb = active_bodies.b ? active_bodies.b : active_bodies.a;

            // Attempt 1 to replace the 2 lines above
            unsigned ca = a_filter->flags&BF_IS_DYNAMIC ? active_bodies.a : active_bodies.b;    // or !=0?
            unsigned cb = b_filter->flags&BF_IS_DYNAMIC ? active_bodies.b : active_bodies.a;

            // Attempt number 2
            //unsigned ca = active_bodies.a;
            //unsigned cb = active_bodies.b;

            //if (b_filter->flags&BF_IS_STATIC_OR_KINEMATIC) {ca = active_bodies.b;cb = active_bodies.a;}    // not sure if with this the first body of the contact data pair is always static/kinematic (if present)

            //assert(ca!=cb); // asserts (probably in original code too)!
            //assert(!(a_filter->flags&BF_IS_STATIC_OR_KINEMATIC && b_filter->flags&BF_IS_STATIC_OR_KINEMATIC));

#ifdef __AVX2__
            __m256i a = _mm256_set1_epi16(ca);
            __m256i b = _mm256_set1_epi16(cb);

            __m256i scheduled_a_b;

            unsigned j = 0;

            for (;; ++j) {
                scheduled_a_b = _mm256_load_si256((const __m256i*)vacant_pairs[j].ab);

                __m256i conflict = _mm256_packs_epi16(_mm256_cmpeq_epi16(a, scheduled_a_b), _mm256_cmpeq_epi16(b, scheduled_a_b));

                if (!_mm256_movemask_epi8(conflict))
                    break;
            }

            unsigned lane = first_set_bit((unsigned)_mm256_movemask_ps(_mm256_castsi256_ps(_mm256_cmpeq_epi32(scheduled_a_b, invalid_index))));
#else
            __m128i a = _mm_set1_epi16(ca);
            __m128i b = _mm_set1_epi16(cb);

            __m128i scheduled_a_b;

            unsigned j = 0;

            for (;; ++j) {
                scheduled_a_b = _mm_load_si128((const __m128i*)vacant_pairs[j].ab);

                __m128i conflict = _mm_packs_epi16(_mm_cmpeq_epi16(a, scheduled_a_b), _mm_cmpeq_epi16(b, scheduled_a_b));

                if (!_mm_movemask_epi8(conflict))
                    break;
            }

            unsigned lane = first_set_bit((unsigned)_mm_movemask_ps(_mm_castsi128_ps(_mm_cmpeq_epi32(scheduled_a_b, invalid_index))));
#endif

            ContactSlotV* slot = vacant_slots + j;
            ContactPairV* pair = vacant_pairs + j;

            slot->indices[lane] = index;

#ifdef __AVX2__
            _mm_store_ss((float*)pair->ab + lane, _mm_castsi128_ps(_mm_unpacklo_epi16(simd::extract_low(a), simd::extract_low(b))));
#else
            _mm_store_ss((float*)pair->ab + lane, _mm_castsi128_ps(_mm_unpacklo_epi16(a, b)));
#endif

            if (j == vacancy_count) {
                ++vacancy_count;
            }
            else if (lane == simdv_width32-1) {
                simdv_int32 indices = simd_int32::loadv((const int32_t*)slot->indices);

                --vacancy_count;

                ContactPairV* last_pair = vacant_pairs + vacancy_count;
                ContactSlotV* last_slot = vacant_slots + vacancy_count;

                simd_int32::storev((int32_t*)contact_slots[contact_slot_count++].indices, indices);

                *pair = *last_pair;
                *slot = *last_slot;
            }
            else {
                continue;
            }

            // Store count and maintain padding.
            bucket_vacancy_count[bucket] = vacancy_count;
            simd_int32::storev((int32_t*)vacant_pairs[vacancy_count].ab, invalid_index);
        }

        for (unsigned i = 0; i < bucket_count; ++i) {
            ContactPairV* vacant_pairs = vacant_pair_buckets[i];
            ContactSlotV* vacant_slots = vacant_slot_buckets[i];
            unsigned vacancy_count = bucket_vacancy_count[i];

            // Replace any unset indices with the first one, which is always valid.
            // This is safe because the slots will just overwrite each other.
            for (unsigned i = 0; i < vacancy_count; ++i) {
                simdv_int32 ab = simd_int32::loadv((int32_t*)vacant_pairs[i].ab);
                simdv_int32 indices = simd_int32::loadv((const int32_t*)vacant_slots[i].indices);

                simdv_int32 mask = simd_int32::cmp_eq(ab, invalid_index);
                simdv_int32 first_index = simd128::shuffle32<0, 0, 0, 0>(indices);

#if NUDGE_SIMDV_WIDTH == 256
                first_index = simd256::shuffle128<0,0>(first_index);
#endif

                indices = simd::blendv32(indices, first_index, mask);

                simd_int32::storev((int32_t*)contact_slots[contact_slot_count++].indices, indices);
            }
        }
    }
    commit_array<ContactSlotV>(memory, contact_slot_count);

    ContactConstraintV* constraints = allocate_array<ContactConstraintV>(memory, contact_slot_count, 32);
    ContactConstraintStateV* constraint_states = allocate_array<ContactConstraintStateV>(memory, contact_slot_count, 32);

    data->constraints = constraints;
    data->constraint_states = constraint_states;

    memset(constraint_states, 0, sizeof(ContactConstraintStateV)*contact_slot_count);

    for (unsigned i = 0; i < contact_slot_count; ++i) {
        ContactSlotV slot = contact_slots[i];

        for (unsigned j = 0; j < simdv_width32; ++j)
            constraint_to_contact[i*simdv_width32 + j] = slot.indices[j];

        simdv_float position_x, position_y, position_z, penetration;
        simdv_float normal_x, normal_y, normal_z, friction;
        load8<sizeof(contacts.data[0]), 1>((const float*)contacts.data, slot.indices,
                                           position_x, position_y, position_z, penetration,
                                           normal_x, normal_y, normal_z, friction);


        NUDGE_SIMDV_ALIGNED uint16_t ab_array[simdv_width32*2];

        for (unsigned j = 0; j < simdv_width32; ++j) {
            BodyPair pair = contacts.bodies[slot.indices[j]];
            ab_array[j*2 + 0] = pair.a;
            ab_array[j*2 + 1] = pair.b;
        }

        unsigned a0 = ab_array[0]; unsigned a1 = ab_array[2]; unsigned a2 = ab_array[4]; unsigned a3 = ab_array[6];
        unsigned b0 = ab_array[1]; unsigned b1 = ab_array[3]; unsigned b2 = ab_array[5]; unsigned b3 = ab_array[7];

#if NUDGE_SIMDV_WIDTH == 256
        unsigned a4 = ab_array[8]; unsigned a5 = ab_array[10]; unsigned a6 = ab_array[12]; unsigned a7 = ab_array[14];
        unsigned b4 = ab_array[9]; unsigned b5 = ab_array[11]; unsigned b6 = ab_array[13]; unsigned b7 = ab_array[15];

        simdv_float a_mass_inverse = simd_float::make8(bodies.momentum[a0].unused0, bodies.momentum[a1].unused0, bodies.momentum[a2].unused0, bodies.momentum[a3].unused0,
                                                     bodies.momentum[a4].unused0, bodies.momentum[a5].unused0, bodies.momentum[a6].unused0, bodies.momentum[a7].unused0);
        simdv_float b_mass_inverse = simd_float::make8(bodies.momentum[b0].unused0, bodies.momentum[b1].unused0, bodies.momentum[b2].unused0, bodies.momentum[b3].unused0,
                                                     bodies.momentum[b4].unused0, bodies.momentum[b5].unused0, bodies.momentum[b6].unused0, bodies.momentum[b7].unused0);
#else
        simdv_float a_mass_inverse = simd_float::make4(bodies.momentum[a0].unused0, bodies.momentum[a1].unused0, bodies.momentum[a2].unused0, bodies.momentum[a3].unused0);
        simdv_float b_mass_inverse = simd_float::make4(bodies.momentum[b0].unused0, bodies.momentum[b1].unused0, bodies.momentum[b2].unused0, bodies.momentum[b3].unused0);
#endif

        simdv_float a_position_x, a_position_y, a_position_z, a_position_w;
        simdv_float b_position_x, b_position_y, b_position_z, b_position_w;
        load4<sizeof(bodies.transforms[0]), 2>(bodies.transforms[0].position, ab_array,
                                               a_position_x, a_position_y, a_position_z, a_position_w);
        load4<sizeof(bodies.transforms[0]), 2>(bodies.transforms[0].position, ab_array + 1,
                                               b_position_x, b_position_y, b_position_z, b_position_w);

        simdv_float pa_x = position_x - a_position_x;
        simdv_float pa_y = position_y - a_position_y;
        simdv_float pa_z = position_z - a_position_z;

        simdv_float pb_x = position_x - b_position_x;
        simdv_float pb_y = position_y - b_position_y;
        simdv_float pb_z = position_z - b_position_z;

        simdv_float a_momentum_to_velocity_xx, a_momentum_to_velocity_yy, a_momentum_to_velocity_zz, a_momentum_to_velocity_u0;
        simdv_float a_momentum_to_velocity_xy, a_momentum_to_velocity_xz, a_momentum_to_velocity_yz, a_momentum_to_velocity_u1;
        load8<sizeof(momentum_to_velocity[0]), 2>((const float*)momentum_to_velocity, ab_array,
                                                  a_momentum_to_velocity_xx, a_momentum_to_velocity_yy, a_momentum_to_velocity_zz, a_momentum_to_velocity_u0,
                                                  a_momentum_to_velocity_xy, a_momentum_to_velocity_xz, a_momentum_to_velocity_yz, a_momentum_to_velocity_u1);

        simdv_float na_xt, na_yt, na_zt;
        simd_soa::cross(pa_x, pa_y, pa_z, normal_x, normal_y, normal_z, na_xt, na_yt, na_zt);

        simdv_float na_x = a_momentum_to_velocity_xx*na_xt + a_momentum_to_velocity_xy*na_yt + a_momentum_to_velocity_xz*na_zt;
        simdv_float na_y = a_momentum_to_velocity_xy*na_xt + a_momentum_to_velocity_yy*na_yt + a_momentum_to_velocity_yz*na_zt;
        simdv_float na_z = a_momentum_to_velocity_xz*na_xt + a_momentum_to_velocity_yz*na_yt + a_momentum_to_velocity_zz*na_zt;

        simdv_float b_momentum_to_velocity_xx, b_momentum_to_velocity_yy, b_momentum_to_velocity_zz, b_momentum_to_velocity_u0;
        simdv_float b_momentum_to_velocity_xy, b_momentum_to_velocity_xz, b_momentum_to_velocity_yz, b_momentum_to_velocity_u1;
        load8<sizeof(momentum_to_velocity[0]), 2>((const float*)momentum_to_velocity, ab_array + 1,
                                                  b_momentum_to_velocity_xx, b_momentum_to_velocity_yy, b_momentum_to_velocity_zz, b_momentum_to_velocity_u0,
                                                  b_momentum_to_velocity_xy, b_momentum_to_velocity_xz, b_momentum_to_velocity_yz, b_momentum_to_velocity_u1);

        simdv_float nb_xt, nb_yt, nb_zt;
        simd_soa::cross(pb_x, pb_y, pb_z, normal_x, normal_y, normal_z, nb_xt, nb_yt, nb_zt);

        simdv_float nb_x = b_momentum_to_velocity_xx*nb_xt + b_momentum_to_velocity_xy*nb_yt + b_momentum_to_velocity_xz*nb_zt;
        simdv_float nb_y = b_momentum_to_velocity_xy*nb_xt + b_momentum_to_velocity_yy*nb_yt + b_momentum_to_velocity_yz*nb_zt;
        simdv_float nb_z = b_momentum_to_velocity_xz*nb_xt + b_momentum_to_velocity_yz*nb_yt + b_momentum_to_velocity_zz*nb_zt;

        simd_soa::cross(na_x, na_y, na_z, pa_x, pa_y, pa_z, na_xt, na_yt, na_zt);
        simd_soa::cross(nb_x, nb_y, nb_z, pb_x, pb_y, pb_z, nb_xt, nb_yt, nb_zt);

        simdv_float normal_impulse_to_rotational_velocity_x = na_xt + nb_xt;
        simdv_float normal_impulse_to_rotational_velocity_y = na_yt + nb_yt;
        simdv_float normal_impulse_to_rotational_velocity_z = na_zt + nb_zt;

        simdv_float r_dot_n = normal_impulse_to_rotational_velocity_x*normal_x + normal_impulse_to_rotational_velocity_y*normal_y + normal_impulse_to_rotational_velocity_z*normal_z;

        simdv_float mass_inverse = a_mass_inverse + b_mass_inverse;
        simdv_float normal_velocity_to_normal_impulse = mass_inverse + r_dot_n;

        simdv_float nonzero = simd_float::cmp_neq(normal_velocity_to_normal_impulse, simd_float::zerov());
        normal_velocity_to_normal_impulse = simd::bitwise_and(simd_float::makev(-1.0f) / normal_velocity_to_normal_impulse, nonzero);

        simdv_float bias = simd_float::makev(-bias_factor) * simd_float::max(penetration - simd_float::makev(allowed_penetration), simd_float::zerov()) * normal_velocity_to_normal_impulse;

        // Compute a tangent from the normal. Care is taken to compute a smoothly varying basis to improve stability.
        simdv_float s = simd_float::abs(normal_x);

        simdv_float u_x = normal_z*s;
        simdv_float u_y = u_x - normal_z;
        simdv_float u_z = simd_float::madd(normal_x - normal_y, s, normal_y);

        u_x = simd::bitwise_xor(u_x, simd_float::makev(-0.0f));
        simd_soa::normalize(u_x, u_y, u_z);

        // Compute the rest of the basis.
        simdv_float v_x, v_y, v_z;
        simd_soa::cross(u_x, u_y, u_z, normal_x, normal_y, normal_z, v_x, v_y, v_z);

        simdv_float ua_x, ua_y, ua_z, va_x, va_y, va_z;
        simd_soa::cross(pa_x, pa_y, pa_z, u_x, u_y, u_z, ua_x, ua_y, ua_z);
        simd_soa::cross(pa_x, pa_y, pa_z, v_x, v_y, v_z, va_x, va_y, va_z);

        simdv_float ub_x, ub_y, ub_z, vb_x, vb_y, vb_z;
        simd_soa::cross(pb_x, pb_y, pb_z, u_x, u_y, u_z, ub_x, ub_y, ub_z);
        simd_soa::cross(pb_x, pb_y, pb_z, v_x, v_y, v_z, vb_x, vb_y, vb_z);

        simdv_float a_duu = a_momentum_to_velocity_xx*ua_x*ua_x + a_momentum_to_velocity_yy*ua_y*ua_y + a_momentum_to_velocity_zz*ua_z*ua_z;
        simdv_float a_dvv = a_momentum_to_velocity_xx*va_x*va_x + a_momentum_to_velocity_yy*va_y*va_y + a_momentum_to_velocity_zz*va_z*va_z;
        simdv_float a_duv = a_momentum_to_velocity_xx*ua_x*va_x + a_momentum_to_velocity_yy*ua_y*va_y + a_momentum_to_velocity_zz*ua_z*va_z;

        simdv_float a_suu = a_momentum_to_velocity_xy*ua_x*ua_y + a_momentum_to_velocity_xz*ua_x*ua_z + a_momentum_to_velocity_yz*ua_y*ua_z;
        simdv_float a_svv = a_momentum_to_velocity_xy*va_x*va_y + a_momentum_to_velocity_xz*va_x*va_z + a_momentum_to_velocity_yz*va_y*va_z;
        simdv_float a_suv = a_momentum_to_velocity_xy*(ua_x*va_y + ua_y*va_x) + a_momentum_to_velocity_xz*(ua_x*va_z + ua_z*va_x) + a_momentum_to_velocity_yz*(ua_y*va_z + ua_z*va_y);

        simdv_float b_duu = b_momentum_to_velocity_xx*ub_x*ub_x + b_momentum_to_velocity_yy*ub_y*ub_y + b_momentum_to_velocity_zz*ub_z*ub_z;
        simdv_float b_dvv = b_momentum_to_velocity_xx*vb_x*vb_x + b_momentum_to_velocity_yy*vb_y*vb_y + b_momentum_to_velocity_zz*vb_z*vb_z;
        simdv_float b_duv = b_momentum_to_velocity_xx*ub_x*vb_x + b_momentum_to_velocity_yy*ub_y*vb_y + b_momentum_to_velocity_zz*ub_z*vb_z;

        simdv_float b_suu = b_momentum_to_velocity_xy*ub_x*ub_y + b_momentum_to_velocity_xz*ub_x*ub_z + b_momentum_to_velocity_yz*ub_y*ub_z;
        simdv_float b_svv = b_momentum_to_velocity_xy*vb_x*vb_y + b_momentum_to_velocity_xz*vb_x*vb_z + b_momentum_to_velocity_yz*vb_y*vb_z;
        simdv_float b_suv = b_momentum_to_velocity_xy*(ub_x*vb_y + ub_y*vb_x) + b_momentum_to_velocity_xz*(ub_x*vb_z + ub_z*vb_x) + b_momentum_to_velocity_yz*(ub_y*vb_z + ub_z*vb_y);

        simdv_float friction_x = mass_inverse + a_duu + a_suu + a_suu + b_duu + b_suu + b_suu;
        simdv_float friction_y = mass_inverse + a_dvv + a_svv + a_svv + b_dvv + b_svv + b_svv;
        simdv_float friction_z = a_duv + a_duv + a_suv + a_suv + b_duv + b_duv + b_suv + b_suv;

        simdv_float ua_xt = a_momentum_to_velocity_xx*ua_x + a_momentum_to_velocity_xy*ua_y + a_momentum_to_velocity_xz*ua_z;
        simdv_float ua_yt = a_momentum_to_velocity_xy*ua_x + a_momentum_to_velocity_yy*ua_y + a_momentum_to_velocity_yz*ua_z;
        simdv_float ua_zt = a_momentum_to_velocity_xz*ua_x + a_momentum_to_velocity_yz*ua_y + a_momentum_to_velocity_zz*ua_z;

        simdv_float va_xt = a_momentum_to_velocity_xx*va_x + a_momentum_to_velocity_xy*va_y + a_momentum_to_velocity_xz*va_z;
        simdv_float va_yt = a_momentum_to_velocity_xy*va_x + a_momentum_to_velocity_yy*va_y + a_momentum_to_velocity_yz*va_z;
        simdv_float va_zt = a_momentum_to_velocity_xz*va_x + a_momentum_to_velocity_yz*va_y + a_momentum_to_velocity_zz*va_z;

        simdv_float ub_xt = b_momentum_to_velocity_xx*ub_x + b_momentum_to_velocity_xy*ub_y + b_momentum_to_velocity_xz*ub_z;
        simdv_float ub_yt = b_momentum_to_velocity_xy*ub_x + b_momentum_to_velocity_yy*ub_y + b_momentum_to_velocity_yz*ub_z;
        simdv_float ub_zt = b_momentum_to_velocity_xz*ub_x + b_momentum_to_velocity_yz*ub_y + b_momentum_to_velocity_zz*ub_z;

        simdv_float vb_xt = b_momentum_to_velocity_xx*vb_x + b_momentum_to_velocity_xy*vb_y + b_momentum_to_velocity_xz*vb_z;
        simdv_float vb_yt = b_momentum_to_velocity_xy*vb_x + b_momentum_to_velocity_yy*vb_y + b_momentum_to_velocity_yz*vb_z;
        simdv_float vb_zt = b_momentum_to_velocity_xz*vb_x + b_momentum_to_velocity_yz*vb_y + b_momentum_to_velocity_zz*vb_z;

        constraints[i].a[0] = a0; constraints[i].a[1] = a1; constraints[i].a[2] = a2; constraints[i].a[3] = a3;
        constraints[i].b[0] = b0; constraints[i].b[1] = b1; constraints[i].b[2] = b2; constraints[i].b[3] = b3;

#if NUDGE_SIMDV_WIDTH == 256
        constraints[i].a[4] = a4; constraints[i].a[5] = a5; constraints[i].a[6] = a6; constraints[i].a[7] = a7;
        constraints[i].b[4] = b4; constraints[i].b[5] = b5; constraints[i].b[6] = b6; constraints[i].b[7] = b7;
#endif

        simd_float::storev(constraints[i].n_x, normal_x);
        simd_float::storev(constraints[i].n_y, normal_y);
        simd_float::storev(constraints[i].n_z, normal_z);

        simd_float::storev(constraints[i].pa_x, pa_x);
        simd_float::storev(constraints[i].pa_y, pa_y);
        simd_float::storev(constraints[i].pa_z, pa_z);

        simd_float::storev(constraints[i].pb_x, pb_x);
        simd_float::storev(constraints[i].pb_y, pb_y);
        simd_float::storev(constraints[i].pb_z, pb_z);

        simd_float::storev(constraints[i].normal_velocity_to_normal_impulse, normal_velocity_to_normal_impulse);

        simd_float::storev(constraints[i].bias, bias);
        simd_float::storev(constraints[i].friction, friction);

        simd_float::storev(constraints[i].u_x, u_x);
        simd_float::storev(constraints[i].u_y, u_y);
        simd_float::storev(constraints[i].u_z, u_z);

        simd_float::storev(constraints[i].v_x, v_x);
        simd_float::storev(constraints[i].v_y, v_y);
        simd_float::storev(constraints[i].v_z, v_z);

        simd_float::storev(constraints[i].friction_coefficient_x, friction_x);
        simd_float::storev(constraints[i].friction_coefficient_y, friction_y);
        simd_float::storev(constraints[i].friction_coefficient_z, friction_z);

        simd_float::storev(constraints[i].ua_x, -ua_xt);
        simd_float::storev(constraints[i].ua_y, -ua_yt);
        simd_float::storev(constraints[i].ua_z, -ua_zt);

        simd_float::storev(constraints[i].va_x, -va_xt);
        simd_float::storev(constraints[i].va_y, -va_yt);
        simd_float::storev(constraints[i].va_z, -va_zt);

        simd_float::storev(constraints[i].na_x, -na_x);
        simd_float::storev(constraints[i].na_y, -na_y);
        simd_float::storev(constraints[i].na_z, -na_z);

        simd_float::storev(constraints[i].ub_x, ub_xt);
        simd_float::storev(constraints[i].ub_y, ub_yt);
        simd_float::storev(constraints[i].ub_z, ub_zt);

        simd_float::storev(constraints[i].vb_x, vb_xt);
        simd_float::storev(constraints[i].vb_y, vb_yt);
        simd_float::storev(constraints[i].vb_z, vb_zt);

        simd_float::storev(constraints[i].nb_x, nb_x);
        simd_float::storev(constraints[i].nb_y, nb_y);
        simd_float::storev(constraints[i].nb_z, nb_z);

        simdv_float cached_impulse_x, cached_impulse_y, cached_impulse_z, unused0;
        load4<sizeof(impulses[0]), 1>((const float*)impulses, slot.indices,
                                      cached_impulse_x, cached_impulse_y, cached_impulse_z, unused0);

        simdv_float a_velocity_x, a_velocity_y, a_velocity_z;
        simdv_float a_angular_velocity_x, a_angular_velocity_y, a_angular_velocity_z, a_angular_velocity_w;
        load8<sizeof(bodies.momentum[0]), 1>((const float*)bodies.momentum, constraints[i].a,
                                             a_velocity_x, a_velocity_y, a_velocity_z, a_mass_inverse,
                                             a_angular_velocity_x, a_angular_velocity_y, a_angular_velocity_z, a_angular_velocity_w);

        simdv_float b_velocity_x, b_velocity_y, b_velocity_z;
        simdv_float b_angular_velocity_x, b_angular_velocity_y, b_angular_velocity_z, b_angular_velocity_w;
        load8<sizeof(bodies.momentum[0]), 1>((const float*)bodies.momentum, constraints[i].b,
                                             b_velocity_x, b_velocity_y, b_velocity_z, b_mass_inverse,
                                             b_angular_velocity_x, b_angular_velocity_y, b_angular_velocity_z, b_angular_velocity_w);

        simdv_float normal_impulse = simd_float::max(normal_x*cached_impulse_x + normal_y*cached_impulse_y + normal_z*cached_impulse_z, simd_float::zerov());
        simdv_float max_friction_impulse = normal_impulse * friction;

        simdv_float friction_impulse_x = u_x*cached_impulse_x + u_y*cached_impulse_y + u_z*cached_impulse_z;
        simdv_float friction_impulse_y = v_x*cached_impulse_x + v_y*cached_impulse_y + v_z*cached_impulse_z;

        simdv_float friction_clamp_scale = friction_impulse_x*friction_impulse_x + friction_impulse_y*friction_impulse_y;

        friction_clamp_scale = simd_float::rsqrt(friction_clamp_scale);
        friction_clamp_scale = friction_clamp_scale * max_friction_impulse;
        friction_clamp_scale = simd_float::min(simd_float::makev(1.0f), friction_clamp_scale); // Note: First operand is returned on NaN.

        friction_impulse_x = friction_impulse_x * friction_clamp_scale;
        friction_impulse_y = friction_impulse_y * friction_clamp_scale;

        simdv_float linear_impulse_x = friction_impulse_x*u_x + friction_impulse_y*v_x + normal_x * normal_impulse;
        simdv_float linear_impulse_y = friction_impulse_x*u_y + friction_impulse_y*v_y + normal_y * normal_impulse;
        simdv_float linear_impulse_z = friction_impulse_x*u_z + friction_impulse_y*v_z + normal_z * normal_impulse;

        simdv_float a_angular_impulse_x = friction_impulse_x*simd_float::loadv(constraints[i].ua_x) + friction_impulse_y*simd_float::loadv(constraints[i].va_x) + normal_impulse*simd_float::loadv(constraints[i].na_x);
        simdv_float a_angular_impulse_y = friction_impulse_x*simd_float::loadv(constraints[i].ua_y) + friction_impulse_y*simd_float::loadv(constraints[i].va_y) + normal_impulse*simd_float::loadv(constraints[i].na_y);
        simdv_float a_angular_impulse_z = friction_impulse_x*simd_float::loadv(constraints[i].ua_z) + friction_impulse_y*simd_float::loadv(constraints[i].va_z) + normal_impulse*simd_float::loadv(constraints[i].na_z);

        simdv_float b_angular_impulse_x = friction_impulse_x*simd_float::loadv(constraints[i].ub_x) + friction_impulse_y*simd_float::loadv(constraints[i].vb_x) + normal_impulse*simd_float::loadv(constraints[i].nb_x);
        simdv_float b_angular_impulse_y = friction_impulse_x*simd_float::loadv(constraints[i].ub_y) + friction_impulse_y*simd_float::loadv(constraints[i].vb_y) + normal_impulse*simd_float::loadv(constraints[i].nb_y);
        simdv_float b_angular_impulse_z = friction_impulse_x*simd_float::loadv(constraints[i].ub_z) + friction_impulse_y*simd_float::loadv(constraints[i].vb_z) + normal_impulse*simd_float::loadv(constraints[i].nb_z);

        a_velocity_x -= linear_impulse_x * a_mass_inverse;
        a_velocity_y -= linear_impulse_y * a_mass_inverse;
        a_velocity_z -= linear_impulse_z * a_mass_inverse;

        a_angular_velocity_x += a_angular_impulse_x;
        a_angular_velocity_y += a_angular_impulse_y;
        a_angular_velocity_z += a_angular_impulse_z;

        b_velocity_x += linear_impulse_x * b_mass_inverse;
        b_velocity_y += linear_impulse_y * b_mass_inverse;
        b_velocity_z += linear_impulse_z * b_mass_inverse;

        b_angular_velocity_x += b_angular_impulse_x;
        b_angular_velocity_y += b_angular_impulse_y;
        b_angular_velocity_z += b_angular_impulse_z;

        simd_float::storev(constraint_states[i].applied_normal_impulse, normal_impulse);
        simd_float::storev(constraint_states[i].applied_friction_impulse_x, friction_impulse_x);
        simd_float::storev(constraint_states[i].applied_friction_impulse_y, friction_impulse_y);

        store8<sizeof(bodies.momentum[0]), 1>((float*)bodies.momentum, constraints[i].a,
                                              a_velocity_x, a_velocity_y, a_velocity_z, a_mass_inverse,
                                              a_angular_velocity_x, a_angular_velocity_y, a_angular_velocity_z, a_angular_velocity_w);

        store8<sizeof(bodies.momentum[0]), 1>((float*)bodies.momentum, constraints[i].b,
                                              b_velocity_x, b_velocity_y, b_velocity_z, b_mass_inverse,
                                              b_angular_velocity_x, b_angular_velocity_y, b_angular_velocity_z, b_angular_velocity_w);
    }

    data->constraint_batches = contact_slot_count;

    return data;
}

uintptr_t get_required_arena_size_for_setup_contact_constraints(context_t* c) {
return
    sizeof(ContactConstraintData)+63+
    sizeof(InertiaTransform)*c->bodies.count+31+
    sizeof(uint32_t)*c->contact_data.count*simdv_width32+31+
    sizeof(ContactSlotV)*c->contact_data.count+
    NUDGE_SETUP_CONTACT_CONSTRAINTS_BUCKET_COUNT*((2*c->contact_data.count+1)+31)*sizeof(ContactPairV);
}

void apply_impulses(ContactConstraintData* data, BodyData bodies) {
    ContactConstraintV* constraints = data->constraints;
    ContactConstraintStateV* constraint_states = data->constraint_states;

    unsigned constraint_batches = data->constraint_batches;

    for (unsigned i = 0; i < constraint_batches; ++i) {
        const ContactConstraintV& constraint = constraints[i];

        simdv_float a_velocity_x, a_velocity_y, a_velocity_z, a_mass_inverse;
        simdv_float a_angular_velocity_x, a_angular_velocity_y, a_angular_velocity_z, a_angular_velocity_w;
        load8<sizeof(bodies.momentum[0]), 1>((const float*)bodies.momentum, constraint.a,
                                             a_velocity_x, a_velocity_y, a_velocity_z, a_mass_inverse,
                                             a_angular_velocity_x, a_angular_velocity_y, a_angular_velocity_z, a_angular_velocity_w);

        simdv_float pa_z = simd_float::loadv(constraint.pa_z);
        simdv_float pa_x = simd_float::loadv(constraint.pa_x);
        simdv_float pa_y = simd_float::loadv(constraint.pa_y);

        simdv_float v_xa = simd_float::madd(a_angular_velocity_y, pa_z, a_velocity_x);
        simdv_float v_ya = simd_float::madd(a_angular_velocity_z, pa_x, a_velocity_y);
        simdv_float v_za = simd_float::madd(a_angular_velocity_x, pa_y, a_velocity_z);

        simdv_float b_velocity_x, b_velocity_y, b_velocity_z, b_mass_inverse;
        simdv_float b_angular_velocity_x, b_angular_velocity_y, b_angular_velocity_z, b_angular_velocity_w;
        load8<sizeof(bodies.momentum[0]), 1>((const float*)bodies.momentum, constraint.b,
                                             b_velocity_x, b_velocity_y, b_velocity_z, b_mass_inverse,
                                             b_angular_velocity_x, b_angular_velocity_y, b_angular_velocity_z, b_angular_velocity_w);

        simdv_float pb_z = simd_float::loadv(constraint.pb_z);
        simdv_float pb_x = simd_float::loadv(constraint.pb_x);
        simdv_float pb_y = simd_float::loadv(constraint.pb_y);

        simdv_float v_xb = simd_float::madd(b_angular_velocity_y, pb_z, b_velocity_x);
        simdv_float v_yb = simd_float::madd(b_angular_velocity_z, pb_x, b_velocity_y);
        simdv_float v_zb = simd_float::madd(b_angular_velocity_x, pb_y, b_velocity_z);

        v_xa = simd_float::madd(b_angular_velocity_z, pb_y, v_xa);
        v_ya = simd_float::madd(b_angular_velocity_x, pb_z, v_ya);
        v_za = simd_float::madd(b_angular_velocity_y, pb_x, v_za);

        simdv_float n_x = simd_float::loadv(constraint.n_x);
        simdv_float fu_x = simd_float::loadv(constraint.u_x);
        simdv_float fv_x = simd_float::loadv(constraint.v_x);

        v_xb = simd_float::madd(a_angular_velocity_z, pa_y, v_xb);
        v_yb = simd_float::madd(a_angular_velocity_x, pa_z, v_yb);
        v_zb = simd_float::madd(a_angular_velocity_y, pa_x, v_zb);

        simdv_float n_y = simd_float::loadv(constraint.n_y);
        simdv_float fu_y = simd_float::loadv(constraint.u_y);
        simdv_float fv_y = simd_float::loadv(constraint.v_y);

        simdv_float v_x = v_xb - v_xa;
        simdv_float v_y = v_yb - v_ya;
        simdv_float v_z = v_zb - v_za;

        simdv_float t_z = n_x * v_x;
        simdv_float t_x = v_x * fu_x;
        simdv_float t_y = v_x * fv_x;

        simdv_float n_z = simd_float::loadv(constraint.n_z);
        simdv_float fu_z = simd_float::loadv(constraint.u_z);
        simdv_float fv_z = simd_float::loadv(constraint.v_z);

        simdv_float normal_bias = simd_float::loadv(constraint.bias);
        simdv_float old_normal_impulse = simd_float::loadv(constraint_states[i].applied_normal_impulse);
        simdv_float normal_factor = simd_float::loadv(constraint.normal_velocity_to_normal_impulse);

        t_z = simd_float::madd(n_y, v_y, t_z);
        t_x = simd_float::madd(v_y, fu_y, t_x);
        t_y = simd_float::madd(v_y, fv_y, t_y);

        normal_bias = normal_bias + old_normal_impulse;

        t_z = simd_float::madd(n_z, v_z, t_z);
        t_x = simd_float::madd(v_z, fu_z, t_x);
        t_y = simd_float::madd(v_z, fv_z, t_y);

        simdv_float normal_impulse = simd_float::madd(normal_factor, t_z, normal_bias);

        simdv_float t_xx = t_x*t_x;
        simdv_float t_yy = t_y*t_y;
        simdv_float t_xy = t_x*t_y;
        simdv_float tl2 = t_xx + t_yy;

        normal_impulse = simd_float::max(normal_impulse, simd_float::zerov());

        t_x *= tl2;
        t_y *= tl2;

        simd_float::storev(constraint_states[i].applied_normal_impulse, normal_impulse);

        simdv_float max_friction_impulse = normal_impulse * simd_float::loadv(constraint.friction);
        normal_impulse = normal_impulse - old_normal_impulse;

        simdv_float friction_x = simd_float::loadv(constraint.friction_coefficient_x);
        simdv_float friction_factor = t_xx * friction_x;
        simdv_float linear_impulse_x = n_x * normal_impulse;

        simdv_float friction_y = simd_float::loadv(constraint.friction_coefficient_y);
        friction_factor = simd_float::madd(t_yy, friction_y, friction_factor);
        simdv_float linear_impulse_y = n_y * normal_impulse;

        simdv_float friction_z = simd_float::loadv(constraint.friction_coefficient_z);
        friction_factor = simd_float::madd(t_xy, friction_z, friction_factor);
        simdv_float linear_impulse_z = n_z * normal_impulse;

        friction_factor = simd_float::recip(friction_factor);

        simdv_float na_x = simd_float::loadv(constraint.na_x);
        simdv_float na_y = simd_float::loadv(constraint.na_y);
        simdv_float na_z = simd_float::loadv(constraint.na_z);

        a_angular_velocity_x = simd_float::madd(na_x, normal_impulse, a_angular_velocity_x);
        a_angular_velocity_y = simd_float::madd(na_y, normal_impulse, a_angular_velocity_y);
        a_angular_velocity_z = simd_float::madd(na_z, normal_impulse, a_angular_velocity_z);

        simdv_float old_friction_impulse_x = simd_float::loadv(constraint_states[i].applied_friction_impulse_x);
        simdv_float old_friction_impulse_y = simd_float::loadv(constraint_states[i].applied_friction_impulse_y);

        friction_factor = simd_float::min(simd_float::makev(1e+6f), friction_factor); // Note: First operand is returned on NaN.

        simdv_float friction_impulse_x = t_x*friction_factor;
        simdv_float friction_impulse_y = t_y*friction_factor;

        friction_impulse_x = old_friction_impulse_x - friction_impulse_x; // Note: Friction impulse has the wrong sign until this point. This is really an addition.
        friction_impulse_y = old_friction_impulse_y - friction_impulse_y;

        simdv_float friction_clamp_scale = friction_impulse_x*friction_impulse_x + friction_impulse_y*friction_impulse_y;

        simdv_float nb_x = simd_float::loadv(constraint.nb_x);
        simdv_float nb_y = simd_float::loadv(constraint.nb_y);
        simdv_float nb_z = simd_float::loadv(constraint.nb_z);

        friction_clamp_scale = simd_float::rsqrt(friction_clamp_scale);

        b_angular_velocity_x = simd_float::madd(nb_x, normal_impulse, b_angular_velocity_x);
        b_angular_velocity_y = simd_float::madd(nb_y, normal_impulse, b_angular_velocity_y);
        b_angular_velocity_z = simd_float::madd(nb_z, normal_impulse, b_angular_velocity_z);

        friction_clamp_scale = friction_clamp_scale * max_friction_impulse;
        friction_clamp_scale = simd_float::min(simd_float::makev(1.0f), friction_clamp_scale); // Note: First operand is returned on NaN.

        friction_impulse_x = friction_impulse_x * friction_clamp_scale;
        friction_impulse_y = friction_impulse_y * friction_clamp_scale;

        simd_float::storev(constraint_states[i].applied_friction_impulse_x, friction_impulse_x);
        simd_float::storev(constraint_states[i].applied_friction_impulse_y, friction_impulse_y);

        friction_impulse_x -= old_friction_impulse_x;
        friction_impulse_y -= old_friction_impulse_y;

        linear_impulse_x = simd_float::madd(fu_x, friction_impulse_x, linear_impulse_x);
        linear_impulse_y = simd_float::madd(fu_y, friction_impulse_x, linear_impulse_y);
        linear_impulse_z = simd_float::madd(fu_z, friction_impulse_x, linear_impulse_z);

        linear_impulse_x = simd_float::madd(fv_x, friction_impulse_y, linear_impulse_x);
        linear_impulse_y = simd_float::madd(fv_y, friction_impulse_y, linear_impulse_y);
        linear_impulse_z = simd_float::madd(fv_z, friction_impulse_y, linear_impulse_z);

        simdv_float a_mass_inverse_neg = simd::bitwise_xor(a_mass_inverse, simd_float::makev(-0.0f));

        a_velocity_x = simd_float::madd(linear_impulse_x, a_mass_inverse_neg, a_velocity_x);
        a_velocity_y = simd_float::madd(linear_impulse_y, a_mass_inverse_neg, a_velocity_y);
        a_velocity_z = simd_float::madd(linear_impulse_z, a_mass_inverse_neg, a_velocity_z);

        simdv_float ua_x = simd_float::loadv(constraint.ua_x);
        simdv_float ua_y = simd_float::loadv(constraint.ua_y);
        simdv_float ua_z = simd_float::loadv(constraint.ua_z);

        a_angular_velocity_x = simd_float::madd(ua_x, friction_impulse_x, a_angular_velocity_x);
        a_angular_velocity_y = simd_float::madd(ua_y, friction_impulse_x, a_angular_velocity_y);
        a_angular_velocity_z = simd_float::madd(ua_z, friction_impulse_x, a_angular_velocity_z);

        simdv_float va_x = simd_float::loadv(constraint.va_x);
        simdv_float va_y = simd_float::loadv(constraint.va_y);
        simdv_float va_z = simd_float::loadv(constraint.va_z);

        a_angular_velocity_x = simd_float::madd(va_x, friction_impulse_y, a_angular_velocity_x);
        a_angular_velocity_y = simd_float::madd(va_y, friction_impulse_y, a_angular_velocity_y);
        a_angular_velocity_z = simd_float::madd(va_z, friction_impulse_y, a_angular_velocity_z);

        a_angular_velocity_w = simd_float::zerov(); // Reduces register pressure.

        store8<sizeof(bodies.momentum[0]), 1>((float*)bodies.momentum, constraint.a,
                                              a_velocity_x, a_velocity_y, a_velocity_z, a_mass_inverse,
                                              a_angular_velocity_x, a_angular_velocity_y, a_angular_velocity_z, a_angular_velocity_w);

        b_velocity_x = simd_float::madd(linear_impulse_x, b_mass_inverse, b_velocity_x);
        b_velocity_y = simd_float::madd(linear_impulse_y, b_mass_inverse, b_velocity_y);
        b_velocity_z = simd_float::madd(linear_impulse_z, b_mass_inverse, b_velocity_z);

        simdv_float ub_x = simd_float::loadv(constraint.ub_x);
        simdv_float ub_y = simd_float::loadv(constraint.ub_y);
        simdv_float ub_z = simd_float::loadv(constraint.ub_z);

        b_angular_velocity_x = simd_float::madd(ub_x, friction_impulse_x, b_angular_velocity_x);
        b_angular_velocity_y = simd_float::madd(ub_y, friction_impulse_x, b_angular_velocity_y);
        b_angular_velocity_z = simd_float::madd(ub_z, friction_impulse_x, b_angular_velocity_z);

        simdv_float vb_x = simd_float::loadv(constraint.vb_x);
        simdv_float vb_y = simd_float::loadv(constraint.vb_y);
        simdv_float vb_z = simd_float::loadv(constraint.vb_z);

        b_angular_velocity_x = simd_float::madd(vb_x, friction_impulse_y, b_angular_velocity_x);
        b_angular_velocity_y = simd_float::madd(vb_y, friction_impulse_y, b_angular_velocity_y);
        b_angular_velocity_z = simd_float::madd(vb_z, friction_impulse_y, b_angular_velocity_z);

        b_angular_velocity_w = simd_float::zerov(); // Reduces register pressure.

        store8<sizeof(bodies.momentum[0]), 1>((float*)bodies.momentum, constraint.b,
                                              b_velocity_x, b_velocity_y, b_velocity_z, b_mass_inverse,
                                              b_angular_velocity_x, b_angular_velocity_y, b_angular_velocity_z, b_angular_velocity_w);
    }
}

void update_cached_impulses(ContactConstraintData* data, ContactImpulseData* contact_impulses) {
    uint32_t* constraint_to_contact = data->constraint_to_contact;

    ContactConstraintV* constraints = data->constraints;
    ContactConstraintStateV* constraint_states = data->constraint_states;
    unsigned constraint_count = data->constraint_batches * simdv_width32;

    for (unsigned i = 0; i < constraint_count; ++i) {
        unsigned contact = constraint_to_contact[i];

        unsigned b = i >> simdv_width32_log2;
        unsigned l = i & (simdv_width32-1);

        float* impulse = contact_impulses->data[contact].impulse;

        impulse[0] = (constraint_states[b].applied_normal_impulse[l] * constraints[b].n_x[l] +
                      constraint_states[b].applied_friction_impulse_x[l] * constraints[b].u_x[l] +
                      constraint_states[b].applied_friction_impulse_y[l] * constraints[b].v_x[l]);

        impulse[1] = (constraint_states[b].applied_normal_impulse[l] * constraints[b].n_y[l] +
                      constraint_states[b].applied_friction_impulse_x[l] * constraints[b].u_y[l] +
                      constraint_states[b].applied_friction_impulse_y[l] * constraints[b].v_y[l]);

        impulse[2] = (constraint_states[b].applied_normal_impulse[l] * constraints[b].n_z[l] +
                      constraint_states[b].applied_friction_impulse_x[l] * constraints[b].u_z[l] +
                      constraint_states[b].applied_friction_impulse_y[l] * constraints[b].v_z[l]);
    }
}

void advance(context_t* c,float time_step) {
    ActiveBodies* active_bodies = &c->active_bodies;
    BodyData* bodies = &c->bodies;
    float half_time_step = 0.5f * time_step;
	const float sleeping_threshold_linear_velocity_squared = c->simulation_params.sleeping_threshold_linear_velocity_squared;
	const float sleeping_threshold_angular_velocity_squared = c->simulation_params.sleeping_threshold_angular_velocity_squared;
	

    // TODO: Consider SIMD-optimizing this loop.
    for (unsigned n = 0; n < active_bodies->count; ++n) {
        unsigned i = active_bodies->indices[n];

        BodyFilter* filter = &bodies->filters[i];
        if (filter->flags&BF_IS_DYNAMIC
             //&& !(filter->flags&BF_IS_SENSOR)  // Nope!
               )    {
            float3 velocity = make_float3(bodies->momentum[i].velocity);
            float3 angular_velocity = make_float3(bodies->momentum[i].angular_velocity);
            uint8_t* idle_counter = &c->bodies.idle_counters[i];
            if (length2(velocity) < sleeping_threshold_linear_velocity_squared && length2(angular_velocity) < sleeping_threshold_angular_velocity_squared) {
                if (*idle_counter < 0xff) {
                   ++(*idle_counter);
                   //*idle_counter=0xff; // nope
                    /*// New stuff I'm testing (no way!)
                    if (*idle_counter==0xff)  {
                        memset(&bodies->momentum[i].velocity,0,sizeof(float3));
                        memset(&bodies->momentum[i].angular_velocity,0,sizeof(float3));
                        velocity.x=velocity.y=velocity.z=0.f;
                        angular_velocity.x=angular_velocity.y=angular_velocity.z=0.f;
                    }*/
                    //if (*idle_counter==0xff) continue;   // attempt to try? NOPE: does nothing
                }
                //else continue; // attempt to try? NOPE: with this alone bodies keep waking up
            }
            else {
                *idle_counter = 0;
            }
            //if (filter->flags&BF_IS_SENSOR) continue;   // test to remove

            Rotation dr = { angular_velocity, 0.f }; // last value was missing (warning: missing initializer for member ‘{anonymous}::Rotation::s’ [-Wmissing-field-initializers])

            dr = dr * make_rotation(bodies->transforms[i].rotation);
            dr.v *= half_time_step;
            dr.s *= half_time_step;

            // 3 new lines (original code did not use these 3 substitutions)
            Transform* bodyTransform = &bodies->transforms[i];
            float* bodyPosition3 = bodyTransform->position;
            float* bodyRotation4 = bodyTransform->rotation;

            bodyPosition3[0] += velocity.x * time_step;
            bodyPosition3[1] += velocity.y * time_step;
            bodyPosition3[2] += velocity.z * time_step;

            bodyRotation4[0] += dr.v.x;
            bodyRotation4[1] += dr.v.y;
            bodyRotation4[2] += dr.v.z;
            bodyRotation4[3] += dr.s;

            Rotation rotation = normalize(make_rotation(bodyRotation4));

            bodyRotation4[0] = rotation.v.x;
            bodyRotation4[1] = rotation.v.y;
            bodyRotation4[2] = rotation.v.z;
            bodyRotation4[3] = rotation.s;
        }
    }
}

}   // namespace nudge

#endif //NUDGE_IMPLEMENTATION_GUARD
#endif //NUDGE_IMPLEMENTATION

