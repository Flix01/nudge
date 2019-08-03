### nudge simde folder

In this folder you can find:

- *"nudge.h/cpp"* This is the SIMDE version of the nudge library
- *"simde/"* This subfolder contains the SIMDE library (https://github.com/nemequ/simde) 

## Why all this stuff?

SIMD is a requirement of the original nudge repository.
With this version it can work (slower) without it (I use it mainly for emscripten builds).

## How can I test it?

In the example folder of this repository, **main_no_ffp_with_shadows.cpp** can be compiled with both the definitions: **-DUSE_SIMDE -DSIMDE_NO_NATIVE** (the include paths must point to this simde folder and its simde/simde subfolder).
A read-to-use emscripten command-line can be found in **main_no_ffp_with_shadows.cpp** itself.

## Notes

- If you need SIMD, do NOT use this version of nudge
- All other examples in this repository can't be compiled with the SIMDE version of nudge

