#pragma once

//Start & Stop Functions
extern "C" __declspec(dllexport) int start_haptic_loop(int input);
extern "C" __declspec(dllexport) void stop_haptic_loop();

/*** PEN PROPERTIES ***/
//Position
extern "C" __declspec(dllexport) void position_get(double* output);
extern "C" __declspec(dllexport) void position_reframed_get(double* frame_properties, bool zscaling, double surface_location, double pos_gain, double* output);

//Velocity
extern "C" __declspec(dllexport) void velocity_get(double* output);
extern "C" __declspec(dllexport) void velocity_reframed_get(const double* frame_properties, double* output);

//Force
extern "C" __declspec(dllexport) void forces_get(double* output);
extern "C" __declspec(dllexport) void forces_set(double* input);
extern "C" __declspec(dllexport) void drag_force_set(int* toggle, double* coeff);
extern "C" __declspec(dllexport) void force_extrema_set(double* maxforce, double* minforce);

//Angles
extern "C" __declspec(dllexport) void angles_get(double *output);

//Button State
extern "C" __declspec(dllexport) int8_t buttonstate_get();

/*** OTHER ***/
extern "C" __declspec(dllexport) void safety_trigger(bool input, bool* output);
extern "C" __declspec(dllexport) void frame_wall_set(bool input, double k_wall);
extern "C" __declspec(dllexport) void signal_input(double input_signal, double input_gain);
extern "C" __declspec(dllexport) void surface_force_set(double* input, double* output, int setting);
extern "C" __declspec(dllexport) int8_t planing_set(bool reset, bool toggle);

