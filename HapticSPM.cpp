// HapticSPM.cpp : Defines the exported functions for the DLL.
#include "pch.h" // use stdafx.h in Visual Studio 2017 and earlier
#include <utility>
#include <limits.h>
#include <ctime>
#include "HapticSPM.h"

#ifdef  _WIN64
#pragma warning (disable:4996)
#endif
#include <cstdio>
#include <cassert>
#if defined(WIN32)
# include <conio.h>
#else
# include "conio.h"
#endif

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>


#include <string>
#include <iostream>
#include <Windows.h>
#include <vector>

/***** BEGIN GLOBAL VARIABLES *****/



/*** HAPTIC FRAME PROPERTIES ***/
//Frame origin, the bottom left corner of the haptic workspace (in units of mm), the area in which the pen is bounded.
const double frame_center_haptic[2] = { 0 , 20 };
//The frame width and frame height of the haptic workspace (in units of mm).
const double frame_size_haptic[2] = { 120 , 120 };

int mode = 0;

hduVector3Dd position;
hduVector3Dd velocity;
hduVector3Dd force_labview;

bool wall_toggle = false;
double wall_stiffness = 0.2;

bool drag_toggle[3] = {false, false, false};
double k_drag[3] = { 0.001 , 0.001 , 0.001 };

double force_max[3] = { 3, 3, 3 };
double force_min[3] = { -3, -3, -3 };

double signal = 0;
double gain = 1;

double k[4] = { 1, 1, 1, 1 };

int surface_force = 0;

bool safetip = false;


/*** FEEDBACK MODE ***/
/* Note: "Feedback mode" corresponds to the mode in which the haptic pen controls the x - y position in nanonis but not the z position.
   The STM is left tunneling, the z position is read and converted to a force. "Raw" or "Feedbackless" mode is the mode in which all
   positions of the haptic pen (x, y, z) control the STM tip. The STM tunneling is turned off, and the tunneling current is converted
   to a force in the y direction. */
//The initial height of the plane from LabView before the planing.
double height_i = 20;
//The height of the plane based on the input data from nanonis (i.e. how the features are felt).
double h_nano = 20;

/*** CURRENT ***/
//Setpoint Current, set in nanonis, roughly determines the current (pA) at which the force is neutral.
double current_setpoint = 100;
//Maximum current is maximum allowed current (pA) before the safety mechanisms kick in
double current_max = 10000;
//Live input current
double current_current = 0;
double current_last = 0;
//The percent of the setpointcurrent which triggers Zthresh
double spc_percent = 0.50;
//Minimum current it takes to write a force
double mincurrent = 0;
//Safetip, true if the tip exhibits dangerous behavior

bool afmtoggle = false;



/*** FORCE PARAMETERS ***/
//Sets the drag coefficients.



//The force component in each direction.

//force_y limits.
double force_y_max = 2.485;
double force_y_min = 0.275;
//dragless force_y numbers
double force_y_nodrag_current;
double force_y_nodrag_last;
//determines force scaling

//Scaling factors
double k_a = 1;
double k_b = 1;
double k_c = 1;



/*** PLANING SEQUENCE ***/
//Number of clicks for planing, 3 is necessary before the plane becomes 'active'. 
//A click is triggered by experiencing a sufficient level of downward force.
int num_clicks = 0;
//Planing toggle
bool planing_toggle = false;
//Frame count used for delay
int frames = 0;
//Wait after each click
int wait;
//Normal vector & points for plane
hduVector3Dd normal;
hduVector3Dd point1, point2, point3;
//This calculates plane attributes
void plane() {
    hduVector3Dd a = point2 - point1;
    hduVector3Dd b = point3 - point1;
    normal = -1 * a.crossProduct(b);
    if (normal[1] < 0) {
        normal = -1 * normal;
    }
}
//Ouput value from LabView data, this is the y value used for planing
double planingdata_current = 0;
double planingdata_last = 0;

double lin(double xv) {
    if (std::pow(10, 8) * xv >= 1) {
        return 10 * std::log(std::pow(10, 8) * xv);
    }
    else {
        return 0;
    }
}


/*** SCALED Y ***/
//Y position from the scaling done in labview (in units of nm)


/*** POSITIONS ***/
//Final positions being sent to Labview
double xf_current = 0;
double xf_last = 0;
double zf_current = 0;
double zf_last = 0;
//input position from LabView (to make flipping data easier)
hduVector3Dd posLV;
//input y position from LabView BEFORE the scaling function has been applied (raw y from labview in units of nm, doesn't slow down at Zthresh)
std::vector<double> ylabview_last(10);
double ylabview_current = 0;
//The threshold Z at which the pen position starts scaling (decreasing the distance it travels)
double Zthresh = 0;
std::string Zthresh_data;
//The maximum Z at which the current has exceeded the maximum current
double Zmax = 0;
double yk = 1;



/*** HAPTIC CALLBACK ***/
//This is where the arm code is run and the forces are written.
HDCallbackCode HDCALLBACK FrictionlessPlaneCallback(void* data)
{
    hdEnable(HD_MAX_FORCE_CLAMPING);

    const double planeStiffness = 0.35;
    const double time_interval = 2;
    time_t time_initial = time(NULL);
    const double forcetrigger = 2;

    hduVector3Dd force;

    hdBeginFrame(hdGetCurrentDevice());

    hdGetDoublev(HD_CURRENT_POSITION, position);
    hdGetDoublev(HD_CURRENT_VELOCITY, velocity);

    //Calculate z force
    switch (mode) {
    case 0: //Custom Mode
        force = force_labview;
        break;
    case 1: //Read Mode
        if (position[1] <= gain * signal) {
            double penetrationDistance = fabs(position[1] - gain * signal);
            switch (surface_force) {
            case 0: //Linear Force
                force[1] = k[0] * penetrationDistance;
                break;
            case 1: //Coulomb
                force[1] = k[0] * 0.7 / std::pow(penetrationDistance / -15 + 2.17, 2) - 0.12;
                break;
            case 2: //Lennard-Jones Force
                force[1] = k[0] * -4.0 * 0.5 * (6 / std::pow(penetrationDistance / -30 + 2.05, 7) - 12 / std::pow(penetrationDistance / -30 + 2.05, 13));
                break;
            case 3: //Van der Waals Force
                force[1] = k[0] * 1 / std::pow(penetrationDistance / -15 + 1.84, 7);
                break;
            case 4: //Exponential
                force[1] = k[0] * std::pow(2.71828, 0.1 * penetrationDistance - 2) - 0.1;
                break;
            default: //Linear
                force[1] = k[0] * penetrationDistance;
                break;
            }
        }
        else {
            force[1] = 0;
        }
        break;
    case 2: //Write Mode
        switch (surface_force) {
        case 0: //Log Force
            force[1] = k[0] * 2.48218 * std::log(k[1] * (1 / k[2]) * gain * signal + 1);
            break;
        case 1: //Linear Force
            force[1] = k[0] * gain * signal;
        case 2: //Coulomb Force
            if (gain * signal >= 300) {
                force[1] = force_max[1];
            }
            else {
                force[1] = k[0] * 100 / std::pow(lin(gain * signal) - 240, 2);
            }
            break;
        case 3: //Lennard-Jones Potential w/ Exponential position scaling.
            if (gain * signal >= 200) {
                force[1] = force_max[1];
            }
            else {
                force[1] = k[1] * -1.0 * 4.0 * 3.0 * ((6 * std::pow(50, 6)) / std::pow(fabs(10 * lin(gain * signal) - 240), 7) - (12 * std::pow(50, 12)) / std::pow(fabs(10 * lin(gain * signal) - 240), 13));
            }
            break;
        case 4: //Van der Waals
            if (gain * signal >= 300) {
                force[1] = force_max[1];
            }
            else {
                force[1] = k[0] * 1000 / std::pow(lin(gain * signal) - 240, 7);
            }
            break;
        default: //Log
            force[1] = k[0] * 2.48218 * std::log(k[1] * (1 / k[2]) * gain * signal + 1);
            break;
        }
        break;
    }

    //Calculate xy forces & drag forces
    if (wall_toggle) {
        if (position[0] < frame_center_haptic[0] - frame_size_haptic[0] / 2) {
            force[0] = wall_stiffness * (frame_center_haptic[0] - frame_size_haptic[0] / 2 - position[0]) + force[0];
        }
        else if (position[0] > frame_center_haptic[0] + frame_size_haptic[0] / 2) {
            force[0] = (frame_center_haptic[0] + frame_size_haptic[0] / 2 - position[0]) + force[0];
        }
        else if (drag_toggle[0]){
            force[0] = -1 * k_drag[0] * velocity[0] + force[0];
        }

        if (position[2] < frame_center_haptic[1] - frame_size_haptic[1] / 2) {
            force[2] = wall_stiffness * (frame_center_haptic[1] - frame_size_haptic[1] / 2 - position[2]) + force[2];
        }
        else if (position[2] > frame_center_haptic[1] + frame_size_haptic[1] / 2) {
            force[2] = wall_stiffness * (frame_center_haptic[1] + frame_size_haptic[1] / 2 - position[2]) + force[2];
        }
        else if (drag_toggle[2]) {
            force[2] = -1 * k_drag[2] * velocity[2] + force[2];
        }

        if (drag_toggle[1]) {
            force[1] = -1 * k_drag[1] * velocity[1] + force[1];
        }
    } 
    else {
        for (int i = 0; i < 3; i++) {
            if (drag_toggle[i]) {
                force[i] = -1 * k_drag[i] * velocity[i] + force[i];
            }
        }
    }

    //Checks if the calculated forces are within the range of acceptable forces, and, if not, coerces them to within the range.
    for (int i = 0; i < 3; i++) {
        if (force[i] > force_max[i]) {
            force[i] = force_max[i];
        }
        else if (force[i] < force_min[i]) {
            force[i] = force_min[i];
        }
    }

    //Writes the final forces
    if (safetip == false) {
        hdSetDoublev(HD_CURRENT_FORCE, force);
    }
    else {
        hduVector3Dd zeroforce = { 0, 0, 0 };
        hdSetDoublev(HD_CURRENT_FORCE, zeroforce);
    }

    //handles the sequence of choosing points on a plane

    if (num_clicks <= 2 && frames == wait && planing_toggle) {
        h_nano = height_i;
        if ((force_y_nodrag_current <= forcetrigger) && (force_y_nodrag_last >= forcetrigger)) {
            frames = 0;
            if (num_clicks == 0) {
                point1 = posLV;
                num_clicks++;
            }
            else if (num_clicks == 1) {
                point2 = posLV;

                //if statement here
                num_clicks++;
            }
            else {
                point3 = posLV;
                num_clicks++;
                plane();
                frames = 0;
            }
        }
    }
    else {
        h_nano = height_i;
    }

    if (frames < wait && num_clicks < 3) {
        frames++;
    }

    hdEndFrame(hdGetCurrentDevice());

    // In case of error, terminate the callback.
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error detected during main scheduler callback\n");

        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }

    return HD_CALLBACK_CONTINUE;
}

/*** BEGIN LABVIEW FUNCTIONS ***/
/*MAIN*/
//Start & Stop Functions
__declspec(dllexport) int start_haptic_loop(int input)
{
    // Mode switch
        //0: Read mode
        //1: STM Write mode
        //2: AFM Write mode 
    mode = input;
    
    // Main start function
    HDErrorInfo error;
    num_clicks = 0;

    // Initialize the default haptic device.
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        // Failed to initialize haptic device
        return -2;
    }

    // Start the servo scheduler and enable forces.
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        //Error
        return -1;
    }

    // Schedule the frictionless plane callback, which will then run at 
    // servoloop rates and command forces if the user penetrates the plane.
    HDCallbackCode hPlaneCallback = hdScheduleAsynchronous(
        FrictionlessPlaneCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    while (safetip == false)
    {
        return 3;
        if (!hdWaitForCompletion(hPlaneCallback, HD_WAIT_CHECK_STATUS))
        {
            break;
        }
    }

    // Cleanup and shutdown the haptic device, cleanup all callbacks.
    hdStopScheduler();
    hdUnschedule(hPlaneCallback);
    hdDisableDevice(hHD);
    return 0;
}
__declspec(dllexport) void stop_haptic_loop() {
    num_clicks = 0;
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    HDCallbackCode hPlaneCallback = hdScheduleAsynchronous(
        FrictionlessPlaneCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);
    hdStopScheduler();
    hdUnschedule(hPlaneCallback);
    hdDisableDevice(hHD);
}

//Position
__declspec(dllexport) void position_get(double *output) {
    output[0] = position[0];
    output[1] = position[2];
    output[2] = position[1];
}
__declspec(dllexport) void position_reframed_get(double* frame_properties, double* output) {   
    double frame_center_nano[2] = { frame_properties[0], frame_properties[1] };
    double frame_size_nano[2] = { frame_properties[2], frame_properties[3] };

    double position_nano[2] = {position[0], position[2]};
    if (wall_toggle) {
        if (position[0] < frame_center_haptic[0] - frame_size_haptic[0] / 2) {
            position_nano[0] = frame_center_haptic[0] - frame_size_haptic[0] / 2;
        }
        else if (position[0] > frame_center_haptic[0] + frame_size_haptic[0] / 2) {
            position_nano[0] = frame_center_haptic[0] + frame_size_haptic[0] / 2;
        }
        if (position[2] < frame_center_haptic[1] - frame_size_haptic[1] / 2) {
            position_nano[1] = frame_center_haptic[1] - frame_size_haptic[1] / 2;
        }
        else if (position[2] > frame_center_haptic[1] + frame_size_haptic[1] / 2) {
            position_nano[1] = frame_center_haptic[1] + frame_size_haptic[1] / 2;
        }
    }

    position_nano[0] = frame_size_nano[0] / frame_size_haptic[0] * (position_nano[0] - frame_center_haptic[0]) + frame_center_nano[0];
    position_nano[1] = -1 * frame_size_nano[1] / frame_size_haptic[1] * (position_nano[1] - frame_center_haptic[1]) + frame_center_nano[1];

    output[0] = position_nano[0];
    output[1] = position_nano[1];
    output[2] = position[1];
}

//Velocity
__declspec(dllexport) void velocity_get(double* output) {
    output[0] = velocity[0];
    output[1] = velocity[2];
    output[2] = velocity[1];
}
__declspec(dllexport) void velocity_reframed_get(double* frame_properties, double* output) {
    double frame_origin_nano[2] = { frame_properties[0], frame_properties[1] };
    double frame_size_nano[2] = { frame_properties[2], frame_properties[3] };

    output[0] = velocity[0] * frame_size_nano[0] / frame_size_haptic[0];
    output[1] = -velocity[2] * frame_size_nano[1] / frame_size_haptic[1];
    output[2] = velocity[1];
}

//Forces: Exports forces to LabView
__declspec(dllexport) void forces_get(double* output) {
    hduVector3Dd force;
    hdGetDoublev(HD_CURRENT_FORCE, force);
    output[0] = force[0];
    output[1] = force[2];
    output[2] = force[1];
}
__declspec(dllexport) void forces_set(double* input) {
    force_labview[0] = input[0];
    force_labview[1] = input[2];
    force_labview[2] = input[1];
}
__declspec(dllexport) void drag_force_set(int* toggle, double* coeff) {
    drag_toggle[0] = toggle[0];
    drag_toggle[1] = toggle[2];
    drag_toggle[2] = toggle[1];

    k_drag[0] = coeff[0];
    k_drag[1] = coeff[2];
    k_drag[2] = coeff[1];
}
__declspec(dllexport) void force_extrema_set(double* maxforce, double* minforce) {
    force_max[0] = maxforce[0];
    force_max[1] = maxforce[2];
    force_max[2] = maxforce[1];

    force_min[0] = minforce[0];
    force_min[1] = minforce[2];
    force_min[2] = minforce[1];
}

//Angles: Exports angles to LabView
__declspec(dllexport) void angles_get(double *output) {
    hduVector3Dd angles;
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, angles);
    output[0] = angles[0];
    output[1] = angles[1];
    output[2] = angles[2];
}

//Button State
__declspec(dllexport) int8_t buttonstate_get() {
    HDint nCurrentButtons;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nCurrentButtons);
    return nCurrentButtons;
}

/*OTHER*/
__declspec(dllexport) void safety_trigger(bool input, bool *output) {
    if (input) {
        safetip = input;
        stop_haptic_loop();
        *output = true;
    }
    else {
        *output = false;
    }
}
__declspec(dllexport) void frame_wall_set(bool input, double k_wall) {
    wall_toggle = input;
    wall_stiffness = k_wall;
}
__declspec(dllexport) void signal_input(double input_signal, double input_gain) {
    signal = input_signal;
    gain = input_gain;
}
__declspec(dllexport) void surface_force_set(double* input, double* output, int setting) {
    surface_force = setting;
    for (int i = 0; i < 4; i++) {
        k[i] = output[i];
    }
}

//Planing Function
__declspec(dllexport) int8_t planing(bool reset, bool planing, double zlabview, double* output) {
    
    
    output[0] = normal[0];
    output[1] = normal[2];
    output[2] = normal[1];
    output[3] = -1 * (normal[0] * point1[0] + normal[1] * point1[1] + normal[2] * point1[2]);

    planingdata_last = planingdata_current;
    planingdata_current = zlabview;
    posLV = { xf_last, planingdata_last, zf_last };

    if (reset) {
        num_clicks = 0;
    }
    planing_toggle = planing;
    int8_t x = num_clicks;
    return x;
}




__declspec(dllexport) void getcurrent(double currentin, double maxforcey, double minforcey, double setpoint, double maxcurrentin, int forcemode, bool afmmode) {
    afmtoggle = afmmode;
    if (afmmode == false) {
        current_last = fabs(current_current);
        current_current = fabs(currentin);
    }
    else {
        current_last = current_current;
        current_current = currentin;
    }
    force_y_max = maxforcey;
    force_y_min = minforcey;
    current_setpoint = fabs(setpoint);
    current_max = maxcurrentin;
    surface_force = forcemode;
}

__declspec(dllexport) double yrescale(double ylabview, double scalingfactor, double surf, double plunge) {
    double youtput;
    const double c_yscaling = 0.1;
    for (long long int i = ylabview_last.size(); i > 1; i--) {
        ylabview_last[i - 1] = ylabview_last[i - 2];
    }
    ylabview_last[0] = ylabview_current;
    ylabview_current = ylabview;

    if (afmtoggle == false) {
        switch (surface_force) {
        case 0: //Exponential Scaling
            if (current_current >= spc_percent * current_setpoint && current_last < spc_percent * current_setpoint) {
                Zthresh = ylabview;
            }
            if (current_current <= spc_percent * current_setpoint) {
                youtput = ylabview;
            }
            else {
                youtput = (1 / (k_c * 20)) * (exp(k_c * 20 * (ylabview - Zthresh)) - 1) + Zthresh;
            }
            if (youtput >= ylabview && ylabview >= Zthresh) {
                youtput = ylabview;
            }

            break;
        case 1: //Linear Scaling
            if (current_current >= spc_percent * current_setpoint && current_last < spc_percent * current_setpoint) {
                Zthresh = ylabview_current;
            }
            if (current_current <= spc_percent * current_setpoint) {
                youtput = ylabview;
            }
            else {
                youtput = c_yscaling * (ylabview - Zthresh) + Zthresh;
                if (youtput < ylabview) {
                    youtput = ylabview;
                }
            }
            break;
        default://Exponential
            if (current_current >= spc_percent * current_setpoint && current_last < spc_percent * current_setpoint) {
                Zthresh = ylabview;
            }
            if (current_current <= spc_percent * current_setpoint) {
                youtput = ylabview;
            }
            else {
                youtput = (1 / (k_c * 20)) * (exp(k_c * 20 * (ylabview - Zthresh)) - 1) + Zthresh;
            }
            if (youtput >= ylabview && ylabview >= Zthresh) {
                youtput = ylabview;
            }
        }
    }
    else {//AFM scaling
        if (ylabview >= surf) {
            youtput = ylabview;
        }
        else {
            youtput = plunge * exp(1 / plunge * (ylabview - surf)) - plunge + surf;
        }
    }

    if (youtput > 10000) {
        return ylabview;
    }
    else {
        return youtput;
    }
}

__declspec(dllexport) double zlimit(double yscaledinput) {
    double yscaled_current = 75;
    double yscaled_last = yscaled_current;
    yscaled_current = yscaledinput;

    if (current_current >= current_max && current_last < current_max) {
        Zmax = yscaled_last;
    }
    return Zmax;
}

__declspec(dllexport) double zslower(double nanonis_zpos_read, double labview_zpos_write, double maxvel) {
    maxvel = maxvel * 0.001; //nm per millisecond to nm to second
    double newzpos;
    if (labview_zpos_write - nanonis_zpos_read >= maxvel) {
        newzpos = nanonis_zpos_read + maxvel;
    }
    else if (nanonis_zpos_read - labview_zpos_write >= maxvel)
    {
        newzpos = nanonis_zpos_read - maxvel;
    }
    else {
        newzpos = labview_zpos_write;
    }
    return newzpos;
}
