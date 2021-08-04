// Position.cpp : Defines the exported functions for the DLL.
#include "pch.h" // use stdafx.h in Visual Studio 2017 and earlier
#include <utility>
#include <limits.h>
#include "HapticSTM.h"

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
#include <HL/hl.h>

#include <string>
#include <iostream>
#include <Windows.h>
#include <vector>

//Completely unnecessary but at this point I'm too afraid to change anything
double frac(double xin, double yin) {
    return xin / yin;
}

/***** BEGIN GLOBAL VARIABLES *****/

/*** HAPTIC FRAME PROPERTIES ***/
//Frame origin, the bottom left corner of the haptic workspace (in units of mm), the area in which the pen is bounded.
const double x0_haptic = -60;
const double z0_haptic = 80;
//The frame width and frame height of the haptic workspace (in units of mm).
const double fw_haptic = 120;
const double fh_haptic = 120;

/*** NANONIS FRAME PROPERTIES ***/
//Frame origin, the very center of the frame set in scan mode of Nanonis.
double x0_nano = 0;
double z0_nano = 0;
//Frame size, the width (x) and height (z) of the frame set in scan mode of Nanonis.
double fw_nano = 159;
double fh_nano = 159;
//The x-z positions of the pen relative to the nanonis frame
double x_nano;
double z_nano;
//Scaling constants for mapping purposes
double scale_x = frac(fw_nano, fw_haptic);
double scale_z = frac(fh_nano, fh_haptic);
//Functions that maps the data
double mapx(double xunmap) {
    double xvar;
    if (scale_x * (xunmap - x0_haptic) >= fw_nano) {
        xvar = fw_nano;
    }
    else if (scale_x * (xunmap - x0_haptic) <= 0) {
        xvar = 0;
    }
    else {
        xvar = scale_x * (xunmap - x0_haptic);
    }
    return xvar;
}
double mapz(double zunmap) {
    double zvar;
    if (scale_z * (z0_haptic - zunmap) >= fh_nano) {
        zvar = fh_nano;
    }
    else if (scale_z * (z0_haptic - zunmap) <= 0) {
        zvar = 0;
    }
    else {
        zvar = scale_z * (z0_haptic - zunmap);
    }
    return zvar;
}


/*** ZOOM FRAME PROPERTIES ***/
//Parameters for the zoomed in canvas, as of now this feature is ignored and the zooming is done in Nanonis scan mode.
double fw_zoom = 159;
double fh_zoom = 159;


/*** FEEDBACK MODE ***/
/* Note: "Feedback mode" corresponds to the mode in which the haptic pen controls the x - y position in nanonis but not the z position.
   The STM is left tunneling, the z position is read and converted to a force. "Raw" or "Feedbackless" mode is the mode in which all 
   positions of the haptic pen (x, y, z) control the STM tip. The STM tunneling is turned off, and the tunneling current is converted 
   to a force in the y direction. */       
//The initial height of the plane from LabView before the planing.
double height_i = 20;
//The height of the plane based on the input data from nanonis (i.e. how the features are felt).
double h_nano = 20;
//Feedbackmode toggle, when true, feedback mode is on, and when false, feedbackless mode is on.
bool feedbackmode = 1;


/*** CURRENT ***/
//Setpoint Current, set in nanonis, roughly determines the current (pA) at which the force is neutral.
double current_setpoint = 100;
//Maximum current is maximum allowed current (pA) before the safety mechanisms kick in
double current_max = 10000;
//Live input current
double current_current = 0;
double current_last = 0;
//The percent of the setpointcurrent which triggers Zthresh
double spc_percent = 0.75;
//Minimum current it takes to write a force
double mincurrent = 0;


/*** FORCE PARAMETERS ***/
//Sets the drag coefficients.
double dragc_x = 0.001;
double dragc_z = 2 * dragc_x;
//The force component in each direction.
double force_x = 0;
double force_y = 0;
double force_z = 0;
//force_y limits.
double force_y_max = 2.485;
double force_y_min = 0.275;
//y force necessary to register click
double forcetrigger;
//dragless force_y numbers
double force_y_nodrag_current;
double force_y_nodrag_last;
//determines force scaling
int forcesetting = 0;
double force(double c) {
    return 1.2 * std::log(c / (current_setpoint - 40));
}


/*** PLANING SEQUENCE ***/
//Number of clicks for planing, 3 is necessary before the plane becomes 'active'. 
//A click is triggered by experiencing a sufficient level of downward force.
int num_clicks = 0;
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


/*** VELOCITY ***/
hduVector3Dd velocity;


/*** AVERAGE VELOCITY ***/
//Note: This averaging was meant to smooth the drag force. This is no longer necessary, but I'll leave this here in case it becomes useful again.
//Amount of frames to be averaged over.
const int lastframes = 100;
//Vector of last velocities.
std::vector<hduVector3Dd> lastvel(lastframes);
//PROBLEM: no idea what this does
bool testvar = false;
hduVector3Dd avgvel;


/*** SCALED Y ***/
//Y position from the scaling done in labview (in units of nm)
double yscaled_current;
double yscaled_last;


/*** BUTTON ***/
int button_state;


/*** POSITIONS ***/
//Final positions being sent to Labview
double xf_current = 0;
double xf_last = 0;
double zf_current = 0;
double zf_last = 0;
//input position from LabView (to make flipping data easier)
hduVector3Dd posLV;
//input y position from LabView BEFORE the scaling function has been applied (raw y from labview in units of nm, doesn't slow down at Zthresh)
double ylabview_last = 0;
double ylabview_current = 0;
//The threshold Z at which the pen position starts scaling (decreasing the distance it travels)
double Zthresh = 0;
//The maximum Z at which the current has exceeded the maximum current
double Zmax = 0;


/*** HAPTIC CALLBACK ***/
//This is where the main code is run and the forces are written. Most code is in this loop.
HDCallbackCode HDCALLBACK FrictionlessPlaneCallback(void* data)
{
    hdEnable(HD_MAX_FORCE_CLAMPING); 

    // Stiffness, i.e. k value, of the plane and walls.  Higher stiffness results
    // in a harder surface.
    const double planeStiffness = 0.35;
    const double wallStiffness = 0.35;
    
    hdBeginFrame(hdGetCurrentDevice());

    //Gets forces
    hduVector3Dd lastforce;
    hdGetDoublev(HD_LAST_FORCE, lastforce);
    hduVector3Dd currentforce;
    hdGetDoublev(HD_CURRENT_FORCE, currentforce);
    forcetrigger = force(current_setpoint);
    
    // Get the position of the device.
    hduVector3Dd position;
    hdGetDoublev(HD_CURRENT_POSITION, position);

    // Get the velocity of the device.
    hdGetDoublev(HD_CURRENT_VELOCITY, velocity);

    //These map the raw position of the arm to the nanonis frame.
    //Both are bounded by [0, imgsize].
    
    x_nano = mapx(position[0]);
    z_nano = mapz(position[2]);

    //these if statements calculate the forces for the bounding box
    //x:
    if (x_nano == 0)
    {
        double penetrationDistancex = frac(159, fw_nano) * fabs(scale_x * (position[0] - x0_haptic));
        force_x = wallStiffness * penetrationDistancex;
    }
    else if (x_nano == fw_nano)
    {
        double penetrationDistancex = frac(159, fw_nano) * fabs(fw_nano - scale_x * (position[0] - x0_haptic));
        force_x = -1 * wallStiffness * penetrationDistancex;
    }
    else {
        force_x = -1 * dragc_x * velocity[0];
    }
    //z:
    if (z_nano == 0)
    {
        double penetrationDistancez = frac(159, fh_nano) * fabs(scale_z * (z0_haptic - position[2]));
        force_z = -1 * wallStiffness * penetrationDistancez;
    }
    else if (z_nano == fh_nano)
    {
        double penetrationDistancez = frac(159, fh_nano) * fabs(fh_nano - scale_z * (z0_haptic - position[2]));
        force_z = wallStiffness * penetrationDistancez;
    }
    else {
        force_z = -1 * dragc_z * velocity[2];
    }   
    
    if (feedbackmode == 1) {
        //Calculates y force depending on the input height from LabView (or the default value)
        if (position[1] <= h_nano)
        {
            double penetrationDistance = fabs(position[1] - h_nano);
            force_y = planeStiffness * penetrationDistance;

        }
        else {
            force_y = 0;
        }
    }
    else {
        double k2 = 0.1;
        
        switch (forcesetting) {
            case 0: //Stacked Logs
                if (current_current > spc_percent * current_setpoint) {
                    force_y = std::log(current_current / (0.25 * current_setpoint));
                }
                else if (current_current < mincurrent) {
                    force_y = 0;
                }
                else {
                    force_y = (std::log(4 * spc_percent) / std::log(spc_percent * current_setpoint + 1)) * std::log(current_current + 1);
                }
                break;
            case 1: //Simple Log Scale
                if (current_current > current_setpoint) {
                    force_y = std::log(current_current / current_setpoint);
                }
                else if (current_current <= current_setpoint) {
                    force_y = 0;
                }
                break;
            case 2: //Linear Scale
                if (current_current > spc_percent * current_setpoint) {
                    force_y = (k2 / (spc_percent * current_setpoint)) * (current_current - spc_percent * current_setpoint);
                }
                else if (current_current <= spc_percent * current_setpoint) {
                    force_y = 0;
                }
                break;
            default:
                if (current_current > spc_percent * current_setpoint) {
                    force_y = std::log(current_current / (0.25 * current_setpoint));
                }
                else if (current_current < mincurrent) {
                    force_y = 0;
                }
                else {
                    force_y = (std::log(4 * spc_percent) / std::log(spc_percent * current_setpoint + 1)) * std::log(current_current + 1);
                }
        }
        

        if (force_y > force_y_max) {
            force_y = force_y_max;
        }
        else if (force_y < 0) {
            force_y = 0;
        }
    }

    force_y_nodrag_last = force_y_nodrag_current;
    force_y_nodrag_current = force_y;

    //if (current <= 5) {
        force_y = -2 * dragc_z * velocity[1] + force_y;
    //}

    
    //Sets wait time between choosing points, 1st takes longer
    if (num_clicks == 0) {
        wait = 3500;
    }
    else {
        wait = 1500;
    }
    
    //writes calculated forces to the device
    hduVector3Dd finalforce = { force_x, force_y, force_z };
    hdSetDoublev(HD_CURRENT_FORCE, finalforce);
        
    HDint nCurrentButtons;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nCurrentButtons);

    button_state = nCurrentButtons;


    //handles the sequence of choosing points on a plane
    if (num_clicks <= 2 && frames == wait) {
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

    // Finds the average velocity, could be used if the drag forces get out of control. As of now, they aren't used anywhere.
    if (num_clicks == 3) {
        lastvel[frames] = velocity;
    }
    hduVector3Dd x;

    if (testvar == 1) {
        for (int i = 0; i < lastframes; i++) {
            if (i == 0) {
                x = lastvel[i];
            }
            else {
                x = x + lastvel[i];
            }
        }
        x = x / lastframes;
        avgvel = x;
    }
    
    if (num_clicks == 3) {
        if (frames < lastframes - 1) {
            frames++;

        }
        else {
            frames = 0;
            testvar = 1;
        }
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

//this function is run only once, and is responsible for starting the device/frames
__declspec(dllexport) int start()
{
    HDErrorInfo error;
    num_clicks = 0;

    // Initialize the default haptic device.
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        //Failed to initialize haptic device
        return -1;
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

    while (!_kbhit())
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

//Positions: These map the position on the canvas to desired scope of the data
__declspec(dllexport) double getposx() {
    xf_last = xf_current;
    xf_current = frac(fw_zoom, fw_nano) * x_nano + x0_nano;
    return xf_current;
}
__declspec(dllexport) double getposy() {
    //Note: Y rescaling is done in labview
    hduVector3Dd position;
    hdGetDoublev(HD_CURRENT_POSITION, position);
    return position[1];
}
__declspec(dllexport) double getposz() {
    zf_last = zf_current;
    zf_current = frac(fh_zoom, fh_nano) * z_nano + z0_nano;
    return zf_current;
}

//Exports forces to LabView
__declspec(dllexport) double getforcex() {
    hduVector3Dd force;
    hdGetDoublev(HD_CURRENT_FORCE, force);
    return force[0];
}
__declspec(dllexport) double getforcey() {
    hduVector3Dd force;
    hdGetDoublev(HD_CURRENT_FORCE, force);
    return force[1];
}
__declspec(dllexport) double getforcez() {
    hduVector3Dd force;
    hdGetDoublev(HD_CURRENT_FORCE, force);
    return force[2];
}


//Exports angles to LabView
__declspec(dllexport) double gettheta() {
    hduVector3Dd angles;
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, angles);
    return angles[0];
}
__declspec(dllexport) double getphi() {
    hduVector3Dd angles;
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, angles);
    return angles[1];
}
__declspec(dllexport) double getpsi() {
    hduVector3Dd angles;
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, angles);
    return angles[2];
}

//Parameters from LabView
__declspec(dllexport) void config(double ypos, int scopex, int scopez, int sizeofimgx, int sizeofimgz, double dragc, bool feedback) {
    height_i = ypos;
    fw_nano = sizeofimgx;
    fh_nano = sizeofimgz;
    fw_zoom = scopex;
    fh_zoom = scopez;
    scale_x = frac(fw_nano, fw_haptic);
    scale_z = frac(fh_nano, fh_haptic);
    dragc_x = dragc;
    dragc_z = dragc_x;
    feedbackmode = feedback;
}
__declspec(dllexport) void origin(double xorg, double zorg) {
    x0_nano = xorg;
    z0_nano = zorg;
}

//counts the number of clicks
__declspec(dllexport) int clicks(bool reset) {
    if (reset) {
        num_clicks = 0;
    }
    int seq;
    seq = num_clicks;
    return seq;
}


//Imports position data from LabView (to allow flipping of data)
__declspec(dllexport) void datain(double output) {
    planingdata_last = planingdata_current;
    planingdata_current = output;
    posLV = { xf_last, planingdata_last, zf_last };
}

//Exports plane attributes to LabView
__declspec(dllexport) double pd() {
    return -1 * (normal[0] * point1[0] + normal[1] * point1[1] + normal[2] * point1[2]);
}
__declspec(dllexport) double pnx() {
    return normal[0];
}
__declspec(dllexport) double pny() {
    return normal[1];
}
__declspec(dllexport) double pnz() {
    return normal[2];
}

__declspec(dllexport) double velx() {
    return velocity[0] * scale_x * frac(fw_zoom, fw_nano);
}

__declspec(dllexport) double vely() {
    return velocity[1];
}

__declspec(dllexport) double velz() {
    return velocity[2] * scale_z * frac(fh_zoom, fh_nano);
}
__declspec(dllexport) void getcurrent(double currentin, double maxforcey, double minforcey, double setpoint, double maxcurrentin, int forcemode) {
    current_last = fabs(current_current);
    current_current = fabs(currentin);
    force_y_max = maxforcey;
    force_y_min = minforcey;
    current_setpoint = fabs(setpoint);
    current_max = maxcurrentin;
    forcesetting = forcemode;
}

__declspec(dllexport) double yrescale(double ylabview, double scalingfactor) {
    double youtput;
    double logscale = -1 * scalingfactor;
    ylabview_last = ylabview_current;
    ylabview_current = ylabview;

    switch (forcesetting) {
        case 0:
            if (current_current >= current_setpoint && current_last < current_setpoint) {
                Zthresh = ylabview_last;
            }
            break;
        case 1:
            if (current_current >= current_setpoint && current_last < current_setpoint) {
                Zthresh = ylabview_last;
            }
            break;
        case 2:
            if (current_current >= spc_percent * current_setpoint && current_last < spc_percent * current_setpoint) {
                Zthresh = ylabview_last;
            }
            break;
        default:
            if (current_current >= spc_percent * current_setpoint && current_last < spc_percent * current_setpoint) {
                Zthresh = ylabview_last;
            }
    }
    
    
    //if (ylabview >= Zthresh) {
    //    youtput = ylabview;
    //}
    //else {
    //    //Log Scaling:
    //    //youtput = 1 / logscale * std::log(logscale * (ylabview - Zthresh) + 1) + Zthresh; 
    //    //Linear scaling:
    //    youtput = 0.1 * (ylabview - Zthresh) + Zthresh;
    //}

    if (current_current <= spc_percent * current_setpoint) {
        youtput = ylabview;
    }
    else {
        //Log Scaling:
        //youtput = 1 / lmaogscale * std::log(logscale * (ylabview - Zthresh) + 1) + Zthresh; 
        //Linear scaling:
        youtput = 0.1 * (ylabview - Zthresh) + Zthresh;
        if (youtput < ylabview) {
            youtput = ylabview;
        }
    }
    
    if (youtput > 10000) {
        return ylabview;
    }
    else {
        return youtput;
    }
    
}

__declspec(dllexport) int buttonstate() {
    return button_state;
}

__declspec(dllexport) double zlimit(double yscaledinput) {
    yscaled_last = yscaled_current;
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

//Shuts down device
__declspec(dllexport) void shutdown() {
    num_clicks = 0;
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    HDCallbackCode hPlaneCallback = hdScheduleAsynchronous(
    FrictionlessPlaneCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);
    hdStopScheduler();
    hdUnschedule(hPlaneCallback);
    hdDisableDevice(hHD); 
}