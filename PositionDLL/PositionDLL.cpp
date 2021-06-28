// Position.cpp : Defines the exported functions for the DLL.
#include "pch.h" // use stdafx.h in Visual Studio 2017 and earlier
#include <utility>
#include <limits.h>
#include "PositionDLL.h"

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

//This is a dumb way to use fractions when declaring variables.
double frac(double xin, double yin) {
    return xin / yin;
}

//height of the plane from LabView
double height = 20;

double xorigin = 0;
double zorigin = 0;

//initial height of plane
double h = 20;

//parameters for the initial 159x159 canvas
double imgsizex = 159;
double imgsizez = 159;
double imgsize = imgsizex;
const double xzero = -60;
const double zzero = 80;
double scalex = frac(imgsizex, 120);
double scalez = frac(imgsizez, 120);

//parameters for the zoomed in canvas
double sizeofbox = 159;

//Sets drag coefficients
double coeffx = 0.001;
double coeffz = coeffx;

//the force component in each direction
double forcex = 0;
double forcey = 0;
double forcez = 0;

//the positions mapped to the initial 159x159 canvas
double xmap0;
double zmap0;

int count = 0;

//params for clicking sequence, waits a certain amount before the subsequent click
int sequence = 0;
int frames = 0;
int wait;

const int lastframes = 100;
std::vector<hduVector3Dd> lastvel(lastframes);
bool testvar = false;
hduVector3Dd avgvel;
//ouput value from LabView data
double dataout = 0;


//input position from LabView (to make flipping data easier)
hduVector3Dd posLV = {0,0,0};

//normal & points for plane
hduVector3Dd n;
hduVector3Dd point1 = { 0,0,0 };
hduVector3Dd point2, point3;
hduVector3Dd vel = { 0,0,0 };

//This calculates plane attributes
void plane() {
    hduVector3Dd a = point2 - point1;
    hduVector3Dd b = point3 - point1;
    n = a.crossProduct(b);
}

//Haptic plane callback.
HDCallbackCode HDCALLBACK FrictionlessPlaneCallback(void* data)
{
    hdEnable(HD_MAX_FORCE_CLAMPING);

    // Stiffness, i.e. k value, of the plane.  Higher stiffness results
    // in a harder surface.
    const double planeStiffness = 0.35;
    const double wallStiffness = 0.35;
    
    //Amount of force to trigger choosing point
    int forcetrigger = 2;


    hdBeginFrame(hdGetCurrentDevice());
    
    // Get the position of the device.
    hduVector3Dd position;
    hdGetDoublev(HD_CURRENT_POSITION, position);

    // Get the velocity of the device.
    hduVector3Dd velocity;
    hdGetDoublev(HD_CURRENT_VELOCITY, velocity);

    vel = velocity;

    //These if statements map the raw position of the arm to a imgsize x imgsize canvas.
    //Both are bounded by [0, imgsize].
    if (scalex * (position[0] - xzero) >= imgsizex) {
        xmap0 = imgsizex;
    }
    else if (scalex * (position[0] - xzero) <= 0) {
        xmap0 = 0;
    }
    else {
        xmap0 = scalex * (position[0] - xzero);
    }

    if (scalez * (zzero - position[2]) >= imgsizez) {
        zmap0 = imgsizez;
    }
    else if (scalez * (zzero - position[2]) <= 0) {
        zmap0 = 0;
    }
    else {
        zmap0 = scalez * (zzero - position[2]);
    }

   

    //these if statements calculate the forces for the bounding box
    //x:
    if (xmap0 == 0)
    {
        double penetrationDistancex = frac(159, imgsizex) * fabs(scalex * (position[0] - xzero));
        double kx = wallStiffness;
        double xx = penetrationDistancex;
        forcex = kx * xx;
    }
    else if (xmap0 == imgsizex)
    {
        double penetrationDistancex = frac(159, imgsizex) * fabs(imgsizex - scalex * (position[0] - xzero));
        double kx = wallStiffness;
        double xx = penetrationDistancex;
        forcex = -1 * kx * xx;
 
    }
    else {
        forcex = -1 * coeffx * velocity[0];
    }
    //z:
    if (zmap0 == 0)
    {
        double penetrationDistancex = frac(159, imgsizez) * fabs(scalez * (zzero - position[2]));
        double kz = wallStiffness;
        double xz = penetrationDistancex;
        forcez = -1 * kz * xz;
    }
    else if (zmap0 == imgsizez)
    {
        double penetrationDistancex = frac(159, imgsizez) * fabs(imgsizez - scalez * (zzero - position[2]));
        double kz = wallStiffness;
        double xz = penetrationDistancex;
        forcez = kz * xz;
    }
    else {
        forcez = -2 * coeffz * velocity[2];
    }   

    //Calculates y force depending on the input height from LabView (or the default value)
    if (position[1] <= h )
    {
        double penetrationDistance = fabs(position[1] - h);
        // Hooke's law explicitly:
        double k = planeStiffness;
        double x = penetrationDistance;
        forcey = k * x;
       
    }
    else {
        forcey = 0;
    }
    
    
    //Sets wait time between choosing points, 1st takes longer
    if (sequence == 0) {
        wait = 3500;
    }
    else {
        wait = 1500;
    }
    
    //writes calculated forces to the device
    hduVector3Dd finalforce = { forcex, forcey, forcez };
    hdSetDoublev(HD_CURRENT_FORCE, finalforce);
        
    //Logs the current and last forces
    hduVector3Dd lastforce;
    hdGetDoublev(HD_LAST_FORCE, lastforce);
    hduVector3Dd currentforce;
    hdGetDoublev(HD_CURRENT_FORCE, currentforce);


    //handles the sequence of choosing points on a plane
    if (sequence <= 2 && frames == wait) {
        h = height;
        if ((currentforce[1] <= forcetrigger) && (lastforce[1] >= forcetrigger)) {
            frames = 0;
            if (sequence == 0) {
                point1 = posLV;
                sequence++;
            }
            else if (sequence == 1) {
                point2 = posLV;
                sequence++;
            }
            else {
                point3 = posLV;
                sequence++;
                plane();
                frames = 0;
            }
        }
    }
    else {
        h = height;
    }

    if (frames < wait && sequence < 3) {
        frames++;
    }

    
    if (sequence == 3) {
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
    
    if (sequence == 3) {
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

    sequence = 0;

    // Initialize the default haptic device.
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }

    // Start the servo scheduler and enable forces.
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        getch();
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
            getch();
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
    return frac(sizeofbox, imgsizex) * xmap0 + xorigin;
}
__declspec(dllexport) double getposy() {
    hduVector3Dd position;
    hdGetDoublev(HD_CURRENT_POSITION, position);
    return position[1];
}
__declspec(dllexport) double getposz() {
    return frac(sizeofbox, imgsizez) * zmap0 + zorigin;
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
__declspec(dllexport) void config(double ypos, int scope, int sizeofimgx, int sizeofimgz, double dragc) {
    height = ypos;
    imgsizex = sizeofimgx;
    imgsizez = sizeofimgz;
    sizeofbox = scope;
    scalex = frac(imgsizex, 120);
    scalez = frac(imgsizez, 120);
    coeffx = dragc;
    coeffz = coeffx;
    
}

__declspec(dllexport) void origin(double xorg, double zorg) {
    xorigin = xorg;
    zorigin = zorg;
}

//counts the number of clicks
__declspec(dllexport) int clicks() {
    double seq;
    seq = sequence;
    return seq;
}

//Imports position data from LabView (to allow flipping of data)
__declspec(dllexport) void datain(double xin, double output, double zin) {
    dataout = output;
    posLV = { xin, output, zin };
}

//maps data (probably could replace in HDCALLBACK to make more efficient)
double mapx(double xunmap) {
    double xvar;
    if (scalex * (xunmap - xzero) >= imgsizex) {
        xvar = imgsizex;
    }
    else if (scalex * (xunmap - xzero) <= 0) {
        xvar = 0;
    }
    else {
        xvar = scalex * (xunmap - xzero);
    }
    return xvar;
}
double mapz(double zunmap) {
    double zvar;
    if (scalez * (zzero - zunmap) >= imgsizez) {
        zvar = imgsizez;
    }
    else if (scalez * (zzero - zunmap) <= 0) {
        zvar = 0;
    }
    else {
        zvar = scalez * (zzero - zunmap);
    }
    return zvar;
}

//Exports plane attributes to LabView
__declspec(dllexport) double pd() {
    return -1 * (n[0] * point1[0] + n[1] * point1[1] + n[2] * point1[2]);
}
__declspec(dllexport) double pnx() {
    return n[0];
}
__declspec(dllexport) double pny() {
    return n[1];
}
__declspec(dllexport) double pnz() {
    return n[2];
}

__declspec(dllexport) double velx() {
    return avgvel[0] * scalex * frac(sizeofbox, imgsizex);
}

__declspec(dllexport) double vely() {
    return avgvel[1];
}

__declspec(dllexport) double velz() {
    return avgvel[2] * scalez * frac(sizeofbox, imgsizez);
}


//Shuts down device
__declspec(dllexport) void shutdown() {
    sequence = 0;
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    HDCallbackCode hPlaneCallback = hdScheduleAsynchronous(
    FrictionlessPlaneCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);
    hdStopScheduler();
    hdUnschedule(hPlaneCallback);
    hdDisableDevice(hHD); 
}

