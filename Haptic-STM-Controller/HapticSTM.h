#pragma once

#pragma once


extern "C" __declspec(dllexport) void config(double ypos, int scopex, int scopez, int sizeofimgx, int sizeofimgz, double dragc, bool feedback);

extern "C" __declspec(dllexport) int start();

extern "C" __declspec(dllexport) double getposx();

extern "C" __declspec(dllexport) double getposy();

extern "C" __declspec(dllexport) double getposz();

extern "C" __declspec(dllexport) void shutdown();

extern "C" __declspec(dllexport) double getforcex();

extern "C" __declspec(dllexport) double getforcey();

extern "C" __declspec(dllexport) double getforcez();

extern "C" __declspec(dllexport) double gettheta();

extern "C" __declspec(dllexport) double getphi();

extern "C" __declspec(dllexport) double getpsi();

extern "C" __declspec(dllexport) int clicks(bool reset, bool planing);

extern "C" __declspec(dllexport) void datain(double output);

extern "C" __declspec(dllexport) double pd();

extern "C" __declspec(dllexport) double pnx();

extern "C" __declspec(dllexport) double pny();

extern "C" __declspec(dllexport) double pnz();

extern "C" __declspec(dllexport) void origin(double xorg, double zorg);

extern "C" __declspec(dllexport) double velx();

extern "C" __declspec(dllexport) double vely();

extern "C" __declspec(dllexport) double velz();

extern "C" __declspec(dllexport) int buttonstate();

extern "C" __declspec(dllexport) double threshhold();

extern "C" __declspec(dllexport) void getcurrent(double currentin, double maxforcey, double minforcey, double setpoint, double maxcurrentin, int forcemode);

extern "C" __declspec(dllexport) double yrescale(double ylabview, double scalingfactor);

extern "C" __declspec(dllexport) double zlimit(double yscaledinput);

extern "C" __declspec(dllexport) bool zlimitreached();

extern "C" __declspec(dllexport) double zslower(double nanonis_zpos_read, double labview_zpos_write, double maxvel);

extern "C" __declspec(dllexport) double yforcetestnodrag();

extern "C" __declspec(dllexport) void forceconfig(double a, double b, double c);