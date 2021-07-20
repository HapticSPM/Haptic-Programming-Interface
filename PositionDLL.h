#pragma once
//Position.h - Contains declarations of functions that provide position data
#pragma once
#include "string"
#include "vector"


#ifdef POSITION_EXPORTS
#define POSITION_API __declspec(dllexport)
#else
#define POSITION_API __declspec(dllimport)
#endif


extern "C" POSITION_API void config(double ypos, int scopex, int scopez, int sizeofimgx, int sizeofimgz, double dragc);

extern "C" POSITION_API int start();

extern "C" POSITION_API double getposx();

extern "C" POSITION_API double getposy();

extern "C" POSITION_API double getposz();

extern "C" POSITION_API void shutdown();

extern "C" POSITION_API double getforcex();

extern "C" POSITION_API double getforcey();

extern "C" POSITION_API double getforcez();

extern "C" POSITION_API double gettheta();

extern "C" POSITION_API double getphi();

extern "C" POSITION_API double getpsi();

extern "C" POSITION_API int clicks();

extern "C" POSITION_API void datain(double xin, double output, double zin);

extern "C" POSITION_API double pd();

extern "C" POSITION_API double pnx();

extern "C" POSITION_API double pny();

extern "C" POSITION_API double pnz();

extern "C" POSITION_API void origin(double xorg, double zorg);

extern "C" POSITION_API double velx();

extern "C" POSITION_API double vely();

extern "C" POSITION_API double velz();