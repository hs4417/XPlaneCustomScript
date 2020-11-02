#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMDisplay.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include <string.h>
#include <iostream>
#include <string>
#include <cmath>
#if IBM
	#include <windows.h>
#endif
#if LIN
	#include <GL/gl.h>
#elif __GNUC__
	#include <OpenGL/gl.h>
#else
	#include <GL/gl.h>
#endif
       
#ifndef XPLM300
	#error This is made to be compiled against the XPLM300 SDK
#endif

//HotKey
static XPLMHotKeyID	gHotKey = NULL;
static void	MyHotKeyCallback(void * inRefcon);

//FlightLoop
static float	MyFlightLoopCallback(
	float                inElapsedSinceLastCall,
	float                inElapsedTimeSinceLastFlightLoop,
	int                  inCounter,
	void *               inRefcon);
XPLMFlightLoopID floopid = NULL;

const float pi = 3.14159265359;
//Timing 
int count = 0;
float time0= 0;
float time = 0;
XPLMDataRef timeid = XPLMFindDataRef("sim/time/total_flight_time_sec");
std::string countstr;

//Forces and Angles  
XPLMDataRef fsidid = XPLMFindDataRef("sim/flightmodel/forces/fside_plug_acf");
XPLMDataRef fnrmid = XPLMFindDataRef("sim/flightmodel/forces/fnrml_plug_acf");
XPLMDataRef faxilid = XPLMFindDataRef("sim/flightmodel/forces/faxil_plug_acf");
XPLMDataRef Lplgid = XPLMFindDataRef("sim/flightmodel/forces/L_plug_acf");
XPLMDataRef Mplgid = XPLMFindDataRef("sim/flightmodel/forces/M_plug_acf");
XPLMDataRef Nplgid = XPLMFindDataRef("sim/flightmodel/forces/N_plug_acf");
XPLMDataRef thetaid = XPLMFindDataRef("sim/flightmodel/position/theta");
XPLMDataRef phid = XPLMFindDataRef("sim/flightmodel/position/phi");

float X = 0;//Rightmost distance from centreline 
float Y = -0.753;//Distance Above CG
float Z = 0.024; //Longitudional Distance From CG

float dr = 0.4559; //Damping Ratio
float w = 50; //Natural Frequency

float timedelay = 0;
float tforcechange = 1.385;//Time at which forces changes from 

float F1 = 100;
float F2 = 0;
float latangle = 22; //Angle from Helicopter (Left=Positive)
float longangle = 34;//Trail Angle
float xVmulti = 0;
float yVmulti = 0;
float zVmulti = 0;

float Fx = 0; //Body Side Force
float Fy = 0; //Body Normal Force
float Fz = 0; //Body Axial Force

float fside= 0; 
float L =0;
float fnrml = 0; 
float M = 0;
float faxil = 0;
float N = 0;

//Calculating Force Vector Using Slung Line angle, and Euler angles to transform into body fixed
void forceanglecalc(float _F) { 

	xVmulti = cos(pi*((90 - latangle) + XPLMGetDataf(phid)) / 180.0)*-1;
	yVmulti = sin(pi*( (90-latangle)+XPLMGetDataf(phid) ) / 180.0)*-1;
	yVmulti *= sin(pi*((90 - longangle) - XPLMGetDataf(thetaid)) / 180.0);
	zVmulti = cos(pi*((90 - longangle) - XPLMGetDataf(thetaid)) / 180.0);
	xVmulti *= sin(pi*((90 - longangle) - XPLMGetDataf(thetaid)) / 180.0);
	Fx = _F * xVmulti;
	Fy = _F * yVmulti;
	Fz = _F * zVmulti;
}

//Calculating Forces and Moments from force vecter applied at certian distance from CG
void forcecalc(float _Fx,float _Fy,float _Fz) {
	fside = _Fx;
	L = _Fx * Y - _Fy * X;
	fnrml = _Fy;
	M = _Fz * Y - _Fy * Z;
	faxil = _Fz;
	N = _Fz * X - _Fx * Z;
};

//Applys Force
void forceapply() {
	XPLMSetDataf(fsidid, fside);
	XPLMSetDataf(fnrmid, fnrml);
	XPLMSetDataf(faxilid, faxil);
	XPLMSetDataf(Lplgid, L);
	XPLMSetDataf(Mplgid, M);
	XPLMSetDataf(Nplgid, N);
};

PLUGIN_API int XPluginStart(
							char *		outName,
							char *		outSig,
							char *		outDesc)
{
	strcpy(outName, "ForcePlugin");
	strcpy(outSig, "xpsdk.examples.helloworld3plugin");
	strcpy(outDesc, "Plugin To simulate dynamic force on aircraft");

	//HotKey
	gHotKey = XPLMRegisterHotKey(XPLM_KEY_TAB, xplm_DownFlag,
		"Applys Force",
		MyHotKeyCallback,
		NULL);
	
	for (float t = 0; t < 4; t = t + 0.01) { //Finding time in Second Order system where Force=Maximum Displacement Driven Force
		F2 = 1.8607e+04 * (1.0 - (1.0 / (sqrt(1.0-(dr*dr) )))*exp(-dr * w*t)*sin(sqrt(1 -(dr*dr) )*w*t
			+ acos(dr)));
		if (F2 >= 1.8607e+04) {
			timedelay = t;
			timedelay = timedelay - tforcechange; //Creating Time Delay for force, so the the Second Order System Starts at the right force 
			break;
		}
	}
	return 1;
}

PLUGIN_API void	XPluginStop(void)
{
	
	XPLMUnregisterHotKey(gHotKey);
	XPLMDestroyFlightLoop(floopid);

}

PLUGIN_API void XPluginDisable(void) { }
PLUGIN_API int  XPluginEnable(void)  { return 1; }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam) { }


float	MyFlightLoopCallback(
	float                inElapsedSinceLastCall,
	float                inElapsedTimeSinceLastFlightLoop,
	int                  inCounter,
	void *               inRefcon)
{ 

	if (count == 0) { //Starting Timer
		time0=XPLMGetDataf(timeid);
	};
	time=XPLMGetDataf(timeid)-time0; //Time elapsed since start of force
	if (time > 5) {
		XPLMSpeakString("Finish");
		countstr = std::to_string(XPLMGetDataf(timeid)); //Display at what time force ended
		char const *pchar = countstr.c_str();
		XPLMSpeakString(pchar);
		time = 0;
		count = 0;
		return 0;
	}
	if (time<tforcechange) { //Displacement driven 
		count = count + 1;
		F1 = (2188.5*time*time)+(10212*time)+(143.67);
		forceanglecalc(F1);
		forcecalc(Fx, Fy, Fz);
		forceapply();
		return -1;
		};
	if (time >= tforcechange) { //Second Order System
		F2= 1.8607e+04 * (1.0 - (1.0 / (sqrt(1.0 - (dr*dr))))*exp(-dr * w*(time+timedelay))
			*sin((sqrt(1 - (dr*dr))*w*(time+timedelay))+ acos(dr))); 
		forceanglecalc(F2);
		forcecalc(Fx, Fy, Fz);
		forceapply();
		return -1;
	}
	return -1;
	
}

void MyHotKeyCallback(void* inRefcon) {
	XPLMCreateFlightLoop_t floop = { sizeof(XPLMCreateFlightLoop_t),xplm_FlightLoop_Phase_AfterFlightModel,
		MyFlightLoopCallback, nullptr };
	floopid = XPLMCreateFlightLoop(&floop);
	XPLMScheduleFlightLoop(floopid, 3, 1);
}
