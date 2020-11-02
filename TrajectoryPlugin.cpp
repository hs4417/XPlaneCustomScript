#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMDisplay.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMPlanes.h"
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

//Timing 
int count = 0;
float time0 = 0;
float time = 0;
XPLMDataRef timeid = XPLMFindDataRef("sim/time/total_flight_time_sec");
std::string countstr;
const float pi = 3.14159265359;

//dATAREFS 
//AI
XPLMDataRef xid = XPLMFindDataRef("sim/multiplayer/position/plane1_x");
XPLMDataRef yid = XPLMFindDataRef("sim/multiplayer/position/plane1_y");
XPLMDataRef zid = XPLMFindDataRef("sim/multiplayer/position/plane1_z");
XPLMDataRef vxid = XPLMFindDataRef("sim/multiplayer/position/plane1_v_x");
XPLMDataRef vyid = XPLMFindDataRef("sim/multiplayer/position/plane1_v_y");
XPLMDataRef vzid = XPLMFindDataRef("sim/multiplayer/position/plane1_v_z");
XPLMDataRef aglid = XPLMFindDataRef("sim/multiplayer/position/plane1_el");
XPLMDataRef psid = XPLMFindDataRef("sim/multiplayer/position/plane1_psi");
XPLMDataRef thetaid = XPLMFindDataRef("sim/multiplayer/position/plane1_the");
XPLMDataRef phid = XPLMFindDataRef("sim/multiplayer/position/plane1_phi");
//Player
XPLMDataRef pxid = XPLMFindDataRef(" sim/multiplayer/position/local");
XPLMDataRef pyid = XPLMFindDataRef("sim/flightmodel/position/local_y");
XPLMDataRef pzid = XPLMFindDataRef("sim/flightmodel/position/local_z");
XPLMDataRef pvxid = XPLMFindDataRef("sim/flightmodel/position/local_vx");
XPLMDataRef pvyid = XPLMFindDataRef("sim/flightmodel/position/local_vy");
XPLMDataRef pvzid = XPLMFindDataRef("sim/flightmodel/position/local_vz");
XPLMDataRef paglid = XPLMFindDataRef("sim/flightmodel/position/y_agl");
XPLMDataRef ppsid = XPLMFindDataRef("sim/flightmodel/position/psi");
XPLMDataRef pthetaid = XPLMFindDataRef("sim/flightmodel/position/theta");
XPLMDataRef pphid = XPLMFindDataRef("sim/flightmodel/position/phi");

// Initializing Vairables 
double V = 0;
double alt = 0;
float vx = 0;
float vy = 0;
float vz = 0;

//Payload Trajectory Calculation
void speedcalc() {
	alt = XPLMGetDataf(aglid);
	V = (pow(alt, 3)*2.22591485182242e-12) + (pow(alt, 2)*3.94620905456984e-08) +
	
	(alt*0.00126496448830393) + 26.2941758548717;
	vx = V * cos(15 * pi / 180);
	vy = -V * sin(15 * pi / 180);
	XPLMSetDataf(vxid, vx);
	XPLMSetDataf(vyid, vy);
	XPLMSetDataf(vzid, vz);
	XPLMSetDataf(psid, 90);
	XPLMSetDataf(thetaid, 180);
	XPLMSetDataf(phid, 0);
};

PLUGIN_API int XPluginStart(
	char *		outName,
	char *		outSig,
	char *		outDesc)
{
	strcpy(outName, "Phase1");
	strcpy(outSig, "xpsdk.examples");
	strcpy(outDesc, "Scenario Begin");

	//HotKey
	gHotKey = XPLMRegisterHotKey(XPLM_VK_F2, xplm_DownFlag,
		"Applys Force",
		MyHotKeyCallback,
		NULL);

	return 1;

}

PLUGIN_API void	XPluginStop(void)
{
	XPLMUnregisterHotKey(gHotKey);
	XPLMDestroyFlightLoop(floopid);
}

PLUGIN_API void XPluginDisable(void) { }
PLUGIN_API int  XPluginEnable(void) { return 1; }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam) { }

//Flight loop Function
float	MyFlightLoopCallback(
	float                inElapsedSinceLastCall,
	float                inElapsedTimeSinceLastFlightLoop,
	int                  inCounter,
	void *               inRefcon)
{
	speedcalc();
	//-1 to indicate we want function to run again on next flight loop
	return -1;
}

//When Hotkey is pressed Do this:
void MyHotKeyCallback(void* inRefcon) {
	XPLMSpeakString("IWORK");
	XPLMSetDataf(xid, 0);
	XPLMSetDataf(yid, 6250);
	XPLMSetDataf(zid, 0);
	XPLMSetDataf(pxid, 500); //Places Player Near AI aircraft
	XPLMSetDataf(pyid, 6200);
	XPLMSetDataf(pzid, 0);
	XPLMSetDataf(pvxid, 0);

	//Creating and scheduling flight looop function
	XPLMCreateFlightLoop_t floop = { sizeof(XPLMCreateFlightLoop_t),
	xplm_FlightLoop_Phase_AfterFlightModel,	MyFlightLoopCallback, nullptr };
	floopid = XPLMCreateFlightLoop(&floop);
	XPLMScheduleFlightLoop(floopid, 0.1, 1);

}
