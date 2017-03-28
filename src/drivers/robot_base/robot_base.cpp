
#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

static void drive(int index, tCarElt* car, tSituation *s); 
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 

/* 
 * Module entry point  
 */ 
extern "C" int 
robot_base(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("robot_base");		/* name of the module (short) */
    modInfo->desc    = strdup("");	/* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		/* init function */
    modInfo->gfId    = ROB_IDENT;		/* supported framework version */
    modInfo->index   = 1;

    return 0; 
} 

/* Called for every track change or new race. */
static void
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
	static tTrack	*curTrack;
	curTrack = track;
	*carParmHandle = NULL;
}

static void
newRace(int index, tCarElt* car, tSituation *s)
{
}
/* Start a new race. */
static void
endRace(int index, tCarElt* car, tSituation *s)
{
}


/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newRace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endRace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 

static void endRace(int index, tCarElt *car, tSituation *s);
static void newRace(int index, tCarElt* car, tSituation *s);
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);

bool isStuck(tCarElt* car)
{
	static int stuck = 0;

	int maxUnstuckCount;
	int index;

	static  float maxUnstuckAngle = 30.0 / 180.0*PI;
	static  float unstuckTimeLimit = 2.0;

	maxUnstuckCount = int(maxUnstuckAngle / RCM_MAX_DT_ROBOTS);

	float angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
	NORM0_2PI(angle);
	if (fabs(angle) < 30.0 / 180.0*PI)
	{
		stuck = 0;
		return false;
	}
	if (stuck < 100)
	{
		stuck++;
		return false;
	}
	else
	{
		return true;
	}
}

/* Drive during race. */

static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
	system("cls");
		float angle;
		const float SC = 1.0;
		memset(&car->ctrl, 0, sizeof(tCarCtrl));

		if (isStuck(car))
		{
			angle = -RtTrackSideTgAngleL(&(car->_trkPos)) + car->_yaw;
			NORM0_2PI(angle);
			car->ctrl.steer = angle / car->_steerLock;
			car->ctrl.gear = -1;
			car->ctrl.accelCmd = 0.3;
			car->ctrl.brakeCmd = 0.0;
		}
		else
		{
			angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
			NORM_PI_PI(angle); // put the angle back in the range from -PI to PI
			angle -= SC*car->_trkPos.toMiddle / car->_trkPos.seg->width;
			// set up the values to return
			car->ctrl.steer = angle / car->_steerLock;
			car->ctrl.gear = 1; // first gear
			car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
			car->ctrl.brakeCmd = 0.0; // no brakes
		}
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
}








