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
#include "driver.h"

#define BUFSIZE 20
#define NBBOTS 10

static const char* botname[NBBOTS] = {
	"Geoff 1", "Geoff 2", "Geoff 3", "Geoff 4", "Geoff 5",
	"Geoff 6", "Geoff 7", "Geoff 8", "Geoff 9", "Geoff 10"
};

static const char* botdesc[NBBOTS] = {
	"Geoff 1", "Geoff 2", "Geoff 3", "Geoff 4", "Geoff 5",
	"Geoff 6", "Geoff 7", "Geoff 8", "Geoff 9", "Geoff 10"
};
static Driver *driver[NBBOTS];

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newRace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static int pitcmd(int index, tCarElt* car, tSituation *s);
static void shutdown(int index);
static int InitFuncPt(int index, void *pt);
static void endRace(int index, tCarElt *car, tSituation *s);

///* Module entry point */
extern "C" int robot_base(tModInfo *modInfo)
{
	char buffer[BUFSIZE];
	int i;

	/* clear all structures */
	memset(modInfo, 0, NBBOTS * sizeof(tModInfo));

	for (i = 0; i < NBBOTS; i++) {
		modInfo[i].name = strdup(botname[i]);  /* name of the module (short) */
		modInfo[i].desc = strdup(botdesc[i]);          /* description of the module (can be long) */
		modInfo[i].fctInit = InitFuncPt;  /* init function */
		modInfo[i].gfId = ROB_IDENT;   /* supported framework version */
		modInfo[i].index = i;           /* indices from 0 to 9 */
	}
	return 0;
}

/* Module interface initialization. */
static int InitFuncPt(int index, void *pt)
{
	tRobotItf *itf = (tRobotItf *)pt;

	/* create robot instance for index */
	driver[index] = new Driver(index);
	itf->rbNewTrack = initTrack;	/* Give the robot the track view called */
	itf->rbNewRace = newRace;		/* Start a new race */
	itf->rbDrive = drive;		/* Drive during race */
	itf->rbPitCmd = pitcmd;		/* Pit commands */
	itf->rbEndRace = endRace;		/* End of the current race */
	itf->rbShutdown = shutdown;		/* Called before the module is unloaded */
	itf->index = index;		/* Index used if multiple interfaces */
	return 0;
}

/* Called for every track change or new race. */
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
	driver[index]->initTrack(track, carHandle, carParmHandle, s);
}

/* Start a new race. */
static void newRace(int index, tCarElt* car, tSituation *s)
{
	driver[index]->newRace(car, s);
}

/* Drive during race. */
static void drive(int index, tCarElt* car, tSituation *s)
{
	driver[index]->drive(s);
}

/* Pitstop callback */
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
	return driver[index]->pitCommand(s);
}

/* End of the current race */
static void endRace(int index, tCarElt *car, tSituation *s)
{
	driver[index]->endRace(s);
}

/* Called before the module is unloaded */
static void shutdown(int index)
{
	delete driver[index];
}








