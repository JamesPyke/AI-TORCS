#pragma once
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

class Driver
{

public:

	Driver(int index);

	void initTrack(tTrack* t, void *carHandle, void ** carParmHandle, tSituation *s);
	void newRace(tCarElt* car, tSituation *s);
	void drive(tSituation *s);
	int pitCommand(tSituation *s);
	void endRace(tSituation *s);

	float getAllowedSpeed(tTrackSeg *segment);
	float getAccel();
	float getDisToSegEnd();
	float getBrake();
	float filterABS(float brake);

	int getGear();

	void initCA();
	void initCW();

	float mass;
	tCarElt* car;
	float carMass;
	float CA;
	float CW;

	float filterTCL(float accel);
	float filterTCL_RWD();
	float filterTCL_FWD();
	float filterTCL_4WD();

	void initTCLFilter();
	float(Driver::*GET_DRIVEN_WHEEL_SPEED)();
	static const float TCLSlip;
	static const float TCLMinSpeed;

private:

	bool isStuck();
	void update(tSituation *s);
	int stuck;
	float trackAngle;
	float angle;
	int maxUnstuckCount;
	int INDEX;

	static const float G;
	static const float fullAccelMargin;

	static const float maxUnstuckAngle;
	static const float unstuckTimeLimit;
	static const float maxUnstuckSpeed;
	static const float minUnstuckDist;

	static const float shift;
	static const float shiftMargin;
	static const float ABSSlip;
	static const float ABSMinSpeed;

	tTrack* track;
};

