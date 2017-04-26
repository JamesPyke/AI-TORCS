#include "driver.h"

const float Driver::maxUnstuckAngle = 30.0 / 180.0*PI;
const float Driver::unstuckTimeLimit = 2.0;
const float Driver::maxUnstuckSpeed = 5.0;
const float Driver::minUnstuckDist = 3.0;
const float Driver::G = 9.81;
const float Driver::fullAccelMargin = 1.0;

const float Driver::shift = 0.9;
const float Driver::shiftMargin = 4.0;

const float Driver::ABSSlip = 0.9;
const float Driver::ABSMinSpeed = 3.0;

const float Driver::TCLSlip = 0.9;
const float Driver::TCLMinSpeed = 3.0;

Driver::Driver(int index)
{
	INDEX = index;
}

void Driver::initTrack(tTrack* t, void *carHandle, void ** carParmHandle, tSituation *s)
{
	track = t;
	*carParmHandle = NULL;
}

void Driver::newRace(tCarElt* car, tSituation *s)
{
	maxUnstuckCount = int(unstuckTimeLimit / RCM_MAX_DT_ROBOTS);
	stuck = 0;
	this->car = car;
	carMass = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0);
	initCA();
	initCW();
	initTCLFilter();
}

void Driver::drive(tSituation *s)
{
	update(s);
	memset(&car->ctrl, 0, sizeof(tCarCtrl));

	if (isStuck())
	{
		car->ctrl.steer = -angle / car->_steerLock;
		car->ctrl.gear = -1;
		car->ctrl.accelCmd = 0.5;
		car->ctrl.brakeCmd = 0.0;
	}
	else
	{
		float steerAngle = angle - car->_trkPos.toMiddle / car->_trkPos.seg->width;
		// set up the values to return
		car->ctrl.steer = steerAngle / car->_steerLock;
		car->ctrl.gear = getGear(); 
		car->ctrl.brakeCmd = filterABS(getBrake());
		if (car->ctrl.brakeCmd == 0.0)
		{
			car->ctrl.accelCmd = filterTCL(getAccel());
		}
		else
		{
			car->ctrl.accelCmd = 0.0;
		}
	}
}

int Driver::pitCommand(tSituation *s)
{
	return ROB_PIT_IM;
}

void Driver::endRace(tSituation *s)
{

}

bool Driver::isStuck()
{
	if (fabs(angle) < maxUnstuckAngle && car->_speed_x < maxUnstuckSpeed && fabs(car->_trkPos.toMiddle) > minUnstuckDist)
	{
		if (stuck > maxUnstuckCount && car->_trkPos.toMiddle*angle < 0.0)
		{
			return true;
		}
	}
	else{
		stuck++;
		return false;
	}
}

void Driver::update(tSituation *s)
{
	trackAngle = RtTrackSideTgAngleL(&(car->_trkPos));
	angle = trackAngle - car->_yaw;
	NORM_PI_PI(angle);
	mass = carMass + car->_fuel;
}

//Compute the allowed speed on the track
float Driver::getAllowedSpeed(tTrackSeg *segment)
{
	if (segment->type == TR_STR)
	{
		return FLT_MAX;
	}
	else
	{
		float mu = segment->surface->kFriction;
		return sqrt((mu*G*segment->radius)/(1.0 - MIN(1.0, segment ->radius*CA*mu/mass)));
	}
}

//Compute the length to the end of the segment
float Driver::getDisToSegEnd()
{
	if (car->_trkPos.seg->type == TR_STR)
	{
		return car->_trkPos.seg->length - car->_trkPos.toStart;
	}
	else
	{
		return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
	}
}

//Compute ideal acceleration
float Driver::getAccel()
{
	float allowedSpeed = getAllowedSpeed(car->_trkPos.seg);
	float gr = car->_gearRatio[car->_gear + car->_gearOffset];
	float rm = car->_enginerpmRedLine;
	if (allowedSpeed > car->_speed_x + fullAccelMargin)
	{
		return 1.0;
	}
	else
	{
		return allowedSpeed / car->_wheelRadius(REAR_RGT)*gr / rm;
	}
}

float Driver::getBrake()
{
	tTrackSeg *segptr = car->_trkPos.seg;
	float currentSpeedSqr = car->_speed_x*car->_speed_x;
	float mu = segptr->surface->kFriction;
	float maxLookAheadList = currentSpeedSqr / (2.0*mu*G);
	float lookAheadList = getDisToSegEnd();
	float allowedSpeed = getAllowedSpeed(segptr);

	if (allowedSpeed < car->_speed_x)
		return 1.0;

	segptr = segptr->next;

	while (lookAheadList < maxLookAheadList)
	{
		allowedSpeed = getAllowedSpeed(segptr);
		if (allowedSpeed < car->_speed_x)
		{
			float allowedSpeedSqr = allowedSpeed*allowedSpeed;
			float brakeList = mass*(currentSpeedSqr - allowedSpeedSqr) / (2.0*(mu*G*mass + allowedSpeedSqr*(CA*mu + CW)));
			if (brakeList > lookAheadList)
			{
				return 1.0;
			}
		}
		lookAheadList += segptr->length;
		segptr = segptr->next;
	}
	return 0;
}

//Calculate gear
int Driver::getGear()
{
	if (car->_gear <= 0)
	{
		return 1;
	}

	float gr_up = car->_gearRatio[car->_gear + car-> _gearOffset];
	float omega = car->_enginerpmRedLine / gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*shift < car->_speed_x)
	{
		return car->_gear + 1;
	}
	else 
	{
		float gr_down = car->_gearRatio[car->_gear + car-> _gearOffset - 1];
		omega = car->_enginerpmRedLine / gr_down;
		if (car->_gear > 1 && omega*wr*shift > car->_speed_x + shiftMargin)
		{
			return car->_gear - 1;
		}
	}
	return car->_gear;
}

//calculate aerodynamic downforce coefficient
void Driver::initCA()
{
	char *WheelSect[4] = { SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL };
	float rearWingArea = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*)NULL, 0.0);
	float rearWingAngle = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*)NULL, 0.0);
	float wingCA = 1.23*rearWingArea*sin(rearWingAngle);

	float CL = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char*)NULL, 0.0) + GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char*)NULL, 0.0);
	float h = 0.0;
	int i;

	for (i = 0; i < 4; i++)
	{
		h += GfParmGetNum(car->_carHandle, WheelSect[i], PRM_RIDEHEIGHT, (char*)NULL, 0.2);
	}

	h*= 1.5; h = h*h; h = h*h; h = 2.0 * exp(-3.0*h);
	CA = h*CL + 4.0*wingCA;
}

//calculate aerodynamic drag coefficient
void Driver::initCW()
{
	float cx = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_CX, (char*)NULL, 0.0);
	float frontArea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char*)NULL, 0.0);
	CW = 0.645*cx*frontArea;
}

//Antilocking filter for brakes
float Driver::filterABS(float brake)
{
	if (car->_speed_x < ABSMinSpeed)
		return brake;

	int i;
	float slip = 0.0;
	for (i = 0; i < 4; i++)
	{
		slip += car->_wheelSpinVel(i) * car->_wheelRadius(i) / car->_speed_x;
	}
	slip = slip / 4.0;

	if (slip < ABSSlip) brake = brake*slip;
		return brake;
}

//TCL filter for accelerator peddle
float Driver::filterTCL(float accel)
{
	if (car->_speed_x < TCLMinSpeed)
		return accel;

	float slip = car->_speed_x / (this->*GET_DRIVEN_WHEEL_SPEED)();

	if (slip < TCLSlip)
	{
		accel = 0.0;
	}
	return accel;
}

// Traction Contorl setup
void Driver::initTCLFilter()
{
	const char *trainType = GfParmGetStr(car->_carHandle, SECT_DRIVETRAIN, PRM_TYPE, VAL_TRANS_RWD);
	if (strcmp(trainType, VAL_TRANS_RWD) == 0)
	{
		GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_RWD;
	}
	else if (strcmp(trainType, VAL_TRANS_FWD) == 0)
	{
		GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_FWD;
	}
	else if (strcmp(trainType, VAL_TRANS_4WD) == 0)
	{
		GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_4WD;
	}
}

//TCL filter for each type of drive
float Driver::filterTCL_RWD()
{
	return(car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) * car->_wheelRadius(REAR_LFT) / 2.0;
}

float Driver::filterTCL_FWD()
{
	return(car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) * car->_wheelRadius(FRNT_LFT) / 2.0;
}

float Driver::filterTCL_4WD()
{
	return(car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) * car->_wheelRadius(FRNT_LFT) / 4.0 
		+ (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) * car->_wheelRadius(REAR_LFT) / 4.0;
}