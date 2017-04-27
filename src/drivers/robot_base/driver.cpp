#include "driver.h"

const float Driver::maxUnstuckAngle = 30.0 / 180.0*PI;
const float Driver::unstuckTimeLimit = 3.0;
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

const float Driver::witdthDiv = 4.0;

Driver::Driver(int index)
{
	INDEX = index;
}
//initialize track
void Driver::initTrack(tTrack* t, void *carHandle, void ** carParmHandle, tSituation *s)
{
	track = t;
	*carParmHandle = NULL;
}
//initialize new race parameters
void Driver::newRace(tCarElt* car, tSituation *s)
{
	//set unstuck count
	maxUnstuckCount = int(unstuckTimeLimit / RCM_MAX_DT_ROBOTS);
	//reset stuck count
	stuck = 0;
	this->car = car;
	//set car mass
	carMass = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0);
	//initialize down force coefficient
	initCA();
	//initialize drag coefficient
	initCW();
	//initialize traction control
	initTCLFilter();
}
//Base drive function
void Driver::drive(tSituation *s)
{
	update(s);
	memset(&car->ctrl, 0, sizeof(tCarCtrl)); 
	//Check if car is stuck
	if (isStuck()) 
	{
		car->ctrl.steer = -angle / car->_steerLock;
		car->ctrl.gear = -1;
		car->ctrl.accelCmd = 0.5;
		car->ctrl.brakeCmd = 0.0;
	}
	//set steering and update drive method
	else
	{
		float steerAngle = angle - car->_trkPos.toMiddle / car->_trkPos.seg->width;
		// set up the values to return
		car->ctrl.steer = steerAngle / car->_steerLock;
		car->ctrl.gear = getGear(); 
		//filter braking through ABS function and brake accordingly
		car->ctrl.brakeCmd = filterABS(getBrake());
		if (car->ctrl.brakeCmd == 0.0)
		{
			//filter traction contorl parameters and track parameters and adjust acceleration accordingly
			car->ctrl.accelCmd = filterTCL(filterTrk(getAccel()));
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
//Base stuck function
bool Driver::isStuck()
{
	//Check the angle of the car in relation to the track and direct the car accordingly
	if (fabs(angle) > maxUnstuckAngle && car->_speed_x < maxUnstuckSpeed && fabs(car->_trkPos.toMiddle) > minUnstuckDist)
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
//Update driver function
void Driver::update(tSituation *s)
{
	//update the angle of the track frame by frame
	trackAngle = RtTrackSideTgAngleL(&(car->_trkPos));
	//set the cars angle according to the track angle
	angle = trackAngle - car->_yaw;
	//normalize the angle
	NORM_PI_PI(angle);
	//update the car mass according to the fuel
	mass = carMass + car->_fuel;
}

//calculate the allowed speed on the track
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

//calculate the length to the end of the segment
float Driver::getDisToSegEnd()
{
	//check the type of the segment
	if (car->_trkPos.seg->type == TR_STR)
	{
		//return the segment type and lenght till end
		return car->_trkPos.seg->length - car->_trkPos.toStart;
	}
	else
	{
		return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
	}
}

//calculate ideal acceleration
float Driver::getAccel()
{
	//check the track segment and set an allowed speed
	float allowedSpeed = getAllowedSpeed(car->_trkPos.seg);
	//set the gear offset 
	float gr = car->_gearRatio[car->_gear + car->_gearOffset];
	//set rm to cars rpm
	float rm = car->_enginerpmRedLine;
	//set speed according to allowed speed
	if (allowedSpeed > car->_speed_x + fullAccelMargin)
	{
		return 1.0;
	}
	else
	{
		return allowedSpeed / car->_wheelRadius(REAR_RGT)*gr / rm;
	}
}
//Base brake function
float Driver::getBrake()
{
	//set track segment to cars current segment
	tTrackSeg *segptr = car->_trkPos.seg;
	//get current speed of car squared 
	float currentSpeedSqr = car->_speed_x*car->_speed_x;
	//get segments surface friction
	float mu = segptr->surface->kFriction;
	float maxLookAheadList = currentSpeedSqr / (2.0*mu*G);
	float lookAheadList = getDisToSegEnd();
	float allowedSpeed = getAllowedSpeed(segptr);
	//Check cars speed
	if (allowedSpeed < car->_speed_x)
		return 1.0;

	segptr = segptr->next;

	while (lookAheadList < maxLookAheadList)
	{
		allowedSpeed = getAllowedSpeed(segptr);
		if (allowedSpeed < car->_speed_x)
		{
			//get allowed speed squared
			float allowedSpeedSqr = allowedSpeed*allowedSpeed;
			//check the speed incase of braking
			float brakeList = mass*(currentSpeedSqr - allowedSpeedSqr) / (2.0*(mu*G*mass + allowedSpeedSqr*(CA*mu + CW)));
			if (brakeList > lookAheadList)
			{
				return 1.0;
			}
		}
		//looks for next segment
		lookAheadList += segptr->length;
		segptr = segptr->next;
	}
	return 0;
}

//Calculate gear
int Driver::getGear()
{	
//check if car is in neutral 
	if (car->_gear <= 0)
	{
		return 1;
	}
	//gets gear up conditions
	float gr_up = car->_gearRatio[car->_gear + car-> _gearOffset];
	//sets gear up condition
	float omega = car->_enginerpmRedLine / gr_up;
	float wr = car->_wheelRadius(2);
	//gets the current and next gear to enable gear up
	if (omega*wr*shift < car->_speed_x)
	{
		return car->_gear + 1;
	}
	else 
	{
		//gets gear down conditions
		float gr_down = car->_gearRatio[car->_gear + car-> _gearOffset - 1];
		//sets gear down conditions
		omega = car->_enginerpmRedLine / gr_down;
		//gets the current and next gear to enable gear down
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
	//Create array for each wheel section
	char *WheelSect[4] = { SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL };
	//get rear wing area
	float rearWingArea = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*)NULL, 0.0);
	//get rear wing angle
	float rearWingAngle = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*)NULL, 0.0);
	float wingCA = 1.23*rearWingArea*sin(rearWingAngle);
	//get ground effect coefficient
	float CL = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char*)NULL, 0.0) + GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char*)NULL, 0.0);
	float h = 0.0;
	int i;
	//calculate the ffect for distance of the car to the track
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
	//use aerodynamics of the car and to front area of the can to calculate drag coefficient
	CW = 0.645*cx*frontArea;
}

//Antilocking filter for brakes
float Driver::filterABS(float brake)
{
	//check spped for ABS
	if (car->_speed_x < ABSMinSpeed)
		return brake;

	int i;
	float slip = 0.0;
	//Check wheel velocity for brake parameters
	for (i = 0; i < 4; i++)
	{
		slip += car->_wheelSpinVel(i) * car->_wheelRadius(i) / car->_speed_x;
	}
	slip = slip / 4.0;
	//check if slipping to enable ABS
	if (slip < ABSSlip) brake = brake*slip;
		return brake;
}

//TCL filter for accelerator peddle
float Driver::filterTCL(float accel)
{
	//Check speed to enable traction control
	if (car->_speed_x < TCLMinSpeed)
		return accel;

	float slip = car->_speed_x / (this->*GET_DRIVEN_WHEEL_SPEED)();
	//Check parameters to enable traction control
	if (slip < TCLSlip)
	{
		accel = 0.0;
	}
	return accel;
}

// Traction Contorl setup for all drive types
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
//Additional checks for track correction
float Driver::filterTrk(float accel)
{
	tTrackSeg* seg = car->_trkPos.seg;

	if (car->_speed_x < maxUnstuckSpeed) 
		return accel;

	if (seg->type == TR_STR)
	{
		float tm = fabs(car->_trkPos.toMiddle);
		float w = seg->width / witdthDiv;

		if (tm > w)
		{
			return 0.0;
		}
		else
		{
			return accel;
		}
	}
	else
	{
		float sign = (seg->type == TR_RGT) ? -1 : 1;
		if (car->_trkPos.toMiddle*sign > 0.0)
		{
			return accel;
		}
		else
		{
			float tm = fabs(car->_trkPos.toMiddle);
			float w = seg->width / witdthDiv;
			if (tm > w)
			{
				return 0.0;
			}
			else
			{
				return accel;
			}
		}
	}
}