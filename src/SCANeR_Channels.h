#ifndef SCANERCHANNEL_H
#define SCANERCHANNEL_H

#include "moduleDriver.h"
#include "params.h"

struct vehicleInfostruct {
	double linearSpeed;
	double COGPos_x;
	double COGPos_y;
	double COGPos_z;
	double heading;
	double speed_x;
	double speed_y;
	double speed_z;
	double yawRate;
	double accel_x;
	double accel_y;
	double accel_z;

	double nextPos_x;
	double nextPos_y;
	double nextPos_z;
	double nextHeading;
	double nextSpeed_x;
	double nextSpeed_y;
	double nextSpeed_z;
	double nextLinearSpeed;
	double dummy;
};
extern vehicleInfostruct Out_SCA_vehicleInfo[];
extern vehicleInfostruct NextStep_vehicleInfo[];

#endif /* SCANERCHANNEL_H */