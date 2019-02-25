#ifndef SCANERCHANNEL_H
#define SCANERCHANNEL_H

#include "moduleDriver.h"
#include "params.h"

extern vehicleInfo_t Out_SCA_vehicleInfo[];
extern vehicleInfo_t NextStep_vehicleInfo[];

struct vehicleInfo_t {
	// Current
	double COGPos_x;
	double COGPos_y;
	double heading;
	double speed_x;
	double speed_y;
	double yawRate;
  double linearSpeed;
	double accel_x;
	double accel_y;
  // Next
	double nextPos_x;
	double nextPos_y;
	double nextHeading;
	double nextSpeed_x;
	double nextSpeed_y;
	double nextLinearSpeed;
};

inline double linearSpeed(vehicleInfo_t* vehicleInfo) {
  return 3.6 * sqrt((vehicleInfo->speed_x*vehicleInfo->speed_x) + (vehicleInfo->speed_y*vehicleInfo->speed_y));
}

inline APIProcessState printProcessState(APIProcessState oldStatus) {
  APIProcessState status = Process_GetState();

  if (oldStatus != status) {
    switch (status) {
    case PS_DAEMON:
      std::cout << " SCANER API STATUS : DAEMON " << std::endl;
      break;
    case PS_DEAD:
      std::cout << " SCANER API STATUS : DEAD " << std::endl;
      break;
    case PS_LOADED:
      std::cout << " SCANER API STATUS : LOADED " << std::endl;
      break;
    case PS_PAUSED:
      std::cout << " SCANER API STATUS : PAUSED " << std::endl;
      break;
    case PS_READY:
      std::cout << " SCANER API STATUS : READY " << std::endl;
      break;
    case PS_RUNNING:
      std::cout << " SCANER API STATUS : RUNNING " << std::endl;
      break;
    default:;
    }
  }
  return status;
}

#endif /* SCANERCHANNEL_H */