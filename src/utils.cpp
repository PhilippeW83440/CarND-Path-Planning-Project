#include "utils.h"

const int quietMode = 1;

double linearSpeed(vehicleInfostruct* vehicleInfo) {
	double result;
	result = 3.6 * sqrt((vehicleInfo->speed_x*vehicleInfo->speed_x) + (vehicleInfo->speed_y*vehicleInfo->speed_y));
	return result;
}

APIProcessState printProcessState(APIProcessState oldStatus) {
	/*! Process manager State **/
	APIProcessState status = Process_GetState();

	if ((oldStatus != status) && !quietMode) {
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
		default:
			break;
		}
	}
	return status;
}