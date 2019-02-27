#ifndef SCANERCHANNEL_H
#define SCANERCHANNEL_H

struct vehicleInfo_t {
	// SCANeR -> current
	double COGPos_x;
	double COGPos_y;
	double heading;
	double speed_x;
	double speed_y;
  // next -> SCANeR
	double nextPos_x;
	double nextPos_y;
	double nextHeading;
};

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