#ifndef MODULEDRIVER_H
#define MODULEDRIVER_H

// SCANeR specific
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <iterator>
#include <string>
#include <vector>
#include <math.h>
#include <map>
#include <sstream>

#include "ScanerAPI\scanerAPI_DLL_C.h" // SCANeR API. Location: $(STUDIO_PATH)\$(STUDIO_VER)\APIs\include
#include "ScanerAPI\ScanerAPImessagesShm.h"
#include "ScanerAPI\ScanerAPImessagesNetwork.h"

#include "scanerChannels.h"
#include "interfaces.h"

#define EGO_VEHID            0 // Vehicle Id for Ego
#define EGO_SPEED          150 // Speed of Ego in Km/h (only used in follow-the-line)	
#define VEHICLE_NUM_MAX			15 // Number maximum of vehicle in the scene
#define DRIVEN_VEHICLE_NUM   1 // Number of vehicle driven by this module
#define SAMPLE_TIME_MS      10 // Sample time in ms

using namespace std;

enum class SCENARIO {
  INIT = -1, // Initialize process
  FIRST_RECEIVE = 0, // 1st reading of sensors from SCANeR
  FIRST_SEND = 1 // 1st sending of data sent to SCANeR
}; 

struct IVehicleStruct {
  DataInterface* vehicleSetSpeedObligatory;
  DataInterface* vehicleMove;
};

struct DataScaner {
  map <string, double> mapEgoInfo;
  map <string, vector<double>> mapPreviousPath;
  map <string, vector<vector<double>>> mapSensorFusion;
};

void initSCANeR(int argc, char* argv[], int* scenarioStarted, IVehicleStruct* vehicle);
void receiveFromScaner(long frameNumber, int* scenarioStarted, DataScaner& datascaner, vehicleInfo_t* Out_SCA_vehicleInfo, APIProcessState* status);
void send2Scaner(long frameNumber, int* scenarioStarted, IVehicleStruct* vehicle, vehicleInfo_t* Out_SCA_vehicleInfo, APIProcessState* status, bool* processState);
void wrapperFusionScaner(ItfFusionPlanning &myscanerdata, DataScaner &datascaner, long frameNumber);
void wrapperCtrlScaner(double x_ego, double y_ego, double x, double y, int* scenarioStarted, vehicleInfo_t* Out_SCA_vehicleInfo);

#endif /* MODULEDRIVER_H */
