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

// FIXME: below files to be merged
#include "SCANeR_Channels.h"
#include "DataFromScanner.h"
#include "interfaces.h"

#define EGO_VEHID            0 // Vehicle Id for Ego
#define EGO_SPEED          150 // Speed of Ego in Km/h (only used in follow-the-line)	
#define VEHICLE_NUM_MAX			15 // Number maximum of vehicle in the scene
#define DRIVEN_VEHICLE_NUM   1 // Number of vehicle driven by this module
#define SAMPLE_TIME_MS      10 // Sample time in ms

using namespace std;

struct DataScaner {
  map <string, double> mapEgoInfo;
  map <string, vector<double>> mapPreviousPath;
  map <string, vector<vector<double>>> mapSensorFusion;
};

void Init(int argc, char* argv[]); // 
DataScaner From_SCANeR_Info(long frameNumber);
void SetNewPath_PP(double x_ego, double y_ego, double x, double y);
void To_SCANeR_Info(long frameNumber);
void wrapperScaner(ItfFusionPlanning &myscanerdata, DataScaner &datascaner, long frameNumber);

#endif /* MODULEDRIVER_H */
