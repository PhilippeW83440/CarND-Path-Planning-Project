#include "moduleDriver.h"

using namespace std;

void initSCANeR(int argc, char* argv[], int* scenarioStarted, IVehicleStruct* vehicle) {
  Process_Init(argc, argv);
  Com_registerEvent(NETWORK_IVEHICLE_VEHICLEUPDATE);
  Com_registerEvent(NETWORK_ISENSOR_ROADLANESPOINTS);
  Com_registerEvent(NETWORK_ISENSOR_ROADLINESPOINTS);
  Com_registerEvent(NETWORK_ISENSOR_ROADSENSORDETECTEDPOINTS);

  // Initialization of structures and data interfaces
  for (int i = 0; i < DRIVEN_VEHICLE_NUM; ++i) {
    vehicle->vehicleSetSpeedObligatory = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLESETSPEEDOBLIGATORY, i);
    (vehicle++)->vehicleMove = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLEMOVE, i);
  }
  *scenarioStarted = (int)SCENARIO::INIT;
}

// Collect data from SCANeR
void receiveFromScaner(long frameNumber, int* scenarioStarted, DataScaner& datascaner, vehicleInfo_t* Out_SCA_vehicleInfo, APIProcessState* status) {
  map <string, vector<double> > mapPreviousPath;
  map <string, vector<vector<double>>> mapSensorFusion;
  map<string, double> mapEgoInfo;

  Event* event;
  while (event = Com_getNextEvent()) { // Get Event from SCANeR
    EventType evtType = Com_getTypeEvent(event);
    if (evtType == ET_message) { // Data receive
      DataInterface* dEventDataInterf = Com_getMessageEventDataInterface(event);
      std::string msgId = Com_getMessageEventDataStringId(event);

      if (strstr(msgId.c_str(), NETWORK_IVEHICLE_VEHICLEUPDATE)) {
        if (*scenarioStarted == (int)SCENARIO::INIT) *scenarioStarted = (int)SCENARIO::FIRST_RECEIVE;

        vehicleInfo_t* vehicle = Out_SCA_vehicleInfo + Com_getShortData(dEventDataInterf, "vhlId");
        vehicle->COGPos_x = (double)Com_getDoubleData(dEventDataInterf, "pos[0]");
        vehicle->COGPos_y = (double)Com_getDoubleData(dEventDataInterf, "pos[1]");
        vehicle->heading  = (double)Com_getDoubleData(dEventDataInterf, "pos[3]");
        vehicle->speed_x  = (double)Com_getFloatData(dEventDataInterf, "speed[0]");
        vehicle->speed_y  = (double)Com_getFloatData(dEventDataInterf, "speed[1]");
      }
    }
  }

  // Collect ego data
  vehicleInfo_t* ego = Out_SCA_vehicleInfo;
  mapEgoInfo[     "fn"] = (double)frameNumber;
  mapEgoInfo[      "s"] = 0;
  mapEgoInfo[      "d"] = 0;
  mapEgoInfo[  "speed"] = 0;
  mapEgoInfo[      "x"] = ego->COGPos_x;
  mapEgoInfo[      "y"] = ego->COGPos_y;
  mapEgoInfo["heading"] = ego->heading * 360 / (2 * M_PI);
  mapEgoInfo["x_speed"] = ego->speed_x;
  mapEgoInfo["y_speed"] = ego->speed_y;

  // Collect fusion data
  vector<vector<double>> fusion_container;
  for (int vehId = DRIVEN_VEHICLE_NUM; vehId < VEHICLE_NUM_MAX; ++vehId) {
    fusion_container.push_back({ (double)vehId,
      (Out_SCA_vehicleInfo + vehId)->COGPos_x,
      (Out_SCA_vehicleInfo + vehId)->COGPos_y,
      (Out_SCA_vehicleInfo + vehId)->speed_x,
      (Out_SCA_vehicleInfo + vehId)->speed_y,
      (double)0/*s*/, (double)0/*d*/ });
  }
  mapSensorFusion.insert(make_pair("sensor_fusion", fusion_container));

  datascaner.mapEgoInfo = mapEgoInfo;
  datascaner.mapPreviousPath = mapPreviousPath;
  datascaner.mapSensorFusion = mapSensorFusion;

  *status = printProcessState(*status);
  Com_updateInputs(UT_AllData); // SCANeR -> Fusion
}

// Send data to SCANeR (via pseudo control law)
void send2Scaner(long frameNumber, int* scenarioStarted, IVehicleStruct* vehicle, vehicleInfo_t* Out_SCA_vehicleInfo, APIProcessState* status, bool* processState) {
  if (*scenarioStarted == (int)SCENARIO::FIRST_RECEIVE) {
    *scenarioStarted = (int)SCENARIO::FIRST_SEND;

    // Initial speed of all vehicles to zero
    for (int vehId = 0; vehId < DRIVEN_VEHICLE_NUM; ++vehId) {
      DataInterface* vehIdSpeed = (vehicle++)->vehicleSetSpeedObligatory;
      Com_setShortData(vehIdSpeed, "vhlId", vehId);
      Com_setFloatData(vehIdSpeed, "speed", 0);
      Com_setCharData(vehIdSpeed, "state", 1);
      Com_setFloatData(vehIdSpeed, "smoothingTime", 0);
    }
  }
  else if (*scenarioStarted == (int)SCENARIO::FIRST_SEND) {
    for (int vehId = 0; vehId < (int)DRIVEN_VEHICLE_NUM; ++vehId) {
      DataInterface* vehIdMove = (vehicle++)->vehicleMove;
      Com_setShortData(vehIdMove, "vhlId", vehId);
      Com_setDoubleData(vehIdMove, "pos0", Out_SCA_vehicleInfo->nextPos_x);
      Com_setDoubleData(vehIdMove, "pos1", Out_SCA_vehicleInfo->nextPos_y);
      Com_setFloatData(vehIdMove, "h", (float)((Out_SCA_vehicleInfo++)->nextHeading));
    }
  }
  else {
    assert(1 && "SCANeR connection and scenario status unconsistent");
  }

  Com_updateOutputs(UT_AllData);
  *processState = (*status != PS_DEAD);
}

// Wrapper SCANeR fusion -> DPL
void wrapperFusionScaner(ItfFusionPlanning& fusion, DataScaner& datascaner, long frameNumber) {
  CarData car; // ego: x,y,yaw,norm_v(,s,d)
  // PreviousPath previous_path; // SCANeR => 49 pts, Unity => 8 pts in mean
  std::vector<std::vector<double>> sensor_fusion; // other objects: car_id,x,y,vx,vy(,s,d)

  // myscanerdata.fn = (int)frameNumber;
  fusion.car.x = datascaner.mapEgoInfo.find("x")->second;
  fusion.car.y = datascaner.mapEgoInfo.find("y")->second;
  fusion.car.yaw = datascaner.mapEgoInfo.find("heading")->second;
  // fusion.car.speed = datascaner.mapEgoInfo.find("linear_speed")->second;
  // myscanerdata.x_speed = datascaner.mapEgoInfo.find("x_speed")->second;
  // myscanerdata.y_speed = datascaner.mapEgoInfo.find("y_speed")->second;
  // myscanerdata.x_acc   = datascaner.mapEgoInfo.find(  "x_acc")->second;
  // myscanerdata.y_acc   = datascaner.mapEgoInfo.find(  "y_acc")->second;

  vector<vector<double>> temp_sensor_fusion = datascaner.mapSensorFusion.find("sensor_fusion")->second;
  fusion.sensor_fusion.clear();
  for (size_t i = 0; i < temp_sensor_fusion.size(); ++i) {
    fusion.sensor_fusion.push_back(temp_sensor_fusion[i]);
  }
  temp_sensor_fusion.clear();
}

// Wrapper DPL -> SCANeR ctrl
// Emulate simplistic (point to point shift + heading) ctrl law)
void wrapperCtrlScaner(double x_ego, double y_ego, double x, double y, int* scenarioStarted, vehicleInfo_t* Out_SCA_vehicleInfo) {
  if (*scenarioStarted == (int)SCENARIO::FIRST_SEND) {
    double dx = x - x_ego, dy = y - y_ego;
    for (int vehId = 0; vehId < (int)DRIVEN_VEHICLE_NUM; ++vehId) {
      Out_SCA_vehicleInfo->nextPos_x = Out_SCA_vehicleInfo[vehId].COGPos_x + dx;
      Out_SCA_vehicleInfo->nextPos_y = Out_SCA_vehicleInfo[vehId].COGPos_y + dy;
      (Out_SCA_vehicleInfo++)->nextHeading = atan2(dy, dx); // valid for all 4 quadrants (+dx,+dy), (+dx,-dy), (-dx,+dy), (-dx,-dy)
    }
  }
}
