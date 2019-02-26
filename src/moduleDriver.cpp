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
void receiveFromScaner(long frameNumber, int* scenarioStarted, DataScaner& datascaner, vehicleInfo_t* Out_SCA_vehicleInfo) {
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
        vehicle->linearSpeed = (double)linearSpeed(vehicle);
        vehicle->yawRate  = (double)Com_getFloatData(dEventDataInterf, "speed[3]");
        vehicle->accel_x  = (double)Com_getFloatData(dEventDataInterf, "accel[0]");
        vehicle->accel_y  = (double)Com_getFloatData(dEventDataInterf, "accel[1]");
      }
    }
  }

  // ?
  for (int vehId = 0; vehId < (int)DRIVEN_VEHICLE_NUM; ++vehId) {
    Out_SCA_vehicleInfo->speed_x = Out_SCA_vehicleInfo[vehId].nextSpeed_x;
    Out_SCA_vehicleInfo->speed_y = Out_SCA_vehicleInfo[vehId].nextSpeed_y;
    Out_SCA_vehicleInfo->linearSpeed = (double)linearSpeed(Out_SCA_vehicleInfo);
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
  mapEgoInfo[    "yaw"] = ego->yawRate;
  mapEgoInfo["x_speed"] = ego->speed_x;
  mapEgoInfo["y_speed"] = ego->speed_y;
  mapEgoInfo["linear_speed"] = ego->linearSpeed;
  mapEgoInfo[  "x_acc"] = ego->accel_x;
  mapEgoInfo[  "y_acc"] = ego->accel_y;

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
}

// Emulate simplistic (point to point shift + heading) ctrl law
void ctrlScaner(double x_ego, double y_ego, double x, double y, int* scenarioStarted, vehicleInfo_t* Out_SCA_vehicleInfo) {
  if (*scenarioStarted == (int)SCENARIO::FIRST_SEND) {
    double dx = x - x_ego, dy = y - y_ego;
    for (int vehId = 0; vehId < (int)DRIVEN_VEHICLE_NUM; ++vehId) {
      Out_SCA_vehicleInfo->nextPos_x = Out_SCA_vehicleInfo[vehId].COGPos_x + dx;
      Out_SCA_vehicleInfo->nextPos_y = Out_SCA_vehicleInfo[vehId].COGPos_y + dy;
      double next_vx = dx / 5.0/*in legacy 5th point was, to rm*/ * ((double)SAMPLE_TIME_MS / 1000.0);
      double next_vy = dy / 5.0 * ((double)SAMPLE_TIME_MS / 1000.0);
      Out_SCA_vehicleInfo->nextSpeed_x = next_vx;
      Out_SCA_vehicleInfo->nextSpeed_y = next_vy;
      Out_SCA_vehicleInfo->nextLinearSpeed = 3.6*sqrt(next_vx*next_vx + next_vy*next_vy);
      (Out_SCA_vehicleInfo++)->nextHeading = atan2(dy, dx); // valid for all 4 quadrants (+dx,+dy), (+dx,-dy), (-dx,+dy), (-dx,-dy)
    }
  }
}

// Send data to SCANeR (via pseudo control law)
void send2Scaner(long frameNumber, int* scenarioStarted, IVehicleStruct* vehicle, vehicleInfo_t* Out_SCA_vehicleInfo) {
  if (*scenarioStarted == (int)SCENARIO::FIRST_RECEIVE) {
    *scenarioStarted = (int)SCENARIO::FIRST_SEND;

    // Initial speed of all vehicles to zero
    for (int vehId = 0; vehId < DRIVEN_VEHICLE_NUM; ++vehId) {
      DataInterface* vehIdSpeed = (vehicle++)->vehicleSetSpeedObligatory;
      Com_setShortData(vehIdSpeed, "vhlId",     vehId);
      Com_setFloatData(vehIdSpeed, "speed",         0);
      Com_setCharData(vehIdSpeed,  "state",         1);
      Com_setFloatData(vehIdSpeed, "smoothingTime", 0);
    }
  } else if (*scenarioStarted == (int)SCENARIO::FIRST_SEND) {    
    for (int vehId = 0; vehId < (int)DRIVEN_VEHICLE_NUM; ++vehId) {
      DataInterface* vehIdMove = (vehicle++)->vehicleMove;
      Com_setShortData( vehIdMove, "vhlId", vehId);
      Com_setDoubleData(vehIdMove,  "pos0", Out_SCA_vehicleInfo->nextPos_x);
      Com_setDoubleData(vehIdMove,  "pos1", Out_SCA_vehicleInfo->nextPos_y);
      Com_setFloatData(vehIdMove,      "h", (float)((Out_SCA_vehicleInfo++)->nextHeading));
    }
  } else {
    assert(1 && "SCANeR connection and scenario status unconsistent");
  }
}

// Wrapper SCANeR fusion -> DPL
void wrapperScaner(ItfFusionPlanning &myscanerdata, DataScaner &datascaner, long frameNumber) {
  CarData car; // ego: x,y,yaw,norm_v(,s,d)
  // PreviousPath previous_path; // SCANeR => 49 pts, Unity => 8 pts in mean
  std::vector<std::vector<double>> sensor_fusion; // other objects: car_id,x,y,vx,vy(,s,d)

  // myscanerdata.fn = (int)frameNumber;
  myscanerdata.car.x     = datascaner.mapEgoInfo.find(           "x")->second;
  myscanerdata.car.y     = datascaner.mapEgoInfo.find(           "y")->second;
  myscanerdata.car.yaw   = datascaner.mapEgoInfo.find(     "heading")->second;
  myscanerdata.car.speed = datascaner.mapEgoInfo.find("linear_speed")->second;
  // myscanerdata.x_speed = datascaner.mapEgoInfo.find("x_speed")->second;
  // myscanerdata.y_speed = datascaner.mapEgoInfo.find("y_speed")->second;
  // myscanerdata.x_acc   = datascaner.mapEgoInfo.find(  "x_acc")->second;
  // myscanerdata.y_acc   = datascaner.mapEgoInfo.find(  "y_acc")->second;

  vector<vector<double>> temp_sensor_fusion = datascaner.mapSensorFusion.find("sensor_fusion")->second;
  myscanerdata.sensor_fusion.clear();
  for (size_t i = 0; i < temp_sensor_fusion.size(); ++i) {
      myscanerdata.sensor_fusion.push_back(temp_sensor_fusion[i]);
  }
  temp_sensor_fusion.clear();
}