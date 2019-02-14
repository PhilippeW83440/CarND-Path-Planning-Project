#include "moduleDriver.h"
#include "utils.h"

using namespace std;

struct IVehicleStruct {
  DataInterface* vehicleSetSpeedObligatory;
  DataInterface* vehicleMove;
};

/*********** GLOBAL VARIABLES ***********/
// Variables for DataObj creation // 
DataScaner datascaner;
IVehicleStruct vehicle[VEHICLE_NUM_MAX];
DataInterface* CabToModelCorrectiveInput;
DataInterface* CabToSteeringCorrectiveInput;
int scenarioStarted;
vehicleInfostruct Out_SCA_vehicleInfo[VEHICLE_NUM_MAX];
vehicleInfostruct NextStep_vehicleInfo[VEHICLE_NUM_MAX];

void Init(int argc, char* argv[]) {
  Process_Init(argc, argv);
  Com_registerEvent(NETWORK_IVEHICLE_VEHICLEUPDATE);
  Com_registerEvent(NETWORK_ISENSOR_ROADLANESPOINTS);
  Com_registerEvent(NETWORK_ISENSOR_ROADLINESPOINTS);
  Com_registerEvent(NETWORK_ISENSOR_ROADSENSORDETECTEDPOINTS);

  // Initialization of structures and data interfaces
  for (int i = 0; i < DRIVEN_VEHICLE_NUM; ++i) {
    vehicle[i].vehicleSetSpeedObligatory = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLESETSPEEDOBLIGATORY, i);
    vehicle[i].vehicleMove = Com_declareOutputData(NETWORK_IVEHICLE_VEHICLEMOVE, i);
  }
  scenarioStarted = -1;
}

// Collect data from SCANeR
DataScaner From_SCANeR_Info(long frameNumber) {
  Event* event;

  // Get Event from SCANeR Network
  DataFromScanner dataObj;
  map <string, vector<double> > mapPreviousPath;
  map <string, vector<vector<double>>> mapSensorFusion;
  map<string, double> mapEgoInfo;

  while (event = Com_getNextEvent()) {
    EventType evtType = Com_getTypeEvent(event);
    if (evtType == ET_message) {
      DataInterface* dEventDataInterf = Com_getMessageEventDataInterface(event);
      std::string msgId = Com_getMessageEventDataStringId(event);

      if (strstr(msgId.c_str(), NETWORK_IVEHICLE_VEHICLEUPDATE)) {
        if (scenarioStarted < 0) scenarioStarted = 0;

        short vehId = Com_getShortData(dEventDataInterf, "vhlId");
        Out_SCA_vehicleInfo[vehId].COGPos_x = (double)Com_getDoubleData(dEventDataInterf, "pos[0]");
        Out_SCA_vehicleInfo[vehId].COGPos_y = (double)Com_getDoubleData(dEventDataInterf, "pos[1]");
        Out_SCA_vehicleInfo[vehId].COGPos_z = (double)Com_getDoubleData(dEventDataInterf, "pos[2]");
        Out_SCA_vehicleInfo[vehId].heading = (double)Com_getDoubleData(dEventDataInterf, "pos[3]");
        Out_SCA_vehicleInfo[vehId].speed_x = (double)Com_getFloatData(dEventDataInterf, "speed[0]");
        Out_SCA_vehicleInfo[vehId].speed_y = (double)Com_getFloatData(dEventDataInterf, "speed[1]");
        Out_SCA_vehicleInfo[vehId].speed_z = (double)Com_getFloatData(dEventDataInterf, "speed[2]");
        Out_SCA_vehicleInfo[vehId].linearSpeed = (double)linearSpeed(&Out_SCA_vehicleInfo[vehId]);
        Out_SCA_vehicleInfo[vehId].yawRate = (double)Com_getFloatData(dEventDataInterf, "speed[3]");
        Out_SCA_vehicleInfo[vehId].accel_x = (double)Com_getFloatData(dEventDataInterf, "accel[0]");
        Out_SCA_vehicleInfo[vehId].accel_y = (double)Com_getFloatData(dEventDataInterf, "accel[1]");
        Out_SCA_vehicleInfo[vehId].accel_z = (double)Com_getFloatData(dEventDataInterf, "accel[2]");
      }
    }
  }

  for (int vehId = 0; vehId < (int)DRIVEN_VEHICLE_NUM; vehId++) {
    Out_SCA_vehicleInfo[vehId].speed_x = Out_SCA_vehicleInfo[vehId].nextSpeed_x;
    Out_SCA_vehicleInfo[vehId].speed_y = Out_SCA_vehicleInfo[vehId].nextSpeed_y;
    Out_SCA_vehicleInfo[vehId].speed_z = Out_SCA_vehicleInfo[vehId].nextSpeed_z;
    Out_SCA_vehicleInfo[vehId].linearSpeed = (double)linearSpeed(&Out_SCA_vehicleInfo[vehId]);;
  }

  // Collect ego data
  mapEgoInfo[     "fn"] = { (double)frameNumber };
  mapEgoInfo[      "s"] = { 0 };
  mapEgoInfo[      "d"] = { 0 };
  mapEgoInfo[  "speed"] = { 0 };
  mapEgoInfo[      "x"] = Out_SCA_vehicleInfo[0].COGPos_x;
  mapEgoInfo[      "y"] = Out_SCA_vehicleInfo[0].COGPos_y;
  mapEgoInfo["heading"] = (Out_SCA_vehicleInfo[0].heading) * 360 / (2 * 3.1416);
  mapEgoInfo[    "yaw"] = Out_SCA_vehicleInfo[0].yawRate;
  // mapEgoInfo["z"] = Out_SCA_vehicleInfo[0].COGPos_z;  
  mapEgoInfo["x_speed"] = Out_SCA_vehicleInfo[0].speed_x;
  mapEgoInfo["y_speed"] = Out_SCA_vehicleInfo[0].speed_y;
  // mapEgoInfo["z_speed"] = Out_SCA_vehicleInfo[0].speed_z;
  mapEgoInfo["linear_speed"] = Out_SCA_vehicleInfo[0].linearSpeed;
  mapEgoInfo["x_acc"] = Out_SCA_vehicleInfo[0].accel_x;
  mapEgoInfo["y_acc"] = Out_SCA_vehicleInfo[0].accel_y;
  // mapEgoInfo["z_acc"] = Out_SCA_vehicleInfo[0].accel_z;

  // Collect fusion data
  // double fusion_temp[SIZE] = { 0 }; // car_id, x, y, vx, vy(, s, d)
  
  vector<vector<double>> fusion_container;

  // Information about the cars driven by AI in SCANeR
  vector<double> fusion_data;
  for (int vehId = DRIVEN_VEHICLE_NUM; vehId < VEHICLE_NUM_MAX; ++vehId) {
    if (Out_SCA_vehicleInfo[vehId].COGPos_x == 0) {
      continue;
    }
    fusion_data.push_back(vehId); // car_id
    fusion_data.push_back(Out_SCA_vehicleInfo[vehId].COGPos_x); // x
    fusion_data.push_back(Out_SCA_vehicleInfo[vehId].COGPos_y); // y
    fusion_data.push_back(Out_SCA_vehicleInfo[vehId].speed_x);  // vx
    fusion_data.push_back(Out_SCA_vehicleInfo[vehId].speed_y);  // vy
    fusion_data.push_back(0); // s
    fusion_data.push_back(0); // d

    fusion_container.push_back(fusion_data);
    fusion_data.clear();
  }
  mapSensorFusion.insert(make_pair("sensor_fusion", fusion_container));

  datascaner.mapEgoInfo = mapEgoInfo;
  datascaner.mapPreviousPath = mapPreviousPath;
  datascaner.mapSensorFusion = mapSensorFusion;

  return datascaner;
}

void SetNewPath_PP(double x_ego, double y_ego, double x, double y) {
  if (scenarioStarted == 1) {
    for (int vehId = 0; vehId < (int)DRIVEN_VEHICLE_NUM; vehId++) {
      double deltaPath_x = x - x_ego;
      double deltaPath_y = y - y_ego;

      // Taking care of the angle
      double newHeading;
      if (deltaPath_x > 0 && deltaPath_y > 0) {
        newHeading = atan(deltaPath_y / deltaPath_x); // First quadrant -- no change 
      }
      if (deltaPath_x > 0 && deltaPath_y < 0) {
        newHeading = atan(deltaPath_y / deltaPath_x); // Fourth quadrant -- no change 
      }
      if (deltaPath_x < 0 && deltaPath_y > 0) {
        newHeading = 3.1416 + atan(deltaPath_y / deltaPath_x); // Second quadrant -- 180 - angle 
      }
      if (deltaPath_x < 0 && deltaPath_y < 0) {
        newHeading = -3.1416 + atan(deltaPath_y / deltaPath_x);
      }

      // double newHeading = atan(deltaPath_y / deltaPath_x);
      double deltaPosAbs = sqrt((x - x_ego)*(x - x_ego) + (y - y_ego)*(y - y_ego));
      double offsetPos_x = deltaPosAbs * cos(newHeading);
      double offsetPos_y = deltaPosAbs * sin(newHeading);

      Out_SCA_vehicleInfo[vehId].nextPos_x = Out_SCA_vehicleInfo[vehId].COGPos_x + offsetPos_x;
      Out_SCA_vehicleInfo[vehId].nextPos_y = Out_SCA_vehicleInfo[vehId].COGPos_y + offsetPos_y;
      Out_SCA_vehicleInfo[vehId].nextPos_z = Out_SCA_vehicleInfo[vehId].COGPos_z;
      Out_SCA_vehicleInfo[vehId].nextHeading = newHeading;

      Out_SCA_vehicleInfo[vehId].nextSpeed_x = offsetPos_x / 5 * ((double)SAMPLE_TIME_MS / 1000);
      Out_SCA_vehicleInfo[vehId].nextSpeed_y = offsetPos_y / 5 * ((double)SAMPLE_TIME_MS / 1000);
      Out_SCA_vehicleInfo[vehId].nextSpeed_z = 0;
      Out_SCA_vehicleInfo[vehId].nextLinearSpeed = sqrt(((3.6*Out_SCA_vehicleInfo[vehId].nextSpeed_x) * (3.6*Out_SCA_vehicleInfo[vehId].nextSpeed_x)) + ((3.6*Out_SCA_vehicleInfo[vehId].nextSpeed_y) * (3.6*Out_SCA_vehicleInfo[vehId].nextSpeed_y)));

      Out_SCA_vehicleInfo[vehId].dummy = 0.0;
    }
  }
}

// Send data to SCANeR (via pseudo control law)
void To_SCANeR_Info(long frameNumber) {
  if (scenarioStarted == 0) {
    scenarioStarted = 1;

    for (int i = 0; i < DRIVEN_VEHICLE_NUM; ++i) {
      /* Initial speed of all vehicles to zero */
      Com_setShortData(vehicle[i].vehicleSetSpeedObligatory, "vhlId", i);
      Com_setFloatData(vehicle[i].vehicleSetSpeedObligatory, "speed", 0);
      Com_setCharData(vehicle[i].vehicleSetSpeedObligatory, "state", 1);
      Com_setFloatData(vehicle[i].vehicleSetSpeedObligatory, "smoothingTime", 0);
    }
  } else if (scenarioStarted == 1) {
    for (int vehId = 0; vehId < (int)DRIVEN_VEHICLE_NUM; ++vehId) {
      Com_setShortData(vehicle[vehId].vehicleMove, "vhlId", vehId);
      Com_setDoubleData(vehicle[vehId].vehicleMove, "pos0", Out_SCA_vehicleInfo[vehId].nextPos_x);
      Com_setDoubleData(vehicle[vehId].vehicleMove, "pos1", Out_SCA_vehicleInfo[vehId].nextPos_y);
      Com_setDoubleData(vehicle[vehId].vehicleMove, "pos2", Out_SCA_vehicleInfo[vehId].nextPos_z);
      Com_setFloatData(vehicle[vehId].vehicleMove, "h", (float)Out_SCA_vehicleInfo[vehId].nextHeading);
    }
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