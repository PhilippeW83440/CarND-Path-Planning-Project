#ifndef DATAFROMSCANNER_H
#define DATAFROMSCANNER_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <map>
#include <iterator>
#include <string>
#include <vector>
#include <math.h>

using namespace std;

class DataFromScanner {

/***** Attributes *****/
public:
	double				 m_fn;
	
	double         m_ego_x;
	double         m_ego_y;
	double				 m_ego_z;
	double				 m_ego_heading;

	double         m_ego_speed_x;
	double         m_ego_speed_y;
	double				 m_ego_speed_z;
	double				 m_ego_linear_speed;
	double         m_ego_speed;
	double         m_ego_yaw;

	double         m_ego_acc_x;
	double         m_ego_acc_y;
	double				 m_ego_acc_z;

	double         m_ego_s;
	double         m_ego_d;
	
	vector<double> m_ego_path_front_x; // Still to decide if this is going to be the reference line or if we should take the left - FRONT SENSOR
	vector<double> m_ego_path_front_y;

	vector<double> m_lane_left_x; // The coordinates are absolute with respect to EGO - LEFT LANE
	vector<double> m_lane_left_y;

	vector<double> m_ego_path_back_x; // The coordinates are absolute with respect to EGO - BACK SENSOR
	vector<double> m_ego_path_back_y;

	vector<double> m_prev_path_x; // Previous path data given to the Planner
	vector<double> m_prev_path_y;

	vector<vector<double>> m_sensor_fusion;

/***** Constructor *****/
public:
	DataFromScanner() {};

	DataFromScanner(map <string, double> mapEgoInfo, map <string, vector<double> > mapPreviousPath, map <string, vector<vector<double>>> mapSensorFusion) {
		init(mapEgoInfo, mapPreviousPath, mapSensorFusion);
	}

/***** Mutators *****/
public:
	void init(map<string, double> mapEgoInfo, map <string, vector<double> > mapPreviousPath, map <string, vector<vector<double>>> mapSensorFusion) {
		m_fn = mapEgoInfo.find("fn")->second;
		m_ego_x = mapEgoInfo.find("x")->second;
		m_ego_y = mapEgoInfo.find("y")->second;
		m_ego_z = mapEgoInfo.find("z")->second;
		m_ego_heading = mapEgoInfo.find("heading")->second;

		m_ego_speed_x = mapEgoInfo.find("x_speed")->second;
		m_ego_speed_y = mapEgoInfo.find("y_speed")->second;
		m_ego_speed_z = mapEgoInfo.find("z_speed")->second;
		m_ego_linear_speed = mapEgoInfo.find("linear_speed")->second;
		m_ego_speed = mapEgoInfo.find("speed")->second;
		m_ego_yaw = mapEgoInfo.find("yaw")->second;
		
		m_ego_acc_x = mapEgoInfo.find("x_acc")->second;
		m_ego_acc_y = mapEgoInfo.find("y_acc")->second;
		m_ego_acc_z = mapEgoInfo.find("z_acc")->second;

		m_ego_s = mapEgoInfo.find("s")->second;
		m_ego_d = mapEgoInfo.find("d")->second;

		m_ego_path_front_x = mapPreviousPath.find("ego_path_front_x")->second;
		m_ego_path_front_y = mapPreviousPath.find("ego_path_front_y")->second;

		m_lane_left_x = mapPreviousPath.find("lane_left_x")->second;
		m_lane_left_y = mapPreviousPath.find("lane_left_y")->second;

		m_ego_path_back_x = mapPreviousPath.find("ego_path_back_x")->second;
		m_ego_path_back_y = mapPreviousPath.find("ego_path_back_y")->second;

		// To be done with a setter function // 
		//m_prev_path_x = mapPreviousPath.find("ego_path_front_x")->second;
		//m_prev_path_y = mapPreviousPath.find("ego_path_front_y")->second;


		m_sensor_fusion = mapSensorFusion.find("sensor_fusion")->second;
	}

/***** Accesors *****/

public:
	
	double                 getFn()         const { return m_fn; };

	double                 getEgoX()         const { return m_ego_x; };
	double                 getEgoY()         const { return m_ego_y; };
	double                 getEgoZ()         const { return m_ego_z; };
	double				   getEgoHeading()   const { return m_ego_heading; };

	double                 getEgoXspeed()    const { return m_ego_speed_x; };
	double                 getEgoYspeed()    const { return m_ego_speed_y; };
	double                 getEgoZspeed()    const { return m_ego_speed_z; };
	double                 getEgoLinSpeed()  const { return m_ego_linear_speed; };
	double                 getEgoSpeed()     const { return m_ego_speed; };
	double                 getEgoYaw()       const { return m_ego_heading; }; ///////

	double                 getEgoXacc()      const { return m_ego_acc_x; };
	double                 getEgoYacc()      const { return m_ego_acc_y; };
	double                 getEgoZacc()      const { return m_ego_acc_z; };

	double                 getEgoS()         const { return m_ego_s; };
	double                 getEgoD()         const { return m_ego_d; };
	
	vector<double>         getEgoPathX()    const { return m_ego_path_front_x; };
	vector<double>         getEgoPathY()    const { return m_ego_path_front_y; };

	vector<double>         getLeftLaneX()    const { return m_lane_left_x; };
	vector<double>         getLeftLaneY()    const { return m_lane_left_y; };

	vector<double>         getEgoPathBackX()    const { return m_ego_path_back_x; };
	vector<double>         getEgoPathBackY()    const { return m_ego_path_back_y; };

	vector<double>         getPrevPathX()    const { return m_prev_path_x; };
	vector<double>         getPrevPathY()    const { return m_prev_path_y; };

	vector<vector<double>> getSensorFusion() const { return m_sensor_fusion; };

/***** Mutators *****/
public:
	void setFn(double fn) { m_fn = fn; }
	
	void setEgoX(double ego_x) { m_ego_x = ego_x; }
	void setEgoY(double ego_y) { m_ego_y = ego_y; }
	void setEgoZ(double ego_z) { m_ego_z = ego_z; }
	void setEgoHeading(double ego_heading) { m_ego_heading = ego_heading; }

	void setEgoXspeed(double ego_x_speed) { m_ego_speed_x = ego_x_speed; }
	void setEgoYspeed(double ego_y_speed) { m_ego_speed_y = ego_y_speed; }
	void setEgoZspeed(double ego_z_speed) { m_ego_speed_z = ego_z_speed; }
	void setEgolinSpeed(double ego_linear_speed) { m_ego_linear_speed = ego_linear_speed; }
	void setEgoSpeed(double ego_speed) { m_ego_speed = ego_speed; }
	void setEgoYaw(double ego_yaw) { m_ego_yaw = ego_yaw; }

	void setEgoXacc(double ego_x_acc) { m_ego_acc_x = ego_x_acc; }
	void setEgoYacc(double ego_y_acc) { m_ego_acc_y = ego_y_acc; }
	void setEgoZacc(double ego_z_acc) { m_ego_acc_z = ego_z_acc; }

	void setEgoS(double ego_s) { m_ego_s = ego_s; }
	void setEgoD(double ego_d) { m_ego_d = ego_d; }

	void setPrevPathX(vector<double> prev_path_x) { m_prev_path_x = prev_path_x; }
	void setPrevPathY(vector<double> prev_path_y) { m_prev_path_x = prev_path_y; }

	void setSensorFusion(vector<vector<double>> SensorFusion) { m_sensor_fusion = SensorFusion; }
	

/***** Operators *****/
public:
	DataFromScanner & operator=(DataFromScanner const& data) {
		
		m_fn = data.m_fn;
		
		m_ego_x = data.m_ego_x;
		m_ego_y = data.m_ego_y;
		m_ego_z = data.m_ego_z;
		m_ego_heading = data.m_ego_heading;
		
		m_ego_speed_x = data.m_ego_speed_x;
		m_ego_speed_y = data.m_ego_speed_y;
		m_ego_speed_z = data.m_ego_speed_z;
		m_ego_linear_speed = data.m_ego_linear_speed;
		m_ego_speed = data.m_ego_speed;
		m_ego_yaw = data.m_ego_yaw;

		m_ego_acc_x = data.m_ego_acc_x;
		m_ego_acc_y = data.m_ego_acc_y;
		m_ego_acc_z = data.m_ego_acc_z;

		m_ego_s = data.m_ego_s;
		m_ego_d = data.m_ego_d;
		
		m_ego_path_front_x = data.m_ego_path_front_x;
		m_ego_path_front_y = data.m_ego_path_front_y;

		m_lane_left_x = data.m_lane_left_x;
		m_lane_left_y = data.m_lane_left_y;

		m_ego_path_back_x = data.m_ego_path_back_x;
		m_ego_path_back_y = data.m_ego_path_back_y;

		m_prev_path_x = data.m_prev_path_x;
		m_prev_path_y = data.m_prev_path_y;

		m_sensor_fusion = data.m_sensor_fusion;
		return *this;
	};
};

#endif