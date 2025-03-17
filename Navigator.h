#pragma once
#include "Auxiliary.h"
#include "simulator/simulator.h"
#include "RoomExit/RoomExit.h"
#include <cmath>
#include <fstream>
#include <random>

class Navigator
{
private:
	Simulator& simulator;
	bool doLog;
	Eigen::Vector3d currPos;
	double currAngle;
	bool setNavigatorData;
	
	void circularScan(int angleDiff, int sleepInterval);
	
	std::string reverseCommand(const std::string& command);

	Eigen::Vector3d getCurrentLocation();
	
	Eigen::Vector3d GetCameraForwardDirection();

	void navigateToCenter();

	std::string getTime();
		
	double getCurrentAngle();


public:
	Navigator(Simulator& simulator, bool doLog) : simulator(simulator), doLog(doLog), currAngle(0), currPos{0, 0, 0}, setNavigatorData(false) {}

	void scan(int angleDiff, int sleepInterval);
	std::vector<std::pair<float, Eigen::Vector3d>> getExitPoints();
	void navigateToPoint(Eigen::Vector3d& targetPoint, bool toExit = false);
	void command(string comm);
};
