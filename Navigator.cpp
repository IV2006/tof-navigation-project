#include <iostream>
#include <unistd.h>
#include <sys/wait.h>
#include "Navigator/Navigator.h"
#include <math.h>

string Navigator::reverseCommand(const string& comm)
{	
	// switch can't handle strings ):
	if (comm == "forward")
		return "back";
	if (comm == "back")
		return "forward";
	if (comm == "left")
		return "right";
	if (comm == "right")
		return "left";
	if (comm == "cw")
		return "ccw";
	if (comm == "ccw")
		return "cw";
	throw invalid_argument("reverseCommand got invalid command: \"" + comm + "\"");
}


vector<pair<float, Eigen::Vector3d>> Navigator::getExitPoints()
{
	double y = currPos.y();
	vector<Eigen::Vector3d> eigenData;
	vector<pair<float, Eigen::Vector3d>> res;
	auto scanMap = simulator.getCurrentMap();
	int disregardedPoints = 0;
	
	cond_log(doLog, "points:");
	cout << "--------------------------------------------------ALL-POINTS--------------------------------------------------" << endl;
	for (auto &mp: scanMap) {
		if (mp != nullptr && !mp->isBad()) {
			auto vector = ORB_SLAM2::Converter::toVector3d(mp->GetWorldPos());
			if (-2 <= vector.x() && vector.x() <= 2 && -2 <= vector.z() && vector.z() <= 2) {
				cout << vector.x() << " " << vector.y() << " " << vector.z() << "\n";
				eigenData.emplace_back(vector);
			} else {
				disregardedPoints++;
			}
		}
	}
	cout << "--------------------------------------------------ALL-POINTS--------------------------------------------------" << endl;
	cond_log(doLog, "disregardedPoints: " + to_string(disregardedPoints));
	
    RoomExit roomExit(eigenData);
    std::vector<Eigen::Vector2d> exitPoints2d = roomExit.getExitPointsFromCircIntersect();
	cond_log(doLog, "BEFORE ADDING Y VALUE");
	for (const Eigen::Vector2d& exitPoint : exitPoints2d) {
		cond_log(doLog, "NAVIGATOR: EXIT POINT: " + to_string(exitPoint.x()) + ", " + to_string(exitPoint.y()));
	}
	
	for (auto p: exitPoints2d) {
		auto point = Eigen::Vector3d(p.x(), y, p.y());
		res.push_back(make_pair(678, point));
	}
	
	cond_log(doLog, "AFTER ADDING Y VALUE");
	for (const auto& point : res) {
		cond_log(doLog, "NAVIGATOR: EXIT POINT: " + to_string(point.first) + ", .second: " + to_string(point.second.x()) + ", " + to_string(point.second.y()) + ", " + to_string(point.second.z()));
	}
    return res;
}

void Navigator::circularScan(int angleDiff, int sleepInterval)
{	
	string rotCommand = "ccw";
	int currRelativeAngle;
	for (currRelativeAngle = 0; currRelativeAngle < 360; currRelativeAngle += angleDiff) {
		command(rotCommand + " " + to_string(angleDiff));
		usleep(sleepInterval);
	}
	if (currRelativeAngle > 360) {
		command(reverseCommand(rotCommand) + " " + to_string(currRelativeAngle - 360)); // return to original orientation
	}
	cond_log(doLog, string("After circularScan finish, num points so far: " + to_string(simulator.getCurrentMap().size())));
}

Eigen::Vector3d Navigator::getCurrentLocation() { 
	cv::Mat Tcw = simulator.getCurrentLocationSlam();
    // extract rotation (Rcw) and translation (tcw)
    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat Ow = -Rwc * tcw;
    return ORB_SLAM2::Converter::toVector3d(Ow);
}

Eigen::Vector3d Navigator::GetCameraForwardDirection() {
	cv::Mat Tcw = simulator.getCurrentLocationSlam();
    // extract rotation (Rcw)
    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat forward = Rwc.col(2);
    return ORB_SLAM2::Converter::toVector3d(forward);
}


void Navigator::navigateToPoint(Eigen::Vector3d& targetPoint, bool toExit)
{	
	cond_log(doLog, "----------------------------------------------------------------------------------------------------");
	Eigen::Vector3d startingPoint = setNavigatorData ? currPos : getCurrentLocation();
	cond_log(doLog, "Start: " + to_string(startingPoint.x()) + " " + to_string(startingPoint.y()) + " " + to_string(startingPoint.z()));
	cond_log(doLog, "Target: " + to_string(targetPoint.x()) + " " + to_string(targetPoint.y()) + " " + to_string(targetPoint.z()));
	
	auto start_2d = Eigen::Vector2d(startingPoint.x(), startingPoint.z());
	auto target_2d = Eigen::Vector2d(targetPoint.x(), targetPoint.z());
	Eigen::Vector2d curr_dir(cos(currAngle * M_PI / 180), sin(currAngle * M_PI / 180));
	Eigen::Vector2d target_dir = target_2d - start_2d;
	double angleDiff = acos(curr_dir.dot(target_dir) / (curr_dir.norm() * target_dir.norm())) * 180 / M_PI;
	double crossProduct = curr_dir.x() * target_dir.y() - curr_dir.y() * target_dir.x();	
	cond_log(doLog, "curr_dir: " + to_string(curr_dir.x()) + ", " + to_string(curr_dir.y()) + "\ttarget_dir: " + to_string(target_dir.x()) + ", " + to_string(target_dir.y()) + "\tcurrAngle: " + to_string(currAngle) + "\tangleDiff: " + to_string(angleDiff) + "\tcross product: " + to_string(crossProduct));
	angleDiff = crossProduct > 0 ? angleDiff : -angleDiff;
	cond_log(doLog, "angleDiff * sign(crossProduct): " + to_string(angleDiff));
	
	string rotCommand = "ccw " + to_string(angleDiff);
	cond_log(doLog, " rotCommand: " + rotCommand); 
	command(rotCommand);
	sleep(5);
	
	double dist = target_dir.norm();
	
	int failCounter = 0;
	while (toExit) { // keep going forward until we no longer know where we are (hit a wall)
		dist = 0.1;
		string forward_command = "forward " + to_string(dist);
		cond_log(doLog, "forward_command: " + forward_command);
		command(forward_command);
		usleep(100000); // 0.1s
		try {
			auto _ = getCurrentLocation();
		} catch (...) {
			failCounter++;
			if (failCounter >= 3) {
				string back_command = "back 0.6";
				cond_log(doLog, "exiting forward loop, back_command: " + back_command);
				command(back_command);
				break;
			}
		}
	}
	
	if (!toExit) { // return_orientation
		// reverse rotation to keep looking in original direction
		string reverse_rotCommand = "cw " + to_string(angleDiff);
		cond_log(doLog, "reverse_rotCommand: " + reverse_rotCommand);
		command(reverse_rotCommand);
		sleep(5);
	}
	
	cond_log(doLog, "after navigateToPoint: (x, y, z) " + to_string(currPos.x()) + ",  " + to_string(currPos.y()) + ", " + to_string(currPos.z()) + ", currAngle: " + to_string(currAngle));
	cond_log(doLog, "----------------------------------------------------------------------------------------------------");
}

void Navigator::navigateToCenter()
{
	auto scanMap = simulator.getCurrentMap();
	Eigen::Vector3d center(0, 0, 0);
	
	for (auto &mp: scanMap) {
		if (mp != nullptr && !mp->isBad()) {
			center += ORB_SLAM2::Converter::toVector3d(mp->GetWorldPos());
		}
	}
	center /= scanMap.size();

	cond_log(doLog, string("recentering, going to (x, y, z):") + to_string(center.x()) + " " + to_string(center.y()) + " " + to_string(center.z()));
	navigateToPoint(center);
}

string Navigator::getTime()
{
	auto end = chrono::system_clock::now();
    time_t t = chrono::system_clock::to_time_t(end);
	int hour = (t /3600) % 24;
	int minute = (t / 60) % 60;
	int second = t % 60;
	string s_hour = hour < 10 ? "0" + to_string(hour) : to_string(hour);
	string s_minute = minute < 10 ? "0" + to_string(minute) : to_string(minute);
	string s_second = second < 10 ? "0" + to_string(second) : to_string(second);
	
	return s_hour + ":" + s_minute + ":" + s_second;
}


void Navigator::scan(int angleDiff, int sleepInterval)
{
	// algorithm: in each iteration:
	//  	go a bit back to get a wider view range
	//		go a bit to the right and look a bit to the right (cw), revrse by ccw and going left
	//  	go a bit to the left and look a bit to the left (ccw), revrse by cw and going right
	//		go a bit to the front (where we started)
	// for each wall:
	// 		go to the wall until it's hit, back up a bit
	// 		circularScan
	
	cond_log(doLog, string("start initialScan, angleDiff=") + to_string(angleDiff) + ", sleepInterval=" + to_string(sleepInterval));
	
	string rotCommand = "cw";
	string reverse_rotCommand = reverseCommand(rotCommand);
	rotCommand += " " + to_string(angleDiff);
	reverse_rotCommand += " " + to_string(angleDiff);
	
	string move_command = "right";
	string reverse_move_command = reverseCommand(move_command);
	move_command += " 0.5";
	reverse_move_command += " 0.5";
	
	string commands[] = {"back 0.5", 
						 move_command, rotCommand, reverse_rotCommand, reverse_move_command,
						 reverse_move_command, reverse_rotCommand, rotCommand, move_command,
						 "forward 0.5"};

	
	for (int i = 0; i < 360 / angleDiff; i++) {
		cond_log(doLog, "initialScan, iter: " + to_string(i) + ", num points so far: " + to_string(simulator.getCurrentMap().size()));
		for(const auto &comm : commands) {
			command(comm);
		}
		command(rotCommand);
	}
	
	for (int i = 0; i < 4; i++) {
		double dist = 0;
		cond_log(doLog, "going until hitting a wall");
		command("cw 90");
		sleep(5);
		while (true) { // keep going forward until we no longer know where we are (hit a wall)
			dist += 0.1;
			string forward_command = "forward 0.1";
			cond_log(doLog, "forward_command: " + forward_command);
			command(forward_command);
			try {
				auto _ = getCurrentLocation();
			} catch (...) {
				dist -= 2.1; // -2.1 because "back 2.1"
				string back_command = "back 2.1"; // 2.1 and not 2.0 to negate the affect of the current into wall movement
				cond_log(doLog, "exiting forward loop, back_command: " + back_command);
				command(back_command);
				break;
			}
		}
		cond_log(doLog, "hit a wall");
		circularScan(angleDiff * 2, 0);
		string toOrgCommand = "back " + to_string(dist);
		cond_log(doLog, "toOrgCommand: " + toOrgCommand);
		command(toOrgCommand);
	}
	circularScan(angleDiff, sleepInterval);
	
	currPos = getCurrentLocation();
	currPos.x() *= -1;
	currAngle = getCurrentAngle();
	setNavigatorData = true;
	cond_log(doLog, string("After initialScan finish, num points so far: " + to_string(simulator.getCurrentMap().size())));
}


void Navigator::command(string comm)
{
	if (setNavigatorData) {
		istringstream iss(comm);
		string c;
		double value;
		iss >> c;
		string stringValue;
		iss >> stringValue;
		value = std::stod(stringValue);
		cond_log(doLog, "Navigator::command: comm: " + comm + ", value: " + to_string(value) + ", c: " + c);
		// switch can't handle strings ):
		if (c == "forward" || c == "back" || c == "left" || c == "right") {
			Eigen::Vector3d diff_vector; // changed the sign of the z value in diff_vector
			if (c == "forward")
				diff_vector = Eigen::Vector3d(value * cos(currAngle * M_PI / 180), 0, value * sin(currAngle * M_PI / 180));
			else if (c == "back")
				diff_vector = Eigen::Vector3d(-value * cos(currAngle * M_PI / 180), 0, -value * sin(currAngle * M_PI / 180));
			else if (c == "left")
				diff_vector = Eigen::Vector3d(-value * sin(currAngle * M_PI / 180), 0, value * cos(currAngle * M_PI / 180));
			else if (c == "right")
				diff_vector = Eigen::Vector3d(value * sin(currAngle * M_PI / 180), 0, -value * cos(currAngle * M_PI / 180));
			
			currPos += diff_vector;
			cond_log(doLog, "diff_vector: " + to_string(diff_vector.x()) + ", " + to_string(diff_vector.y()) + ", " + to_string(diff_vector.z()));
		} else if (c == "cw" || c == "ccw") {
			value = fmod(fmod(value, 360) + 360, 360); // make angle in range [0, 360)
			if (value > 180) { // make the smaller rotation
				value = 360 - value;
				c = reverseCommand(c);
			}
			if (c == "cw")
				currAngle -= value;
			else
				currAngle += value;

			comm = c + " " + to_string(value);
			currAngle = fmod(fmod(currAngle, 360) + 360, 360); // make angle in range [0, 360)
			cond_log(doLog, "final rot command: " + comm);
		} else {
			throw invalid_argument("Navigator::command got invalud command: \"" + comm + "\"");
		}
	}
	
	simulator.command(comm);
	
	if (setNavigatorData) {
		try {
			Eigen::Vector3d curr_loc = getCurrentLocation();
			Eigen::Vector3d curr_direction = GetCameraForwardDirection();
			double sim_currAngle = atan(curr_direction.z() / curr_direction.x()) * 180.0 / M_PI;
			if (curr_direction.x() < 0)
				sim_currAngle += 180;
			else if (curr_direction.z() < 0) // and curr_direction.x() > 0
				sim_currAngle += 360;
			cond_log(doLog, "SIMULATOR: curr_loc: " + to_string(curr_loc.x()) + ",  " + to_string(curr_loc.y()) + ", " + to_string(curr_loc.z()) + ", curr angle: " + to_string(sim_currAngle));
		} catch (...) {
			cond_log(doLog, "SIMULATOR: curr_loc or currAngle: ERROR");
		}
		cond_log(doLog, "NAVIGATOR: currPos: " + to_string(currPos.x()) + ",  " + to_string(currPos.y()) + ", " + to_string(currPos.z()) + ", curr angle: " + to_string(currAngle));
	}
}

double Navigator::getCurrentAngle() {
	Eigen::Vector3d curr_direction = GetCameraForwardDirection();
	double sim_currAngle = atan(curr_direction.z() / curr_direction.x()) * 180.0 / M_PI;
	if (curr_direction.x() < 0)
		sim_currAngle += 180;
	else if (curr_direction.z() < 0) // and curr_direction.x() > 0
		sim_currAngle += 360;
	return sim_currAngle;
}
