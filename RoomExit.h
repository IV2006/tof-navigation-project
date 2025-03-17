#pragma once
#include <Eigen/Dense>
#include <vector>
#include <set>
#include <string>

using std::string;

class RoomExit
{
public:
    RoomExit(const std::vector<Eigen::Vector3d>& data) : cloudData(data) {}
	std::vector<std::pair<float, Eigen::Vector2d>> getExitPointsByDist(const Eigen::Vector2d& start_point, bool ipcl = true);
    std::vector<Eigen::Vector2d> getExitPointsFromCirc(bool ipcl = true/*, int ver*/);
    std::vector<Eigen::Vector2d> getExitPointsFromCircIntersect(bool ipcl = true/*, int ver*/);

private:
    std::vector<Eigen::Vector3d> cloudData;

	// remove y coordinates, round to x.xx and remove duplicates
    std::vector<Eigen::Vector2d> clampData(const std::vector<Eigen::Vector3d> &path);
	
	// get center of all given points
	Eigen::Vector2d getCenter(const std::vector<Eigen::Vector2d>& points);

    // line vector is AB, point is P
    float distancePointToLine(const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& P);

    double crossProduct(const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C) {
		return (B - A).x() * (C - B).y() - (B - A).y() * (C - B).x();
    }
	
    // given 4 lines (each defined by 2 points), find 4 points defining Quad that is made by lines
    std::vector<Eigen::Vector2d> getQuad(const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& lines, const Eigen::Vector2d& center);

	// data - points, range - dist from line, iter - #of iterations, returns a line on wall (represented by two points on the line)
    std::pair<Eigen::Vector2d, Eigen::Vector2d> RANSAC(const std::vector<Eigen::Vector2d>& data, const float range, const size_t iter);
	
    // stands for Psuedo-CLustering, if not reverse: removes points that are too isolated, if reverse: removes points that are too dense, minP considers each point as a neighbor of itself
    std::vector<Eigen::Vector2d> PCL(const std::vector<Eigen::Vector2d>& data, const float epsilon, const int minP, bool reverse = false);

    // stands for Iterative-Psuedo-CLustering, does PCL iteratively
    std::vector<Eigen::Vector2d> IPCL(const std::vector<Eigen::Vector2d>& data, const int iter, bool reverse = false);

    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> getBestLines(const std::vector<Eigen::Vector2d> &data, const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& lines, const int num_of_lines, const float range);
	
	std::vector<Eigen::Vector2d> getExitFromQuadCirc(const std::vector<Eigen::Vector2d>& data, const std::vector<Eigen::Vector2d>& quad, int numOfP, const int maxP, const float epsilon);

    std::vector<Eigen::Vector2d> getExitFromQuadCircIntersect(const std::vector<Eigen::Vector2d>& data, const std::vector<Eigen::Vector2d>& quad, int numOfP, const int maxP, const float epsilon);

    std::vector<Eigen::Vector2d> getExitPoints(bool ipcl);
	
	std::vector<Eigen::Vector2d> getExitPointsFromAngles(bool ipcl);
	
};
