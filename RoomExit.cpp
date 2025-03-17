#include "RoomExit/RoomExit.h"
#include <random>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <fstream>
#include <algorithm>

std::vector<Eigen::Vector2d> RoomExit::clampData(const std::vector<Eigen::Vector3d>& data)
{
    //remove y coordinate and round to x.xx	
    std::vector<Eigen::Vector2d> res;
	res.reserve(data.size());
	std::transform(data.begin(), data.end(), std::back_inserter(res), [](const Eigen::Vector3d& p) {
		return Eigen::Vector2d(round(p.x() * 100.0) / 100.0, round(p.z() * 100.0) / 100.0);
	});
    
	// remove duplicates
	std::sort(res.begin(), res.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return a.norm() < b.norm();
    });
    auto last = std::unique(res.begin(), res.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return a.isApprox(b);
    });
    res.erase(last, res.end());
    return res;
}

Eigen::Vector2d RoomExit::getCenter(const std::vector<Eigen::Vector2d>& points)
{
    //average points
	Eigen::Vector2d centerPoint = Eigen::Vector2d(0 ,0);
	for(const auto& p : points) {
		centerPoint += p;
	}
	centerPoint /= points.size();
	return centerPoint;
}

float RoomExit::distancePointToLine(const Eigen::Vector2d &A, const Eigen::Vector2d &B, const Eigen::Vector2d &P)
{
    //math and stuff
    Eigen::Vector2d AB = B - A, AP = P - A;
    float crossProduct2d = AB.x() * AP.y() - AB.y() * AP.x(), lineLength = std::max(AB.norm(), 1e-6);
    return std::abs(crossProduct2d) / lineLength;
}

std::vector<Eigen::Vector2d> RoomExit::getQuad(const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &lines, const Eigen::Vector2d& center)
{
    // find all 6 intersection points
	float tempDet;
    std::vector<std::pair<Eigen::Vector2d, float>> intersectionPoints;
	std::vector<float> dist_from_center;
    for (size_t i = 0; i < lines.size(); i++) {
        for (size_t j = i + 1; j < lines.size(); j++) {
            std::pair<Eigen::Vector2d, Eigen::Vector2d> line1 = lines[i];
            std::pair<Eigen::Vector2d, Eigen::Vector2d> line2 = lines[j];
            Eigen::Vector2d p1 = line1.first;
            Eigen::Vector2d p2 = line1.second;
            Eigen::Vector2d p3 = line2.first;
            Eigen::Vector2d p4 = line2.second;
			Eigen::Vector2d point(-1000, -1000);
            Eigen::Matrix2d temp;
            temp << (p2 - p1).x(), (p3 - p4).x(),
                (p2 - p1).y(), (p3 - p4).y();
			
			if((tempDet = std::abs(temp.determinant())) >= 1e-6) { // if det is close enough to 0, the lines are essentially parallel
                Eigen::Vector2d sol = temp.fullPivLu().solve(p3 - p1);
				point = p1 + sol.x() * (p2 - p1);
                intersectionPoints.push_back({point, (center - point).squaredNorm()});
            }
			std::cout << "line " << i << " line " << j << " det: " << tempDet << " point: " << point.x() << " " << point.y() << std::endl;
        }
    }
	
	// 4 lines representing a room have at most 6 intersection points, the 4 that define the quad are the closest to the center
	std::sort(intersectionPoints.begin(), intersectionPoints.end(), [&](const std::pair<Eigen::Vector2d, float>& a, const std::pair<Eigen::Vector2d, float>& b)
    {
       return (a.second < b.second); 
    });
	std::vector<Eigen::Vector2d> res = std::vector<Eigen::Vector2d>({intersectionPoints[0].first, intersectionPoints[1].first, intersectionPoints[2].first, intersectionPoints[3].first});
	
	// sort in anticlockwise manner TODO: this is around the (0, 0), not the real center
	auto cx = center.x();
	auto cy = center.y();
	std::sort(res.begin(), res.end(), [&](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
       return (atan2(a.y() - cy, a.x() - cx) > atan2(b.y() - cy, b.x() - cx)); 
    });
	return res;
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> RoomExit::RANSAC(const std::vector<Eigen::Vector2d> &data, const float range, const size_t iter)
{
	size_t size = data.size();
    static std::random_device rd;
    static std::mt19937 rng(rd());
    static std::uniform_int_distribution<int> dist(0, size - 1);

    // for finding best point
    std::pair<Eigen::Vector2d, Eigen::Vector2d> bestPoints;
	int mostPoints = 0, count;
	float rangeSquared = range * range;

    for (size_t i = 0; i < iter; i++) {
        // sample 2 diff points
        int index1 = dist(rng);
        int index2;
        do {
            index2 = dist(rng);
        } while (((data[index1] - data[index2]).squaredNorm() < rangeSquared)); // this also gurentees index1 != index2
        Eigen::Vector2d point1 = data[index1];
        Eigen::Vector2d point2 = data[index2];

        // count close points
        count = 0;
        for (size_t j = 0; j < size; j++) {
		    if (distancePointToLine(point1, point2, data[j]) <= range) {
                count++;
            }
        }

        if (count > mostPoints) {
            bestPoints = std::pair<Eigen::Vector2d, Eigen::Vector2d>(point1, point2);
            mostPoints = count;
        }
    }
	
	std::cout << "line has " << count << "close points" << std::endl;
    return bestPoints;
}

std::vector<Eigen::Vector2d> RoomExit::PCL(const std::vector<Eigen::Vector2d>& data, const float epsilon, const int minP, const bool reverse)
{
    std::vector<Eigen::Vector2d> filteredData;
    float epsilonSquared = epsilon * epsilon;
	
    for(const auto& p: data) {
        int count = 0;
        for(const auto& q: data) {
            float dist = (p - q).squaredNorm();
            if(dist <= epsilonSquared) {
                if(++count >= minP) {
                    break;
                }
			}
        }
        if(reverse == (count < minP)) {
			filteredData.push_back(p);
        }
    }
    return filteredData;
}

std::vector<Eigen::Vector2d> RoomExit::IPCL(const std::vector<Eigen::Vector2d>& data, const int iter, bool reverse)
//stands for Iterative-Psuedo-CLustering
{
    //hyperparameters
    const float epsilon = 0.01f;
    const int minP = reverse ? 25 : 3;

	std::vector<Eigen::Vector2d> filteredData = data;
    for(int i = 0; i < iter; i++) {
        filteredData = this->PCL(filteredData, epsilon, minP, reverse);
    }
    return filteredData;
}

std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> RoomExit::getBestLines(const std::vector<Eigen::Vector2d> &data,
 const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& lines, const int num_of_lines, const float range)
{
	std::cout << "RoomExit::getBestLines, data.size(): " << data.size() << std::endl;
	
    std::vector<std::pair<int, int>> scores;
    for(size_t i = 0; i < lines.size(); i++)
    {
        // get all points close to the line
        std::vector<Eigen::Vector2d> lineData;
        for(const Eigen::Vector2d& point: data)
        {
            if (distancePointToLine(lines[i].first, lines[i].second, point) <= range) {
                lineData.push_back(point);
			}
        }
		
        // RPCL
		std::cout << "line " << i << ": lineData.size() before RPCL:" << lineData.size() << std::endl;
        lineData = this->IPCL(lineData, 1, true);
        // score is num of points left after RPCL
        scores.push_back({lineData.size(),i});
		std::cout << "line " << i << ": lineData.size(): after RPCL" << lineData.size() << std::endl;
    }
	
    // sort lines by score
    std::sort(scores.begin(),scores.end(), [&](const std::pair<int, int>& a, const std::pair<int, int>& b)
    {
       return (a.first > b.first); 
    });
    
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> bestLines;
    std::cout << "best lines: " << std::endl;
    for(int i = 0; i < num_of_lines && i < scores.size(); i++) {
		std::pair<Eigen::Vector2d, Eigen::Vector2d> line = lines[scores[i].second];
		std::cout << line.first.x() << " " << line.first.y() << ",\t" << line.second.x() << " " << line.second.y() << std::endl;
        bestLines.push_back(line);
    }
	
    return bestLines;
}

std::vector<Eigen::Vector2d> RoomExit::getExitFromQuadCirc(const std::vector<Eigen::Vector2d>& data, const std::vector<Eigen::Vector2d>& quad, int numOfP, const int maxP, const float epsilon)
{
    //calc num of points for each side by length of sides of quad and it's circumfrence
    std::vector<float> quadNorms; //calc lengths
    float circ = 0;
    for(int i = 0; i < 4; i++) {
        float iNorm = (quad[(i + 1) % 4] - quad[i]).norm();
        quadNorms.push_back(iNorm);

        circ += iNorm;
    }
    std::vector<int> numOfPointsPerSide(4); //calc num of points
    for(int i = 0; i < 4; i++) {
        numOfPointsPerSide[i] = (quadNorms[i]/circ) * numOfP;
    }
    //get exit points
    std::vector<Eigen::Vector2d> exitPoints;

    for(int i = 0; i < 4; i++) {
        int iNext = (i + 1) % 4;
        for(int j = 0; j < numOfPointsPerSide[i]; j++) {
            Eigen::Vector2d point = quad[i] + (quad[iNext] - quad[i])*(j/(float)numOfPointsPerSide[i]); // equally distributed points on edge
            int count = 0; //counter for num of points in epsilon neighbourhood of point
            for(const Eigen::Vector2d& p: data) { //check if counter is >= maxP, if so add point to exitPoints
            
                float temp = (p - point).squaredNorm();
                if(0 < temp && temp < epsilon*epsilon) {
                    count++;
                } if(count > maxP) {
                    break;
                }
            }
            if(count <= maxP) {
                exitPoints.push_back(point);
            }
        }
    }
    return exitPoints;
}

// returns 2 Eigen::Vector2d objects: the first is a point on the wall where the door is, the second is the exit
std::vector<Eigen::Vector2d> RoomExit::getExitFromQuadCircIntersect(const std::vector<Eigen::Vector2d>& data, const std::vector<Eigen::Vector2d>& quad, int numOfP, const int maxP, const float epsilon)
{
    //redundant calculations and code but whatever
    //calc num of points for each side by length of sides of quad and it's circumfrence
    std::vector<float> quadNorms; //calc lengths
    float circ = 0;
    for(int i = 0; i < 4; i++) {
        float iNorm = (quad[(i + 1) % 4] - quad[i]).norm();
        quadNorms.push_back(iNorm);

        circ += iNorm;
		std::cout << "norm[i=" << i << "]=" << iNorm << std::endl;
    }
	std::cout << "circ=" << circ << std::endl;
	
    std::vector<int> numOfPointsPerSide(4); //calc num of points
    for(int i = 0; i<4; i++) {
        numOfPointsPerSide[i] = (quadNorms[i]/circ) * numOfP;
		std::cout << "numOfPointsPerSide[i=" << i << "]=" << numOfPointsPerSide[i] << std::endl;
    }


    //get points on llines that have small amount of neighbours
    std::vector<Eigen::Vector2d> linePoints;
    
    for(int i = 0; i < 4; i++) {
        int iNext = (i + 1) % 4;
        for(int j = 0; j < numOfPointsPerSide[i]; j++) {
            Eigen::Vector2d point = quad[i] + (quad[iNext] - quad[i])*(j/(float)numOfPointsPerSide[i]); // equally distributed points on edge
            int count = 0; //counter for num of points in epsilon neighbourhood of point
            for(const Eigen::Vector2d& p: data) { //check if counter is >= maxP, if so add point to linePoints
            
                float temp = (p - point).squaredNorm();
                if(0 < temp && temp < epsilon*epsilon) {
                    count++;
                } if(count > maxP) {
                    break;
				}
            }
            if(count <= maxP) {
                linePoints.push_back(point);
            }
        }
    }

    //get exit point
    size_t counters[4] = {0}; //counter for each point in quad

    for(int i = 0; i < 4; i++) {
        int iNext = (i + 1) % 4;
        size_t countP = 0;
        for(int j = 0; j < numOfPointsPerSide[i]; j++) {
            Eigen::Vector2d point = quad[i] + (quad[iNext] - quad[i])*(j/(float)numOfPointsPerSide[i]); // equally distributed points on edge
            int count = 0; //counter for num of points in epsilon neighbourhood of point
            for(const Eigen::Vector2d& p: data) { //check if counter is >= maxP, if so add point to exitPoints
            
                float temp = (p - point).squaredNorm();
                if(0 < temp && temp < epsilon*epsilon) {
                    count++;
                } if(count > maxP) {
                    break;
				}
            }
            if(count <= maxP){
                for(const Eigen::Vector2d& p: linePoints) { //check if counter is >= maxP, if so add point to exitPoints
                
                    float temp = (p - point).squaredNorm();
                    if(0 < temp && temp < epsilon*epsilon) {
                        countP++;
                    }
                }
            }
        }
        counters[i] += countP;
        counters[iNext] += countP;
    }
	
	std::cout << "--------------------------------------------------EDGE-POINTS--------------------------------------------------" << std::endl;
	for (auto p: linePoints) {
		std::cout << p.x() << " " << p.y() << std::endl;
	}
	std::cout << "--------------------------------------------------EDGE-POINTS--------------------------------------------------" << std::endl;
	
    size_t maxCounterVal = 0;
    size_t maxCounterIndex;
    for(int i = 0; i < 4; i++) {
		std::cout << "counter[i=" << i << "]=" << counters[i] << ", quad=" << quad[i].x() << ", " << quad[i].y() << std::endl;
        if(counters[i] > maxCounterVal)
        {
            maxCounterVal = counters[i];
            maxCounterIndex = i;
        }
    }
	std::cout << "maxCounterIndex=" << maxCounterIndex << ", maxCounterVal=" << maxCounterVal << std::endl;
	std::vector<Eigen::Vector2d> res;
	
	res.push_back((quad[(maxCounterIndex + 1) % 4] + quad[maxCounterIndex]) / 2.0); // point on door wall, middle between of the line
	res.push_back(quad[maxCounterIndex]); // exit point
	std::cout << "--------------------------------------------------EXIT-POINTS--------------------------------------------------" << std::endl;
	std::cout << quad[maxCounterIndex].x() << " " << quad[maxCounterIndex].y() << std::endl;
	std::cout << "--------------------------------------------------EXIT-POINTS--------------------------------------------------" << std::endl;
    return res;
}

std::vector<Eigen::Vector2d> RoomExit::getExitPointsFromCirc(bool ipcl)
{
    // hyperparameters
    // for RANSAC
    const float range = 0.04;
    const int iter = 2000;
    // for get exit point from quad later...
    const int numOfP = 2000;
    const int maxP = 5;
    const float epsilon = 0.1; // set the epsilon to be lower then when we do dbscan


    //remove y dimension
    std::vector<Eigen::Vector2d> data_2d = clampData(cloudData);
	std::vector<Eigen::Vector2d> data_before_ipcl = data_2d;
	std::cout << "clampData" << std::endl;
    
	if (ipcl) {
		std::cout << "IPCL" << std::endl;
		data_2d = IPCL(data_2d, 1);
	}
	
	Eigen::Vector2d center = getCenter(data_2d);
	std::cout << "\ncenter: " << center.x() << " " << center.y() << std::endl << std::endl;

	//find lines, and remove points on lines
    std::vector<Eigen::Vector2d> org_data = data_2d;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> lines;
    int num_lines = 6;
	
    for(int i = 0; i < num_lines; i++) {
        std::pair<Eigen::Vector2d, Eigen::Vector2d> line = RANSAC(data_2d, range, iter);
		std::cout << "RANSAC\titer: " << i << "\tdata_size:" << data_2d.size() << ",\tline: ";
		std::cout << line.first.x() << " " << line.first.y() << ",\t" << line.second.x() << " " << line.second.y() << std::endl;
        lines.push_back(line);

        std::vector<Eigen::Vector2d> new_data;
        for (const Eigen::Vector2d& p: data_2d) {
            if (distancePointToLine(line.first, line.second, p) > range)
                new_data.push_back(p);
        }
		
        data_2d = new_data;
		std::cout << "remaining points after iter " << i << ": " << data_2d.size() << std::endl;
		for (const Eigen::Vector2d& p: data_2d) {
			std::cout << p.x() << " " << p.y() << std::endl;
		}
		std::cout << std::endl << std::endl;
    }
	
    lines = getBestLines(org_data, lines, 4, range);
    std::vector<Eigen::Vector2d> quad = getQuad(lines, center);	
	std::cout << "\nquad (intersection points):" << quad.size() << std::endl;
	for (const Eigen::Vector2d& p: quad) {
		std::cout << p.x() << " " << p.y() << "" << std::endl;
	}
	std::cout << std::endl << std::endl;

    //after we find quad, get exit points from it and org_data (data after ipcl)
	std::vector<Eigen::Vector2d> exitPoints;
	exitPoints = getExitFromQuadCirc(org_data, quad, numOfP, maxP, epsilon);
    return exitPoints;
}

std::vector<Eigen::Vector2d> RoomExit::getExitPointsFromCircIntersect(bool ipcl)
{
    // hyperparameters
    // for RANSAC
    const float range = 0.04;
    const int iter = 2000;
    // for get exit point from quad later...
    const int numOfP = 2000;
    const int maxP = 5;
    const float epsilon = 0.1; // set the epsilon to be lower then when we do dbscan


    //remove y dimension
    std::vector<Eigen::Vector2d> data_2d = clampData(cloudData);
	std::vector<Eigen::Vector2d> data_before_ipcl = data_2d;
	std::cout << "clampData" << std::endl;
    
	if (ipcl) {
		std::cout << "IPCL" << std::endl;
		data_2d = IPCL(data_2d, 1);
	}
	
	Eigen::Vector2d center = getCenter(data_2d);
	std::cout << "\ncenter: " << center.x() << " " << center.y() << std::endl << std::endl;

	//find lines, and remove points on lines
    std::vector<Eigen::Vector2d> org_data = data_2d;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> lines;
    int num_lines = 6;
	
    for(int i = 0; i < num_lines; i++) {
        std::pair<Eigen::Vector2d, Eigen::Vector2d> line = RANSAC(data_2d, range, iter);
		std::cout << "RANSAC\titer: " << i << "\tdata_size:" << data_2d.size() << ",\tline: ";
		std::cout << line.first.x() << " " << line.first.y() << ",\t" << line.second.x() << " " << line.second.y() << std::endl;
        lines.push_back(line);

        std::vector<Eigen::Vector2d> new_data;
        for (const Eigen::Vector2d& p: data_2d) {
            if (distancePointToLine(line.first, line.second, p) > range)
                new_data.push_back(p);
        }
		
        data_2d = new_data;
		std::cout << "remaining points after iter " << i << ": " << data_2d.size() << std::endl;
		for (const Eigen::Vector2d& p: data_2d) {
			std::cout << p.x() << " " << p.y() << std::endl;
		}
		std::cout << std::endl << std::endl;
    }
	
    lines = getBestLines(org_data, lines, 4, range);
    std::vector<Eigen::Vector2d> quad = getQuad(lines, center);	
	std::cout << "\nquad (intersection points):" << quad.size() << std::endl;
	for (const Eigen::Vector2d& p: quad) {
		std::cout << p.x() << " " << p.y() << "" << std::endl;
	}
	std::cout << std::endl << std::endl;

    //after we find quad, get exit points from it and org_data (data after ipcl)
    std::vector<Eigen::Vector2d> exitPoints;
	exitPoints = getExitFromQuadCircIntersect(org_data, quad, numOfP, maxP, epsilon);
    return exitPoints;
}
