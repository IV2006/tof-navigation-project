#include "Navigator/Navigator.h"
#include <cstdio>  // for freopen()

int main(int argc, char **argv)
{
	freopen("/media/sf_AI_extras/output.txt", "w", stdout);
	
	std::cout << std::fixed;
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string configPath = data["slam_configuration"]["drone_yaml_path"];
    std::string vocabulary_path = data["slam_configuration"]["vocabulary_path"];
    std::string modelTextureNameToAlignTo = data["simulator_configuration"]["align_model_to_texture"]["texture"];
    bool alignModelToTexture = data["simulator_configuration"]["align_model_to_texture"]["align"];
    std::string model_path = data["simulator_configuration"]["model_path"];
    std::string simulatorOutputDir = data["simulator_configuration"]["simulator_output_dir"];
    std::string loadMapPath = data["slam_configuration"]["load_map_path"];
    double movementFactor = data["simulator_configuration"]["movement_factor"];
    double simulatorStartingSpeed = data["simulator_configuration"]["simulator_starting_speed"];
	bool logs = data["logs"];
	std::vector<Eigen::Vector3d> eigenData;
	
	Eigen::Vector3f startingPoint(-0.5, 1, 0);
	std::cout << "startingPoint: " << startingPoint.x() << ", " << startingPoint.y() << ", " << startingPoint.z() << std::endl;

    Simulator simulator(configPath, model_path, alignModelToTexture, modelTextureNameToAlignTo, startingPoint, true, simulatorOutputDir, false,
                        loadMapPath, movementFactor, simulatorStartingSpeed, vocabulary_path);
    auto simulatorThread = simulator.run();

	cout << "waiting for simulator to be ready" << std::endl;
    while (!simulator.isReady()) {
		sleep(1);
    }

	cout << "waiting for simulator to startScanning (press tab)" << std::endl;
	simulator.setTrack(true);
	while (!simulator.startScanning()) {
		sleep(1);
	}
	
	Navigator navigator(simulator, logs);

	navigator.scan(10, 350000);
	auto points = navigator.getExitPoints();
	std::cout << "exit points (dist, x, y, z):" << std::endl;
	for (const auto& p: points) {
		std::cout << p.first << "\t(" << p.second.x() << " " << p.second.y() << " " << p.second.z() << ")" << std::endl;
	}
	
	char c;
	std::cout << "ALL READY, ENTER A CHAR TO NAVIGATE TO WALL" << std::endl;
	std::cin >> c;
	navigator.navigateToPoint(points[0].second, true);
	
	std::cout << "ALL READY, ENTER A CHAR TO NAVIGATE TO EXIT" << std::endl;
	std::cin >> c;
	navigator.navigateToPoint(points[1].second, true);
	
	std::cout << "ALL READY, ENTER A CHAR TO VISUALIZE" << std::endl;
	std::cin >> c;
	system("python3 /media/sf_AI_extras/visualize.py");
	
	cout << "\ndone" << std::endl;
    simulatorThread.join();
	
    return 0;
}
