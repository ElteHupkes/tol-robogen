/**
 * @TODO License
 *
 * The first-step goal of this file will be to
 * produce an SDF file of the basic robot.
 */
// External libraries
#include <vector>
#include <fstream>
#include <cmath>
#include <string>

#include <tol_robogen/model/Models.h>
#include <sdf_builder/Parts.h>
#include <sdf_builder/Types.h>
#include <tol_robogen/evolution/representation/RobotRepresentation.h>
#include <tol_robogen/model/Robot.h>
#include <tol_robogen/configuration/Configuration.h>

#include <sdf_builder/util/Util.h>

using namespace tol_robogen;
namespace sb = sdf_builder;

int main(int argc, char *argv[]) {
//	ModelPtr core(new CoreComponentModel("my_core", true));
//	core->initModel();
//
//	ModelPtr component(new CoreComponentModel("my_hinge", false));
//	component->initModel();
//
//	ModelPtr brick(new CoreComponentModel("my_brick", false));
//	brick->initModel();
//
//	component->attach(core, 0, 0, 1);
//
//	brick->attach(component, 1, 0, 1);
//
//	sb::Model model("temp_bot");
//
//	// Add component inner joints as well as fixing joint of brick
//	auto joints = component->joints();
//	for (auto itb = joints.begin(); itb != joints.end(); ++itb) {
//		model.addJoint(*itb);
//	}
//
//	joints = brick->joints();
//	for (auto itb = joints.begin(); itb != joints.end(); ++itb) {
//		model.addJoint(*itb);
//	}
//
//	model.addPosable(core->getPosableGroup());
//	model.addPosable(component->getPosableGroup());
//	model.addPosable(brick->getPosableGroup());
//	//model.position(sb::Vector3(0, 0, 0.3));
//
//	//model.rotateAround(sb::Vector3(0, 1, 0), 0.5);
//
//	std::cout << "<?xml version=\"1.0\"?>" << '\n';
//	std::cout << "<sdf version=\"1.5\">" << '\n';
//	std::cout << model.toXML();
//	std::cout << "</sdf>" << std::endl;

    // Full robot test
	if (argc != 4) {
		std::cout << "Call with robot reference file, output file and scaling factor." << std::endl;
		return EXIT_FAILURE;
	}

	// Load the reference robot into a RobotRepresentation
	std::string referenceRobotFile(argv[1]);
	RobotRepresentationPtr referenceBot(new RobotRepresentation());

	if (!referenceBot->init(referenceRobotFile)) {
		std::cout << "Failed interpreting robot from text file"
				<< std::endl;
		return EXIT_FAILURE;
	}

	// Create configuration instance
	ConfigurationPtr conf(new Configuration());
	conf->scaling = std::stod(argv[3]);

	RobotPtr bot = referenceBot->toRobot(conf);
	sb::ModelPtr model = bot->toSDFModel("temp_bot");

	model->position(sb::Vector3(0, 0, 0.025 * conf->scaling));

	std::ofstream out;
	out.open(argv[2]);

	out << "<?xml version=\"1.0\"?>" << '\n';
	out << "<sdf version=\"1.5\">" << '\n';
	out << model->toXML();
	out << "</sdf>" << std::endl;

	out.close();
	return 0;
}
