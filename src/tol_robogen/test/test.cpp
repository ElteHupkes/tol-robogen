/**
 * @TODO License
 *
 * The first-step goal of this file will be to
 * produce an SDF file of the basic robot.
 */
// External libraries
#include <vector>

#include <tol_robogen/model/components/HingeModel.h>
#include <sdf_builder/Parts.h>
#include <sdf_builder/Types.h>
#include <tol_robogen/evolution/representation/RobotRepresentation.h>
#include <tol_robogen/model/Robot.h>

using namespace tol_robogen;
namespace sb = sdf_builder;

int main(int argc, char *argv[]) {
//	HingeModel component("my_cc");
//	component.initModel();
//
//	sb::Model model("temp_bot");
//	model.addPosable(component.getPosableGroup());
//
//	std::cout << "<?xml version=\"1.0\"?>" << '\n';
//	std::cout << "<sdf version=\"1.5\">" << '\n';
//	std::cout << model.toXML();
//	std::cout << "</sdf>" << std::endl;
//
//	if (argc != 2) {
//		std::cout << "Call with robot reference file only for now." << std::endl;
//		return EXIT_FAILURE;
//	}
//
//	// Load the reference robot into a RobotRepresentation
//	std::string referenceRobotFile(argv[1]);
//	RobotRepresentationPtr referenceBot(new RobotRepresentation());
//
//	if (!referenceBot->init(referenceRobotFile)) {
//		std::cout << "Failed interpreting robot from text file"
//				<< std::endl;
//		return EXIT_FAILURE;
//	}
//
//	RobotPtr bot = referenceBot->toRobot();
//	sb::ModelPtr model = bot->toSDFModel("temp_bot");
//
//	std::cout << "<?xml version=\"1.0\"?>" << '\n';
//	std::cout << "<sdf version=\"1.5\">" << '\n';
//	std::cout << model->toXML();
//	std::cout << "</sdf>" << std::endl;

	return 0;
}
