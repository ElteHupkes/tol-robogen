/**
 * @TODO License
 *
 * The first-step goal of this file will be to
 * produce an SDF file of the basic robot.
 */
// External libraries
#include <vector>
#include <fstream>

#include <tol_robogen/model/Models.h>
#include <sdf_builder/Parts.h>
#include <sdf_builder/Types.h>
#include <tol_robogen/evolution/representation/RobotRepresentation.h>
#include <tol_robogen/model/Robot.h>

using namespace tol_robogen;
namespace sb = sdf_builder;

int main(int argc, char *argv[]) {
//	ModelPtr core(new CoreComponentModel("my_core", true));
//	core->initModel();
//
//	ModelPtr component(new ActiveHingeModel("my_hinge"));
//	component->initModel();
//
//	ModelPtr brick(new CoreComponentModel("my_brick", false));
//	brick->initModel();
//
//	core->setRootPosition(sb::Vector3(1, 1, 1));
//	component->attach(core, 0, 0, 0);
//	brick->attach(component, 1, 0, 0);
//
//	sb::Model model("temp_bot");
//
//	// Know for a fact only the component has joints so ignore the
//	// rest in this test.
//	auto joints = component->joints();
//	auto itb = joints.begin();
//	for (; itb != joints.end(); ++itb) {
//		model.addJoint(*itb);
//	}
//
//	model.addPosable(component->getPosableGroup());
//	model.addPosable(core->getPosableGroup());
//	model.addPosable(brick->getPosableGroup());
//	model.position(sb::Vector3(0, 0, 1));
//
//	//model.rotateAround(sb::Vector3(0, 1, 0), 0.5);
//
//	std::cout << "<?xml version=\"1.0\"?>" << '\n';
//	std::cout << "<sdf version=\"1.5\">" << '\n';
//	std::cout << model.toXML();
//	std::cout << "</sdf>" << std::endl;

//  Full robot test
	if (argc != 3) {
		std::cout << "Call with robot reference file and output file." << std::endl;
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

	RobotPtr bot = referenceBot->toRobot();
	sb::ModelPtr model = bot->toSDFModel("temp_bot");

	// Attach our model controller plugin
	sb::ElementPtr plugin(new sb::StringElement("<plugin name=\"control\" filename=\"libtolmodelcontrol.so\" />"));
	model->addElement(plugin);

	std::ofstream out;
	out.open(argv[2]);

	out << "<?xml version=\"1.0\"?>" << '\n';
	out << "<sdf version=\"1.5\">" << '\n';
	out << model->toXML();
	out << "</sdf>" << std::endl;

	out.close();
	return 0;
}
