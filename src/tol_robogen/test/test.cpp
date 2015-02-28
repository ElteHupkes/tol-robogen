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

using namespace tol_robogen;
namespace sb = sdf_builder;

int main() {
	HingeModel component("my_cc");
	component.initModel();

	sb::Model model("temp_bot");
	model.addPosable(component.getPosableGroup());

	std::cout << "<?xml version=\"1.0\"?>" << '\n';
	std::cout << "<sdf version=\"1.5\">" << '\n';
	std::cout << model.toXML();
	std::cout << "</sdf>" << std::endl;

	/**
	 * Plan of attack for creating the Gazebo bots:
	 * - File is read into RobotRepresentation (or random bot is created)
	 * - Instead of turning that representation into protobuf, I make my
	 *   own robot class, where body parts and connections will be added
	 *   to instead.
	 * - This will then be converted to the SDF bot using functionality
	 *   similar to RobogenUtils::connect().
	 */

//	if (argc != 2) {
//		std::cout << "Call with robot reference file only for now." << std::endl;
//		return exitRobogen(EXIT_FAILURE);
//	}
//
//	// Load the reference robot into a RobotRepresentation
//	std::string referenceRobotFile(argv[1]);
//	boost::shared_ptr<RobotRepresentation> referenceBot(
//				new RobotRepresentation());
//
//	if (!referenceBot->init(referenceRobotFile)) {
//		std::cout << "Failed interpreting robot from text file"
//				<< std::endl;
//		return exitRobogen(EXIT_FAILURE);
//	}

	// Here, I want something like
	// referenceBot->getRobot()
	// In steps, creating a robot goes like this:
	// - A new robot object is created, and Robot::init() is called
	//   (Simulator.cpp)
	// - Robot::init() creates a joint group, and calls `decodeBody`
	//   and `decodeBrain` (which for now I'm ignoring). These methods
	//	 use the protobuf message which contains the robot config.
	// - `decodeBody` iterates the body parts in the protobuf message,
	//   and calls RobogenUtils::createModel() for each body part. Note
	//	 the bodyPart protobuf message contains the orientation of the
	//	 body part relative to the parent. This function instantiates
	//   the model and sets some variables (this seems to have no effect
	//   on the position of the model; though the variables it sets might
	//   later on, especially orientationToParentModel).
	// - Next up `decodeBody` calls `initModel`, which creates the geometry
	//   but also does not seem to set a position whatsoever.
	// - It then calls `setRootPosition` on the model, which you'd think
	//   would set the position, but I'm as of yet clueless how this could
	//   be accurate - it merely applies some fixed x / y spacing at every body
	//   part iteration.
	// - It also registers the sensors and motors in the model
	// - Next up, the connections between the body parts are stored
	// - It checks whether the robot is in one piece, which is kinda useful
	// - Then for the important part, it calls Robot::reconnect(). This appears
	//   to do a breadth first search over the graph that is the robot, and
	//   specifies a `BodyConnectionVisiter`; which has functionality called when
	//   an edge is visited.
	// - This functionality in turn calls RobogenUtils::connect() with the
	//   information about each connection / body part.
	// - Here I finally found what positions the body parts. `RobogenUtils::connect()`
	//   operates on a part A, a part B, and the respective slots on which these
	//   should be connected, which are defined by a position and an axis. It
	//   rotates and translates A such that its slot axis aligns with that of
	//   B.
	// - This function also creates the joints.
	// - Alright, so after the robot body parts have been connected, their positions
	//   and rotations are all accurate. I should probably hook into this process so
	//   that instead of sending the positions to ODE, it stores the positions somewhere
	//   so that I can extract them, and create the SDF from it.
	// - I can use gazebo::math::Pose to convert the created quaternions to
	//   roll/pitch/yaw as is used in SDF.
	//   Q: Gazebo uses C++11; can I even use those headers?

	// Ok, starting now, I'm creating an additional Model class
	// (namespaced robogen::gazebo) that does not depend on ODE whatsoever.


	return 0;
}
