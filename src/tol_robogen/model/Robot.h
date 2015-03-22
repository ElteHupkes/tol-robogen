/*
 * Robot.h
 *
 *  Created on: Feb 26, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_ROBOT_H_
#define TOL_ROBOGEN_ROBOT_H_

#include <tol_robogen/tol.h>
#include <tol_robogen/model/Model.h>
#include <tol_robogen/model/Connection.h>
#include <tol_robogen/evolution/representation/PartRepresentation.h>
#include <tol_robogen/evolution/representation/NeuralNetworkRepresentation.h>
#include <tol_robogen/configuration/Configuration.h>

#include <sdf_builder/Types.h>

namespace tol_robogen {

/**
 * The Robot class is the intermediary between a `RobotRepresentation`
 * and an SDF model that can be used in Gazebo.
 */
class Robot {
public:
	/**
	 * Initialize empty robot
	 */
	Robot();

	/**
	 * Initialize robot from core component part
	 * representation.
	 */
	Robot(PartRepresentationPtr core, NeuralNetworkRepresentationPtr brain, const Configuration & config);

	virtual ~Robot();

	/**
	 * Initializes the robot with the given core.
	 */
	void init(PartRepresentationPtr core, NeuralNetworkRepresentationPtr brain, const Configuration & config);

	// TODO copy constructor, assignment operator

	/**
	 *  @return  the body parts of which the robot is composed of
	 */
	const std::vector< ModelPtr >& getBodyParts()
			const;

	/**
	 * @return The connections between body parts
	 */
	const std::vector< ConnectionPtr >& getBodyConnections()
			const;

	/**
	 * Adds a new body part
	 */
	void addBodyPart(ModelPtr bodyPart);

	/**
	 * Adds a new body connection
	 */
	void addBodyConnection(ModelPtr from, ModelPtr to, unsigned int fromSlot,
			unsigned int toSlot, unsigned int orientation);

	/**
	 * Output SDF for this robot
	 */
	sdf_builder::ModelPtr toSDFModel(const std::string & name);

protected:
	/**
	 * Robot body parts
	 */
	std::vector< ModelPtr > bodyParts_;

	/**
	 * Connections between the body parts.
	 */
	std::vector< ConnectionPtr > bodyConnections_;

	/**
	 * The core component of the robot
	 */
	ModelPtr coreComponent_;

	/**
	 * Simplest possible representation of the brain.
	 * @todo We might need an actual brain structure instead
	 */
	std::string brainXML_;

};

} /* namespace sdf_builder */

#endif /* TOL_ROBOGEN_ROBOT_H_ */
