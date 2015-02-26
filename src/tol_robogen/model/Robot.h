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

namespace tol_robogen {

class Robot {
public:
	Robot();
	virtual ~Robot();

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
	void addBodyConnection(ConnectionPtr connection);

protected:
	/**
	 * Robot body parts
	 */
	std::vector< std::shared_ptr<Model> > bodyParts_;

	/**
	 * Connections between the body parts.
	 */
	std::vector<std::shared_ptr<Connection> > bodyConnections_;

	/**
	 * The core component of the robot
	 */
	std::shared_ptr<Model> coreComponent_;

};

} /* namespace sdf_builder */

#endif /* TOL_ROBOGEN_ROBOT_H_ */
