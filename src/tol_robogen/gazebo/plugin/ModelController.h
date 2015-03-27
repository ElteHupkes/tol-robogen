/*
 * ModelController.h
 *
 *  Created on: Mar 4, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_GAZEBO_PLUGIN_MODELCONTROLLER_H_
#define TOL_ROBOGEN_GAZEBO_PLUGIN_MODELCONTROLLER_H_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>

#include <tol_robogen/gazebo/Types.h>

#include <vector>

namespace tol_robogen {
namespace gazebo {

class ModelController: public ::gazebo::ModelPlugin {
public:
	ModelController();
	virtual ~ModelController();

public:
	void Load(::gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

	/**
	 * Called when the driver sensor updates
	 */
	void OnUpdate();
protected:
	/**
	 * Detects and loads motors in the plugin spec
	 */
	void loadMotors(sdf::ElementPtr sdf);

	/**
	 * Detecs and loads sensors in the plugin spec.
	 */
	void loadSensors(sdf::ElementPtr sdf);

	/**
	 * Loads the brain specification and creates
	 * the neural network.
	 */
	void loadBrain(sdf::ElementPtr sdf);

	/**
	 * Brain controlling this model
	 */
	BrainPtr brain_;

	/**
	 * Actuation time, in nanoseconds
	 */
	unsigned int actuationTime_;

	/**
	 * Time of the last actuation, in
	 * seconds and nanoseconds
	 */
	unsigned int lastActuationSec_;
	unsigned int lastActuationNsec_;

	/**
	 * Motors in this model
	 */
	std::vector< MotorPtr > motors_;

	/**
	 * Sensors in this model
	 */
	std::vector< SensorPtr > sensors_;

    // Pointer to the model
    ::gazebo::physics::ModelPtr model;

    // Pointer to the world
	::gazebo::physics::WorldPtr world;

    // Pointer to the driver sensor
    SensorPtr driver;
private:
    // Driver update event pointer
    ::gazebo::event::ConnectionPtr updateConnection_;
};

} /* namespace gazebo */
}

GZ_REGISTER_MODEL_PLUGIN(tol_robogen::gazebo::ModelController)

#endif /* TOL_ROBOGEN_GAZEBO_PLUGIN_MODELCONTROLLER_H_ */
