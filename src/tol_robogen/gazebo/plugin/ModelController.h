/*
 * ModelController.h
 *
 *  Created on: Mar 4, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_GAZEBO_PLUGIN_MODELCONTROLLER_H_
#define TOL_ROBOGEN_GAZEBO_PLUGIN_MODELCONTROLLER_H_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo {

class ModelController: public ModelPlugin {
public:
	ModelController();
	virtual ~ModelController();

public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	void OnUpdate(const common::UpdateInfo & _info);

private:
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
};

GZ_REGISTER_MODEL_PLUGIN(ModelController)

} /* namespace gazebo */

#endif /* TOL_ROBOGEN_GAZEBO_PLUGIN_MODELCONTROLLER_H_ */
