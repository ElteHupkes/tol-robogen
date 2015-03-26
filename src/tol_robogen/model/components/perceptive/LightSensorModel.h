/*
 * LightSensorModel.h
 *
 *  Created on: Mar 26, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_MODEL_COMPONENTS_PERCEPTIVE_LIGHTSENSORMODEL_H_
#define TOL_ROBOGEN_MODEL_COMPONENTS_PERCEPTIVE_LIGHTSENSORMODEL_H_

#include <tol_robogen/model/Component.h>

namespace tol_robogen {

class LightSensorModel: public Component {
public:
	static const float MASS;
	static const float SENSOR_BASE_WIDTH;
	static const float SENSOR_BASE_THICKNESS;
	static const float SENSOR_DISPLACEMENT;

	static const unsigned int SLOT_A = 0;
	static const unsigned int B_SENSOR_BASE_ID = 0;

	LightSensorModel(std::string id, ConfigurationPtr conf,
			bool internalSensor);
	virtual ~LightSensorModel();

	virtual bool initModel();

	virtual sdf_builder::LinkPtr getRoot();

	virtual sdf_builder::LinkPtr getSlot(unsigned int i);

	virtual sdf_builder::Vector3 getSlotPosition(unsigned int i);

	virtual sdf_builder::Vector3 getSlotOrientation(unsigned int i);

	virtual sdf_builder::Vector3 getSlotAxis(unsigned int i);

protected:
	bool internalSensor_;

	sdf_builder::LinkPtr sensorRoot_;

	void checkSlot(unsigned int i);
};

} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_MODEL_COMPONENTS_PERCEPTIVE_LIGHTSENSORMODEL_H_ */
