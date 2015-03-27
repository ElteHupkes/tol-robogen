/*
 * TouchSensorModel.h
 *
 *  Created on: Mar 27, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOGEN_MODEL_COMPONENTS_PERCEPTIVE_TOUCHSENSORMODEL_H_
#define TOL_ROBOGEN_MODEL_COMPONENTS_PERCEPTIVE_TOUCHSENSORMODEL_H_

#include <tol_robogen/model/Component.h>

namespace tol_robogen {

class TouchSensorModel: public Component {
public:
	static const float MASS;
	static const float SENSOR_BASE_WIDTH;
	static const float SENSOR_BASE_THICKNESS;
	static const float SENSOR_THICKNESS;
	static const float SENSOR_WIDTH;
	static const float SENSOR_HEIGHT;

	static const unsigned int SLOT_A = 0;

	static const unsigned int B_SENSOR_BASE_ID = 0;
	static const unsigned int B_SENSOR_LEFT = 1;
	static const unsigned int B_SENSOR_RIGHT = 2;

	using Component::Component;
	virtual ~TouchSensorModel();

	virtual bool initModel();

	virtual sdf_builder::LinkPtr getRoot();

	virtual sdf_builder::LinkPtr getSlot(unsigned int i);

	virtual sdf_builder::Vector3 getSlotPosition(unsigned int i);

	virtual sdf_builder::Vector3 getSlotOrientation(unsigned int i);

	virtual sdf_builder::Vector3 getSlotAxis(unsigned int i);

protected:
	sdf_builder::LinkPtr sensorRoot_;
	void checkSlot(unsigned int i);

	// Helper function to keep creating left / right sensors DRY
	void sensorHelper(unsigned int id,
			double mass, double thickness, double width, double height,
			double xSensors, double ySensor, std::string name,
			unsigned int ioId);
};

} /* namespace tol_robogen */

#endif /* TOL_ROBOGEN_MODEL_COMPONENTS_PERCEPTIVE_TOUCHSENSORMODEL_H_ */
