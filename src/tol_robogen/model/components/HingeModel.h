/*
 *
 */
#ifndef TOL_ROBOGEN_HINGE_MODEL_H_
#define TOL_ROBOGEN_HINGE_MODEL_H_

#include <tol_robogen/model/Model.h>

namespace tol_robogen {

/**
 * A Hinge is modelled with 4 boxes and a hinge joint
 */
class HingeModel: public Model {

public:

	static const float MASS_SLOT;
	static const float MASS_FRAME;

	static const float SLOT_WIDTH;
	static const float SLOT_THICKNESS;
	static const float CONNECTION_PART_LENGTH;
	static const float CONNECTION_PART_HEIGHT;
	static const float CONNECTION_PART_THICKNESS;
	static const float CONNECTION_ROTATION_OFFSET;

	static const unsigned int SLOT_A = 0;
	static const unsigned int SLOT_B = 1;

	static const unsigned int B_SLOT_A_ID = 0;
	static const unsigned int B_SLOT_B_ID = 1;
	static const unsigned int B_CONNECTION_A_ID = 2;
	static const unsigned int B_CONNECTION_B_ID = 3;

	HingeModel(std::string id);

	virtual ~HingeModel();

	virtual bool initModel();

	virtual sdf_builder::LinkPtr getRoot();

	virtual sdf_builder::LinkPtr getSlot(unsigned int i);

	virtual sdf_builder::Vector3 getSlotPosition(unsigned int i);

	virtual sdf_builder::Vector3 getSlotOrientation(unsigned int i);

	virtual sdf_builder::Vector3 getSlotAxis(unsigned int i);

private:

	sdf_builder::LinkPtr hingeRoot_;
	sdf_builder::LinkPtr hingeTail_;

};

}

#endif /* TOL_ROBOGEN_HINGE_MODEL_H_ */
