/*
 * TODO License
 */
#ifndef TOLROBOGEN_ACTIVE_HINGE_MODEL_H_
#define TOLROBOGEN_ACTIVE_HINGE_MODEL_H_

#include <tol_robogen/model/Component.h>

namespace tol_robogen {

/**
 * An active hinge is modelled with 4 boxes and a hinge joint
 */
class ActiveHingeModel: public Component {

public:

	static const float MASS_SLOT;
	static const float MASS_SERVO;
	static const float MASS_FRAME;
	static const float SLOT_WIDTH;
	static const float SLOT_THICKNESS;
	static const float FRAME_LENGTH;
	static const float FRAME_HEIGHT;
	static const float FRAME_ROTATION_OFFSET;

	static const float SERVO_LENGTH;
	static const float SERVO_HEIGHT;
	static const float SERVO_ROTATION_OFFSET;

	static const unsigned int SLOT_A = 0;
	static const unsigned int SLOT_B = 1;

	static const unsigned int B_SLOT_A_ID = 0;
	static const unsigned int B_SLOT_B_ID = 1;
	static const unsigned int B_FRAME_ID = 2;
	static const unsigned int B_SERVO_ID = 3;

	ActiveHingeModel(std::string id, const Configuration & conf);

	virtual ~ActiveHingeModel();

	virtual bool initModel();

	virtual sdf_builder::LinkPtr getRoot();

	virtual sdf_builder::LinkPtr getSlot(unsigned int i);

	virtual sdf_builder::Vector3 getSlotPosition(unsigned int i);

	virtual sdf_builder::Vector3 getSlotOrientation(unsigned int i);

	virtual sdf_builder::Vector3 getSlotAxis(unsigned int i);

	//virtual void getMotors(std::vector<boost::shared_ptr<Motor> >& motors);

private:

	sdf_builder::LinkPtr hingeRoot_;
	sdf_builder::LinkPtr hingeTail_;
	//boost::shared_ptr<Motor> motor_;

};

}

#endif /* ROBOGEN_ACTIVE_HINGE_MODEL_H_ */
