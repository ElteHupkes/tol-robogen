/*
 * TODO License
 */
#include <tol_robogen/model/components/actuated/ActiveHingeModel.h>
#include <tol_robogen/model/motors/ServoMotor.h>

namespace sb = sdf_builder;

namespace tol_robogen {

const float ActiveHingeModel::MASS_SERVO = inGrams(9);
const float ActiveHingeModel::MASS_SLOT = inGrams(7);
const float ActiveHingeModel::MASS_FRAME = inGrams(1.2);

const float ActiveHingeModel::SLOT_WIDTH = inMm(34);
const float ActiveHingeModel::SLOT_THICKNESS = inMm(1.5);

const float ActiveHingeModel::FRAME_LENGTH = inMm(18);
const float ActiveHingeModel::FRAME_HEIGHT = inMm(10);
const float ActiveHingeModel::FRAME_ROTATION_OFFSET = inMm(14); // Left to right

const float ActiveHingeModel::SERVO_LENGTH = inMm(24.5);
const float ActiveHingeModel::SERVO_HEIGHT = inMm(10);
const float ActiveHingeModel::SERVO_ROTATION_OFFSET = inMm(20.5); // Right to left

ActiveHingeModel::ActiveHingeModel(std::string id) :
		ActuatedComponent(id)
{}

ActiveHingeModel::~ActiveHingeModel() {

}

bool ActiveHingeModel::initModel() {
	// Create the 4 components of the hinge
	hingeRoot_ = this->createLink(B_SLOT_A_ID);
	sb::LinkPtr frame = this->createLink(B_FRAME_ID);
	sb::LinkPtr servo = this->createLink(B_SERVO_ID);
	hingeTail_ = this->createLink(B_SLOT_B_ID);

	// Set the masses, geometries and positions for the various boxes
	float separation = inMm(0.1);

	// Create box for root
	hingeRoot_->makeBox(MASS_SLOT, SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);

	// Box and position for frame
	double xFrame = SLOT_THICKNESS / 2 + separation + FRAME_LENGTH / 2 -
			SLOT_THICKNESS;
	frame->makeBox(MASS_FRAME, FRAME_LENGTH, SLOT_WIDTH, FRAME_HEIGHT);
	frame->position(sb::Vector3(xFrame, 0, 0));

	// Box and position for servo
	double xServo = xFrame + (FRAME_ROTATION_OFFSET - (FRAME_LENGTH / 2))
				+ (SERVO_LENGTH / 2 - (SERVO_LENGTH - SERVO_ROTATION_OFFSET));
	servo->makeBox(MASS_SERVO, SERVO_LENGTH, SLOT_WIDTH, SERVO_HEIGHT);
	servo->position(sb::Vector3(xServo, 0, 0));

	// Box and position for tail
	double xTail = xServo + SERVO_LENGTH / 2 + separation + SLOT_THICKNESS / 2;
	hingeTail_->makeBox(MASS_SLOT, SLOT_THICKNESS, SLOT_WIDTH, SLOT_WIDTH);
	hingeTail_->position(sb::Vector3(xTail, 0, 0));

	// Create joints to hold pieces in position
	// root <-> frame
	this->fixLinks(hingeRoot_, frame, sb::Vector3(-FRAME_LENGTH / 2, 0, 0));

	// Frame <-> servo (hinge)
	sb::JointPtr revolve(new sb::RevoluteJoint(frame, servo));
	revolve->axis->xyz(sb::Vector3(0, 1, 0));
	revolve->position(sb::Vector3(
			SERVO_LENGTH / 2 - SERVO_ROTATION_OFFSET, 0, 0));
	this->addJoint(revolve);

	// Servo <-> tail
	this->fixLinks(servo, hingeTail_, sb::Vector3(-SLOT_THICKNESS / 2, 0, 0));

	// Register the hinge as a motor
	MotorPtr motor(new ServoMotor(id_, 0, revolve, ServoMotor::DEFAULT_MAX_FORCE_SERVO));
	this->addMotor(motor);

	// TODO there was a max force / default gain here we should include
	//	this->motor_.reset(
	//			new ServoMotor(joint, ServoMotor::DEFAULT_MAX_FORCE_SERVO,
	//					ServoMotor::DEFAULT_GAIN,
	//					ioPair(this->getId(),0)));

	return true;
}

sb::LinkPtr ActiveHingeModel::getRoot() {
	return hingeRoot_;
}

sb::LinkPtr ActiveHingeModel::getSlot(unsigned int i) {
	if (i == SLOT_A) {
		return hingeRoot_;
	} else {
		return hingeTail_;
	}
}

sb::Vector3 ActiveHingeModel::getSlotPosition(unsigned int i) {

	if (i > 2) {
		std::cout << "[HingeModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	sb::Vector3 slotPos;
	if (i == SLOT_A) {

		sb::Vector3 curPos = hingeRoot_->position();
		sb::Vector3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SLOT_THICKNESS / 2);

	} else {

		sb::Vector3 curPos = hingeTail_->position();
		sb::Vector3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SLOT_THICKNESS / 2);

	}

	return slotPos;

}

sb::Vector3 ActiveHingeModel::getSlotAxis(unsigned int i) {

	if (i > 2) {
		std::cout << "[HingeModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	sb::Quaternion quat;
	sb::Vector3 axis;

	if (i == SLOT_A) {

		quat = hingeRoot_->rotation();
		axis[0] = -1; axis[1] = 0; axis[2] = 0;

	} else if (i == SLOT_B) {

		quat = hingeTail_->rotation();
		axis[0] = 1; axis[1] = 0; axis[2] = 0;

	}

	return quat * axis;

}

sb::Vector3 ActiveHingeModel::getSlotOrientation(unsigned int i) {

	if (i > 2) {
		std::cout << "[HingeModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	sb::Quaternion quat;
	sb::Vector3 axis;

	if (i == SLOT_A) {

		quat = hingeRoot_->rotation();
		axis[0] = 0; axis[1] = 1; axis[2] = 0;

	} else if (i == SLOT_B) {

		quat = hingeTail_->rotation();
		axis[0] = 0; axis[1] = 1; axis[2] = 0;

	}

	return quat * axis;

}

//void ActiveHingeModel::getMotors(
//		std::vector<boost::shared_ptr<Motor> >& motors) {
//	motors.resize(1);
//	motors[0] = this->motor_;
//}

}
