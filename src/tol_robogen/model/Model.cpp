/*
 *
 */
#include <stdexcept>
#include <sstream>
#include <string>
#include <cmath>

#include <iostream>

#include <tol_robogen/model/Model.h>
#include <sdf_builder/util/Util.h>
#include <sdf_builder/joint/FixedJoint.h>

namespace tol_robogen {

namespace sb = sdf_builder;

Model::Model(std::string id) :
		id_(id),
		posableGroup_(sb::PosableGroupPtr(new sb::PosableGroup("group_"+id)))
{}

Model::~Model() {

}

const std::string &Model::getId(){
	return id_;
}

const sb::Vector3 & Model::getRootPosition() {
	return posableGroup_->position();
}

const sb::Quaternion & Model::getRootAttitude() {
	return posableGroup_->rotation();
}

void Model::setRootPosition(const sb::Vector3& pos) {
	posableGroup_->position(pos);
}

void Model::translateRootPosition(const sb::Vector3& translation) {

	sb::Vector3 newPosition = this->getRootPosition() + translation;
	this->setRootPosition(newPosition);

}

void Model::setRootAttitude(const sb::Quaternion& quat) {
	this->posableGroup_->rotation(quat);
}

void Model::addLink(sb::LinkPtr body, int id) {
	this->links_.insert(std::pair<int, sb::LinkPtr>(id, body));
}

sb::LinkPtr Model::createLink(int label) {
	sb::LinkPtr b(new sb::Link("link_"+id_+std::to_string(label)));
	if (label >= 0) {
		this->addLink(b, label);
	}

	// Add the model to the posable group
	this->posableGroup_->addPosable(b);
	return b;
}

sb::LinkPtr Model::createLink() {
	return this->createLink(-1);
}

const std::vector< sb::JointPtr > & Model::getJoints() {
	return joints_;
}

const sb::PosableGroupPtr & Model::getPosableGroup() {
	return posableGroup_;
}

void Model::addJoint(sb::JointPtr joint) {
	joints_.push_back(joint);

	// Do *NOT* add the joint to the posables, its pose is relative
	// to the child frame.
}

const std::vector< sb::JointPtr > & Model::joints() {
	return joints_;
}


sb::JointPtr Model::fixLinks(sb::LinkPtr parent, sb::LinkPtr child,
		const sb::Vector3& anchor, const sb::Vector3& axis) {
	sb::JointPtr joint(new sb::FixedJoint(parent, child));
	joint->position(anchor);
	joint->axis->xyz(axis);
	this->addJoint(joint);
	return joint;
}

bool Model::attach(ModelPtr from, unsigned int fromSlot, unsigned int toSlot, unsigned int orientation) {
	if (orientation > 3){
		std::cerr << "Specified orientation to parent slot is not"\
				" between 0 and 3." << std::endl;
		return false;
	}

	auto toPosable = from->getPosableGroup();

	// Beware that here "toSlot" belongs to self
	sb::Vector3 aSlotPosition = getSlotPosition(toSlot);
	sb::Vector3 bSlotPosition = from->getSlotPosition(fromSlot);

	sb::Vector3 aSlotNormal = getSlotAxis(toSlot);
	sb::Vector3 bSlotNormal = from->getSlotAxis(fromSlot);

	sb::Vector3 aSlotTangent = getSlotOrientation(toSlot);
	sb::Vector3 bSlotTangent = from->getSlotOrientation(fromSlot);

//	std::cerr << aSlotPosition << '\n' << std::endl;
//	std::cerr << bSlotPosition << '\n' << std::endl;
//	std::cerr << aSlotNormal << '\n' << std::endl;
//	std::cerr << bSlotNormal << '\n' << std::endl;
//	std::cerr << aSlotTangent << '\n' << std::endl;
//	std::cerr << bSlotTangent << '\n' << std::endl;

	posableGroup_->align(
		aSlotPosition,
		aSlotNormal,
		aSlotTangent,
		bSlotPosition,
		bSlotNormal,
		bSlotTangent,
		toPosable,

		// Vectors were already calculated relative to parent frame
		// This is redundant in many cases, but I'll leave it in now
		// for convenience.
		sb::Posable::RELATIVE_TO_PARENT_FRAME
	);

	// TODO Set orientation
	posableGroup_->rotateAround(getSlotAxis(toSlot), 0);

	// TODO Create a fixed link
	sb::LinkPtr child = getSlot(toSlot);
	sb::LinkPtr parent = from->getSlot(fromSlot);
	// Position of the link is in the child frame, take the child
	// position and subtract the root position.
	sb::Vector3 anchor = child->position() - getRootPosition();

	// Direction vector is also in the child frame, turn to local direction
	sb::Vector3 axis = sb::Util::toLocalDirection(getSlotAxis(toSlot), child.get());
	this->fixLinks(parent, child, anchor, axis);

	return true;
}

//dxGeom* Model::createCylinderGeom(dBodyID body, float mass,
//		const osg::Vec3& pos, int direction, float radius, float height) {
//
//	dMass massOde;
//	dMassSetCylinderTotal(&massOde, mass, direction, radius, height);
//	dBodySetMass(body, &massOde);
//	dxGeom* g = dCreateCylinder(this->getCollisionSpace(), radius, height);
//	dBodySetPosition(body, pos.x(), pos.y(), pos.z());
//	dGeomSetPosition(g, pos.x(), pos.y(), pos.z());
//
//	if (direction == 1) {
//
//		osg::Quat rotateCylinder;
//		rotateCylinder.makeRotate(osg::inDegrees(90.0), osg::Vec3(0, 1, 0));
//		dsb::Quaternion quatOde;
//		quatOde[0] = rotateCylinder.w();
//		quatOde[1] = rotateCylinder.x();
//		quatOde[2] = rotateCylinder.y();
//		quatOde[3] = rotateCylinder.z();
//		dBodySetsb::Quaternion(body, quatOde);
//
//	} else if (direction == 2) {
//
//		osg::Quat rotateCylinder;
//		rotateCylinder.makeRotate(osg::inDegrees(90.0), osg::Vec3(1, 0, 0));
//		dsb::Quaternion quatOde;
//		quatOde[0] = rotateCylinder.w();
//		quatOde[1] = rotateCylinder.x();
//		quatOde[2] = rotateCylinder.y();
//		quatOde[3] = rotateCylinder.z();
//		dBodySetsb::Quaternion(body, quatOde);
//
//	}
//
//	dGeomSetBody(g, body);
//
//	return g;
//
//}

//dxGeom* Model::createCapsuleGeom(dBodyID body, float mass, const osg::Vec3& pos,
//		int direction, float radius, float height) {
//
//	dMass massOde;
//	dMassSetCapsuleTotal(&massOde, mass, direction, radius, height);
//	dBodySetMass(body, &massOde);
//	dxGeom* g = dCreateCapsule(this->getCollisionSpace(), radius, height);
//	dBodySetPosition(body, pos.x(), pos.y(), pos.z());
//	dGeomSetPosition(g, pos.x(), pos.y(), pos.z());
//
//	if (direction == 1) {
//
//		osg::Quat rotateCapsule;
//		rotateCapsule.makeRotate(osg::inDegrees(90.0), osg::Vec3(0, 1, 0));
//		dsb::Quaternion quatOde;
//		quatOde[0] = rotateCapsule.w();
//		quatOde[1] = rotateCapsule.x();
//		quatOde[2] = rotateCapsule.y();
//		quatOde[3] = rotateCapsule.z();
//		dBodySetsb::Quaternion(body, quatOde);
//
//	} else if (direction == 2) {
//
//		osg::Quat rotateCapsule;
//		rotateCapsule.makeRotate(osg::inDegrees(90.0), osg::Vec3(1, 0, 0));
//		dQuaternion quatOde;
//		quatOde[0] = rotateCapsule.w();
//		quatOde[1] = rotateCapsule.x();
//		quatOde[2] = rotateCapsule.y();
//		quatOde[3] = rotateCapsule.z();
//		dBodySetQuaternion(body, quatOde);
//
//	}
//
//	dGeomSetBody(g, body);
//
//	return g;
//
//}

//dJointID Model::fixBodies(dBodyID b1, dBodyID b2, const osg::Vec3& /*axis*/) {
//	dJointID joint = dJointCreateFixed(this->getPhysicsWorld(), 0);
//	dJointAttach(joint, b1, b2);
//	dJointSetFixed(joint);
//	return joint;
//
//}

}
