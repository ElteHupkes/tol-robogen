#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <boost/bind.hpp>
#include <iostream>

namespace gazebo
{
class Stress : public WorldPlugin
{
  public: void Load(physics::WorldPtr /*_parent */, sdf::ElementPtr /*_sdf*/)
  {
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Stress::OnUpdate, this, _1));

      lastSec = -1;
  }

  public: void OnUpdate(const common::UpdateInfo & _info)
  {
	int sec = _info.simTime.sec;
	if (sec != lastSec) {
		std::cout << sec << "...";
		std::cout.flush();
		lastSec = sec;

		if (lastSec >= 6) {
			std::cout << "\nFinal time: " << _info.realTime.Double() << std::endl;
		}
	}
  }

  private: int lastSec;

  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Stress)
}
