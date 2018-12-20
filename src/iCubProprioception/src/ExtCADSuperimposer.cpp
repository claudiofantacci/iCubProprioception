#include <iCubProprioception/ExtCADSuperimposer.h>

#include <exception>
#include <utility>

#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>
#include <yarp/os/LogStream.h>

using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace iCub::iKin;


ExtCADSuperimposer::ExtCADSuperimposer
(
    const ConstString& robot,
    const ConstString& camera,
    const SICAD::ModelPathContainer& cad_hand,
    const ConstString& shader_path,
    const ConstString& port_prefix,
    const bool draw_thumb,
    const bool draw_forearm
 ) :
    iKinCADSuperimposer(robot, camera, cad_hand, shader_path, port_prefix, draw_thumb, draw_forearm)
{
    yInfo() << log_ID_ << "Invoked ExtCADSuperimposer (derived class) ctor...";

    /* Port for the external input (6D pose) */
    yInfo() << log_ID_ << "Opening ports for external end effector pose.";
    if (!inport_pose_ext_.open("/" + ID_ + "/cam/" + camera_ + "/hand/right_pose:i"))
    {
        yError() << log_ID_ << "Cannot open input port for external pose!";
        throw std::runtime_error("Cannot open input port for external pose!");
    }

    yInfo() << log_ID_ << "...ExtCADSuperimposer ctor completed!";
}


ExtCADSuperimposer::~ExtCADSuperimposer() noexcept { }


void ExtCADSuperimposer::onStop()
{
    if (!inport_pose_ext_.isClosed()) inport_pose_ext_.interrupt();

    iKinCADSuperimposer::onStop();
}


void ExtCADSuperimposer::threadRelease()
{
    if (!inport_pose_ext_.isClosed()) inport_pose_ext_.close();

    iKinCADSuperimposer::threadRelease();
}


yarp::sig::Vector ExtCADSuperimposer::getEndEffectorPose()
{
    Vector* ext_ee_pose = inport_pose_ext_.read(synch_);

    if   (ext_ee_pose != YARP_NULLPTR) return *ext_ee_pose;
    else                               return zeros(1);
}


bool ExtCADSuperimposer::sync_input(const bool status)
{
    synch_ = status;

    return true;
}
