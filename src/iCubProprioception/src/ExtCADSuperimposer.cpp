#include "iCubProprioception/ExtCADSuperimposer.h"

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


ExtCADSuperimposer::ExtCADSuperimposer(const ConstString& port_prefix, const ConstString& robot, const ConstString& camera,
                                       const SICAD::ModelPathContainer& cad_hand, const ConstString& shader_path) :
    iKinCADSuperimposer(port_prefix, robot, camera, cad_hand, shader_path)
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
    Vector* estimates_mean = inport_pose_ext_.read(synch_);

    if (estimates_mean        != YARP_NULLPTR) ext_pose_copy_ = *estimates_mean;
    if (ext_pose_copy_.size() == 0 &&
        !inport_pose_ext_.isClosed())          ext_pose_copy_ = *(inport_pose_ext_.read(true));

    return ext_pose_copy_;
}


void ExtCADSuperimposer::getExtraObjPoseMap(Superimpose::ModelPoseContainer& hand_pose)
{
    if (view_forearm_)
        iKinCADSuperimposer::getExtraObjPoseMap(hand_pose);
}


bool ExtCADSuperimposer::sync_input(const bool status)
{
    synch_ = status;

    return true;
}


bool ExtCADSuperimposer::render_extra_mesh(const bool status)
{
    view_forearm_ = status;

    return true;
}
