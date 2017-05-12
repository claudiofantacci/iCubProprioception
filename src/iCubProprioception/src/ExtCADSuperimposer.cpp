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


ExtCADSuperimposer::ExtCADSuperimposer(const ConstString& project_name, const ConstString& robot, const ConstString& camera,
                                       const SuperImpose::ObjFileMap& cad_hand, const ConstString& shader_path) :
    iKinCADSuperimposer(project_name + "/ExtCADSuperimposer", robot, camera, cad_hand, shader_path)
{
    yInfo() << log_ID_ << "Invoked ExtCADSuperimposer (derived class) ctor...";


    /* Get arm interfaces */
    if (!setArmRemoteControlboard())
    {
        yError() << log_ID_ << "Arm remote_controlboard errored!";
        throw std::runtime_error("remote_controlboard errored!");
    }


    /* Port for the external input (6D pose) */
    yInfo() << log_ID_ << "Opening ports for external end effector pose.";
    if (!inport_pose_ext_.open("/" + ID_ + "/pose:i"))
    {
        yError() << log_ID_ << "Cannot open input port for external pose!";
        throw std::runtime_error("Cannot open input port for external pose!");
    }

    yInfo() << log_ID_ << "...ExtCADSuperimposer ctor completed!";
}


ExtCADSuperimposer::~ExtCADSuperimposer() noexcept { }


yarp::sig::Vector ExtCADSuperimposer::getEndEffectorPose()
{
    Vector* estimates_mean = inport_pose_ext_.read(synch_);

    if (estimates_mean        != YARP_NULLPTR) ext_pose_copy_ = *estimates_mean;
    if (ext_pose_copy_.size() == 0)            ext_pose_copy_ = *(inport_pose_ext_.read(true));

    return ext_pose_copy_;
}


void ExtCADSuperimposer::getExtraObjPoseMap(SuperImpose::ObjPoseMap& hand_pose)
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
