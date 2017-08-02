#include "iCubProprioception/BatchCADSuperimposer.h"

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


BatchCADSuperimposer::BatchCADSuperimposer(const ConstString& port_prefix, const ConstString& robot, const ConstString& camera,
                                           const SICAD::ModelPathContainer& cad_hand, const ConstString& shader_path) :
    CADSuperimposer(port_prefix, robot, camera, cad_hand, shader_path)
{
    yInfo() << log_ID_ << "Invoked BatchCADSuperimposer (derived class) ctor...";


    /* Port for the external pose */
    if(!inport_pose_ext_.open("/" + ID_ + "/cam/" + camera_ + "/hand/right_pose:i"))
    {
        yError() << log_ID_ << "Cannot open input port for external pose!";
        throw std::runtime_error("Cannot open input port for external pose!");
    }
    

    /* Port for the head encoders */
    if(!inport_head_enc_.open("/" + ID_ + "/cam/" + camera_ + "/head:i"))
    {
        yError() << log_ID_ << "Cannot open input port for head encoders!";
        throw std::runtime_error("Cannot open input port for head encoders!");
    }


    /* Port for the right arm encoders */
    if(!inport_right_arm_enc_.open("/" + ID_ + "/cam/" + camera_ + "/right_arm:i"))
    {
        yError() << log_ID_ << "Cannot open input port for right arm encoders!";
        throw std::runtime_error("Cannot open input port for right arm encoders!");
    }


    /* Port for the torso encoders */
    if(!inport_torso_enc_.open("/" + ID_ + "/cam/" + camera_ + "/torso:i"))
    {
        yError() << log_ID_ << "Cannot open input port for torso encoders!";
        throw std::runtime_error("Cannot open input port for torso encoders!");
    }


    /* Port for the right hand analogs */
    if(!inport_right_hand_analogs_.open("/" + ID_ + "/cam/" + camera_ + "/right_analogs:i"))
    {
        yError() << log_ID_ << "Cannot open input port for right hand analogs!";
        throw std::runtime_error("Cannot open input port for right hand analogs!");
    }


    /* Initialize right arm interface */
    yInfo() << log_ID_ << "Setting right arm interface.";

    right_arm_ = iCubArm("right");

    right_arm_.setAllConstraints(false);
    right_arm_.releaseLink(0);
    right_arm_.releaseLink(1);
    right_arm_.releaseLink(2);


    yInfo() << log_ID_ << "...BatchCADSuperimposer ctor completed!";
}


BatchCADSuperimposer::~BatchCADSuperimposer() noexcept { }


void BatchCADSuperimposer::threadRelease()
{
    if (!inport_pose_ext_.isClosed())           inport_pose_ext_.close();
    if (!inport_head_enc_.isClosed())           inport_head_enc_.close();
    if (!inport_torso_enc_.isClosed())          inport_torso_enc_.close();
    if (!inport_right_arm_enc_.isClosed())      inport_right_arm_enc_.close();
    if (!inport_right_hand_analogs_.isClosed()) inport_right_hand_analogs_.close();

    CADSuperimposer::threadRelease();
}


Vector BatchCADSuperimposer::getEndEffectorPose()
{
    return readFromBufferedPort(inport_pose_ext_);
}


Vector BatchCADSuperimposer::getHeadEncoders()
{
    return readFromBufferedPort(inport_head_enc_);
}


Vector BatchCADSuperimposer::getRightArmEncoders()
{
    return readFromBufferedPort(inport_right_arm_enc_);
}


#if ICP_USE_ANALOGS == 1
Vector BatchCADSuperimposer::getRightHandAnalogs()
{
    return readFromBufferedPort(inport_right_hand_analogs_);
}
#endif


Vector BatchCADSuperimposer::getTorsoEncoders()
{
    Vector enc_torso = readFromBufferedPort(inport_torso_enc_);

    if (enc_torso == zeros(enc_torso.size()))
        return enc_torso;

    std::swap(enc_torso(0), enc_torso(2));

    return enc_torso;
}


void BatchCADSuperimposer::getExtraObjPoseMap(Superimpose::ModelPoseContainer& hand_pose)
{
    if (view_forearm_)
    {
        Vector root_ee_enc = readRootToEE();
        if (root_ee_enc == zeros(root_ee_enc.size()))
            return;

        Superimpose::ModelPose pose;

        right_arm_.setAng(CTRL_DEG2RAD * root_ee_enc);

        Matrix H_forearm = right_arm_.getH(7, true);

        Vector j_x = H_forearm.getCol(3).subVector(0, 2);
        Vector j_o = dcm2axis(H_forearm);

        pose.assign(j_x.data(), j_x.data()+3);
        pose.insert(pose.end(), j_o.data(), j_o.data()+4);
        
        hand_pose.emplace("forearm", pose);
    }
}


bool BatchCADSuperimposer::sync_input(const bool status)
{
    synch_ = status;

    return true;
}


bool BatchCADSuperimposer::render_extra_mesh(const bool status)
{
    view_forearm_ = status;

    return true;
}


Vector BatchCADSuperimposer::readRootToEE()
{
    Vector root_ee_enc = zeros(10);

    Vector enc_arm = getRightArmEncoders();
    if (enc_arm == zeros(enc_arm.size()))
        return root_ee_enc;

    Vector enc_torso = getTorsoEncoders();
    if (enc_torso == zeros(enc_torso.size()))
        return enc_torso;

    root_ee_enc.setSubvector(0, enc_torso);

    for (size_t i = 0; i < 7; ++i)
        root_ee_enc(i+3) = enc_arm(i);

    return root_ee_enc;
}


Vector BatchCADSuperimposer::readFromBufferedPort(BufferedPort<Vector>& bp)
{
    Vector* data = bp.read(synch_);

    if (data != YARP_NULLPTR) return *data;
    else                      return zeros(1);
}
