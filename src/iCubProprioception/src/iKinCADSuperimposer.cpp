#include "iCubProprioception/iKinCADSuperimposer.h"

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


iKinCADSuperimposer::iKinCADSuperimposer(const ConstString& robot, const ConstString& camera,
                                         const SICAD::ModelPathContainer& cad_hand, const ConstString& shader_path,
                                         const ConstString& port_prefix,
                                         const bool draw_thumb, const bool draw_forearm) :
    CADSuperimposer(robot, camera,
                    cad_hand, shader_path,
                    port_prefix,
                    draw_thumb, draw_forearm)
{
    yInfo() << log_ID_ << "Invoked iKinCADSuperimposer (derived class) ctor...";


    /* Get head interfaces */
    if (!setHeadRemoteControlboard())
    {
        yError() << log_ID_ << "Head remote_controlboard errored!";
        throw std::runtime_error("Head remote_controlboard errored!");
    }


    /* Get torso interfaces */
    if (!setTorsoRemoteControlboard())
    {
        yError() << log_ID_ << "Torso remote_controlboard errored!";
        throw std::runtime_error("Torso remote_controlboard errored!");
    }


    /* Get arm interfaces */
    if (!setArmRemoteControlboard())
    {
        yError() << log_ID_ << "Arm remote_controlboard errored!";
        throw std::runtime_error("Arm remote_controlboard errored!");
    }


    /* Get arm Cartesian interfaces */
    if (!setArmCartesianController())
    {
        yError() << log_ID_ << "Arm cartesiancontrollerclient errored!";
        throw std::runtime_error("Arm cartesiancontrollerclient errored!");
    }


    yInfo() << log_ID_ << "...iKinCADSuperimposer ctor completed!";
}


iKinCADSuperimposer::~iKinCADSuperimposer() noexcept { }


void iKinCADSuperimposer::onStop()
{
    CADSuperimposer::onStop();
}


void iKinCADSuperimposer::threadRelease()
{
    CADSuperimposer::threadRelease();
}


Vector iKinCADSuperimposer::getEndEffectorPose()
{
    Vector ee_pose = zeros(7);
    Vector ee_x(3);
    Vector ee_o(4);

    if (!itf_right_arm_cart_->getPose(ee_x, ee_o))
        return ee_pose;

    ee_pose.setSubvector(0, ee_x);
    ee_pose.setSubvector(3, ee_o);

    return ee_pose;
}


Vector iKinCADSuperimposer::getHeadEncoders()
{
    Vector enc_head = zeros(1);

    int enc_num;
    if (!itf_head_encoders_->getAxes(&enc_num))
        return enc_head;

    enc_head = zeros(enc_num);
    if (!itf_head_encoders_->getEncoders(enc_head.data()))
    {
        enc_head.zero();
        return enc_head;
    }

    return enc_head;
}


Vector iKinCADSuperimposer::getRightArmEncoders()
{
    Vector enc_arm = zeros(1);

    int enc_num;
    if (!itf_right_arm_encoders_->getAxes(&enc_num))
        return enc_arm;

    enc_arm = zeros(enc_num);
    if (!itf_right_arm_encoders_->getEncoders(enc_arm.data()))
    {
        enc_arm.zero();
        return enc_arm;
    }

    return enc_arm;
}


#if ICP_USE_ANALOGS == 1
Vector iKinCADSuperimposer::getRightHandAnalogs()
{
    Vector analog_arm = zeros(1);

    int enc_num;
    if (!itf_right_hand_analog_->getAxes(&enc_num))
        return analog_arm;

    analog_arm = zeros(enc_num);
    if (!itf_right_hand_analog_->getEncoders(analog_arm.data()))
    {
        analog_arm.zero();
        return analog_arm;
    }

    return analog_arm;
}
#endif


Vector iKinCADSuperimposer::getTorsoEncoders()
{
    Vector enc_torso = zeros(1);

    int enc_num;
    if (!itf_torso_encoders_->getAxes(&enc_num))
        return enc_torso;

    enc_torso = zeros(enc_num);
    if (!itf_torso_encoders_->getEncoders(enc_torso.data()))
    {
        enc_torso.zero();
        return enc_torso;
    }

    std::swap(enc_torso(0), enc_torso(2));

    return enc_torso;
}


bool iKinCADSuperimposer::setHeadRemoteControlboard()
{
    Property head_remote_options;
    head_remote_options.put("device", "remote_controlboard");
    head_remote_options.put("local", "/" + ID_ + "/cam/" + camera_ + "/control_head");
    head_remote_options.put("remote", "/" + robot_ + "/head");

    drv_head_remote_.open(head_remote_options);
    if (drv_head_remote_.isValid())
    {
        yInfo() << log_ID_ << "Head remote_controlboard succefully opened.";

        drv_head_remote_.view(itf_head_encoders_);
        if (!itf_head_encoders_)
        {
            yError() << log_ID_ << "Error getting Head IEncoders interface in thread.";
            throw std::runtime_error("Error getting Head IEncoders interface in thread.");
        }

        return true;
    }
    else
    {
        yError() << log_ID_ << "Error opening Head remote_controlboard device.";
        return false;
    }
}


bool iKinCADSuperimposer::setTorsoRemoteControlboard()
{
    Property torso_remote_options;
    torso_remote_options.put("device", "remote_controlboard");
    torso_remote_options.put("local", "/" + ID_ + "/cam/" + camera_ + "/control_torso");
    torso_remote_options.put("remote", "/" + robot_ + "/torso");

    drv_torso_remote_.open(torso_remote_options);
    if (drv_torso_remote_.isValid())
    {
        yInfo() << log_ID_ << "Torso remote_controlboard succefully opened.";

        drv_torso_remote_.view(itf_torso_encoders_);
        if (!itf_torso_encoders_)
        {
            yError() << log_ID_ << "Error getting torso IEncoders interface in thread.";
            throw std::runtime_error("Error getting torso IEncoders interface in thread.");
        }

        return true;
    }
    else
    {
        yError() << log_ID_ << "Error opening Torso remote_controlboard device.";
        return false;
    }
}


bool iKinCADSuperimposer::setArmRemoteControlboard()
{
    Property rightarm_remote_options;
    rightarm_remote_options.put("device", "remote_controlboard");
    rightarm_remote_options.put("local", "/" + ID_ + "/cam/" + camera_ + "/control_right_arm");
    rightarm_remote_options.put("remote", "/" + robot_ + "/right_arm");

    drv_right_arm_remote_.open(rightarm_remote_options);
    if (drv_right_arm_remote_.isValid())
    {
        yInfo() << log_ID_ << "Right arm remote_controlboard succefully opened.";

        drv_right_arm_remote_.view(itf_right_arm_encoders_);
        if (!itf_right_arm_encoders_)
        {
            yError() << log_ID_ << "Error getting right arm IEncoders interface.";
            return false;
        }
    }
    else
    {
        yError() << log_ID_ << "Error opening right arm remote_controlboard device.";
        return false;
    }

#if ICP_USE_ANALOGS == 1
    Property righthand_remote_analog;
    righthand_remote_analog.put("device", "analogsensorclient");
    righthand_remote_analog.put("local",  "/" + ID_ + "/cam/" + camera_ + "/right_hand");
    righthand_remote_analog.put("remote", "/" + robot_ + "/right_hand/analog:o");

    drv_right_hand_analog_.open(righthand_remote_analog);
    if (drv_right_hand_analog_.isValid())
    {
        yInfo() << log_ID_ << "Right arm analogsensorclient succefully opened.";

        if (!drv_right_hand_analog_.view(itf_right_hand_analog_))
        {
            yError() << log_ID_ << "Error getting right hand IAnalogSensor interface!";
            throw std::runtime_error("Error getting right hand IAnalogSensor interface!");
        }
    }
    else
    {
        yError() << log_ID_ << "Error opening right arm analogsensorclient device.";
        return false;
    }
#endif

    return true;
}


bool iKinCADSuperimposer::setArmCartesianController()
{
    Property rightarm_cartesian_options;
    rightarm_cartesian_options.put("device", "cartesiancontrollerclient");
    rightarm_cartesian_options.put("local", "/" + ID_ + "/cam/" + camera_ + "/cart_right_arm");
    rightarm_cartesian_options.put("remote", "/" + robot_ + "/cartesianController/right_arm");

    drv_right_arm_cartesian_.open(rightarm_cartesian_options);
    if (drv_right_arm_cartesian_.isValid())
    {
        drv_right_arm_cartesian_.view(itf_right_arm_cart_);
        if (!itf_right_arm_cart_)
        {
            yError() << log_ID_ << "Error getting ICartesianControl interface.";
            return false;
        }
        yInfo() << log_ID_ << "cartesiancontrollerclient succefully opened.";
    }
    else
    {
        yError() << log_ID_ << "Error opening cartesiancontrollerclient device.";
        return false;
    }

    return true;
}


Vector iKinCADSuperimposer::readRootToEE()
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
