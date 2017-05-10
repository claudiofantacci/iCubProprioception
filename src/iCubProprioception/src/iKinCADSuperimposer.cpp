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


iKinCADSuperimposer::iKinCADSuperimposer(const ConstString& project_name, const ConstString& robot, const ConstString& camera,
                                 const SuperImpose::ObjFileMap& cad_hand, const ConstString& shader_path) :
    ID_(project_name + "/iKinCADSuperimposer"), log_ID_("[" + ID_ + "]"),
    robot_(robot), camera_(camera), camsel_((camera == "left")? 0:1),
    cad_hand_(cad_hand), shader_path_(shader_path)
{
    yInfo() << log_ID_ << "Initializing hand CAD drawing thread...";


    /* Get torso interfaces */
    if (!setTorsoRemoteControlboard())
    {
        yError() << log_ID_ << "Torso remote_controlboard errored!";
        throw std::runtime_error("remote_controlboard errored!");
    }

    /* Get arm interfaces */
    if (!setArmRemoteControlboard())
    {
        yError() << log_ID_ << "Arm remote_controlboard errored!";
        throw std::runtime_error("remote_controlboard errored!");
    }

    /* Get arm Cartesian interfaces */
    if (!setArmCartesianController())
    {
        yError() << log_ID_ << "Arm cartesiancontrollerclient errored!";
        throw std::runtime_error("cartesiancontrollerclient errored!");
    }

    /* Get gaze interface */
    if (!setGazeController())
    {
        yError() << log_ID_ << "Head gazecontrollerclient errored!";
        throw std::runtime_error("gazecontrollerclient errored!");
    }


    /* Port for the CAD superimposer to work */
    yInfo() << log_ID_ << "Opening ports for CAD images.";
    if (!inport_renderer_img_.open("/" + ID_ + "/cam/" + camera_ + ":i"))
    {
        yError() << log_ID_ << "Cannot open input image port for " + camera_ + " camera!";
        throw std::runtime_error("Cannot open input image port for " + camera_ + " camera!");
    }

    if (!outport_renderer_img_.open("/" + ID_ + "/cam/" + camera_ + ":o"))
    {
        yError() << log_ID_ << "Cannot open output image port for " + camera_ + " camera!";
        throw std::runtime_error("Cannot open output image port for " + camera_ + " camera!");
    }


    /* Get camera info */
    Bottle btl_cam_left_info;
    itf_head_gaze_->getInfo(btl_cam_left_info);
    yInfo() << log_ID_ << "[CAM INFO]" << btl_cam_left_info.toString();
    Bottle* cam_left_info = btl_cam_left_info.findGroup("camera_intrinsics_left").get(1).asList();
    cam_width_  = btl_cam_left_info.findGroup("camera_width_" + camera_).get(1).asInt();
    cam_height_ = btl_cam_left_info.findGroup("camera_height_" + camera_).get(1).asInt();
    cam_fx_     = static_cast<float>(cam_left_info->get(0).asDouble());
    cam_cx_     = static_cast<float>(cam_left_info->get(2).asDouble());
    cam_fy_     = static_cast<float>(cam_left_info->get(5).asDouble());
    cam_cy_     = static_cast<float>(cam_left_info->get(6).asDouble());


    /* Initialize right hand finger interfaces */
    yInfo() << log_ID_ << "Setting fingers.";
    right_finger_[0] = iCubFinger("right_thumb");
    right_finger_[1] = iCubFinger("right_index");
    right_finger_[2] = iCubFinger("right_middle");

    std::deque<IControlLimits*> temp_lim;
    temp_lim.push_front(itf_right_fingers_limits_);
    for (int i = 0; i < 3; ++i)
    {
        if (!right_finger_[i].alignJointsBounds(temp_lim))
        {
            yError() << log_ID_ << "Cannot set joint bound for finger " + std::to_string(i) + ".";
            throw std::runtime_error("Cannot set joint bound for finger " + std::to_string(i) + ".");
        }
    }


    /* Initialize right arm interface */
    yInfo() << log_ID_ << "Setting arms.";
    right_arm_ = iCubArm("right");
    right_arm_.setAllConstraints(false);
    right_arm_.releaseLink(0);
    right_arm_.releaseLink(1);
    right_arm_.releaseLink(2);


    /* Initialize CAD superimposer */
    yInfo() << log_ID_ << "Setting up OpenGL drawer.";
    drawer_ = new SICAD(cad_hand_, cam_width_, cam_height_, 1, shader_path_,
                        cam_fx_, cam_fy_, cam_cx_, cam_cy_);
    drawer_->setBackgroundOpt(true);
    drawer_->setWireframeOpt(true);
    

    /* Open command port */
    if (!setCommandPort()) throw std::runtime_error("Cannot attach the command port.");

    
    yInfo() << log_ID_ << "...initialization completed!";
}


iKinCADSuperimposer::~iKinCADSuperimposer() noexcept
{
    delete drawer_;
}


void iKinCADSuperimposer::run()
{
    Vector ee_x (3);
    Vector ee_o (4);
    Vector cam_x(3);
    Vector cam_o(4);

    while (!isStopping())
    {
        ImageOf<PixelRgb>* imgin = inport_renderer_img_.read(true);

        if (imgin != NULL)
        {
            itf_right_arm_cart_->getPose(ee_x, ee_o);

            itf_head_gaze_->getLeftEyePose(cam_x, cam_o);

            Matrix Ha = axis2dcm(ee_o);
            ee_x.push_back(1.0);
            Ha.setCol(3, ee_x);

            Vector encs_arm(static_cast<size_t>(num_right_arm_enc_));
            itf_right_arm_encoders_->getEncoders(encs_arm.data());
            yAssert(encs_arm.size() == 16);

//            encs_arm(7) = 32.0;
//            encs_arm(8) = 30.0;
//            encs_arm(9) = 0.0;
//            encs_arm(10) = 0.0;
//            encs_arm(11) = 0.0;
//            encs_arm(12) = 0.0;
//            encs_arm(13) = 0.0;
//            encs_arm(14) = 0.0;

#if ICP_USE_ANALOGS == 1
            Vector analogs;
            itf_right_hand_analog_->read(analogs);
            yAssert(analogs.size() >= 15);
#endif

            Vector chainjoints;
            for (unsigned int i = 0; i < 3; ++i)
            {
#if ICP_USE_ANALOGS == 1
                right_finger_[i].getChainJoints(encs_arm, analogs, chainjoints);
#else
                right_finger_[i].getChainJoints(encs_arm, chainjoints);
#endif

                right_finger_[i].setAng(CTRL_DEG2RAD * chainjoints);
            }

            Vector encs_torso(3);
            itf_torso_encoders_->getEncoders(encs_torso.data());
            yAssert(encs_torso.size() == 3);
            Vector encs_tot(10);
            encs_tot(0) = encs_torso(2);
            encs_tot(1) = encs_torso(1);
            encs_tot(2) = encs_torso(0);
            encs_tot.setSubvector(3, encs_arm.subVector(0, 6));
            right_arm_.setAng(CTRL_DEG2RAD * encs_tot);

            SuperImpose::ObjPoseMap hand_pose;
            SuperImpose::ObjPose    pose;
            pose.assign(ee_x.data(), ee_x.data()+3);
            pose.insert(pose.end(), ee_o.data(), ee_o.data()+4);
            hand_pose.emplace("palm", pose);
            for (size_t fng = 0; fng < 3; ++fng)
            {
                std::string finger_s;
                pose.clear();
                if (fng != 0)
                {
                    Vector j_x = (Ha * (right_finger_[fng].getH0().getCol(3))).subVector(0, 2);
                    Vector j_o = dcm2axis(Ha * right_finger_[fng].getH0());

                    if      (fng == 1) { finger_s = "index0"; }
                    else if (fng == 2) { finger_s = "medium0"; }

                    pose.assign(j_x.data(), j_x.data()+3);
                    pose.insert(pose.end(), j_o.data(), j_o.data()+4);
                    hand_pose.emplace(finger_s, pose);
                }

                for (size_t i = 0; i < right_finger_[fng].getN(); ++i)
                {
                    Vector j_x = (Ha * (right_finger_[fng].getH(i, true).getCol(3))).subVector(0, 2);
                    Vector j_o = dcm2axis(Ha * right_finger_[fng].getH(i, true));

                    if      (fng == 0) { finger_s = "thumb"+std::to_string(i+1); }
                    else if (fng == 1) { finger_s = "index"+std::to_string(i+1); }
                    else if (fng == 2) { finger_s = "medium"+std::to_string(i+1); }

                    pose.assign(j_x.data(), j_x.data()+3);
                    pose.insert(pose.end(), j_o.data(), j_o.data()+4);
                    hand_pose.emplace(finger_s, pose);
                }
            }
            Matrix H_forearm = right_arm_.getH(7, true);
            Vector j_x = H_forearm.getCol(3).subVector(0, 2);
            Vector j_o = dcm2axis(H_forearm);
            pose.clear();
            pose.assign(j_x.data(), j_x.data()+3);
            pose.insert(pose.end(), j_o.data(), j_o.data()+4);
            hand_pose.emplace("forearm", pose);

            cv::Mat img = cv::cvarrToMat(imgin->getIplImage(), true);
            drawer_->superimpose(hand_pose, cam_x.data(), cam_o.data(), img);
        }
    }
}


void iKinCADSuperimposer::onStop()
{
    inport_renderer_img_.interrupt();
}


void iKinCADSuperimposer::threadRelease()
{
    yInfo() << log_ID_ << "Deallocating resource...";

    outport_renderer_img_.interrupt();

    if (!inport_renderer_img_.isClosed())  inport_renderer_img_.close();
    if (!outport_renderer_img_.isClosed()) outport_renderer_img_.close();

    if (port_command_.isOpen()) port_command_.close();

    yInfo() << log_ID_ << "Deallocation completed!";
}


bool iKinCADSuperimposer::setCommandPort()
{
    yInfo() << log_ID_ << "Opening command port.";
    if (!port_command_.open("/" + ID_ + "/render/rpc"))
    {
        yError() << log_ID_ << "Cannot open /" + ID_ + "/render/rpc port.";
        return false;
    }
    if (!this->yarp().attachAsServer(port_command_))
    {
        yError() << log_ID_ << "Cannot attach the renderer RPC port.";
        return false;
    }
    yInfo() << log_ID_ << "Renderer RPC port succesfully opened and attached. Ready to recieve commands.";
    
    return true;
}


bool iKinCADSuperimposer::setTorsoRemoteControlboard()
{
    Property torso_remote_options;
    torso_remote_options.put("device", "remote_controlboard");
    torso_remote_options.put("local", "/" + ID_ + "/control_torso");
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
    rightarm_remote_options.put("local", "/" + ID_ + "/control_right_arm");
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

        itf_right_arm_encoders_->getAxes(&num_right_arm_enc_);
        yInfo() << log_ID_ << "Right arm encorders succefully read.";

        drv_right_arm_remote_.view(itf_right_fingers_limits_);
        if (!itf_right_fingers_limits_)
        {
            yError() << log_ID_ << "Error getting fingers IControlLimits interface in thread!";
            throw std::runtime_error("Error getting fingers IControlLimits interface in thread!");
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
    righthand_remote_analog.put("local",  "/" + ID_ + "/right_hand");
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
    rightarm_cartesian_options.put("local", "/" + ID_ + "/cart_right_arm");
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


bool iKinCADSuperimposer::setGazeController()
{
    Property gaze_option;
    gaze_option.put("device", "gazecontrollerclient");
    gaze_option.put("local", "/" + ID_ + "/gaze");
    gaze_option.put("remote", "/iKinGazeCtrl");

    drv_gaze_.open(gaze_option);
    if (drv_gaze_.isValid())
    {
        drv_gaze_.view(itf_head_gaze_);
        if (!itf_head_gaze_)
        {
            yError() << log_ID_ << "Error getting IGazeControl interface.";
            return false;
        }
    }
    else
    {
        yError() << log_ID_ << "Gaze control device not available.";
        return false;
    }

    return true;
}


bool iKinCADSuperimposer::mesh_background(const bool status)
{
    yInfo() << log_ID_ << ConstString((status ? "Enable" : "Disable")) + " background of the mesh window.";

    drawer_->setBackgroundOpt(status);

    return true;
}


bool iKinCADSuperimposer::mesh_wireframe(const bool status)
{
    yInfo() << log_ID_ << ConstString((status ? "Enable" : "Disable")) + " wireframe rendering.";

    drawer_->setWireframeOpt(status);

    return true;
}


bool iKinCADSuperimposer::sync_input(const bool status)
{
    return false;
}
