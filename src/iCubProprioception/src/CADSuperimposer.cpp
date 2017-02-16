#include "iCubProprioception/CADSuperimposer.h"
#include "iCubProprioception/common.h"

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


CADSuperimposer::CADSuperimposer(const ConstString& project_name,
                                 const ConstString& laterality,
                                 const ConstString& camera,
                                 PolyDriver& torso_remote_driver,
                                 PolyDriver& arm_remote_driver,
                                 PolyDriver& arm_cartesian_driver,
                                 PolyDriver& gaze_driver,
                                 PolyDriver& drv_right_hand_analog,
                                 const SuperImpose::ObjFileMap& cad_hand) :
    log_ID_("[CADSuperimposer]"), project_name_(project_name), laterality_(laterality), camera_(camera), camsel_((camera == "left")? 0:1), torso_remote_driver_(torso_remote_driver), arm_remote_driver_(arm_remote_driver), arm_cartesian_driver_(arm_cartesian_driver), drv_right_hand_analog_(drv_right_hand_analog), gaze_driver_(gaze_driver), cad_hand_(cad_hand)
{
    yInfo() << log_ID_ << "Initializing hand CAD drawing thread...";

    yInfo() << log_ID_ << "Setting interfaces...";
    // TODO: standardizzare la gestione degli errori con una funzione.
    arm_cartesian_driver_.view(itf_arm_cart_);
    if (!itf_arm_cart_)
    {
        yError() << log_ID_ << "Error getting arm ICartesianControl interface in thread.";
        throw std::runtime_error("Error getting arm ICartesianControl interface in thread.");
    }

    gaze_driver_.view(itf_head_gaze_);
    if (!itf_head_gaze_)
    {
        yError() << log_ID_ << "Error getting head IGazeControl interface!";
        throw std::runtime_error("Error getting head IGazeControl interface!");
    }

    IControlLimits *itf_fingers_lim;
    arm_remote_driver_.view(itf_fingers_lim);
    if (!itf_fingers_lim)
    {
        yError() << log_ID_ << "Error getting fingers IControlLimits interface in thread!";
        throw std::runtime_error("Error getting fingers IControlLimits interface in thread!");
    }

    torso_remote_driver_.view(itf_torso_encoders_);
    if (!itf_torso_encoders_)
    {
        yError() << log_ID_ << "Error getting torso IEncoders interface in thread.";
        throw std::runtime_error("Error getting torso IEncoders interface in thread.");
    }

    arm_remote_driver_.view(itf_arm_encoders_);
    if (!itf_arm_encoders_)
    {
        yError() << log_ID_ << "Error getting arm IEncoders interface!";
        throw std::runtime_error("Error getting arm IEncoders interface!");
    }
    itf_arm_encoders_->getAxes(&num_arm_enc_);
    yInfo() << log_ID_ << "Interfaces set!";


    yInfo() << log_ID_ << "Opening ports for CAD images...";
    if (!inport_renderer_img_.open("/"+project_name_+"/cad/cam/"+camera_+":i"))
    {
        yError() << log_ID_ << "Cannot open input image port for "+camera_+" camera!";
        throw std::runtime_error("Cannot open input image port for "+camera_+" camera!");
    }

    if (!outport_renderer_img_.open("/"+project_name_+"/cad/cam/"+camera_+":o"))
    {
        yError() << log_ID_ << "Cannot open output image port for "+camera_+" camera!";
        throw std::runtime_error("Cannot open output image port for "+camera_+" camera!");
    }
    yInfo() << log_ID_ << "CAD image ports succesfully opened!";


    yInfo() << log_ID_ << "Opening ports for "+camera_+" camera pose...";
    if (!port_cam_pose_.open("/"+project_name_+"/cad/"+camera_+"/pose:o"))
    {
        yError() << log_ID_ << "Cannot open "+camera_+" camera pose output port!";
        throw std::runtime_error("Cannot open "+camera_+" camera pose output port!");
    }
    yInfo() << log_ID_ << "Port for "+camera_+" camera succesfully opened!";

#if ICP_USE_ANALOGS == 1
    if (!drv_right_hand_analog_.view(itf_right_hand_analog_))
    {
        yError() << log_ID_ << "Error getting right hand IAnalogSensor interface!";
        throw std::runtime_error("Error getting right hand IAnalogSensor interface!");
    }
#endif

    // FIXME: far diventare la camera parametrica utilizzando CAMERA e rinominare le variabili
    Bottle btl_cam_left_info;
    itf_head_gaze_->getInfo(btl_cam_left_info);
    yInfo() << log_ID_ << "[CAM INFO]" << btl_cam_left_info.toString();
    cam_width_  = 320;
    cam_height_ = 240;
    Bottle * cam_left_info = btl_cam_left_info.findGroup("camera_intrinsics_left").get(1).asList();
    eye_fx_ = static_cast<float>(cam_left_info->get(0).asDouble());
    eye_cx_ = static_cast<float>(cam_left_info->get(2).asDouble());
    eye_fy_ = static_cast<float>(cam_left_info->get(5).asDouble());
    eye_cy_ = static_cast<float>(cam_left_info->get(6).asDouble());


    yInfo() << log_ID_ << "Setting "+laterality_+" fingers...";
    finger_[0] = iCubFinger(laterality_+"_thumb");
    finger_[1] = iCubFinger(laterality_+"_index");
    finger_[2] = iCubFinger(laterality_+"_middle");

    //    std::deque<IControlLimits*> temp_lim;
    //    temp_lim.push_front(itf_fingers_lim);
    //    for (int i = 0; i < 3; ++i)
    //    {
    //        if (!finger_[i].alignJointsBounds(temp_lim))
    //        {
    //            yError() << log_ID_ << "Cannot set joint bound for finger " + std::to_string(i) + ".";
    //            return false;
    //        }
    //    }
    yInfo() << log_ID_ << "Finger succesfully set!";


    yInfo() << log_ID_ << "Setting "+laterality_+" arm...";
    arm_ = iCubArm(laterality_);
    arm_.setAllConstraints(false);
    arm_.releaseLink(0);
    arm_.releaseLink(1);
    arm_.releaseLink(2);
    yInfo() << log_ID_ << "Arm set!";


    yInfo() << log_ID_ << "Setting up OpenGL drawer...";
    drawer_ = new SICAD(cad_hand_, cam_width_, cam_height_, eye_fx_, eye_fy_, eye_cx_, eye_cy_);
    drawer_->setBackgroundOpt(true);
    drawer_->setWireframeOpt(true);
    yInfo() << log_ID_ << "OpenGL drawer succesfully set!";
    
    
    if (!setCommandPort()) throw std::runtime_error("Cannot attach the command port.");
    yInfo() << log_ID_ << "Initialization completed!";
}


CADSuperimposer::~CADSuperimposer() noexcept
{
    delete drawer_;
}


bool CADSuperimposer::initOGL(const GLsizei width, const GLsizei height, const GLint view)
{
    return SICAD::initOGL(width, height, view);
}


void CADSuperimposer::run()
{
    Vector ee_x (3);
    Vector ee_o (4);
    Vector cam_x(3);
    Vector cam_o(4);

    while (!isStopping())
    {
        ImageOf<PixelRgb> * imgin = inport_renderer_img_.read(true);

        if (imgin != NULL)
        {
            itf_arm_cart_->getPose(ee_x, ee_o);

            itf_head_gaze_->getLeftEyePose(cam_x, cam_o);

            Matrix Ha = axis2dcm(ee_o);
            ee_x.push_back(1.0);
            Ha.setCol(3, ee_x);

            Vector encs_arm(static_cast<size_t>(num_arm_enc_));
            itf_arm_encoders_->getEncoders(encs_arm.data());
            yAssert(encs_arm.size() == 16);

#if ICP_USE_ANALOGS == 1
            Vector analogs;
            itf_right_hand_analog_->read(analogs);
            yAssert(analogs.size() >= 15);
#endif

            Vector chainjoints;
            for (unsigned int i = 0; i < 3; ++i)
            {
#if ICP_USE_ANALOGS == 1
                finger_[i].getChainJoints(encs_arm, analogs, chainjoints);
#else
                finger_[i].getChainJoints(encs_arm, chainjoints);
#endif

                finger_[i].setAng(CTRL_DEG2RAD * chainjoints);
            }
            Vector encs_torso(3);
            itf_torso_encoders_->getEncoders(encs_torso.data());
            yAssert(encs_torso.size() == 3);
            Vector encs_tot(10);
            encs_tot(0) = encs_torso(2);
            encs_tot(1) = encs_torso(1);
            encs_tot(2) = encs_torso(0);
            encs_tot.setSubvector(3, encs_arm.subVector(0, 6));
            arm_.setAng(CTRL_DEG2RAD * encs_tot);

            SuperImpose::ObjPoseMap hand_pose;
            SuperImpose::ObjPose    pose;
            pose.assign(ee_x.data(), ee_x.data()+3);
            pose.insert(pose.end(), ee_o.data(), ee_o.data()+4);
            hand_pose.emplace("palm", pose);
            for (unsigned int fng = 0; fng < 3; ++fng)
            {
                std::string finger_s;
                pose.clear();
                if (fng != 0)
                {
                    Vector j_x = (Ha * (finger_[fng].getH0().getCol(3))).subVector(0, 2);
                    Vector j_o = dcm2axis(Ha * finger_[fng].getH0());

                    if      (fng == 1) { finger_s = "index0"; }
                    else if (fng == 2) { finger_s = "medium0"; }

                    pose.assign(j_x.data(), j_x.data()+3);
                    pose.insert(pose.end(), j_o.data(), j_o.data()+4);
                    hand_pose.emplace(finger_s, pose);
                }

                for (unsigned int i = 0; i < finger_[fng].getN(); ++i)
                {
                    Vector j_x = (Ha * (finger_[fng].getH(i, true).getCol(3))).subVector(0, 2);
                    Vector j_o = dcm2axis(Ha * finger_[fng].getH(i, true));

                    if      (fng == 0) { finger_s = "thumb"+std::to_string(i+1); }
                    else if (fng == 1) { finger_s = "index"+std::to_string(i+1); }
                    else if (fng == 2) { finger_s = "medium"+std::to_string(i+1); }

                    pose.assign(j_x.data(), j_x.data()+3);
                    pose.insert(pose.end(), j_o.data(), j_o.data()+4);
                    hand_pose.emplace(finger_s, pose);
                }
            }
            Matrix H_forearm = arm_.getH(7, true);
            Vector j_x = H_forearm.getCol(3).subVector(0, 2);
            Vector j_o = dcm2axis(H_forearm);
            pose.clear();
            pose.assign(j_x.data(), j_x.data()+3);
            pose.insert(pose.end(), j_o.data(), j_o.data()+4);
            hand_pose.emplace("forearm", pose);

            cv::Mat img = cv::cvarrToMat(imgin->getIplImage(), true);
            drawer_->superimpose(hand_pose, cam_x.data(), cam_o.data(), img);

            ImageOf<PixelRgb>& imgout = outport_renderer_img_.prepare();
            imgout.setExternal(img.data, img.cols, img.rows);

            Bottle &camPoseBottle = port_cam_pose_.prepare();
            camPoseBottle.clear();
            camPoseBottle.addString(cam_x.toString() + " " + cam_o.toString());

            outport_renderer_img_.write();
            port_cam_pose_.write();
        }
    }
}


void CADSuperimposer::onStop()
{
    inport_renderer_img_.interrupt();
}


void CADSuperimposer::threadRelease()
{
    yInfo() << log_ID_ << "Deallocating resource...";

    outport_renderer_img_.interrupt();
    port_cam_pose_.interrupt();

    if (!inport_renderer_img_.isClosed())  inport_renderer_img_.close();
    if (!outport_renderer_img_.isClosed()) outport_renderer_img_.close();
    if (!port_cam_pose_.isClosed())        port_cam_pose_.close();

    if (port_command_.isOpen()) port_command_.close();

    yInfo() << log_ID_ << "Deallocation completed!";
}


bool CADSuperimposer::setCommandPort()
{
    //    yInfo() << log_ID_ << "Opening command port.";
    //    if (!port_command.open("/"+project_name_+"/cad/cmd"))
    //    {
    //        yError() << log_ID_ << "Cannot open the command port.";
    //        return false;
    //    }
    //    if (!helper.yarp().attachAsServer(port_command))
    //    {
    //        yError() << log_ID_ << "Cannot attach the command port.";
    //        return false;
    //    }
    //    yInfo() << log_ID_ << "Command port and thread helper succesfully opened and attached. Ready to recieve commands.";
    
    return true;
}
