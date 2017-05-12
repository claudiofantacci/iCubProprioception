#include "iCubProprioception/CADSuperimposer.h"

#include <exception>
#include <utility>

#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace iCub::iKin;


std::mutex    CADSuperimposer::mtx_gaze_;
PolyDriver    CADSuperimposer::drv_gaze_;
IGazeControl* CADSuperimposer::itf_gaze_ = YARP_NULLPTR;


CADSuperimposer::CADSuperimposer(const ConstString& project_name, const ConstString& robot, const ConstString& camera,
                                 const SuperImpose::ObjFileMap& cad_hand, const ConstString& shader_path) :
    ID_(project_name + "/CADSuperimposer"), log_ID_("[" + ID_ + "]"),
    robot_(robot), camera_(camera),
    cad_hand_(cad_hand), shader_path_(shader_path)
{
    yInfo() << log_ID_ << "Invoked CADSuperimposer (base class) ctor...";


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


    /* Setting up common gaze controller and camera parameters */
    if (openGazeController())
    {
        std::lock_guard<std::mutex> lock(mtx_gaze_);

        Bottle cam_info;
        itf_gaze_->getInfo(cam_info);
        yInfo() << log_ID_ << "[CAM PARAMS]" << cam_info.toString();
        Bottle* cam_sel_intrinsic  = cam_info.findGroup("camera_intrinsics_" + camera_).get(1).asList();
        cam_width_  = cam_info.findGroup("camera_width_" + camera_).get(1).asInt();
        cam_height_ = cam_info.findGroup("camera_height_" + camera_).get(1).asInt();
        cam_fx_     = static_cast<float>(cam_sel_intrinsic->get(0).asDouble());
        cam_cx_     = static_cast<float>(cam_sel_intrinsic->get(2).asDouble());
        cam_fy_     = static_cast<float>(cam_sel_intrinsic->get(5).asDouble());
        cam_cy_     = static_cast<float>(cam_sel_intrinsic->get(6).asDouble());
    }
    else
    {
        yWarning() << log_ID_ << "[CAM PARAMS]" << "No intrinisc camera information could be found by the ctor. Looking for fallback values in parameters.ini.";

        ResourceFinder rf;
        rf.setVerbose();
        rf.setDefaultContext(project_name);
        rf.setDefaultConfigFile("parameters.ini");
        rf.configure(0, YARP_NULLPTR);

        Bottle* fallback_intrinsic = rf.findGroup("FALLBACK").find("intrinsic_" + camera_).asList();
        if (fallback_intrinsic)
        {
            yInfo() << log_ID_ << "[FALLBACK][CAM PARAMS]" << fallback_intrinsic->toString();

            cam_width_  = fallback_intrinsic->get(0).asDouble();
            cam_height_ = fallback_intrinsic->get(1).asDouble();
            cam_fx_     = fallback_intrinsic->get(2).asDouble();
            cam_cx_     = fallback_intrinsic->get(3).asDouble();
            cam_fy_     = fallback_intrinsic->get(4).asDouble();
            cam_cy_     = fallback_intrinsic->get(5).asDouble();
        }
        else
        {
            yWarning() << log_ID_ << "[CAM PARAMS]" << "No fallback values could be found in parameters.ini by the ctor for the intrinisc camera parameters. Falling (even more) back to iCub_SIM values.";
            cam_width_  = 320;
            cam_height_ = 240;
            cam_fx_     = 257.34;
            cam_cx_     = 160;
            cam_fy_     = 120;
            cam_cy_     = 257.34;
        }
    }
    yInfo() << log_ID_ << "[CAM]" << "Running with:";
    yInfo() << log_ID_ << "[CAM]" << " - width:"  << cam_width_;
    yInfo() << log_ID_ << "[CAM]" << " - height:" << cam_height_;
    yInfo() << log_ID_ << "[CAM]" << " - fx:"     << cam_fx_;
    yInfo() << log_ID_ << "[CAM]" << " - fy:"     << cam_fy_;
    yInfo() << log_ID_ << "[CAM]" << " - cx:"     << cam_cx_;
    yInfo() << log_ID_ << "[CAM]" << " - cy:"     << cam_cy_;


    /* Initiliaze left eye interface */
    left_eye_ = iCubEye(camera_ + "_v2");
    left_eye_.setAllConstraints(false);
    left_eye_.releaseLink(0);
    left_eye_.releaseLink(1);
    left_eye_.releaseLink(2);


    /* Initialize right arm interface */
    yInfo() << log_ID_ << "Setting arms.";

    right_arm_ = iCubArm("right");

    right_arm_.setAllConstraints(false);
    right_arm_.releaseLink(0);
    right_arm_.releaseLink(1);
    right_arm_.releaseLink(2);


    /* Initialize right hand finger interfaces */
    yInfo() << log_ID_ << "Setting fingers.";

    right_finger_[0] = iCubFinger("right_thumb");
    right_finger_[1] = iCubFinger("right_index");
    right_finger_[2] = iCubFinger("right_middle");

    right_finger_[0].setAllConstraints(false);
    right_finger_[1].setAllConstraints(false);
    right_finger_[2].setAllConstraints(false);


    /* Initialize CAD superimposer */
    yInfo() << log_ID_ << "Setting up OpenGL drawer.";

    drawer_ = new SICAD(cad_hand_, cam_width_, cam_height_, 1, shader_path_,
                        cam_fx_, cam_fy_, cam_cx_, cam_cy_);

    drawer_->setBackgroundOpt(true);
    drawer_->setWireframeOpt(true);


    /* Open command port */
    if (!setCommandPort()) throw std::runtime_error("Cannot attach the command port.");


    yInfo() << log_ID_ << "...CADSuperimposer ctor completed!";
}


CADSuperimposer::~CADSuperimposer() noexcept
{
    delete drawer_;
}


void CADSuperimposer::run()
{
    Vector ee_pose(7);
    Vector cam_pose(7);
    Vector encs_arm(16);
    Vector encs_torso(3);
    Vector encs_rot_ee(10);

    while (!isStopping())
    {
        ImageOf<PixelRgb>* imgin = inport_renderer_img_.read(true);

        if (imgin != NULL)
        {
            left_eye_.setAng (CTRL_DEG2RAD * readRootToLeftEye());
            right_arm_.setAng(CTRL_DEG2RAD * readRootToEE()     );


            ee_pose = getEndEffectorPose();
            if (ee_pose.size() == 6)
            {
                double ang =  norm(ee_pose.subVector(3, 5));
                ee_pose(3) /= ang;
                ee_pose(4) /= ang;
                ee_pose(5) /= ang;
                ee_pose.push_back(ang);
            }


            cam_pose = left_eye_.EndEffPose();


            encs_arm = getRightArmEncoders();
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


            SuperImpose::ObjPoseMap hand_pose;
            getRightHandObjPoseMap(ee_pose, hand_pose);
            getExtraObjPoseMap(hand_pose);

            cv::Mat img = cv::cvarrToMat(imgin->getIplImage(), true);
            drawer_->superimpose(hand_pose, cam_pose.data(), cam_pose.data()+3, img);
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

    if (!inport_renderer_img_.isClosed())  inport_renderer_img_.close();
    if (!outport_renderer_img_.isClosed()) outport_renderer_img_.close();

    if (port_command_.isOpen()) port_command_.close();

    yInfo() << log_ID_ << "Deallocation completed!";
}


void CADSuperimposer::getRightHandObjPoseMap(const Vector& ee_pose, SuperImpose::ObjPoseMap& hand_pose)
{
    SuperImpose::ObjPose pose;

    Matrix Ha = axis2dcm(ee_pose.subVector(3, 5));
    Ha.setSubcol(ee_pose.subVector(0, 2), 0, 3);
    Ha(3, 3) = 1.0;

    pose.assign(ee_pose.data(), ee_pose.data()+7);
    hand_pose.emplace("palm", pose);
    for (size_t fng = 0; fng < 3; ++fng)
    {
        std::string finger_s;
        pose.clear();
        if (fng != 0)
        {
            Vector j_x = (Ha * (right_finger_[fng].getH0().getCol(3))).subVector(0, 2);
            Vector j_o = dcm2axis(Ha * right_finger_[fng].getH0());

            if      (fng == 1) { finger_s = "index0";  }
            else if (fng == 2) { finger_s = "medium0"; }

            pose.assign(j_x.data(), j_x.data()+3);
            pose.insert(pose.end(), j_o.data(), j_o.data()+4);
            hand_pose.emplace(finger_s, pose);
        }

        for (size_t i = 0; i < right_finger_[fng].getN(); ++i)
        {
            Vector j_x = (Ha * (right_finger_[fng].getH(i, true).getCol(3))).subVector(0, 2);
            Vector j_o = dcm2axis(Ha * right_finger_[fng].getH(i, true));

            if      (fng == 0) { finger_s = "thumb"  + std::to_string(i+1); }
            else if (fng == 1) { finger_s = "index"  + std::to_string(i+1); }
            else if (fng == 2) { finger_s = "medium" + std::to_string(i+1); }

            pose.assign(j_x.data(), j_x.data()+3);
            pose.insert(pose.end(), j_o.data(), j_o.data()+4);
            hand_pose.emplace(finger_s, pose);
        }
    }
}


void CADSuperimposer::getExtraObjPoseMap(SuperImpose::ObjPoseMap& hand_pose)
{
    SuperImpose::ObjPose pose;

    Matrix H_forearm = right_arm_.getH(7, true);

    Vector j_x = H_forearm.getCol(3).subVector(0, 2);
    Vector j_o = dcm2axis(H_forearm);

    pose.assign(j_x.data(), j_x.data()+3);
    pose.insert(pose.end(), j_o.data(), j_o.data()+4);

    hand_pose.emplace("forearm", pose);
}


bool CADSuperimposer::mesh_background(const bool status)
{
    yInfo() << log_ID_ << ConstString((status ? "Enable" : "Disable")) + " background of the mesh window.";
    
    drawer_->setBackgroundOpt(status);
    
    return true;
}


bool CADSuperimposer::mesh_wireframe(const bool status)
{
    yInfo() << log_ID_ << ConstString((status ? "Enable" : "Disable")) + " wireframe rendering.";

    drawer_->setWireframeOpt(status);
    
    return true;
}


bool CADSuperimposer::sync_input(const bool status)
{
    return false;
}


bool CADSuperimposer::openGazeController()
{
    std::lock_guard<std::mutex> lock(mtx_gaze_);

    if (itf_gaze_)
        return true;
    else
    {
        Property opt_gaze;
        opt_gaze.put("device", "gazecontrollerclient");
        opt_gaze.put("local",  "/" + ID_ + "/gaze");
        opt_gaze.put("remote", "/iKinGazeCtrl");

        if (drv_gaze_.open(opt_gaze))
        {
            drv_gaze_.view(itf_gaze_);
            if (!itf_gaze_)
            {
                yError() << log_ID_ << "Cannot get head gazecontrollerclient interface!";
                drv_gaze_.close();
                return false;
            }
        }
        else
        {
            yError() << log_ID_ << "Cannot open head gazecontrollerclient!";
            return false;
        }

        return true;
    }
}


bool CADSuperimposer::setCommandPort()
{
    yInfo() << log_ID_ << "Opening command port.";
    if (!port_command_.open("/" + ID_ + "/render/cmd:i"))
    {
        yError() << log_ID_ << "Cannot open /" + ID_ + "/render/cmd:i port.";
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


Vector CADSuperimposer::readRootToEE()
{
    Vector enc_arm = getRightArmEncoders();

    Vector root_ee_enc(10);

    root_ee_enc.setSubvector(0, getTorsoEncoders());

    for (size_t i = 0; i < 7; ++i)
        root_ee_enc(i+3) = enc_arm(i);

    return root_ee_enc;
}


Vector CADSuperimposer::readRootToLeftEye()
{
    Vector enc_head = getHeadEncoders();

    Vector root_eye_enc(8);

    root_eye_enc.setSubvector(0, getTorsoEncoders());

    for (size_t i = 0; i < 4; ++i)
        root_eye_enc(3+i) = enc_head(i);

    root_eye_enc(7) = enc_head(4) + enc_head(5) / 2.0;

    /* if (cam == "right") root_eye_enc(7) = enc_head(4) - enc_head(5) / 2.0; */

    return root_eye_enc;
}
