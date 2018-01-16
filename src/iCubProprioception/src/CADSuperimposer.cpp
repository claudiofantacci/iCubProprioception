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


CADSuperimposer::CADSuperimposer(const yarp::os::ConstString& robot, const yarp::os::ConstString& camera,
                                 const SICAD::ModelPathContainer& cad_hand, const yarp::os::ConstString& shader_path,
                                 const yarp::os::ConstString& port_prefix,
                                 const bool draw_thumb, const bool draw_forearm) :
    ID_(port_prefix),
    log_ID_("[" + ID_ + "]"),
    robot_(robot),
    camera_(camera),
    cad_hand_(cad_hand),
    shader_path_(shader_path),
    draw_thumb_(draw_thumb),
    draw_forearm_(draw_forearm),
    eye_(iCubEye(camera_ + "_v2")),
    right_arm_(iCubArm("right_v2")),
    right_finger_{iCubFinger("right_thumb"), iCubFinger("right_index"), iCubFinger("right_middle")}
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
        rf.setVerbose(true);
        rf.setDefaultConfigFile("parameters.ini");
        rf.setDefaultContext("iCubProprioception");
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
            cam_fy_     = 257.34;
            cam_cy_     = 120;
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
    yInfo() << log_ID_ << "Setting" + camera_ + "eye.";

    eye_.setAllConstraints(false);
    eye_.releaseLink(0);
    eye_.releaseLink(1);
    eye_.releaseLink(2);


    /* Initialize right arm interface */
    yInfo() << log_ID_ << "Setting right arm.";

    right_arm_.setAllConstraints(false);
    right_arm_.releaseLink(0);
    right_arm_.releaseLink(1);
    right_arm_.releaseLink(2);


    /* Initialize right hand finger interfaces */
    yInfo() << log_ID_ << "Setting right hand fingers.";

    right_finger_[0].setAllConstraints(false);
    right_finger_[1].setAllConstraints(false);
    right_finger_[2].setAllConstraints(false);


    /* Initialize CAD superimposer */
    yInfo() << log_ID_ << "Setting up OpenGL drawer.";

    drawer_ = new SICAD(cad_hand_,
                        cam_width_, cam_height_, cam_fx_, cam_fy_, cam_cx_, cam_cy_,
                        1,
                        {1.0, 0.0, 0.0, static_cast<float>(M_PI)},
                        shader_path_,
                        false);

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
    while (!isStopping())
    {
        ImageOf<PixelRgb>* tmp_imgin = YARP_NULLPTR;
        tmp_imgin = inport_renderer_img_.read(false);
        if (tmp_imgin != YARP_NULLPTR)
            imgin_ = tmp_imgin;

        Vector tmp_root_eye_enc = readRootToEye(camera_);
        if (!(tmp_root_eye_enc == zeros(tmp_root_eye_enc.size())))
            enc_root_eye_ = tmp_root_eye_enc;

        Vector tmp_ee_pose = getEndEffectorPose();
        if (!(tmp_ee_pose == zeros(tmp_ee_pose.size())))
            ee_pose_ = tmp_ee_pose;

        Vector tmp_encs_root_arm = readRootToEndEffector();
        if (!(tmp_encs_root_arm == zeros(tmp_encs_root_arm.size())))
            encs_root_arm_ = tmp_encs_root_arm;

        if (  imgin_         != YARP_NULLPTR                  &&
            !(enc_root_eye_  == zeros(enc_root_eye_.size()))  &&
            !(ee_pose_       == zeros(ee_pose_.size()))       &&
            !(encs_root_arm_ == zeros(encs_root_arm_.size()))   )
        {
            eye_.setAng(CTRL_DEG2RAD * enc_root_eye_);
            Vector cam_pose = eye_.EndEffPose();

#if ICP_USE_ANALOGS == 1
            Vector analogs;
            itf_right_hand_analog_->read(analogs);
            yAssert(analogs.size() >= 15);
#endif

            right_arm_.setAng(encs_root_arm_.subVector(0, 9) * (M_PI/180.0));

            Vector chainjoints;
            for (unsigned int i = 0; i < 3; ++i)
            {
#if ICP_USE_ANALOGS == 1
                right_finger_[i].getChainJoints(encs_root_arm_.subVector(3, 18), analogs, chainjoints);
#else
                right_finger_[i].getChainJoints(encs_root_arm_.subVector(3, 18), chainjoints);
#endif

                right_finger_[i].setAng(CTRL_DEG2RAD * chainjoints);
            }

            Superimpose::ModelPoseContainer hand_pose;
            getRightHandObjPoseMap(ee_pose_, hand_pose);

            ImageOf<PixelRgb>& imgout = outport_renderer_img_.prepare();
            imgout = *imgin_;

            cv::Mat img = cv::cvarrToMat(imgout.getIplImage());
            drawer_->superimpose(hand_pose, cam_pose.data(), cam_pose.data()+3, img);

            outport_renderer_img_.write();
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


    if (!inport_renderer_img_.isClosed())  inport_renderer_img_.close();
    if (!outport_renderer_img_.isClosed()) outport_renderer_img_.close();

    if (port_command_.isOpen()) port_command_.close();


    yInfo() << log_ID_ << "...deallocation completed!";
}


void CADSuperimposer::getRightHandObjPoseMap(const Vector& ee_pose, Superimpose::ModelPoseContainer& hand_pose)
{
    Superimpose::ModelPose pose;

    Matrix Ha = axis2dcm(ee_pose.subVector(3, 6));
    Ha.setSubcol(ee_pose.subVector(0, 2), 0, 3);
    Ha(3, 3) = 1.0;

    pose.assign(ee_pose.data(), ee_pose.data()+7);
    hand_pose.emplace("palm", pose);
    for (size_t fng = (draw_thumb_? 0 : 1); fng < 3; ++fng)
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
    if (draw_forearm_)
    {
        yarp::sig::Matrix invH6 = Ha *
                                  getInvertedH(-0.0625, -0.02598,       0,   -M_PI, -right_arm_.getAng(9)) *
                                  getInvertedH(      0,        0, -M_PI_2, -M_PI_2, -right_arm_.getAng(8)) *
                                  getInvertedH(      0,   0.1413, -M_PI_2,  M_PI_2, 0);
        Vector j_x = invH6.getCol(3).subVector(0, 2);
        Vector j_o = dcm2axis(invH6);
        pose.clear();
        pose.assign(j_x.data(), j_x.data()+3);
        pose.insert(pose.end(), j_o.data(), j_o.data()+4);
        hand_pose.emplace("forearm", pose);
    }
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


Matrix CADSuperimposer::getInvertedH(const double a, const double d, const double alpha, const double offset, const double q)
{
    /** Table of the DH parameters for the right arm V2.
     *  Link i  Ai (mm)     d_i (mm)    alpha_i (rad)   theta_i (deg)
     *  i = 0	32          0           pi/2               0 + (-22 ->    84)
     *  i = 1	0           -5.5        pi/2             -90 + (-39 ->    39)
     *  i = 2	-23.3647    -143.3      pi/2            -105 + (-59 ->    59)
     *  i = 3	0           -107.74     pi/2             -90 + (  5 ->   -95)
     *  i = 4	0           0           -pi/2            -90 + (  0 -> 160.8)
     *  i = 5	-15.0       -152.28     -pi/2           -105 + (-37 ->   100)
     *  i = 6	15.0        0           pi/2               0 + (5.5 ->   106)
     *  i = 7	0           -141.3      pi/2             -90 + (-50 ->    50)
     *  i = 8	0           0           pi/2              90 + ( 10 ->   -65)
     *  i = 9	62.5        25.98       0                180 + (-25 ->    25)
     **/

    yarp::sig::Matrix H(4, 4);

    double theta = offset + q;
    double c_th  = cos(theta);
    double s_th  = sin(theta);
    double c_al  = cos(alpha);
    double s_al  = sin(alpha);

    H(0,0) =        c_th;
    H(0,1) =       -s_th;
    H(0,2) =           0;
    H(0,3) =           a;

    H(1,0) = s_th * c_al;
    H(1,1) = c_th * c_al;
    H(1,2) =       -s_al;
    H(1,3) =   -d * s_al;

    H(2,0) = s_th * s_al;
    H(2,1) = c_th * s_al;
    H(2,2) =        c_al;
    H(2,3) =    d * c_al;
    
    H(3,0) =           0;
    H(3,1) =           0;
    H(3,2) =           0;
    H(3,3) =           1;
    
    return H;
}


Vector CADSuperimposer::readRootToEye(const ConstString& camera)
{
    Vector enc_root_eye = zeros(8);

    if (camera != "left" && camera != "right")
    {
        yError() << "Wrong 'camera' argument for CADSuperimposer::readRootToEye(). Shall be 'left' or 'right'.";
        return enc_root_eye;
    }

    Vector enc_torso = getTorsoEncoders();
    if (enc_torso == zeros(enc_torso.size()))
        return enc_root_eye;

    Vector enc_head = getHeadEncoders();
    if (enc_head == zeros(enc_head.size()))
        return enc_root_eye;

    enc_root_eye.setSubvector(0, enc_torso);

    for (size_t i = 0; i < 4; ++i)
        enc_root_eye(enc_torso.size() + i) = enc_head(i);

    if (camera == "left")
        enc_root_eye(7) = enc_head(4) + enc_head(5) / 2.0;
    else if (camera == "right")
        enc_root_eye(7) = enc_head(4) - enc_head(5) / 2.0;
    
    return enc_root_eye;
}


Vector CADSuperimposer::readRootToEndEffector()
{
    Vector enc_root_ee = zeros(1);

    Vector enc_torso = getTorsoEncoders();
    if (enc_torso == zeros(enc_torso.size()))
        return enc_root_ee;

    Vector enc_right_arm = getRightArmEncoders();
    if (enc_right_arm == zeros(enc_right_arm.size()))
        return enc_root_ee;

    enc_root_ee.resize(enc_torso.size() + enc_right_arm.size());

    enc_root_ee.setSubvector(0,                enc_torso);
    enc_root_ee.setSubvector(enc_torso.size(), enc_right_arm);

    return enc_root_ee;
}


bool CADSuperimposer::openGazeController()
{
    Property opt_gaze;
    opt_gaze.put("device", "gazecontrollerclient");
    opt_gaze.put("local",  "/" + ID_ + "/cam/" + camera_ + "/gaze");
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


bool CADSuperimposer::setCommandPort()
{
    yInfo() << log_ID_ << "Opening command port.";
    if (!port_command_.open("/" + ID_ + "/cam/" + camera_ + "/render/cmd:i"))
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

