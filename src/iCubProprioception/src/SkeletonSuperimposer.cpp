#include "iCubProprioception/SkeletonSuperimposer.h"
#include "iCubProprioception/common.h"

#include <exception>

#include <iCub/ctrl/math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yarp/os/LogStream.h>
#include <yarp/math/Math.h>


using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace iCub::iKin;


SkeletonSuperimposer::SkeletonSuperimposer(const ConstString& port_prefix, const ConstString& robot, const ConstString& camera) :
    ID_(port_prefix), log_ID_("[" + ID_ + "]"),
    robot_(robot), camera_(camera)
{
    yInfo() << log_ID_ << "Initializing skeleton superimposer thread.";


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


    Bottle btl_cam_left_info;
    itf_head_gaze_->getInfo(btl_cam_left_info);
    Bottle * cam_left_info = btl_cam_left_info.findGroup("camera_intrinsics_" + camera_).get(1).asList();
    yInfo() << log_ID_ << "Camera Info: [" + cam_left_info->toString() + "].";
    eye_fx_ = static_cast<float>(cam_left_info->get(0).asDouble());
    eye_cx_ = static_cast<float>(cam_left_info->get(2).asDouble());
    eye_fy_ = static_cast<float>(cam_left_info->get(5).asDouble());
    eye_cy_ = static_cast<float>(cam_left_info->get(6).asDouble());


    yInfo() << log_ID_ << "Initilizing finger interfaces and bounds.";

    right_finger_[0] = iCubFinger("right_thumb");
    right_finger_[1] = iCubFinger("right_index");
    right_finger_[2] = iCubFinger("right_middle");

    std::deque<IControlLimits*> temp_lim;
    temp_lim.push_front(itf_fingers_limits_);
    for (int i = 0; i < 3; ++i)
    {
        if (!right_finger_[i].alignJointsBounds(temp_lim))
        {
            yError() << log_ID_ << "Cannot set joint bound for finger " + std::to_string(i) + "!";
            throw std::runtime_error("Cannot set joint bound for finger " + std::to_string(i) + "!");
        }
    }


    yInfo() << log_ID_ << "Opening ports for skeleton images.";

    if (!inport_skeleton_img_.open("/" + ID_ + "/cam/"+ camera_ +":i"))
    {
        yError() << log_ID_ << "Cannot open input image port for "+ camera_ +"!";
        throw std::runtime_error("Cannot open input image port for "+ camera_ +"!");
    }

    if (!outport_skeleton_img_.open("/" + ID_ + "/cam/"+ camera_ +":o"))
    {
        yError() << log_ID_ << "Cannot open output image port for "+ camera_ +"!";
        throw std::runtime_error("Cannot open output image port for "+ camera_ +"!");
    }


    yInfo() << log_ID_ << "Setting up skeleton superimposer.";

    drawer_ = new SISkeleton(eye_fx_, eye_fy_, eye_cx_, eye_cy_);


    yInfo() << log_ID_ << "Initialization completed!";
}


SkeletonSuperimposer::~SkeletonSuperimposer() noexcept
{
    delete drawer_;
}


void SkeletonSuperimposer::run()
{
    ImageOf<PixelRgb>* tmp_imgin = YARP_NULLPTR;
    Vector ee_x(3);
    Vector ee_o(4);
    Vector cam_x(3);
    Vector cam_o(4);
    Vector encs(num_right_arm_enc_);

    while (!isStopping())
    {
        bool have_data = true;

        tmp_imgin = inport_skeleton_img_.read(false);
        if (tmp_imgin != YARP_NULLPTR)
            imgin_ = tmp_imgin;

        have_data     &= itf_right_arm_cart_->getPose(ee_x, ee_o);

        if (camera_ == "left")
            have_data &= itf_head_gaze_->getLeftEyePose(cam_x, cam_o);
        else
            have_data &= itf_head_gaze_->getRightEyePose(cam_x, cam_o);

        have_data     &= itf_right_arm_encoders_->getEncoders(encs.data());

        if (imgin_ != YARP_NULLPTR && have_data)
        {
            Matrix Ha = axis2dcm(ee_o);
            ee_x.push_back(1.0);
            Ha.setCol(3, ee_x);

            Vector chainjoints;
            for (unsigned int i = 0; i < 3; ++i)
            {
                right_finger_[i].getChainJoints(encs, chainjoints);
                right_finger_[i].setAng(CTRL_DEG2RAD * chainjoints);
            }

            Superimpose::ModelPoseContainer hand_pose;
            Superimpose::ModelPose          pose;
            pose.assign(ee_x.data(), ee_x.data()+3);
            hand_pose.emplace("palm", pose);
            for (unsigned int fng = 0; fng < 3; ++fng)
            {
                std::string finger_s;
                pose.clear();
                if (fng != 0)
                {
                    Vector j_x = (Ha * (right_finger_[fng].getH0().getCol(3))).subVector(0, 2);

                    if      (fng == 1) { finger_s = "index"; }
                    else if (fng == 2) { finger_s = "medium"; }

                    pose.assign(j_x.data(), j_x.data()+3);
                    hand_pose.emplace(finger_s, pose);
                }

                for (unsigned int i = 0; i < right_finger_[fng].getN(); ++i)
                {
                    Vector j_x = (Ha * (right_finger_[fng].getH(i, true).getCol(3))).subVector(0, 2);

                    if      (fng == 0) { finger_s = "thumb"; }
                    else if (fng == 1) { finger_s = "index"; }
                    else if (fng == 2) { finger_s = "medium"; }

                    pose.assign(j_x.data(), j_x.data()+3);
                    hand_pose.emplace(finger_s, pose);
                }
            }

            ImageOf<PixelRgb>& imgout = outport_skeleton_img_.prepare();
            imgout = *imgin_;

            cv::Mat img = cv::cvarrToMat(imgout.getIplImage());
            drawer_->superimpose(hand_pose, cam_x.data(), cam_o.data(), img);

            outport_skeleton_img_.write();
        }
    }
}


void SkeletonSuperimposer::onStop()
{
    inport_skeleton_img_.interrupt();
}


void SkeletonSuperimposer::threadRelease()
{
    yInfo() << log_ID_ << "Deallocating resource...";


    if (!inport_skeleton_img_.isClosed())  inport_skeleton_img_.close();
    if (!outport_skeleton_img_.isClosed()) outport_skeleton_img_.close();


    yInfo() << log_ID_ << "...deallocation completed!";
}


bool SkeletonSuperimposer::setArmRemoteControlboard()
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

        itf_right_arm_encoders_->getAxes(&num_right_arm_enc_);
        yInfo() << log_ID_ << "Right arm encorders succefully read.";

        drv_right_arm_remote_.view(itf_fingers_limits_);
        if (!itf_fingers_limits_)
        {
            yError() << log_ID_ << "Error getting fingers IControlLimits interface.";
            throw std::runtime_error("Error getting fingers IControlLimits interface.");
        }
    }
    else
    {
        yError() << log_ID_ << "Error opening right arm remote_controlboard device.";
        return false;
    }

    // FIXME: Add getChainJoints() methods with analogs.
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


bool SkeletonSuperimposer::setArmCartesianController()
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


bool SkeletonSuperimposer::setGazeController()
{
    Property gaze_option;
    gaze_option.put("device", "gazecontrollerclient");
    gaze_option.put("local", "/" + ID_ + "/cam/" + camera_ + "/gaze");
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
