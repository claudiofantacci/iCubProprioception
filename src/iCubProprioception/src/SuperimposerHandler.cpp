#include "iCubProprioception/SuperimposerHandler.h"
#include "iCubProprioception/common.h"

#include <exception>
#include <list>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


SuperimposerHandler::SuperimposerHandler(const yarp::os::ConstString& project_name) :
    ID_(project_name), log_ID_("[" + project_name + "]") { }


SuperimposerHandler::SuperimposerHandler() :
    SuperimposerHandler("SuperimposerModule") { }


bool SuperimposerHandler::configure(ResourceFinder &rf)
{
    this->setName(ID_.c_str());

    /* Setting default parameters. */
    ConstString context = rf.getContext();


    /* Parsing parameters from CLI. */
    /* Robot name */
    robot_    = rf.check("robot", Value("icubSim")).asString();
    skeleton_ = ((rf.findGroup("ske").size()   == 1 ? true : (rf.findGroup("ske").size()   == 2 ? rf.find("ske").asBool()   : false)));
    ikin_     = ((rf.findGroup("ikin").size()  == 1 ? true : (rf.findGroup("ikin").size()  == 2 ? rf.find("ikin").asBool()  : false)));
    ext_      = ((rf.findGroup("ext").size()   == 1 ? true : (rf.findGroup("ext").size()   == 2 ? rf.find("ext").asBool()   : false)));
    batch_    = ((rf.findGroup("batch").size() == 1 ? true : (rf.findGroup("batch").size() == 2 ? rf.find("batch").asBool() : false)));

    yInfo() << log_ID_ << "Running with:";
    yInfo() << log_ID_ << " - robot name:"              << robot_;
    yInfo() << log_ID_ << " - draw skeleton:"           << (skeleton_ ? "true" : "false");
    yInfo() << log_ID_ << " - render using ikin:"       << (ikin_     ? "true" : "false");
    yInfo() << log_ID_ << " - render using ext source:" << (ext_      ? "true" : "false");
    yInfo() << log_ID_ << " - render batch data:"       << (batch_    ? "true" : "false");


    /* Search mesh files in /mesh context folder */
    rf.setDefaultContext(context + "/mesh");

    cad_hand_["palm"] = rf.findFileByName("r_palm.obj");
    if (!fileFound(cad_hand_["palm"])) return false;
    cad_hand_["thumb1"] = rf.findFileByName("r_tl0.obj");
    if (!fileFound(cad_hand_["thumb1"])) return false;
    cad_hand_["thumb2"] = rf.findFileByName("r_tl1.obj");
    if (!fileFound(cad_hand_["thumb2"])) return false;
    cad_hand_["thumb3"] = rf.findFileByName("r_tl2.obj");
    if (!fileFound(cad_hand_["thumb3"])) return false;
    cad_hand_["thumb4"] = rf.findFileByName("r_tl3.obj");
    if (!fileFound(cad_hand_["thumb4"])) return false;
    cad_hand_["thumb5"] = rf.findFileByName("r_tl4.obj");
    if (!fileFound(cad_hand_["thumb5"])) return false;
    cad_hand_["index0"] = rf.findFileByName("r_indexbase.obj");
    if (!fileFound(cad_hand_["index0"])) return false;
    cad_hand_["index1"] = rf.findFileByName("r_ail0.obj");
    if (!fileFound(cad_hand_["index1"])) return false;
    cad_hand_["index2"] = rf.findFileByName("r_ail1.obj");
    if (!fileFound(cad_hand_["index2"])) return false;
    cad_hand_["index3"] = rf.findFileByName("r_ail2.obj");
    if (!fileFound(cad_hand_["index3"])) return false;
    cad_hand_["index4"] = rf.findFileByName("r_ail3.obj");
    if (!fileFound(cad_hand_["index4"])) return false;
    cad_hand_["medium0"] = rf.findFileByName("r_ml0.obj");
    if (!fileFound(cad_hand_["medium0"])) return false;
    cad_hand_["medium1"] = rf.findFileByName("r_ml1.obj");
    if (!fileFound(cad_hand_["medium1"])) return false;
    cad_hand_["medium2"] = rf.findFileByName("r_ml2.obj");
    if (!fileFound(cad_hand_["medium2"])) return false;
    cad_hand_["medium3"] = rf.findFileByName("r_ml3.obj");
    if (!fileFound(cad_hand_["medium3"])) return false;
    cad_hand_["forearm"] = rf.findFileByName("r_forearm.obj");
    if (!fileFound(cad_hand_["forearm"])) return false;


    /* Search shader files in /shader context folder */
    rf.setDefaultContext(context + "/shader");

    shader_path_ = rf.findFileByName("shader_model.vert");
    if (!fileFound(shader_path_)) return false;
    shader_path_ = shader_path_.substr(0, shader_path_.rfind("/"));


    /* Initializing useful pose matrices and vectors for the hand. */
    frontal_view_R_.resize(3, 3);
    frontal_view_R_(0,0) =  0.0;   frontal_view_R_(0,1) =  0.0;   frontal_view_R_(0,2) =  1.0;
    frontal_view_R_(1,0) = -1.0;   frontal_view_R_(1,1) =  0.0;   frontal_view_R_(1,2) =  0.0;
    frontal_view_R_(2,0) =  0.0;   frontal_view_R_(2,1) = -1.0;   frontal_view_R_(2,2) =  0.0;

    frontal_view_x_.resize(3);
    frontal_view_x_[0] = -0.25;
    frontal_view_x_[1] = +0.00;
    frontal_view_x_[2] = +0.20;

    table_view_R_.resize(3, 3);
    table_view_R_(0,0) = -1.0;   table_view_R_(0,1) =  0.0;   table_view_R_(0,2) =  0.0;
    table_view_R_(1,0) =  0.0;   table_view_R_(1,1) =  1.0;   table_view_R_(1,2) =  0.0;
    table_view_R_(2,0) =  0.0;   table_view_R_(2,1) =  0.0;   table_view_R_(2,2) = -1.0;

    table_view_x_.resize(3);
    table_view_x_[0] = -0.40;
    table_view_x_[1] = +0.10;
    table_view_x_[2] = +0.10;

    open_hand_joints_[0] = 0;
    open_hand_joints_[1] = 0;
    open_hand_joints_[2] = 0;
    open_hand_joints_[3] = 10;
    open_hand_joints_[4] = 0;
    open_hand_joints_[5] = 0;

    closed_hand_joints_[0] = 80;
    closed_hand_joints_[1] = 150;
    closed_hand_joints_[2] = 180;
    closed_hand_joints_[3] = 80;
    closed_hand_joints_[4] = 10;
    closed_hand_joints_[5] = 80;


    // FIXME: Spostare i movimenti del robot in un'altro thread. L'RFM module serve solo da handler.
    /* Torso control board. */
    setTorsoRemoteControlboard();

    /* Right arm control board. */
    setRightArmRemoteControlboard();

    /* Right arm cartesian controler. */
    if (setRightArmCartesianController()) setTorsoDOF();

    /* Head control board. */
    setHeadRemoteControlboard();

    /* Gaze control. */
    setGazeController();


    /* Launching skeleton superimposer thread */
    if (skeleton_)
    {
        /* Left camera */
        try { trd_left_cam_skeleton_ = new SkeletonSuperimposer(ID_ + "/SkeletonSuperimposer", robot_, "left"); }
        catch (const std::runtime_error& e) { yError() << e.what(); }

        if (trd_left_cam_skeleton_ != YARP_NULLPTR)
        {
            yInfo() << log_ID_ << "Starting skeleton superimposing thread for the right hand on the left camera images...";

            if (!trd_left_cam_skeleton_->start()) yError() << log_ID_ << "...thread could not be started!";
            else                                  yInfo()  << log_ID_ << "...done.";
        }
        else
            yError() << log_ID_ << "Could not initialize hand skeleton superimposition!";

        /* Right camera */
        try { trd_left_cam_skeleton_ = new SkeletonSuperimposer(ID_ + "/SkeletonSuperimposer", robot_, "right"); }
        catch (const std::runtime_error& e) { yError() << e.what(); }

        if (trd_left_cam_skeleton_ != YARP_NULLPTR)
        {
            yInfo() << log_ID_ << "Starting skeleton superimposing thread for the right hand on the right camera images...";

            if (!trd_left_cam_skeleton_->start()) yError() << log_ID_ << "...thread could not be started!";
            else                                  yInfo()  << log_ID_ << "...done.";
        }
        else
            yError() << log_ID_ << "Could not initialize hand skeleton superimposition!";
    }


    /* Lunching iKin CAD superimposer thread */
    if (ikin_)
    {
        /* Left camera */
        try { trd_left_cam_ikin_cad_ = new iKinCADSuperimposer(ID_ + "/iKinCADSuperimposer", robot_, "left", cad_hand_, shader_path_); }
        catch (const std::runtime_error& e) { yError() << e.what(); }

        if (trd_left_cam_ikin_cad_ != YARP_NULLPTR)
        {
            yInfo() << log_ID_ << "Starting iKinFwd mesh superimposing thread for the right hand on the left camera images...";

            if (!trd_left_cam_ikin_cad_->start()) yError() << log_ID_ << "...thread could not be started!";
            else                                  yInfo()  << log_ID_ << "...done.";
        }
        else
            yError() << log_ID_ << "Could not initialize iKinFwd hand mesh superimposition!";

        /* Right camera */
        try { trd_left_cam_ikin_cad_ = new iKinCADSuperimposer(ID_ + "/iKinCADSuperimposer", robot_, "right", cad_hand_, shader_path_); }
        catch (const std::runtime_error& e) { yError() << e.what(); }

        if (trd_left_cam_ikin_cad_ != YARP_NULLPTR)
        {
            yInfo() << log_ID_ << "Starting iKinFwd mesh superimposing thread for the right hand on the right camera images...";

            if (!trd_left_cam_ikin_cad_->start()) yError() << log_ID_ << "...thread could not be started!";
            else                                  yInfo()  << log_ID_ << "...done.";
        }
        else
            yError() << log_ID_ << "Could not initialize iKinFwd hand mesh superimposition!";
    }


    /* Lunching External (pose) CAD superimposer thread */
    if (ext_)
    {
        /* Left camera */
        try { trd_left_cam_ext_cad_ = new ExtCADSuperimposer(ID_ + "/ExtCADSuperimposer", robot_, "left", cad_hand_, shader_path_); }
        catch (const std::runtime_error& e) { yError() << e.what(); }

        if (trd_left_cam_ext_cad_ != YARP_NULLPTR)
        {
            yInfo() << log_ID_ << "Starting iKinFwd external (pose) mesh superimposing thread for the right hand on the left camera images...";

            if (!trd_left_cam_ext_cad_->start()) yError() << log_ID_ << "...thread could not be started!";
            else                                 yInfo()  << log_ID_ << "...done.";
        }
        else
            yError() << log_ID_ << "Could not initialize iKinFwd external (pose) hand mesh superimposition for the left camera!";


        /* Right camera */
        try { trd_left_cam_ext_cad_ = new ExtCADSuperimposer(ID_ + "/ExtCADSuperimposer", robot_, "right", cad_hand_, shader_path_); }
        catch (const std::runtime_error& e) { yError() << e.what(); }

        if (trd_left_cam_ext_cad_ != YARP_NULLPTR)
        {
            yInfo() << log_ID_ << "Starting iKinFwd external (pose) mesh superimposing thread for the right hand on the right camera images...";

            if (!trd_left_cam_ext_cad_->start()) yError() << log_ID_ << "...thread could not be started!";
            else                                 yInfo()  << log_ID_ << "...done.";
        }
        else
            yError() << log_ID_ << "Could not initialize iKinFwd external (pose) hand mesh superimposition for the right camera!";
    }


    /* Lunching Batch (pose and ecnoders) CAD superimposer thread */
    if (batch_)
    {
        /* Left camera */
        try { trd_left_cam_batch_cad_ = new BatchCADSuperimposer(ID_ + "/BatchCADSuperimposer", robot_, "left", cad_hand_, shader_path_); }
        catch (const std::runtime_error& e) { yError() << e.what(); }

        if (trd_left_cam_batch_cad_ != YARP_NULLPTR)
        {
            yInfo() << log_ID_ << "Starting Batch mesh superimposing thread for the right hand on the left camera images...";

            if (!trd_left_cam_batch_cad_->start()) yError() << log_ID_ << "...thread could not be started!";
            else                                   yInfo()  << log_ID_ << "...done.";
        }
        else
            yError() << log_ID_ << "Could not initialize Batch hand mesh superimposition for the left camera!";

        /* Right camera */
        try { trd_left_cam_batch_cad_ = new BatchCADSuperimposer(ID_ + "/BatchCADSuperimposer", robot_, "right", cad_hand_, shader_path_); }
        catch (const std::runtime_error& e) { yError() << e.what(); }

        if (trd_left_cam_batch_cad_ != YARP_NULLPTR)
        {
            yInfo() << log_ID_ << "Starting Batch mesh superimposing thread for the right hand on the right camera images...";

            if (!trd_left_cam_batch_cad_->start()) yError() << log_ID_ << "...thread could not be started!";
            else                                   yInfo()  << log_ID_ << "...done.";
        }
        else
            yError() << log_ID_ << "Could not initialize Batch hand mesh superimposition for the right camera!";
    }


    /* Open a remote command port and allow the program be started */
    if (!setCommandPort()) return false;


    return true;
}


bool SuperimposerHandler::updateModule()
{
    glfwPollEvents();

    return true;
}


bool SuperimposerHandler::close()
{
    yInfo() << log_ID_ << "Calling close functions...";

    if (trd_left_cam_skeleton_ != YARP_NULLPTR) trd_left_cam_skeleton_->stop();
    if (trd_left_cam_ikin_cad_ != YARP_NULLPTR) trd_left_cam_ikin_cad_->stop();
    if (trd_left_cam_ext_cad_  != YARP_NULLPTR) trd_left_cam_ext_cad_->stop();

    delete trd_left_cam_skeleton_;
    delete trd_left_cam_ikin_cad_;
    delete trd_left_cam_ext_cad_;

    if (itf_rightarm_cart_) itf_rightarm_cart_->removeTipFrame();

    if (rightarm_cartesian_driver_.isValid()) rightarm_cartesian_driver_.close();
    if (rightarm_remote_driver_.isValid())    rightarm_remote_driver_.close();
    if (head_remote_driver_.isValid())        head_remote_driver_.close();
    if (gaze_driver_.isValid())               gaze_driver_.close();
    
#if ICP_USE_ANALOGS == 1
    if (drv_right_hand_analog_.isValid())     drv_right_hand_analog_.close();
#endif
    
    if (port_command_.isOpen()) port_command_.close();
    return true;
}


bool SuperimposerHandler::initial_position()
{
    yInfo() << log_ID_ << "Reaching initial position...";

    bool motion_done = moveHand(table_view_R_, table_view_x_);

    if (motion_done) yInfo()    << log_ID_ << "...done!";
    else             yWarning() << log_ID_ << "...could not reach initial position!";

    return motion_done;
}


bool SuperimposerHandler::view_hand()
{
    yInfo() << log_ID_ << "Reaching a position close to iCub left camera with the right hand...";

    bool motion_done = moveHand(frontal_view_R_, frontal_view_x_);

    if (motion_done) yInfo()    << log_ID_ << "...done!";
    else             yWarning() << log_ID_ << "...could not reach the desired position!";

    return motion_done;
}


bool SuperimposerHandler::open_fingers()
{
    yInfo() << log_ID_ << "Opening fingers...";

    bool motion_done = moveFingers(open_hand_joints_);

    if (motion_done) yInfo()    << log_ID_ << "...done!";
    else             yWarning() << log_ID_ << "...fingers could not be opened!";

    return motion_done;
}


bool SuperimposerHandler::close_fingers()
{
    yInfo() << log_ID_ << "Closing fingers...";

    bool motion_done = moveFingers(closed_hand_joints_);

    if (motion_done) yInfo()    << log_ID_ << "...done!";
    else             yWarning() << log_ID_ << "...fingers could not be closed!";

    return motion_done;
}


std::string SuperimposerHandler::quit()
{
    yInfo() << log_ID_ << "Quitting...";

    this->stopModule();

    return "[bye]";
}


bool SuperimposerHandler::fileFound(const ConstString& file)
{
    if (file.empty())
    {
        yError() << log_ID_ << "File not found!";
        return false;
    }
    return true;
}


bool SuperimposerHandler::setTorsoRemoteControlboard()
{
    Property torso_remote_options;
    torso_remote_options.put("device", "remote_controlboard");
    torso_remote_options.put("local", "/"+ID_+"/control_torso");
    torso_remote_options.put("remote", "/"+robot_+"/torso");

    torso_remote_driver_.open(torso_remote_options);
    if (torso_remote_driver_.isValid())
    {
        yInfo() << log_ID_ << "Torso remote_controlboard succefully opened.";
        return true;
    }
    else
    {
        yError() << log_ID_ << "Error opening Torso remote_controlboard device.";
        return false;
    }
}


bool SuperimposerHandler::setRightArmRemoteControlboard()
{
    Property rightarm_remote_options;
    rightarm_remote_options.put("device", "remote_controlboard");
    rightarm_remote_options.put("local", "/"+ID_+"/control_right_arm");
    rightarm_remote_options.put("remote", "/"+robot_+"/right_arm");

    rightarm_remote_driver_.open(rightarm_remote_options);
    if (rightarm_remote_driver_.isValid())
    {
        yInfo() << log_ID_ << "Right arm remote_controlboard succefully opened.";

        rightarm_remote_driver_.view(itf_rightarm_enc_);
        if (!itf_rightarm_enc_)
        {
            yError() << log_ID_ << "Error getting right arm IEncoders interface.";
            return false;
        }
        num_rightarm_enc_ = 0;
        itf_rightarm_enc_->getAxes(&num_rightarm_enc_);
        yInfo() << log_ID_ << "Right arm encorders succefully read.";

        rightarm_remote_driver_.view(itf_rightarm_pos_);
        if (!itf_rightarm_pos_)
        {
            yError() << log_ID_ << "Error getting right arm IPositionControl2 interface.";
            return false;
        }
        yInfo() << log_ID_ << "Right arm positions succefully read.";
    }
    else
    {
        yError() << log_ID_ << "Error opening right arm remote_controlboard device.";
        return false;
    }

#if ICP_USE_ANALOGS == 1
    Property righthand_remote_analog;
    righthand_remote_analog.put("device", "analogsensorclient");
    righthand_remote_analog.put("local",  "/"+ID_+"/right_hand");
    righthand_remote_analog.put("remote", "/"+robot_+"/right_hand/analog:o");

    drv_right_hand_analog_.open(righthand_remote_analog);
    if (drv_right_hand_analog_.isValid())
    {
        yInfo() << log_ID_ << "Right arm analogsensorclient succefully opened.";
    }
    else
    {
        yError() << log_ID_ << "Error opening right arm analogsensorclient device.";
        return false;
    }
#endif

    return true;
}


bool SuperimposerHandler::setRightArmCartesianController()
{
    Property rightarm_cartesian_options;
    rightarm_cartesian_options.put("device", "cartesiancontrollerclient");
    rightarm_cartesian_options.put("local", "/"+ID_+"/cart_right_arm");
    rightarm_cartesian_options.put("remote", "/"+robot_+"/cartesianController/right_arm");

    rightarm_cartesian_driver_.open(rightarm_cartesian_options);
    if (rightarm_cartesian_driver_.isValid())
    {
        rightarm_cartesian_driver_.view(itf_rightarm_cart_);
        if (!itf_rightarm_cart_)
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

    if (!itf_rightarm_cart_->setTrajTime(2.5))
    {
        yError() << log_ID_ << "Error setting ICartesianControl trajectory time.";
        return false;
    }
    yInfo() << log_ID_ << "Succesfully set ICartesianControl trajectory time!";

    if (!itf_rightarm_cart_->setInTargetTol(0.01))
    {
        yError() << log_ID_ << "Error setting ICartesianControl target tolerance.";
        return false;
    }
    yInfo() << log_ID_ << "Succesfully set ICartesianControl target tolerance!";

    return true;
}


bool SuperimposerHandler::setHeadRemoteControlboard()
{
    Property head_option;
    head_option.put("device", "remote_controlboard");
    head_option.put("local", "/"+ID_+"/control_head");
    head_option.put("remote", "/"+robot_+"/head");

    head_remote_driver_.open(head_option);
    if (head_remote_driver_.isValid())
    {
        yInfo() << log_ID_ << "Head remote_controlboard succefully opened.";

        head_remote_driver_.view(itf_head_pos_);
        if (!itf_head_pos_)
        {
            yError() << log_ID_ << "Error getting head IPositionControl interface.";
            return false;
        }
        yInfo() << log_ID_ << "Head positions succefully read.";
    }
    else
    {
        yError() << log_ID_ << "Error opening head remote_controlboard device.";
        return false;
    }

    return true;
}


bool SuperimposerHandler::setGazeController()
{
    Property gaze_option;
    gaze_option.put("device", "gazecontrollerclient");
    gaze_option.put("local", "/"+ID_+"/gaze");
    gaze_option.put("remote", "/iKinGazeCtrl");

    gaze_driver_.open(gaze_option);
    if (gaze_driver_.isValid())
    {
        gaze_driver_.view(itf_head_gaze_);
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


bool SuperimposerHandler::setTorsoDOF()
{
    Vector curDOF;
    itf_rightarm_cart_->getDOF(curDOF);
    yInfo() << log_ID_ << "Old DOF: [" + curDOF.toString(0) + "].";
    yInfo() << log_ID_ << "Setting iCub to use the DOF from the torso.";
    Vector newDOF(curDOF);
    newDOF[0] = 1;
    newDOF[1] = 2;
    newDOF[2] = 1;
    if (!itf_rightarm_cart_->setDOF(newDOF, curDOF))
    {
        yError() << log_ID_ << "Cannot use torso DOF.";
        return false;
    }
    yInfo() << log_ID_ << "Setting the DOF done.";
    yInfo() << log_ID_ << "New DOF: [" + curDOF.toString(0) + "]";

    return true;
}


bool SuperimposerHandler::setCommandPort()
{
    yInfo() << log_ID_ << "Opening command port.";
    if (!port_command_.open("/"+ID_+"/cmd:i"))
    {
        yError() << log_ID_ << "Cannot open the command port.";
        return false;
    }
    if (!this->yarp().attachAsServer(port_command_))
    {
        yError() << log_ID_ << "Cannot attach the command port.";
        return false;
    }
    yInfo() << log_ID_ << "Command port and helper succesfully opened and attached. Ready to recieve commands.";

    return true;
}


bool SuperimposerHandler::moveFingers(const double joint[6])
{
    /* Close iCub hand. */
    yInfo() << log_ID_ << "Closing fingers.";
    Vector rightarm_encoder(static_cast<size_t>(num_rightarm_enc_));
    itf_rightarm_enc_->getEncoders(rightarm_encoder.data());
    std::list<std::pair<unsigned int, double>> joint_pos_map = {{13, joint[0]},
                                                                {14, joint[1]},
                                                                {15, joint[2]},
                                                                { 8, joint[3]},
                                                                { 9, joint[4]},
                                                                {10, joint[5]}};
    for (auto map = joint_pos_map.cbegin(); map != joint_pos_map.cend(); ++map)
    {
        yInfo() << log_ID_ << "Moving joint "+std::to_string(map->first)+" to the position "+std::to_string(map->second)+".";
        if (std::abs(rightarm_encoder[map->first] - map->second) > 5.0)
        {
            rightarm_encoder[map->first] = map->second;
            itf_rightarm_pos_->positionMove(rightarm_encoder.data());
            Time::delay(2.0);
        }
    }
    yInfo() << log_ID_ << "Fingers succesfully closed.";

    return true;
}


bool SuperimposerHandler::moveHand(const Matrix &R, const Vector &init_x)
{
    /* Setting hand pose */
    yInfo() << log_ID_ << "Moving hand to the initial position.";
    yInfo() << log_ID_ << "R = " << R.toString();
    yInfo() << log_ID_ << "x = " << init_x.toString();

    Vector init_o(dcm2axis(R));

    itf_rightarm_cart_->goToPoseSync(init_x, init_o);
    itf_rightarm_cart_->waitMotionDone(0.2, 15.0);

    yInfo() << log_ID_ << "The hand is in position.";

    /* Set fixation point */
    Vector tmp;
    itf_head_gaze_->getFixationPoint(tmp);
    Vector init_fixation(init_x);
    init_fixation[0] -= 0.05;
    init_fixation[1] -= 0.05;
    if (norm(tmp - init_fixation) > 0.10)
    {
        yInfo() << log_ID_ << "Moving head to initial fixation point: [" << init_fixation.toString() << "].";
        itf_head_gaze_->lookAtFixationPoint(init_fixation);
        itf_head_gaze_->waitMotionDone(0.1, 15.0);
    }
    yInfo() << log_ID_ << "Gaze motion done.";

    return true;
}
