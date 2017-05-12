#ifndef SUPERIMPOSEFACTORY_H
#define SUPERIMPOSEFACTORY_H

#include <unordered_map>

#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

#include <SuperImpose/SuperImpose.h>

#include "iCubProprioception/SkeletonSuperimposer.h"
#include "iCubProprioception/ExtCADSuperimposer.h"
#include "iCubProprioception/iKinCADSuperimposer.h"
#include "thrift/iCubProprioceptionIDL.h"


class SuperimposerHandler : public yarp::os::RFModule,
                            public iCubProprioceptionIDL
{
public:
    SuperimposerHandler();

    SuperimposerHandler(const yarp::os::ConstString& project_name);

    double getPeriod() { return 0.033; }

    bool   configure(yarp::os::ResourceFinder& rf);
    
    bool   updateModule();

    bool   close();

protected:
    bool initial_position ();

    bool view_hand();

    bool open_fingers();

    bool close_fingers();

    bool view_skeleton(const bool status);

    bool view_mesh(const bool status);

    std::string quit();

    bool fileFound(const yarp::os::ConstString& file);

    bool setTorsoRemoteControlboard();

    bool setRightArmRemoteControlboard();

    bool setRightArmCartesianController();

    bool setHeadRemoteControlboard();

    bool setGazeController();

    bool setTorsoDOF();

    bool setCommandPort();

    bool moveFingers(const double joint[6]);

    bool moveHand(const yarp::sig::Matrix& R, const yarp::sig::Vector& init_x);

private:
    const yarp::os::ConstString   ID_;
    const yarp::os::ConstString   log_ID_;

    yarp::os::ConstString         robot_;

    yarp::dev::PolyDriver         torso_remote_driver_;

    yarp::dev::PolyDriver         rightarm_remote_driver_;
    yarp::dev::IEncoders        * itf_rightarm_enc_;
    yarp::dev::IPositionControl2* itf_rightarm_pos_;
    int                           num_rightarm_enc_;

    yarp::dev::PolyDriver         rightarm_cartesian_driver_;
    yarp::dev::ICartesianControl* itf_rightarm_cart_;

    yarp::dev::PolyDriver         head_remote_driver_;
    yarp::dev::IPositionControl2* itf_head_pos_;

    yarp::dev::PolyDriver         gaze_driver_;
    yarp::dev::IGazeControl     * itf_head_gaze_;

    yarp::dev::PolyDriver         drv_right_hand_analog_;

    SkeletonSuperimposer        * trd_left_cam_skeleton_ = YARP_NULLPTR;

    iKinCADSuperimposer         * trd_left_cam_ikin_cad_ = YARP_NULLPTR;
    ExtCADSuperimposer          * trd_left_cam_ext_cad_  = YARP_NULLPTR;
    SuperImpose::ObjFileMap       cad_hand_;
    yarp::os::ConstString         shader_path_;

    yarp::os::Port                port_command_;

    yarp::sig::Matrix             frontal_view_R_;
    yarp::sig::Vector             frontal_view_x_;

    yarp::sig::Matrix             table_view_R_;
    yarp::sig::Vector             table_view_x_;

    double                        open_hand_joints_[6];
    double                        closed_hand_joints_[6];
};

#endif /* SUPERIMPOSEFACTORY_H */
