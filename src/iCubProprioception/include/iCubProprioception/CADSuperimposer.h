#ifndef CADSUPERIMPOSER_H
#define CADSUPERIMPOSER_H

#include "thrift/iCubProprioceptionOGLIDL.h"

#include <mutex>
#include <vector>

#include <iCub/iKin/iKinFwd.h>
#include <SuperimposeMesh/SICAD.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Port.h>
#include <yarp/os/Thread.h>


class CADSuperimposer : public yarp::os::Thread,
                        public iCubProprioceptionOGLIDL
{
public:
    CADSuperimposer(const yarp::os::ConstString& robot, const yarp::os::ConstString& camera,
                    const SICAD::ModelPathContainer& cad_hand, const yarp::os::ConstString& shader_path,
                    const yarp::os::ConstString& port_prefix,
                    const bool draw_thumb, const bool draw_forearm);

    ~CADSuperimposer() noexcept;

    void run() override;

    void onStop() override;

    void threadRelease() override;

protected:
    virtual yarp::sig::Vector getEndEffectorPose() = 0;

    virtual yarp::sig::Vector getHeadEncoders() = 0;

    virtual yarp::sig::Vector getRightArmEncoders() = 0;

#if ICP_USE_ANALOGS == 1
    virtual yarp::sig::Vector getRightHandAnalogs() = 0;
#endif

    virtual yarp::sig::Vector getTorsoEncoders() = 0;


    bool mesh_background(const bool status) override;

    bool mesh_wireframe(const bool status) override;


    yarp::sig::Matrix getInvertedH(const double a, const double d, const double alpha, const double offset, const double q);


    const yarp::os::ConstString ID_;
    const yarp::os::ConstString log_ID_;
    const yarp::os::ConstString robot_;
    const yarp::os::ConstString camera_;

private:
    void getRightHandObjPoseMap(const yarp::sig::Vector& ee_pose, Superimpose::ModelPoseContainer& hand_pose);

    bool openGazeController();

    yarp::sig::Vector readRootToEye(const yarp::os::ConstString& camera);

    bool setCommandPort();


    SICAD*                           drawer_;
    const SICAD::ModelPathContainer& cad_hand_;
    const yarp::os::ConstString      shader_path_;

    bool draw_thumb_;
    bool draw_forearm_;

    yarp::dev::PolyDriver            drv_gaze_;
    yarp::dev::IGazeControl*         itf_gaze_;

    unsigned int                     cam_width_;
    unsigned int                     cam_height_;
    float                            cam_fx_;
    float                            cam_fy_;
    float                            cam_cx_;
    float                            cam_cy_;

    iCub::iKin::iCubEye              eye_;
    iCub::iKin::iCubArm              right_arm_;
    iCub::iKin::iCubFinger           right_finger_[3];


    yarp::sig::ImageOf<yarp::sig::PixelRgb>* imgin_ = YARP_NULLPTR;
    yarp::sig::Vector                        root_eye_enc_;
    yarp::sig::Vector                        ee_pose_;
    yarp::sig::Vector                        encs_arm_;


    yarp::os::Port                                                  port_command_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> inport_renderer_img_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> outport_renderer_img_;
};

#endif /* CADSUPERIMPOSER_H */
