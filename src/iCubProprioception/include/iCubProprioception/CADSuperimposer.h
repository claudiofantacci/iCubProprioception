#ifndef CADSUPERIMPOSER_H
#define CADSUPERIMPOSER_H

#include "thrift/iCubProprioceptionOGLIDL.h"

#include <mutex>
#include <vector>

#include <iCub/iKin/iKinFwd.h>
#include <SuperImpose/SICAD.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Port.h>
#include <yarp/os/Thread.h>


class CADSuperimposer : public yarp::os::Thread,
                        public iCubProprioceptionOGLIDL
{
public:
    CADSuperimposer(const yarp::os::ConstString& project_name, const yarp::os::ConstString& robot, const yarp::os::ConstString& camera,
                    const SuperImpose::ObjFileMap& cad_hand, const yarp::os::ConstString& shader_path);

    ~CADSuperimposer() noexcept;

    void run() override;

    void onStop() override;

    void threadRelease() override;

protected:
    virtual yarp::sig::ImageOf<yarp::sig::PixelRgb>* getImage() = 0;

    virtual yarp::sig::Vector                        getEndEffectorPose() = 0;

    virtual yarp::sig::Vector                        getLeftEyePose() = 0;

    virtual yarp::sig::Vector                        getRightArmEncoders() = 0;

    virtual yarp::sig::Vector                        getRightArmAnalogs() = 0;

    virtual yarp::sig::Vector                        getTorsoEncoders() = 0;


    virtual void getObjPoseMap(const yarp::sig::Vector& ee_pose, SuperImpose::ObjPoseMap& hand_pose);


    bool mesh_background(const bool status) override;

    bool mesh_wireframe(const bool status) override;

    bool sync_input(const bool status) override;

private:
    const yarp::os::ConstString     ID_;
    const yarp::os::ConstString     log_ID_;
    const yarp::os::ConstString     robot_;
    const yarp::os::ConstString     camera_;

    const SuperImpose::ObjFileMap & cad_hand_;
    const yarp::os::ConstString     shader_path_;

    static std::mutex               mtx_gaze_;
    static yarp::dev::PolyDriver    drv_gaze_;
    static yarp::dev::IGazeControl* itf_gaze_;

    unsigned int                    cam_width_;
    unsigned int                    cam_height_;
    float                           cam_fx_;
    float                           cam_fy_;
    float                           cam_cx_;
    float                           cam_cy_;

    iCub::iKin::iCubFinger          right_finger_[3];
    iCub::iKin::iCubArm             right_arm_;

    SICAD                         * drawer_;

    yarp::os::Port                                                  port_command_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> inport_renderer_img_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> outport_renderer_img_;


    bool openGazeController();

    bool setCommandPort();
};

#endif /* CADSUPERIMPOSER_H */
