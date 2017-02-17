#ifndef SKELETONSUPERIMPOSER_H
#define SKELETONSUPERIMPOSER_H

#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Thread.h>
#include <yarp/sig/Image.h>
#include <iCub/iKin/iKinFwd.h>

#include <SuperImpose/SISkeleton.h>


class SkeletonSuperimposer : public yarp::os::Thread
{
public:
    SkeletonSuperimposer(const yarp::os::ConstString& project_name, const yarp::os::ConstString& laterality, const yarp::os::ConstString& camera, yarp::dev::PolyDriver& arm_remote_driver, yarp::dev::PolyDriver& arm_cartesian_driver, yarp::dev::PolyDriver& gaze_driver);

    ~SkeletonSuperimposer() noexcept;

    void run          ();
    void onStop       ();
    void threadRelease();

private:
    const yarp::os::ConstString    log_ID_;
    const yarp::os::ConstString    project_name_;
    const yarp::os::ConstString    laterality_;
    const yarp::os::ConstString    camera_;
    const int                      camsel_;
    yarp::dev::PolyDriver        & arm_remote_driver_;
    yarp::dev::PolyDriver        & arm_cartesian_driver_;
    yarp::dev::PolyDriver        & gaze_driver_;

    yarp::dev::IEncoders        *  itf_arm_encoders_;
    yarp::dev::ICartesianControl*  itf_arm_cart_;
    yarp::dev::IGazeControl     *  itf_head_gaze_;
    int                            num_arm_enc_;
    float                          eye_fx_;
    float                          eye_fy_;
    float                          eye_cx_;
    float                          eye_cy_;

    iCub::iKin::iCubFinger         finger_[3];

    SISkeleton                  *  drawer_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> inport_skeleton_img_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> outport_skeleton_img_;
    yarp::os::BufferedPort<yarp::os::Bottle>                        port_cam_pose_;

};

#endif /* SKELETONSUPERIMPOSER_H */
