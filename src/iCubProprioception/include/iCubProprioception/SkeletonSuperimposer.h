#ifndef SKELETONSUPERIMPOSER_H
#define SKELETONSUPERIMPOSER_H

#include "iCubProprioception/common.h"

#include <iCub/iKin/iKinFwd.h>
#include <SuperimposeMesh/SISkeleton.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Thread.h>
#include <yarp/sig/Image.h>


class SkeletonSuperimposer : public yarp::os::Thread
{
public:
    SkeletonSuperimposer(const yarp::os::ConstString& port_prefix, const yarp::os::ConstString& robot, const yarp::os::ConstString& camera);

    ~SkeletonSuperimposer() noexcept;

    void run() override;

    void onStop() override;

    void threadRelease() override;

protected:
    bool setArmRemoteControlboard();

    bool setArmCartesianController();

    bool setGazeController();

private:
    const yarp::os::ConstString    ID_;
    const yarp::os::ConstString    log_ID_;
    const yarp::os::ConstString    robot_;
    const yarp::os::ConstString    camera_;

    yarp::dev::PolyDriver          drv_right_arm_remote_;
#if ICP_USE_ANALOGS == 1
    yarp::dev::PolyDriver          drv_right_hand_analog_;
#endif
    yarp::dev::PolyDriver          drv_right_arm_cartesian_;
    yarp::dev::PolyDriver          drv_gaze_;

    yarp::dev::IEncoders        *  itf_right_arm_encoders_;
    yarp::dev::IControlLimits   *  itf_fingers_limits_;
    yarp::dev::ICartesianControl*  itf_right_arm_cart_;
#if ICP_USE_ANALOGS == 1
    yarp::dev::IAnalogSensor    *  itf_right_hand_analog_;
#endif
    yarp::dev::IGazeControl     *  itf_head_gaze_;

    int                            num_right_arm_enc_;
    float                          eye_fx_;
    float                          eye_fy_;
    float                          eye_cx_;
    float                          eye_cy_;
    iCub::iKin::iCubFinger         right_finger_[3];

    SISkeleton                  *  drawer_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> inport_skeleton_img_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> outport_skeleton_img_;
};

#endif /* SKELETONSUPERIMPOSER_H */
