#ifndef CADSUPERIMPOSER_H
#define CADSUPERIMPOSER_H

#include <unordered_map>

#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Port.h>
#include <yarp/os/Thread.h>
#include <iCub/iKin/iKinFwd.h>

#include <SuperImpose/SICAD.h>

#include "iCubProprioception/ThreadControllerSHC.h"


class CADSuperimposer : public yarp::os::Thread
{
public:
    CADSuperimposer(const yarp::os::ConstString& project_name, const yarp::os::ConstString& robot, const yarp::os::ConstString& camera,
                    const SuperImpose::ObjFileMap& cad_hand, const yarp::os::ConstString& shader_path);

    ~CADSuperimposer() noexcept;

    void run();

    void onStop();

    void threadRelease();

protected:
    bool setTorsoRemoteControlboard();

    bool setArmRemoteControlboard();

    bool setArmCartesianController();

    bool setGazeController();
    
private:
    const yarp::os::ConstString    ID_;
    const yarp::os::ConstString    log_ID_;
    const yarp::os::ConstString    robot_;
    const yarp::os::ConstString    camera_;
    const int                      camsel_;
    
    yarp::dev::PolyDriver          drv_torso_remote_;
    yarp::dev::PolyDriver          drv_right_arm_remote_;
    yarp::dev::PolyDriver          drv_right_arm_cartesian_;
#if ICP_USE_ANALOGS == 1
    yarp::dev::PolyDriver          drv_right_hand_analog_;
#endif
    yarp::dev::PolyDriver          drv_gaze_;
    yarp::dev::IEncoders         * itf_torso_encoders_;
    yarp::dev::IEncoders         * itf_rightarm_encoders_;
    yarp::dev::IControlLimits    * itf_fingers_limits_;
    yarp::dev::ICartesianControl * itf_rightarm_cart_;
#if ICP_USE_ANALOGS == 1
    yarp::dev::IAnalogSensor     * itf_right_hand_analog_;
#endif
    yarp::dev::IGazeControl      * itf_head_gaze_;

    const SuperImpose::ObjFileMap& cad_hand_;
    const yarp::os::ConstString    shader_path_;

    int                            num_rightarm_enc_ = 0;
    unsigned int                   cam_width_;
    unsigned int                   cam_height_;
    float                          cam_fx_;
    float                          cam_fy_;
    float                          cam_cx_;
    float                          cam_cy_;

    iCub::iKin::iCubFinger         finger_[3];
    iCub::iKin::iCubArm            arm_;

    SICAD                        * drawer_;
    ThreadControllerSHC            helper_;

    yarp::os::Port                                                  port_command_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> inport_renderer_img_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> outport_renderer_img_;
    
    bool setCommandPort();
};

#endif /* CADSUPERIMPOSER */
