#ifndef EXTCADSUPERIMPOSER_H
#define EXTCADSUPERIMPOSER_H

#include "thrift/iCubProprioceptionOGLIDL.h"

#include <vector>

#include <iCub/iKin/iKinFwd.h>
#include <SuperImpose/SICAD.h>
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


class ExtCADSuperimposer : public yarp::os::Thread,
                           public iCubProprioceptionOGLIDL
{
public:
    ExtCADSuperimposer(const yarp::os::ConstString& project_name, const yarp::os::ConstString& robot, const yarp::os::ConstString& camera,
                       const SuperImpose::ObjFileMap& cad_hand, const yarp::os::ConstString& shader_path);

    ~ExtCADSuperimposer() noexcept;

    void run() override;

    void onStop() override;

    void threadRelease() override;

protected:
    bool setArmRemoteControlboard();

    bool setGazeController();

    bool mesh_background(const bool status) override;

    bool mesh_wireframe(const bool status) override;

    bool sync_input(const bool status) override;

    void getPose(const yarp::sig::Vector& cur_state, SuperImpose::ObjPoseMap& hand_poses);

private:
    const yarp::os::ConstString    ID_;
    const yarp::os::ConstString    log_ID_;
    const yarp::os::ConstString    robot_;
    const yarp::os::ConstString    camera_;
    const int                      camsel_;

    yarp::dev::PolyDriver          drv_right_arm_remote_;
#if ICP_USE_ANALOGS == 1
    yarp::dev::PolyDriver          drv_right_hand_analog_;
#endif
    yarp::dev::PolyDriver          drv_gaze_;

    yarp::dev::IEncoders         * itf_right_arm_encoders_;
    yarp::dev::IControlLimits    * itf_right_fingers_limits_;
#if ICP_USE_ANALOGS == 1
    yarp::dev::IAnalogSensor     * itf_right_hand_analog_;
#endif
    yarp::dev::IGazeControl      * itf_head_gaze_;

    const SuperImpose::ObjFileMap& cad_hand_;
    const yarp::os::ConstString    shader_path_;

    int                            num_right_arm_enc_ = 0;
    unsigned int                   cam_width_;
    unsigned int                   cam_height_;
    float                          cam_fx_;
    float                          cam_fy_;
    float                          cam_cx_;
    float                          cam_cy_;

    iCub::iKin::iCubFinger         right_finger_[3];

    SICAD                        * drawer_;

    yarp::sig::Vector              estimates_mean_copy_;
    yarp::sig::Vector              estimates_mode_copy_;

    bool                           synch_ = true;


    bool setCommandPort();

    yarp::os::Port                                                  port_command_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> inport_renderer_img_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> outport_renderer_img_;

    yarp::os::BufferedPort<yarp::sig::Vector>                       inport_renderer_pf_mean_;
    yarp::os::BufferedPort<yarp::sig::Vector>                       inport_renderer_pf_mode_;
};

#endif /* EXTCADSUPERIMPOSER_H */
