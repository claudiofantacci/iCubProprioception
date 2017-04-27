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

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <SuperImpose/SICAD.h>

#include "iCubProprioception/ThreadControllerSHC.h"


class CADSuperimposer : public yarp::os::Thread
{
public:
    CADSuperimposer(const yarp::os::ConstString& project_name,
                    const yarp::os::ConstString& laterality,
                    const yarp::os::ConstString& camera,
                    yarp::dev::PolyDriver& torso_remote_driver,
                    yarp::dev::PolyDriver& arm_remote_driver,
                    yarp::dev::PolyDriver& arm_cartesian_driver,
                    yarp::dev::PolyDriver& gaze_driver,
                    yarp::dev::PolyDriver& drv_right_hand_analog,
                    const SuperImpose::ObjFileMap& cad_hand,
                    const yarp::os::ConstString& shader_path);

    ~CADSuperimposer() noexcept;

    static bool initOGL(const GLsizei width, const GLsizei height, const GLint view = 1);

    void run          ();

    void onStop       ();

    void threadRelease();
    
private:
    const yarp::os::ConstString    log_ID_;
    const yarp::os::ConstString    project_name_;
    
    // TODO: rivedere il costruttore ed instanziare i PolyDriver in ThreadInit.
    const yarp::os::ConstString    laterality_;
    const yarp::os::ConstString    camera_;
    const int                      camsel_;
    yarp::dev::PolyDriver        & torso_remote_driver_;
    yarp::dev::IEncoders         * itf_torso_encoders_;
    yarp::dev::PolyDriver        & arm_remote_driver_;
    yarp::dev::IEncoders         * itf_arm_encoders_;
    yarp::dev::PolyDriver        & arm_cartesian_driver_;
    yarp::dev::ICartesianControl * itf_arm_cart_;
    yarp::dev::PolyDriver        & drv_right_hand_analog_;
    yarp::dev::IGazeControl      * itf_head_gaze_;
    yarp::dev::PolyDriver        & gaze_driver_;
    yarp::dev::IAnalogSensor     * itf_right_hand_analog_;
    const SuperImpose::ObjFileMap& cad_hand_;
    const yarp::os::ConstString    shader_path_;
    int                            num_arm_enc_;
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
    yarp::os::BufferedPort<yarp::os::Bottle>                        port_cam_pose_;
    
    bool setCommandPort();
};

#endif /* CADSUPERIMPOSER */
