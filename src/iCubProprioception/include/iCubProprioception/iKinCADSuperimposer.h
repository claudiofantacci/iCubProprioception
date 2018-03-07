#ifndef IKINCADSUPERIMPOSER_H
#define IKINCADSUPERIMPOSER_H

#include "CADSuperimposer.h"

#include <vector>

#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/ConstString.h>


class iKinCADSuperimposer : public CADSuperimposer
{
public:
    iKinCADSuperimposer(const yarp::os::ConstString& robot, const yarp::os::ConstString& camera,
                        const SICAD::ModelPathContainer& cad_hand, const yarp::os::ConstString& shader_path,
                        const yarp::os::ConstString& port_prefix,
                        const bool draw_thumb, const bool draw_forearm);

    ~iKinCADSuperimposer() noexcept;

    void onStop() override;

    void threadRelease() override;

protected:
    yarp::sig::Vector getEndEffectorPose() override;

    yarp::sig::Vector getHeadEncoders() override;

    yarp::sig::Vector getRightArmEncoders() override;

#if ICP_USE_ANALOGS == 1
    yarp::sig::Vector getRightHandAnalogs() override;
#endif

    yarp::sig::Vector getTorsoEncoders() override;


    bool setHeadRemoteControlboard();

    bool setTorsoRemoteControlboard();

    bool setArmRemoteControlboard();

    bool setArmCartesianController();

private:
    yarp::sig::Vector readRootToEE();


    yarp::dev::PolyDriver         drv_head_remote_;
    yarp::dev::PolyDriver         drv_torso_remote_;
    yarp::dev::PolyDriver         drv_right_arm_remote_;
    yarp::dev::PolyDriver         drv_right_arm_cartesian_;
#if ICP_USE_ANALOGS == 1
    yarp::dev::PolyDriver         drv_right_hand_analog_;
#endif

    yarp::dev::IEncoders*         itf_head_encoders_;
    yarp::dev::IEncoders*         itf_torso_encoders_;
    yarp::dev::IEncoders*         itf_right_arm_encoders_;
    yarp::dev::ICartesianControl* itf_right_arm_cart_;
#if ICP_USE_ANALOGS == 1
    yarp::dev::IAnalogSensor*     itf_right_hand_analog_;
#endif
};

#endif /* IKINCADSUPERIMPOSER_H */
