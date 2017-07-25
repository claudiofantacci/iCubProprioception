#ifndef BATCHCADSUPERIMPOSER_H
#define BATCHCADSUPERIMPOSER_H

#include "CADSuperimposer.h"

#include <vector>

#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/ConstString.h>


class BatchCADSuperimposer : public CADSuperimposer
{
public:
    BatchCADSuperimposer(const yarp::os::ConstString& port_prefix, const yarp::os::ConstString& robot, const yarp::os::ConstString& camera,
                         const SICAD::ModelPathContainer& cad_hand, const yarp::os::ConstString& shader_path);

    ~BatchCADSuperimposer() noexcept;

    void threadRelease() override;

protected:
    yarp::sig::Vector getEndEffectorPose() override;

    yarp::sig::Vector getHeadEncoders() override;

    yarp::sig::Vector getRightArmEncoders() override;

#if ICP_USE_ANALOGS == 1
    yarp::sig::Vector getRightHandAnalogs() override;
#endif

    yarp::sig::Vector getTorsoEncoders() override;

    void getExtraObjPoseMap(Superimpose::ModelPoseContainer& hand_pose) override;


    bool sync_input(const bool status) override;

    bool render_extra_mesh(const bool status) override;


    bool setHeadRemoteControlboard();

    bool setTorsoRemoteControlboard();

    bool setArmRemoteControlboard();

    bool setArmCartesianController();

private:
    iCub::iKin::iCubArm right_arm_;

    yarp::sig::Vector   ext_pose_copy_;


    bool synch_ = true;

    bool view_forearm_ = false;


    yarp::sig::Vector readRootToEE();

    
    yarp::os::BufferedPort<yarp::sig::Vector> inport_pose_ext_;
    yarp::os::BufferedPort<yarp::sig::Vector> inport_head_enc_;
    yarp::os::BufferedPort<yarp::sig::Vector> inport_torso_enc_;
    yarp::os::BufferedPort<yarp::sig::Vector> inport_right_arm_enc_;
    yarp::os::BufferedPort<yarp::sig::Vector> inport_right_hand_analogs_;
};

#endif /* BATCHCADSUPERIMPOSER_H */
