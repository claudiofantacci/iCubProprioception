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
    BatchCADSuperimposer(const yarp::os::ConstString& robot, const yarp::os::ConstString& camera,
                         const SICAD::ModelPathContainer& cad_hand, const yarp::os::ConstString& shader_path,
                         const yarp::os::ConstString& port_prefix,
                         const bool draw_thumb, const bool draw_forearm);

    ~BatchCADSuperimposer() noexcept;

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


    bool sync_input(const bool status) override;

private:
    yarp::sig::Vector readRootToEE();

    yarp::sig::Vector readFromBufferedPort(yarp::os::BufferedPort<yarp::sig::Vector>& bp);


    bool synch_ = false;


    yarp::os::BufferedPort<yarp::sig::Vector> inport_pose_ext_;
    yarp::os::BufferedPort<yarp::sig::Vector> inport_head_enc_;
    yarp::os::BufferedPort<yarp::sig::Vector> inport_torso_enc_;
    yarp::os::BufferedPort<yarp::sig::Vector> inport_right_arm_enc_;
    yarp::os::BufferedPort<yarp::sig::Vector> inport_right_hand_analogs_;
};

#endif /* BATCHCADSUPERIMPOSER_H */
