#ifndef EXTCADSUPERIMPOSER_H
#define EXTCADSUPERIMPOSER_H

#include "iKinCADSuperimposer.h"

#include <vector>

#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/ConstString.h>


class ExtCADSuperimposer : public iKinCADSuperimposer
{
public:
    ExtCADSuperimposer(const yarp::os::ConstString& robot, const yarp::os::ConstString& camera, const SICAD::ModelPathContainer& cad_hand, const yarp::os::ConstString& shader_path, const yarp::os::ConstString& port_prefix, const bool draw_thumb, const bool draw_forearm);

    ~ExtCADSuperimposer() noexcept;

    void onStop() override;

    void threadRelease() override;

protected:
    yarp::sig::Vector getEndEffectorPose() override;

    bool sync_input(const bool status) override;

private:
    bool synch_ = false;

    yarp::os::BufferedPort<yarp::sig::Vector> inport_pose_ext_;
};

#endif /* EXTCADSUPERIMPOSER_H */
