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
    ExtCADSuperimposer(const yarp::os::ConstString& port_prefix, const yarp::os::ConstString& robot, const yarp::os::ConstString& camera,
                       const SuperImpose::ObjFileMap& cad_hand, const yarp::os::ConstString& shader_path);

    ~ExtCADSuperimposer() noexcept;

    void threadRelease() override;

protected:
    yarp::sig::Vector getEndEffectorPose() override;

    void getExtraObjPoseMap(SuperImpose::ObjPoseMap& hand_pose) override;

    bool sync_input(const bool status) override;

    bool render_extra_mesh(const bool status) override;

private:
    yarp::sig::Vector                         ext_pose_copy_;

    bool                                      view_forearm_ = false;

    bool                                      synch_ = true;

    yarp::os::BufferedPort<yarp::sig::Vector> inport_pose_ext_;
};

#endif /* EXTCADSUPERIMPOSER_H */
