#ifndef THREADCONTROLLERSHC_H
#define THREADCONTROLLERSHC_H

#include <iosfwd>

#include <yarp/os/ConstString.h>

#include "thrift/iCubProprioceptionIDL.h"


class ThreadControllerSHC : public iCubProprioceptionIDL
{
public:
    enum MipMaps
    {
        NEAREST = 0,
        LINEAR  = 1
    };
    
    ThreadControllerSHC  ();
    bool getBackgroundOpt() { return mesh_back;  }
    bool getWireframeOpt () { return mesh_wires; }
    MipMaps getMipmapsOpt() { return mesh_mmaps; }
    
protected:
    bool mesh_background(const bool status);
    bool mesh_wireframe (const bool status);
    bool mesh_mipmaps   (const std::string& type);

private:
    yarp::os::ConstString log_ID;

    bool mesh_back;
    bool mesh_wires;
    MipMaps mesh_mmaps;
};

#endif /* THREADCONTROLLERSHC_H */
