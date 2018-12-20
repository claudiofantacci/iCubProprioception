#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "iCubProprioception/SuperimposerHandler.h"

using namespace yarp::os;


int main(int argc, char* argv[])
{
    ConstString log_ID = "[Main]";

    yInfo() << log_ID << "Configuring and starting module...";

#if PROPRIO_USE_ANALOGS == 1
    yInfo() << log_ID << "PROPRIO_USE_ANALOGS: TRUE";
#else
    yInfo() << log_ID << "PROPRIO_USE_ANALOGS: FALSE";
#endif

    Network yarp;
    if (!yarp.checkNetwork(3.0))
    {
        yError() << log_ID << "YARP seems unavailable.";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("parameters.ini");
    rf.setDefaultContext("iCubProprioception");
    rf.configure(argc, argv);

    /* SuperimposerHandler, derived from RFModule, must be declared, initialized and run in the main thread (thread_0). */
    SuperimposerHandler sh("iCubProprioception");
    sh.runModule(rf);

    yInfo() << log_ID << "Main returning.";
    yInfo() << log_ID << "Application closed.";
    return EXIT_SUCCESS;
}
