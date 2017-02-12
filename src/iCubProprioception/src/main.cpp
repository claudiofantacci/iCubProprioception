#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "iCubProprioception/SuperimposerFactory.h"

using namespace yarp::os;


int main(int argc, char* argv[])
{
    ConstString log_ID = "[Main]";
    yInfo() << log_ID << "Configuring and starting module...";

    Network yarp;
    if (!yarp.checkNetwork(3.0))
    {
        yError() << log_ID << "YARP seems unavailable.";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("iCubProprioception-Config-SIM.ini");
    rf.setDefaultContext("iCubProprioception");
    rf.configure(argc, argv);

    /* Initialize OpenGL context */
    SuperimposerFactory::initOGL(320, 240, 1);

    /* SuperimposerFactory, derived from RFModule, must be declared, initialized and run in the main thread (thread_0). */
    SuperimposerFactory sh;
    sh.setProjectName("iCubProprioception");

    if (sh.runModuleThreaded(rf) == 0)
    {
        while (!sh.isStopping())
        {
            glfwPollEvents();
        }
    }

    sh.joinModule();

    glfwMakeContextCurrent(NULL);
    glfwTerminate();

    yInfo() << log_ID << "Main returning.";
    yInfo() << log_ID << "Application closed.";
    return EXIT_SUCCESS;
}
