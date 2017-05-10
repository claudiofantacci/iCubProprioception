# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Claudio Fantacci
#
# idl_mesh.thrift

/**
 * iCubProprioceptionOGLIDL
 *
 * IDL Interface to \ref iCubProprioception OpenGL services.
 */

service iCubProprioceptionOGLIDL
{
    /**
     * View or hide background image of the mesh window.
     *
     * @param status true/false to turn background on/off.
     *
     * @return true activation/deactivation success, false otherwise.
     */
    bool mesh_background(1:bool status);

    /**
     * Render the mesh as a wireframe. If disabled (default behaviour)
     * the mesh is filled without light.
     *
     * @param status true/false to turn wireframe rendering on/off.
     *
     * @return true activation/deactivation success, false otherwise.
     */
    bool mesh_wireframe(1:bool status);

    /**
     * Synchronize rendering FPS to the input rate of the poses received from
     * the ports.
     *
     * If the synchronization is off, then the last received pose is used by
     * the rendered (pose lock).
     *
     * @param status true/false to turn synchronization on/off.
     *
     * @return true activation/deactivation success, false otherwise.
     */
    bool sync_input(1:bool status);
}
