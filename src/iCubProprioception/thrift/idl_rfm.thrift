# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Claudio Fantacci
#
# idl_rfm.thrift

/**
 * iCubProprioceptionIDL
 *
 * IDL Interface to \ref iCubProprioception services.
 */

service iCubProprioceptionIDL
{
    /**
     * Move the iCub right hand to the initial position
     *     [-0.40]        [-1.0    0.0    0.0]
     * p = [ 0.10]    R = [ 0.0    1.0    0.0]
     *     [ 0.10]        [ 0.0    0.0   -1.0]
     * @return true on reaching succesfully, false otherwise.
     */
    bool initial_position();

    /**
     * Move the iCub right hand to the initial position
     *     [-0.25]        [ 0.0    0.0    1.0]
     * p = [ 0.00]    R = [-1.0    0.0    0.0]
     *     [ 0.20]        [ 0.0   -1.0    0.0]
     * @return true on reaching succesfully, false otherwise.
     */
    bool view_hand();

    /**
     * Open all iCub right fingers.
     * @return true on succesful motion, false otherwise.
     */
    bool open_fingers();

    /**
     * Close iCub right thumb, medium, ring and little fingers.
     * @return true on succesful motion, false otherwise.
     */
    bool close_fingers();

    /**
     * Superimpose the right hand skeleton (starting from the end effector)
     * onto the iCub right hand in the left camera images.
     * @param status true/false to turn superimposition on/off.
     * @return true activation/deactivation success, false otherwise.
     */
    bool view_skeleton(1:bool status);

    /**
     * Superimpose the right hand CAD mesh (palm, thumb, index and medium)
     * onto the iCub right hand in the left camera images.
     * @param status true/false to turn superimposition on/off.
     * @return true activation/deactivation success, false otherwise.
     */
    bool view_mesh(1:bool status);

    /**
     * Safely close the module.
     */
    string quit();
}
