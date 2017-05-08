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
     * Safely close the module.
     */
    string quit();
}
