#!/usr/bin/env python

import argparse
import struct
import sys
import numpy as np
import rospy
import baxter_interface
import math

from math import fabs

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    String,
    UInt8,
    Float64,
    Bool,
    Int8
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_interface import CHECK_VERSION

from baxter_core_msgs.msg import EndpointState

import ar_track_alvar_msgs

from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker


# tag_msg = AlvarMarkers()

#Publish to topics the scanned ID and associated color of the present to look for, as well the T/F state of sweep
pub_color = rospy.Publisher('color_identifier',String)
pub_id = rospy.Publisher('scanned_stocking_id',Int8)
pub_statesweep = rospy.Publisher('/start/sweep',Bool)



completed_list = []
StateSweep =  True
run = True



#Listens to ar_pose to identify the stocking and which color the associated present is
def identify_pres(msg):

    #Global variables being used
    global StateSweep
    global completed_list

    #Sets the T/F status of sweep to True, such that it continously runs until finds an ar tracker
    StateSweep = True

    #Onlys runs when told to
    if run == True:

        #Then obtains the markers message of AlvarMarkers
        for m in msg.markers:

            #Creates local variables for the ID of found stocking
            identity = m.id
            sent_identity = int(identity)

            #Runs while found stocking has not already been filled
            while identity not in completed_list:

                #Stocking #1 found
                if identity==1:

                    #Associated color of present is red
                    color = "red"

                    print "Identified as person 1"

                    #Adds Stocking #1 to completed list, and turns the status of sweep to F so it does not continue looking
                    completed_list.append(identity)
                    StateSweep = False

                    #Publishes color of present to look for, as well as stocking ID
                    pub_color.publish(color)
                    pub_id.publish(sent_identity)

                #Stocking #2 found
                elif identity==2:

                    #Associated color of present is blue
                    color = "blue"

                    print "Identified as person 2"

                    #Adds Stocking #2 to completed list, and turns the status of sweep to F so it does not continue looking
                    completed_list.append(identity)
                    StateSweep = False

                    #Publishes color of present to look for, as well as stocking ID
                    pub_color.publish(color)
                    pub_id.publish(sent_identity)

                #Stocking #3 found
                elif identity==3:

                    #Associated color of present is green
                    color = "green"

                    print "Identified as person 3"

                    #Adds Stocking #3 to completed list, and turns the status of sweep to F so it does not continue looking
                    completed_list.append(identity)
                    StateSweep = False

                    #Publishes color of present to look for, as well as stocking ID
                    pub_color.publish(color)
                    pub_id.publish(sent_identity)

                #Stocking #4 found
                elif identity==4:

                    #Associated color of present is yellow
                    color = "yellow"

                    print "Identified as person 4"

                    #Adds Stocking #4 to completed list, and turns the status of sweep to F so it does not continue looking
                    completed_list.append(identity)
                    StateSweep = False

                    #Publishes color of present to look for, as well as stocking ID
                    pub_color.publish(color)
                    pub_id.publish(sent_identity)


    #Publishes the status of sweep to F if a new stocking was found, otherwise it remains T
    pub_statesweep.publish(StateSweep)

    return




#Obtain T/F state of whether to start looking for a new present
def getStatusSweep(msg):

    global run

    run = msg.data



#Initializes node and subscribers
def tag_identity_listener():

    rospy.init_node('tag_identity_listener',anonymous = True)

    #Subscribe to AlvarMarkers message and T/F status of sweep
    rospy.Subscriber("/ar_pose_marker",AlvarMarkers,identify_pres)
    rospy.Subscriber("/start/sweep",Bool,getStatusSweep)


    rospy.spin()



if __name__ == '__main__':
    tag_identity_listener()