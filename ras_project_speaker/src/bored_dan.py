#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import Bool
 
Expdone = True
def stringCallback(msg):
     rospy.loginfo('We cant reach the goooooaaaal!')
     if(msg.data==False):
        os.system('play '+ '/home/ras11/catkin_ws/src/ras_project/ras_project_speaker/sounds/cantdo.wav')

def ExpCallback(msg):
    global Expdone
    if(Expdone):
        if(msg.data):
            os.system('play ' +'/home/ras11/catkin_ws/src/ras_project/ras_project_speaker/sounds/completed.wav')
            Expdone = False

if __name__=='__main__':
     rospy.init_node('meme_sound')
    
     string_sub = rospy.Subscriber('/path/unreachable', Bool, stringCallback)
     expdone_sub = rospy.Subscriber('Explored/done',Bool,ExpCallback)
     rospy.spin()
                              
