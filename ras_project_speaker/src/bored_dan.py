#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import Bool
 
 
def stringCallback(msg):
     rospy.loginfo('We cant reach the goooooaaaal!')
     if(msg.data==False):
        os.system('play '+ '/home/ras11/catkin_ws/src/ras_project/ras_project_speaker/sounds/nooo.wav')
 
if __name__=='__main__':
     rospy.init_node('meme_sound')
    
     string_sub = rospy.Subscriber('/path/unreachable', Bool, stringCallback)
      
     rospy.spin()
                              
