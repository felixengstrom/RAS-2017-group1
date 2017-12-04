#!/usr/bin/env python
import rospy
import roslib
import sys, time
import cv2 as cv
import std_msgs.msg
from cv_bridge import CvBridge
from PIL.Image import fromarray
from PIL import Image
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ras_project_camera.msg import stringStamped
from keras.applications.vgg16 import preprocess_input
from keras.preprocessing import image
from keras.models import load_model
import numpy as np
import operator
import time


class classification(object): 
    def __init__(self):
        
        self.object_detected = False
        self.cv_image = None
        self.lastReading = 0
        self.sub_image = rospy.Subscriber('/camera/object_detected_image', Image, self.image_cb, queue_size = 1 ) #queue_size = 1, buff_size=self.buffer_size
        #self.sub_image = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_cb, queue_size = 1 )
        self.pub_object_class = rospy.Publisher('/camera/object_class', stringStamped, queue_size = 1)
        self.bridge = CvBridge()
        
        rospy.loginfo("subscribed to /camera/image/image_raw")
        
    def image_cb (self, img):
        #rospy.loginfo("in image callback")        
        self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        self.object_detected = True
        self.lastReading = img.header.stamp

def main(args):

    cl = classification()
    rospy.init_node('classification')

    model_dir = '/home/ras11/catkin_ws/src/ras_project/ras_project_camera/src/VGG16_62_v5.h5'
    model = load_model(model_dir)
    print('model loaded')
    size = (150, 150)
    r = rospy.Rate(7)

    while not rospy.is_shutdown():
		if(cl.object_detected == True):
			cl.object_detected = False
			t0 = time.time()
			# Input 
			# convert form cv:Mat to PIL
			pil_im = fromarray(cl.cv_image)
			pil_im = pil_im.resize(size) #, resample=0)
			x = image.img_to_array(pil_im)
			x = preprocess_input(x)
			#x = cv.normalize(x, alpha=0, beta=1, norm_type=cv.NORM_MINMAX)
			x = np.expand_dims(x, axis=0)
			preds = model.predict(x, steps = 1)
			classes = ['Blue Cube', 'Blue Triangle', 'Green Cube', 'Green Hollow Cube', 'Green Cyllinder', 'No Object', 'Orange Cross', 'Patric', 'Purple Cross', 'Purple Star', 'Red Hollow Cube', 'Red Cyllinder', 'Red Ball', 'Yellow Cube', 'Yellow Ball']
			#print (preds)
			pred = preds[0]
			#print (pred)

			t1 = time.time()
			print("prediction time: {:0.3f}s".format(t1-t0))
			index, value = max(enumerate(pred), key=operator.itemgetter(1))
			clas = classes[index]
			#rospy.loginfo("index %i", index)
			rospy.loginfo("value %d", value*100)
			result = (clas, value)
			#rospy.loginfo('Predicted object is %s', result[0])

			rospy.loginfo('object class %s', clas)
			h = std_msgs.msg.Header()
			h.stamp = cl.lastReading
			cl.pub_object_class.publish (header = h, data = clas)
			#cl.pub_object_class.publish (data = clas)
			#cv2.imshow("Image window", cv_image)
			#cv2.waitKey(3)'''
			r.sleep()

if __name__ == '__main__':
    main(sys.argv)
    
